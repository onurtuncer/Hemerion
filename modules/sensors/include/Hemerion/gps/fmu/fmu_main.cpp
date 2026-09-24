// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file fmu_main.cpp
/// @brief GPS UBX hardware simulator, exported as an FMI Co-Simulation FMU.
///
/// This FMU has no FMI output variables -- its effect is a UDP side channel:
/// each step turns the current truth-state inputs into a noisy GpsFix
/// (GpsNoiseModel), gates it through the receiver's dynamics envelope
/// (GpsDynamicsModel), encodes it as a real UBX-NAV-PVT frame (UbxEmitter),
/// and sends it to a fixed UDP peer (UdpSender) -- typically Renode's
/// emulated UART4 RX backend, so the real, unmodified GpsDriver/UbxParser on
/// the firmware side decodes it exactly as it would bytes from a physical
/// u-blox M9N.
///
/// The dynamics envelope is what makes the simulated receiver stop reporting
/// a fix under launch-vehicle dynamics, the way the real part does: see
/// gpsDynamicsModel.hpp. It is configured through three FMI parameters --
/// dynamic_platform (the u-blox dynModel code, default 8 = airborne 4 g),
/// cocom_limits_enabled and reacquisition_time_s.
///
/// The error model (gpsNoiseModel.hpp) is configured through a further set
/// of fixed parameters: the white and time-correlated 1-sigma per channel,
/// the two correlation times, the reported-accuracy scale, and `seed`. Their
/// start values are the previous, white-only model, so a master that sets
/// none of them gets the receiver it had; the examples' `--gps-errors
/// correlated` sets the realistic configuration through them. They are
/// fixed rather than tunable because a part's noise does not change in
/// flight, and are read once, when initialisation mode is exited.
///
/// All the FMI plumbing -- entry points, variable marshalling, GUID handling
/// and modelDescription.xml generation -- belongs to the vendored fmu4cpp
/// export layer (vendor/fmu4cpp). This file only registers the variables and
/// implements do_step(); see cmake/generate_fmu.cmake for how the two halves
/// are compiled and packaged into an .fmu archive.

#include "Hemerion/gps/fmu/gpsDynamicsModel.hpp"
#include "Hemerion/gps/fmu/gpsNoiseModel.hpp"
#include "Hemerion/gps/fmu/ubxEmitter.hpp"
#include "Hemerion/gps/fmu/udpSender.hpp"

#include <fmu4cpp/fmu_base.hpp>
#include <fmu4cpp/fmu_except.hpp>

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

namespace hemerion::sensors::gps::fmu
{

namespace
{

using fmu4cpp::causality_t;
using fmu4cpp::variability_t;

constexpr char kUdpHostVariable[] = "HEMERION_GPS_FMU_UDP_HOST";
constexpr char kUdpPortVariable[] = "HEMERION_GPS_FMU_UDP_PORT";
constexpr char kDefaultUdpHost[] = "127.0.0.1";
constexpr std::uint16_t kDefaultUdpPort = 5762;

/// Single source of truth for the dynamics parameters' FMI start values, so
/// the numbers in modelDescription.xml cannot drift from the ones
/// GpsDynamicsModel would have used on its own.
constexpr GpsDynamicsConfig kDefaultDynamics{};

/// Likewise for the error model: modelDescription.xml's start values and the
/// model's own defaults come from one place.
constexpr GpsNoiseConfig kDefaultNoise{};

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

/// Human-readable reason for a fix-validity transition, for the FMI log.
[[nodiscard]] const char* verdict_name(GpsDynamicsVerdict verdict)
{
  switch (verdict)
  {
    case GpsDynamicsVerdict::kValid:
      return "valid";
    case GpsDynamicsVerdict::kCocomLimit:
      return "COCOM altitude+speed limit";
    case GpsDynamicsVerdict::kAltitudeLimit:
      return "platform altitude limit";
    case GpsDynamicsVerdict::kHorizontalSpeedLimit:
      return "platform horizontal speed limit";
    case GpsDynamicsVerdict::kVerticalSpeedLimit:
      return "platform vertical speed limit";
    case GpsDynamicsVerdict::kAccelerationLimit:
      return "platform acceleration limit";
    case GpsDynamicsVerdict::kReacquiring:
      return "re-acquiring";
  }
  return "unknown";
}

}  // namespace

/// @brief Co-simulation slave turning trajectory truth into a UBX-NAV-PVT
/// stream on a UDP socket, with the receiver's dynamics envelope applied.
class GpsSimulatorFmu final : public fmu4cpp::fmu_base
{
public:
  FMU4CPP_CTOR(GpsSimulatorFmu)
  {
    register_real("latitude_deg", &truth_.latitude_deg)
        .setCausality(causality_t::INPUT)
        .setDescription("True geodetic latitude [degrees], north positive");
    register_real("longitude_deg", &truth_.longitude_deg)
        .setCausality(causality_t::INPUT)
        .setDescription("True geodetic longitude [degrees], east positive");
    register_real("altitude_m", &altitude_m_)
        .setCausality(causality_t::INPUT)
        .setDescription("True altitude above mean sea level [m]");

    register_real("ground_speed_mps", &ground_speed_mps_)
        .setCausality(causality_t::INPUT)
        .setDescription("True speed over ground [m/s]; ignored once any NED velocity component is written");
    register_real("course_deg", &course_deg_)
        .setCausality(causality_t::INPUT)
        .setDescription("True course over ground [degrees]; ignored once any NED velocity component is written");

    // NED truth velocity, the natural output set of a 6-DoF plant (e.g. Aetherion's TwoStageRocket FMU). Once any of
    // these is written, the ground_speed_mps/course_deg inputs above are ignored and the emitted speed-over-ground
    // and course are derived from v_north/v_east instead. v_down feeds the dynamics envelope's vertical-rate check,
    // but NAV-PVT's velD field is still encoded as 0 -- GpsFix carries no vertical velocity yet.
    const auto onNedWritten = [this] { ned_velocity_set_ = true; };
    register_real("v_north_mps", &v_north_mps_, onNedWritten)
        .setCausality(causality_t::INPUT)
        .setDescription("True NED velocity, north component [m/s]");
    register_real("v_east_mps", &v_east_mps_, onNedWritten)
        .setCausality(causality_t::INPUT)
        .setDescription("True NED velocity, east component [m/s]");
    register_real("v_down_mps", &v_down_mps_, onNedWritten)
        .setCausality(causality_t::INPUT)
        .setDescription("True NED velocity, down component [m/s], positive downwards");

    // Dynamics envelope. Each of the three is pushed straight into
    // GpsDynamicsModel, which clears its derived state whenever the
    // configuration changes.
    const auto onDynamicsChanged = [this] { apply_dynamics_config(); };
    register_integer("dynamic_platform", &dynamic_platform_code_, onDynamicsChanged)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::TUNABLE)
        .setDescription("u-blox dynModel code for the navigation-engine platform model (8 = airborne 4 g, "
                        "-1 = no envelope checks)");
    register_boolean("cocom_limits_enabled", &cocom_limits_enabled_, onDynamicsChanged)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::TUNABLE)
        .setDescription("Apply the COCOM export-control cut-off (navigation output stops above 18000 m AND 515 m/s)");
    register_real("reacquisition_time_s", &reacquisition_time_s_, onDynamicsChanged)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::TUNABLE)
        .setDescription("Hold-off before a fix returns after any limit trips [s]");

    // Error model. Fixed, not tunable: read once in exit_initialisation_mode,
    // where the noise model is built from them -- see the file comment.
    const auto noise_parameter = [this](const char* name, double* storage, const char* description) {
      register_real(name, storage)
          .setCausality(causality_t::PARAMETER)
          .setVariability(variability_t::FIXED)
          .setDescription(description);
    };
    register_integer("seed", &seed_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::FIXED)
        .setDescription("Error-model RNG seed; 0 draws a nondeterministic one, any other value makes the run "
                        "reproducible");
    noise_parameter("horizontal_pos_noise_m",
                    &horizontal_pos_noise_m_,
                    "White position error 1-sigma, north and east independently [m]");
    noise_parameter("vertical_pos_noise_m", &vertical_pos_noise_m_, "White altitude error 1-sigma [m]");
    noise_parameter("speed_noise_mps", &speed_noise_mps_, "White speed-over-ground error 1-sigma [m/s]");
    noise_parameter("course_noise_deg", &course_noise_deg_, "White course error 1-sigma [degrees]");
    noise_parameter("horizontal_pos_correlated_m",
                    &horizontal_pos_correlated_m_,
                    "Time-correlated (Gauss-Markov) position error stationary 1-sigma, north and east [m]; "
                    "0 = off");
    noise_parameter("vertical_pos_correlated_m",
                    &vertical_pos_correlated_m_,
                    "Time-correlated altitude error stationary 1-sigma [m]; 0 = off");
    noise_parameter(
        "position_correlation_time_s", &position_correlation_time_s_, "Correlation time of the position error [s]");
    noise_parameter("speed_correlated_mps",
                    &speed_correlated_mps_,
                    "Time-correlated speed error stationary 1-sigma [m/s]; 0 = off");
    noise_parameter("course_correlated_deg",
                    &course_correlated_deg_,
                    "Time-correlated course error stationary 1-sigma [degrees]; 0 = off");
    noise_parameter("velocity_correlation_time_s",
                    &velocity_correlation_time_s_,
                    "Correlation time of the speed and course errors [s]");
    noise_parameter("accuracy_scale",
                    &accuracy_scale_,
                    "Reported hAcc/vAcc as a multiple of the true total 1-sigma; 1 = honest, <1 = optimistic");
  }

  /// Opens the UDP socket. Deliberately not done in the constructor: the
  /// build-time modelDescription.xml generator instantiates the model purely
  /// to enumerate its variables, and that must not touch the network.
  void exit_initialisation_mode() override
  {
    apply_noise_config();
    sender_ = UdpSender::create_from_env(kUdpHostVariable, kUdpPortVariable, kDefaultUdpHost, kDefaultUdpPort);
    if (!sender_.has_value())
    {
      throw fmu4cpp::fatal_error("[hemerion_gps_fmu] Unable to open the UBX UDP socket");
    }
  }

  void terminate() override { sender_.reset(); }

  /// fmi2Reset equivalent: back to a cold receiver with the default envelope
  /// and the default error model. The error the receiver was carrying is
  /// forgotten -- it belonged to the environment the previous run flew
  /// through -- and the parameters return to their start values; the model
  /// itself is rebuilt from them when initialisation mode is next exited, so
  /// with a fixed seed a reset reproduces the run and with seed 0 it draws a
  /// fresh part.
  void reset() override
  {
    noise_model_.reset_state();
    seed_ = 0;
    horizontal_pos_noise_m_ = kDefaultNoise.horizontal_pos_noise_m;
    vertical_pos_noise_m_ = kDefaultNoise.vertical_pos_noise_m;
    speed_noise_mps_ = kDefaultNoise.speed_noise_mps;
    course_noise_deg_ = kDefaultNoise.course_noise_deg;
    horizontal_pos_correlated_m_ = kDefaultNoise.horizontal_pos_correlated_m;
    vertical_pos_correlated_m_ = kDefaultNoise.vertical_pos_correlated_m;
    position_correlation_time_s_ = kDefaultNoise.position_correlation_time_s;
    speed_correlated_mps_ = kDefaultNoise.speed_correlated_mps;
    course_correlated_deg_ = kDefaultNoise.course_correlated_deg;
    velocity_correlation_time_s_ = kDefaultNoise.velocity_correlation_time_s;
    accuracy_scale_ = kDefaultNoise.accuracy_scale;

    truth_ = GpsTruthSample{};
    altitude_m_ = 0.0;
    ground_speed_mps_ = 0.0;
    course_deg_ = 0.0;
    v_north_mps_ = 0.0;
    v_east_mps_ = 0.0;
    v_down_mps_ = 0.0;
    ned_velocity_set_ = false;

    dynamic_platform_code_ = static_cast<int>(kDefaultDynamics.platform);
    cocom_limits_enabled_ = kDefaultDynamics.cocom_limits_enabled;
    reacquisition_time_s_ = kDefaultDynamics.reacquisition_time_s;
    dynamics_.set_config(kDefaultDynamics);
    last_verdict_ = GpsDynamicsVerdict::kValid;

    sender_.reset();
  }

protected:
  bool do_step(double dt) override
  {
    if (!sender_.has_value())
    {
      throw fmu4cpp::fatal_error("[hemerion_gps_fmu] Stepped before initialisation mode was exited");
    }

    // A real receiver reports the solution for the epoch it has just closed,
    // so the fix is stamped at the end of the communication step.
    truth_.timestamp_us = static_cast<std::uint64_t>((currentTime() + dt) * 1e6);
    truth_.altitude_m = static_cast<float>(altitude_m_);
    truth_.v_down_mps = static_cast<float>(v_down_mps_);

    if (ned_velocity_set_)
    {
      truth_.ground_speed_mps = static_cast<float>(std::hypot(v_north_mps_, v_east_mps_));
      const double course_deg = std::atan2(v_east_mps_, v_north_mps_) * kRadToDeg;
      truth_.course_deg = static_cast<float>(std::fmod(course_deg + 360.0, 360.0));
    }
    else
    {
      truth_.ground_speed_mps = static_cast<float>(ground_speed_mps_);
      truth_.course_deg = static_cast<float>(course_deg_);
    }

    GpsFix fix = noise_model_.apply(truth_);

    // Gated against truth, not against the noisy fix: it is the vehicle's
    // real motion that breaks carrier tracking, not the receiver's estimate
    // of it.
    const GpsDynamicsVerdict verdict = dynamics_.apply(truth_, fix);
    if (verdict != last_verdict_)
    {
      debugLog(fmiOK, std::string("[hemerion_gps_fmu] Fix validity now: ") + verdict_name(verdict));
      last_verdict_ = verdict;
    }

    const UbxEmitter::Frame frame = UbxEmitter::encode_nav_pvt(fix);
    if (!sender_->send(frame.data(), frame.size()))
    {
      // A dropped datagram is a dropped sensor byte, not a simulation error
      // -- warn and keep stepping rather than returning false, which fmu4cpp
      // maps to "discard this step and terminate".
      debugLog(fmiWarning, "[hemerion_gps_fmu] UBX-NAV-PVT frame could not be sent");
    }
    return true;
  }

private:
  /// Rebuilds the dynamics configuration from the three FMI parameters. An
  /// unknown dynModel code (u-blox reserves code 1) leaves the model alone
  /// rather than silently selecting some other envelope.
  void apply_dynamics_config()
  {
    GpsDynamicsConfig config;
    if (!platform_from_code(dynamic_platform_code_, config.platform))
    {
      debugLog(fmiWarning,
               "[hemerion_gps_fmu] Unknown dynModel code " + std::to_string(dynamic_platform_code_) +
                   "; keeping the current platform model");
      dynamic_platform_code_ = static_cast<int>(dynamics_.config().platform);
      return;
    }
    config.cocom_limits_enabled = cocom_limits_enabled_;
    config.reacquisition_time_s = static_cast<float>(reacquisition_time_s_);
    dynamics_.set_config(config);
  }

  /// Builds the noise model from the fixed parameters. Called once per
  /// initialisation, so a seeded run is reproducible from the start of the
  /// first step and a reset-and-reinitialise repeats it.
  void apply_noise_config()
  {
    GpsNoiseConfig config;
    config.horizontal_pos_noise_m = static_cast<float>(horizontal_pos_noise_m_);
    config.vertical_pos_noise_m = static_cast<float>(vertical_pos_noise_m_);
    config.speed_noise_mps = static_cast<float>(speed_noise_mps_);
    config.course_noise_deg = static_cast<float>(course_noise_deg_);
    config.horizontal_pos_correlated_m = static_cast<float>(horizontal_pos_correlated_m_);
    config.vertical_pos_correlated_m = static_cast<float>(vertical_pos_correlated_m_);
    config.position_correlation_time_s = static_cast<float>(position_correlation_time_s_);
    config.speed_correlated_mps = static_cast<float>(speed_correlated_mps_);
    config.course_correlated_deg = static_cast<float>(course_correlated_deg_);
    config.velocity_correlation_time_s = static_cast<float>(velocity_correlation_time_s_);
    config.accuracy_scale = static_cast<float>(accuracy_scale_);
    noise_model_ = (seed_ == 0) ? GpsNoiseModel(config) : GpsNoiseModel(config, static_cast<std::uint64_t>(seed_));
  }

  GpsNoiseModel noise_model_;
  GpsDynamicsModel dynamics_{ kDefaultDynamics };
  std::optional<UdpSender> sender_;
  GpsTruthSample truth_;
  GpsDynamicsVerdict last_verdict_ = GpsDynamicsVerdict::kValid;

  // GpsTruthSample stores these as float (they are wire-scale quantities);
  // FMI reals are double, so the registered variables live here and are
  // narrowed once per step.
  double altitude_m_ = 0.0;
  double ground_speed_mps_ = 0.0;
  double course_deg_ = 0.0;

  double v_north_mps_ = 0.0;
  double v_east_mps_ = 0.0;
  double v_down_mps_ = 0.0;
  bool ned_velocity_set_ = false;

  int dynamic_platform_code_ = static_cast<int>(kDefaultDynamics.platform);
  bool cocom_limits_enabled_ = kDefaultDynamics.cocom_limits_enabled;
  double reacquisition_time_s_ = kDefaultDynamics.reacquisition_time_s;

  // Error-model parameters, FMI-typed (double/int) and narrowed once in
  // apply_noise_config().
  int seed_ = 0;
  double horizontal_pos_noise_m_ = kDefaultNoise.horizontal_pos_noise_m;
  double vertical_pos_noise_m_ = kDefaultNoise.vertical_pos_noise_m;
  double speed_noise_mps_ = kDefaultNoise.speed_noise_mps;
  double course_noise_deg_ = kDefaultNoise.course_noise_deg;
  double horizontal_pos_correlated_m_ = kDefaultNoise.horizontal_pos_correlated_m;
  double vertical_pos_correlated_m_ = kDefaultNoise.vertical_pos_correlated_m;
  double position_correlation_time_s_ = kDefaultNoise.position_correlation_time_s;
  double speed_correlated_mps_ = kDefaultNoise.speed_correlated_mps;
  double course_correlated_deg_ = kDefaultNoise.course_correlated_deg;
  double velocity_correlation_time_s_ = kDefaultNoise.velocity_correlation_time_s;
  double accuracy_scale_ = kDefaultNoise.accuracy_scale;
};

}  // namespace hemerion::sensors::gps::fmu

/// @cond FMI_ENTRY_POINTS
/// fmu4cpp's C entry points. Excluded from the API reference: they are
/// the FMI 2.0/3.0 ABI the packaging layer requires, not Hemerion API,
/// and fmu4cpp's own headers are not part of the Doxygen input.
fmu4cpp::model_info fmu4cpp::get_model_info()
{
  model_info info;
  info.modelName = "HemerionGpsUbxSimulator";
  info.author = "Onur Tuncer, Istanbul Technical University";
  info.description = "Truth-state-to-UBX-NAV-PVT GPS hardware simulator for SWIL/HIL co-simulation";
  // Names such as latitude_deg carry no FMI structured-naming hierarchy.
  info.variableNamingConvention = "flat";
  return info;
}

FMU4CPP_INSTANTIATE(hemerion::sensors::gps::fmu::GpsSimulatorFmu);
/// @endcond
