// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file fmu_main.cpp
/// @brief IMU hardware simulator, exported as an FMI Co-Simulation FMU.
///
/// This FMU has no FMI output variables -- its effect is a UDP side
/// channel: each step turns the current body-frame truth inputs (specific
/// force + angular rate) into noisy raw register counts (ImuNoiseModel),
/// encodes them as Hemerion IMU raw-sample frames (ImuPacketEmitter), and
/// sends them to a fixed UDP peer (UdpSender), so the real, unmodified
/// ImuPacketParser + convert_raw_to_si() on the firmware side decodes them
/// exactly as it would bytes from a physical part.
///
/// Unlike the 10 Hz GPS FMU, a real IMU samples much faster than a
/// co-simulation communication step, so the `sample_rate_hz` parameter
/// (default 100 Hz) makes each step emit round(step * rate) frames. The
/// truth inputs are zero-order-held across the step (the master only
/// exchanges variables at communication points), so the sub-step samples
/// differ in noise draw and timestamp only -- the honest equivalent of
/// oversampling a plant that is itself only resolved at the communication
/// rate.
///
/// All the FMI plumbing -- entry points, variable marshalling, GUID handling
/// and modelDescription.xml generation -- belongs to the vendored fmu4cpp
/// export layer (vendor/fmu4cpp). This file only registers the variables and
/// implements do_step(); see cmake/generate_fmu.cmake for how the two halves
/// are compiled and packaged into an .fmu archive.
///
/// UdpSender is reused from the GPS FMU one module subtree over
/// (Hemerion/gps/fmu/) -- it is sensor-agnostic and lives in the same
/// module, so duplicating it here would only invite drift.

#include "Hemerion/gps/fmu/udpSender.hpp"
#include "Hemerion/imu/fmu/imu_noise_model.h"
#include "Hemerion/imu/fmu/imu_packet_emitter.h"

#include <fmu4cpp/fmu_base.hpp>
#include <fmu4cpp/fmu_except.hpp>

#include <cmath>
#include <cstdint>
#include <optional>

namespace hemerion::sensors::imu::fmu
{

namespace
{

using fmu4cpp::causality_t;
using fmu4cpp::variability_t;
using hemerion::sensors::gps::fmu::UdpSender;

constexpr char kUdpHostVariable[] = "HEMERION_IMU_FMU_UDP_HOST";
constexpr char kUdpPortVariable[] = "HEMERION_IMU_FMU_UDP_PORT";
constexpr char kDefaultUdpHost[] = "127.0.0.1";
constexpr std::uint16_t kDefaultUdpPort = 5763;
constexpr double kDefaultSampleRateHz = 100.0;

}  // namespace

/// @brief Co-simulation slave turning body-frame truth into a stream of raw
/// IMU register frames on a UDP socket.
class ImuSimulatorFmu final : public fmu4cpp::fmu_base
{
public:
  FMU4CPP_CTOR(ImuSimulatorFmu)
  {
    // True specific force in the body frame (what an ideal accelerometer triad would read): the sum of all
    // non-gravitational forces divided by vehicle mass. For a 6-DoF plant that reports thrust, aero force and mass
    // rather than specific force directly, the co-simulation host computes f = (F_thrust + F_aero) / m -- see
    // examples/rocket_gps_ecos/cosim_host_main.cpp.
    register_real("f_x_mps2", &truth_.specific_force_x_mps2)
        .setCausality(causality_t::INPUT)
        .setDescription("True specific force, body X (nose) [m/s^2]");
    register_real("f_y_mps2", &truth_.specific_force_y_mps2)
        .setCausality(causality_t::INPUT)
        .setDescription("True specific force, body Y [m/s^2]");
    register_real("f_z_mps2", &truth_.specific_force_z_mps2)
        .setCausality(causality_t::INPUT)
        .setDescription("True specific force, body Z [m/s^2]");

    // True body angular rates p/q/r, matching a 6-DoF plant's gyroscopic outputs.
    register_real("p_rad_s", &truth_.angular_rate_x_rad_s)
        .setCausality(causality_t::INPUT)
        .setDescription("True body roll rate p [rad/s]");
    register_real("q_rad_s", &truth_.angular_rate_y_rad_s)
        .setCausality(causality_t::INPUT)
        .setDescription("True body pitch rate q [rad/s]");
    register_real("r_rad_s", &truth_.angular_rate_z_rad_s)
        .setCausality(causality_t::INPUT)
        .setDescription("True body yaw rate r [rad/s]");

    register_real("sample_rate_hz", &sample_rate_hz_)
        .setCausality(causality_t::PARAMETER)
        .setVariability(variability_t::TUNABLE)
        .setDescription("Sensor output data rate [Hz]; each step emits round(step * rate) frames, at least one");
  }

  /// Opens the UDP socket. Deliberately not done in the constructor: the
  /// build-time modelDescription.xml generator instantiates the model purely
  /// to enumerate its variables, and that must not touch the network.
  void exit_initialisation_mode() override
  {
    sender_ = UdpSender::create_from_env(kUdpHostVariable, kUdpPortVariable, kDefaultUdpHost, kDefaultUdpPort);
    if (!sender_.has_value())
    {
      throw fmu4cpp::fatal_error("[hemerion_imu_fmu] Unable to open the raw-sample UDP socket");
    }
  }

  void terminate() override { sender_.reset(); }

  /// fmi2Reset equivalent. The turn-on biases ImuNoiseModel drew at
  /// construction are kept: they model this instance's physical part, which a
  /// reset does not swap out.
  void reset() override
  {
    truth_ = ImuTruthSample{};
    sample_rate_hz_ = kDefaultSampleRateHz;
    sender_.reset();
  }

protected:
  bool do_step(double dt) override
  {
    if (!sender_.has_value())
    {
      throw fmu4cpp::fatal_error("[hemerion_imu_fmu] Stepped before initialisation mode was exited");
    }

    // Truth is zero-order-held over the step; emit one frame per sensor
    // sample period that elapses within it, each with a fresh noise draw and
    // its own timestamp.
    const long samples = std::lround(dt * sample_rate_hz_);
    const long count = (samples > 0) ? samples : 1;
    const double sample_period_s = dt / static_cast<double>(count);

    for (long k = 1; k <= count; ++k)
    {
      const double sample_time_s = currentTime() + static_cast<double>(k) * sample_period_s;
      truth_.timestamp_us = static_cast<std::uint64_t>(sample_time_s * 1e6);
      const ImuPacketEmitter::Frame frame = ImuPacketEmitter::encode_raw_sample(noise_model_.apply(truth_));
      if (!sender_->send(frame.data(), frame.size()))
      {
        // A dropped datagram is a dropped sensor byte, not a simulation
        // error -- warn and keep stepping rather than returning false, which
        // fmu4cpp maps to "discard this step and terminate".
        debugLog(fmiWarning, "[hemerion_imu_fmu] Raw-sample frame could not be sent");
      }
    }
    return true;
  }

private:
  ImuNoiseModel noise_model_;
  std::optional<UdpSender> sender_;
  ImuTruthSample truth_;
  double sample_rate_hz_ = kDefaultSampleRateHz;
};

}  // namespace hemerion::sensors::imu::fmu

fmu4cpp::model_info fmu4cpp::get_model_info()
{
  model_info info;
  info.modelName = "HemerionImuSimulator";
  info.author = "Onur Tuncer, Istanbul Technical University";
  info.description = "Body-frame truth-state-to-raw-register-counts IMU hardware simulator for SWIL/HIL co-simulation";
  // Names such as f_x_mps2 carry no FMI structured-naming hierarchy.
  info.variableNamingConvention = "flat";
  return info;
}

FMU4CPP_INSTANTIATE(hemerion::sensors::imu::fmu::ImuSimulatorFmu);
