// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file fmu_main.cpp
/// @brief Radar altimeter hardware simulator, exported as an FMI
/// Co-Simulation FMU.
///
/// This FMU has no FMI output variables -- its effect is a UDP side channel:
/// each step turns the current truth height above ground into noisy raw range
/// register counts (RadAltNoiseModel), encodes them as Hemerion radar
/// altimeter raw-sample frames (RadAltPacketEmitter), and sends them to a
/// fixed UDP peer (UdpSender), so the real, unmodified RadAltPacketParser +
/// convert_raw_to_si() on the firmware side decodes them exactly as it would
/// bytes from a physical part.
///
/// `sample_rate_hz` (default 25 Hz) makes each step emit round(step * rate)
/// frames; the truth input is zero-order-held across the step, so sub-step
/// frames differ in noise draw and timestamp only. The IMU FMU one module
/// subtree over documents that reasoning in full.
///
/// All the FMI plumbing -- entry points, variable marshalling, GUID handling
/// and modelDescription.xml generation -- belongs to the vendored fmu4cpp
/// export layer (vendor/fmu4cpp). This file only registers the variables and
/// implements do_step(); see cmake/generate_fmu.cmake for how the two halves
/// are compiled and packaged into an .fmu archive.
///
/// UdpSender is reused from the GPS FMU (Hemerion/gps/fmu/) -- it is
/// sensor-agnostic and lives in the same module, so duplicating it here would
/// only invite drift.

#include "Hemerion/gps/fmu/udpSender.hpp"
#include "Hemerion/radalt/fmu/radalt_noise_model.h"
#include "Hemerion/radalt/fmu/radalt_packet_emitter.h"

#include <fmu4cpp/fmu_base.hpp>
#include <fmu4cpp/fmu_except.hpp>

#include <cmath>
#include <cstdint>
#include <optional>

namespace hemerion::sensors::radalt::fmu
{

namespace
{

using fmu4cpp::causality_t;
using fmu4cpp::variability_t;
using hemerion::sensors::gps::fmu::UdpSender;

constexpr char kUdpHostVariable[] = "HEMERION_RADALT_FMU_UDP_HOST";
constexpr char kUdpPortVariable[] = "HEMERION_RADALT_FMU_UDP_PORT";
constexpr char kDefaultUdpHost[] = "127.0.0.1";
constexpr std::uint16_t kDefaultUdpPort = 5765;
constexpr double kDefaultSampleRateHz = 25.0;

}  // namespace

/// @brief Co-simulation slave turning truth height above ground into a stream
/// of raw radar altimeter range frames on a UDP socket.
class RadAltSimulatorFmu final : public fmu4cpp::fmu_base
{
public:
  FMU4CPP_CTOR(RadAltSimulatorFmu)
  {
    // Beyond the noise model's maximum tracking range (default 6000 m) or below zero, the emitted frames carry a
    // no-return status instead of a range word, exactly as a real part loses ground track.
    register_real("h_agl_m", &truth_.height_agl_m)
        .setCausality(causality_t::INPUT)
        .setDescription("True height above ground level along the radar beam [m]");

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
      throw fmu4cpp::fatal_error("[hemerion_radalt_fmu] Unable to open the raw-sample UDP socket");
    }
  }

  void terminate() override { sender_.reset(); }

  /// fmi2Reset equivalent. The turn-on bias RadAltNoiseModel drew at
  /// construction is kept: it models this instance's physical part, which a
  /// reset does not swap out.
  void reset() override
  {
    truth_ = RadAltTruthSample{};
    sample_rate_hz_ = kDefaultSampleRateHz;
    sender_.reset();
  }

protected:
  bool do_step(double dt) override
  {
    if (!sender_.has_value())
    {
      throw fmu4cpp::fatal_error("[hemerion_radalt_fmu] Stepped before initialisation mode was exited");
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
      const RadAltPacketEmitter::Frame frame = RadAltPacketEmitter::encode_raw_sample(noise_model_.apply(truth_));
      if (!sender_->send(frame.data(), frame.size()))
      {
        // A dropped datagram is a dropped sensor byte, not a simulation
        // error -- warn and keep stepping rather than returning false, which
        // fmu4cpp maps to "discard this step and terminate".
        debugLog(fmiWarning, "[hemerion_radalt_fmu] Raw-sample frame could not be sent");
      }
    }
    return true;
  }

private:
  RadAltNoiseModel noise_model_;
  std::optional<UdpSender> sender_;
  RadAltTruthSample truth_;
  double sample_rate_hz_ = kDefaultSampleRateHz;
};

}  // namespace hemerion::sensors::radalt::fmu

/// @cond FMI_ENTRY_POINTS
/// fmu4cpp's C entry points. Excluded from the API reference: they are
/// the FMI 2.0/3.0 ABI the packaging layer requires, not Hemerion API,
/// and fmu4cpp's own headers are not part of the Doxygen input.
fmu4cpp::model_info fmu4cpp::get_model_info()
{
  model_info info;
  info.modelName = "HemerionRadAltSimulator";
  info.author = "Onur Tuncer, Istanbul Technical University";
  info.description = "Truth-height-to-raw-range-counts radar altimeter hardware simulator for SWIL/HIL co-simulation";
  // Names such as h_agl_m carry no FMI structured-naming hierarchy.
  info.variableNamingConvention = "flat";
  return info;
}

FMU4CPP_INSTANTIATE(hemerion::sensors::radalt::fmu::RadAltSimulatorFmu);
/// @endcond
