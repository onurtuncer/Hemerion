// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file fmu_main.cpp
/// @brief IMU hardware simulator, exported as an FMI Co-Simulation FMU.
///
/// This FMU has no FMI output variables -- its effect is a **SPI side
/// channel**. Each step turns the current body-frame truth inputs (specific
/// force + angular rate) into noisy raw register counts (ImuNoiseModel),
/// encodes them as Hemerion IMU raw-sample frames (ImuPacketEmitter), and
/// pushes them into the simulated part's sample FIFO (ImuSpiSlave). A
/// controller -- the host flight computer in examples/rocket_gps_ecos, or
/// emulated firmware under Renode -- then reads them out the way it would read
/// a real part: sample the data-ready line, read STATUS/FIFO_COUNT, burst the
/// FIFO port. The bytes it recovers are exactly the bytes ImuPacketEmitter
/// produced, so the unmodified on-target ImuPacketParser +
/// convert_raw_to_si() decode them as they would bytes from physical silicon.
///
/// Three things have to be true for that, and each belongs somewhere
/// different -- this file is only the third:
///
/// * **the datasheet** -- register map, sample FIFO, shift register:
///   ImuSpiSlave (imu_spi_slave.h), which knows nothing about how bytes reach
///   it and is unit-tested against the real on-target driver with no
///   transport in the picture;
/// * **the board** -- which bus the part sits on, when it comes up and goes
///   down, how the data-ready line is driven: SpiPeripheralEndpoint
///   (sim/spi_shm), sensor-agnostic and shared by any FMU that models an SPI
///   part;
/// * **the part number** -- which device model on which bus, plus the FMI
///   variables and the physics feeding it. That is all this file does.
///
/// The bus name defaults to `hemerion_imu_spi` and is overridable through
/// HEMERION_IMU_FMU_SPI_BUS, matching how the other sensor FMUs take their UDP
/// destination from the environment (no FMI String-typed variables,
/// retargetable without repackaging the archive).
///
/// Unlike the 10 Hz GPS FMU, a real IMU samples much faster than a
/// co-simulation communication step, so the `sample_rate_hz` parameter
/// (default 100 Hz) makes each step buffer round(step * rate) frames. The
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

#include "Hemerion/imu/fmu/imu_noise_model.h"
#include "Hemerion/imu/fmu/imu_packet_emitter.h"
#include "Hemerion/imu/fmu/imu_spi_slave.h"

#include <hemerion/sim/spi_shm/spi_peripheral_endpoint.h>

#include <fmu4cpp/fmu_base.hpp>
#include <fmu4cpp/fmu_except.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>

namespace hemerion::sensors::imu::fmu
{

namespace
{

using fmu4cpp::causality_t;
using fmu4cpp::variability_t;

constexpr double kDefaultSampleRateHz = 100.0;

/// How long terminate() lets a controller finish draining the sample FIFO.
/// Long enough for a poll cycle of any sane controller, short enough that a
/// scripted co-simulation never appears to hang on teardown.
constexpr auto kDrainTimeout = std::chrono::seconds(2);

/// Where this part sits: the bus it creates, and the environment variable a
/// launch script can retarget it with.
const sim::spi_shm::SpiPeripheralConfig kSpiBus{ "hemerion_imu_spi", "HEMERION_IMU_FMU_SPI_BUS" };

}  // namespace

/// @brief Co-simulation slave turning body-frame truth into raw IMU register
/// frames a controller reads over a simulated SPI bus.
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
        .setDescription("Sensor output data rate [Hz]; each step buffers round(step * rate) samples, at least one");
  }

  /// Brings the simulated part up on its bus. Deliberately not done in the
  /// constructor: the build-time modelDescription.xml generator instantiates
  /// the model purely to enumerate its variables, and that must not create
  /// shared-memory objects or spawn threads.
  void exit_initialisation_mode() override
  {
    if (!endpoint_.attach())
    {
      throw fmu4cpp::fatal_error("[hemerion_imu_fmu] Unable to create the SPI bus '" + endpoint_.bus_name() +
                                 "' (already in use by another run?)");
    }
    debugLog(fmiOK, "[hemerion_imu_fmu] SPI peripheral ready on bus '" + endpoint_.bus_name() + "'");
  }

  /// Powers the part down -- but not out from under a controller that is
  /// still reading it. A real part keeps answering chip select until the board
  /// loses power, so the sample stream should end where the simulation does,
  /// not wherever the controller's last poll happened to land. Bounded, and
  /// abandoned immediately if nobody is left on the bus.
  void terminate() override
  {
    const auto deadline = std::chrono::steady_clock::now() + kDrainTimeout;
    while (slave_.fifo_used() > 0 && endpoint_.controller_attached() &&
           std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    endpoint_.detach();
  }

  /// fmi2Reset equivalent. The turn-on biases ImuNoiseModel drew at
  /// construction are kept: they model this instance's physical part, which a
  /// reset does not swap out.
  void reset() override
  {
    truth_ = ImuTruthSample{};
    sample_rate_hz_ = kDefaultSampleRateHz;
    slave_.reset();
    reported_overflow_ = false;
    endpoint_.detach();
  }

protected:
  bool do_step(double dt) override
  {
    if (!endpoint_.attached())
    {
      throw fmu4cpp::fatal_error("[hemerion_imu_fmu] Stepped before initialisation mode was exited");
    }

    // Truth is zero-order-held over the step; latch one sample per sensor
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
      if (!slave_.push_frame(frame.data(), frame.size()) && !reported_overflow_)
      {
        // A controller too slow to drain the FIFO loses samples, exactly as it
        // would on the real part -- that is a property of the system under
        // test, not a simulation error, so warn once and keep stepping rather
        // than returning false (which fmu4cpp maps to "discard this step and
        // terminate"). The sticky STATUS bit tells the controller too.
        debugLog(fmiWarning, "[hemerion_imu_fmu] Sample FIFO full -- the controller is not draining it fast enough");
        reported_overflow_ = true;
      }
    }

    // Data-ready is a level, so it is re-driven every step rather than pulsed:
    // it stays asserted for as long as the FIFO holds a sample.
    endpoint_.publish_data_ready(slave_.data_ready());
    return true;
  }

private:
  ImuNoiseModel noise_model_;
  ImuSpiSlave slave_;
  sim::spi_shm::SpiPeripheralEndpoint<ImuSpiSlave> endpoint_{ slave_, kSpiBus };
  ImuTruthSample truth_;
  double sample_rate_hz_ = kDefaultSampleRateHz;
  bool reported_overflow_ = false;
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
