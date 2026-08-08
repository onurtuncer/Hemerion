// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/sim/spi_shm/spi_peripheral_endpoint.h
//
// Everything a hardware-simulator FMU needs in order to *be* an SPI peripheral,
// so that the FMU itself only has to declare what part it is.
//
// The division of labour this header exists to enforce:
//
//   device model  (e.g. Hemerion/imu/fmu/imu_spi_slave.h)
//       the datasheet: register map, FIFO, shift register. Knows nothing
//       about shared memory, environment variables or FMI -- which is also
//       why it can be unit-tested against the real on-target driver with no
//       transport in the picture at all.
//   this endpoint (sim/spi_shm)
//       the board: where the bus is, how it is named, when it comes up and
//       goes down, and the adapter that lets the device model answer it.
//   fmu_main.cpp  (the FMU)
//       the part number: which device model, on which bus, plus the FMI
//       variables and the physics that feed it.
//
// A device model is bound duck-typed rather than by inheritance (see
// SpiShiftable): implementing chip_select()/shift() is what makes something an
// SPI peripheral, and requiring it to inherit a sim/ base class would push the
// transport back down into the model this header is trying to keep clean.
// ------------------------------------------------------------------------------
#pragma once

#include <concepts>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "hemerion/sim/spi_shm/spi_shm_link.h"

namespace hemerion::sim::spi_shm
{

// What it takes to be a device model: the two operations an SPI peripheral
// performs, and nothing else.
template <class Device>
concept SpiShiftable = requires(Device& device, std::uint8_t byte) {
  { device.chip_select(true) };
  { device.shift(byte) } -> std::same_as<std::uint8_t>;
};

// Where a peripheral sits. The environment variable is how a launch script
// retargets a bus without repackaging the FMU archive -- the same convention
// the UDP-emitting sensor FMUs use for their destination, and the reason none
// of these FMUs needs an FMI String-typed variable.
struct SpiPeripheralConfig
{
  std::string default_bus_name;       // Bus to create when nothing overrides it.
  std::string bus_name_env_variable;  // Environment variable that may override it; empty disables the override.
};

// Resolves a bus name: the environment variable if it is set and non-empty,
// otherwise the fallback. Declared here (and defined in the .cpp) so the
// template below stays free of the platform's getenv() deprecation noise.
[[nodiscard]] std::string resolve_bus_name(const std::string& env_variable, const std::string& fallback);

// Owns a device model's presence on a shared-memory SPI bus: the segment, the
// service thread, the data-ready line, and the adapter between the two.
//
// Non-copyable and non-movable, matching SpiShmPeripheral: the service thread
// refers to the adapter this object holds.
template <SpiShiftable Device>
class SpiPeripheralEndpoint
{
public:
  // `device` must outlive this endpoint. Nothing is created here: an FMU is
  // instantiated at build time purely so the modelDescription.xml generator
  // can enumerate its variables, and that must not touch the OS.
  SpiPeripheralEndpoint(Device& device, SpiPeripheralConfig config)
    : adapter_(device)
    , config_(std::move(config))
    , bus_name_(resolve_bus_name(config_.bus_name_env_variable, config_.default_bus_name))
  {
  }

  SpiPeripheralEndpoint(const SpiPeripheralEndpoint&) = delete;
  SpiPeripheralEndpoint& operator=(const SpiPeripheralEndpoint&) = delete;
  SpiPeripheralEndpoint(SpiPeripheralEndpoint&&) = delete;
  SpiPeripheralEndpoint& operator=(SpiPeripheralEndpoint&&) = delete;
  ~SpiPeripheralEndpoint() = default;

  // Powers the part up: creates the bus and starts answering transfers.
  // Returns false if the segment could not be created -- typically a bus of
  // this name left behind by another run -- leaving the caller to report it in
  // whatever vocabulary it speaks.
  [[nodiscard]] bool attach()
  {
    // Re-resolved on every attach so a host that sets the variable between
    // FMU instantiation and initialisation is still obeyed.
    bus_name_ = resolve_bus_name(config_.bus_name_env_variable, config_.default_bus_name);
    bus_ = SpiShmPeripheral::create(bus_name_);
    if (bus_ == nullptr)
    {
      return false;
    }
    bus_->start(adapter_);
    return true;
  }

  // Powers the part down: the service thread stops and the bus goes to its
  // terminal phase, so a controller mid-transfer fails fast instead of hanging.
  void detach() { bus_.reset(); }

  // Drives the part's data-ready line. Ignored while detached, so a device
  // model does not have to know whether its bus is up.
  void publish_data_ready(bool asserted)
  {
    if (bus_ != nullptr)
    {
      bus_->set_data_ready(asserted);
    }
  }

  [[nodiscard]] bool attached() const { return bus_ != nullptr; }

  // True while a controller is mapped onto this bus. Lets a peripheral wait
  // for its buffers to be read out before powering down, and stop waiting
  // once there is nobody left to read them.
  [[nodiscard]] bool controller_attached() const { return bus_ != nullptr && bus_->controller_attached(); }

  // Bus this endpoint is on (or would be on), after the environment override.
  [[nodiscard]] const std::string& bus_name() const { return bus_name_; }

  // Transfers answered so far this run; zero while detached.
  [[nodiscard]] std::uint64_t transfers_serviced() const { return (bus_ != nullptr) ? bus_->transfers_serviced() : 0; }

private:
  // The one place the duck-typed device model meets the transport's interface.
  class Adapter final : public SpiPeripheral
  {
  public:
    explicit Adapter(Device& device) : device_(device) {}

    void chip_select(bool asserted) override { device_.chip_select(asserted); }

    std::uint8_t shift(std::uint8_t mosi) override { return device_.shift(mosi); }

  private:
    Device& device_;
  };

  Adapter adapter_;
  SpiPeripheralConfig config_;
  std::string bus_name_;
  std::unique_ptr<SpiShmPeripheral> bus_;
};

}  // namespace hemerion::sim::spi_shm
