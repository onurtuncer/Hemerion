// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/sim/i2c_shm/i2c_peripheral_endpoint.h
//
// Everything a hardware-simulator FMU needs in order to *be* an I2C
// peripheral, so that the FMU itself only has to declare what part it is --
// the I2C twin of sim/spi_shm's SpiPeripheralEndpoint, with the same division
// of labour:
//
//   device model  (e.g. Hemerion/baro/bmp390/fmu/bmp390_i2c_slave.h)
//       the datasheet: register map, data registers, bus state machine.
//       Knows nothing about shared memory, environment variables or FMI --
//       which is also why it can be unit-tested against the real on-target
//       driver with no transport in the picture at all.
//   this endpoint (sim/i2c_shm)
//       the board: where the bus is, how it is named, when it comes up and
//       goes down, and the adapter that lets the device model answer it.
//   fmu_main.cpp  (the FMU)
//       the part number: which device model, on which bus, plus the FMI
//       variables and the physics that feed it.
//
// A device model is bound duck-typed rather than by inheritance (see
// I2cAddressable): implementing start()/write()/read()/stop() is what makes
// something an I2C peripheral, and requiring it to inherit a sim/ base class
// would push the transport back down into the model this header is trying to
// keep clean.
// ------------------------------------------------------------------------------
#pragma once

#include <concepts>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "hemerion/sim/i2c_shm/i2c_shm_link.h"

namespace hemerion::sim::i2c_shm
{

// What it takes to be a device model: the four bus events an I2C peripheral
// answers, and nothing else.
template <class Device>
concept I2cAddressable = requires(Device& device, std::uint8_t byte) {
  { device.start(byte, true) } -> std::same_as<bool>;
  { device.write(byte) } -> std::same_as<bool>;
  { device.read(true) } -> std::same_as<std::uint8_t>;
  { device.stop() };
};

// Where a peripheral sits. The environment variable is how a launch script
// retargets a bus without repackaging the FMU archive -- the same convention
// the SPI endpoint and the UDP-emitting sensor FMUs use, and the reason none
// of these FMUs needs an FMI String-typed variable.
struct I2cPeripheralConfig
{
  std::string default_bus_name;       // Bus to create when nothing overrides it.
  std::string bus_name_env_variable;  // Environment variable that may override it; empty disables the override.
};

// Resolves a bus name: the environment variable if it is set and non-empty,
// otherwise the fallback. Declared here (and defined in the .cpp) so the
// template below stays free of the platform's getenv() deprecation noise.
[[nodiscard]] std::string resolve_bus_name(const std::string& env_variable, const std::string& fallback);

// Owns a device model's presence on a shared-memory I2C bus: the segment, the
// service thread, the interrupt line, and the adapter between the two.
//
// Non-copyable and non-movable, matching I2cShmPeripheral: the service thread
// refers to the adapter this object holds.
template <I2cAddressable Device>
class I2cPeripheralEndpoint
{
public:
  // `device` must outlive this endpoint. Nothing is created here: an FMU is
  // instantiated at build time purely so the modelDescription.xml generator
  // can enumerate its variables, and that must not touch the OS.
  I2cPeripheralEndpoint(Device& device, I2cPeripheralConfig config)
    : adapter_(device)
    , config_(std::move(config))
    , bus_name_(resolve_bus_name(config_.bus_name_env_variable, config_.default_bus_name))
  {
  }

  I2cPeripheralEndpoint(const I2cPeripheralEndpoint&) = delete;
  I2cPeripheralEndpoint& operator=(const I2cPeripheralEndpoint&) = delete;
  I2cPeripheralEndpoint(I2cPeripheralEndpoint&&) = delete;
  I2cPeripheralEndpoint& operator=(I2cPeripheralEndpoint&&) = delete;
  ~I2cPeripheralEndpoint() = default;

  // Powers the part up: creates the bus and starts answering transactions.
  // Returns false if the segment could not be created -- typically a bus of
  // this name left behind by another run -- leaving the caller to report it in
  // whatever vocabulary it speaks.
  [[nodiscard]] bool attach()
  {
    // Re-resolved on every attach so a host that sets the variable between
    // FMU instantiation and initialisation is still obeyed.
    bus_name_ = resolve_bus_name(config_.bus_name_env_variable, config_.default_bus_name);
    bus_ = I2cShmPeripheral::create(bus_name_);
    if (bus_ == nullptr)
    {
      return false;
    }
    bus_->start(adapter_);
    return true;
  }

  // Powers the part down: the service thread stops and the bus goes to its
  // terminal phase, so a controller mid-transaction fails fast instead of
  // hanging.
  void detach() { bus_.reset(); }

  // Drives the part's interrupt line. Ignored while detached, so a device
  // model does not have to know whether its bus is up.
  void publish_interrupt(bool asserted)
  {
    if (bus_ != nullptr)
    {
      bus_->set_interrupt(asserted);
    }
  }

  [[nodiscard]] bool attached() const { return bus_ != nullptr; }

  // True while a controller is mapped onto this bus. Lets a peripheral wait
  // for its data to be read out before powering down, and stop waiting once
  // there is nobody left to read it.
  [[nodiscard]] bool controller_attached() const { return bus_ != nullptr && bus_->controller_attached(); }

  // Bus this endpoint is on (or would be on), after the environment override.
  [[nodiscard]] const std::string& bus_name() const { return bus_name_; }

  // Transactions answered so far this run; zero while detached.
  [[nodiscard]] std::uint64_t transactions_serviced() const
  {
    return (bus_ != nullptr) ? bus_->transactions_serviced() : 0;
  }

private:
  // The one place the duck-typed device model meets the transport's interface.
  class Adapter final : public I2cPeripheral
  {
  public:
    explicit Adapter(Device& device) : device_(device) {}

    [[nodiscard]] bool start(std::uint8_t address, bool read) override { return device_.start(address, read); }

    [[nodiscard]] bool write(std::uint8_t byte) override { return device_.write(byte); }

    [[nodiscard]] std::uint8_t read(bool last) override { return device_.read(last); }

    void stop() override { device_.stop(); }

  private:
    Device& device_;
  };

  Adapter adapter_;
  I2cPeripheralConfig config_;
  std::string bus_name_;
  std::unique_ptr<I2cShmPeripheral> bus_;
};

}  // namespace hemerion::sim::i2c_shm
