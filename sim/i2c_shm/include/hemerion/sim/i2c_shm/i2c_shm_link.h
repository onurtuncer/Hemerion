// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/sim/i2c_shm/i2c_shm_link.h
//
// The two endpoints of a simulated I2C bus (see i2c_shm_protocol.h for the
// shared region and the handshake):
//
//   I2cShmPeripheral -- the simulated part. Creates the segment and runs a
//                       service thread that walks posted transactions through
//                       an I2cPeripheral implementation, because real silicon
//                       answers its address whenever START goes out, not when
//                       its physics model happens to be stepping.
//   I2cShmController -- the MCU side. Attaches to a segment the peripheral
//                       created and issues whole transactions, the same
//                       granularity a HAL I2C master call has on the target.
//
// Synchronization is a spin-wait on the shared phase atomic with a short sleep
// backoff, matching sim/spi_shm: acceptable for a same-host link where a
// transaction completes in microseconds. Revisit with a named semaphore/event
// if profiling ever shows the spin costing real CPU.
// ------------------------------------------------------------------------------
#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "hemerion/sim/i2c_shm/i2c_shm_protocol.h"
#include "hemerion/sim/shm_bridge/shm_segment.h"

namespace hemerion::sim::i2c_shm
{

// What a simulated I2C peripheral has to implement: the bus events a part's
// interface state machine sees. Deliberately event-at-a-time even though the
// transport moves a whole transaction at once, so a device model is written
// the way the datasheet describes the part -- "the first byte after the
// address selects the register, subsequent reads auto-increment" -- and stays
// honest about what it could have known when it acked each byte.
class I2cPeripheral
{
public:
  I2cPeripheral() = default;
  I2cPeripheral(const I2cPeripheral&) = delete;
  I2cPeripheral& operator=(const I2cPeripheral&) = delete;
  I2cPeripheral(I2cPeripheral&&) = delete;
  I2cPeripheral& operator=(I2cPeripheral&&) = delete;
  virtual ~I2cPeripheral() = default;

  // START (or repeated START) plus the address byte. `read` is the R/W bit.
  // Returns the address ack: false is the part not answering this address,
  // which the controller-side HAL reports as a missing device.
  [[nodiscard]] virtual bool start(std::uint8_t address, bool read) = 0;

  // One byte of the write phase. Returns the data ack.
  [[nodiscard]] virtual bool write(std::uint8_t byte) = 0;

  // One byte of the read phase. `last` is true on the byte the controller
  // NACKs to end the read, which is how a part knows to stop advancing.
  [[nodiscard]] virtual std::uint8_t read(bool last) = 0;

  // STOP condition ending the transaction.
  virtual void stop() = 0;
};

// Peripheral endpoint. Non-copyable *and* non-movable: the service thread's
// loop refers to this object, so it is handed out through unique_ptr rather
// than the optional<> the rest of sim/ returns.
class I2cShmPeripheral
{
public:
  // Creates the bus segment. Fails if one of this name already exists, which
  // on POSIX also covers a leftover segment from a crashed run -- report it
  // rather than silently sharing a bus with a stale process.
  [[nodiscard]] static std::unique_ptr<I2cShmPeripheral> create(const std::string& bus_name);

  I2cShmPeripheral(const I2cShmPeripheral&) = delete;
  I2cShmPeripheral& operator=(const I2cShmPeripheral&) = delete;
  I2cShmPeripheral(I2cShmPeripheral&&) = delete;
  I2cShmPeripheral& operator=(I2cShmPeripheral&&) = delete;
  // Requests shutdown and joins the service thread.
  ~I2cShmPeripheral();

  // Starts servicing transactions with `device`, which must outlive this
  // object. From here on `device`'s bus events are delivered from the service
  // thread, so a device model shared with a stepping simulation has to do its
  // own locking.
  void start(I2cPeripheral& device);

  // Stops the service thread and marks the bus shut down.
  void stop();

  // Drives the interrupt line the controller samples.
  void set_interrupt(bool asserted);

  // True once a controller has attached to this bus.
  [[nodiscard]] bool controller_attached() const;

  // Transactions serviced so far this run.
  [[nodiscard]] std::uint64_t transactions_serviced() const;

private:
  explicit I2cShmPeripheral(shm_bridge::ShmSegment segment);

  void service_loop(I2cPeripheral& device);

  [[nodiscard]] I2cBusRegion& region() noexcept;
  [[nodiscard]] const I2cBusRegion& region() const noexcept;

  shm_bridge::ShmSegment segment_;
  std::thread service_thread_;
  std::atomic<bool> stop_requested_{ false };
};

// Controller (MCU-side) endpoint.
class I2cShmController
{
public:
  // How a transaction ended, as the controller-side HAL would report it.
  enum class Result : std::uint8_t
  {
    kOk,           ///< Every byte acknowledged; read bytes are valid.
    kAddressNack,  ///< No part answered the address (absent or unpowered).
    kDataNack,     ///< The part refused a write byte mid-transaction.
    kBusFault,     ///< Bus shut down, transaction over-long, or timeout.
  };

  // Attaches to a bus the peripheral has already created and marked ready.
  // Returns std::nullopt if the segment does not exist yet, its protocol
  // version does not match, or the peripheral has not finished powering up --
  // all three are retryable, which is what attach_within() does.
  [[nodiscard]] static std::optional<I2cShmController> attach(const std::string& bus_name);

  // Retries attach() until it succeeds or `timeout` elapses, so the two
  // processes can be started in either order.
  [[nodiscard]] static std::optional<I2cShmController> attach_within(const std::string& bus_name,
                                                                     std::chrono::milliseconds timeout);

  I2cShmController(const I2cShmController&) = delete;
  I2cShmController& operator=(const I2cShmController&) = delete;
  I2cShmController(I2cShmController&&) noexcept = default;
  I2cShmController& operator=(I2cShmController&&) noexcept = default;
  // Clears the attached flag, so a peripheral waiting for a reader stops
  // waiting once nobody is left to read it.
  ~I2cShmController();

  // One complete I2C transaction against `address`: `write_length` bytes of
  // write phase (a register address plus any data), then `read_length` bytes
  // of read phase under a repeated START. Either phase may be empty; both
  // empty is the address probe HAL_I2C_IsDeviceReady() sends. Returns
  // kBusFault if a phase exceeds kMaxPhaseBytes, the peripheral shut the bus
  // down, or it did not answer within `timeout`.
  [[nodiscard]] Result transaction(std::uint8_t address,
                                   const std::uint8_t* write_bytes,
                                   std::size_t write_length,
                                   std::uint8_t* read_bytes,
                                   std::size_t read_length,
                                   std::chrono::milliseconds timeout);

  // Samples the peripheral's interrupt line.
  [[nodiscard]] bool interrupt_line() const;

  // False once the peripheral has powered down (its FMU terminated).
  [[nodiscard]] bool peripheral_present() const;

  // Transactions this bus has serviced, from either side's point of view.
  [[nodiscard]] std::uint64_t transaction_index() const;

private:
  explicit I2cShmController(shm_bridge::ShmSegment segment);

  [[nodiscard]] I2cBusRegion& region() noexcept;
  [[nodiscard]] const I2cBusRegion& region() const noexcept;

  shm_bridge::ShmSegment segment_;
};

}  // namespace hemerion::sim::i2c_shm
