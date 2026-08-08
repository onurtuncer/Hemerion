// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/sim/spi_shm/spi_shm_link.h
//
// The two endpoints of a simulated SPI bus (see spi_shm_protocol.h for the
// shared region and the handshake):
//
//   SpiShmPeripheral -- the simulated part. Creates the segment and runs a
//                       service thread that shifts posted transfers through a
//                       SpiPeripheral implementation, because real silicon
//                       answers a transfer whenever chip select goes low, not
//                       when its physics model happens to be stepping.
//   SpiShmController -- the MCU side. Attaches to a segment the peripheral
//                       created and issues chip-select-framed transfers, the
//                       same granularity a HAL SPI call has on the target.
//
// Synchronization is a spin-wait on the shared phase atomic with a short sleep
// backoff, matching sim/shm_bridge: acceptable for a same-host link where a
// transfer completes in microseconds. Revisit with a named semaphore/event if
// profiling ever shows the spin costing real CPU.
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

#include "hemerion/sim/shm_bridge/shm_segment.h"
#include "hemerion/sim/spi_shm/spi_shm_protocol.h"

namespace hemerion::sim::spi_shm
{

// What a simulated SPI peripheral has to implement: the shift register and the
// chip-select edges around it. Deliberately byte-at-a-time even though the
// transport moves a whole transfer at once, so a device model is written the
// way the datasheet describes the part -- "the first byte is the command, the
// device returns the addressed register on the next" -- and stays honest about
// what it could have known when it drove each MISO byte.
class SpiPeripheral
{
public:
  SpiPeripheral() = default;
  SpiPeripheral(const SpiPeripheral&) = delete;
  SpiPeripheral& operator=(const SpiPeripheral&) = delete;
  SpiPeripheral(SpiPeripheral&&) = delete;
  SpiPeripheral& operator=(SpiPeripheral&&) = delete;
  virtual ~SpiPeripheral() = default;

  // Chip-select edge. `asserted` is true on the falling edge that starts a
  // transfer and false on the rising edge that ends it; parts reset their
  // command state machine on one or the other.
  virtual void chip_select(bool asserted) = 0;

  // Shifts one byte: `mosi` goes in, the returned byte goes out on MISO.
  [[nodiscard]] virtual std::uint8_t shift(std::uint8_t mosi) = 0;
};

// Peripheral endpoint. Non-copyable *and* non-movable: the service thread's
// loop refers to this object, so it is handed out through unique_ptr rather
// than the optional<> the rest of sim/ returns.
class SpiShmPeripheral
{
public:
  // Creates the bus segment. Fails if one of this name already exists, which
  // on POSIX also covers a leftover segment from a crashed run -- report it
  // rather than silently sharing a bus with a stale process.
  [[nodiscard]] static std::unique_ptr<SpiShmPeripheral> create(const std::string& bus_name);

  SpiShmPeripheral(const SpiShmPeripheral&) = delete;
  SpiShmPeripheral& operator=(const SpiShmPeripheral&) = delete;
  SpiShmPeripheral(SpiShmPeripheral&&) = delete;
  SpiShmPeripheral& operator=(SpiShmPeripheral&&) = delete;
  // Requests shutdown and joins the service thread.
  ~SpiShmPeripheral();

  // Starts servicing transfers with `device`, which must outlive this object.
  // From here on `device`'s chip_select()/shift() are called from the service
  // thread, so a device model shared with a stepping simulation has to do its
  // own locking.
  void start(SpiPeripheral& device);

  // Stops the service thread and marks the bus shut down.
  void stop();

  // Drives the data-ready line the controller samples.
  void set_data_ready(bool asserted);

  // True once a controller has attached to this bus.
  [[nodiscard]] bool controller_attached() const;

  // Transfers serviced so far this run.
  [[nodiscard]] std::uint64_t transfers_serviced() const;

private:
  explicit SpiShmPeripheral(shm_bridge::ShmSegment segment);

  void service_loop(SpiPeripheral& device);

  [[nodiscard]] SpiBusRegion& region() noexcept;
  [[nodiscard]] const SpiBusRegion& region() const noexcept;

  shm_bridge::ShmSegment segment_;
  std::thread service_thread_;
  std::atomic<bool> stop_requested_{ false };
};

// Controller (MCU-side) endpoint.
class SpiShmController
{
public:
  // Attaches to a bus the peripheral has already created and marked ready.
  // Returns std::nullopt if the segment does not exist yet, its protocol
  // version does not match, or the peripheral has not finished powering up --
  // all three are retryable, which is what attach_within() does.
  [[nodiscard]] static std::optional<SpiShmController> attach(const std::string& bus_name);

  // Retries attach() until it succeeds or `timeout` elapses, so the two
  // processes can be started in either order.
  [[nodiscard]] static std::optional<SpiShmController> attach_within(const std::string& bus_name,
                                                                     std::chrono::milliseconds timeout);

  SpiShmController(const SpiShmController&) = delete;
  SpiShmController& operator=(const SpiShmController&) = delete;
  SpiShmController(SpiShmController&&) noexcept = default;
  SpiShmController& operator=(SpiShmController&&) noexcept = default;
  // Clears the attached flag, so a peripheral waiting for its buffers to be
  // read out stops waiting once nobody is left to read them.
  ~SpiShmController();

  // One chip-select-framed full-duplex transfer: shifts `length` bytes out of
  // `tx` and the peripheral's answer into `rx` (either may be null for a
  // half-duplex leg). Returns false if `length` exceeds kMaxTransferBytes, the
  // peripheral shut the bus down, or it did not answer within `timeout`.
  [[nodiscard]] bool
  transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t length, std::chrono::milliseconds timeout);

  // Samples the peripheral's data-ready line.
  [[nodiscard]] bool data_ready() const;

  // False once the peripheral has powered down (its FMU terminated).
  [[nodiscard]] bool peripheral_present() const;

  // Transfers this bus has serviced, from either side's point of view.
  [[nodiscard]] std::uint64_t transfer_index() const;

private:
  explicit SpiShmController(shm_bridge::ShmSegment segment);

  [[nodiscard]] SpiBusRegion& region() noexcept;
  [[nodiscard]] const SpiBusRegion& region() const noexcept;

  shm_bridge::ShmSegment segment_;
};

}  // namespace hemerion::sim::spi_shm
