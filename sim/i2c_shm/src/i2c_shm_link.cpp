// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
#include "hemerion/sim/i2c_shm/i2c_shm_link.h"

#include <algorithm>
#include <cstring>
#include <new>
#include <utility>

namespace hemerion::sim::i2c_shm
{

namespace
{

using shm_bridge::ShmSegment;

// Spin hot for a short while before sleeping: a transaction is answered in
// microseconds when the peer is running, and the sleep only exists so an idle
// bus does not burn a core. Matches sim/spi_shm.
constexpr int kSpinsBeforeSleep = 256;
constexpr auto kBackoff = std::chrono::microseconds(50);

void backoff(int& spins)
{
  if (spins < kSpinsBeforeSleep)
  {
    ++spins;
    std::this_thread::yield();
    return;
  }
  std::this_thread::sleep_for(kBackoff);
}

[[nodiscard]] std::uint32_t phase_value(BusPhase phase) { return static_cast<std::uint32_t>(phase); }

[[nodiscard]] std::uint32_t result_value(TransactionResult result) { return static_cast<std::uint32_t>(result); }

// Walks one posted transaction through the device's bus-event state machine:
// START + address(W), the write bytes, then under a repeated START + address(R)
// the read bytes, then STOP. Split out of the service loop so the early-exit
// NACK paths still fall through to the single STOP below.
[[nodiscard]] TransactionResult run_transaction(I2cPeripheral& device, I2cBusRegion& bus)
{
  const auto address = static_cast<std::uint8_t>(bus.target_address & 0x7FU);
  const std::size_t write_length = std::min<std::size_t>(bus.write_length, kMaxPhaseBytes);
  const std::size_t read_length = std::min<std::size_t>(bus.read_length, kMaxPhaseBytes);

  TransactionResult result = TransactionResult::kAcknowledged;

  // Write phase -- also the address probe when both phases are empty: the
  // probe is a START in write mode followed directly by STOP.
  if (write_length > 0 || read_length == 0)
  {
    if (!device.start(address, /*read=*/false))
    {
      device.stop();
      return TransactionResult::kAddressNack;
    }
    for (std::size_t i = 0; i < write_length; ++i)
    {
      if (!device.write(bus.write_bytes[i]))
      {
        result = TransactionResult::kDataNack;
        break;
      }
    }
  }

  // Read phase, under a repeated START (or the sole START of a pure read).
  if (result == TransactionResult::kAcknowledged && read_length > 0)
  {
    if (!device.start(address, /*read=*/true))
    {
      device.stop();
      return TransactionResult::kAddressNack;
    }
    for (std::size_t i = 0; i < read_length; ++i)
    {
      bus.read_bytes[i] = device.read(/*last=*/i + 1 == read_length);
    }
  }

  device.stop();
  return result;
}

}  // namespace

// ------------------------------------------------------------------------------
// I2cShmPeripheral
// ------------------------------------------------------------------------------

std::unique_ptr<I2cShmPeripheral> I2cShmPeripheral::create(const std::string& bus_name)
{
  std::optional<ShmSegment> segment = ShmSegment::create(bus_name, sizeof(I2cBusRegion));
  if (!segment.has_value())
  {
    return nullptr;
  }
  // The peripheral owns the region's lifetime, so it is the side that
  // constructs it -- the controller only ever reinterprets these bytes.
  new (segment->data()) I2cBusRegion{};
  return std::unique_ptr<I2cShmPeripheral>(new I2cShmPeripheral(std::move(*segment)));
}

I2cShmPeripheral::I2cShmPeripheral(ShmSegment segment) : segment_(std::move(segment)) {}

I2cShmPeripheral::~I2cShmPeripheral() { stop(); }

void I2cShmPeripheral::start(I2cPeripheral& device)
{
  if (service_thread_.joinable())
  {
    return;
  }
  stop_requested_.store(false, std::memory_order_relaxed);
  service_thread_ = std::thread([this, &device] { service_loop(device); });
  region().peripheral_ready.store(1, std::memory_order_release);
}

void I2cShmPeripheral::stop()
{
  stop_requested_.store(true, std::memory_order_relaxed);
  if (service_thread_.joinable())
  {
    service_thread_.join();
  }
  region().peripheral_ready.store(0, std::memory_order_relaxed);
  region().interrupt_line.store(0, std::memory_order_relaxed);
  // Terminal: a controller waiting on (or about to post) a transaction fails
  // fast instead of waiting out its timeout against a part that is gone.
  region().phase.store(phase_value(BusPhase::kShutdownRequested), std::memory_order_release);
}

void I2cShmPeripheral::set_interrupt(bool asserted)
{
  region().interrupt_line.store(asserted ? 1U : 0U, std::memory_order_release);
}

bool I2cShmPeripheral::controller_attached() const
{
  return region().controller_attached.load(std::memory_order_acquire) != 0U;
}

std::uint64_t I2cShmPeripheral::transactions_serviced() const
{
  return region().transaction_index.load(std::memory_order_acquire);
}

void I2cShmPeripheral::service_loop(I2cPeripheral& device)
{
  I2cBusRegion& bus = region();
  int spins = 0;
  while (!stop_requested_.load(std::memory_order_relaxed))
  {
    if (bus.phase.load(std::memory_order_acquire) != phase_value(BusPhase::kTransactionPosted))
    {
      backoff(spins);
      continue;
    }
    spins = 0;

    bus.result = result_value(run_transaction(device, bus));

    bus.transaction_index.fetch_add(1, std::memory_order_relaxed);
    bus.phase.store(phase_value(BusPhase::kTransactionComplete), std::memory_order_release);
  }
}

I2cBusRegion& I2cShmPeripheral::region() noexcept { return *static_cast<I2cBusRegion*>(segment_.data()); }

const I2cBusRegion& I2cShmPeripheral::region() const noexcept
{
  return *static_cast<const I2cBusRegion*>(segment_.data());
}

// ------------------------------------------------------------------------------
// I2cShmController
// ------------------------------------------------------------------------------

std::optional<I2cShmController> I2cShmController::attach(const std::string& bus_name)
{
  std::optional<ShmSegment> segment = ShmSegment::open_existing(bus_name, sizeof(I2cBusRegion));
  if (!segment.has_value())
  {
    return std::nullopt;
  }

  auto* bus = static_cast<I2cBusRegion*>(segment->data());
  if (bus->protocol_version != kProtocolVersion)
  {
    return std::nullopt;
  }
  // The segment exists from the moment the peripheral creates it, a moment
  // before it constructs the region and starts servicing. Attaching in that
  // window would mean talking to a chip that has not finished powering up.
  if (bus->peripheral_ready.load(std::memory_order_acquire) == 0U)
  {
    return std::nullopt;
  }

  bus->controller_attached.store(1, std::memory_order_release);
  return I2cShmController(std::move(*segment));
}

std::optional<I2cShmController> I2cShmController::attach_within(const std::string& bus_name,
                                                                std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (true)
  {
    std::optional<I2cShmController> controller = attach(bus_name);
    if (controller.has_value())
    {
      return controller;
    }
    if (std::chrono::steady_clock::now() >= deadline)
    {
      return std::nullopt;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

I2cShmController::I2cShmController(ShmSegment segment) : segment_(std::move(segment)) {}

I2cShmController::~I2cShmController()
{
  // A moved-from controller has no mapping and must not touch the region.
  if (segment_.data() != nullptr)
  {
    region().controller_attached.store(0, std::memory_order_release);
  }
}

I2cShmController::Result I2cShmController::transaction(std::uint8_t address,
                                                       const std::uint8_t* write_bytes,
                                                       std::size_t write_length,
                                                       std::uint8_t* read_bytes,
                                                       std::size_t read_length,
                                                       std::chrono::milliseconds timeout)
{
  if (write_length > kMaxPhaseBytes || read_length > kMaxPhaseBytes)
  {
    return Result::kBusFault;
  }
  if (write_length > 0 && write_bytes == nullptr)
  {
    return Result::kBusFault;
  }

  I2cBusRegion& bus = region();
  if (bus.phase.load(std::memory_order_acquire) == phase_value(BusPhase::kShutdownRequested))
  {
    return Result::kBusFault;
  }

  if (write_length > 0)
  {
    std::memcpy(bus.write_bytes.data(), write_bytes, write_length);
  }
  bus.target_address = address & 0x7FU;
  bus.write_length = static_cast<std::uint32_t>(write_length);
  bus.read_length = static_cast<std::uint32_t>(read_length);
  bus.phase.store(phase_value(BusPhase::kTransactionPosted), std::memory_order_release);

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  int spins = 0;
  while (true)
  {
    const std::uint32_t phase = bus.phase.load(std::memory_order_acquire);
    if (phase == phase_value(BusPhase::kTransactionComplete))
    {
      break;
    }
    if (phase == phase_value(BusPhase::kShutdownRequested))
    {
      return Result::kBusFault;
    }
    if (std::chrono::steady_clock::now() >= deadline)
    {
      // The transaction is still posted and the peripheral may complete it at
      // any moment, so the bus phase is no longer something this side can
      // reason about. A timeout means the simulated part is hung or gone;
      // leave the bus alone and let the caller fail the run.
      return Result::kBusFault;
    }
    backoff(spins);
  }

  const std::uint32_t result = bus.result;
  if (result == result_value(TransactionResult::kAcknowledged) && read_length > 0 && read_bytes != nullptr)
  {
    std::memcpy(read_bytes, bus.read_bytes.data(), read_length);
  }
  bus.phase.store(phase_value(BusPhase::kIdle), std::memory_order_release);

  switch (result)
  {
    case 1U:
      return Result::kAddressNack;
    case 2U:
      return Result::kDataNack;
    default:
      return Result::kOk;
  }
}

bool I2cShmController::interrupt_line() const { return region().interrupt_line.load(std::memory_order_acquire) != 0U; }

bool I2cShmController::peripheral_present() const
{
  return region().peripheral_ready.load(std::memory_order_acquire) != 0U;
}

std::uint64_t I2cShmController::transaction_index() const
{
  return region().transaction_index.load(std::memory_order_acquire);
}

I2cBusRegion& I2cShmController::region() noexcept { return *static_cast<I2cBusRegion*>(segment_.data()); }

const I2cBusRegion& I2cShmController::region() const noexcept
{
  return *static_cast<const I2cBusRegion*>(segment_.data());
}

}  // namespace hemerion::sim::i2c_shm
