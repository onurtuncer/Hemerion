// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
#include "hemerion/sim/spi_shm/spi_shm_link.h"

#include <algorithm>
#include <cstring>
#include <new>
#include <utility>

namespace hemerion::sim::spi_shm
{

namespace
{

using shm_bridge::ShmSegment;

// Spin hot for a short while before sleeping: a transfer is answered in
// microseconds when the peer is running, and the sleep only exists so an idle
// bus does not burn a core.
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

}  // namespace

// ------------------------------------------------------------------------------
// SpiShmPeripheral
// ------------------------------------------------------------------------------

std::unique_ptr<SpiShmPeripheral> SpiShmPeripheral::create(const std::string& bus_name)
{
  std::optional<ShmSegment> segment = ShmSegment::create(bus_name, sizeof(SpiBusRegion));
  if (!segment.has_value())
  {
    return nullptr;
  }
  // The peripheral owns the region's lifetime, so it is the side that
  // constructs it -- the controller only ever reinterprets these bytes.
  new (segment->data()) SpiBusRegion{};
  return std::unique_ptr<SpiShmPeripheral>(new SpiShmPeripheral(std::move(*segment)));
}

SpiShmPeripheral::SpiShmPeripheral(ShmSegment segment) : segment_(std::move(segment)) {}

SpiShmPeripheral::~SpiShmPeripheral() { stop(); }

void SpiShmPeripheral::start(SpiPeripheral& device)
{
  if (service_thread_.joinable())
  {
    return;
  }
  stop_requested_.store(false, std::memory_order_relaxed);
  service_thread_ = std::thread([this, &device] { service_loop(device); });
  region().peripheral_ready.store(1, std::memory_order_release);
}

void SpiShmPeripheral::stop()
{
  stop_requested_.store(true, std::memory_order_relaxed);
  if (service_thread_.joinable())
  {
    service_thread_.join();
  }
  region().peripheral_ready.store(0, std::memory_order_relaxed);
  region().interrupt_line.store(0, std::memory_order_relaxed);
  // Terminal: a controller waiting on (or about to post) a transfer fails
  // fast instead of waiting out its timeout against a part that is gone.
  region().phase.store(phase_value(BusPhase::kShutdownRequested), std::memory_order_release);
}

void SpiShmPeripheral::set_data_ready(bool asserted)
{
  region().interrupt_line.store(asserted ? 1U : 0U, std::memory_order_release);
}

bool SpiShmPeripheral::controller_attached() const
{
  return region().controller_attached.load(std::memory_order_acquire) != 0U;
}

std::uint64_t SpiShmPeripheral::transfers_serviced() const
{
  return region().transfer_index.load(std::memory_order_acquire);
}

void SpiShmPeripheral::service_loop(SpiPeripheral& device)
{
  SpiBusRegion& bus = region();
  int spins = 0;
  while (!stop_requested_.load(std::memory_order_relaxed))
  {
    if (bus.phase.load(std::memory_order_acquire) != phase_value(BusPhase::kTransferPosted))
    {
      backoff(spins);
      continue;
    }
    spins = 0;

    const std::size_t length = std::min<std::size_t>(bus.length, kMaxTransferBytes);
    device.chip_select(true);
    for (std::size_t i = 0; i < length; ++i)
    {
      bus.miso[i] = device.shift(bus.mosi[i]);
    }
    device.chip_select(false);

    bus.transfer_index.fetch_add(1, std::memory_order_relaxed);
    bus.phase.store(phase_value(BusPhase::kTransferComplete), std::memory_order_release);
  }
}

SpiBusRegion& SpiShmPeripheral::region() noexcept { return *static_cast<SpiBusRegion*>(segment_.data()); }

const SpiBusRegion& SpiShmPeripheral::region() const noexcept
{
  return *static_cast<const SpiBusRegion*>(segment_.data());
}

// ------------------------------------------------------------------------------
// SpiShmController
// ------------------------------------------------------------------------------

std::optional<SpiShmController> SpiShmController::attach(const std::string& bus_name)
{
  std::optional<ShmSegment> segment = ShmSegment::open_existing(bus_name, sizeof(SpiBusRegion));
  if (!segment.has_value())
  {
    return std::nullopt;
  }

  auto* bus = static_cast<SpiBusRegion*>(segment->data());
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
  return SpiShmController(std::move(*segment));
}

std::optional<SpiShmController> SpiShmController::attach_within(const std::string& bus_name,
                                                                std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (true)
  {
    std::optional<SpiShmController> controller = attach(bus_name);
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

SpiShmController::SpiShmController(ShmSegment segment) : segment_(std::move(segment)) {}

bool SpiShmController::transfer(const std::uint8_t* tx,
                                std::uint8_t* rx,
                                std::size_t length,
                                std::chrono::milliseconds timeout)
{
  if (length == 0 || length > kMaxTransferBytes)
  {
    return false;
  }

  SpiBusRegion& bus = region();
  if (bus.phase.load(std::memory_order_acquire) == phase_value(BusPhase::kShutdownRequested))
  {
    return false;
  }

  if (tx != nullptr)
  {
    std::memcpy(bus.mosi.data(), tx, length);
  }
  else
  {
    // A half-duplex read still clocks *something* out; real drivers send
    // zeros, and so does the peripheral-side model when it sees them.
    std::memset(bus.mosi.data(), 0, length);
  }
  bus.length = static_cast<std::uint32_t>(length);
  bus.phase.store(phase_value(BusPhase::kTransferPosted), std::memory_order_release);

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  int spins = 0;
  while (true)
  {
    const std::uint32_t phase = bus.phase.load(std::memory_order_acquire);
    if (phase == phase_value(BusPhase::kTransferComplete))
    {
      break;
    }
    if (phase == phase_value(BusPhase::kShutdownRequested))
    {
      return false;
    }
    if (std::chrono::steady_clock::now() >= deadline)
    {
      // The transfer is still posted and the peripheral may complete it at any
      // moment, so the bus phase is no longer something this side can reason
      // about. A timeout means the simulated part is hung or gone; leave the
      // bus alone and let the caller fail the run.
      return false;
    }
    backoff(spins);
  }

  if (rx != nullptr)
  {
    std::memcpy(rx, bus.miso.data(), length);
  }
  bus.phase.store(phase_value(BusPhase::kIdle), std::memory_order_release);
  return true;
}

bool SpiShmController::data_ready() const
{
  return region().interrupt_line.load(std::memory_order_acquire) != 0U;
}

bool SpiShmController::peripheral_present() const
{
  return region().peripheral_ready.load(std::memory_order_acquire) != 0U;
}

std::uint64_t SpiShmController::transfer_index() const
{
  return region().transfer_index.load(std::memory_order_acquire);
}

SpiBusRegion& SpiShmController::region() noexcept { return *static_cast<SpiBusRegion*>(segment_.data()); }

const SpiBusRegion& SpiShmController::region() const noexcept
{
  return *static_cast<const SpiBusRegion*>(segment_.data());
}

}  // namespace hemerion::sim::spi_shm
