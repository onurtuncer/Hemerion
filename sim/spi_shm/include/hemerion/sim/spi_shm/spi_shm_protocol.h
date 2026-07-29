// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/sim/spi_shm/spi_shm_protocol.h
//
// Wire format of a simulated SPI bus carried in shared memory: one region per
// bus, mapped by exactly two local processes -- the controller (the MCU side,
// which drives chip select and the clock) and the peripheral (the simulated
// part, e.g. a sensor FMU). Both sides reinterpret this struct over the same
// bytes of a named segment (see sim/shm_bridge/shm_segment.h), so it must stay
// a single stable, standard-layout block: no virtual functions, no pointers,
// no type whose representation could differ across the two binaries.
//
// Granularity. Real SPI shifts one bit at a time and the peripheral's MISO
// byte k is a function of MOSI bytes 0..k-1. Handshaking over shared memory
// per *bit* or per *byte* would cost a round trip each and buy nothing: a
// firmware driver hands the peripheral a whole chip-select-framed transfer
// (HAL_SPI_TransmitReceive() and friends), never a byte at a time, so the
// controller cannot make byte k's MOSI content depend on byte k-1's MISO
// either. One transfer is therefore one handshake here -- and the peripheral
// side still shifts the posted buffer byte by byte through its own state
// machine (see SpiPeripheral in spi_shm_link.h), so the MISO bytes are exactly
// what per-bit clocking would have produced.
//
// The interrupt_line field is the peripheral's data-ready output (DRDY on most
// MEMS parts): a level the peripheral drives and the controller samples,
// standing in for the GPIO/EXTI line a board would wire between them.
// ------------------------------------------------------------------------------
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace hemerion::sim::spi_shm
{

inline constexpr std::uint32_t kProtocolVersion = 1;

// Longest single chip-select-framed transfer the bus carries. Generous next to
// the burst sizes a sensor driver uses (tens of bytes), and the whole region is
// allocated once per run, so there is no reason to make it tight.
inline constexpr std::size_t kMaxTransferBytes = 256;

// Transfer handshake. Each transition is driven by exactly one side:
//   kIdle            -> kTransferPosted     driven by the controller
//   kTransferPosted  -> kTransferComplete   driven by the peripheral
//   kTransferComplete-> kIdle               driven by the controller
// kShutdownRequested is set by the peripheral when it powers down and is a
// terminal state -- it never transitions back to kIdle, so a controller
// blocked mid-transfer fails fast instead of timing out.
enum class BusPhase : std::uint32_t
{
  kIdle = 0,
  kTransferPosted = 1,
  kTransferComplete = 2,
  kShutdownRequested = 3,
};

// The full shared-memory region. Constructed exactly once, in place, by the
// peripheral (see SpiShmPeripheral::create); the controller only ever
// reinterprets an already-constructed region -- never reconstructs it.
struct SpiBusRegion
{
  std::uint32_t protocol_version = kProtocolVersion;
  std::atomic<std::uint32_t> phase{ static_cast<std::uint32_t>(BusPhase::kIdle) };
  // Non-zero once the peripheral is servicing transfers: the simulated part
  // has finished powering up. A controller that posts before this is talking
  // to a chip that is not there yet.
  std::atomic<std::uint32_t> peripheral_ready{ 0 };
  std::atomic<std::uint32_t> controller_attached{ 0 };
  // Data-ready line, peripheral -> controller. A level, not an edge, exactly
  // like DRDY: the controller samples it whenever it likes.
  std::atomic<std::uint32_t> interrupt_line{ 0 };
  std::atomic<std::uint64_t> transfer_index{ 0 };

  // Only valid while phase is kTransferPosted (mosi/length) or
  // kTransferComplete (miso): the writer fills them in before the store that
  // publishes the phase, and the reader observes them after the matching load.
  std::uint32_t length = 0;
  std::array<std::uint8_t, kMaxTransferBytes> mosi{};
  std::array<std::uint8_t, kMaxTransferBytes> miso{};
};

static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
              "SpiBusRegion's 32-bit atomics must be lock-free to be safely shared across processes");
static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
              "SpiBusRegion::transfer_index must be lock-free to be safely shared across processes");
static_assert(std::is_trivially_copyable_v<std::array<std::uint8_t, kMaxTransferBytes>>,
              "The transfer buffers are copied byte-for-byte across the shm boundary");

}  // namespace hemerion::sim::spi_shm
