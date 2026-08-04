// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/sim/i2c_shm/i2c_shm_protocol.h
//
// Wire format of a simulated I2C bus carried in shared memory: one region per
// bus, mapped by exactly two local processes -- the controller (the MCU side,
// which generates START/STOP and the clock) and the peripheral (the simulated
// part, e.g. a sensor FMU). Both sides reinterpret this struct over the same
// bytes of a named segment (see sim/shm_bridge/shm_segment.h), so it must stay
// a single stable, standard-layout block: no virtual functions, no pointers,
// no type whose representation could differ across the two binaries.
//
// Granularity. Real I2C clocks one bit at a time, but a firmware driver never
// sees bits: it issues whole transactions -- HAL_I2C_Mem_Write() is
// START, address+W, register, data, STOP, and HAL_I2C_Mem_Read() is
// START, address+W, register, repeated START, address+R, data, STOP. One
// transaction is therefore one handshake here: an optional write phase
// followed by an optional read phase under one repeated START, which covers
// every shape the HAL I2C master API can produce (including the zero-length
// address probe HAL_I2C_IsDeviceReady() sends). The peripheral side still
// walks the posted transaction through its own state machine event by event
// -- START, byte, byte, repeated START, STOP -- so the bytes it answers are
// exactly what per-bit clocking would have produced, acks included.
//
// The interrupt_line field is the peripheral's interrupt output (INT on a
// BMP390): a level the peripheral drives and the controller samples, standing
// in for the GPIO/EXTI line a board would wire between them.
// ------------------------------------------------------------------------------
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace hemerion::sim::i2c_shm
{

inline constexpr std::uint32_t kProtocolVersion = 1;

// Longest write phase / read phase the bus carries [bytes each]. Generous next
// to what a register-level sensor driver issues (a handful of bytes), and the
// whole region is allocated once per run, so there is no reason to make it
// tight.
inline constexpr std::size_t kMaxPhaseBytes = 256;

// Transaction handshake. Each transition is driven by exactly one side:
//   kIdle               -> kTransactionPosted    driven by the controller
//   kTransactionPosted  -> kTransactionComplete  driven by the peripheral
//   kTransactionComplete-> kIdle                 driven by the controller
// kShutdownRequested is set by the peripheral when it powers down and is a
// terminal state -- it never transitions back to kIdle, so a controller
// blocked mid-transaction fails fast instead of timing out.
enum class BusPhase : std::uint32_t
{
  kIdle = 0,
  kTransactionPosted = 1,
  kTransactionComplete = 2,
  kShutdownRequested = 3,
};

// How the peripheral answered a completed transaction. An address NACK is what
// an absent or unpowered part looks like electrically; a data NACK is a part
// refusing a byte mid-write. Both are bus facts a controller-side HAL reports
// distinctly, so they are carried distinctly here.
enum class TransactionResult : std::uint32_t
{
  kAcknowledged = 0,
  kAddressNack = 1,
  kDataNack = 2,
};

// The full shared-memory region. Constructed exactly once, in place, by the
// peripheral (see I2cShmPeripheral::create); the controller only ever
// reinterprets an already-constructed region -- never reconstructs it.
struct I2cBusRegion
{
  std::uint32_t protocol_version = kProtocolVersion;
  std::atomic<std::uint32_t> phase{ static_cast<std::uint32_t>(BusPhase::kIdle) };
  // Non-zero once the peripheral is servicing transactions: the simulated part
  // has finished powering up. A controller that posts before this is talking
  // to a chip that is not there yet.
  std::atomic<std::uint32_t> peripheral_ready{ 0 };
  std::atomic<std::uint32_t> controller_attached{ 0 };
  // Interrupt line, peripheral -> controller. A level, not an edge, exactly
  // like a part's INT pin: the controller samples it whenever it likes.
  std::atomic<std::uint32_t> interrupt_line{ 0 };
  std::atomic<std::uint64_t> transaction_index{ 0 };

  // Only valid while phase is kTransactionPosted (address/write/read lengths
  // and the write bytes) or kTransactionComplete (result and the read bytes):
  // the writer fills them in before the store that publishes the phase, and
  // the reader observes them after the matching load.
  std::uint32_t target_address = 0;  ///< 7-bit target address, right-aligned.
  std::uint32_t write_length = 0;    ///< Bytes in the write phase; 0 skips it.
  std::uint32_t read_length = 0;     ///< Bytes in the read phase; 0 skips it.
  std::uint32_t result = 0;          ///< TransactionResult, peripheral -> controller.
  std::array<std::uint8_t, kMaxPhaseBytes> write_bytes{};
  std::array<std::uint8_t, kMaxPhaseBytes> read_bytes{};
};

static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
              "I2cBusRegion's 32-bit atomics must be lock-free to be safely shared across processes");
static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
              "I2cBusRegion::transaction_index must be lock-free to be safely shared across processes");
static_assert(std::is_trivially_copyable_v<std::array<std::uint8_t, kMaxPhaseBytes>>,
              "The transaction buffers are copied byte-for-byte across the shm boundary");

}  // namespace hemerion::sim::i2c_shm
