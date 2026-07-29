// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file imu_spi_slave.h
/// @brief The simulated IMU part's SPI peripheral: register file + sample FIFO.
///
/// The silicon side of @ref imu_spi_protocol.h. Where ImuSpiDriver
/// (on-target) builds command bytes and burst-reads the FIFO port, this class
/// answers them: it decodes the command byte, walks the auto-incrementing
/// register map, and pops the sample FIFO one byte at a time, exactly as the
/// part's shift register would.
///
/// What goes into that FIFO is complete Hemerion IMU raw-sample frames from
/// ImuPacketEmitter -- the same bytes the FMU used to put on a UDP socket.
/// The transport changed; the wire content did not, which is what keeps the
/// firmware-side parser byte-exact across the change.
///
/// **Threading.** The FMU's `do_step()` pushes frames from the co-simulation
/// master's thread while the SPI bus service thread shifts bytes out, because
/// a real part answers chip select whenever it is asserted, not when its
/// physics happens to advance. All state is therefore behind one mutex, and
/// that mutex is held for the whole chip-select frame: the part cannot latch a
/// new sample into the middle of a transaction, so a `FIFO_COUNT` a driver
/// just read is still true for the burst that follows it.
///
/// Host-only, like the rest of the fmu/ subtree -- built only when
/// HEMERION_BUILD_FMU is on (or into a unit test), never cross-compiled.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>

#include "Hemerion/imu/imu_spi_protocol.h"

namespace hemerion::sensors::imu::fmu
{

/// @brief Register-accurate model of the IMU part's SPI interface.
class ImuSpiSlave
{
public:
  ImuSpiSlave() = default;
  ImuSpiSlave(const ImuSpiSlave&) = delete;
  ImuSpiSlave& operator=(const ImuSpiSlave&) = delete;
  ImuSpiSlave(ImuSpiSlave&&) = delete;
  ImuSpiSlave& operator=(ImuSpiSlave&&) = delete;
  ~ImuSpiSlave() = default;

  /// @brief Buffers one complete sample frame in the FIFO.
  ///
  /// All or nothing: a frame that does not fit is dropped whole and the
  /// sticky overflow flag is set, so a controller that falls behind sees
  /// missing *samples*, never a truncated frame. (Handing out half a frame
  /// would be a fiction -- a real part's FIFO is written by the part itself,
  /// which knows where its samples end.)
  ///
  /// @param bytes  Frame bytes to buffer.
  /// @param length Frame length [bytes].
  /// @return False if the frame was dropped: FIFO disabled, no room, or a
  ///         frame longer than the FIFO.
  bool push_frame(const std::uint8_t* bytes, std::size_t length);

  /// @brief Chip-select edge; `asserted` true on the falling edge.
  ///
  /// Resets the command state machine on assert. Holds the internal lock for
  /// the duration of the frame, so callers must always pair a true with a
  /// false.
  void chip_select(bool asserted);

  /// @brief Shifts one byte through the part; call only between chip_select()
  /// edges.
  ///
  /// @param mosi Byte the controller clocked in.
  /// @return The byte the part clocked out.
  [[nodiscard]] std::uint8_t shift(std::uint8_t mosi);

  /// @brief Clears the FIFO, the sticky flags and the command state machine,
  /// leaving `CONTROL` at its power-on value (fmi2Reset's equivalent).
  void reset();

  /// FIFO fill level [bytes].
  [[nodiscard]] std::size_t fifo_used() const;

  /// True when the part would be driving its data-ready line: at least one
  /// buffered sample.
  [[nodiscard]] bool data_ready() const;

  /// Frames dropped for want of FIFO room since construction (the sticky
  /// `STATUS` bit only says "at least one, since you last looked").
  [[nodiscard]] std::uint64_t frames_dropped() const;

private:
  // Callers hold mutex_ for all of these.
  [[nodiscard]] std::uint8_t read_register(std::uint8_t address);
  void write_register(std::uint8_t address, std::uint8_t value);
  [[nodiscard]] std::uint8_t pop_fifo();
  void clear_fifo();

  mutable std::mutex mutex_;
  // Held between the chip_select() edges of one transaction. Not a scoped
  // lock, because the transaction spans several calls.
  std::unique_lock<std::mutex> transaction_lock_;

  std::array<std::uint8_t, kImuSpiFifoCapacityBytes> fifo_{};
  std::size_t fifo_head_ = 0;  ///< Next byte to pop.
  std::size_t fifo_used_ = 0;

  std::uint8_t control_ = kImuSpiControlFifoEnable;  ///< Power-on default: buffering enabled.
  bool overflow_sticky_ = false;
  std::uint64_t frames_dropped_ = 0;

  // Command state machine, valid while chip select is asserted.
  std::size_t byte_index_ = 0;
  std::uint8_t address_ = 0;
  bool reading_ = false;
};

}  // namespace hemerion::sensors::imu::fmu
