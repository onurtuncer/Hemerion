// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file imu_spi_driver.h
/// @brief Register-level SPI driver for the IMU, over an injected bus.
///
/// On-target code: this is what the STM32H743 firmware's IMU task runs. It
/// knows the part's register map (@ref imu_spi_protocol.h) and the polling
/// sequence -- sample the data-ready line, read `STATUS` + `FIFO_COUNT`, burst
/// the FIFO port, hand every byte to ImuPacketParser -- and nothing about
/// *how* the bytes reach the part.
///
/// That last part is what @ref ImuSpiBus abstracts, and it is the only piece
/// that differs between builds: on hardware it wraps `HAL_SPI_TransmitReceive`
/// plus the chip-select and data-ready GPIOs; in the host co-simulation
/// (`examples/rocket_gps_ecos`) it wraps the shared-memory SPI bus
/// (`sim/spi_shm`) the IMU hardware-simulator FMU answers on; under Renode it
/// wraps the emulated SPI peripheral. The driver, the parser and the unit
/// conversion below it are the same object code in all three.
///
/// One virtual call per transfer is the deliberate cost of that: an SPI
/// transaction is thousands of cycles of bus time, so the indirection is
/// noise, and the alternative (templating the driver on its bus) would put
/// the register sequence in a header and recompile it per transport.
///
/// No allocation, no exceptions, no <random>, fixed-size buffers -- this
/// header is cross-compiled into firmware.

#pragma once

#include <cstddef>
#include <cstdint>

#include "Hemerion/imu/imu_packet.h"
#include "Hemerion/imu/imu_spi_protocol.h"
#include "Hemerion/imu/imu_types.h"

namespace hemerion::sensors::imu
{

/// @brief The SPI transactions the driver needs from its board.
///
/// Implementations own chip select: one call is one CS-framed transfer, the
/// granularity a HAL SPI call has, and the part resets its command state
/// machine on the CS edges around it.
class ImuSpiBus
{
public:
  ImuSpiBus() = default;
  ImuSpiBus(const ImuSpiBus&) = delete;
  ImuSpiBus& operator=(const ImuSpiBus&) = delete;
  ImuSpiBus(ImuSpiBus&&) = delete;
  ImuSpiBus& operator=(ImuSpiBus&&) = delete;
  virtual ~ImuSpiBus() = default;

  /// @brief Shifts `length` bytes out of `tx` while shifting the part's answer
  /// into `rx`, framed by one chip-select assertion.
  ///
  /// @param tx     Bytes to clock out; never null for this driver's transfers.
  /// @param rx     Destination for the bytes clocked in; never null here.
  /// @param length Transfer length [bytes].
  /// @return False if the transfer could not be completed at all (bus fault,
  ///         peripheral gone); the driver treats that as fatal for the sample
  ///         stream rather than retrying blindly.
  [[nodiscard]] virtual bool transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t length) = 0;

  /// @brief Samples the part's data-ready output.
  ///
  /// Boards that do not wire the line can return true unconditionally; the
  /// driver then falls back on the `STATUS`/`FIFO_COUNT` read, which is
  /// authoritative either way.
  [[nodiscard]] virtual bool data_ready_line() const = 0;
};

/// Result of ImuSpiDriver::probe() and the fatal cases of poll().
enum class ImuSpiError : std::uint8_t
{
  kNone,              ///< The exchange completed.
  kTransferFailed,    ///< The bus could not complete a transfer.
  kIdentityMismatch,  ///< `WHO_AM_I` did not read back @ref kImuSpiWhoAmI.
};

/// @brief Register-level IMU driver: drains the part's sample FIFO into
/// decoded raw samples.
///
/// Holds one ImuPacketParser across calls, so a burst that ends mid-frame
/// resumes correctly on the next poll -- the FIFO is a byte stream, and a
/// controller has no way to align its reads to frame boundaries.
class ImuSpiDriver
{
public:
  /// Longest FIFO burst the driver issues [bytes]. Bounded so the transfer
  /// buffer stays a small fixed array on the caller's stack.
  static constexpr std::size_t kMaxBurstBytes = 128;

  /// What one poll() moved off the part.
  struct PollResult
  {
    std::size_t samples = 0;                 ///< Raw samples decoded into the caller's array.
    std::size_t bytes_read = 0;              ///< FIFO bytes clocked out of the part.
    std::size_t checksum_errors = 0;         ///< Frames the parser rejected.
    std::size_t dropped_samples = 0;         ///< Samples decoded past the caller's capacity (see poll()).
    bool fifo_overflow = false;              ///< The part reported dropping samples since the last poll.
    ImuSpiError error = ImuSpiError::kNone;  ///< kTransferFailed if the bus gave up mid-poll.
  };

  /// @param bus Transport to drive; must outlive the driver.
  explicit ImuSpiDriver(ImuSpiBus& bus) : bus_(bus) {}

  /// @brief Identifies the part and starts it buffering samples.
  ///
  /// Reads `WHO_AM_I` and, on a match, writes `CONTROL` with
  /// @ref kImuSpiControlStartup -- FIFO enabled and emptied of whatever
  /// accumulated before this driver took over.
  ///
  /// @return kNone once the part is identified and started.
  [[nodiscard]] ImuSpiError probe();

  /// @brief Samples the part's data-ready line without touching the bus.
  [[nodiscard]] bool data_ready() const { return bus_.data_ready_line(); }

  /// @brief Reads `STATUS`/`FIFO_COUNT` and drains the FIFO into `out`.
  ///
  /// Bursts are capped at @ref kMaxBurstBytes and the total read is capped at
  /// what `capacity` frames can occupy, so a full FIFO takes several polls
  /// rather than one unbounded one. Bytes are never discarded: a burst that
  /// ends mid-frame leaves the remainder in the FIFO and the partial frame in
  /// the parser.
  ///
  /// @param out      Destination for decoded raw samples.
  /// @param capacity Capacity of `out` [samples]; zero polls status only.
  /// @return Counts for this poll; `error` is kTransferFailed if the bus
  ///         failed, in which case the samples decoded before that point are
  ///         still reported.
  [[nodiscard]] PollResult poll(ImuRawSample* out, std::size_t capacity);

private:
  /// Reads `count` consecutive registers starting at `first` (auto-increment).
  [[nodiscard]] bool read_registers(ImuSpiRegister first, std::uint8_t* out, std::size_t count);

  /// Writes one register.
  [[nodiscard]] bool write_register(ImuSpiRegister reg, std::uint8_t value);

  /// Bursts `count` bytes off the FIFO port (which does not auto-increment).
  [[nodiscard]] bool burst_read_fifo(std::uint8_t* out, std::size_t count);

  ImuSpiBus& bus_;
  ImuPacketParser parser_;
};

}  // namespace hemerion::sensors::imu
