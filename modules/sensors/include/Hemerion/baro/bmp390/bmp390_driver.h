// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file bmp390_driver.h
/// @brief Register-level I2C driver for the Bosch BMP390 barometer, over an
/// injected bus.
///
/// On-target code: this is what the STM32H743 firmware's baro task runs. It
/// knows the part's register map (@ref bmp390_registers.h) and the bring-up
/// sequence -- identify, soft-reset, read the calibration NVM, configure
/// oversampling/ODR/interrupt, enter normal mode, then poll the drdy status
/// and burst the shadowed data registers -- and nothing about *how* the bytes
/// reach the part.
///
/// That last part is what @ref Bmp390I2cBus abstracts, and it is the only
/// piece that differs between builds: on hardware it wraps
/// `HAL_I2C_Mem_Read/Write` (see @ref bmp390_hal_i2c_bus.h); in the host
/// co-simulation it wraps the shared-memory I2C bus (`sim/i2c_shm`) the
/// BMP390 hardware-simulator FMU answers on; under Renode it would wrap the
/// emulated I2C peripheral. The driver and the compensation below it are the
/// same object code in all three.
///
/// One virtual call per transaction is the deliberate cost of that: an I2C
/// transaction is tens of microseconds of bus time, so the indirection is
/// noise, and the alternative (templating the driver on its bus) would put
/// the register sequence in a header and recompile it per transport.
///
/// No allocation, no exceptions, no <random>, fixed-size buffers -- this
/// header is cross-compiled into firmware.

#pragma once

#include <cstddef>
#include <cstdint>

#include "Hemerion/baro/baro_types.h"
#include "Hemerion/baro/bmp390/bmp390_compensation.h"
#include "Hemerion/baro/bmp390/bmp390_registers.h"

namespace hemerion::sensors::baro::bmp390
{

/// @brief The I2C transactions the driver needs from its board.
///
/// Implementations own addressing: the driver never sees the 7-bit target
/// address, which is a wiring fact (SDO strap) the board knows. One call is
/// one complete transaction -- the granularity a HAL I2C master call has.
class Bmp390I2cBus
{
public:
  Bmp390I2cBus() = default;
  Bmp390I2cBus(const Bmp390I2cBus&) = delete;
  Bmp390I2cBus& operator=(const Bmp390I2cBus&) = delete;
  Bmp390I2cBus(Bmp390I2cBus&&) = delete;
  Bmp390I2cBus& operator=(Bmp390I2cBus&&) = delete;
  virtual ~Bmp390I2cBus() = default;

  /// @brief Writes one register: write phase of {register, value}.
  /// @return False if the transaction could not be completed (bus fault,
  ///         address/data NACK); the driver treats that as fatal for the
  ///         bring-up or sample stream rather than retrying blindly.
  [[nodiscard]] virtual bool write_register(std::uint8_t reg, std::uint8_t value) = 0;

  /// @brief Reads `count` consecutive registers starting at `reg`: a
  /// one-byte write phase (the register pointer) followed by a `count`-byte
  /// read phase under a repeated START -- the part auto-increments.
  [[nodiscard]] virtual bool read_registers(std::uint8_t reg, std::uint8_t* out, std::size_t count) = 0;

  /// @brief Blocks for at least `milliseconds` -- the soft-reset settle wait.
  virtual void delay_ms(std::uint32_t milliseconds) = 0;

  /// @brief Samples the part's INT output.
  ///
  /// Boards that do not wire the line can return false unconditionally; the
  /// driver then falls back on the `STATUS` read, which is authoritative
  /// either way.
  [[nodiscard]] virtual bool interrupt_line() const = 0;
};

/// Result of Bmp390Driver::probe().
enum class Bmp390Error : std::uint8_t
{
  kNone,              ///< The part is identified, calibrated and converting.
  kTransferFailed,    ///< The bus could not complete a transaction.
  kIdentityMismatch,  ///< `CHIP_ID` did not read back @ref kBmp390ChipId.
};

/// Result of Bmp390Driver::read_sample().
enum class Bmp390ReadResult : std::uint8_t
{
  kSample,          ///< A fresh conversion was read and compensated; `out` is valid.
  kNoNewData,       ///< The part has not finished a new conversion since the last read.
  kTransferFailed,  ///< The bus could not complete a transaction.
};

/// Configuration probe() programs into the part. The defaults are a sane
/// flight profile: x4 pressure / x1 temperature oversampling at 50 Hz, INT
/// pin asserting active-high on data ready.
struct Bmp390Config
{
  std::uint8_t pressure_oversampling = 0x02;     ///< log2: x4.
  std::uint8_t temperature_oversampling = 0x00;  ///< log2: x1.
  std::uint8_t odr_sel = kBmp390OdrSel50Hz;      ///< 50 Hz output data rate.
};

/// @brief Register-level BMP390 driver: brings the part up and turns its
/// shadowed data bursts into compensated BaroSamples.
class Bmp390Driver
{
public:
  /// @param bus Transport to drive; must outlive the driver.
  explicit Bmp390Driver(Bmp390I2cBus& bus) : bus_(bus) {}

  /// @brief Identifies, resets, calibrates and starts the part.
  ///
  /// Reads `CHIP_ID`; on a match, soft-resets (settling per
  /// @ref kBmp390SoftResetDelayMs and re-verifying identity), reads the
  /// 21-byte calibration NVM block, then programs `OSR`, `ODR`, `INT_CTRL`
  /// and finally `PWR_CTRL` into normal mode -- last, so the part starts
  /// converting with the configuration it will convert under.
  ///
  /// @return kNone once the part is converting.
  [[nodiscard]] Bmp390Error probe(const Bmp390Config& config = {});

  /// @brief Samples the part's INT line without touching the bus.
  [[nodiscard]] bool data_ready() const { return bus_.interrupt_line(); }

  /// @brief Reads `STATUS` and, when a fresh conversion is waiting, bursts
  /// the six shadowed data registers and compensates them.
  ///
  /// @param out Receives the compensated sample on kSample. `timestamp_us`
  ///            is left untouched: the part carries no clock, so stamping is
  ///            the caller's job at whatever timebase the system runs.
  /// @return kSample, kNoNewData, or kTransferFailed.
  [[nodiscard]] Bmp390ReadResult read_sample(BaroSample& out);

  /// Calibration words probe() read from the part's NVM (for logging and
  /// tests).
  [[nodiscard]] const Bmp390CalibData& calibration() const { return calib_; }

private:
  [[nodiscard]] bool read_register(Bmp390Register reg, std::uint8_t& out);

  Bmp390I2cBus& bus_;
  Bmp390CalibData calib_{};
  Bmp390Compensator compensator_{};
};

}  // namespace hemerion::sensors::baro::bmp390
