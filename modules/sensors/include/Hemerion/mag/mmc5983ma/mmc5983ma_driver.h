// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file mmc5983ma_driver.h
/// @brief Register-level I2C driver for the MEMSIC MMC5983MA magnetometer,
/// over an injected bus.
///
/// On-target code: this is what the STM32H743 firmware's magnetometer task
/// runs. It knows the part's register map (@ref mmc5983ma_registers.h) and
/// the bring-up sequence -- identify, software-reset, program bandwidth,
/// cancel the bridge offset with a SET/RESET pair, enter continuous mode,
/// then poll `Status` and burst the seven data bytes -- and nothing about
/// *how* the bytes reach the part.
///
/// That last part is what @ref hemerion::sensors::mag::mmc5983ma::Mmc5983maI2cBus "Mmc5983maI2cBus" abstracts, and it
/// is the only piece that differs between builds: on hardware it wraps `hal_i2c_mem_read/write` (see @ref
/// mmc5983ma_hal_i2c_bus.h); in the host co-simulation it wraps the shared-memory I2C bus (`sim/i2c_shm`) the MMC5983MA
/// hardware-simulator FMU answers on; under Renode it wraps the emulated I2C peripheral. The driver is the same object
/// code in all three.
///
/// **Why there is no compensation layer here.** Unlike the BMP390 one module
/// subtree over, this part has no calibration NVM and no compensation
/// polynomial: sensitivity is the fixed scalar @ref hemerion::sensors::mag::mmc5983ma::kMmc5983maScale
/// "kMmc5983maScale", so decoding is a bit unpack, a null-field subtraction and convert_raw_to_si(). What the driver
/// *does* own instead is the bridge offset -- see @ref
/// hemerion::sensors::mag::mmc5983ma::Mmc5983maDriver::calibrate_offset() "Mmc5983maDriver::calibrate_offset()", which
/// is the only non-obvious thing about driving this part correctly.
///
/// No allocation, no exceptions, no `<random>`, fixed-size buffers -- this
/// header is cross-compiled into firmware.

#pragma once

#include <cstddef>
#include <cstdint>

#include "Hemerion/mag/mag_types.h"
#include "Hemerion/mag/mmc5983ma/mmc5983ma_registers.h"

namespace hemerion::sensors::mag::mmc5983ma
{

/// @brief The I2C transactions the driver needs from its board.
///
/// Implementations own addressing: the driver never sees the 7-bit target
/// address, which is a wiring fact the board knows. One call is one complete
/// transaction -- the granularity a HAL I2C master call has.
class Mmc5983maI2cBus
{
public:
  Mmc5983maI2cBus() = default;
  Mmc5983maI2cBus(const Mmc5983maI2cBus&) = delete;
  Mmc5983maI2cBus& operator=(const Mmc5983maI2cBus&) = delete;
  Mmc5983maI2cBus(Mmc5983maI2cBus&&) = delete;
  Mmc5983maI2cBus& operator=(Mmc5983maI2cBus&&) = delete;
  virtual ~Mmc5983maI2cBus() = default;

  /// @brief Writes one register: write phase of {register, value}.
  /// @return False if the transaction could not be completed (bus fault,
  ///         address/data NACK); the driver treats that as fatal for the
  ///         bring-up or sample stream rather than retrying blindly.
  [[nodiscard]] virtual bool write_register(std::uint8_t reg, std::uint8_t value) = 0;

  /// @brief Reads `count` consecutive registers starting at `reg`: a
  /// one-byte write phase (the register pointer) followed by a `count`-byte
  /// read phase under a repeated START -- the part auto-increments.
  [[nodiscard]] virtual bool read_registers(std::uint8_t reg, std::uint8_t* out, std::size_t count) = 0;

  /// @brief Blocks for at least `milliseconds` -- the reset settle wait and
  /// the poll interval while a triggered measurement completes.
  virtual void delay_ms(std::uint32_t milliseconds) = 0;

  /// @brief Reads a free-running monotonic clock [microseconds].
  ///
  /// This part has no on-chip time counter (the BMP390's `SENSORTIME` has no
  /// counterpart here), so a sample can only be stamped from the
  /// controller's clock -- which is what MagRawSample::timestamp_us is
  /// documented to hold. Boards without a microsecond timebase may return 0
  /// and stamp downstream.
  [[nodiscard]] virtual std::uint64_t now_us() const = 0;

  /// @brief Samples the part's INT output.
  ///
  /// Boards that do not wire the line can return false unconditionally; the
  /// driver then falls back on the `Status` read, which is authoritative
  /// either way.
  [[nodiscard]] virtual bool interrupt_line() const = 0;
};

/// Result of Mmc5983maDriver::probe() and ::calibrate_offset().
enum class Mmc5983maError : std::uint8_t
{
  kNone,                 ///< The part is identified, offset-calibrated and measuring.
  kTransferFailed,       ///< The bus could not complete a transaction.
  kIdentityMismatch,     ///< `Product ID 1` did not read back @ref kMmc5983maProductId.
  kMeasurementTimeout,   ///< A triggered measurement did not report done within the poll budget.
  kInvalidConfiguration  ///< The rate is unreachable at that bandwidth, or the call contradicts the configuration.
};

/// Result of Mmc5983maDriver::read_sample().
enum class Mmc5983maReadResult : std::uint8_t
{
  kSample,          ///< A fresh measurement was read and converted; `out` is valid.
  kNoNewData,       ///< The part has not finished a new measurement since the last read.
  kTransferFailed,  ///< The bus could not complete a transaction.
};

/// Configuration probe() programs into the part. The defaults are a sane
/// flight profile: the 8 ms bandwidth (lowest noise, 0.4 mG RMS) free-running
/// at the 50 Hz that bandwidth sustains, with the bridge offset cancelled in
/// software from one SET/RESET pair at bring-up.
struct Mmc5983maConfig
{
  /// Decimation-filter length; also the floor on measurement latency.
  Mmc5983maBandwidth bandwidth = Mmc5983maBandwidth::k100Hz;

  /// Continuous measurement rate. kOff leaves the part in one-shot mode,
  /// where read_sample() never reports a sample -- drive it with
  /// measure_once() instead.
  Mmc5983maContinuousRate continuous_rate = Mmc5983maContinuousRate::k50Hz;

  /// Let the part run its own SET/RESET pair per measurement instead of
  /// applying the software offset. Halves the sustainable output rate,
  /// makes calibrate_offset_on_probe a no-op, and makes a direct
  /// calibrate_offset() call an error; the two are alternatives.
  bool automatic_set_reset = false;

  /// Re-SET the bridges every `periodic_set` measurements. The part only
  /// honours this alongside automatic_set_reset and continuous mode, so
  /// probe() programs the enable bit only when both are on.
  Mmc5983maPeriodicSet periodic_set = Mmc5983maPeriodicSet::kEvery100;

  /// Run a SET/RESET pair during probe() and keep its offset. Turning this
  /// off is what an application does when it restores an offset measured
  /// earlier with set_bridge_offset(). Ignored -- and any stored offset
  /// cleared -- when automatic_set_reset puts the cancellation in hardware.
  bool calibrate_offset_on_probe = true;

  /// Drive the INT pin on measurement-done.
  bool interrupt_enable = true;
};

/// @brief Register-level MMC5983MA driver: brings the part up, cancels its
/// bridge offset, and turns its data bursts into MagSamples in microtesla.
class Mmc5983maDriver
{
public:
  /// Poll interval while waiting for a triggered measurement [ms]. One
  /// millisecond is well under the 8 ms longest measurement, so the wait
  /// costs at most one extra poll of latency.
  static constexpr std::uint32_t kMeasurementPollIntervalMs = 1;

  /// Polls before a triggered measurement is declared timed out. Two
  /// hundred is enormous against the 8 ms longest measurement and that is the
  /// point: it is a wedged-part threshold, not a latency budget, and the
  /// normal path leaves after five or six polls. The margin is spent under
  /// co-simulation and SWIL, where the part only advances when the FMI master
  /// steps the FMU (or when the standalone peripheral's pacing loop comes
  /// round), and each poll additionally crosses a TCP hop and a shared-memory
  /// bus -- so the budget has to exceed the slowest thing between the two
  /// ends, not the part's own conversion time.
  static constexpr std::uint32_t kMeasurementPollAttempts = 200;

  /// @param bus Transport to drive; must outlive the driver.
  explicit Mmc5983maDriver(Mmc5983maI2cBus& bus) : bus_(bus) {}

  /// @brief Identifies, resets, calibrates and starts the part.
  ///
  /// Reads `Product ID 1`; on a match, software-resets (settling per
  /// @ref kMmc5983maSoftResetDelayMs and re-verifying identity), programs
  /// `Internal control 1` (bandwidth) and `Internal control 0` (interrupt
  /// and automatic set/reset), optionally runs calibrate_offset(), and
  /// finally programs `Internal control 2` -- last, so the part starts
  /// free-running under the configuration it will measure with.
  ///
  /// @return kNone once the part is measuring.
  [[nodiscard]] Mmc5983maError probe(const Mmc5983maConfig& config = {});

  /// @brief Measures and stores the bridge offset with one SET/RESET pair.
  ///
  /// The sequence the datasheet prescribes, and the reason this part is
  /// worth its price: leave continuous mode, SET and take a measurement
  /// (`+H + offset`), RESET and take another (`-H + offset`), average the
  /// two to get the offset, re-SET so the sensing polarity is positive
  /// again, and restore continuous mode. Everything read_sample() returns
  /// afterwards has that offset subtracted.
  ///
  /// Call it at bring-up and again whenever the part may have seen a
  /// disturbing field (the datasheet's threshold is 10 gauss) or moved far
  /// in temperature. It blocks for at least two measurement times.
  ///
  /// Refused with kInvalidConfiguration when
  /// Mmc5983maConfig::automatic_set_reset is on, and not merely as
  /// redundant: that bit makes the part run its own SET/RESET pair inside
  /// every measurement and return the positive-polarity difference, so the
  /// manual pair below reads the same value twice, the field does not
  /// cancel, and the offset this would store is the standing field. The two
  /// mechanisms are alternatives, never layers.
  ///
  /// @return kNone once the offset is stored.
  [[nodiscard]] Mmc5983maError calibrate_offset();

  /// @brief Triggers one measurement and blocks until it lands.
  ///
  /// The one-shot path: what a low-rate or power-constrained task uses
  /// instead of continuous mode, and what calibrate_offset() is built from.
  /// Applies the stored bridge offset exactly as read_sample() does.
  [[nodiscard]] Mmc5983maError measure_once(MagSample& out);

  /// @brief Samples the part's INT line without touching the bus.
  [[nodiscard]] bool data_ready() const { return bus_.interrupt_line(); }

  /// @brief Reads `Status` and, when a measurement is waiting, bursts the
  /// seven data bytes, subtracts null field and bridge offset, and converts
  /// to microtesla.
  ///
  /// @param out Receives the converted sample on kSample, stamped from
  ///            Mmc5983maI2cBus::now_us() -- this part has no clock of its
  ///            own to stamp from.
  /// @return kSample, kNoNewData, or kTransferFailed.
  [[nodiscard]] Mmc5983maReadResult read_sample(MagSample& out);

  /// @brief Triggers and reads one temperature measurement.
  ///
  /// The part cannot measure field and temperature at once (the datasheet
  /// forbids raising both trigger bits), so this interrupts the sample
  /// stream: continuous mode keeps running, but the temperature measurement
  /// occupies the converter for one measurement time. Resolution is 0.8 C --
  /// die temperature for health monitoring, not a calibration input.
  [[nodiscard]] Mmc5983maError read_temperature(float& temperature_c);

  /// The bridge offset in use (from calibrate_offset(), or whatever
  /// set_bridge_offset() installed).
  [[nodiscard]] const Mmc5983maBridgeOffset& bridge_offset() const { return bridge_offset_; }

  /// @brief Installs a previously measured bridge offset -- restoring one
  /// from non-volatile storage across a reboot, so a flight does not have to
  /// begin with a blocking calibration.
  void set_bridge_offset(const Mmc5983maBridgeOffset& offset) { bridge_offset_ = offset; }

private:
  [[nodiscard]] bool read_register(Mmc5983maRegister reg, std::uint8_t& out);
  [[nodiscard]] bool write_register(Mmc5983maRegister reg, std::uint8_t value);

  // Writes `Internal control 2` from config_ -- the register that decides
  // whether the part free-runs, and the last one both probe() and
  // calibrate_offset() touch.
  [[nodiscard]] Mmc5983maError apply_continuous_mode();

  // Raises one Internal control 0 trigger bit and polls Status for its done
  // flag, clearing the flag once it lands.
  [[nodiscard]] Mmc5983maError trigger_and_wait(std::uint8_t trigger_bit, std::uint8_t done_bit);

  // Bursts the seven data registers and unpacks them; no offset applied.
  [[nodiscard]] bool read_field_counts(Mmc5983maFieldCounts& out);

  // Null field and bridge offset removed, converted to microtesla.
  [[nodiscard]] MagConversionError convert(const Mmc5983maFieldCounts& counts, MagSample& out) const;

  Mmc5983maI2cBus& bus_;
  Mmc5983maConfig config_{};
  Mmc5983maBridgeOffset bridge_offset_{};
  // Internal control 0 is write-only, so its persistent bits (interrupt
  // enable, automatic set/reset) cannot be read back before a
  // read-modify-write. The driver shadows them instead.
  std::uint8_t control0_shadow_ = 0;
};

}  // namespace hemerion::sensors::mag::mmc5983ma
