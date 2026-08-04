// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file bmp390_i2c_slave.h
/// @brief The simulated BMP390's I2C interface: register file + conversion
/// data path.
///
/// The silicon side of @ref bmp390_registers.h. Where Bmp390Driver
/// (on-target) writes configuration registers and bursts the shadowed data
/// block, this class answers them with the part's documented I2C behaviour:
///
/// * **Addressing.** The part acks its strapped 7-bit address and nothing
///   else, so a driver probing the wrong SDO strap sees the address NACK a
///   missing part produces electrically.
/// * **Register pointer.** The first byte after START in write mode sets the
///   pointer. Further write bytes come as address/data *pairs* (datasheet
///   section 5.2.3 -- unlike reads, multi-byte writes do not auto-increment).
///   Reads auto-increment from the pointer.
/// * **Data shadowing.** Reading `DATA_0` latches all six data bytes into a
///   shadow served for the rest of the burst, so a pressure/temperature pair
///   always comes from one conversion; the same latch clears the drdy status
///   bits and the INT line, as on the real part.
/// * **CMD.** A soft reset returns every user register to its power-on value
///   and drops any pending conversion; the calibration NVM survives.
///
/// **Threading.** The FMU's `do_step()` latches conversions from the
/// co-simulation master's thread while the I2C bus service thread answers
/// transactions, because a real part acks its address whenever START goes
/// out, not when its physics happens to advance. All state is therefore
/// behind one mutex, and that mutex is held for the whole transaction: the
/// part cannot latch a new conversion into the middle of a burst.
///
/// Host-only, like the rest of the fmu/ subtree -- built only when
/// HEMERION_BUILD_FMU is on (or into a unit test), never cross-compiled.

#pragma once

#include <array>
#include <cstdint>
#include <mutex>

#include "Hemerion/baro/bmp390/bmp390_registers.h"
#include "Hemerion/baro/bmp390/fmu/bmp390_reference_calibration.h"

namespace hemerion::sensors::baro::bmp390::fmu
{

/// @brief Register-accurate model of the BMP390's I2C interface.
class Bmp390I2cSlave
{
public:
  /// @param calibration    NVM burn served from registers 0x31..0x45.
  /// @param target_address 7-bit address the part acks (the SDO strap).
  explicit Bmp390I2cSlave(const Bmp390CalibData& calibration = kBmp390ReferenceCalibration,
                          std::uint8_t target_address = kBmp390I2cAddressPrimary);

  Bmp390I2cSlave(const Bmp390I2cSlave&) = delete;
  Bmp390I2cSlave& operator=(const Bmp390I2cSlave&) = delete;
  Bmp390I2cSlave(Bmp390I2cSlave&&) = delete;
  Bmp390I2cSlave& operator=(Bmp390I2cSlave&&) = delete;
  ~Bmp390I2cSlave() = default;

  // -- Bus events (the I2cAddressable contract of sim/i2c_shm) ---------------

  /// START/repeated START + address byte; returns the address ack. Acquires
  /// the internal lock for the duration of the transaction, so callers must
  /// always pair it with stop().
  [[nodiscard]] bool start(std::uint8_t address, bool read);

  /// One write-phase byte (pointer or paired data); returns the data ack.
  [[nodiscard]] bool write(std::uint8_t byte);

  /// One read-phase byte; auto-increments the register pointer.
  [[nodiscard]] std::uint8_t read(bool last);

  /// STOP condition; releases the transaction lock.
  void stop();

  // -- Physics-side API (called from the FMU's stepping thread) --------------

  /// @brief Latches one finished conversion into the data registers, as the
  /// part's measurement engine does at each ODR tick.
  ///
  /// Sets the drdy status bits for whichever of pressure/temperature is
  /// enabled in `PWR_CTRL` and asserts the INT condition if `INT_CTRL` has
  /// data-ready enabled. A conversion latched while a previous one is unread
  /// simply overwrites it -- the data registers are not a FIFO, so a
  /// controller polling slower than the ODR observes only the newest
  /// conversion (which is also why `SENSORTIME` matters: it tells the
  /// controller *when* that conversion happened).
  ///
  /// `SENSORTIME` is served as the counter value latched here -- the
  /// conversion's own timestamp. (Real silicon serves the live counter, a
  /// few hundred microseconds later by the time a poll reads it; latching at
  /// the conversion is the same idealization every other Hemerion simulator
  /// applies to its timestamps.)
  ///
  /// @param uncomp_press Raw 24-bit pressure conversion word.
  /// @param uncomp_temp  Raw 24-bit temperature conversion word.
  /// @param timestamp_us Simulation clock at this conversion [microseconds];
  ///                     stored as `SENSORTIME` ticks (32768 Hz, 24-bit
  ///                     wrap).
  void latch_conversion(std::uint32_t uncomp_press, std::uint32_t uncomp_temp, std::uint64_t timestamp_us);

  /// @brief Interval between conversions the current configuration produces
  /// [microseconds]; 0 when the part is not free-running (sleep or forced
  /// mode, or both measurements disabled).
  [[nodiscard]] std::uint64_t sampling_period_us() const;

  /// @brief True once after `PWR_CTRL` selected forced mode: the part owes
  /// exactly one conversion. Calling this consumes the obligation and drops
  /// the mode field back to sleep, as the real part does when the forced
  /// conversion completes.
  [[nodiscard]] bool take_forced_conversion();

  /// True while the part would be driving its INT pin.
  [[nodiscard]] bool interrupt_asserted() const;

  /// Power-on state (fmi2Reset's equivalent): user registers to defaults,
  /// pending data dropped; the calibration NVM and strap survive.
  void reset();

private:
  // Callers hold mutex_ for all of these.
  [[nodiscard]] std::uint8_t read_register(std::uint8_t address);
  void write_register(std::uint8_t address, std::uint8_t value);
  void apply_soft_reset();
  [[nodiscard]] bool interrupt_condition_locked() const;

  mutable std::mutex mutex_;
  // Held between start() and stop() of one transaction. Not a scoped lock,
  // because the transaction spans several calls.
  std::unique_lock<std::mutex> transaction_lock_;

  const std::array<std::uint8_t, kBmp390CalibNvmLength> nvm_;
  const std::uint8_t target_address_;

  // Live conversion words + sensor time (what the measurement engine last
  // produced) and the shadow served during a data burst.
  std::array<std::uint8_t, kBmp390DataBurstLength> data_{};
  std::array<std::uint8_t, kBmp390DataBurstLength> data_shadow_{};
  std::uint32_t sensor_time_ticks_ = 0;
  std::uint32_t sensor_time_shadow_ticks_ = 0;

  std::uint8_t status_ = kBmp390StatusCmdReady;
  bool drdy_interrupt_ = false;
  bool forced_pending_ = false;

  std::uint8_t pwr_ctrl_ = 0;
  std::uint8_t osr_ = 0;
  std::uint8_t odr_ = 0;
  std::uint8_t config_ = 0;
  std::uint8_t int_ctrl_ = 0;
  std::uint8_t if_conf_ = 0;

  // Bus state machine, valid while a transaction is open.
  std::uint8_t pointer_ = 0;
  bool awaiting_pointer_ = false;
};

}  // namespace hemerion::sensors::baro::bmp390::fmu
