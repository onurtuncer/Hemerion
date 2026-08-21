// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file mmc5983ma_i2c_slave.h
/// @brief The simulated MMC5983MA's I2C interface: register file +
/// measurement data path + SET/RESET magnetization state.
///
/// The silicon side of @ref mmc5983ma_registers.h. Where Mmc5983maDriver
/// (on-target) writes control registers and bursts the data block, this class
/// answers them with the part's documented I2C behaviour:
///
/// * **Addressing.** The part acks @ref hemerion::sensors::mag::mmc5983ma::kMmc5983maI2cAddress "kMmc5983maI2cAddress"
/// and nothing
///   else, so a driver pointed at the wrong address sees the address NACK a
///   missing part produces electrically.
/// * **Register pointer.** The first byte after START in write mode sets the
///   pointer; further write bytes store through it and *auto-increment*, as
///   do reads (datasheet "DATA TRANSFER": "Multiple data bytes can be
///   written or read to numerically sequential registers"). That differs
///   from the BMP390's register/data write pairs -- the two models differ
///   here deliberately, because a driver carrying one part's assumption to
///   the other would silently corrupt its configuration.
/// * **Write-only control registers.** `Internal control 0..3` read back as
///   zero, because the datasheet marks them Mode: W. A driver doing
///   read-modify-write on one of them therefore reads zero and clobbers
///   every bit it meant to preserve -- on this model exactly as on the part.
///   (That is why Mmc5983maDriver shadows `Internal control 0` instead.)
/// * **Self-clearing bits.** TM_M, TM_T, Set, Reset and OTP Read take effect
///   at the write and do not persist; INT_meas_done_en and Auto_SR_en do.
/// * **Status.** The done flags are set when a measurement lands, cleared
///   when a new measurement command is issued, and cleared by writing them
///   back as ones -- the datasheet phrases that last one as clearing the
///   interrupt, and this model clears the flag with it, which is how every
///   driver in the wild reads it.
/// * **SET/RESET.** The magnetization sign the coil pulses leave behind is
///   *the* piece of state this part has that a plain register file does not.
///   It is served to the measurement model through @ref
///   hemerion::sensors::mag::mmc5983ma::fmu::Mmc5983maI2cSlave::sensing_state() "sensing_state()", so a measurement
///   taken after a RESET really comes back negated, and a driver that forgets to leave the part SET really reads a
///   sign-flipped field.
///
/// **Coherence.** The part documents no data-register shadowing, so on real
/// silicon a burst can in principle straddle two measurements. Here it
/// cannot: the transaction lock below is held from start() to stop(), so the
/// measurement engine cannot latch into the middle of a burst. The model is
/// therefore slightly *more* coherent than the part -- the same idealization
/// the BMP390 model applies to `SENSORTIME`, and worth knowing before
/// blaming the simulator for a race that only hardware shows.
///
/// **Threading.** The FMU's `do_step()` latches measurements from the
/// co-simulation master's thread while the I2C bus service thread answers
/// transactions, because a real part acks its address whenever START goes
/// out, not when its physics happens to advance. All state is therefore
/// behind one mutex.
///
/// Host-only, like the rest of the fmu/ subtree -- built only when
/// HEMERION_BUILD_FMU is on (or into a unit test), never cross-compiled.

#pragma once

#include <array>
#include <cstdint>
#include <mutex>

#include "Hemerion/mag/mmc5983ma/mmc5983ma_registers.h"

namespace hemerion::sensors::mag::mmc5983ma::fmu
{

/// @brief Register-accurate model of the MMC5983MA's I2C interface.
class Mmc5983maI2cSlave
{
public:
  /// @param target_address 7-bit address the part acks. The real part has no
  ///                       address strap -- the ordering guide defines one
  ///                       I2C address code -- so this exists to let a test
  ///                       point a driver at an address nothing answers.
  explicit Mmc5983maI2cSlave(std::uint8_t target_address = kMmc5983maI2cAddress);

  Mmc5983maI2cSlave(const Mmc5983maI2cSlave&) = delete;
  Mmc5983maI2cSlave& operator=(const Mmc5983maI2cSlave&) = delete;
  Mmc5983maI2cSlave(Mmc5983maI2cSlave&&) = delete;
  Mmc5983maI2cSlave& operator=(Mmc5983maI2cSlave&&) = delete;
  ~Mmc5983maI2cSlave() = default;

  // -- Bus events (the I2cAddressable contract of sim/i2c_shm) ---------------

  /// START/repeated START + address byte; returns the address ack. Acquires
  /// the internal lock for the duration of the transaction, so callers must
  /// always pair it with stop().
  [[nodiscard]] bool start(std::uint8_t address, bool read);

  /// One write-phase byte (pointer, then data); returns the data ack.
  [[nodiscard]] bool write(std::uint8_t byte);

  /// One read-phase byte; auto-increments the register pointer.
  [[nodiscard]] std::uint8_t read(bool last);

  /// STOP condition; releases the transaction lock.
  void stop();

  // -- Physics-side API (called from the FMU's stepping thread) --------------

  /// @brief Latches one finished field measurement into the data registers,
  /// as the part's converter does at the end of a measurement time.
  ///
  /// Sets Meas_M_Done and asserts the INT condition if INT_meas_done_en is
  /// on. A measurement latched while a previous one is unread simply
  /// overwrites it -- the data registers are not a FIFO, so a controller
  /// polling slower than the continuous rate observes only the newest.
  ///
  /// Also advances the periodic-set counter: once the configured number of
  /// measurements has gone by with En_prd_set (and its two prerequisites)
  /// on, the part re-SETs itself, which this model applies to
  /// @ref sensing_state().
  ///
  /// @param counts 18-bit unsigned counts, already null-field-centred --
  ///               i.e. what Mmc5983maMeasurementModel produced.
  void latch_measurement(const Mmc5983maFieldCounts& counts);

  /// @brief Latches one finished temperature measurement into `Tout` and
  /// sets Meas_T_Done.
  void latch_temperature(std::uint8_t tout);

  /// @brief Interval between measurements the current configuration
  /// produces [microseconds]; 0 when the part is not free-running (Cmm_en
  /// clear, or the rate field left at kOff).
  [[nodiscard]] std::uint64_t sampling_period_us() const;

  /// @brief True once after TM_M was written: the part owes exactly one
  /// field measurement. Calling this consumes the obligation, as the real
  /// part's self-clearing trigger bit does.
  [[nodiscard]] bool take_triggered_measurement();

  /// @brief True once after TM_T was written: the part owes exactly one
  /// temperature measurement.
  [[nodiscard]] bool take_triggered_temperature();

  /// The magnetization and automatic-set/reset state the control registers
  /// have left the front end in.
  [[nodiscard]] Mmc5983maSensingState sensing_state() const;

  /// True while the part would be driving its INT pin.
  [[nodiscard]] bool interrupt_asserted() const;

  /// Power-on state (fmi2Reset's equivalent): registers to defaults, pending
  /// measurements dropped. The magnetization goes to SET polarity -- see the
  /// note on @ref kPowerOnMagnetization.
  void reset();

  /// Magnetization the model comes up in. Real silicon comes up however the
  /// last field it saw left it, which is why the datasheet's bring-up ends
  /// with a SET; the model picks the benign case so that a driver skipping
  /// that SET fails on the *offset* (which no polarity hides) rather than
  /// intermittently on the sign.
  static constexpr int kPowerOnMagnetization = +1;

private:
  // Callers hold mutex_ for all of these.
  [[nodiscard]] std::uint8_t read_register(std::uint8_t address);
  void write_register(std::uint8_t address, std::uint8_t value);
  void write_control0(std::uint8_t value);
  void apply_soft_reset();
  [[nodiscard]] bool periodic_set_active_locked() const;
  [[nodiscard]] bool interrupt_condition_locked() const;

  mutable std::mutex mutex_;
  // Held between start() and stop() of one transaction. Not a scoped lock,
  // because the transaction spans several calls.
  std::unique_lock<std::mutex> transaction_lock_;

  const std::uint8_t target_address_;

  // The data registers, byte-exact: Xout0..XYZout2 as the part packs them.
  std::array<std::uint8_t, kMmc5983maDataBurstLength> data_{};
  std::uint8_t tout_ = 0;
  std::uint8_t status_ = 0;

  // Write-only control registers. Kept because their bits drive behaviour,
  // not because they can be read back -- read_register() returns zero for
  // all four, as the part does.
  std::uint8_t control1_ = 0;
  std::uint8_t control2_ = 0;
  std::uint8_t control3_ = 0;
  // The two persistent bits of the otherwise self-clearing control 0.
  bool interrupt_enable_ = false;
  bool automatic_set_reset_ = false;

  int magnetization_ = kPowerOnMagnetization;
  std::uint32_t measurements_since_set_ = 0;

  bool measurement_pending_ = false;
  bool temperature_pending_ = false;

  // Bus state machine, valid while a transaction is open.
  std::uint8_t pointer_ = 0;
  bool awaiting_pointer_ = false;
};

}  // namespace hemerion::sensors::mag::mmc5983ma::fmu
