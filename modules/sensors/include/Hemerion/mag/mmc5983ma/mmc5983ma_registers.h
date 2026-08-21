// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file mmc5983ma_registers.h
/// @brief Register map and data encoding of the MEMSIC MMC5983MA three-axis
/// AMR magnetometer (datasheet MMC5983MA Rev A, 4/3/2019).
///
/// The single source of truth for both ends of the bus: the on-target driver
/// (@ref mmc5983ma_driver.h) and the hardware simulator's device model
/// (`mag/mmc5983ma/fmu/mmc5983ma_i2c_slave.h`) include this header, so a
/// change to the register map cannot land on only one side.
///
/// **I2C behaviour of the real part** (datasheet "DATA TRANSFER"): the byte
/// after the address in write mode sets the register pointer; both further
/// write bytes *and* read bytes auto-increment from it ("Multiple data bytes
/// can be written or read to numerically sequential registers without the
/// need of another START condition"). That is a real difference from the
/// BMP390 one module subtree over, whose multi-byte writes are register/data
/// pairs -- the two device models differ accordingly, and a driver that
/// assumed one part's rule on the other would corrupt its configuration.
///
/// **Data words.** Field is 18-bit *unsigned*, split across three registers
/// per axis: `Xout0` holds X[17:10], `Xout1` holds X[9:2], and `XYZout2`
/// packs the low two bits of all three axes. Null field (zero applied field)
/// reads @ref kMmc5983maNullFieldOutput, not zero, so a driver must subtract
/// it before the counts mean anything signed. Sensitivity is a fixed scalar
/// -- 16384 counts/gauss, i.e. @ref kMmc5983maLsbPerMicrotesla -- with no
/// per-part calibration NVM and no compensation polynomial, so
/// @ref hemerion::sensors::mag::convert_raw_to_si() converts the result
/// directly.
///
/// **SET/RESET.** The part carries an on-chip coil that re-magnetizes the
/// AMR bridges. After a SET the sensing polarity is positive, after a RESET
/// negative, while the bridge's *electrical* offset does not flip -- which is
/// what makes offset cancellation possible at all:
///
///     output_after_set   = null + H*S + offset
///     output_after_reset = null - H*S + offset
///     H*S    = (output_after_set - output_after_reset) / 2
///     offset = (output_after_set + output_after_reset) / 2 - null
///
/// The part's null field output is specified only to +/-0.5 gauss -- as large
/// as Earth's whole field -- so a driver that never runs that pair reads a
/// bridge offset it cannot distinguish from field. See
/// Mmc5983maDriver::calibrate_offset().
///
/// No allocation, no exceptions, constexpr throughout -- this header is
/// cross-compiled into firmware.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "Hemerion/mag/mag_types.h"

/// @namespace hemerion::sensors::mag::mmc5983ma
/// @brief Register-level driver and register map for the MEMSIC MMC5983MA.
namespace hemerion::sensors::mag::mmc5983ma
{

/// 7-bit I2C target address. Unlike the BMP390's SDO strap this is fixed in
/// silicon: the ordering guide defines a single I2C address code (3 =
/// 0110000), so two of these parts cannot share one bus.
inline constexpr std::uint8_t kMmc5983maI2cAddress = 0x30;

/// Register addresses of the part (its complete map -- there are only 14).
enum class Mmc5983maRegister : std::uint8_t
{
  kXout0 = 0x00,             ///< Read-only Xout[17:10].
  kXout1 = 0x01,             ///< Read-only Xout[9:2].
  kYout0 = 0x02,             ///< Read-only Yout[17:10].
  kYout1 = 0x03,             ///< Read-only Yout[9:2].
  kZout0 = 0x04,             ///< Read-only Zout[17:10].
  kZout1 = 0x05,             ///< Read-only Zout[9:2].
  kXyzout2 = 0x06,           ///< Read-only Xout[1:0], Yout[1:0], Zout[1:0]; bits 1:0 read zero.
  kTout = 0x07,              ///< Read-only temperature output.
  kStatus = 0x08,            ///< Measurement-done flags; write-1-to-clear.
  kInternalControl0 = 0x09,  ///< Write-only: measurement triggers, SET/RESET, interrupt enable.
  kInternalControl1 = 0x0A,  ///< Write-only: bandwidth, channel inhibit, software reset.
  kInternalControl2 = 0x0B,  ///< Write-only: continuous mode, periodic set.
  kInternalControl3 = 0x0C,  ///< Write-only: self test, 3-wire SPI.
  kProductId = 0x2F,         ///< Read-only device identity, always @ref kMmc5983maProductId.
};

/// Value `Product ID 1` always reads back on an MMC5983MA. (It coincides
/// with the part's I2C address; the two are unrelated.)
inline constexpr std::uint8_t kMmc5983maProductId = 0x30;

/// `Status` bit: a magnetic-field measurement finished. Set when the
/// measurement completes, cleared when a new measurement command is issued,
/// and -- per the datasheet's "Writing 1 into this bit will clear the
/// corresponding interrupt" -- cleared by writing it back as a one.
inline constexpr std::uint8_t kMmc5983maStatusMeasurementDone = 0x01;
/// `Status` bit: a temperature measurement finished; same clearing rules.
inline constexpr std::uint8_t kMmc5983maStatusTemperatureDone = 0x02;
/// `Status` bit: the part successfully read its OTP memory at start-up.
inline constexpr std::uint8_t kMmc5983maStatusOtpReadDone = 0x10;

/// `Internal control 0` bit: take one magnetic-field measurement.
/// Self-clearing at the end of the measurement.
inline constexpr std::uint8_t kMmc5983maControl0TakeMeasurement = 0x01;
/// `Internal control 0` bit: take one temperature measurement. Self-clearing,
/// and the datasheet forbids raising it together with @ref
/// kMmc5983maControl0TakeMeasurement.
inline constexpr std::uint8_t kMmc5983maControl0TakeTemperature = 0x02;
/// `Internal control 0` bit: drive the INT pin when a measurement completes.
inline constexpr std::uint8_t kMmc5983maControl0InterruptEnable = 0x04;
/// `Internal control 0` bit: pulse the coil in the SET direction (500 ns).
/// Self-clearing at the end of the operation.
inline constexpr std::uint8_t kMmc5983maControl0Set = 0x08;
/// `Internal control 0` bit: pulse the coil in the RESET direction (500 ns).
/// Self-clearing at the end of the operation.
inline constexpr std::uint8_t kMmc5983maControl0Reset = 0x10;
/// `Internal control 0` bit: let the part run its own SET/RESET pair per
/// measurement and output the offset-cancelled difference. Costs two
/// measurement times per sample, and makes a *manual* pair actively wrong
/// rather than merely redundant: both halves come back positive-polarity, so
/// the field no longer cancels out of their average and
/// Mmc5983maDriver::calibrate_offset() would store the standing field as an
/// offset. It refuses instead. The two mechanisms are alternatives, never
/// layers.
inline constexpr std::uint8_t kMmc5983maControl0AutoSetResetEnable = 0x20;
/// `Internal control 0` bit: re-read the OTP shadow registers. Self-clearing.
inline constexpr std::uint8_t kMmc5983maControl0OtpRead = 0x40;

/// Mask of the `Internal control 1` bandwidth field (bits 1:0).
inline constexpr std::uint8_t kMmc5983maControl1BandwidthMask = 0x03;
/// `Internal control 1` bit: disable the X channel.
inline constexpr std::uint8_t kMmc5983maControl1InhibitX = 0x04;
/// `Internal control 1` bits: disable the Y and Z channels (both bits).
inline constexpr std::uint8_t kMmc5983maControl1InhibitYz = 0x18;
/// `Internal control 1` bit: software reset -- clears every register and
/// re-reads OTP, as a power-up does. Settle per @ref
/// kMmc5983maSoftResetDelayMs.
inline constexpr std::uint8_t kMmc5983maControl1SoftReset = 0x80;

/// Mask of the `Internal control 2` continuous-rate field (bits 2:0).
inline constexpr std::uint8_t kMmc5983maControl2ContinuousRateMask = 0x07;
/// `Internal control 2` bit: enter continuous measurement mode. The datasheet
/// requires a non-zero rate field alongside it.
inline constexpr std::uint8_t kMmc5983maControl2ContinuousEnable = 0x08;
/// Mask of the `Internal control 2` periodic-set field (bits 6:4).
inline constexpr std::uint8_t kMmc5983maControl2PeriodicSetMask = 0x70;
/// Bit position of the `Internal control 2` periodic-set field.
inline constexpr unsigned kMmc5983maControl2PeriodicSetShift = 4;
/// `Internal control 2` bit: enable periodic SET. The datasheet requires
/// @ref kMmc5983maControl0AutoSetResetEnable and
/// @ref kMmc5983maControl2ContinuousEnable to be set as well.
inline constexpr std::uint8_t kMmc5983maControl2PeriodicSetEnable = 0x80;

/// `Internal control 3` bit: self-test, positive coil current (St_enp).
inline constexpr std::uint8_t kMmc5983maControl3SelfTestPositive = 0x02;
/// `Internal control 3` bit: self-test, negative coil current (St_enm).
inline constexpr std::uint8_t kMmc5983maControl3SelfTestNegative = 0x04;
/// `Internal control 3` bit: switch the SPI interface to 3-wire mode.
inline constexpr std::uint8_t kMmc5983maControl3Spi3Wire = 0x40;

/// Measurement bandwidth: the length of the decimation filter, and so the
/// duration of one measurement (datasheet `Internal control 1`, BW1:BW0).
enum class Mmc5983maBandwidth : std::uint8_t
{
  k100Hz = 0x00,  ///< 8 ms per measurement; lowest noise (0.4 mG RMS), max 50 Hz output.
  k200Hz = 0x01,  ///< 4 ms; 0.6 mG RMS, max 100 Hz output.
  k400Hz = 0x02,  ///< 2 ms; 0.8 mG RMS, max 225 Hz output.
  k800Hz = 0x03,  ///< 0.5 ms; 1.2 mG RMS, max 580 Hz output.
};

/// Continuous measurement rate (datasheet `Internal control 2`, CM_Freq).
/// The tabulated frequencies assume the 8 ms bandwidth; the top two entries
/// are reachable only at the bandwidths named against them.
enum class Mmc5983maContinuousRate : std::uint8_t
{
  kOff = 0x00,     ///< Continuous mode off (and rejected while Cmm_en is set).
  k1Hz = 0x01,     ///< 1 Hz.
  k10Hz = 0x02,    ///< 10 Hz.
  k20Hz = 0x03,    ///< 20 Hz.
  k50Hz = 0x04,    ///< 50 Hz -- the fastest the 8 ms bandwidth sustains.
  k100Hz = 0x05,   ///< 100 Hz.
  k200Hz = 0x06,   ///< 200 Hz; requires Mmc5983maBandwidth::k200Hz.
  k1000Hz = 0x07,  ///< 1000 Hz; requires Mmc5983maBandwidth::k800Hz.
};

/// Periodic-set interval (datasheet `Internal control 2`, Prd_set): how many
/// measurements the part takes between automatic SET operations. This
/// re-magnetizes the bridges after a disturbing field; it is not offset
/// cancellation.
enum class Mmc5983maPeriodicSet : std::uint8_t
{
  kEvery1 = 0x00,     ///< SET before every measurement.
  kEvery25 = 0x01,    ///< Every 25 measurements.
  kEvery75 = 0x02,    ///< Every 75.
  kEvery100 = 0x03,   ///< Every 100.
  kEvery250 = 0x04,   ///< Every 250.
  kEvery500 = 0x05,   ///< Every 500.
  kEvery1000 = 0x06,  ///< Every 1000.
  kEvery2000 = 0x07,  ///< Every 2000.
};

/// Reading of zero applied field, 18-bit mode (datasheet specifications):
/// mid-scale, not zero. Its part-to-part tolerance is +/-0.5 gauss, which is
/// the bridge offset SET/RESET exists to remove.
inline constexpr std::uint32_t kMmc5983maNullFieldOutput = 131072;

/// Largest value the 18-bit converter can produce.
inline constexpr std::uint32_t kMmc5983maFullScaleCounts = 0x0003FFFF;

/// Sensitivity, 18-bit mode [counts per gauss] (datasheet specifications).
inline constexpr float kMmc5983maCountsPerGauss = 16384.0F;

/// Sensitivity in Hemerion's working unit [LSB per uT]: 16384 counts/gauss
/// over 100 uT/gauss. A fixed scalar -- there is no per-part calibration to
/// read and no compensation polynomial to run.
inline constexpr float kMmc5983maLsbPerMicrotesla = kMmc5983maCountsPerGauss / 100.0F;

/// The sensitivity a driver must hand to convert_raw_to_si() for this part.
inline constexpr MagScale kMmc5983maScale{ kMmc5983maLsbPerMicrotesla };

/// Field range per axis [uT]: the datasheet's +/-8 gauss.
inline constexpr float kMmc5983maFieldRangeUt = 800.0F;

/// Temperature output offset [degrees C]: `Tout` == 0 means -75 C.
inline constexpr float kMmc5983maTemperatureOffsetC = -75.0F;
/// Temperature output step [degrees C per count].
inline constexpr float kMmc5983maTemperatureStepC = 0.8F;

/// Length of the field data burst from `Xout0` through `XYZout2` [bytes].
inline constexpr std::size_t kMmc5983maDataBurstLength = 7;

/// Time the part needs after a software reset before it answers again [ms]
/// (datasheet: "The power on time is 10 mS"; rounded up for margin).
inline constexpr std::uint32_t kMmc5983maSoftResetDelayMs = 15;

/// One SET or RESET coil pulse [nanoseconds]. Far shorter than the I2C bus
/// free time between the STOP that ends the triggering write and the START
/// that begins anything else, so a driver needs no explicit wait after one.
inline constexpr std::uint32_t kMmc5983maSetResetPulseNs = 500;

/// @brief Duration of one measurement at the given bandwidth [microseconds]
/// (datasheet `Internal control 1`, BW0/BW1 table).
[[nodiscard]] constexpr std::uint32_t mmc5983ma_measurement_time_us(Mmc5983maBandwidth bandwidth)
{
  switch (bandwidth)
  {
    case Mmc5983maBandwidth::k100Hz:
      return 8000;
    case Mmc5983maBandwidth::k200Hz:
      return 4000;
    case Mmc5983maBandwidth::k400Hz:
      return 2000;
    case Mmc5983maBandwidth::k800Hz:
      return 500;
  }
  return 8000;
}

/// @brief Total RMS noise at the given bandwidth [uT] (datasheet
/// specifications, in mG: 0.4 / 0.6 / 0.8 / 1.2). Lives here rather than in
/// the simulator so driver-side noise budgeting and the hardware simulator
/// quote the same datasheet number.
[[nodiscard]] constexpr float mmc5983ma_rms_noise_ut(Mmc5983maBandwidth bandwidth)
{
  switch (bandwidth)
  {
    case Mmc5983maBandwidth::k100Hz:
      return 0.04F;
    case Mmc5983maBandwidth::k200Hz:
      return 0.06F;
    case Mmc5983maBandwidth::k400Hz:
      return 0.08F;
    case Mmc5983maBandwidth::k800Hz:
      return 0.12F;
  }
  return 0.04F;
}

/// @brief Interval between measurements the continuous rate field selects
/// [microseconds]; 0 for Mmc5983maContinuousRate::kOff.
[[nodiscard]] constexpr std::uint64_t mmc5983ma_continuous_period_us(Mmc5983maContinuousRate rate)
{
  switch (rate)
  {
    case Mmc5983maContinuousRate::kOff:
      return 0;
    case Mmc5983maContinuousRate::k1Hz:
      return 1000000;
    case Mmc5983maContinuousRate::k10Hz:
      return 100000;
    case Mmc5983maContinuousRate::k20Hz:
      return 50000;
    case Mmc5983maContinuousRate::k50Hz:
      return 20000;
    case Mmc5983maContinuousRate::k100Hz:
      return 10000;
    case Mmc5983maContinuousRate::k200Hz:
      return 5000;
    case Mmc5983maContinuousRate::k1000Hz:
      return 1000;
  }
  return 0;
}

/// @brief Measurements between automatic SET operations, as the periodic-set
/// field encodes them (datasheet Prd_set table).
[[nodiscard]] constexpr std::uint32_t mmc5983ma_periodic_set_interval(Mmc5983maPeriodicSet periodic_set)
{
  switch (periodic_set)
  {
    case Mmc5983maPeriodicSet::kEvery1:
      return 1;
    case Mmc5983maPeriodicSet::kEvery25:
      return 25;
    case Mmc5983maPeriodicSet::kEvery75:
      return 75;
    case Mmc5983maPeriodicSet::kEvery100:
      return 100;
    case Mmc5983maPeriodicSet::kEvery250:
      return 250;
    case Mmc5983maPeriodicSet::kEvery500:
      return 500;
    case Mmc5983maPeriodicSet::kEvery1000:
      return 1000;
    case Mmc5983maPeriodicSet::kEvery2000:
      return 2000;
  }
  return 1;
}

/// @brief Builds an `Internal control 2` register value.
[[nodiscard]] constexpr std::uint8_t mmc5983ma_control2(Mmc5983maContinuousRate rate,
                                                        bool continuous_enable,
                                                        Mmc5983maPeriodicSet periodic_set,
                                                        bool periodic_set_enable)
{
  std::uint8_t value =
      static_cast<std::uint8_t>(static_cast<std::uint8_t>(rate) & kMmc5983maControl2ContinuousRateMask);
  if (continuous_enable)
  {
    value |= kMmc5983maControl2ContinuousEnable;
  }
  value |= static_cast<std::uint8_t>((static_cast<std::uint8_t>(periodic_set) << kMmc5983maControl2PeriodicSetShift) &
                                     kMmc5983maControl2PeriodicSetMask);
  if (periodic_set_enable)
  {
    value |= kMmc5983maControl2PeriodicSetEnable;
  }
  return value;
}

/// @brief Converts a `Tout` count to degrees Celsius.
[[nodiscard]] constexpr float mmc5983ma_temperature_c(std::uint8_t tout)
{
  return kMmc5983maTemperatureOffsetC + (static_cast<float>(tout) * kMmc5983maTemperatureStepC);
}

/// @brief Converts degrees Celsius to a `Tout` count, saturating at the
/// sensor's -75..+125 C span as the real converter does.
[[nodiscard]] constexpr std::uint8_t mmc5983ma_encode_temperature(float temperature_c)
{
  const float counts = (temperature_c - kMmc5983maTemperatureOffsetC) / kMmc5983maTemperatureStepC;
  if (counts <= 0.0F)
  {
    return 0;
  }
  if (counts >= 255.0F)
  {
    return 255;
  }
  return static_cast<std::uint8_t>(counts + 0.5F);
}

/// The magnetic front end's state, as the control bits have left it. Both
/// ends of the bus reason about it -- the device model owns it, the
/// measurement model needs it to know what counts a truth field produces,
/// and the driver exists largely to keep it in the one state that reads
/// correctly -- so it lives here with the register map that defines it.
struct Mmc5983maSensingState
{
  /// +1 after a SET, -1 after a RESET: the sign the applied field enters the
  /// bridges with. The bridge's own offset does not follow it, which is what
  /// makes the pair difference cancel one and keep the other.
  int magnetization = +1;

  /// Auto_SR_en: the part runs its own SET/RESET pair per measurement and
  /// outputs the difference, so the result is positive-polarity and already
  /// free of bridge offset regardless of @ref magnetization.
  bool automatic_set_reset = false;
};

/// A per-axis offset in raw counts: what a SET/RESET pair measures, what the
/// driver subtracts from every sample, and what the simulated part is born
/// with.
struct Mmc5983maBridgeOffset
{
  std::int32_t x = 0;  ///< X bridge offset [LSB].
  std::int32_t y = 0;  ///< Y bridge offset [LSB].
  std::int32_t z = 0;  ///< Z bridge offset [LSB].
};

/// One conversion's three axes as 18-bit unsigned counts, exactly as the
/// data registers hold them -- i.e. centred on @ref
/// kMmc5983maNullFieldOutput, not on zero.
struct Mmc5983maFieldCounts
{
  std::uint32_t x = kMmc5983maNullFieldOutput;  ///< Xout[17:0].
  std::uint32_t y = kMmc5983maNullFieldOutput;  ///< Yout[17:0].
  std::uint32_t z = kMmc5983maNullFieldOutput;  ///< Zout[17:0].
};

/// @brief Decodes the 7-byte data burst into 18-bit counts (datasheet
/// register details for Xout0/Xout1/XYZout2 and its Y and Z twins).
[[nodiscard]] constexpr Mmc5983maFieldCounts
decode_mmc5983ma_field(const std::array<std::uint8_t, kMmc5983maDataBurstLength>& burst)
{
  const auto axis = [&burst](std::size_t high, std::size_t mid, unsigned low_shift) {
    return (static_cast<std::uint32_t>(burst[high]) << 10) | (static_cast<std::uint32_t>(burst[mid]) << 2) |
           ((static_cast<std::uint32_t>(burst[6]) >> low_shift) & 0x03U);
  };
  Mmc5983maFieldCounts counts;
  counts.x = axis(0, 1, 6);
  counts.y = axis(2, 3, 4);
  counts.z = axis(4, 5, 2);
  return counts;
}

/// @brief Encodes 18-bit counts back into the 7-byte data burst -- the
/// inverse of @ref decode_mmc5983ma_field(), used by the hardware simulator
/// to fill its data registers so the real driver decodes real bytes.
/// `XYZout2`'s low two bits read zero on the part and do here too.
[[nodiscard]] constexpr std::array<std::uint8_t, kMmc5983maDataBurstLength>
encode_mmc5983ma_field(const Mmc5983maFieldCounts& counts)
{
  std::array<std::uint8_t, kMmc5983maDataBurstLength> burst{};
  const auto put = [&burst](std::size_t high, std::size_t mid, unsigned low_shift, std::uint32_t value) {
    burst[high] = static_cast<std::uint8_t>((value >> 10) & 0xFFU);
    burst[mid] = static_cast<std::uint8_t>((value >> 2) & 0xFFU);
    burst[6] |= static_cast<std::uint8_t>((value & 0x03U) << low_shift);
  };
  put(0, 1, 6, counts.x);
  put(2, 3, 4, counts.y);
  put(4, 5, 2, counts.z);
  return burst;
}

}  // namespace hemerion::sensors::mag::mmc5983ma
