// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file bmp390_registers.h
/// @brief Register map and calibration NVM layout of the Bosch BMP390
/// barometric pressure sensor (datasheet BST-BMP390-DS002).
///
/// The single source of truth for both ends of the bus: the on-target driver
/// (@ref bmp390_driver.h) and the hardware simulator's device model
/// (`baro/bmp390/fmu/bmp390_i2c_slave.h`) include this header, so a change to
/// the register map cannot land on only one side.
///
/// **I2C behaviour of the real part** (datasheet section 5.2): the byte after
/// the address in write mode sets the register pointer; further write bytes
/// store through it and reads auto-increment from it, so `DATA_0..DATA_5`
/// come out of one six-byte burst -- which is also the only way to read a
/// pressure/temperature pair guaranteed to come from the same conversion
/// (shadowing: the part latches all six on the read of `DATA_0`).
///
/// **Data words.** Pressure and temperature are 24-bit unsigned raw
/// conversion words, xlsb first (`DATA_0` = press_xlsb ... `DATA_5` =
/// temp_msb). They mean nothing in SI units until run through the
/// per-part calibration polynomial -- see @ref bmp390_compensation.h and the
/// NVM block parsed by @ref hemerion::sensors::baro::bmp390::parse_bmp390_calibration() "parse_bmp390_calibration()".
///
/// No allocation, no exceptions, constexpr throughout -- this header is
/// cross-compiled into firmware.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace hemerion::sensors::baro::bmp390
{

/// 7-bit I2C target address with SDO tied to GND.
inline constexpr std::uint8_t kBmp390I2cAddressPrimary = 0x76;
/// 7-bit I2C target address with SDO tied to VDDIO.
inline constexpr std::uint8_t kBmp390I2cAddressSecondary = 0x77;

/// Register addresses of the part (the subset Hemerion drives).
enum class Bmp390Register : std::uint8_t
{
  kChipId = 0x00,       ///< Read-only device identity, always @ref kBmp390ChipId.
  kRevId = 0x01,        ///< Read-only mask revision.
  kErrReg = 0x02,       ///< Read-only error flags.
  kStatus = 0x03,       ///< Read-only status: cmd_rdy / drdy_press / drdy_temp.
  kData0 = 0x04,        ///< press_xlsb -- start of the 6-byte shadowed data burst.
  kData1 = 0x05,        ///< press_lsb.
  kData2 = 0x06,        ///< press_msb.
  kData3 = 0x07,        ///< temp_xlsb.
  kData4 = 0x08,        ///< temp_lsb.
  kData5 = 0x09,        ///< temp_msb.
  kSensorTime0 = 0x0C,  ///< Sensor time counter, low byte.
  kSensorTime1 = 0x0D,  ///< Sensor time counter, middle byte.
  kSensorTime2 = 0x0E,  ///< Sensor time counter, high byte.
  kIntStatus = 0x11,    ///< Read-only interrupt status; clears on read.
  kIntCtrl = 0x19,      ///< Interrupt pin configuration.
  kIfConf = 0x1A,       ///< Serial interface configuration.
  kPwrCtrl = 0x1B,      ///< Measurement enables + power mode.
  kOsr = 0x1C,          ///< Pressure/temperature oversampling.
  kOdr = 0x1D,          ///< Output data rate (normal mode subdivision).
  kConfig = 0x1F,       ///< IIR filter coefficient.
  kCalibNvm = 0x31,     ///< First byte of the 21-byte calibration NVM block.
  kCmd = 0x7E,          ///< Command register: soft reset, FIFO flush.
};

/// Value `CHIP_ID` always reads back on a BMP390.
inline constexpr std::uint8_t kBmp390ChipId = 0x60;

/// `STATUS` bit: the command decoder is ready for a new `CMD` write.
inline constexpr std::uint8_t kBmp390StatusCmdReady = 0x10;
/// `STATUS` bit: a fresh pressure word is waiting; cleared when it is read.
inline constexpr std::uint8_t kBmp390StatusPressureReady = 0x20;
/// `STATUS` bit: a fresh temperature word is waiting; cleared when it is read.
inline constexpr std::uint8_t kBmp390StatusTemperatureReady = 0x40;

/// `INT_STATUS` bit: data ready; cleared on read.
inline constexpr std::uint8_t kBmp390IntStatusDataReady = 0x08;

/// `INT_CTRL` bit: interrupt pin active high.
inline constexpr std::uint8_t kBmp390IntCtrlActiveHigh = 0x02;
/// `INT_CTRL` bit: assert the pin on data ready.
inline constexpr std::uint8_t kBmp390IntCtrlDataReadyEnable = 0x40;

/// `PWR_CTRL` bit: enable the pressure sensor.
inline constexpr std::uint8_t kBmp390PwrCtrlPressureEnable = 0x01;
/// `PWR_CTRL` bit: enable the temperature sensor.
inline constexpr std::uint8_t kBmp390PwrCtrlTemperatureEnable = 0x02;
/// `PWR_CTRL` mode field (bits 5:4): sleep -- no conversions.
inline constexpr std::uint8_t kBmp390PwrCtrlModeSleep = 0x00;
/// `PWR_CTRL` mode field: forced -- one conversion, then back to sleep.
inline constexpr std::uint8_t kBmp390PwrCtrlModeForced = 0x10;
/// `PWR_CTRL` mode field: normal -- free-running conversions at the ODR.
inline constexpr std::uint8_t kBmp390PwrCtrlModeNormal = 0x30;
/// Mask of the `PWR_CTRL` mode field.
inline constexpr std::uint8_t kBmp390PwrCtrlModeMask = 0x30;

/// `CMD` value: soft reset -- user configuration is reset to power-on
/// defaults; the calibration NVM survives.
inline constexpr std::uint8_t kBmp390CmdSoftReset = 0xB6;
/// `CMD` value: flush the FIFO (Hemerion does not use the part's FIFO).
inline constexpr std::uint8_t kBmp390CmdFifoFlush = 0xB0;

/// Time the part needs after a soft reset before it answers again [ms]
/// (datasheet start-up time 2 ms; rounded up for margin).
inline constexpr std::uint32_t kBmp390SoftResetDelayMs = 5;

/// Length of the shadowed pressure+temperature data burst [bytes].
inline constexpr std::size_t kBmp390DataBurstLength = 6;

/// Rate of the part's free-running `SENSORTIME` counter [Hz]: a 24-bit word
/// at 32768 ticks/s, wrapping every 512 s. Reading it in the same burst as
/// the data registers is how a driver timestamps conversions from the part's
/// own clock instead of guessing from its poll cadence.
inline constexpr std::uint32_t kBmp390SensorTimeTickHz = 32768;

/// Length of the extended burst from `DATA_0` through `SENSORTIME_2`
/// [bytes]: the six data bytes, the two reserved addresses 0x0A/0x0B (which
/// read zero and are clocked past by auto-increment), and the three counter
/// bytes.
inline constexpr std::size_t kBmp390DataSensorTimeBurstLength = 11;

/// Length of the calibration NVM block at @ref Bmp390Register::kCalibNvm [bytes].
inline constexpr std::size_t kBmp390CalibNvmLength = 21;

/// @brief Builds an `OSR` register value.
/// @param pressure_oversampling    log2 of the pressure oversampling (0 = x1 ... 5 = x32).
/// @param temperature_oversampling log2 of the temperature oversampling.
[[nodiscard]] constexpr std::uint8_t bmp390_osr(std::uint8_t pressure_oversampling,
                                                std::uint8_t temperature_oversampling)
{
  return static_cast<std::uint8_t>((pressure_oversampling & 0x07U) |
                                   (static_cast<std::uint8_t>(temperature_oversampling & 0x07U) << 3));
}

/// @brief Sampling period the `ODR` register selects [microseconds]:
/// 5 ms x 2^odr_sel (datasheet table 45: 0x00 = 200 Hz ... 0x11 = one sample
/// per 17.1 min).
[[nodiscard]] constexpr std::uint64_t bmp390_odr_period_us(std::uint8_t odr_sel)
{
  return 5000ULL << (odr_sel & 0x1FU);
}

/// `ODR` value for 50 Hz (5 ms x 2^2 = 20 ms), the rate Hemerion configures.
inline constexpr std::uint8_t kBmp390OdrSel50Hz = 0x02;

/// Raw (unscaled) calibration words as burned into the part's NVM, in the
/// order and widths of datasheet table 21. These are what the 21-byte block
/// at @ref Bmp390Register::kCalibNvm decodes to; @ref bmp390_compensation.h
/// turns them into the floating-point coefficients the compensation runs on.
struct Bmp390CalibData
{
  std::uint16_t par_t1 = 0;
  std::uint16_t par_t2 = 0;
  std::int8_t par_t3 = 0;
  std::int16_t par_p1 = 0;
  std::int16_t par_p2 = 0;
  std::int8_t par_p3 = 0;
  std::int8_t par_p4 = 0;
  std::uint16_t par_p5 = 0;
  std::uint16_t par_p6 = 0;
  std::int8_t par_p7 = 0;
  std::int8_t par_p8 = 0;
  std::int16_t par_p9 = 0;
  std::int8_t par_p10 = 0;
  std::int8_t par_p11 = 0;
};

/// @brief Decodes the 21-byte calibration NVM block (little-endian
/// multi-byte words, datasheet table 21).
[[nodiscard]] constexpr Bmp390CalibData
parse_bmp390_calibration(const std::array<std::uint8_t, kBmp390CalibNvmLength>& nvm)
{
  const auto u16 = [&nvm](std::size_t offset) {
    return static_cast<std::uint16_t>(nvm[offset] | (static_cast<std::uint16_t>(nvm[offset + 1]) << 8));
  };
  Bmp390CalibData calib;
  calib.par_t1 = u16(0);
  calib.par_t2 = u16(2);
  calib.par_t3 = static_cast<std::int8_t>(nvm[4]);
  calib.par_p1 = static_cast<std::int16_t>(u16(5));
  calib.par_p2 = static_cast<std::int16_t>(u16(7));
  calib.par_p3 = static_cast<std::int8_t>(nvm[9]);
  calib.par_p4 = static_cast<std::int8_t>(nvm[10]);
  calib.par_p5 = u16(11);
  calib.par_p6 = u16(13);
  calib.par_p7 = static_cast<std::int8_t>(nvm[15]);
  calib.par_p8 = static_cast<std::int8_t>(nvm[16]);
  calib.par_p9 = static_cast<std::int16_t>(u16(17));
  calib.par_p10 = static_cast<std::int8_t>(nvm[19]);
  calib.par_p11 = static_cast<std::int8_t>(nvm[20]);
  return calib;
}

/// @brief Encodes calibration words back into the 21-byte NVM block --
/// the inverse of @ref parse_bmp390_calibration(), used by the hardware
/// simulator to populate its NVM registers so the real driver reads real
/// bytes.
[[nodiscard]] constexpr std::array<std::uint8_t, kBmp390CalibNvmLength>
encode_bmp390_calibration(const Bmp390CalibData& calib)
{
  std::array<std::uint8_t, kBmp390CalibNvmLength> nvm{};
  const auto put_u16 = [&nvm](std::size_t offset, std::uint16_t value) {
    nvm[offset] = static_cast<std::uint8_t>(value & 0xFFU);
    nvm[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFFU);
  };
  put_u16(0, calib.par_t1);
  put_u16(2, calib.par_t2);
  nvm[4] = static_cast<std::uint8_t>(calib.par_t3);
  put_u16(5, static_cast<std::uint16_t>(calib.par_p1));
  put_u16(7, static_cast<std::uint16_t>(calib.par_p2));
  nvm[9] = static_cast<std::uint8_t>(calib.par_p3);
  nvm[10] = static_cast<std::uint8_t>(calib.par_p4);
  put_u16(11, calib.par_p5);
  put_u16(13, calib.par_p6);
  nvm[15] = static_cast<std::uint8_t>(calib.par_p7);
  nvm[16] = static_cast<std::uint8_t>(calib.par_p8);
  put_u16(17, static_cast<std::uint16_t>(calib.par_p9));
  nvm[19] = static_cast<std::uint8_t>(calib.par_p10);
  nvm[20] = static_cast<std::uint8_t>(calib.par_p11);
  return nvm;
}

}  // namespace hemerion::sensors::baro::bmp390
