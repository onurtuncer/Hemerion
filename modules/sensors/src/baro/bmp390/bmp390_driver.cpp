// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file bmp390_driver.cpp
/// @brief Implements the register-level BMP390 driver declared in
/// bmp390_driver.h.

#include "Hemerion/baro/bmp390/bmp390_driver.h"

#include <array>

namespace hemerion::sensors::baro::bmp390
{

namespace
{

[[nodiscard]] constexpr std::uint8_t reg_address(Bmp390Register reg) { return static_cast<std::uint8_t>(reg); }

// Assembles a 24-bit conversion word from its xlsb/lsb/msb bytes.
[[nodiscard]] constexpr std::uint32_t word24(std::uint8_t xlsb, std::uint8_t lsb, std::uint8_t msb)
{
  return static_cast<std::uint32_t>(xlsb) | (static_cast<std::uint32_t>(lsb) << 8) |
         (static_cast<std::uint32_t>(msb) << 16);
}

}  // namespace

Bmp390Error Bmp390Driver::probe(const Bmp390Config& config)
{
  // Identify before touching anything: a soft reset sent to whatever part is
  // actually at this address would be an unfriendly way to discover a wiring
  // mistake.
  std::uint8_t chip_id = 0;
  if (!read_register(Bmp390Register::kChipId, chip_id))
  {
    return Bmp390Error::kTransferFailed;
  }
  if (chip_id != kBmp390ChipId)
  {
    return Bmp390Error::kIdentityMismatch;
  }

  // Known state: soft reset drops any configuration a previous run (or
  // bootloader probe) left behind. The calibration NVM survives it.
  if (!bus_.write_register(reg_address(Bmp390Register::kCmd), kBmp390CmdSoftReset))
  {
    return Bmp390Error::kTransferFailed;
  }
  bus_.delay_ms(kBmp390SoftResetDelayMs);

  // The part re-answering its CHIP_ID is the sign the reset is over; a wrong
  // answer here means the settle time was not respected or the bus glitched.
  if (!read_register(Bmp390Register::kChipId, chip_id))
  {
    return Bmp390Error::kTransferFailed;
  }
  if (chip_id != kBmp390ChipId)
  {
    return Bmp390Error::kIdentityMismatch;
  }

  std::array<std::uint8_t, kBmp390CalibNvmLength> nvm{};
  if (!bus_.read_registers(reg_address(Bmp390Register::kCalibNvm), nvm.data(), nvm.size()))
  {
    return Bmp390Error::kTransferFailed;
  }
  calib_ = parse_bmp390_calibration(nvm);
  compensator_ = Bmp390Compensator::from_calibration(calib_);

  // Configuration before mode: the part starts converting the moment
  // PWR_CTRL selects normal mode, so everything it converts under goes first.
  const std::uint8_t osr = bmp390_osr(config.pressure_oversampling, config.temperature_oversampling);
  if (!bus_.write_register(reg_address(Bmp390Register::kOsr), osr))
  {
    return Bmp390Error::kTransferFailed;
  }
  if (!bus_.write_register(reg_address(Bmp390Register::kOdr), config.odr_sel))
  {
    return Bmp390Error::kTransferFailed;
  }
  if (!bus_.write_register(reg_address(Bmp390Register::kIntCtrl),
                           kBmp390IntCtrlDataReadyEnable | kBmp390IntCtrlActiveHigh))
  {
    return Bmp390Error::kTransferFailed;
  }
  const std::uint8_t pwr_ctrl =
      kBmp390PwrCtrlPressureEnable | kBmp390PwrCtrlTemperatureEnable | kBmp390PwrCtrlModeNormal;
  if (!bus_.write_register(reg_address(Bmp390Register::kPwrCtrl), pwr_ctrl))
  {
    return Bmp390Error::kTransferFailed;
  }

  return Bmp390Error::kNone;
}

Bmp390ReadResult Bmp390Driver::read_sample(BaroSample& out)
{
  std::uint8_t status = 0;
  if (!read_register(Bmp390Register::kStatus, status))
  {
    return Bmp390ReadResult::kTransferFailed;
  }
  const std::uint8_t ready = kBmp390StatusPressureReady | kBmp390StatusTemperatureReady;
  if ((status & ready) != ready)
  {
    return Bmp390ReadResult::kNoNewData;
  }

  // One burst from DATA_0 through SENSORTIME_2 (auto-increment clocks past
  // the two reserved bytes in between): the part shadows the whole block on
  // the first byte's read, so pressure, temperature and the sensor-time
  // stamp are from the same conversion -- reading them register-by-register
  // would not guarantee that.
  std::array<std::uint8_t, kBmp390DataSensorTimeBurstLength> data{};
  if (!bus_.read_registers(reg_address(Bmp390Register::kData0), data.data(), data.size()))
  {
    return Bmp390ReadResult::kTransferFailed;
  }

  const std::uint32_t uncomp_press = word24(data[0], data[1], data[2]);
  const std::uint32_t uncomp_temp = word24(data[3], data[4], data[5]);
  const std::uint32_t sensor_time_ticks = word24(data[8], data[9], data[10]);

  const double temperature_c = compensator_.compensate_temperature(uncomp_temp);
  const double pressure_pa = compensator_.compensate_pressure(uncomp_press, temperature_c);

  out.pressure_pa = static_cast<float>(pressure_pa);
  out.temperature_c = static_cast<float>(temperature_c);
  // The part's own clock, in the sample type's time unit. 24 bits at
  // 32768 Hz wrap every 512 s; disambiguating longer spans is the caller's
  // job, exactly as with the real counter.
  out.timestamp_us = (static_cast<std::uint64_t>(sensor_time_ticks) * 1000000ULL) / kBmp390SensorTimeTickHz;
  return Bmp390ReadResult::kSample;
}

bool Bmp390Driver::read_register(Bmp390Register reg, std::uint8_t& out)
{
  return bus_.read_registers(reg_address(reg), &out, 1);
}

}  // namespace hemerion::sensors::baro::bmp390
