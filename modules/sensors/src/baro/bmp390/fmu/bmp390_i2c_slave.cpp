// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file bmp390_i2c_slave.cpp
/// @brief Implements the simulated BMP390's I2C interface declared in
/// bmp390_i2c_slave.h.

#include "Hemerion/baro/bmp390/fmu/bmp390_i2c_slave.h"

namespace hemerion::sensors::baro::bmp390::fmu
{

namespace
{

[[nodiscard]] constexpr std::uint8_t reg_address(Bmp390Register reg) { return static_cast<std::uint8_t>(reg); }

/// Mask revision the simulated part reports.
constexpr std::uint8_t kRevId = 0x01;

}  // namespace

Bmp390I2cSlave::Bmp390I2cSlave(const Bmp390CalibData& calibration, std::uint8_t target_address)
  : nvm_(encode_bmp390_calibration(calibration)), target_address_(target_address)
{
}

bool Bmp390I2cSlave::start(std::uint8_t address, bool read)
{
  // A repeated START arrives with the transaction lock already held; the
  // opening START takes it. Either way it is released by stop().
  if (!transaction_lock_.owns_lock())
  {
    transaction_lock_ = std::unique_lock<std::mutex>(mutex_);
  }

  if (address != target_address_)
  {
    return false;
  }
  if (!read)
  {
    awaiting_pointer_ = true;
  }
  return true;
}

bool Bmp390I2cSlave::write(std::uint8_t byte)
{
  if (awaiting_pointer_)
  {
    pointer_ = byte;
    awaiting_pointer_ = false;
    return true;
  }
  write_register(pointer_, byte);
  // Datasheet 5.2.3: multi-byte writes are register/data pairs -- the next
  // write-phase byte is a new register address, not pointer+1's data.
  awaiting_pointer_ = true;
  return true;
}

std::uint8_t Bmp390I2cSlave::read(bool last)
{
  (void)last;  // the part stops driving SDA either way; nothing to model
  const std::uint8_t value = read_register(pointer_);
  ++pointer_;  // reads auto-increment across the whole map
  return value;
}

void Bmp390I2cSlave::stop()
{
  awaiting_pointer_ = false;
  if (transaction_lock_.owns_lock())
  {
    transaction_lock_.unlock();
  }
}

void Bmp390I2cSlave::latch_conversion(std::uint32_t uncomp_press, std::uint32_t uncomp_temp, std::uint64_t timestamp_us)
{
  const std::lock_guard<std::mutex> lock(mutex_);

  // 32768 ticks per second, 24-bit wrap -- the microsecond clock in the
  // part's own time base.
  sensor_time_ticks_ = static_cast<std::uint32_t>((timestamp_us * kBmp390SensorTimeTickHz) / 1000000ULL) & 0x00FFFFFFU;

  data_[0] = static_cast<std::uint8_t>(uncomp_press & 0xFFU);
  data_[1] = static_cast<std::uint8_t>((uncomp_press >> 8) & 0xFFU);
  data_[2] = static_cast<std::uint8_t>((uncomp_press >> 16) & 0xFFU);
  data_[3] = static_cast<std::uint8_t>(uncomp_temp & 0xFFU);
  data_[4] = static_cast<std::uint8_t>((uncomp_temp >> 8) & 0xFFU);
  data_[5] = static_cast<std::uint8_t>((uncomp_temp >> 16) & 0xFFU);

  if ((pwr_ctrl_ & kBmp390PwrCtrlPressureEnable) != 0U)
  {
    status_ |= kBmp390StatusPressureReady;
  }
  if ((pwr_ctrl_ & kBmp390PwrCtrlTemperatureEnable) != 0U)
  {
    status_ |= kBmp390StatusTemperatureReady;
  }
  drdy_interrupt_ = true;
}

std::uint64_t Bmp390I2cSlave::sampling_period_us() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  const bool normal = (pwr_ctrl_ & kBmp390PwrCtrlModeMask) == kBmp390PwrCtrlModeNormal;
  const bool measuring = (pwr_ctrl_ & (kBmp390PwrCtrlPressureEnable | kBmp390PwrCtrlTemperatureEnable)) != 0U;
  if (!normal || !measuring)
  {
    return 0;
  }
  return bmp390_odr_period_us(odr_);
}

bool Bmp390I2cSlave::take_forced_conversion()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!forced_pending_)
  {
    return false;
  }
  forced_pending_ = false;
  // The real part falls back to sleep once the forced conversion completes.
  pwr_ctrl_ = static_cast<std::uint8_t>(pwr_ctrl_ & ~kBmp390PwrCtrlModeMask);
  return true;
}

bool Bmp390I2cSlave::interrupt_asserted() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return interrupt_condition_locked();
}

void Bmp390I2cSlave::reset()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  apply_soft_reset();
}

std::uint8_t Bmp390I2cSlave::read_register(std::uint8_t address)
{
  // Reading DATA_0 latches the whole conversion -- data words *and* its
  // sensor time -- into the burst shadow and clears the data-ready
  // condition: one conversion, one coherent block.
  if (address == reg_address(Bmp390Register::kData0))
  {
    data_shadow_ = data_;
    sensor_time_shadow_ticks_ = sensor_time_ticks_;
    status_ &= static_cast<std::uint8_t>(~(kBmp390StatusPressureReady | kBmp390StatusTemperatureReady));
    drdy_interrupt_ = false;
  }
  if (address >= reg_address(Bmp390Register::kData0) && address <= reg_address(Bmp390Register::kData5))
  {
    return data_shadow_[address - reg_address(Bmp390Register::kData0)];
  }
  if (address >= reg_address(Bmp390Register::kSensorTime0) && address <= reg_address(Bmp390Register::kSensorTime2))
  {
    const unsigned shift = 8U * (address - reg_address(Bmp390Register::kSensorTime0));
    return static_cast<std::uint8_t>((sensor_time_shadow_ticks_ >> shift) & 0xFFU);
  }
  if (address >= reg_address(Bmp390Register::kCalibNvm) &&
      address < reg_address(Bmp390Register::kCalibNvm) + kBmp390CalibNvmLength)
  {
    return nvm_[address - reg_address(Bmp390Register::kCalibNvm)];
  }

  switch (address)
  {
    case reg_address(Bmp390Register::kChipId):
      return kBmp390ChipId;
    case reg_address(Bmp390Register::kRevId):
      return kRevId;
    case reg_address(Bmp390Register::kErrReg):
      return 0;
    case reg_address(Bmp390Register::kStatus):
      return status_;
    case reg_address(Bmp390Register::kIntStatus):
    {
      // Clears on read, and the pin follows.
      const std::uint8_t value = drdy_interrupt_ ? kBmp390IntStatusDataReady : 0U;
      drdy_interrupt_ = false;
      return value;
    }
    case reg_address(Bmp390Register::kIntCtrl):
      return int_ctrl_;
    case reg_address(Bmp390Register::kIfConf):
      return if_conf_;
    case reg_address(Bmp390Register::kPwrCtrl):
      return pwr_ctrl_;
    case reg_address(Bmp390Register::kOsr):
      return osr_;
    case reg_address(Bmp390Register::kOdr):
      return odr_;
    case reg_address(Bmp390Register::kConfig):
      return config_;
    default:
      return 0;
  }
}

void Bmp390I2cSlave::write_register(std::uint8_t address, std::uint8_t value)
{
  switch (address)
  {
    case reg_address(Bmp390Register::kIntCtrl):
      int_ctrl_ = value;
      return;
    case reg_address(Bmp390Register::kIfConf):
      if_conf_ = value;
      return;
    case reg_address(Bmp390Register::kPwrCtrl):
      pwr_ctrl_ = value;
      if ((pwr_ctrl_ & kBmp390PwrCtrlModeMask) == kBmp390PwrCtrlModeForced)
      {
        forced_pending_ = true;
      }
      return;
    case reg_address(Bmp390Register::kOsr):
      osr_ = value;
      return;
    case reg_address(Bmp390Register::kOdr):
      odr_ = value;
      return;
    case reg_address(Bmp390Register::kConfig):
      config_ = value;
      return;
    case reg_address(Bmp390Register::kCmd):
      if (value == kBmp390CmdSoftReset)
      {
        apply_soft_reset();
      }
      // kBmp390CmdFifoFlush: the FIFO is not modelled; flushing nothing is a
      // no-op, exactly what the real part does with an empty FIFO.
      return;
    default:
      // Read-only and reserved registers swallow writes, as on the part.
      return;
  }
}

void Bmp390I2cSlave::apply_soft_reset()
{
  data_.fill(0);
  data_shadow_.fill(0);
  sensor_time_ticks_ = 0;
  sensor_time_shadow_ticks_ = 0;
  status_ = kBmp390StatusCmdReady;
  drdy_interrupt_ = false;
  forced_pending_ = false;
  pwr_ctrl_ = 0;
  osr_ = 0;
  odr_ = 0;
  config_ = 0;
  int_ctrl_ = 0;
  if_conf_ = 0;
}

bool Bmp390I2cSlave::interrupt_condition_locked() const
{
  return drdy_interrupt_ && (int_ctrl_ & kBmp390IntCtrlDataReadyEnable) != 0U;
}

}  // namespace hemerion::sensors::baro::bmp390::fmu
