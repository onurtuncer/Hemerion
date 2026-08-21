// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file mmc5983ma_i2c_slave.cpp
/// @brief Implements the simulated MMC5983MA's I2C interface declared in
/// mmc5983ma_i2c_slave.h.

#include "Hemerion/mag/mmc5983ma/fmu/mmc5983ma_i2c_slave.h"

namespace hemerion::sensors::mag::mmc5983ma::fmu
{

namespace
{

[[nodiscard]] constexpr std::uint8_t reg_address(Mmc5983maRegister reg) { return static_cast<std::uint8_t>(reg); }

}  // namespace

Mmc5983maI2cSlave::Mmc5983maI2cSlave(std::uint8_t target_address) : target_address_(target_address)
{
  apply_soft_reset();
}

bool Mmc5983maI2cSlave::start(std::uint8_t address, bool read)
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

bool Mmc5983maI2cSlave::write(std::uint8_t byte)
{
  if (awaiting_pointer_)
  {
    pointer_ = byte;
    awaiting_pointer_ = false;
    return true;
  }
  // Datasheet "DATA TRANSFER": multi-byte writes walk consecutive registers
  // from the pointer -- unlike the BMP390, which expects a fresh register
  // address before every data byte.
  write_register(pointer_, byte);
  ++pointer_;
  return true;
}

std::uint8_t Mmc5983maI2cSlave::read(bool last)
{
  (void)last;  // the part stops driving SDA either way; nothing to model
  const std::uint8_t value = read_register(pointer_);
  ++pointer_;  // reads auto-increment across the whole map
  return value;
}

void Mmc5983maI2cSlave::stop()
{
  awaiting_pointer_ = false;
  if (transaction_lock_.owns_lock())
  {
    transaction_lock_.unlock();
  }
}

void Mmc5983maI2cSlave::latch_measurement(const Mmc5983maFieldCounts& counts)
{
  const std::lock_guard<std::mutex> lock(mutex_);

  data_ = encode_mmc5983ma_field(counts);
  status_ |= kMmc5983maStatusMeasurementDone;

  // Periodic set: the part re-magnetizes itself every Prd_set measurements,
  // which is how it recovers from a disturbing field without the controller
  // noticing. Only counted while the part will actually honour the feature.
  if (periodic_set_active_locked())
  {
    ++measurements_since_set_;
    const std::uint32_t interval = mmc5983ma_periodic_set_interval(static_cast<Mmc5983maPeriodicSet>(
        (control2_ & kMmc5983maControl2PeriodicSetMask) >> kMmc5983maControl2PeriodicSetShift));
    if (measurements_since_set_ >= interval)
    {
      magnetization_ = +1;
      measurements_since_set_ = 0;
    }
  }
}

void Mmc5983maI2cSlave::latch_temperature(std::uint8_t tout)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  tout_ = tout;
  status_ |= kMmc5983maStatusTemperatureDone;
}

std::uint64_t Mmc5983maI2cSlave::sampling_period_us() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if ((control2_ & kMmc5983maControl2ContinuousEnable) == 0U)
  {
    return 0;
  }
  return mmc5983ma_continuous_period_us(
      static_cast<Mmc5983maContinuousRate>(control2_ & kMmc5983maControl2ContinuousRateMask));
}

bool Mmc5983maI2cSlave::take_triggered_measurement()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!measurement_pending_)
  {
    return false;
  }
  measurement_pending_ = false;
  return true;
}

bool Mmc5983maI2cSlave::take_triggered_temperature()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!temperature_pending_)
  {
    return false;
  }
  temperature_pending_ = false;
  return true;
}

Mmc5983maSensingState Mmc5983maI2cSlave::sensing_state() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return Mmc5983maSensingState{ magnetization_, automatic_set_reset_ };
}

bool Mmc5983maI2cSlave::interrupt_asserted() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return interrupt_condition_locked();
}

void Mmc5983maI2cSlave::reset()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  apply_soft_reset();
}

std::uint8_t Mmc5983maI2cSlave::read_register(std::uint8_t address)
{
  if (address < kMmc5983maDataBurstLength)
  {
    return data_[address];
  }

  // Internal control 0..3 are Mode: W on the datasheet, so they read back as
  // zero -- like every reserved address, which is why this is a guard rather
  // than a case group returning the same thing as `default`. Serving the
  // stored value instead would be the friendly choice and would hide a real
  // firmware bug: a read-modify-write of a control register reads zero on the
  // part, so it clobbers every bit it meant to preserve.
  if (address >= reg_address(Mmc5983maRegister::kInternalControl0) &&
      address <= reg_address(Mmc5983maRegister::kInternalControl3))
  {
    return 0;
  }

  switch (address)
  {
    case reg_address(Mmc5983maRegister::kTout):
      return tout_;
    case reg_address(Mmc5983maRegister::kStatus):
      return status_;
    case reg_address(Mmc5983maRegister::kProductId):
      return kMmc5983maProductId;
    default:
      return 0;
  }
}

void Mmc5983maI2cSlave::write_register(std::uint8_t address, std::uint8_t value)
{
  switch (address)
  {
    case reg_address(Mmc5983maRegister::kStatus):
      // Write-1-to-clear, and the INT line follows the flags down.
      status_ &= static_cast<std::uint8_t>(~value);
      return;
    case reg_address(Mmc5983maRegister::kInternalControl0):
      write_control0(value);
      return;
    case reg_address(Mmc5983maRegister::kInternalControl1):
      if ((value & kMmc5983maControl1SoftReset) != 0U)
      {
        apply_soft_reset();
        return;
      }
      control1_ = value;
      return;
    case reg_address(Mmc5983maRegister::kInternalControl2):
      control2_ = value;
      // Re-arming the periodic-set schedule on a configuration change; the
      // counter is meaningless across a different interval.
      measurements_since_set_ = 0;
      return;
    case reg_address(Mmc5983maRegister::kInternalControl3):
      // Stored so writes do not fault, and so a driver can set 3-wire SPI
      // without the model objecting. St_enp/St_enm are NOT modelled
      // magnetically: the self-test coil's extra field does not appear in
      // the measurements, so a driver self-test would pass vacuously here.
      // Build it against hardware, not against this model.
      control3_ = value;
      return;
    default:
      // Read-only and reserved registers swallow writes, as on the part.
      return;
  }
}

void Mmc5983maI2cSlave::write_control0(std::uint8_t value)
{
  // The two persistent bits.
  interrupt_enable_ = (value & kMmc5983maControl0InterruptEnable) != 0U;
  automatic_set_reset_ = (value & kMmc5983maControl0AutoSetResetEnable) != 0U;

  // The coil pulses. Both in one write would be a contradictory command; the
  // datasheet does not say which wins, and neither does silicon documentation
  // anywhere else, so SET is applied last and wins -- deliberately the
  // benign outcome.
  if ((value & kMmc5983maControl0Reset) != 0U)
  {
    magnetization_ = -1;
    measurements_since_set_ = 0;
  }
  if ((value & kMmc5983maControl0Set) != 0U)
  {
    magnetization_ = +1;
    measurements_since_set_ = 0;
  }

  // The measurement triggers. "This bit and TM_M cannot be high at the same
  // time" -- the field measurement wins here, and the temperature request is
  // dropped rather than queued.
  if ((value & kMmc5983maControl0TakeMeasurement) != 0U)
  {
    measurement_pending_ = true;
    // "When the new measurement command is occurred, this bit turns to 0."
    status_ &= static_cast<std::uint8_t>(~kMmc5983maStatusMeasurementDone);
  }
  else if ((value & kMmc5983maControl0TakeTemperature) != 0U)
  {
    temperature_pending_ = true;
    status_ &= static_cast<std::uint8_t>(~kMmc5983maStatusTemperatureDone);
  }

  // OTP Read re-reads the shadow registers and reports done. There is no OTP
  // content in this model -- the part's trim is baked into the sensitivity
  // constant, not served from registers -- so only the flag is observable.
  if ((value & kMmc5983maControl0OtpRead) != 0U)
  {
    status_ |= kMmc5983maStatusOtpReadDone;
  }
}

void Mmc5983maI2cSlave::apply_soft_reset()
{
  data_.fill(0);
  tout_ = 0;
  // The reset value tabulated for Status is all-zero, but that is its value
  // at the instant of reset: the part then re-reads OTP as part of the same
  // 10 ms start-up and reports success. This model settles instantly, so the
  // steady state is the one a driver actually observes.
  status_ = kMmc5983maStatusOtpReadDone;
  control1_ = 0;
  control2_ = 0;
  control3_ = 0;
  interrupt_enable_ = false;
  automatic_set_reset_ = false;
  magnetization_ = kPowerOnMagnetization;
  measurements_since_set_ = 0;
  measurement_pending_ = false;
  temperature_pending_ = false;
}

bool Mmc5983maI2cSlave::periodic_set_active_locked() const
{
  // Datasheet: "This feature needs to work with both Auto_SR_en and Cmm_en
  // bits set to 1."
  return (control2_ & kMmc5983maControl2PeriodicSetEnable) != 0U &&
         (control2_ & kMmc5983maControl2ContinuousEnable) != 0U && automatic_set_reset_;
}

bool Mmc5983maI2cSlave::interrupt_condition_locked() const
{
  const std::uint8_t done = kMmc5983maStatusMeasurementDone | kMmc5983maStatusTemperatureDone;
  return interrupt_enable_ && (status_ & done) != 0U;
}

}  // namespace hemerion::sensors::mag::mmc5983ma::fmu
