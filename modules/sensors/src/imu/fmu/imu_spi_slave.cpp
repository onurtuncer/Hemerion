// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
#include "Hemerion/imu/fmu/imu_spi_slave.h"

namespace hemerion::sensors::imu::fmu
{

bool ImuSpiSlave::push_frame(const std::uint8_t* bytes, std::size_t length)
{
  const std::lock_guard<std::mutex> guard(mutex_);

  if ((control_ & kImuSpiControlFifoEnable) == 0U || bytes == nullptr || length == 0)
  {
    return false;
  }
  if (length > fifo_.size() - fifo_used_)
  {
    overflow_sticky_ = true;
    ++frames_dropped_;
    return false;
  }

  std::size_t tail = (fifo_head_ + fifo_used_) % fifo_.size();
  for (std::size_t i = 0; i < length; ++i)
  {
    fifo_[tail] = bytes[i];
    tail = (tail + 1) % fifo_.size();
  }
  fifo_used_ += length;
  return true;
}

void ImuSpiSlave::chip_select(bool asserted)
{
  if (asserted)
  {
    transaction_lock_ = std::unique_lock<std::mutex>(mutex_);
    byte_index_ = 0;
    address_ = 0;
    reading_ = false;
    return;
  }
  if (transaction_lock_.owns_lock())
  {
    transaction_lock_.unlock();
  }
}

std::uint8_t ImuSpiSlave::shift(std::uint8_t mosi)
{
  if (!transaction_lock_.owns_lock())
  {
    // Clocking a deselected part reaches nothing on a real board either.
    return kImuSpiDummyByte;
  }

  if (byte_index_ == 0)
  {
    reading_ = (mosi & kImuSpiReadBit) != 0U;
    address_ = static_cast<std::uint8_t>(mosi & kImuSpiAddressMask);
    ++byte_index_;
    // The command byte is still shifting in while this one shifts out, so
    // there is nothing yet to answer with.
    return kImuSpiDummyByte;
  }

  ++byte_index_;
  std::uint8_t miso = kImuSpiDummyByte;
  if (reading_)
  {
    miso = read_register(address_);
  }
  else
  {
    write_register(address_, mosi);
  }

  // The FIFO port is a window onto one buffer, not a register: successive
  // bytes of a burst read pop successive FIFO bytes rather than walking up
  // the map. Everything else auto-increments.
  if (address_ != static_cast<std::uint8_t>(ImuSpiRegister::kFifoData))
  {
    address_ = static_cast<std::uint8_t>((address_ + 1U) & kImuSpiAddressMask);
  }
  return miso;
}

void ImuSpiSlave::reset()
{
  const std::lock_guard<std::mutex> guard(mutex_);
  clear_fifo();
  control_ = kImuSpiControlFifoEnable;
  overflow_sticky_ = false;
  frames_dropped_ = 0;
  byte_index_ = 0;
  address_ = 0;
  reading_ = false;
}

std::size_t ImuSpiSlave::fifo_used() const
{
  const std::lock_guard<std::mutex> guard(mutex_);
  return fifo_used_;
}

bool ImuSpiSlave::data_ready() const
{
  const std::lock_guard<std::mutex> guard(mutex_);
  return fifo_used_ > 0;
}

std::uint64_t ImuSpiSlave::frames_dropped() const
{
  const std::lock_guard<std::mutex> guard(mutex_);
  return frames_dropped_;
}

std::uint8_t ImuSpiSlave::read_register(std::uint8_t address)
{
  switch (static_cast<ImuSpiRegister>(address))
  {
    case ImuSpiRegister::kWhoAmI:
      return kImuSpiWhoAmI;

    case ImuSpiRegister::kStatus:
    {
      std::uint8_t status = 0;
      if (fifo_used_ > 0)
      {
        status |= kImuSpiStatusDataReady;
      }
      if (overflow_sticky_)
      {
        status |= kImuSpiStatusFifoOverflow;
      }
      overflow_sticky_ = false;  // sticky until read, like the real part
      return status;
    }

    case ImuSpiRegister::kFifoCountLow:
      return static_cast<std::uint8_t>(fifo_used_ & 0xFFU);

    case ImuSpiRegister::kFifoCountHigh:
      return static_cast<std::uint8_t>((fifo_used_ >> 8) & 0xFFU);

    case ImuSpiRegister::kControl:
      return control_;

    case ImuSpiRegister::kFifoData:
      return pop_fifo();

    default:
      // Unmapped addresses read as zero rather than aliasing something real.
      return 0;
  }
}

void ImuSpiSlave::write_register(std::uint8_t address, std::uint8_t value)
{
  if (static_cast<ImuSpiRegister>(address) != ImuSpiRegister::kControl)
  {
    // Every other register is read-only on this part; writes are ignored.
    return;
  }
  if ((value & kImuSpiControlFifoReset) != 0U)
  {
    clear_fifo();
    overflow_sticky_ = false;
  }
  // The reset bit is self-clearing, so it never survives into the readback.
  control_ = static_cast<std::uint8_t>(value & ~kImuSpiControlFifoReset);
}

std::uint8_t ImuSpiSlave::pop_fifo()
{
  if (fifo_used_ == 0)
  {
    // Over-reading an empty FIFO yields the part's idle byte; the driver is
    // expected to have read FIFO_COUNT first.
    return kImuSpiDummyByte;
  }
  const std::uint8_t byte = fifo_[fifo_head_];
  fifo_head_ = (fifo_head_ + 1) % fifo_.size();
  --fifo_used_;
  return byte;
}

void ImuSpiSlave::clear_fifo()
{
  fifo_head_ = 0;
  fifo_used_ = 0;
}

}  // namespace hemerion::sensors::imu::fmu
