// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
#include "Hemerion/imu/imu_spi_driver.h"

#include <algorithm>
#include <array>

namespace hemerion::sensors::imu
{

namespace
{

// Every transfer is one command byte followed by data, so the buffers hold one
// more byte than the longest burst.
constexpr std::size_t kTransferBufferBytes = ImuSpiDriver::kMaxBurstBytes + 1;

}  // namespace

ImuSpiError ImuSpiDriver::probe()
{
  std::uint8_t identity = 0;
  if (!read_registers(ImuSpiRegister::kWhoAmI, &identity, 1))
  {
    return ImuSpiError::kTransferFailed;
  }
  if (identity != kImuSpiWhoAmI)
  {
    return ImuSpiError::kIdentityMismatch;
  }
  if (!write_register(ImuSpiRegister::kControl, kImuSpiControlStartup))
  {
    return ImuSpiError::kTransferFailed;
  }
  return ImuSpiError::kNone;
}

ImuSpiDriver::PollResult ImuSpiDriver::poll(ImuRawSample* out, std::size_t capacity)
{
  PollResult result;

  // STATUS, FIFO_COUNT_L and FIFO_COUNT_H are consecutive, so the whole
  // decision comes out of one four-byte transfer.
  std::array<std::uint8_t, 3> status{};
  if (!read_registers(ImuSpiRegister::kStatus, status.data(), status.size()))
  {
    result.error = ImuSpiError::kTransferFailed;
    return result;
  }
  result.fifo_overflow = (status[0] & kImuSpiStatusFifoOverflow) != 0U;

  const auto fifo_bytes =
      static_cast<std::size_t>(static_cast<std::uint16_t>(status[1]) | (static_cast<std::uint16_t>(status[2]) << 8));
  if (fifo_bytes == 0 || out == nullptr || capacity == 0)
  {
    return result;
  }

  // Cap the poll at what the caller's array can hold. A parser holding at most
  // one frame less a byte plus capacity * kFrameLength new bytes completes at
  // most `capacity` frames, so this bound is what keeps `out` from
  // overflowing without ever discarding a byte the part handed over.
  const std::size_t budget = std::min(fifo_bytes, capacity * kImuPacketRawSampleFrameLength);

  std::array<std::uint8_t, kTransferBufferBytes> burst{};
  std::size_t remaining = budget;
  while (remaining > 0)
  {
    const std::size_t chunk = std::min(remaining, kMaxBurstBytes);
    if (!burst_read_fifo(burst.data(), chunk))
    {
      result.error = ImuSpiError::kTransferFailed;
      return result;
    }
    remaining -= chunk;
    result.bytes_read += chunk;

    for (std::size_t i = 0; i < chunk; ++i)
    {
      ImuRawSample sample;
      const ImuPacketError parsed = parser_.parse_byte(burst[i], sample);
      if (parsed == ImuPacketError::kChecksumMismatch)
      {
        ++result.checksum_errors;
      }
      else if (parsed == ImuPacketError::kNone)
      {
        // Defensive: the budget above makes this unreachable, but a decoded
        // sample must never be written past the caller's array if the frame
        // length assumption ever changes.
        if (result.samples < capacity)
        {
          out[result.samples] = sample;
          ++result.samples;
        }
        else
        {
          ++result.dropped_samples;
        }
      }
    }
  }
  return result;
}

bool ImuSpiDriver::read_registers(ImuSpiRegister first, std::uint8_t* out, std::size_t count)
{
  if (count == 0 || count > kMaxBurstBytes)
  {
    return false;
  }

  std::array<std::uint8_t, kTransferBufferBytes> tx{};
  std::array<std::uint8_t, kTransferBufferBytes> rx{};
  tx[0] = imu_spi_command(first, /*read=*/true);
  // tx[1..count] stay at kImuSpiDummyByte: the controller has to clock the
  // part's answer out of it somehow.
  if (!bus_.transfer(tx.data(), rx.data(), count + 1))
  {
    return false;
  }
  // rx[0] was shifted out while the command byte was still shifting in, so it
  // carries nothing.
  std::copy_n(rx.begin() + 1, count, out);
  return true;
}

bool ImuSpiDriver::write_register(ImuSpiRegister reg, std::uint8_t value)
{
  std::array<std::uint8_t, 2> tx{ imu_spi_command(reg, /*read=*/false), value };
  std::array<std::uint8_t, 2> rx{};
  return bus_.transfer(tx.data(), rx.data(), tx.size());
}

bool ImuSpiDriver::burst_read_fifo(std::uint8_t* out, std::size_t count)
{
  return read_registers(ImuSpiRegister::kFifoData, out, count);
}

}  // namespace hemerion::sensors::imu
