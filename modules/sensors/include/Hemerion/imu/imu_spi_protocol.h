// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file imu_spi_protocol.h
/// @brief Register map and SPI command encoding of the IMU part Hemerion
/// drives.
///
/// The single source of truth for both ends of the bus: the on-target driver
/// (@ref imu_spi_driver.h) and the hardware simulator's device model
/// (`imu/fmu/imu_spi_slave.h`) include this header, so a change to the wire
/// protocol cannot land on only one side.
///
/// The layout follows the shape most MEMS inertial parts (ADIS16470,
/// ICM-42688, BMI088) share, reduced to what a flight computer actually needs:
///
/// **Command byte.** Every chip-select-framed transfer opens with one command
/// byte -- bit 7 selects read (1) or write (0), bits 6..0 carry the register
/// address. The part is still shifting that byte in while it shifts a byte
/// out, so the first MISO byte of any transfer is a don't-care
/// (@ref kImuSpiDummyByte). Data bytes follow from the second byte on.
///
/// **Auto-increment.** Successive bytes of a multi-byte read or write walk up
/// the register map, so `STATUS`, `FIFO_COUNT_L` and `FIFO_COUNT_H` come out
/// of one four-byte transfer. `FIFO_DATA` is the exception: it does not
/// auto-increment, so a burst read of N bytes from it pops N bytes off the
/// sample FIFO -- the usual "read the FIFO port until it is empty" idiom.
///
/// **What comes out of the FIFO.** Hemerion IMU raw-sample frames, byte for
/// byte as @ref imu_packet.h documents them: the FIFO is the part's sample
/// buffer, and the driver's job is to move its bytes into ImuPacketParser
/// without interpreting them. Scale is configuration, not wire data -- the
/// counts in those frames mean whatever the full-scale range the driver
/// programmed says they mean.

#pragma once

#include <cstddef>
#include <cstdint>

namespace hemerion::sensors::imu
{

/// Set in a command byte to read the addressed register; clear to write it.
inline constexpr std::uint8_t kImuSpiReadBit = 0x80;

/// Register-address bits of a command byte.
inline constexpr std::uint8_t kImuSpiAddressMask = 0x7F;

/// Byte the part shifts out while the command byte is still shifting in, and
/// the byte a controller clocks out to read.
inline constexpr std::uint8_t kImuSpiDummyByte = 0x00;

/// Register addresses of the simulated part.
enum class ImuSpiRegister : std::uint8_t
{
  kWhoAmI = 0x00,         ///< Read-only device identity, always @ref kImuSpiWhoAmI.
  kStatus = 0x01,         ///< Read-only status bits; the overflow flag clears on read.
  kFifoCountLow = 0x02,   ///< Read-only FIFO fill level [bytes], low byte.
  kFifoCountHigh = 0x03,  ///< Read-only FIFO fill level [bytes], high byte.
  kControl = 0x04,        ///< Read/write control bits.
  kFifoData = 0x7F,       ///< Read-only FIFO port; does not auto-increment.
};

/// Value the `WHO_AM_I` register always reads back, so a driver can prove it
/// is talking to the part it thinks it is (and that the bus is wired at all).
inline constexpr std::uint8_t kImuSpiWhoAmI = 0x6A;

/// `STATUS` bit: at least one complete sample frame is buffered. The part
/// drives its data-ready line from the same condition.
inline constexpr std::uint8_t kImuSpiStatusDataReady = 0x01;

/// `STATUS` bit: at least one sample was dropped because the FIFO was full
/// since this register was last read. Sticky, cleared by reading `STATUS`.
inline constexpr std::uint8_t kImuSpiStatusFifoOverflow = 0x02;

/// `CONTROL` bit: buffer samples into the FIFO. Cleared parts discard them.
inline constexpr std::uint8_t kImuSpiControlFifoEnable = 0x01;

/// `CONTROL` bit: discard everything buffered. Self-clearing, so it reads
/// back as 0.
inline constexpr std::uint8_t kImuSpiControlFifoReset = 0x02;

/// `CONTROL` value a driver writes to bring the part up: buffering on, FIFO
/// emptied of whatever accumulated before it took over.
inline constexpr std::uint8_t kImuSpiControlStartup = kImuSpiControlFifoEnable | kImuSpiControlFifoReset;

/// Capacity of the part's sample FIFO [bytes]. Sized so a controller polling
/// at a leisurely rate cannot overrun it: 16 KiB holds ~420 raw-sample frames,
/// i.e. four seconds of a 100 Hz part.
inline constexpr std::size_t kImuSpiFifoCapacityBytes = 16384;

/// @brief Builds the command byte opening a transfer.
/// @param reg  Register to address.
/// @param read True for a read transfer, false for a write.
[[nodiscard]] constexpr std::uint8_t imu_spi_command(ImuSpiRegister reg, bool read)
{
  const auto address = static_cast<std::uint8_t>(static_cast<std::uint8_t>(reg) & kImuSpiAddressMask);
  return read ? static_cast<std::uint8_t>(address | kImuSpiReadBit) : address;
}

}  // namespace hemerion::sensors::imu
