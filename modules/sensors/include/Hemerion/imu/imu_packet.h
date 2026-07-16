// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file imu_packet.h
/// @brief Hemerion IMU wire protocol: framing constants and streaming parser.
///
/// A minimal binary framing for shipping ImuRawSample register counts off an
/// IMU (or the imu/fmu/ hardware simulator standing in for one) over a byte
/// stream: UART, SPI DMA drain, or the simulator's UDP side channel. The
/// framing deliberately follows the same shape as UBX (sync bytes, message
/// id, little-endian length, payload, two-byte Fletcher checksum) so the
/// parser structure mirrors UbxParser, but the sync bytes are distinct --
/// an interleaved GPS/IMU byte stream can never desync one parser into the
/// other's frames.
///
/// Frame layout (little-endian multi-byte fields):
///
///     offset  size  field
///          0     1  sync1 (0xA5)
///          1     1  sync2 (0x5A)
///          2     1  message id (0x01 = raw IMU sample)
///          3     2  payload length (32 for message 0x01)
///          5    32  payload (see ImuPacketParser::decode_raw_sample)
///         37     2  checksum ck_a, ck_b (UBX-style Fletcher over
///                   id + length + payload bytes)
///
/// The raw-sample payload carries the six ImuRawSample counts as int32 plus
/// the uint64 sample timestamp, all little-endian:
///
///     offset  size  field
///          0     4  accel_x [LSB]        12     4  gyro_x [LSB]
///          4     4  accel_y [LSB]        16     4  gyro_y [LSB]
///          8     4  accel_z [LSB]        20     4  gyro_z [LSB]
///         24     8  timestamp_us
///
/// Scale (LSB per g / LSB per deg/s) is NOT part of the wire format, exactly
/// as on real silicon: the driver knows the full-scale range it configured
/// and applies convert_raw_to_si() with the matching ImuScale.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "Hemerion/imu/imu_types.h"

namespace hemerion::sensors::imu
{

/// First sync byte opening every Hemerion IMU frame.
inline constexpr std::uint8_t kImuPacketSync1 = 0xA5;
/// Second sync byte.
inline constexpr std::uint8_t kImuPacketSync2 = 0x5A;
/// Message id of the raw-sample message (the only one defined so far).
inline constexpr std::uint8_t kImuPacketRawSampleId = 0x01;
/// Raw-sample payload length [bytes]: 6 x int32 counts + uint64 timestamp.
inline constexpr std::size_t kImuPacketRawSamplePayloadLength = 32;
/// Full raw-sample frame length: 2 sync + id + 2 length + payload + 2 checksum.
inline constexpr std::size_t kImuPacketRawSampleFrameLength = 5 + kImuPacketRawSamplePayloadLength + 2;

/// Result of ImuPacketParser::parse_byte().
enum class ImuPacketError : std::uint8_t
{
  kNone,                ///< A raw-sample message completed and verified; `out` was overwritten.
  kIncomplete,          ///< Mid-frame; keep feeding bytes.
  kChecksumMismatch,    ///< A completed frame failed checksum verification.
  kUnsupportedMessage,  ///< A frame completed and verified but carries an unknown message id.
};

/// @brief Byte-at-a-time parser for the Hemerion IMU wire protocol; see
/// @ref imu_packet.h for the frame layout.
///
/// Unknown message ids are checksummed and skipped rather than buffered, so
/// a stream carrying future message types doesn't desync this parser --
/// the same policy UbxParser applies to non-NAV-PVT traffic.
class ImuPacketParser
{
public:
  /// @brief Feeds one byte from the sensor stream.
  ///
  /// @param byte Next raw byte of the IMU stream.
  /// @param out  Overwritten once a raw-sample message completes; the sample
  ///             timestamp comes from the frame payload (the sender's clock),
  ///             not a local clock.
  /// @return kNone once a raw-sample message has been fully decoded and its
  ///         checksum verified (with `out` overwritten); kIncomplete while a
  ///         frame is still being assembled; kChecksumMismatch or
  ///         kUnsupportedMessage if a completed frame is rejected.
  [[nodiscard]] ImuPacketError parse_byte(std::uint8_t byte, ImuRawSample& out);

private:
  enum class State : std::uint8_t
  {
    kSync1,
    kSync2,
    kId,
    kLength1,
    kLength2,
    kPayload,
    kChecksumA,
    kChecksumB,
  };

  // Unknown messages are skipped, not stored, but a corrupt length byte must
  // not park the parser mid-"payload" for thousands of bytes before resync.
  static constexpr std::uint16_t kMaxSkipLength = 1024;

  void update_checksum(std::uint8_t byte);
  void reset();
  void decode_raw_sample(ImuRawSample& out) const;
  [[nodiscard]] std::uint32_t read_u32(std::size_t offset) const;
  [[nodiscard]] std::int32_t read_i32(std::size_t offset) const;
  [[nodiscard]] std::uint64_t read_u64(std::size_t offset) const;

  State state_ = State::kSync1;
  std::uint8_t msg_id_ = 0;
  std::uint16_t length_ = 0;
  std::uint16_t payload_index_ = 0;
  std::array<std::uint8_t, kImuPacketRawSamplePayloadLength> payload_{};
  std::uint8_t ck_a_ = 0;
  std::uint8_t ck_b_ = 0;
};

}  // namespace hemerion::sensors::imu
