// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file radalt_packet.h
/// @brief Hemerion radar altimeter wire protocol: framing constants and
/// streaming parser.
///
/// A minimal binary framing for shipping RadAltRawSample register counts off
/// a radar altimeter (or the radalt/fmu/ hardware simulator standing in for
/// one) over a byte stream. The framing follows the same shape as the
/// Hemerion IMU protocol (sync bytes, message id, little-endian length,
/// payload, two-byte Fletcher checksum -- see imu_packet.h), but the sync
/// bytes are distinct, so an interleaved multi-sensor byte stream can never
/// desync one parser into another's frames.
///
/// Frame layout (little-endian multi-byte fields):
///
///     offset  size  field
///          0     1  sync1 (0xC9)
///          1     1  sync2 (0x9C)
///          2     1  message id (0x01 = raw radar altimeter sample)
///          3     2  payload length (16 for message 0x01)
///          5    16  payload (see RadAltPacketParser::decode_raw_sample)
///         21     2  checksum ck_a, ck_b (UBX-style Fletcher over
///                   id + length + payload bytes)
///
/// The raw-sample payload carries the RadAltRawSample range count, status
/// word and uint64 sample timestamp, all little-endian:
///
///     offset  size  field
///          0     4  range [LSB]
///          4     4  status (kRadAltStatusNoReturn / kRadAltStatusTrackValid)
///          8     8  timestamp_us
///
/// Scale (LSB per meter) is NOT part of the wire format, exactly as on real
/// silicon: the driver knows the part it configured and applies
/// convert_raw_to_si() with the matching RadAltScale.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "Hemerion/radalt/radalt_types.h"

namespace hemerion::sensors::radalt
{

/// First sync byte opening every Hemerion radar altimeter frame.
inline constexpr std::uint8_t kRadAltPacketSync1 = 0xC9;
/// Second sync byte.
inline constexpr std::uint8_t kRadAltPacketSync2 = 0x9C;
/// Message id of the raw-sample message (the only one defined so far).
inline constexpr std::uint8_t kRadAltPacketRawSampleId = 0x01;
/// Raw-sample payload length [bytes]: int32 range + uint32 status + uint64 timestamp.
inline constexpr std::size_t kRadAltPacketRawSamplePayloadLength = 16;
/// Full raw-sample frame length: 2 sync + id + 2 length + payload + 2 checksum.
inline constexpr std::size_t kRadAltPacketRawSampleFrameLength = 5 + kRadAltPacketRawSamplePayloadLength + 2;

/// Result of RadAltPacketParser::parse_byte().
enum class RadAltPacketError : std::uint8_t
{
  kNone,                ///< A raw-sample message completed and verified; `out` was overwritten.
  kIncomplete,          ///< Mid-frame; keep feeding bytes.
  kChecksumMismatch,    ///< A completed frame failed checksum verification.
  kUnsupportedMessage,  ///< A frame completed and verified but carries an unknown message id.
};

/// @brief Byte-at-a-time parser for the Hemerion radar altimeter wire
/// protocol; see @ref radalt_packet.h for the frame layout.
///
/// Unknown message ids are checksummed and skipped rather than buffered, so
/// a stream carrying future message types doesn't desync this parser --
/// the same policy ImuPacketParser and UbxParser apply.
class RadAltPacketParser
{
public:
  /// @brief Feeds one byte from the sensor stream.
  ///
  /// @param byte Next raw byte of the radar altimeter stream.
  /// @param out  Overwritten once a raw-sample message completes; the sample
  ///             timestamp comes from the frame payload (the sender's clock),
  ///             not a local clock.
  /// @return kNone once a raw-sample message has been fully decoded and its
  ///         checksum verified (with `out` overwritten); kIncomplete while a
  ///         frame is still being assembled; kChecksumMismatch or
  ///         kUnsupportedMessage if a completed frame is rejected.
  [[nodiscard]] RadAltPacketError parse_byte(std::uint8_t byte, RadAltRawSample& out);

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
  void decode_raw_sample(RadAltRawSample& out) const;
  [[nodiscard]] std::uint32_t read_u32(std::size_t offset) const;
  [[nodiscard]] std::int32_t read_i32(std::size_t offset) const;
  [[nodiscard]] std::uint64_t read_u64(std::size_t offset) const;

  State state_ = State::kSync1;
  std::uint8_t msg_id_ = 0;
  std::uint16_t length_ = 0;
  std::uint16_t payload_index_ = 0;
  std::array<std::uint8_t, kRadAltPacketRawSamplePayloadLength> payload_{};
  std::uint8_t ck_a_ = 0;
  std::uint8_t ck_b_ = 0;
};

}  // namespace hemerion::sensors::radalt
