// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// Hemerion/gps/ubxParser.hpp
//
// Streaming parser for the u-blox UBX binary protocol. Only UBX-NAV-PVT
// (class 0x01, id 0x07) is decoded -- it alone carries a complete fix
// (position, altitude, speed, course, accuracy, satellite count, fix type),
// unlike NMEA's split across GGA/RMC. Other messages are checksummed and
// skipped rather than buffered, so a receiver streaming additional UBX
// classes doesn't desync this parser.
// ------------------------------------------------------------------------------
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "Hemerion/gps/gpsTypes.hpp"

namespace hemerion::sensors::gps {

class UbxParser {
 public:
  // Feeds one byte from the receiver. Returns kNone and overwrites `out`
  // once a UBX-NAV-PVT message has been fully decoded and its checksum
  // verified; kIncomplete while a message is still being assembled;
  // kChecksumMismatch or kUnsupportedMessage if a completed message is
  // rejected. `timestamp_us` is the caller's local clock value when this
  // byte was received; it is copied into `out.timestamp_us` on success.
  [[nodiscard]] GpsParseError parse_byte(std::uint8_t byte, std::uint64_t timestamp_us, GpsFix& out);

 private:
  enum class State : std::uint8_t {
    kSync1,
    kSync2,
    kClass,
    kId,
    kLength1,
    kLength2,
    kPayload,
    kChecksumA,
    kChecksumB,
  };

  static constexpr std::uint8_t kSync1Byte = 0xB5;
  static constexpr std::uint8_t kSync2Byte = 0x62;
  static constexpr std::uint8_t kNavClass = 0x01;
  static constexpr std::uint8_t kNavPvtId = 0x07;
  // UBX-NAV-PVT's payload is 92 bytes on firmware that reports
  // headVeh/magDec/magAcc, 84 bytes on older firmware that omits them.
  static constexpr std::size_t kMaxPayloadLength = 92;
  static constexpr std::size_t kNavPvtMinPayloadLength = 84;

  void update_checksum(std::uint8_t byte);
  void reset();
  [[nodiscard]] GpsParseError decode_nav_pvt(std::uint16_t payload_length, std::uint64_t timestamp_us,
                                              GpsFix& out) const;
  [[nodiscard]] std::uint32_t read_u32(std::size_t offset) const;
  [[nodiscard]] std::int32_t read_i32(std::size_t offset) const;
  static GpsFixType to_fix_type(std::uint8_t ubx_fix_type);

  State state_ = State::kSync1;
  std::uint8_t msg_class_ = 0;
  std::uint8_t msg_id_ = 0;
  std::uint16_t length_ = 0;
  std::uint16_t payload_index_ = 0;
  std::array<std::uint8_t, kMaxPayloadLength> payload_{};
  std::uint8_t ck_a_ = 0;
  std::uint8_t ck_b_ = 0;
};

}  // namespace hemerion::sensors::gps
