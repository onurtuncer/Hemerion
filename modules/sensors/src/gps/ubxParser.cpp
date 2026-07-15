// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file ubxParser.cpp
/// @brief Implements the streaming UBX frame parser declared in
/// ubxParser.hpp.

#include "Hemerion/gps/ubxParser.hpp"

namespace hemerion::sensors::gps {

void UbxParser::update_checksum(std::uint8_t byte) {
  ck_a_ = static_cast<std::uint8_t>(ck_a_ + byte);
  ck_b_ = static_cast<std::uint8_t>(ck_b_ + ck_a_);
}

void UbxParser::reset() {
  state_ = State::kSync1;
  length_ = 0;
  payload_index_ = 0;
}

GpsParseError UbxParser::parse_byte(std::uint8_t byte, std::uint64_t timestamp_us, GpsFix& out) {
  switch (state_) {
    case State::kSync1:
      state_ = (byte == kSync1Byte) ? State::kSync2 : State::kSync1;
      return GpsParseError::kIncomplete;

    case State::kSync2:
      if (byte == kSync2Byte) {
        ck_a_ = 0;
        ck_b_ = 0;
        state_ = State::kClass;
      } else {
        state_ = State::kSync1;
      }
      return GpsParseError::kIncomplete;

    case State::kClass:
      msg_class_ = byte;
      update_checksum(byte);
      state_ = State::kId;
      return GpsParseError::kIncomplete;

    case State::kId:
      msg_id_ = byte;
      update_checksum(byte);
      state_ = State::kLength1;
      return GpsParseError::kIncomplete;

    case State::kLength1:
      length_ = byte;
      update_checksum(byte);
      state_ = State::kLength2;
      return GpsParseError::kIncomplete;

    case State::kLength2:
      length_ = static_cast<std::uint16_t>(length_ | (static_cast<std::uint16_t>(byte) << 8U));
      update_checksum(byte);
      payload_index_ = 0;
      state_ = (length_ == 0) ? State::kChecksumA : State::kPayload;
      return GpsParseError::kIncomplete;

    case State::kPayload:
      update_checksum(byte);
      if (payload_index_ < payload_.size()) {
        payload_[payload_index_] = byte;
      }
      ++payload_index_;
      if (payload_index_ >= length_) {
        state_ = State::kChecksumA;
      }
      return GpsParseError::kIncomplete;

    case State::kChecksumA:
      if (byte != ck_a_) {
        reset();
        return GpsParseError::kChecksumMismatch;
      }
      state_ = State::kChecksumB;
      return GpsParseError::kIncomplete;

    case State::kChecksumB: {
      const bool checksum_ok = (byte == ck_b_);
      const std::uint8_t msg_class = msg_class_;
      const std::uint8_t msg_id = msg_id_;
      const std::uint16_t payload_length = length_;
      reset();

      if (!checksum_ok) {
        return GpsParseError::kChecksumMismatch;
      }
      if (msg_class != kNavClass || msg_id != kNavPvtId) {
        return GpsParseError::kUnsupportedMessage;
      }
      return decode_nav_pvt(payload_length, timestamp_us, out);
    }
  }

  return GpsParseError::kIncomplete;
}

std::uint32_t UbxParser::read_u32(std::size_t offset) const {
  return static_cast<std::uint32_t>(payload_[offset]) | (static_cast<std::uint32_t>(payload_[offset + 1]) << 8U) |
         (static_cast<std::uint32_t>(payload_[offset + 2]) << 16U) |
         (static_cast<std::uint32_t>(payload_[offset + 3]) << 24U);
}

std::int32_t UbxParser::read_i32(std::size_t offset) const {
  return static_cast<std::int32_t>(read_u32(offset));
}

GpsFixType UbxParser::to_fix_type(std::uint8_t ubx_fix_type) {
  switch (ubx_fix_type) {
    case 2:
      return GpsFixType::kFix2D;
    case 3:
    case 4:
      return GpsFixType::kFix3D;
    default:
      return GpsFixType::kNoFix;
  }
}

GpsParseError UbxParser::decode_nav_pvt(std::uint16_t payload_length, std::uint64_t timestamp_us,
                                         GpsFix& out) const {
  if (payload_length < kNavPvtMinPayloadLength || payload_length > kMaxPayloadLength) {
    return GpsParseError::kMalformed;
  }

  const std::uint8_t fix_type_raw = payload_[20];
  const std::uint8_t num_satellites = payload_[23];
  const std::int32_t lon_1e7 = read_i32(24);
  const std::int32_t lat_1e7 = read_i32(28);
  const std::int32_t height_mm = read_i32(32);
  const std::uint32_t h_acc_mm = read_u32(40);
  const std::uint32_t v_acc_mm = read_u32(44);
  const std::int32_t ground_speed_mm_s = read_i32(60);
  const std::int32_t heading_motion_1e5 = read_i32(64);

  out.latitude_deg = static_cast<double>(lat_1e7) * 1e-7;
  out.longitude_deg = static_cast<double>(lon_1e7) * 1e-7;
  out.altitude_m = static_cast<float>(height_mm) / 1000.0F;
  out.ground_speed_mps = static_cast<float>(ground_speed_mm_s) / 1000.0F;
  out.course_deg = static_cast<float>(heading_motion_1e5) * 1e-5F;
  out.horizontal_accuracy_m = static_cast<float>(h_acc_mm) / 1000.0F;
  out.vertical_accuracy_m = static_cast<float>(v_acc_mm) / 1000.0F;
  out.num_satellites = num_satellites;
  out.fix_type = to_fix_type(fix_type_raw);
  out.timestamp_us = timestamp_us;
  return GpsParseError::kNone;
}

}  // namespace hemerion::sensors::gps
