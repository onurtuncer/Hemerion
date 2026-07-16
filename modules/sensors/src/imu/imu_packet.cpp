// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file imu_packet.cpp
/// @brief Implements the streaming Hemerion IMU packet parser declared in
/// imu_packet.h.

#include "Hemerion/imu/imu_packet.h"

namespace hemerion::sensors::imu
{

ImuPacketError ImuPacketParser::parse_byte(std::uint8_t byte, ImuRawSample& out)
{
  switch (state_)
  {
    case State::kSync1:
      if (byte == kImuPacketSync1)
      {
        state_ = State::kSync2;
      }
      return ImuPacketError::kIncomplete;

    case State::kSync2:
      if (byte == kImuPacketSync2)
      {
        ck_a_ = 0;
        ck_b_ = 0;
        state_ = State::kId;
      }
      else
      {
        // A stray 0xA5 not followed by 0x5A: this byte might itself open a
        // real frame, so re-examine it as a sync1 candidate.
        state_ = (byte == kImuPacketSync1) ? State::kSync2 : State::kSync1;
      }
      return ImuPacketError::kIncomplete;

    case State::kId:
      msg_id_ = byte;
      update_checksum(byte);
      state_ = State::kLength1;
      return ImuPacketError::kIncomplete;

    case State::kLength1:
      length_ = byte;
      update_checksum(byte);
      state_ = State::kLength2;
      return ImuPacketError::kIncomplete;

    case State::kLength2:
    {
      length_ = static_cast<std::uint16_t>(length_ | (static_cast<std::uint16_t>(byte) << 8));
      update_checksum(byte);
      const bool known = (msg_id_ == kImuPacketRawSampleId);
      if (known && length_ != kImuPacketRawSamplePayloadLength)
      {
        // A raw-sample frame with the wrong length can't be decoded; treat
        // the whole frame as garbage and hunt for the next sync sequence.
        reset();
        return ImuPacketError::kIncomplete;
      }
      if (!known && length_ > kMaxSkipLength)
      {
        reset();
        return ImuPacketError::kIncomplete;
      }
      payload_index_ = 0;
      state_ = (length_ == 0) ? State::kChecksumA : State::kPayload;
      return ImuPacketError::kIncomplete;
    }

    case State::kPayload:
      update_checksum(byte);
      if (msg_id_ == kImuPacketRawSampleId)
      {
        payload_[payload_index_] = byte;
      }
      ++payload_index_;
      if (payload_index_ == length_)
      {
        state_ = State::kChecksumA;
      }
      return ImuPacketError::kIncomplete;

    case State::kChecksumA:
      if (byte != ck_a_)
      {
        reset();
        return ImuPacketError::kChecksumMismatch;
      }
      state_ = State::kChecksumB;
      return ImuPacketError::kIncomplete;

    case State::kChecksumB:
    {
      const bool checksum_ok = (byte == ck_b_);
      const std::uint8_t msg_id = msg_id_;
      reset();
      if (!checksum_ok)
      {
        return ImuPacketError::kChecksumMismatch;
      }
      if (msg_id != kImuPacketRawSampleId)
      {
        return ImuPacketError::kUnsupportedMessage;
      }
      decode_raw_sample(out);
      return ImuPacketError::kNone;
    }
  }

  reset();
  return ImuPacketError::kIncomplete;
}

void ImuPacketParser::update_checksum(std::uint8_t byte)
{
  ck_a_ = static_cast<std::uint8_t>(ck_a_ + byte);
  ck_b_ = static_cast<std::uint8_t>(ck_b_ + ck_a_);
}

void ImuPacketParser::reset()
{
  state_ = State::kSync1;
  msg_id_ = 0;
  length_ = 0;
  payload_index_ = 0;
  ck_a_ = 0;
  ck_b_ = 0;
}

void ImuPacketParser::decode_raw_sample(ImuRawSample& out) const
{
  out.accel_x = read_i32(0);
  out.accel_y = read_i32(4);
  out.accel_z = read_i32(8);
  out.gyro_x = read_i32(12);
  out.gyro_y = read_i32(16);
  out.gyro_z = read_i32(20);
  out.timestamp_us = read_u64(24);
}

std::uint32_t ImuPacketParser::read_u32(std::size_t offset) const
{
  return static_cast<std::uint32_t>(payload_[offset]) | (static_cast<std::uint32_t>(payload_[offset + 1]) << 8) |
         (static_cast<std::uint32_t>(payload_[offset + 2]) << 16) |
         (static_cast<std::uint32_t>(payload_[offset + 3]) << 24);
}

std::int32_t ImuPacketParser::read_i32(std::size_t offset) const { return static_cast<std::int32_t>(read_u32(offset)); }

std::uint64_t ImuPacketParser::read_u64(std::size_t offset) const
{
  return static_cast<std::uint64_t>(read_u32(offset)) | (static_cast<std::uint64_t>(read_u32(offset + 4)) << 32);
}

}  // namespace hemerion::sensors::imu
