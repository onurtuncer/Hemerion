// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file mag_packet.cpp
/// @brief Implements the streaming Hemerion magnetometer packet parser
/// declared in mag_packet.h.

#include "Hemerion/mag/mag_packet.h"

namespace hemerion::sensors::mag
{

MagPacketError MagPacketParser::parse_byte(std::uint8_t byte, MagRawSample& out)
{
  switch (state_)
  {
    case State::kSync1:
      if (byte == kMagPacketSync1)
      {
        state_ = State::kSync2;
      }
      return MagPacketError::kIncomplete;

    case State::kSync2:
      if (byte == kMagPacketSync2)
      {
        ck_a_ = 0;
        ck_b_ = 0;
        state_ = State::kId;
      }
      else
      {
        // A stray 0xD1 not followed by 0x1D: this byte might itself open a
        // real frame, so re-examine it as a sync1 candidate.
        state_ = (byte == kMagPacketSync1) ? State::kSync2 : State::kSync1;
      }
      return MagPacketError::kIncomplete;

    case State::kId:
      msg_id_ = byte;
      update_checksum(byte);
      state_ = State::kLength1;
      return MagPacketError::kIncomplete;

    case State::kLength1:
      length_ = byte;
      update_checksum(byte);
      state_ = State::kLength2;
      return MagPacketError::kIncomplete;

    case State::kLength2:
    {
      length_ = static_cast<std::uint16_t>(length_ | (static_cast<std::uint16_t>(byte) << 8));
      update_checksum(byte);
      const bool known = (msg_id_ == kMagPacketRawSampleId);
      if (known && length_ != kMagPacketRawSamplePayloadLength)
      {
        // A raw-sample frame with the wrong length can't be decoded; treat
        // the whole frame as garbage and hunt for the next sync sequence.
        reset();
        return MagPacketError::kIncomplete;
      }
      if (!known && length_ > kMaxSkipLength)
      {
        reset();
        return MagPacketError::kIncomplete;
      }
      payload_index_ = 0;
      state_ = (length_ == 0) ? State::kChecksumA : State::kPayload;
      return MagPacketError::kIncomplete;
    }

    case State::kPayload:
      update_checksum(byte);
      if (msg_id_ == kMagPacketRawSampleId)
      {
        payload_[payload_index_] = byte;
      }
      ++payload_index_;
      if (payload_index_ == length_)
      {
        state_ = State::kChecksumA;
      }
      return MagPacketError::kIncomplete;

    case State::kChecksumA:
      if (byte != ck_a_)
      {
        reset();
        return MagPacketError::kChecksumMismatch;
      }
      state_ = State::kChecksumB;
      return MagPacketError::kIncomplete;

    case State::kChecksumB:
    {
      const bool checksum_ok = (byte == ck_b_);
      const std::uint8_t msg_id = msg_id_;
      reset();
      if (!checksum_ok)
      {
        return MagPacketError::kChecksumMismatch;
      }
      if (msg_id != kMagPacketRawSampleId)
      {
        return MagPacketError::kUnsupportedMessage;
      }
      decode_raw_sample(out);
      return MagPacketError::kNone;
    }
  }

  reset();
  return MagPacketError::kIncomplete;
}

void MagPacketParser::update_checksum(std::uint8_t byte)
{
  ck_a_ = static_cast<std::uint8_t>(ck_a_ + byte);
  ck_b_ = static_cast<std::uint8_t>(ck_b_ + ck_a_);
}

void MagPacketParser::reset()
{
  state_ = State::kSync1;
  msg_id_ = 0;
  length_ = 0;
  payload_index_ = 0;
  ck_a_ = 0;
  ck_b_ = 0;
}

void MagPacketParser::decode_raw_sample(MagRawSample& out) const
{
  out.mag_x = read_i32(0);
  out.mag_y = read_i32(4);
  out.mag_z = read_i32(8);
  out.timestamp_us = read_u64(12);
}

std::uint32_t MagPacketParser::read_u32(std::size_t offset) const
{
  return static_cast<std::uint32_t>(payload_[offset]) | (static_cast<std::uint32_t>(payload_[offset + 1]) << 8) |
         (static_cast<std::uint32_t>(payload_[offset + 2]) << 16) |
         (static_cast<std::uint32_t>(payload_[offset + 3]) << 24);
}

std::int32_t MagPacketParser::read_i32(std::size_t offset) const
{
  return static_cast<std::int32_t>(read_u32(offset));
}

std::uint64_t MagPacketParser::read_u64(std::size_t offset) const
{
  return static_cast<std::uint64_t>(read_u32(offset)) | (static_cast<std::uint64_t>(read_u32(offset + 4)) << 32);
}

}  // namespace hemerion::sensors::mag
