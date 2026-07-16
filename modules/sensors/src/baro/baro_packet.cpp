// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file baro_packet.cpp
/// @brief Implements the streaming Hemerion barometer packet parser declared
/// in baro_packet.h.

#include "Hemerion/baro/baro_packet.h"

namespace hemerion::sensors::baro
{

BaroPacketError BaroPacketParser::parse_byte(std::uint8_t byte, BaroRawSample& out)
{
  switch (state_)
  {
    case State::kSync1:
      if (byte == kBaroPacketSync1)
      {
        state_ = State::kSync2;
      }
      return BaroPacketError::kIncomplete;

    case State::kSync2:
      if (byte == kBaroPacketSync2)
      {
        ck_a_ = 0;
        ck_b_ = 0;
        state_ = State::kId;
      }
      else
      {
        // A stray 0xB7 not followed by 0x7B: this byte might itself open a
        // real frame, so re-examine it as a sync1 candidate.
        state_ = (byte == kBaroPacketSync1) ? State::kSync2 : State::kSync1;
      }
      return BaroPacketError::kIncomplete;

    case State::kId:
      msg_id_ = byte;
      update_checksum(byte);
      state_ = State::kLength1;
      return BaroPacketError::kIncomplete;

    case State::kLength1:
      length_ = byte;
      update_checksum(byte);
      state_ = State::kLength2;
      return BaroPacketError::kIncomplete;

    case State::kLength2:
    {
      length_ = static_cast<std::uint16_t>(length_ | (static_cast<std::uint16_t>(byte) << 8));
      update_checksum(byte);
      const bool known = (msg_id_ == kBaroPacketRawSampleId);
      if (known && length_ != kBaroPacketRawSamplePayloadLength)
      {
        // A raw-sample frame with the wrong length can't be decoded; treat
        // the whole frame as garbage and hunt for the next sync sequence.
        reset();
        return BaroPacketError::kIncomplete;
      }
      if (!known && length_ > kMaxSkipLength)
      {
        reset();
        return BaroPacketError::kIncomplete;
      }
      payload_index_ = 0;
      state_ = (length_ == 0) ? State::kChecksumA : State::kPayload;
      return BaroPacketError::kIncomplete;
    }

    case State::kPayload:
      update_checksum(byte);
      if (msg_id_ == kBaroPacketRawSampleId)
      {
        payload_[payload_index_] = byte;
      }
      ++payload_index_;
      if (payload_index_ == length_)
      {
        state_ = State::kChecksumA;
      }
      return BaroPacketError::kIncomplete;

    case State::kChecksumA:
      if (byte != ck_a_)
      {
        reset();
        return BaroPacketError::kChecksumMismatch;
      }
      state_ = State::kChecksumB;
      return BaroPacketError::kIncomplete;

    case State::kChecksumB:
    {
      const bool checksum_ok = (byte == ck_b_);
      const std::uint8_t msg_id = msg_id_;
      reset();
      if (!checksum_ok)
      {
        return BaroPacketError::kChecksumMismatch;
      }
      if (msg_id != kBaroPacketRawSampleId)
      {
        return BaroPacketError::kUnsupportedMessage;
      }
      decode_raw_sample(out);
      return BaroPacketError::kNone;
    }
  }

  reset();
  return BaroPacketError::kIncomplete;
}

void BaroPacketParser::update_checksum(std::uint8_t byte)
{
  ck_a_ = static_cast<std::uint8_t>(ck_a_ + byte);
  ck_b_ = static_cast<std::uint8_t>(ck_b_ + ck_a_);
}

void BaroPacketParser::reset()
{
  state_ = State::kSync1;
  msg_id_ = 0;
  length_ = 0;
  payload_index_ = 0;
  ck_a_ = 0;
  ck_b_ = 0;
}

void BaroPacketParser::decode_raw_sample(BaroRawSample& out) const
{
  out.pressure = read_i32(0);
  out.temperature = read_i32(4);
  out.timestamp_us = read_u64(8);
}

std::uint32_t BaroPacketParser::read_u32(std::size_t offset) const
{
  return static_cast<std::uint32_t>(payload_[offset]) | (static_cast<std::uint32_t>(payload_[offset + 1]) << 8) |
         (static_cast<std::uint32_t>(payload_[offset + 2]) << 16) |
         (static_cast<std::uint32_t>(payload_[offset + 3]) << 24);
}

std::int32_t BaroPacketParser::read_i32(std::size_t offset) const
{
  return static_cast<std::int32_t>(read_u32(offset));
}

std::uint64_t BaroPacketParser::read_u64(std::size_t offset) const
{
  return static_cast<std::uint64_t>(read_u32(offset)) | (static_cast<std::uint64_t>(read_u32(offset + 4)) << 32);
}

}  // namespace hemerion::sensors::baro
