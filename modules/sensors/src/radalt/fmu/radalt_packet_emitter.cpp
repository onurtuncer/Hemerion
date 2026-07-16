// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file radalt_packet_emitter.cpp
/// @brief Implements the Hemerion radar altimeter frame encoder declared in
/// radalt_packet_emitter.h. Byte offsets mirror
/// RadAltPacketParser::decode_raw_sample() exactly.

#include "Hemerion/radalt/fmu/radalt_packet_emitter.h"

#include <cstdint>

namespace hemerion::sensors::radalt::fmu
{

namespace
{

void write_u32(RadAltPacketEmitter::Frame& frame, std::size_t offset, std::uint32_t value)
{
  frame[offset] = static_cast<std::uint8_t>(value & 0xFFU);
  frame[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFFU);
  frame[offset + 2] = static_cast<std::uint8_t>((value >> 16) & 0xFFU);
  frame[offset + 3] = static_cast<std::uint8_t>((value >> 24) & 0xFFU);
}

void write_i32(RadAltPacketEmitter::Frame& frame, std::size_t offset, std::int32_t value)
{
  write_u32(frame, offset, static_cast<std::uint32_t>(value));
}

void write_u64(RadAltPacketEmitter::Frame& frame, std::size_t offset, std::uint64_t value)
{
  write_u32(frame, offset, static_cast<std::uint32_t>(value & 0xFFFFFFFFULL));
  write_u32(frame, offset + 4, static_cast<std::uint32_t>(value >> 32));
}

}  // namespace

RadAltPacketEmitter::Frame RadAltPacketEmitter::encode_raw_sample(const RadAltRawSample& raw)
{
  Frame frame{};
  frame[0] = kRadAltPacketSync1;
  frame[1] = kRadAltPacketSync2;
  frame[2] = kRadAltPacketRawSampleId;
  frame[3] = static_cast<std::uint8_t>(kRadAltPacketRawSamplePayloadLength & 0xFFU);
  frame[4] = static_cast<std::uint8_t>((kRadAltPacketRawSamplePayloadLength >> 8) & 0xFFU);

  constexpr std::size_t kPayload = 5;  // first payload byte, after sync/id/length
  write_i32(frame, kPayload + 0, raw.range);
  write_u32(frame, kPayload + 4, raw.status);
  write_u64(frame, kPayload + 8, raw.timestamp_us);

  // UBX-style Fletcher checksum over id + length + payload (everything
  // between the sync bytes and the checksum itself).
  std::uint8_t ck_a = 0;
  std::uint8_t ck_b = 0;
  for (std::size_t i = 2; i < kFrameLength - 2; ++i)
  {
    ck_a = static_cast<std::uint8_t>(ck_a + frame[i]);
    ck_b = static_cast<std::uint8_t>(ck_b + ck_a);
  }
  frame[kFrameLength - 2] = ck_a;
  frame[kFrameLength - 1] = ck_b;
  return frame;
}

}  // namespace hemerion::sensors::radalt::fmu
