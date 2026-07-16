// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file baro_packet_emitter.h
/// @brief BaroRawSample to wire-exact Hemerion barometer frame encoder.
///
/// Encodes a BaroRawSample into a complete raw-sample frame of the Hemerion
/// barometer wire protocol -- the inverse of BaroPacketParser::parse_byte()
/// (modules/sensors/include/Hemerion/baro/baro_packet.h, where the frame and
/// payload layouts are documented). The round-trip test in
/// test_baro_packet.cpp feeds this emitter's output back through the real
/// parser to prove the two stay in sync.

#pragma once

#include <array>
#include <cstddef>

#include "Hemerion/baro/baro_packet.h"
#include "Hemerion/baro/baro_types.h"

namespace hemerion::sensors::baro::fmu
{

/// @brief Stateless encoder producing complete Hemerion barometer raw-sample
/// frames; see @ref baro_packet.h for the frame layout.
class BaroPacketEmitter
{
public:
  /// Full raw-sample frame length [bytes].
  static constexpr std::size_t kFrameLength = kBaroPacketRawSampleFrameLength;

  /// One complete raw-sample frame, ready to transmit.
  using Frame = std::array<std::uint8_t, kFrameLength>;

  /// @brief Encodes `raw` as a complete raw-sample frame.
  ///
  /// Ready to hand to UdpSender or any byte-stream sink that feeds a real
  /// or emulated UART RX line.
  ///
  /// @param raw Conversion-word counts + timestamp to encode.
  /// @return The encoded frame.
  [[nodiscard]] static Frame encode_raw_sample(const BaroRawSample& raw);
};

}  // namespace hemerion::sensors::baro::fmu
