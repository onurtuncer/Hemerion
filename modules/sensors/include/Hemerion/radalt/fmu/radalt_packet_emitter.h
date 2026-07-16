// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file radalt_packet_emitter.h
/// @brief RadAltRawSample to wire-exact Hemerion radar altimeter frame
/// encoder.
///
/// Encodes a RadAltRawSample into a complete raw-sample frame of the
/// Hemerion radar altimeter wire protocol -- the inverse of
/// RadAltPacketParser::parse_byte()
/// (modules/sensors/include/Hemerion/radalt/radalt_packet.h, where the frame
/// and payload layouts are documented). The round-trip test in
/// test_radalt_packet.cpp feeds this emitter's output back through the real
/// parser to prove the two stay in sync.

#pragma once

#include <array>
#include <cstddef>

#include "Hemerion/radalt/radalt_packet.h"
#include "Hemerion/radalt/radalt_types.h"

namespace hemerion::sensors::radalt::fmu
{

/// @brief Stateless encoder producing complete Hemerion radar altimeter
/// raw-sample frames; see @ref radalt_packet.h for the frame layout.
class RadAltPacketEmitter
{
public:
  /// Full raw-sample frame length [bytes].
  static constexpr std::size_t kFrameLength = kRadAltPacketRawSampleFrameLength;

  /// One complete raw-sample frame, ready to transmit.
  using Frame = std::array<std::uint8_t, kFrameLength>;

  /// @brief Encodes `raw` as a complete raw-sample frame.
  ///
  /// Ready to hand to UdpSender or any byte-stream sink that feeds a real
  /// or emulated UART RX line.
  ///
  /// @param raw Register counts + status + timestamp to encode.
  /// @return The encoded frame.
  [[nodiscard]] static Frame encode_raw_sample(const RadAltRawSample& raw);
};

}  // namespace hemerion::sensors::radalt::fmu
