// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file mag_packet_emitter.h
/// @brief MagRawSample to wire-exact Hemerion magnetometer frame encoder.
///
/// Encodes a MagRawSample into a complete raw-sample frame of the Hemerion
/// magnetometer wire protocol -- the inverse of MagPacketParser::parse_byte()
/// (modules/sensors/include/Hemerion/mag/mag_packet.h, where the frame and
/// payload layouts are documented). The round-trip test in
/// test_mag_packet.cpp feeds this emitter's output back through the real
/// parser to prove the two stay in sync.

#pragma once

#include <array>
#include <cstddef>

#include "Hemerion/mag/mag_packet.h"
#include "Hemerion/mag/mag_types.h"

namespace hemerion::sensors::mag::fmu
{

/// @brief Stateless encoder producing complete Hemerion magnetometer
/// raw-sample frames; see @ref mag_packet.h for the frame layout.
class MagPacketEmitter
{
public:
  /// Full raw-sample frame length [bytes].
  static constexpr std::size_t kFrameLength = kMagPacketRawSampleFrameLength;

  /// One complete raw-sample frame, ready to transmit.
  using Frame = std::array<std::uint8_t, kFrameLength>;

  /// @brief Encodes `raw` as a complete raw-sample frame.
  ///
  /// Ready to hand to UdpSender or any byte-stream sink that feeds a real
  /// or emulated UART RX line.
  ///
  /// @param raw Register counts + timestamp to encode.
  /// @return The encoded frame.
  [[nodiscard]] static Frame encode_raw_sample(const MagRawSample& raw);
};

}  // namespace hemerion::sensors::mag::fmu
