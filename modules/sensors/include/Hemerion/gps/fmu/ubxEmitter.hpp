// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file ubxEmitter.hpp
/// @brief GpsFix to wire-exact UBX-NAV-PVT frame encoder.
///
/// Encodes a GpsFix into a wire-exact UBX-NAV-PVT (class 0x01, id 0x07)
/// frame -- the inverse of UbxParser::parse_byte()/decode_nav_pvt(). Byte
/// offsets here mirror UbxParser::decode_nav_pvt() exactly
/// (modules/sensors/src/gps/ubxParser.cpp); the round-trip test in
/// test_ubx_emitter.cpp feeds this emitter's output back through the real
/// UbxParser to prove the two stay in sync.
///
/// Only the fields UbxParser actually reads are derived from GpsFix
/// (fixType, numSV, lon, lat, height, hAcc, vAcc, velN/E/D, gSpeed,
/// headMot). Calendar/time fields (iTOW, year/month/day, ...) are left zero
/// -- the on-target parser never reads them, and fabricating a plausible
/// date adds complexity with no consumer. sAcc, headAcc and pDOP are filled
/// with fixed, plausible placeholder values since GpsFix has no
/// corresponding noise terms to derive them from.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "Hemerion/gps/gpsTypes.hpp"

namespace hemerion::sensors::gps::fmu {

/// @brief Stateless encoder producing complete UBX-NAV-PVT frames; see
/// @ref ubxEmitter.hpp for which GpsFix fields are encoded.
class UbxEmitter {
 public:
  /// UBX-NAV-PVT payload length [bytes] (modern firmware layout).
  static constexpr std::size_t kPayloadLength = 92;
  /// Full frame length: 6-byte header + payload + 2 checksum bytes.
  static constexpr std::size_t kFrameLength = 6 /*header*/ + kPayloadLength + 2 /*checksum*/;

  /// One complete UBX-NAV-PVT frame, ready to transmit.
  using Frame = std::array<std::uint8_t, kFrameLength>;

  /// @brief Encodes `fix` as a complete UBX-NAV-PVT frame.
  ///
  /// The frame layout is: sync (0xB5 0x62), class (0x01), id (0x07), 16-bit
  /// little-endian length (92), the 92-byte payload, then the two UBX
  /// checksum bytes. Ready to hand to UdpSender or any byte-stream sink
  /// that feeds a real or emulated UART RX line.
  ///
  /// @param fix Fix to encode.
  /// @return The encoded frame.
  [[nodiscard]] static Frame encode_nav_pvt(const GpsFix& fix);
};

}  // namespace hemerion::sensors::gps::fmu
