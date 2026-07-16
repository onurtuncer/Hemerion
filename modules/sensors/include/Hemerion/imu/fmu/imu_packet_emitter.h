// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file imu_packet_emitter.h
/// @brief ImuRawSample to wire-exact Hemerion IMU frame encoder.
///
/// Encodes an ImuRawSample into a complete raw-sample frame of the Hemerion
/// IMU wire protocol -- the inverse of ImuPacketParser::parse_byte()
/// (modules/sensors/include/Hemerion/imu/imu_packet.h, where the frame and
/// payload layouts are documented). The round-trip test in
/// test_imu_packet.cpp feeds this emitter's output back through the real
/// parser to prove the two stay in sync.

#pragma once

#include <array>
#include <cstddef>

#include "Hemerion/imu/imu_packet.h"
#include "Hemerion/imu/imu_types.h"

namespace hemerion::sensors::imu::fmu
{

/// @brief Stateless encoder producing complete Hemerion IMU raw-sample
/// frames; see @ref imu_packet.h for the frame layout.
class ImuPacketEmitter
{
public:
  /// Full raw-sample frame length [bytes].
  static constexpr std::size_t kFrameLength = kImuPacketRawSampleFrameLength;

  /// One complete raw-sample frame, ready to transmit.
  using Frame = std::array<std::uint8_t, kFrameLength>;

  /// @brief Encodes `raw` as a complete raw-sample frame.
  ///
  /// Ready to hand to UdpSender or any byte-stream sink that feeds a real
  /// or emulated UART RX line.
  ///
  /// @param raw Register counts + timestamp to encode.
  /// @return The encoded frame.
  [[nodiscard]] static Frame encode_raw_sample(const ImuRawSample& raw);
};

}  // namespace hemerion::sensors::imu::fmu
