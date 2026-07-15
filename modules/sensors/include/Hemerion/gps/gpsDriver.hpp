// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file gpsDriver.hpp
/// @brief Protocol-selecting front end for the GPS wire parsers.
///
/// Thin wrapper that feeds a GPS receiver's raw byte stream (UART RX)
/// through whichever wire-protocol parser the receiver is configured to
/// emit. Most receivers (e.g. u-blox modules) speak one protocol at a time
/// rather than a mix, so the protocol is fixed for the driver's lifetime --
/// there is no auto-detection.

#pragma once

#include <cstdint>

#include "Hemerion/gps/gpsTypes.hpp"
#include "Hemerion/gps/nmeaParser.hpp"
#include "Hemerion/gps/ubxParser.hpp"

namespace hemerion::sensors::gps {

/// Wire protocol a GpsDriver expects from its receiver.
enum class GpsProtocol : std::uint8_t {
  kNmea,  ///< NMEA 0183 sentences, decoded by NmeaParser.
  kUbx,   ///< u-blox UBX binary frames, decoded by UbxParser.
};

/// @brief Routes a receiver's byte stream to the parser matching its
/// configured protocol.
class GpsDriver {
 public:
  /// @param protocol Wire protocol the receiver emits; fixed for the
  ///                 driver's lifetime.
  explicit GpsDriver(GpsProtocol protocol) : protocol_(protocol) {}

  /// @brief Feeds one byte from the receiver.
  ///
  /// See NmeaParser::parse_byte() / UbxParser::parse_byte() for the meaning
  /// of the return value and of `timestamp_us`.
  ///
  /// @param byte         Next raw byte of the receiver's stream.
  /// @param timestamp_us Caller's local clock value when this byte was received.
  /// @param out          Updated when the underlying parser completes a fix.
  /// @return The underlying parser's result.
  [[nodiscard]] GpsParseError feed(std::uint8_t byte, std::uint64_t timestamp_us, GpsFix& out) {
    switch (protocol_) {
      case GpsProtocol::kNmea:
        return nmea_parser_.parse_byte(byte, timestamp_us, out);
      case GpsProtocol::kUbx:
        return ubx_parser_.parse_byte(byte, timestamp_us, out);
    }
    return GpsParseError::kUnsupportedMessage;
  }

  /// Wire protocol this driver was constructed with.
  [[nodiscard]] GpsProtocol protocol() const { return protocol_; }

 private:
  GpsProtocol protocol_;
  NmeaParser nmea_parser_;
  UbxParser ubx_parser_;
};

}  // namespace hemerion::sensors::gps
