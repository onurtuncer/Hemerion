// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file nmeaParser.hpp
/// @brief Streaming parser for NMEA 0183 sentences.
///
/// Decodes $--GGA (position, altitude, fix quality, satellite count) and
/// $--RMC (position, speed, course, fix validity); other sentence types are
/// reported as kUnsupportedMessage. GGA and RMC each carry only part of a
/// full fix, so `out` is an in/out parameter -- feed the same GpsFix
/// instance across calls to accumulate a complete fix from a receiver's
/// usual GGA+RMC pair.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "Hemerion/gps/gpsTypes.hpp"

namespace hemerion::sensors::gps {

/// @brief Byte-at-a-time NMEA 0183 sentence parser; see @ref nmeaParser.hpp
/// for which sentences are decoded.
class NmeaParser {
 public:
  /// @brief Feeds one byte from the receiver.
  ///
  /// @param byte         Next raw byte of the NMEA stream.
  /// @param timestamp_us Caller's local clock value when this byte was
  ///                     received; copied into `out.timestamp_us` on success.
  /// @param out          Updated in place when a supported sentence completes;
  ///                     feed the same instance across calls to accumulate a
  ///                     full fix from a GGA+RMC pair.
  /// @return kNone once a supported sentence has been fully decoded and its
  ///         checksum verified (with `out` updated); kIncomplete while a
  ///         sentence is still being assembled; kChecksumMismatch,
  ///         kUnsupportedMessage, or kMalformed if the completed sentence is
  ///         rejected.
  [[nodiscard]] GpsParseError parse_byte(std::uint8_t byte, std::uint64_t timestamp_us, GpsFix& out);

 private:
  // NMEA 0183 caps a sentence (excluding the leading '$' and trailing CRLF)
  // at 79 characters.
  static constexpr std::size_t kMaxSentenceLength = 79;
  static constexpr std::size_t kMaxFields = 16;

  using FieldArray = std::array<std::string_view, kMaxFields>;

  GpsParseError decode_sentence(std::uint64_t timestamp_us, GpsFix& out);
  static std::size_t split_fields(std::string_view sentence, FieldArray& fields);
  static GpsParseError decode_gga(const FieldArray& fields, std::size_t field_count, std::uint64_t timestamp_us,
                                   GpsFix& out);
  static GpsParseError decode_rmc(const FieldArray& fields, std::size_t field_count, std::uint64_t timestamp_us,
                                   GpsFix& out);

  std::array<char, kMaxSentenceLength> buffer_{};
  std::size_t length_ = 0;
  bool in_sentence_ = false;
};

}  // namespace hemerion::sensors::gps
