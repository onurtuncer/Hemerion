// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file nmeaParser.cpp
/// @brief Implements the streaming NMEA 0183 sentence parser declared in
/// nmeaParser.hpp.

#include "Hemerion/gps/nmeaParser.hpp"

namespace hemerion::sensors::gps
{

namespace
{

constexpr float kKnotsToMps = 0.514444F;

bool parse_hex_digit(char c, std::uint8_t& out)
{
  if (c >= '0' && c <= '9')
  {
    out = static_cast<std::uint8_t>(c - '0');
    return true;
  }
  if (c >= 'A' && c <= 'F')
  {
    out = static_cast<std::uint8_t>(c - 'A' + 10);
    return true;
  }
  if (c >= 'a' && c <= 'f')
  {
    out = static_cast<std::uint8_t>(c - 'a' + 10);
    return true;
  }
  return false;
}

bool parse_hex_byte(std::string_view two_chars, std::uint8_t& out)
{
  std::uint8_t hi = 0;
  std::uint8_t lo = 0;
  if (two_chars.size() != 2 || !parse_hex_digit(two_chars[0], hi) || !parse_hex_digit(two_chars[1], lo))
  {
    return false;
  }
  out = static_cast<std::uint8_t>((hi << 4U) | lo);
  return true;
}

// Manual decimal parser: no locale dependency and no heap allocation, unlike
// std::strtod. Rejects anything that isn't a plain [+-]?digits[.digits]
// field, which is all NMEA numeric fields ever contain.
bool parse_double(std::string_view field, double& out)
{
  if (field.empty())
  {
    return false;
  }

  std::size_t i = 0;
  double sign = 1.0;
  if (field[i] == '-')
  {
    sign = -1.0;
    ++i;
  }
  else if (field[i] == '+')
  {
    ++i;
  }

  double value = 0.0;
  bool any_digits = false;
  while (i < field.size() && field[i] >= '0' && field[i] <= '9')
  {
    value = value * 10.0 + static_cast<double>(field[i] - '0');
    any_digits = true;
    ++i;
  }

  if (i < field.size() && field[i] == '.')
  {
    ++i;
    double scale = 0.1;
    while (i < field.size() && field[i] >= '0' && field[i] <= '9')
    {
      value += static_cast<double>(field[i] - '0') * scale;
      scale *= 0.1;
      any_digits = true;
      ++i;
    }
  }

  if (!any_digits || i != field.size())
  {
    return false;
  }
  out = sign * value;
  return true;
}

bool parse_uint(std::string_view field, unsigned& out)
{
  if (field.empty())
  {
    return false;
  }
  unsigned value = 0;
  for (const char c : field)
  {
    if (c < '0' || c > '9')
    {
      return false;
    }
    value = (value * 10U) + static_cast<unsigned>(c - '0');
  }
  out = value;
  return true;
}

// Converts NMEA's "ddmm.mmmm" (or "dddmm.mmmm" for longitude) format to
// signed-magnitude decimal degrees; the caller applies the N/S or E/W sign.
double dmm_to_decimal_degrees(double dmm)
{
  const auto degrees = static_cast<double>(static_cast<long long>(dmm / 100.0));
  const double minutes = dmm - (degrees * 100.0);
  return degrees + (minutes / 60.0);
}

}  // namespace

// Splits `sentence` on ',' into `fields`. Excess fields beyond `fields.size()`
// are silently dropped, matching the bounded-buffer style used throughout
// this module -- a sentence with more fields than expected is still decoded
// using whichever leading fields fit.
std::size_t NmeaParser::split_fields(std::string_view sentence, FieldArray& fields)
{
  std::size_t count = 0;
  std::size_t start = 0;
  for (std::size_t i = 0; i <= sentence.size() && count < fields.size(); ++i)
  {
    if (i == sentence.size() || sentence[i] == ',')
    {
      fields[count] = sentence.substr(start, i - start);
      ++count;
      start = i + 1;
    }
  }
  return count;
}

GpsParseError NmeaParser::parse_byte(std::uint8_t byte, std::uint64_t timestamp_us, GpsFix& out)
{
  const auto c = static_cast<char>(byte);

  if (c == '$')
  {
    in_sentence_ = true;
    length_ = 0;
    return GpsParseError::kIncomplete;
  }

  if (!in_sentence_)
  {
    return GpsParseError::kIncomplete;
  }

  if (c == '\r' || c == '\n')
  {
    in_sentence_ = false;
    if (length_ == 0)
    {
      return GpsParseError::kIncomplete;
    }
    return decode_sentence(timestamp_us, out);
  }

  if (length_ >= buffer_.size())
  {
    in_sentence_ = false;
    length_ = 0;
    return GpsParseError::kMalformed;
  }

  buffer_[length_] = c;
  ++length_;
  return GpsParseError::kIncomplete;
}

GpsParseError NmeaParser::decode_sentence(std::uint64_t timestamp_us, GpsFix& out)
{
  const std::string_view sentence(buffer_.data(), length_);
  const std::size_t star = sentence.rfind('*');
  if (star == std::string_view::npos || star + 3 != sentence.size())
  {
    return GpsParseError::kMalformed;
  }

  std::uint8_t checksum = 0;
  for (std::size_t i = 0; i < star; ++i)
  {
    checksum ^= static_cast<std::uint8_t>(sentence[i]);
  }

  std::uint8_t expected_checksum = 0;
  if (!parse_hex_byte(sentence.substr(star + 1, 2), expected_checksum))
  {
    return GpsParseError::kMalformed;
  }
  if (checksum != expected_checksum)
  {
    return GpsParseError::kChecksumMismatch;
  }

  FieldArray fields;
  const std::size_t field_count = split_fields(sentence.substr(0, star), fields);
  if (field_count == 0 || fields[0].size() < 3)
  {
    return GpsParseError::kUnsupportedMessage;
  }

  // Talker ID (e.g. "GP", "GN", "GL") is the first 1-2 chars; the sentence
  // type is always the trailing 3.
  const std::string_view sentence_id = fields[0].substr(fields[0].size() - 3);
  if (sentence_id == "GGA")
  {
    return decode_gga(fields, field_count, timestamp_us, out);
  }
  if (sentence_id == "RMC")
  {
    return decode_rmc(fields, field_count, timestamp_us, out);
  }
  return GpsParseError::kUnsupportedMessage;
}

GpsParseError
NmeaParser::decode_gga(const FieldArray& fields, std::size_t field_count, std::uint64_t timestamp_us, GpsFix& out)
{
  // $--GGA,time,lat,N/S,lon,E/W,quality,numSV,HDOP,altitude,M,...
  if (field_count < 10 || fields[3].empty() || fields[5].empty())
  {
    return GpsParseError::kMalformed;
  }

  double raw_lat = 0.0;
  double raw_lon = 0.0;
  double altitude_m = 0.0;
  unsigned quality = 0;
  unsigned num_satellites = 0;
  if (!parse_double(fields[2], raw_lat) || !parse_double(fields[4], raw_lon) || !parse_uint(fields[6], quality) ||
      !parse_uint(fields[7], num_satellites) || !parse_double(fields[9], altitude_m))
  {
    return GpsParseError::kMalformed;
  }

  double latitude_deg = dmm_to_decimal_degrees(raw_lat);
  if (fields[3] == "S")
  {
    latitude_deg = -latitude_deg;
  }
  double longitude_deg = dmm_to_decimal_degrees(raw_lon);
  if (fields[5] == "W")
  {
    longitude_deg = -longitude_deg;
  }

  out.latitude_deg = latitude_deg;
  out.longitude_deg = longitude_deg;
  out.altitude_m = static_cast<float>(altitude_m);
  out.num_satellites = static_cast<std::uint8_t>(num_satellites);
  // GGA's fix-quality field only distinguishes GPS/DGPS/RTK fixes from no
  // fix, not 2D vs. 3D -- treat any non-zero quality as a 3D fix.
  out.fix_type = (quality == 0) ? GpsFixType::kNoFix : GpsFixType::kFix3D;
  out.timestamp_us = timestamp_us;
  return GpsParseError::kNone;
}

GpsParseError
NmeaParser::decode_rmc(const FieldArray& fields, std::size_t field_count, std::uint64_t timestamp_us, GpsFix& out)
{
  // $--RMC,time,status,lat,N/S,lon,E/W,speed_kt,course,date,...
  if (field_count < 9 || fields[2].empty() || fields[4].empty() || fields[6].empty())
  {
    return GpsParseError::kMalformed;
  }

  double raw_lat = 0.0;
  double raw_lon = 0.0;
  double speed_knots = 0.0;
  double course_deg = 0.0;
  if (!parse_double(fields[3], raw_lat) || !parse_double(fields[5], raw_lon) || !parse_double(fields[7], speed_knots) ||
      !parse_double(fields[8], course_deg))
  {
    return GpsParseError::kMalformed;
  }

  double latitude_deg = dmm_to_decimal_degrees(raw_lat);
  if (fields[4] == "S")
  {
    latitude_deg = -latitude_deg;
  }
  double longitude_deg = dmm_to_decimal_degrees(raw_lon);
  if (fields[6] == "W")
  {
    longitude_deg = -longitude_deg;
  }

  out.latitude_deg = latitude_deg;
  out.longitude_deg = longitude_deg;
  out.ground_speed_mps = static_cast<float>(speed_knots) * kKnotsToMps;
  out.course_deg = static_cast<float>(course_deg);
  // RMC carries no satellite count or fix-quality field; status A/V (valid /
  // void) is the only fix indicator it provides.
  out.fix_type = (fields[2] == "A") ? GpsFixType::kFix3D : GpsFixType::kNoFix;
  out.timestamp_us = timestamp_us;
  return GpsParseError::kNone;
}

}  // namespace hemerion::sensors::gps
