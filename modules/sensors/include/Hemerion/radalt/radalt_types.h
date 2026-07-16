// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file radalt_types.h
/// @brief Radar altimeter sample and sensitivity types shared by drivers and
/// conversion.

#pragma once

#include <cstdint>

/// @namespace hemerion::sensors::radalt
/// @brief Radar altimeter sample types and raw-count to SI-unit conversion.
namespace hemerion::sensors::radalt
{

/// Track status values carried in RadAltRawSample::status. Values above
/// kTrackValid are reserved for future use and must be treated as invalid.
inline constexpr std::uint32_t kRadAltStatusNoReturn = 0;    ///< No ground return (out of range or loss of track).
inline constexpr std::uint32_t kRadAltStatusTrackValid = 1;  ///< Range word carries a valid ground return.

/// Raw slant-range counts as read off a radar altimeter's range register, in
/// whatever LSB units the part's datasheet specifies, plus its track status
/// word.
struct RadAltRawSample
{
  std::int32_t range = 0;          ///< Range to ground [LSB]; meaningless unless status is kRadAltStatusTrackValid.
  std::uint32_t status = 0;        ///< Track status word (kRadAltStatusNoReturn / kRadAltStatusTrackValid).
  std::uint64_t timestamp_us = 0;  ///< Local clock at sample time [microseconds].
};

/// Radar altimeter sample in SI units.
struct RadAltSample
{
  float range_m = 0.0F;            ///< Range to ground [m]; meaningless unless `valid`.
  bool valid = false;              ///< True when the range word carries a valid ground return.
  std::uint64_t timestamp_us = 0;  ///< Local clock at sample time [microseconds].
};

/// Sensitivity of a specific radar altimeter's range register.
struct RadAltScale
{
  float range_lsb_per_m = 0.0F;  ///< Range sensitivity [LSB per meter].
};

/// Result of convert_raw_to_si().
enum class RadAltConversionError : std::uint8_t
{
  kNone,          ///< Conversion succeeded.
  kInvalidScale,  ///< The sensitivity in RadAltScale was not strictly positive.
};

}  // namespace hemerion::sensors::radalt
