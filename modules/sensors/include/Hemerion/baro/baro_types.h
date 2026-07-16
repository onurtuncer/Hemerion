// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file baro_types.h
/// @brief Barometer sample and sensitivity types shared by drivers and conversion.

#pragma once

#include <cstdint>

/// @namespace hemerion::sensors::baro
/// @brief Barometer sample types and raw-count to SI-unit conversion.
namespace hemerion::sensors::baro
{

/// Raw compensated pressure/temperature counts as read off a barometric
/// pressure sensor's conversion registers (MS5611-class parts output
/// second-order-compensated 24-bit words), in whatever LSB units the part's
/// datasheet specifies.
struct BaroRawSample
{
  std::int32_t pressure = 0;       ///< Compensated pressure [LSB].
  std::int32_t temperature = 0;    ///< Compensated die temperature [LSB].
  std::uint64_t timestamp_us = 0;  ///< Local clock at sample time [microseconds].
};

/// Barometer sample in SI-friendly units: pressure in Pa, temperature in
/// degrees Celsius.
struct BaroSample
{
  float pressure_pa = 0.0F;        ///< Static pressure [Pa].
  float temperature_c = 0.0F;      ///< Die temperature [degrees Celsius].
  std::uint64_t timestamp_us = 0;  ///< Local clock at sample time [microseconds].
};

/// Sensitivity of a specific barometer's compensated output words.
struct BaroScale
{
  float pressure_lsb_per_pa = 0.0F;    ///< Pressure sensitivity [LSB per Pa].
  float temperature_lsb_per_c = 0.0F;  ///< Temperature sensitivity [LSB per degree Celsius].
};

/// Result of convert_raw_to_si().
enum class BaroConversionError : std::uint8_t
{
  kNone,          ///< Conversion succeeded.
  kInvalidScale,  ///< A sensitivity in BaroScale was not strictly positive.
};

}  // namespace hemerion::sensors::baro
