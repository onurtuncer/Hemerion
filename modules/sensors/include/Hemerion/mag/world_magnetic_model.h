// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file world_magnetic_model.h
/// @brief The World Magnetic Model: the real geomagnetic field, to degree and
/// order 12.
///
/// **What this replaces, and why it matters.** The examples drove their
/// magnetometer with a centred tilted dipole, which reproduces the two coarse
/// facts a field has -- it roughly doubles from equator to pole and falls off
/// as 1/r^3 -- and gets *declination* badly wrong, because a centred dipole
/// has almost no declination structure at all. At Kitty Hawk the dipole gives
/// about +0.69 degrees where the real field is about -11. That was harmless
/// while the simulation was its own truth and the flight computer had no
/// independent reference to disagree with, and it stops being harmless the
/// moment anything carries a declination table: a heading algorithm checked
/// against a dipole is checked against the wrong field.
///
/// **The coefficients are not in this file and were not typed.** They are
/// generated from `vendor/wmm/WMM.COF`, NOAA's own distribution, into
/// `wmm_coefficients.h`, and `tools/generate_wmm_coefficients.py --check`
/// fails if the two have drifted. See `vendor/wmm/README.md` for provenance,
/// checksums and the citation NCEI asks for.
///
/// **The model is predictive and expires.** WMM2025 is valid from 2025.0 to
/// 2030.0 and the coefficients carry a linear secular variation that is only
/// meaningful inside that window. Evaluating outside it is not a small
/// extrapolation error -- it is using a straight line far past the data that
/// fitted it -- so `field_ned()` refuses rather than extrapolating quietly,
/// and the caller has to decide what to do about it. That is deliberate: the
/// failure mode this guards against is a vehicle flying in 2031 on a silently
/// stale field model.
///
/// **Geodetic, not spherical.** Unlike the dipole it replaces, this converts
/// WGS-84 geodetic coordinates to the geocentric ones the expansion needs, and
/// rotates the result back. The ellipsoidal correction is a few tenths of a
/// percent, which was false precision beside a dipole's ~10% error and is not
/// beside a model good to tens of nanotesla.
///
/// **On-target by construction.** No allocation, no exceptions, no recursion,
/// compile-time loop bounds, and the recurrences evaluate in fixed work. It
/// compiles in the cross build for the same reason `magnetic_calibration.h`
/// does: a filter that carries a declination correction needs this on the
/// flight computer, not in a ground tool.

#pragma once

#include <array>
#include <cstdint>

#include "Hemerion/mag/wmm_coefficients.h"

namespace hemerion::sensors::mag
{

/// Why a field could not be evaluated.
enum class WmmError : std::uint8_t
{
  kNone = 0,         ///< Evaluated; the result is usable.
  kDateOutOfRange,   ///< Outside the model's validity window. Not extrapolated
                     ///< -- see the file comment on why this is refused rather
                     ///< than approximated.
  kInvalidPosition,  ///< Latitude outside [-90, 90] or a non-finite input.
};

/// A magnetic field in the local geodetic frame [nanotesla].
///
/// Nanotesla, not microtesla, because that is the unit the model and its
/// coefficients are in and converting on the way out would invite a factor of
/// a thousand in the wrong place. `WmmField::to_microtesla()` is explicit
/// where a caller wants the sensor stack's working unit.
struct WmmField
{
  double north_nt = 0.0;  ///< X, geodetic north.
  double east_nt = 0.0;   ///< Y, east.
  double down_nt = 0.0;   ///< Z, down.

  /// @brief Total intensity F [nT].
  [[nodiscard]] double intensity_nt() const;

  /// @brief Horizontal intensity H [nT].
  [[nodiscard]] double horizontal_nt() const;

  /// @brief Declination D [degrees], east of true north.
  [[nodiscard]] double declination_deg() const;

  /// @brief Inclination I [degrees], positive downward.
  [[nodiscard]] double inclination_deg() const;

  /// @brief The same field in the sensor stack's working unit [uT].
  [[nodiscard]] std::array<double, 3> to_microtesla() const;
};

/// @brief Evaluates the World Magnetic Model.
///
/// Stateless and cheap enough to call per step; there is nothing to construct
/// and nothing to keep.
class WorldMagneticModel
{
public:
  /// First year the model is valid for [decimal year].
  static constexpr double kValidFromYear = wmm::kEpochYear;
  /// First year it is *not* valid for [decimal year].
  static constexpr double kValidUntilYear = wmm::kValidUntilYear;
  /// The model this was built from, e.g. "WMM-2025".
  static constexpr const char* kModelName = wmm::kModelName;

  /// @brief The field at a geodetic position and date.
  ///
  /// @param latitude_deg  Geodetic latitude [degrees], north positive.
  /// @param longitude_deg Longitude [degrees], east positive.
  /// @param altitude_m    Height above the WGS-84 ellipsoid [m].
  /// @param decimal_year  Date [decimal year], e.g. 2026.75. Must lie in
  ///                      [kValidFromYear, kValidUntilYear].
  /// @param out           Receives the field on success, untouched otherwise.
  /// @return kNone on success, else why not.
  [[nodiscard]] static WmmError
  field_ned(double latitude_deg, double longitude_deg, double altitude_m, double decimal_year, WmmField& out);

  /// @brief Whether a date is inside the model's validity window.
  ///
  /// Exposed so a caller can check once at start-up rather than discover it
  /// per call -- and so the answer is available before there is a position to
  /// evaluate at.
  [[nodiscard]] static bool covers(double decimal_year);
};

}  // namespace hemerion::sensors::mag
