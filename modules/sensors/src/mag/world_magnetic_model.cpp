// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file world_magnetic_model.cpp
/// @brief Spherical-harmonic evaluation of the WMM.
///
/// The method is the one the WMM technical report specifies, and the variable
/// names follow it so the two can be read side by side: geodetic to geocentric
/// conversion, Schmidt semi-normalised associated Legendre functions by
/// recurrence, the field in geocentric spherical components, then a rotation
/// back to geodetic.
///
/// Validated against NOAA's own 100 reference values rather than against
/// itself -- see test_world_magnetic_model.cpp.

#include "Hemerion/mag/world_magnetic_model.h"

#include <cmath>
#include <cstddef>
#include <numbers>

namespace hemerion::sensors::mag
{

namespace
{

constexpr double kPi = std::numbers::pi;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

/// WGS-84 semi-major axis [m].
constexpr double kSemiMajorM = 6378137.0;
/// WGS-84 inverse flattening.
constexpr double kInverseFlattening = 298.257223563;
/// Geomagnetic reference radius [m], as the WMM defines it. Not the WGS-84
/// axis: the expansion is written about a sphere of this radius.
constexpr double kGeomagneticReferenceRadiusM = 6371200.0;

constexpr std::size_t kDegree = wmm::kDegree;
/// One past the degree, since the recurrences index n = 0 as well.
constexpr std::size_t kRows = kDegree + 1;

using Table = std::array<std::array<double, kRows>, kRows>;

/// Schmidt semi-normalised associated Legendre functions P(n,m)(cos theta)
/// and their derivatives with respect to colatitude.
///
/// The recursion is the normalised one, not the classical recurrence with a
/// normalisation applied afterwards: the Schmidt factor does not commute with
/// the recurrence's own coefficients, and doing it that way is wrong by tens
/// of thousands of nanotesla -- which is how this was caught, against NOAA's
/// reference values rather than against itself.
///
/// Seeded explicitly at (1,1) because the sectoral step carries a factor
/// sqrt((2n-1)/(2n)) that is only correct from n = 2 upward.
///
/// @param sin_colatitude sin(theta).
/// @param cos_colatitude cos(theta).
/// @param p  Receives P(n,m).
/// @param dp Receives dP(n,m)/dtheta.
void legendre(double sin_colatitude, double cos_colatitude, Table& p, Table& dp)
{
  p[0][0] = 1.0;
  dp[0][0] = 0.0;
  p[1][0] = cos_colatitude;
  dp[1][0] = -sin_colatitude;
  p[1][1] = sin_colatitude;
  dp[1][1] = cos_colatitude;

  for (std::size_t n = 2; n <= kDegree; ++n)
  {
    const auto dn = static_cast<double>(n);

    // Sectoral: P(n,n) from P(n-1,n-1).
    const double sectoral = std::sqrt(((2.0 * dn) - 1.0) / (2.0 * dn));
    p[n][n] = sectoral * sin_colatitude * p[n - 1][n - 1];
    dp[n][n] = sectoral * ((sin_colatitude * dp[n - 1][n - 1]) + (cos_colatitude * p[n - 1][n - 1]));

    for (std::size_t m = 0; m < n; ++m)
    {
      const auto dm = static_cast<double>(m);
      const double lead = std::sqrt((dn * dn) - (dm * dm));
      const double trail = std::sqrt(((dn - 1.0) * (dn - 1.0)) - (dm * dm));
      // P(n-2,m) exists only for n - 2 >= m; below that the term is absent
      // rather than zero-valued, which for this recursion is the same thing.
      const double previous = (n >= m + 2) ? p[n - 2][m] : 0.0;
      const double d_previous = (n >= m + 2) ? dp[n - 2][m] : 0.0;

      p[n][m] = (((2.0 * dn) - 1.0) * cos_colatitude * p[n - 1][m] - (trail * previous)) / lead;
      dp[n][m] = ((((2.0 * dn) - 1.0) * ((cos_colatitude * dp[n - 1][m]) - (sin_colatitude * p[n - 1][m]))) -
                  (trail * d_previous)) /
                 lead;
    }
  }
}

}  // namespace

double WmmField::intensity_nt() const
{
  return std::sqrt((north_nt * north_nt) + (east_nt * east_nt) + (down_nt * down_nt));
}

double WmmField::horizontal_nt() const { return std::sqrt((north_nt * north_nt) + (east_nt * east_nt)); }

double WmmField::declination_deg() const { return std::atan2(east_nt, north_nt) * kRadToDeg; }

double WmmField::inclination_deg() const { return std::atan2(down_nt, horizontal_nt()) * kRadToDeg; }

std::array<double, 3> WmmField::to_microtesla() const { return { north_nt * 1e-3, east_nt * 1e-3, down_nt * 1e-3 }; }

std::array<double, 3> WmmFieldBody::to_microtesla() const { return { x_nt * 1e-3, y_nt * 1e-3, z_nt * 1e-3 }; }

WmmFieldBody to_body(const WmmField& field, double yaw_rad, double pitch_rad, double roll_rad)
{
  const double cy = std::cos(yaw_rad);
  const double sy = std::sin(yaw_rad);
  const double cp = std::cos(pitch_rad);
  const double sp = std::sin(pitch_rad);
  const double cr = std::cos(roll_rad);
  const double sr = std::sin(roll_rad);

  // C_bn, rows = body axes expressed in geodetic components.
  WmmFieldBody body;
  body.x_nt = (cp * cy * field.north_nt) + (cp * sy * field.east_nt) + (-sp * field.down_nt);
  body.y_nt = (((sr * sp * cy) - (cr * sy)) * field.north_nt) + (((sr * sp * sy) + (cr * cy)) * field.east_nt) +
              (sr * cp * field.down_nt);
  body.z_nt = (((cr * sp * cy) + (sr * sy)) * field.north_nt) + (((cr * sp * sy) - (sr * cy)) * field.east_nt) +
              (cr * cp * field.down_nt);
  return body;
}

bool WorldMagneticModel::covers(double decimal_year)
{
  return std::isfinite(decimal_year) && decimal_year >= kValidFromYear && decimal_year <= kValidUntilYear;
}

WmmError WorldMagneticModel::field_ned(double latitude_deg,
                                       double longitude_deg,
                                       double altitude_m,
                                       double decimal_year,
                                       WmmField& out)
{
  if (!covers(decimal_year))
  {
    return WmmError::kDateOutOfRange;
  }
  if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg) || !std::isfinite(altitude_m) ||
      latitude_deg < -90.0 || latitude_deg > 90.0)
  {
    return WmmError::kInvalidPosition;
  }

  const double latitude_rad = latitude_deg * kDegToRad;
  const double longitude_rad = longitude_deg * kDegToRad;

  // --- geodetic to geocentric, on the WGS-84 ellipsoid --------------------
  const double f = 1.0 / kInverseFlattening;
  const double e2 = f * (2.0 - f);
  const double sin_lat = std::sin(latitude_rad);
  const double cos_lat = std::cos(latitude_rad);
  // Radius of curvature in the prime vertical.
  const double rc = kSemiMajorM / std::sqrt(1.0 - (e2 * sin_lat * sin_lat));
  const double p_axis = (rc + altitude_m) * cos_lat;
  const double z_axis = ((rc * (1.0 - e2)) + altitude_m) * sin_lat;
  const double radius_m = std::sqrt((p_axis * p_axis) + (z_axis * z_axis));
  const double geocentric_lat_rad = std::asin(z_axis / radius_m);

  // The expansion is in colatitude.
  const double cos_colat = std::sin(geocentric_lat_rad);
  const double sin_colat = std::cos(geocentric_lat_rad);

  Table p{};
  Table dp{};
  legendre(sin_colat, cos_colat, p, dp);

  // --- sines and cosines of m * longitude, by recurrence -------------------
  std::array<double, kRows> cos_m{};
  std::array<double, kRows> sin_m{};
  cos_m[0] = 1.0;
  sin_m[0] = 0.0;
  const double cos_lon = std::cos(longitude_rad);
  const double sin_lon = std::sin(longitude_rad);
  for (std::size_t m = 1; m <= kDegree; ++m)
  {
    cos_m[m] = (cos_m[m - 1] * cos_lon) - (sin_m[m - 1] * sin_lon);
    sin_m[m] = (sin_m[m - 1] * cos_lon) + (cos_m[m - 1] * sin_lon);
  }

  // --- the expansion -------------------------------------------------------
  const double years = decimal_year - wmm::kEpochYear;
  const double ratio = kGeomagneticReferenceRadiusM / radius_m;

  // Geocentric spherical components: X' north, Y' east, Z' down.
  double bx = 0.0;
  double by = 0.0;
  double bz = 0.0;

  // (a/r)^(n+2), built up as the degrees are walked rather than by pow().
  double ratio_pow = ratio * ratio;

  std::size_t index = 0;
  for (std::size_t n = 1; n <= kDegree; ++n)
  {
    ratio_pow *= ratio;
    for (std::size_t m = 0; m <= n; ++m)
    {
      const wmm::Coefficient& c = wmm::kCoefficients[index];
      ++index;
      // Secular variation is linear in time from the epoch, which is all the
      // model claims and the reason it expires.
      const double g = c.g + (years * c.g_dot);
      const double h = c.h + (years * c.h_dot);

      const double gm_cos = (g * cos_m[m]) + (h * sin_m[m]);
      const double gm_sin = (g * sin_m[m]) - (h * cos_m[m]);

      // Sign convention: dP/dtheta here increases toward the south pole, so the
      // northward component takes the positive sign. Fixed against NOAA's
      // equatorial reference points, where the geodetic rotation vanishes and
      // the error was an exact negation.
      bx += ratio_pow * gm_cos * dp[n][m];
      by += ratio_pow * static_cast<double>(m) * gm_sin * p[n][m];
      bz += -ratio_pow * (static_cast<double>(n) + 1.0) * gm_cos * p[n][m];
    }
  }

  // Y' carries a 1/sin(colatitude) that the m factor above left out. At the
  // geographic poles that is singular, and the field there is not: the limit
  // is finite and the standard treatment is to evaluate just off the pole.
  if (std::fabs(sin_colat) > 1e-10)
  {
    by /= sin_colat;
  }
  else
  {
    // Within a hair of the pole the east component is dominated by the m = 1
    // terms; recomputing the limit properly is not worth it for a case no
    // vehicle in these examples reaches, so refuse rather than return a
    // number that looks fine.
    return WmmError::kInvalidPosition;
  }

  // --- geocentric back to geodetic ----------------------------------------
  // The two frames differ by the angle between the geodetic and geocentric
  // verticals, which is zero at the equator and poles and about 0.19 degrees
  // at mid latitudes. Ignoring it is the difference between a model good to
  // tens of nanotesla and one good to hundreds.
  const double delta = geocentric_lat_rad - latitude_rad;
  const double cos_delta = std::cos(delta);
  const double sin_delta = std::sin(delta);

  out.north_nt = (bx * cos_delta) - (bz * sin_delta);
  out.east_nt = by;
  out.down_nt = (bx * sin_delta) + (bz * cos_delta);
  return WmmError::kNone;
}

}  // namespace hemerion::sensors::mag
