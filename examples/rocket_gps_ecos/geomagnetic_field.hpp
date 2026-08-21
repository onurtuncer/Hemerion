// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file geomagnetic_field.hpp
/// @brief The truth magnetic field the magnetometer FMU is driven with: a
/// centered tilted dipole, plus the NED-to-body rotation.
///
/// The rocket plant reports where it is (`out.lat_rad`, `out.lon_rad`,
/// `out.alt_m`) and how it is pointing (`out.yaw_rad`, `out.pitch_rad`,
/// `out.roll_rad`) but says nothing about the magnetic field it is flying
/// through -- there is no `out.b_*` to connect. So the host computes it, the
/// same way and for the same reason it computes specific force: an Ecos
/// connection modifier sees one source variable, and this needs four.
///
/// **The model is a centered tilted dipole, not the WMM.** Earth's field to
/// first order is a bar magnet whose axis is tilted ~11 degrees from the
/// spin axis:
///
///     B(r) = B0 (a/r)^3 [ 3 r_hat (m_hat . r_hat) - m_hat ]
///
/// with `a` the IGRF reference radius, `B0` the mean equatorial surface
/// field, and `m_hat` the dipole moment direction -- pointing geographic
/// *south*, which is why a compass needle points north. That reproduces the
/// two things this example actually needs: the field roughly doubles from
/// equator to pole, and it falls off as 1/r^3 -- 236 km of altitude is a
/// factor (a/(a+h))^3 = 0.897, so the ~31 uT on the pad becomes ~28 uT at the
/// top of this trajectory. Flying 2000 km downrange moves the vehicle
/// relative to the tilted dipole axis as well, which is the larger effect
/// here; both are visible in the decoded stream.
///
/// **What it costs.** A centered dipole is not a geographic reference. It
/// gets total intensity within roughly 10% over most of the globe but the
/// *inclination* can be off by tens of degrees where the real field departs
/// most from a dipole -- and Scenario 17's pad, on the equator at the prime
/// meridian, sits at the edge of exactly that region (the South Atlantic
/// Anomaly). At 0N/0E this model gives about +3 uT of down-component where
/// the real field has roughly -11 uT, i.e. it points the field into the
/// ground where reality points it out. Treat what comes out of here as *a*
/// plausible field, not as a prediction for that spot.
///
/// That is harmless for what this example does: the field is the simulation's
/// own truth, the flight computer has no independent reference to disagree
/// with it, and every byte between the two is exercised identically either
/// way. It would *not* be harmless for testing a heading algorithm against a
/// real site, which needs the WMM or IGRF coefficient set in place of the
/// single dipole term below.
///
/// Geometry is spherical throughout (geodetic latitude used as geocentric,
/// radius as `a + altitude`). The ellipsoidal correction is a few tenths of a
/// percent; the dipole approximation above dwarfs it by two orders of
/// magnitude, so carrying WGS-84 here would be false precision.

#pragma once

#include <cmath>
#include <numbers>
#include <tuple>

namespace hemerion::examples::rocket_gps_ecos
{

/// A magnetic field in the local geographic frame [microtesla].
struct FieldNed
{
  double north_ut = 0.0;
  double east_ut = 0.0;
  double down_ut = 0.0;
};

/// A magnetic field in body axes [microtesla] -- what a magnetometer bolted
/// to the airframe measures, and what the MMC5983MA FMU takes as input.
struct FieldBody
{
  double x_ut = 0.0;
  double y_ut = 0.0;
  double z_ut = 0.0;
};

/// @brief Centered tilted dipole geomagnetic field; see the file comment for
/// what it does and does not model.
class GeomagneticDipole
{
public:
  /// IGRF reference radius [m].
  static constexpr double kReferenceRadiusM = 6371200.0;
  /// Mean equatorial field at the reference radius [uT]. The dipole term of
  /// recent IGRF epochs is ~30 uT; 31.2 is the classical value and puts the
  /// polar field at the ~62 uT observed.
  static constexpr double kEquatorialFieldUt = 31.2;
  /// Geomagnetic north pole, IGRF-13 epoch 2020 [degrees]. The *field* points
  /// down here, so the dipole moment points the other way -- see `moment()`.
  static constexpr double kPoleLatitudeDeg = 80.65;
  static constexpr double kPoleLongitudeDeg = -72.68;

  /// @brief The field at a geodetic position, in local NED.
  ///
  /// @param latitude_rad  Geodetic latitude, used as geocentric (see file comment).
  /// @param longitude_rad Longitude, east positive.
  /// @param altitude_m    Height above the reference sphere.
  [[nodiscard]] static FieldNed field_ned(double latitude_rad, double longitude_rad, double altitude_m)
  {
    const double radius_m = kReferenceRadiusM + altitude_m;
    // Guard the singularity at the Earth's centre rather than trusting the
    // plant never to report it: a plant fault should not become a NaN
    // quietly riding into the sensor stream.
    const double scale = (radius_m > 1.0) ? kEquatorialFieldUt * std::pow(kReferenceRadiusM / radius_m, 3.0) : 0.0;

    const double cos_lat = std::cos(latitude_rad);
    const double sin_lat = std::sin(latitude_rad);
    const double cos_lon = std::cos(longitude_rad);
    const double sin_lon = std::sin(longitude_rad);

    // Position unit vector in an Earth-fixed Cartesian frame.
    const double rx = cos_lat * cos_lon;
    const double ry = cos_lat * sin_lon;
    const double rz = sin_lat;

    const auto [mx, my, mz] = moment();
    const double m_dot_r = (mx * rx) + (my * ry) + (mz * rz);

    // B = scale * [3 r_hat (m_hat . r_hat) - m_hat], still Earth-fixed.
    const double bx = scale * ((3.0 * rx * m_dot_r) - mx);
    const double by = scale * ((3.0 * ry * m_dot_r) - my);
    const double bz = scale * ((3.0 * rz * m_dot_r) - mz);

    // Project onto the local geographic triad.
    FieldNed field;
    field.north_ut = (-sin_lat * cos_lon * bx) + (-sin_lat * sin_lon * by) + (cos_lat * bz);
    field.east_ut = (-sin_lon * bx) + (cos_lon * by);
    field.down_ut = (-cos_lat * cos_lon * bx) + (-cos_lat * sin_lon * by) + (-sin_lat * bz);
    return field;
  }

  /// @brief Rotates a NED field into body axes through a 3-2-1 (yaw, pitch,
  /// roll) Euler sequence -- the convention the rocket FMU reports its
  /// attitude in.
  [[nodiscard]] static FieldBody to_body(const FieldNed& field, double yaw_rad, double pitch_rad, double roll_rad)
  {
    const double cy = std::cos(yaw_rad);
    const double sy = std::sin(yaw_rad);
    const double cp = std::cos(pitch_rad);
    const double sp = std::sin(pitch_rad);
    const double cr = std::cos(roll_rad);
    const double sr = std::sin(roll_rad);

    // C_bn, rows = body axes expressed in NED components.
    FieldBody body;
    body.x_ut = (cp * cy * field.north_ut) + (cp * sy * field.east_ut) + (-sp * field.down_ut);
    body.y_ut = (((sr * sp * cy) - (cr * sy)) * field.north_ut) + (((sr * sp * sy) + (cr * cy)) * field.east_ut) +
                (sr * cp * field.down_ut);
    body.z_ut = (((cr * sp * cy) + (sr * sy)) * field.north_ut) + (((cr * sp * sy) - (sr * cy)) * field.east_ut) +
                (cr * cp * field.down_ut);
    return body;
  }

  /// @brief Total intensity of a NED field [uT] -- the quantity that is
  /// (nearly) rotation-invariant, so a decoded stream can be checked against
  /// truth without knowing the attitude.
  [[nodiscard]] static double magnitude(const FieldNed& field)
  {
    return std::sqrt((field.north_ut * field.north_ut) + (field.east_ut * field.east_ut) +
                     (field.down_ut * field.down_ut));
  }

private:
  /// Unit dipole moment in the Earth-fixed frame. The geomagnetic *north*
  /// pole is where field lines enter the Earth, so the moment points away
  /// from it -- toward the southern hemisphere.
  [[nodiscard]] static std::tuple<double, double, double> moment()
  {
    const double pole_lat_rad = kPoleLatitudeDeg * (std::numbers::pi / 180.0);
    const double pole_lon_rad = kPoleLongitudeDeg * (std::numbers::pi / 180.0);
    const double cos_lat = std::cos(pole_lat_rad);
    return { -cos_lat * std::cos(pole_lon_rad), -cos_lat * std::sin(pole_lon_rad), -std::sin(pole_lat_rad) };
  }
};

}  // namespace hemerion::examples::rocket_gps_ecos
