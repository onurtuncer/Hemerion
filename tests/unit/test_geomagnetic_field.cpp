// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_geomagnetic_field.cpp
//
// The truth magnetic field the examples drive their magnetometer FMU with
// (examples/common/geomagnetic_field.hpp).
//
// Why now: the model was duplicated between two examples, with a third
// reaching across an include path into one of the copies, so there was no one
// place to test. Since 2026-09-27 there is.
//
// What these assert is chosen with the WMM/IGRF replacement in mind. That
// change swaps the single dipole term for a spherical-harmonic sum, and every
// property below except the last should survive it unchanged -- a dipole is
// the degree-1 term of exactly that expansion. The declination check is the
// opposite: it pins the number the documentation currently quotes as *wrong*,
// so that when the real coefficients land the test fails and says so rather
// than letting the docs quietly keep a stale figure.
//
// Plain asserts + exit code, matching the sensors tests.
// ------------------------------------------------------------------------------
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <numbers>

#include "geomagnetic_field.hpp"

using hemerion::examples::FieldBody;
using hemerion::examples::FieldNed;
using hemerion::examples::GeomagneticDipole;

#define CHECK(condition) check((condition), #condition, __FILE__, __LINE__)

namespace
{

// assert() is compiled out under NDEBUG, and these files are compiled (though
// not run) by the Release SWIL job -- where every check below would vanish,
// taking the variables feeding it with it. See test_example_environment.cpp.
[[noreturn]] void check_failed(const char* expression, const char* file, int line)
{
  std::cerr << file << ":" << line << ": check failed: " << expression << "\n";
  std::abort();
}

void check(bool condition, const char* expression, const char* file, int line)
{
  if (!condition)
  {
    check_failed(expression, file, line);
  }
}

bool near(double value, double expected, double tolerance) { return std::fabs(value - expected) <= tolerance; }

constexpr double kRadToDeg = 180.0 / std::numbers::pi;

/// Declination: the angle from true north to the horizontal field.
double declination_deg(const FieldNed& field) { return std::atan2(field.east_ut, field.north_ut) * kRadToDeg; }

/// Inclination: the dip angle, positive downward.
double inclination_deg(const FieldNed& field)
{
  return std::atan2(field.down_ut, std::hypot(field.north_ut, field.east_ut)) * kRadToDeg;
}

// --------------------------------------------------------------------------
// Properties a spherical-harmonic model must keep
// --------------------------------------------------------------------------

/// A dipole's intensity doubles from magnetic equator to magnetic pole.
///
/// |B| = B0 sqrt(1 + 3 sin^2(magnetic latitude)), so the ratio is exactly 2.
/// Sampled at the geomagnetic pole itself rather than at a geographic one,
/// because the axis is tilted ~11 degrees and testing the geographic pole
/// would be testing the tilt instead.
void test_intensity_doubles_from_equator_to_pole()
{
  const FieldNed pole =
      GeomagneticDipole::field_ned(GeomagneticDipole::kPoleLatitudeDeg, GeomagneticDipole::kPoleLongitudeDeg, 0.0);
  // The magnetic equator under the tilted axis: 90 degrees away from the pole.
  const FieldNed equator = GeomagneticDipole::field_ned(
      GeomagneticDipole::kPoleLatitudeDeg - 90.0, GeomagneticDipole::kPoleLongitudeDeg, 0.0);

  const double pole_ut = GeomagneticDipole::magnitude(pole);
  const double equator_ut = GeomagneticDipole::magnitude(equator);
  CHECK(near(equator_ut, GeomagneticDipole::kEquatorialFieldUt, 1e-9));
  CHECK(near(pole_ut / equator_ut, 2.0, 1e-9));

  // And the dip goes from horizontal at the magnetic equator to vertical at
  // the pole, which is the property that makes a magnetometer a usable
  // heading reference at mid latitudes and a poor one near the pole.
  CHECK(near(inclination_deg(equator), 0.0, 1e-6));
  CHECK(near(inclination_deg(pole), 90.0, 1e-6));
}

/// The field falls off as 1/r^3, which is what makes the rocket's decoded
/// stream show the magnitude changing at all.
void test_inverse_cube_falloff()
{
  const double altitude_m = 236000.0;  // top of the rocket example's window
  const FieldNed surface = GeomagneticDipole::field_ned(0.0, 0.0, 0.0);
  const FieldNed high = GeomagneticDipole::field_ned(0.0, 0.0, altitude_m);

  const double a = GeomagneticDipole::kReferenceRadiusM;
  const double expected = std::pow(a / (a + altitude_m), 3.0);
  CHECK(near(expected, 0.897, 0.001));  // the factor the header quotes
  CHECK(near(GeomagneticDipole::magnitude(high) / GeomagneticDipole::magnitude(surface), expected, 1e-9));
}

/// The field points *down* in the northern magnetic hemisphere and *up* in
/// the southern. Getting this backwards is the classic sign error in a field
/// model, and it would put a heading algorithm 180 degrees out.
void test_dip_sign_follows_hemisphere()
{
  CHECK(GeomagneticDipole::field_ned(60.0, -75.0, 0.0).down_ut > 0.0);
  CHECK(GeomagneticDipole::field_ned(-60.0, -75.0, 0.0).down_ut < 0.0);
}

/// to_body is a rotation: it preserves length, and leaves a field alone when
/// the airframe is level and pointing north.
void test_to_body_is_a_rotation()
{
  const FieldNed field = GeomagneticDipole::field_ned(36.0, -75.7, 3000.0);

  const FieldBody level = GeomagneticDipole::to_body(field, 0.0, 0.0, 0.0);
  CHECK(near(level.x_ut, field.north_ut, 1e-12));
  CHECK(near(level.y_ut, field.east_ut, 1e-12));
  CHECK(near(level.z_ut, field.down_ut, 1e-12));

  // Length is preserved whatever the attitude.
  const double expected = GeomagneticDipole::magnitude(field);
  for (const double yaw : { 0.3, 1.9, -2.7 })
  {
    for (const double pitch : { 0.0, 0.4, -0.5 })
    {
      for (const double roll : { 0.0, 1.1, -0.8 })
      {
        const FieldBody body = GeomagneticDipole::to_body(field, yaw, pitch, roll);
        const double length = std::sqrt((body.x_ut * body.x_ut) + (body.y_ut * body.y_ut) + (body.z_ut * body.z_ut));
        CHECK(near(length, expected, 1e-9));
      }
    }
  }

  // A 90-degree yaw swaps north into -y and east into +x, which is the
  // sign convention a heading estimate depends on.
  const FieldBody yawed = GeomagneticDipole::to_body(FieldNed{ 20.0, 0.0, 0.0 }, std::numbers::pi / 2.0, 0.0, 0.0);
  CHECK(near(yawed.x_ut, 0.0, 1e-12));
  CHECK(near(yawed.y_ut, -20.0, 1e-12));
}

/// A plant fault must not become a NaN riding quietly into the sensor stream.
void test_centre_of_the_earth_is_guarded()
{
  const FieldNed field = GeomagneticDipole::field_ned(45.0, 10.0, -GeomagneticDipole::kReferenceRadiusM);
  CHECK(field.north_ut == 0.0 && field.east_ut == 0.0 && field.down_ut == 0.0);
  CHECK(!std::isnan(GeomagneticDipole::magnitude(field)));
}

// --------------------------------------------------------------------------
// The property that is *meant* to change
// --------------------------------------------------------------------------

/// Declination at Kitty Hawk, which the model gets wrong on purpose.
///
/// A centred dipole carries no declination structure, so it produces about
/// +0.69 degrees here where the real field's is about -11. The documentation
/// says so in three places. This pins the figure so that swapping in the WMM
/// or IGRF coefficients fails *this* test loudly, rather than leaving those
/// three passages quietly stale -- at which point the assertion should be
/// replaced by one against the real value, not merely widened.
void test_dipole_declination_is_the_documented_wrong_one()
{
  const FieldNed kitty_hawk = GeomagneticDipole::field_ned(36.0, -75.7, 3052.0);
  CHECK(near(declination_deg(kitty_hawk), 0.69, 0.05));

  // Inclination it gets about right, which is the half of the story that
  // makes the model usable as *a* plausible field: mid-latitude North America
  // is one of the regions a centred dipole describes reasonably.
  CHECK(near(inclination_deg(kitty_hawk), 65.0, 5.0));
}

}  // namespace

int main()
{
  test_intensity_doubles_from_equator_to_pole();
  test_inverse_cube_falloff();
  test_dip_sign_follows_hemisphere();
  test_to_body_is_a_rotation();
  test_centre_of_the_earth_is_guarded();
  test_dipole_declination_is_the_documented_wrong_one();

  std::puts("test_geomagnetic_field: all checks passed");
  return 0;
}
