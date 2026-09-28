// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_world_magnetic_model.cpp
//
// The World Magnetic Model (Hemerion/mag/world_magnetic_model.h), checked
// against NOAA's own reference values rather than against itself.
//
// This is the whole reason vendor/wmm/WMM2025_TestValues.txt is in the tree.
// A spherical-harmonic expansion is the kind of code that runs, produces
// plausible numbers, and is wrong: the first version of this implementation
// agreed with itself perfectly and was out by tens of thousands of nanotesla,
// because it applied the Schmidt normalisation after a recurrence written for
// unnormalised functions. Nothing short of the model's authors' own outputs
// would have caught that -- and the second bug it caught, a sign on the
// northward component, showed up as an *exact* negation at NOAA's equatorial
// points, where the geodetic rotation vanishes and nothing else could mask it.
//
// Plain asserts + exit code, matching the other sensors tests.
// ------------------------------------------------------------------------------
#include <cstdlib>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <numbers>
#include <sstream>
#include <string>

#include "Hemerion/mag/world_magnetic_model.h"

using hemerion::sensors::mag::to_body;
using hemerion::sensors::mag::WmmError;
using hemerion::sensors::mag::WmmField;
using hemerion::sensors::mag::WmmFieldBody;
using hemerion::sensors::mag::WorldMagneticModel;

#define CHECK(condition) check((condition), #condition, __FILE__, __LINE__)

namespace
{

// assert() is compiled out under NDEBUG, and these files are compiled (though
// not run) by the Release SWIL job. See test_sensor_seeding.cpp.
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

/// Smallest signed difference between two angles [degrees].
double angle_difference_deg(double a, double b)
{
  double d = a - b;
  while (d > 180.0)
  {
    d -= 360.0;
  }
  while (d < -180.0)
  {
    d += 360.0;
  }
  return d;
}

/// Every one of NOAA's reference points, to the precision they publish.
///
/// The file gives X, Y, Z and F to six decimal places and declination and
/// inclination to two, so the components are checked to a hundredth of a
/// nanotesla and the angles to half of NOAA's own last digit. Those are not
/// generous tolerances chosen to make this pass -- they are what the
/// published precision permits, and the implementation meets them with room
/// to spare (worst component error 0.0007 nT over the whole set).
void test_against_noaa_reference_values()
{
  std::ifstream file(HEMERION_WMM_TEST_VALUES);
  CHECK(file.is_open());

  int points = 0;
  double worst_component_nt = 0.0;
  double worst_angle_deg = 0.0;

  std::string line;
  while (std::getline(file, line))
  {
    if (line.empty() || line[0] == '#')
    {
      continue;
    }
    std::istringstream in(line);
    double year = 0.0;
    double altitude_km = 0.0;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double declination_deg = 0.0;
    double inclination_deg = 0.0;
    double horizontal_nt = 0.0;
    double x_nt = 0.0;
    double y_nt = 0.0;
    double z_nt = 0.0;
    double f_nt = 0.0;
    if (!(in >> year >> altitude_km >> latitude_deg >> longitude_deg >> declination_deg >> inclination_deg >>
          horizontal_nt >> x_nt >> y_nt >> z_nt >> f_nt))
    {
      continue;
    }

    WmmField field;
    CHECK(WorldMagneticModel::field_ned(latitude_deg, longitude_deg, altitude_km * 1000.0, year, field) ==
          WmmError::kNone);

    worst_component_nt = std::max(worst_component_nt, std::fabs(field.north_nt - x_nt));
    worst_component_nt = std::max(worst_component_nt, std::fabs(field.east_nt - y_nt));
    worst_component_nt = std::max(worst_component_nt, std::fabs(field.down_nt - z_nt));
    worst_component_nt = std::max(worst_component_nt, std::fabs(field.intensity_nt() - f_nt));
    worst_component_nt = std::max(worst_component_nt, std::fabs(field.horizontal_nt() - horizontal_nt));

    worst_angle_deg =
        std::max(worst_angle_deg, std::fabs(angle_difference_deg(field.declination_deg(), declination_deg)));
    worst_angle_deg = std::max(worst_angle_deg, std::fabs(field.inclination_deg() - inclination_deg));
    ++points;
  }

  // A test that silently read nothing is the vacuous green this repo has been
  // bitten by before: NOAA ships exactly 100 points and all of them must have
  // been checked.
  CHECK(points == 100);
  CHECK(worst_component_nt < 0.01);
  CHECK(worst_angle_deg < 0.006);
}

/// The model expires, and says so rather than extrapolating.
///
/// The secular variation is a straight line fitted over five years. Running it
/// past that is not a small error, and the failure it guards against -- a
/// vehicle flying on a silently stale field model -- is the kind that shows up
/// as a heading that is merely a bit wrong.
void test_validity_window_is_enforced()
{
  CHECK(!WorldMagneticModel::covers(2024.999));
  CHECK(WorldMagneticModel::covers(WorldMagneticModel::kValidFromYear));
  CHECK(WorldMagneticModel::covers(WorldMagneticModel::kValidUntilYear));
  CHECK(!WorldMagneticModel::covers(WorldMagneticModel::kValidUntilYear + 0.001));
  CHECK(!WorldMagneticModel::covers(std::nan("")));

  WmmField field;
  CHECK(WorldMagneticModel::field_ned(36.0, -75.7, 0.0, 2031.0, field) == WmmError::kDateOutOfRange);
  CHECK(WorldMagneticModel::field_ned(36.0, -75.7, 0.0, 2020.0, field) == WmmError::kDateOutOfRange);

  // And the result is untouched on refusal, so a caller ignoring the return
  // value gets a zero field rather than a stale or partial one.
  CHECK(field.north_nt == 0.0 && field.east_nt == 0.0 && field.down_nt == 0.0);
}

/// Positions the expansion cannot evaluate are refused, not approximated.
void test_position_guards()
{
  WmmField field;
  CHECK(WorldMagneticModel::field_ned(91.0, 0.0, 0.0, 2026.0, field) == WmmError::kInvalidPosition);
  CHECK(WorldMagneticModel::field_ned(-91.0, 0.0, 0.0, 2026.0, field) == WmmError::kInvalidPosition);
  CHECK(WorldMagneticModel::field_ned(std::nan(""), 0.0, 0.0, 2026.0, field) == WmmError::kInvalidPosition);
  CHECK(WorldMagneticModel::field_ned(36.0, std::nan(""), 0.0, 2026.0, field) == WmmError::kInvalidPosition);

  // The east component carries a 1/sin(colatitude) that is singular at the
  // geographic poles. One degree away is fine -- NOAA's own reference set goes
  // to 89 -- but exactly 90 is refused rather than returned as a number that
  // looks reasonable.
  CHECK(WorldMagneticModel::field_ned(90.0, 0.0, 0.0, 2026.0, field) == WmmError::kInvalidPosition);
  CHECK(WorldMagneticModel::field_ned(89.0, 0.0, 0.0, 2026.0, field) == WmmError::kNone);
}

/// The number this whole item was for.
///
/// The examples flew a centred dipole, which has almost no declination
/// structure and put Kitty Hawk at +0.69 degrees. Three doc passages said the
/// real field is "about -11" and nothing in the tree could produce it. Now
/// something can, and it agrees with those passages -- which is an independent
/// check on the implementation, since that figure was written down long before
/// this code existed.
void test_declination_where_the_examples_fly()
{
  WmmField kitty_hawk;
  CHECK(WorldMagneticModel::field_ned(36.0, -75.7, 3052.0, 2026.75, kitty_hawk) == WmmError::kNone);
  CHECK(near(kitty_hawk.declination_deg(), -10.985, 0.05));
  CHECK(near(kitty_hawk.inclination_deg(), 62.05, 0.1));
  CHECK(near(kitty_hawk.intensity_nt() * 1e-3, 48.35, 0.1));  // uT, the sensor stack's unit

  // The dipole's answer, for contrast, is nowhere near: this is the error the
  // replacement removes, not a refinement of it.
  CHECK(std::fabs(kitty_hawk.declination_deg() - 0.69) > 10.0);

  // The rocket's pad sits where a centred dipole is worst -- the South
  // Atlantic Anomaly region -- and the difference there is not subtle either.
  WmmField pad;
  CHECK(WorldMagneticModel::field_ned(0.0, 0.0, 0.0, 2026.75, pad) == WmmError::kNone);
  CHECK(near(pad.declination_deg(), -3.80, 0.05));
  CHECK(pad.down_nt < -15000.0);  // the dipole had this near zero and positive
}

/// Altitude enters through the ellipsoid and the (a/r)^(n+2) falloff, so a
/// climb must weaken the field monotonically at a fixed ground position.
void test_altitude_weakens_the_field()
{
  double previous = 1e30;
  for (const double altitude_m : { 0.0, 10000.0, 50000.0, 120000.0, 236000.0 })
  {
    WmmField field;
    CHECK(WorldMagneticModel::field_ned(0.0, 0.0, altitude_m, 2026.75, field) == WmmError::kNone);
    CHECK(field.intensity_nt() < previous);
    previous = field.intensity_nt();
  }
  CHECK(previous > 20000.0);  // still a real field at the top, not a rounding artefact
}

/// to_body is a rotation, and the sign conventions a heading depends on.
///
/// These assertions came from the dipole's test, which was deleted when the
/// examples stopped using it -- and the rotation moved here without them, which
/// is the kind of gap that leaves a function every example depends on with no
/// test at all. A wrong rotation preserves the field's *magnitude*, so the one
/// plot that would seem to check it does not.
void test_to_body_is_a_rotation()
{
  WmmField field;
  CHECK(WorldMagneticModel::field_ned(36.0, -75.7, 3052.0, 2026.0, field) == WmmError::kNone);

  // Level and pointing north: body axes are the geodetic ones.
  const WmmFieldBody level = to_body(field, 0.0, 0.0, 0.0);
  CHECK(near(level.x_nt, field.north_nt, 1e-9));
  CHECK(near(level.y_nt, field.east_nt, 1e-9));
  CHECK(near(level.z_nt, field.down_nt, 1e-9));

  // Length is preserved whatever the attitude -- that is what makes it a
  // rotation rather than a transformation that happens to look like one.
  const double expected = field.intensity_nt();
  for (const double yaw : { 0.3, 1.9, -2.7 })
  {
    for (const double pitch : { 0.0, 0.4, -0.5 })
    {
      for (const double roll : { 0.0, 1.1, -0.8 })
      {
        const WmmFieldBody body = to_body(field, yaw, pitch, roll);
        const double length = std::sqrt((body.x_nt * body.x_nt) + (body.y_nt * body.y_nt) + (body.z_nt * body.z_nt));
        CHECK(near(length, expected, 1e-6));
      }
    }
  }

  // A 90-degree yaw takes a northward field onto body -Y. This is the sign
  // convention a heading estimate rests on, and getting it backwards puts a
  // heading 180 degrees out while leaving every magnitude untouched.
  WmmField north_only;
  north_only.north_nt = 20000.0;
  const WmmFieldBody yawed = to_body(north_only, std::numbers::pi / 2.0, 0.0, 0.0);
  CHECK(near(yawed.x_nt, 0.0, 1e-9));
  CHECK(near(yawed.y_nt, -20000.0, 1e-9));
  CHECK(near(yawed.z_nt, 0.0, 1e-9));

  // Pitch and roll act on the axes they should. Nose up 90 degrees takes body
  // Z from pointing down to pointing north, so a northward field lands on
  // +Z; rolling right 90 degrees takes body Y from east to down, so a
  // downward field lands on +Y. (The first expectation here was written as
  // -Z and the test said otherwise, which is the test doing its job: the
  // sign is not guessable from the name of the axis.)
  const WmmFieldBody pitched = to_body(north_only, 0.0, std::numbers::pi / 2.0, 0.0);
  CHECK(near(pitched.z_nt, 20000.0, 1e-9));
  CHECK(near(pitched.x_nt, 0.0, 1e-9));
  WmmField down_only;
  down_only.down_nt = 20000.0;
  const WmmFieldBody rolled = to_body(down_only, 0.0, 0.0, std::numbers::pi / 2.0);
  CHECK(near(rolled.y_nt, 20000.0, 1e-9));

  // And the unit conversion the hosts write through.
  const std::array<double, 3> microtesla = level.to_microtesla();
  CHECK(near(microtesla[0], field.north_nt * 1e-3, 1e-12));
  CHECK(near(microtesla[1], field.east_nt * 1e-3, 1e-12));
  CHECK(near(microtesla[2], field.down_nt * 1e-3, 1e-12));

  const std::array<double, 3> ned_microtesla = field.to_microtesla();
  CHECK(near(ned_microtesla[2], field.down_nt * 1e-3, 1e-12));
}

}  // namespace

int main()
{
  test_against_noaa_reference_values();
  test_validity_window_is_enforced();
  test_position_guards();
  test_declination_where_the_examples_fly();
  test_altitude_weakens_the_field();
  test_to_body_is_a_rotation();

  std::puts("test_world_magnetic_model: all checks passed");
  return 0;
}
