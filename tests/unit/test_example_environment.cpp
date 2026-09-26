// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_example_environment.cpp
//
// The co-simulation hosts' shared option plumbing (examples/common/
// environment.hpp): the MIL-F-8785C turbulence rules, the comma-separated
// option parser, and the two GPS receiver presets.
//
// Why this exists, and why it is here rather than under examples/: the
// examples cannot run in CI. Every one of them needs an Aetherion plant FMU,
// which CI has no install of, so the co-simulation smoke tests those
// directories register are absent there rather than skipped -- a skip scores
// as a pass in ctest, which is precisely the vacuous green this repo has been
// bitten by before. What *can* be covered everywhere is the pure logic: a
// formula out of a standard, a parser with a grammar, and two records the
// documentation quotes. Those live in a header with no FMI, no Ecos and no
// sockets, and are asserted here, in a test the `test-native` preset builds.
//
// Plain asserts + exit code, matching the sensors tests.
// ------------------------------------------------------------------------------
#include <cstdlib>
#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "environment.hpp"

using hemerion::examples::dryden_low_altitude;
using hemerion::examples::kCorrelatedReceiver;
using hemerion::examples::kFeetToMetres;
using hemerion::examples::kWhiteReceiver;
using hemerion::examples::parse_csv_doubles;
using hemerion::examples::total_sigma;

#define CHECK(condition) check((condition), #condition, __FILE__, __LINE__)

namespace
{

// assert() is compiled out under NDEBUG, and these files are compiled (though
// not run) by the Release SWIL job -- where every check below would vanish,
// taking the variables feeding it with it. A test that silently asserts
// nothing is exactly the vacuous green this repo has been bitten by before, so
// the checks here survive the optimiser rather than depending on the build
// type.
[[noreturn]] void check_failed(const char* expression, const char* file, int line)
{
  std::fprintf(stderr, "%s:%d: check failed: %s\n", file, line, expression);
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

// The standard's own formulae, at an altitude where they are the standard
// rather than an extrapolation of it. At 500 ft with W20 = 15 m/s (moderate):
//
//   sigma_w = 0.1 W20                         = 1.5 m/s
//   sigma_u = sigma_w / (h/1000)^0.4          = 1.5 / 0.5^0.4 = 1.9793 m/s
//   L_w     = h                               = 500 ft
//   L_u     = h / (0.177 + 0.000823 h)^1.2    = 500 / 0.5885^1.2 ft
void test_dryden_matches_the_standard()
{
  const double h_ft = 500.0;
  const auto dryden = dryden_low_altitude(h_ft * kFeetToMetres, 15.0);

  CHECK(near(dryden.sigma_w_mps, 1.5, 1e-9));
  const double expected_sigma_uv = 1.5 / std::pow(0.5, 0.4);
  CHECK(near(dryden.sigma_u_mps, expected_sigma_uv, 1e-9));
  CHECK(near(dryden.sigma_u_mps, 1.97926, 1e-4));  // the arithmetic, spelled out
  CHECK(dryden.sigma_v_mps == dryden.sigma_u_mps);

  CHECK(near(dryden.L_w_m, h_ft * kFeetToMetres, 1e-9));
  const double expected_l_uv_ft = h_ft / std::pow(0.177 + 0.000823 * h_ft, 1.2);
  CHECK(near(dryden.L_u_m, expected_l_uv_ft * kFeetToMetres, 1e-9));
  CHECK(dryden.L_v_m == dryden.L_u_m);

  // The horizontal intensity exceeds the vertical below 1000 ft and falls
  // below it above -- the crossover the (h/1000)^0.4 factor exists to produce.
  CHECK(dryden_low_altitude(500.0 * kFeetToMetres, 15.0).sigma_u_mps > 1.5);
  CHECK(near(dryden_low_altitude(1000.0 * kFeetToMetres, 15.0).sigma_u_mps, 1.5, 1e-9));
  CHECK(dryden_low_altitude(2000.0 * kFeetToMetres, 15.0).sigma_u_mps < 1.5);
}

// Intensity is linear in W20 and zero turbulence is zero; altitude floors at
// 10 ft, below which the scale-length formula stops meaning anything.
void test_dryden_scaling_and_floor()
{
  const auto light = dryden_low_altitude(1000.0, 5.0);
  const auto moderate = dryden_low_altitude(1000.0, 15.0);
  CHECK(near(moderate.sigma_w_mps / light.sigma_w_mps, 3.0, 1e-9));
  CHECK(near(moderate.sigma_u_mps / light.sigma_u_mps, 3.0, 1e-9));
  CHECK(near(moderate.L_u_m, light.L_u_m, 1e-9));  // scale lengths do not depend on W20

  const auto calm = dryden_low_altitude(1000.0, 0.0);
  CHECK(calm.sigma_u_mps == 0.0 && calm.sigma_v_mps == 0.0 && calm.sigma_w_mps == 0.0);

  // Below the floor every altitude gives the same answer, and it is finite.
  const auto ground = dryden_low_altitude(0.0, 10.0);
  const auto below = dryden_low_altitude(-50.0, 10.0);
  CHECK(ground.L_w_m == below.L_w_m);
  CHECK(std::isfinite(ground.sigma_u_mps) && ground.L_u_m > 0.0);

  // Scale lengths grow with altitude; intensity does not depend on it
  // vertically, and falls with it horizontally.
  CHECK(dryden_low_altitude(3000.0, 10.0).L_w_m > dryden_low_altitude(300.0, 10.0).L_w_m);
  CHECK(dryden_low_altitude(3000.0, 10.0).sigma_u_mps < dryden_low_altitude(300.0, 10.0).sigma_u_mps);
  CHECK(near(dryden_low_altitude(3000.0, 10.0).sigma_w_mps, dryden_low_altitude(300.0, 10.0).sigma_w_mps, 1e-12));
}

void test_parser_accepts_its_grammar()
{
  double three[3] = { 0.0, 0.0, 0.0 };
  parse_csv_doubles("10,0,-2.5", three, "--wind", "north,east,down");
  CHECK(three[0] == 10.0 && three[1] == 0.0 && three[2] == -2.5);

  double two[2] = { 0.0, 0.0 };
  parse_csv_doubles("20,-1000", two, "--atmosphere", "deltaT_K,deltaP_sl_Pa");
  CHECK(two[0] == 20.0 && two[1] == -1000.0);

  // Scientific notation and leading signs are std::stod's business, and work.
  parse_csv_doubles("1e3,+2.5", two, "--atmosphere", "deltaT_K,deltaP_sl_Pa");
  CHECK(two[0] == 1000.0 && two[1] == 2.5);
}

void test_parser_rejects_everything_else()
{
  const auto rejects = [](const char* text, std::size_t fields) {
    double buffer[3] = { 0.0, 0.0, 0.0 };
    try
    {
      parse_csv_doubles(text, std::span<double>(buffer, fields), "--wind", "north,east,down");
    }
    catch (const std::invalid_argument&)
    {
      return true;
    }
    return false;
  };

  CHECK(rejects("10,0", 3));         // too few
  CHECK(rejects("10,0,1,2", 3));     // too many
  CHECK(rejects("10,,1", 3));        // empty field
  CHECK(rejects("", 3));             // empty value
  CHECK(rejects("10,0,abc", 3));     // not a number
  CHECK(rejects("10,0,1.0abc", 3));  // trailing rubbish, which stod alone would accept
  CHECK(rejects("10,0,", 3));        // trailing separator

  // The message names the option and its grammar, so the user is told what to
  // type rather than which C function failed.
  double buffer[2] = { 0.0, 0.0 };
  try
  {
    parse_csv_doubles("nope", buffer, "--atmosphere", "deltaT_K,deltaP_sl_Pa");
    CHECK(false);
  }
  catch (const std::invalid_argument& ex)
  {
    const std::string message(ex.what());
    CHECK(message.find("--atmosphere") != std::string::npos);
    CHECK(message.find("deltaT_K") != std::string::npos);
  }
}

// The two presets are a claim the documentation quotes: the realistic receiver
// keeps every channel's total 1-sigma within 2 % of the default's, so no
// figure moves by magnitude, while moving most of the variance into the slow
// term. If either half of that stops being true, doc/sensor_models.rst is
// wrong and this is where it should be caught.
void test_gps_presets_keep_their_promise()
{
  CHECK(std::string(kWhiteReceiver.name) == "white");
  CHECK(std::string(kCorrelatedReceiver.name) == "correlated");

  // The default is white: no correlated term at all, and an honest accuracy.
  CHECK(kWhiteReceiver.horizontal_pos_correlated_m == 0.0);
  CHECK(kWhiteReceiver.vertical_pos_correlated_m == 0.0);
  CHECK(kWhiteReceiver.speed_correlated_mps == 0.0);
  CHECK(kWhiteReceiver.course_correlated_deg == 0.0);
  CHECK(kWhiteReceiver.accuracy_scale == 1.0);

  // Position and velocity are held to different bounds, because the preset
  // holds only position to 2 %: the published figures plot position, and
  // rounding the velocity channels to hit a total exactly would mean quoting
  // receiver characteristics chosen to flatter an arithmetic claim.
  struct Channel
  {
    const char* name;
    double white_default;
    double white_correlated_preset;
    double correlated;
    double total_tolerance;       // fraction of the default the total may differ by
    double min_correlated_share;  // how much of the variance must be in the slow term
  };
  const std::array<Channel, 4> channels{ {
      { "horizontal",
        kWhiteReceiver.horizontal_pos_noise_m,
        kCorrelatedReceiver.horizontal_pos_noise_m,
        kCorrelatedReceiver.horizontal_pos_correlated_m,
        0.02,
        0.95 },
      { "vertical",
        kWhiteReceiver.vertical_pos_noise_m,
        kCorrelatedReceiver.vertical_pos_noise_m,
        kCorrelatedReceiver.vertical_pos_correlated_m,
        0.02,
        0.95 },
      { "speed",
        kWhiteReceiver.speed_noise_mps,
        kCorrelatedReceiver.speed_noise_mps,
        kCorrelatedReceiver.speed_correlated_mps,
        0.13,
        0.75 },
      { "course",
        kWhiteReceiver.course_noise_deg,
        kCorrelatedReceiver.course_noise_deg,
        kCorrelatedReceiver.course_correlated_deg,
        0.05,
        0.90 },
  } };

  for (const Channel& channel : channels)
  {
    const double total = total_sigma(channel.white_correlated_preset, channel.correlated);
    CHECK(near(total, channel.white_default, channel.total_tolerance * channel.white_default));

    // And most of the variance is in the slow term, which is what makes the
    // autocorrelation figure show anything at all.
    const double correlated_share = (channel.correlated * channel.correlated) / (total * total);
    CHECK(correlated_share > channel.min_correlated_share);
  }

  // The horizontal share is the number the F-16 page's caption quotes.
  const double horizontal_total =
      total_sigma(kCorrelatedReceiver.horizontal_pos_noise_m, kCorrelatedReceiver.horizontal_pos_correlated_m);
  const double horizontal_share =
      (kCorrelatedReceiver.horizontal_pos_correlated_m * kCorrelatedReceiver.horizontal_pos_correlated_m) /
      (horizontal_total * horizontal_total);
  CHECK(near(horizontal_share, 0.96, 0.005));

  // An optimistic receiver reports less than it makes, which is the point.
  CHECK(kCorrelatedReceiver.accuracy_scale > 0.0 && kCorrelatedReceiver.accuracy_scale < 1.0);

  // Position decorrelates over minutes, velocity over seconds -- the two
  // timescales the model exists to tell apart.
  CHECK(kCorrelatedReceiver.position_correlation_time_s >= 10.0 * kCorrelatedReceiver.velocity_correlation_time_s);
}

}  // namespace

int main()
{
  test_dryden_matches_the_standard();
  test_dryden_scaling_and_floor();
  test_parser_accepts_its_grammar();
  test_parser_rejects_everything_else();
  test_gps_presets_keep_their_promise();

  std::puts("test_example_environment: all checks passed");
  return 0;
}
