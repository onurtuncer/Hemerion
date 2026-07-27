// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_gps_dynamics.cpp
//
// Receiver dynamics envelope of the GPS FMU simulator: the platform-model
// limits (altitude / horizontal velocity / vertical velocity / acceleration),
// the COCOM export cut-off, and the re-acquisition hold-off
// (modules/sensors/include/Hemerion/gps/fmu/gpsDynamicsModel.hpp). The last
// case runs the whole thing through UbxEmitter and the real, on-target
// UbxParser, so what the firmware decodes from an out-of-envelope epoch is
// checked, not just the model's internal verdict.
//
// Plain asserts + exit code, matching test_ubx_emitter.cpp -- Unity is not
// yet vendored (see that file's header comment).
// ------------------------------------------------------------------------------
#include <cassert>
#include <cstdio>

#include "Hemerion/gps/fmu/gpsDynamicsModel.hpp"
#include "Hemerion/gps/fmu/gpsNoiseModel.hpp"
#include "Hemerion/gps/fmu/ubxEmitter.hpp"
#include "Hemerion/gps/ubxParser.hpp"

using hemerion::sensors::gps::GpsFix;
using hemerion::sensors::gps::GpsFixType;
using hemerion::sensors::gps::GpsParseError;
using hemerion::sensors::gps::UbxParser;
using hemerion::sensors::gps::fmu::GpsDynamicPlatform;
using hemerion::sensors::gps::fmu::GpsDynamicsConfig;
using hemerion::sensors::gps::fmu::GpsDynamicsModel;
using hemerion::sensors::gps::fmu::GpsDynamicsVerdict;
using hemerion::sensors::gps::fmu::GpsNoiseConfig;
using hemerion::sensors::gps::fmu::GpsNoiseModel;
using hemerion::sensors::gps::fmu::GpsTruthSample;
using hemerion::sensors::gps::fmu::limits_for;
using hemerion::sensors::gps::fmu::platform_from_code;
using hemerion::sensors::gps::fmu::UbxEmitter;

namespace
{

// A valid 3D fix, as GpsNoiseModel would hand one over before gating.
GpsFix good_fix()
{
  GpsFix fix;
  fix.latitude_deg = 37.8338;
  fix.longitude_deg = -75.4879;
  fix.altitude_m = 100.0F;
  fix.horizontal_accuracy_m = 1.5F;
  fix.vertical_accuracy_m = 3.0F;
  fix.num_satellites = 11;
  fix.fix_type = GpsFixType::kFix3D;
  return fix;
}

GpsTruthSample sample(float altitude_m, float ground_speed_mps, float v_down_mps, std::uint64_t timestamp_us)
{
  GpsTruthSample truth;
  truth.latitude_deg = 37.8338;
  truth.longitude_deg = -75.4879;
  truth.altitude_m = altitude_m;
  truth.ground_speed_mps = ground_speed_mps;
  truth.course_deg = 90.0F;  // due east, so ground speed is entirely v_east
  truth.v_down_mps = v_down_mps;
  truth.timestamp_us = timestamp_us;
  return truth;
}

// Steps the model with a constant state until `until_us`, so a hold-off can
// be walked out at a realistic 10 Hz navigation rate.
GpsDynamicsVerdict coast(GpsDynamicsModel& model, GpsTruthSample truth, std::uint64_t until_us)
{
  GpsDynamicsVerdict verdict = GpsDynamicsVerdict::kValid;
  for (std::uint64_t t = truth.timestamp_us; t <= until_us; t += 100000)
  {
    truth.timestamp_us = t;
    GpsFix fix = good_fix();
    verdict = model.apply(truth, fix);
  }
  return verdict;
}

// The published u-blox dynamic platform model table, spot-checked at the
// entries that matter here: the launch-vehicle setting and the two an
// airborne application must not be left on.
void test_platform_limits_table()
{
  const auto airborne_4g = limits_for(GpsDynamicPlatform::kAirborne4g);
  assert(airborne_4g.max_altitude_m == 80000.0F);
  assert(airborne_4g.max_horizontal_speed_mps == 500.0F);
  assert(airborne_4g.max_vertical_speed_mps == 100.0F);
  assert(airborne_4g.max_acceleration_mps2 > 39.0F && airborne_4g.max_acceleration_mps2 < 39.3F);  // 4 g

  const auto portable = limits_for(GpsDynamicPlatform::kPortable);
  assert(portable.max_altitude_m == 12000.0F);
  assert(portable.max_horizontal_speed_mps == 310.0F);

  const auto automotive = limits_for(GpsDynamicPlatform::kAutomotive);
  assert(automotive.max_altitude_m == 6000.0F);
  assert(automotive.max_horizontal_speed_mps == 100.0F);

  // dynModel codes round-trip; 1 is reserved by u-blox and must be rejected.
  GpsDynamicPlatform platform = GpsDynamicPlatform::kPortable;
  assert(platform_from_code(8, platform) && platform == GpsDynamicPlatform::kAirborne4g);
  assert(platform_from_code(-1, platform) && platform == GpsDynamicPlatform::kUnlimited);
  assert(!platform_from_code(1, platform));
  assert(!platform_from_code(11, platform));
}

// Inside the envelope the model must not touch the fix at all.
void test_within_envelope_passes_through()
{
  GpsDynamicsModel model;  // airborne <4 g, COCOM on
  GpsFix fix = good_fix();
  const GpsDynamicsVerdict verdict = model.apply(sample(5000.0F, 300.0F, -50.0F, 1000000), fix);

  assert(verdict == GpsDynamicsVerdict::kValid);
  assert(fix.fix_type == GpsFixType::kFix3D);
  assert(fix.num_satellites == 11);
  assert(fix.horizontal_accuracy_m == 1.5F);
}

// Each platform-model limit taken on its own, on a receiver configured for a
// ground vehicle -- the classic "left it on the default model" failure.
void test_each_platform_limit_invalidates_the_fix()
{
  const auto check = [](const GpsTruthSample& truth, GpsDynamicsVerdict expected) {
    GpsDynamicsConfig config;
    config.platform = GpsDynamicPlatform::kAutomotive;  // 6000 m, 100 m/s, 15 m/s, ~1 g
    GpsDynamicsModel model(config);
    GpsFix fix = good_fix();
    const GpsDynamicsVerdict verdict = model.apply(truth, fix);
    assert(verdict == expected);
    assert(fix.fix_type == GpsFixType::kNoFix);
    assert(fix.num_satellites == 0);
    assert(fix.horizontal_accuracy_m > 100.0F);  // receiver admits it has nothing
    assert(fix.vertical_accuracy_m > 100.0F);
  };

  check(sample(6001.0F, 10.0F, 0.0F, 1000000), GpsDynamicsVerdict::kAltitudeLimit);
  check(sample(1000.0F, 101.0F, 0.0F, 1000000), GpsDynamicsVerdict::kHorizontalSpeedLimit);
  check(sample(1000.0F, 10.0F, -16.0F, 1000000), GpsDynamicsVerdict::kVerticalSpeedLimit);  // climbing
  check(sample(1000.0F, 10.0F, 16.0F, 1000000), GpsDynamicsVerdict::kVerticalSpeedLimit);   // descending
}

// Acceleration is differenced between consecutive epochs: a single sample can
// never trip it, and a boost harder than the model's g-class must.
void test_acceleration_limit()
{
  GpsDynamicsConfig config;
  config.platform = GpsDynamicPlatform::kAirborne4g;  // ~39.2 m/s^2
  config.reacquisition_time_s = 0.0F;                 // isolate the acceleration check
  GpsDynamicsModel model(config);

  // First epoch: no previous sample, so no acceleration estimate yet.
  GpsFix fix = good_fix();
  assert(model.apply(sample(1000.0F, 100.0F, 0.0F, 1000000), fix) == GpsDynamicsVerdict::kValid);
  assert(model.acceleration_mps2() == 0.0);

  // +30 m/s over 0.1 s = 300 m/s^2 (~30 g): well past 4 g.
  fix = good_fix();
  assert(model.apply(sample(1000.0F, 130.0F, 0.0F, 1100000), fix) == GpsDynamicsVerdict::kAccelerationLimit);
  assert(model.acceleration_mps2() > 290.0 && model.acceleration_mps2() < 310.0);
  assert(fix.fix_type == GpsFixType::kNoFix);

  // +2 m/s over 0.1 s = 20 m/s^2 (~2 g): inside 4 g, fix comes straight back
  // (the hold-off is zero in this configuration).
  fix = good_fix();
  assert(model.apply(sample(1000.0F, 132.0F, 0.0F, 1200000), fix) == GpsDynamicsVerdict::kValid);
  assert(fix.fix_type == GpsFixType::kFix3D);

  // Vertical acceleration counts the same as horizontal.
  GpsDynamicsModel vertical_model(config);
  fix = good_fix();
  (void)vertical_model.apply(sample(1000.0F, 10.0F, -50.0F, 1000000), fix);
  fix = good_fix();
  assert(vertical_model.apply(sample(1000.0F, 10.0F, -60.0F, 1100000), fix) == GpsDynamicsVerdict::kAccelerationLimit);
}

// COCOM needs altitude AND speed: either one alone leaves the fix alone, and
// an export-licensed receiver (flag cleared) keeps its fix through both.
void test_cocom_limits()
{
  GpsDynamicsConfig config;
  config.platform = GpsDynamicPlatform::kUnlimited;  // isolate COCOM from the platform envelope
  GpsDynamicsModel model(config);

  GpsFix fix = good_fix();
  assert(model.apply(sample(25000.0F, 100.0F, 0.0F, 1000000), fix) == GpsDynamicsVerdict::kValid);  // high, slow
  fix = good_fix();
  assert(model.apply(sample(1000.0F, 600.0F, 0.0F, 2000000), fix) == GpsDynamicsVerdict::kValid);  // fast, low
  fix = good_fix();
  assert(model.apply(sample(25000.0F, 600.0F, 0.0F, 3000000), fix) == GpsDynamicsVerdict::kCocomLimit);
  assert(fix.fix_type == GpsFixType::kNoFix);

  config.cocom_limits_enabled = false;  // export-licensed / unlocked receiver
  GpsDynamicsModel unlocked(config);
  fix = good_fix();
  assert(unlocked.apply(sample(25000.0F, 600.0F, 0.0F, 1000000), fix) == GpsDynamicsVerdict::kValid);
  assert(fix.fix_type == GpsFixType::kFix3D);

  // The COCOM speed threshold is on the 3-D speed: a purely vertical ascent
  // past it counts.
  GpsDynamicsModel ascending(GpsDynamicsConfig{ GpsDynamicPlatform::kUnlimited, true, 0.0F });
  fix = good_fix();
  assert(ascending.apply(sample(25000.0F, 0.0F, -600.0F, 1000000), fix) == GpsDynamicsVerdict::kCocomLimit);
}

// After the vehicle is back inside the envelope the fix stays invalid for
// reacquisition_time_s, then returns.
void test_reacquisition_hold_off()
{
  GpsDynamicsConfig config;
  config.platform = GpsDynamicPlatform::kAutomotive;
  config.reacquisition_time_s = 2.0F;
  GpsDynamicsModel model(config);

  // A climb through the model's altitude ceiling, at constant velocity so the
  // acceleration check stays out of the way.
  GpsFix fix = good_fix();
  assert(model.apply(sample(6001.0F, 50.0F, 0.0F, 10000000), fix) == GpsDynamicsVerdict::kAltitudeLimit);

  // Back in the envelope 0.1 s later: still re-acquiring, and still no fix.
  fix = good_fix();
  assert(model.apply(sample(5000.0F, 50.0F, 0.0F, 10100000), fix) == GpsDynamicsVerdict::kReacquiring);
  assert(fix.fix_type == GpsFixType::kNoFix);

  // Just before the hold-off expires: still nothing.
  assert(coast(model, sample(5000.0F, 50.0F, 0.0F, 10200000), 11900000) == GpsDynamicsVerdict::kReacquiring);

  // Past it: the fix comes back.
  fix = good_fix();
  assert(model.apply(sample(5000.0F, 50.0F, 0.0F, 12100000), fix) == GpsDynamicsVerdict::kValid);
  assert(fix.fix_type == GpsFixType::kFix3D);

  // Re-configuring clears the derived state, hold-off included.
  fix = good_fix();
  assert(model.apply(sample(6001.0F, 50.0F, 0.0F, 13000000), fix) == GpsDynamicsVerdict::kAltitudeLimit);
  model.set_config(config);
  fix = good_fix();
  assert(model.apply(sample(5000.0F, 50.0F, 0.0F, 13100000), fix) == GpsDynamicsVerdict::kValid);
}

// What the firmware actually sees: an out-of-envelope epoch, encoded and fed
// through the real UbxParser, must decode as a no-fix NAV-PVT with zero
// satellites -- i.e. the flight software can tell the difference without
// knowing anything about this model.
void test_invalidated_fix_decodes_as_no_fix()
{
  GpsTruthSample truth = sample(30000.0F, 1200.0F, -400.0F, 5000000);  // post-burnout sounding rocket

  GpsNoiseModel noise_model(GpsNoiseConfig{}, /*seed=*/7);
  GpsDynamicsModel dynamics;  // airborne <4 g, COCOM on
  GpsFix fix = noise_model.apply(truth);
  assert(fix.fix_type == GpsFixType::kFix3D);  // the noise model alone always reports a fix
  const GpsDynamicsVerdict verdict = dynamics.apply(truth, fix);
  assert(verdict == GpsDynamicsVerdict::kCocomLimit);

  const auto frame = UbxEmitter::encode_nav_pvt(fix);
  UbxParser parser;
  GpsFix decoded;
  GpsParseError error = GpsParseError::kIncomplete;
  for (const std::uint8_t byte : frame)
  {
    error = parser.parse_byte(byte, truth.timestamp_us, decoded);
  }

  assert(error == GpsParseError::kNone);
  assert(decoded.fix_type == GpsFixType::kNoFix);
  assert(decoded.num_satellites == 0);
}

}  // namespace

int main()
{
  test_platform_limits_table();
  test_within_envelope_passes_through();
  test_each_platform_limit_invalidates_the_fix();
  test_acceleration_limit();
  test_cocom_limits();
  test_reacquisition_hold_off();
  test_invalidated_fix_decodes_as_no_fix();

  std::puts("test_gps_dynamics: all checks passed");
  return 0;
}
