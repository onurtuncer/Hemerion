// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_battery_monitor.cpp
//
// Native unit test for the BMS fault evaluation layer. Run by CTest under
// the test-native preset. Unity (referenced in modules/README.md as the
// project's test framework) is not yet vendored, so this uses plain asserts
// and an exit code; switch to Unity test cases once vendor/Unity exists.
// ------------------------------------------------------------------------------
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "hemerion/power/battery_monitor.h"

using hemerion::power::BatteryFaultBits;
using hemerion::power::BatteryLimits;
using hemerion::power::BatterySample;
using hemerion::power::evaluate_faults;
using hemerion::power::max_cell_voltage_v;
using hemerion::power::min_cell_voltage_v;

namespace {

BatterySample make_nominal_sample() {
  BatterySample sample;
  sample.cell_count = 4;
  sample.cell_voltages_v[0] = 3.7f;
  sample.cell_voltages_v[1] = 3.8f;
  sample.cell_voltages_v[2] = 3.75f;
  sample.cell_voltages_v[3] = 3.72f;
  sample.pack_current_a = 10.0f;
  sample.cell_temperature_c = 25.0f;
  return sample;
}

void test_nominal_sample_has_no_faults() {
  const BatterySample sample = make_nominal_sample();
  const BatteryLimits limits;
  assert(evaluate_faults(sample, limits) == static_cast<std::uint8_t>(BatteryFaultBits::kNone));
}

void test_cell_under_voltage_detected() {
  BatterySample sample = make_nominal_sample();
  sample.cell_voltages_v[2] = 2.5f;
  const BatteryLimits limits;
  const std::uint8_t faults = evaluate_faults(sample, limits);
  assert(faults & static_cast<std::uint8_t>(BatteryFaultBits::kCellUnderVoltage));
}

void test_cell_over_voltage_detected() {
  BatterySample sample = make_nominal_sample();
  sample.cell_voltages_v[0] = 4.3f;
  const BatteryLimits limits;
  const std::uint8_t faults = evaluate_faults(sample, limits);
  assert(faults & static_cast<std::uint8_t>(BatteryFaultBits::kCellOverVoltage));
}

void test_over_current_detected() {
  BatterySample sample = make_nominal_sample();
  sample.pack_current_a = 100.0f;
  const BatteryLimits limits;
  const std::uint8_t faults = evaluate_faults(sample, limits);
  assert(faults & static_cast<std::uint8_t>(BatteryFaultBits::kOverCurrent));
}

void test_over_temperature_detected() {
  BatterySample sample = make_nominal_sample();
  sample.cell_temperature_c = 80.0f;
  const BatteryLimits limits;
  const std::uint8_t faults = evaluate_faults(sample, limits);
  assert(faults & static_cast<std::uint8_t>(BatteryFaultBits::kOverTemperature));
}

void test_min_max_cell_voltage() {
  const BatterySample sample = make_nominal_sample();
  assert(min_cell_voltage_v(sample) == 3.7f);
  assert(max_cell_voltage_v(sample) == 3.8f);
}

void test_zero_cell_count_reports_no_voltage_faults() {
  BatterySample sample;
  sample.cell_count = 0;
  sample.pack_current_a = 5.0f;
  sample.cell_temperature_c = 25.0f;
  const BatteryLimits limits;
  assert(min_cell_voltage_v(sample) == 0.0f);
  assert(max_cell_voltage_v(sample) == 0.0f);
  assert(evaluate_faults(sample, limits) == static_cast<std::uint8_t>(BatteryFaultBits::kNone));
}

}  // namespace

int main() {
  test_nominal_sample_has_no_faults();
  test_cell_under_voltage_detected();
  test_cell_over_voltage_detected();
  test_over_current_detected();
  test_over_temperature_detected();
  test_min_max_cell_voltage();
  test_zero_cell_count_reports_no_voltage_faults();

  std::puts("test_battery_monitor: all checks passed");
  return 0;
}
