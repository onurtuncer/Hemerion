// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
#include "hemerion/power/battery_monitor.h"

#include <algorithm>

namespace hemerion::power {

namespace {

[[nodiscard]] std::uint8_t clamped_cell_count(const BatterySample& sample) {
  return std::min<std::uint8_t>(sample.cell_count, static_cast<std::uint8_t>(kMaxCells));
}

}  // namespace

float min_cell_voltage_v(const BatterySample& sample) {
  const std::uint8_t count = clamped_cell_count(sample);
  if (count == 0) {
    return 0.0f;
  }
  return *std::min_element(sample.cell_voltages_v.begin(), sample.cell_voltages_v.begin() + count);
}

float max_cell_voltage_v(const BatterySample& sample) {
  const std::uint8_t count = clamped_cell_count(sample);
  if (count == 0) {
    return 0.0f;
  }
  return *std::max_element(sample.cell_voltages_v.begin(), sample.cell_voltages_v.begin() + count);
}

std::uint8_t evaluate_faults(const BatterySample& sample, const BatteryLimits& limits) {
  std::uint8_t faults = static_cast<std::uint8_t>(BatteryFaultBits::kNone);

  if (clamped_cell_count(sample) > 0) {
    if (min_cell_voltage_v(sample) < limits.cell_voltage_min_v) {
      faults |= static_cast<std::uint8_t>(BatteryFaultBits::kCellUnderVoltage);
    }
    if (max_cell_voltage_v(sample) > limits.cell_voltage_max_v) {
      faults |= static_cast<std::uint8_t>(BatteryFaultBits::kCellOverVoltage);
    }
  }
  if (sample.pack_current_a > limits.pack_current_max_a) {
    faults |= static_cast<std::uint8_t>(BatteryFaultBits::kOverCurrent);
  }
  if (sample.cell_temperature_c > limits.cell_temperature_max_c) {
    faults |= static_cast<std::uint8_t>(BatteryFaultBits::kOverTemperature);
  }
  return faults;
}

}  // namespace hemerion::power
