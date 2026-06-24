// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/power/battery_monitor.h
//
// BMS pack telemetry model and limit-based fault evaluation. This is a pure
// data layer: it has no dependency on an ADC/BMS peripheral or the HAL
// abstraction (cmake/hemerion_hal/ has no power.h yet), so it only models a
// telemetry sample and checks it against configured limits. A later
// HAL-backed driver fills BatterySample from the physical BMS and feeds it
// here.
// ------------------------------------------------------------------------------
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace hemerion::power {

inline constexpr std::size_t kMaxCells = 12;

struct BatteryLimits {
  float cell_voltage_min_v = 3.0f;
  float cell_voltage_max_v = 4.25f;
  float pack_current_max_a = 80.0f;
  float cell_temperature_max_c = 60.0f;
};

struct BatterySample {
  std::array<float, kMaxCells> cell_voltages_v{};
  std::uint8_t cell_count = 0;
  float pack_current_a = 0.0f;
  float cell_temperature_c = 0.0f;
};

// Bitmask values, OR-able into the std::uint8_t returned by evaluate_faults().
enum class BatteryFaultBits : std::uint8_t {
  kNone = 0,
  kCellUnderVoltage = 1U << 0,
  kCellOverVoltage = 1U << 1,
  kOverCurrent = 1U << 2,
  kOverTemperature = 1U << 3,
};

// Checks `sample` against `limits` and returns the OR of all BatteryFaultBits
// that are violated (BatteryFaultBits::kNone if none). Only the first
// min(sample.cell_count, kMaxCells) entries of cell_voltages_v are inspected.
[[nodiscard]] std::uint8_t evaluate_faults(const BatterySample& sample, const BatteryLimits& limits);

// Returns the lowest cell voltage among the inspected cells, or 0.0f if
// sample.cell_count is 0.
[[nodiscard]] float min_cell_voltage_v(const BatterySample& sample);

// Returns the highest cell voltage among the inspected cells, or 0.0f if
// sample.cell_count is 0.
[[nodiscard]] float max_cell_voltage_v(const BatterySample& sample);

}  // namespace hemerion::power
