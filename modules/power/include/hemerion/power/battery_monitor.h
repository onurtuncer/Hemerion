// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file battery_monitor.h
/// @brief BMS pack telemetry model and limit-based fault evaluation.
///
/// This is a pure data layer: it has no dependency on an ADC/BMS peripheral
/// or the HAL abstraction (cmake/hemerion_hal/ has no power.h yet), so it
/// only models a telemetry sample and checks it against configured limits. A
/// later HAL-backed driver fills BatterySample from the physical BMS and
/// feeds it here.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

/// @namespace hemerion::power
/// @brief Battery telemetry evaluation and regulator rail sequencing.
namespace hemerion::power
{

/// Maximum number of series cells a BatterySample can describe.
inline constexpr std::size_t kMaxCells = 12;

/// Operating limits a BatterySample is checked against by evaluate_faults().
struct BatteryLimits
{
  float cell_voltage_min_v = 3.0f;       ///< Per-cell undervoltage threshold [V].
  float cell_voltage_max_v = 4.25f;      ///< Per-cell overvoltage threshold [V].
  float pack_current_max_a = 80.0f;      ///< Pack discharge current limit [A].
  float cell_temperature_max_c = 60.0f;  ///< Cell temperature limit [degrees C].
};

/// One snapshot of pack telemetry, as a HAL-backed BMS driver would report it.
struct BatterySample
{
  std::array<float, kMaxCells> cell_voltages_v{};  ///< Per-cell voltages [V]; first `cell_count` entries are valid.
  std::uint8_t cell_count = 0;                     ///< Number of valid entries in cell_voltages_v.
  float pack_current_a = 0.0f;                     ///< Pack current [A].
  float cell_temperature_c = 0.0f;                 ///< Representative cell temperature [degrees C].
};

/// Bitmask values, OR-able into the std::uint8_t returned by evaluate_faults().
enum class BatteryFaultBits : std::uint8_t
{
  kNone = 0,                    ///< No limit violated.
  kCellUnderVoltage = 1U << 0,  ///< At least one cell below BatteryLimits::cell_voltage_min_v.
  kCellOverVoltage = 1U << 1,   ///< At least one cell above BatteryLimits::cell_voltage_max_v.
  kOverCurrent = 1U << 2,       ///< Pack current above BatteryLimits::pack_current_max_a.
  kOverTemperature = 1U << 3,   ///< Cell temperature above BatteryLimits::cell_temperature_max_c.
};

/// @brief Checks `sample` against `limits`.
///
/// Only the first min(sample.cell_count, kMaxCells) entries of
/// cell_voltages_v are inspected.
///
/// @param sample Telemetry snapshot to evaluate.
/// @param limits Limits to evaluate against.
/// @return The OR of all violated BatteryFaultBits
///         (BatteryFaultBits::kNone if none).
[[nodiscard]] std::uint8_t evaluate_faults(const BatterySample& sample, const BatteryLimits& limits);

/// @brief Lowest cell voltage among the inspected cells.
/// @param sample Telemetry snapshot to inspect.
/// @return Minimum cell voltage [V], or 0.0f if sample.cell_count is 0.
[[nodiscard]] float min_cell_voltage_v(const BatterySample& sample);

/// @brief Highest cell voltage among the inspected cells.
/// @param sample Telemetry snapshot to inspect.
/// @return Maximum cell voltage [V], or 0.0f if sample.cell_count is 0.
[[nodiscard]] float max_cell_voltage_v(const BatterySample& sample);

}  // namespace hemerion::power
