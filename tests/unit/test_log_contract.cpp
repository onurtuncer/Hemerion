// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_log_contract.cpp
//
// The contract between this repository's co-simulation logs and Aetherion's
// replay (sensor fusion step 2): the column names and run-configuration keys
// in examples/common/log_contract.hpp, which both flight computers and all
// three hosts write from, against the names Aetherion requires.
//
// Aetherion binds the columns by name and Hemerion never compiles against it,
// so a rename here fails nothing in this repository -- it fails a replay
// somewhere else, later. This test is where it fails instead. The required
// names below are a pinned copy of Aetherion v0.18.0's
// Aetherion/Estimation/LogContract.h (namespaces Gps, Imu, Baro, Sidecar),
// because CI has no Aetherion to read them from. Where an install is found,
// examples.log_contract_upstream (tests/unit/CMakeLists.txt) checks the copy
// against the real header with scripts/check_log_contract.py, so the copy
// cannot go stale silently either.
//
// Plain asserts + exit code, matching the other unit tests.
// ------------------------------------------------------------------------------
#include <array>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "environment.hpp"
#include "log_contract.hpp"

namespace contract = hemerion::examples::log_contract;

#define CHECK(condition) check((condition), #condition, __FILE__, __LINE__)

namespace
{

// assert() is compiled out under NDEBUG; these checks must not be.
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

std::vector<std::string> split(std::string_view header)
{
  std::vector<std::string> columns;
  std::size_t start = 0;
  while (true)
  {
    const std::size_t comma = header.find(',', start);
    columns.emplace_back(header.substr(start, comma - start));
    if (comma == std::string_view::npos)
    {
      return columns;
    }
    start = comma + 1;
  }
}

bool contains(const std::vector<std::string>& columns, std::string_view name)
{
  for (const std::string& column : columns)
  {
    if (column == name)
    {
      return true;
    }
  }
  return false;
}

// Aetherion v0.18.0, Aetherion/Estimation/LogContract.h. Every column of the
// three sensor logs that requiredColumns() can ask for, i.e. the 16-state
// nav-baro-gnss16 configuration (GNSS position + velocity + barometer), plus
// the GPS validity-time column the replay prefers when present.
constexpr std::array<std::string_view, 12> kAetherionGps{
  "host_time_s",         "nominal_time_s", "latitude_deg",  "longitude_deg", "altitude_m",   "horizontal_accuracy_m",
  "vertical_accuracy_m", "fix_type",       "vel_north_mps", "vel_east_mps",  "vel_down_mps", "speed_accuracy_mps",
};
constexpr std::array<std::string_view, 8> kAetherionImu{
  "part_time_s",  "host_time_s",  "accel_x_mps2", "accel_y_mps2",
  "accel_z_mps2", "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s",
};
constexpr std::array<std::string_view, 4> kAetherionBaro{ "part_time_s",
                                                          "host_time_s",
                                                          "pressure_pa",
                                                          "temperature_c" };
constexpr std::array<std::string_view, 5> kAetherionSidecar{ "lat0_deg",
                                                             "lon0_deg",
                                                             "alt0_ft",
                                                             "gps_latency_s",
                                                             "realtime_factor" };

template <std::size_t N>
void require_all(std::string_view header, const std::array<std::string_view, N>& required)
{
  const std::vector<std::string> columns = split(header);
  for (std::string_view name : required)
  {
    if (!contains(columns, name))
    {
      std::cerr << "log header is missing '" << name << "' that Aetherion's replay binds: " << header << "\n";
      CHECK(contains(columns, name));
    }
  }
}

void test_every_required_column_is_written()
{
  require_all(contract::kGpsHeader, kAetherionGps);
  require_all(contract::kImuHeader, kAetherionImu);
  require_all(contract::kBaroHeader, kAetherionBaro);
}

void test_headers_are_well_formed()
{
  // A duplicated name binds to whichever the reader meets first, and an empty
  // one is a stray comma: both would read the wrong column without an error.
  for (std::string_view header : { contract::kGpsHeader, contract::kImuHeader, contract::kBaroHeader })
  {
    const std::vector<std::string> columns = split(header);
    const std::set<std::string> unique(columns.begin(), columns.end());
    CHECK(unique.size() == columns.size());
    for (const std::string& column : columns)
    {
      CHECK(!column.empty());
      CHECK(column.find_first_of(" \t\r\n\"") == std::string::npos);
    }
  }
}

void test_existing_columns_kept_their_positions()
{
  // The NED velocity was appended so a reader that indexes GPS columns by
  // position (Paper 1's scripts, older plot scripts) still finds the original
  // twelve where they were. The index is also the time mapping.
  const std::vector<std::string> gps = split(contract::kGpsHeader);
  CHECK(gps.size() == 16);
  CHECK(gps[0] == "fix_index");
  CHECK(gps[1] == "nominal_time_s");
  CHECK(gps[5] == "altitude_m");
  CHECK(gps[11] == "fix_type");
  CHECK(gps[12] == "vel_north_mps");
  CHECK(gps[15] == "speed_accuracy_mps");
}

void test_sidecar_keys()
{
  const std::array<std::string_view, 5> written{
    contract::kLat0Key, contract::kLon0Key, contract::kAlt0FtKey, contract::kGpsLatencyKey, contract::kRealtimeFactorKey
  };
  CHECK(written == kAetherionSidecar);
}

void test_drawn_error_columns()
{
  // Step 3: the realised sensor errors, logged as truth columns. The names
  // are the FMUs' output variables; this checks the list is complete and
  // well-formed, the FMUs' own registration is checked at the FMI level.
  const std::vector<std::string> names = hemerion::examples::drawn_error_variables();
  CHECK(names.size() == 20);
  const std::set<std::string> unique(names.begin(), names.end());
  CHECK(unique.size() == names.size());
  for (const char* expected : { "imu::accel_bias_x_mps2",
                                "imu::gyro_bias_z_rad_s",
                                "imu::accel_scale_error_y",
                                "imu::gyro_scale_error_x",
                                "imu::accel_misalignment_z_rad",
                                "imu::gyro_misalignment_y_rad",
                                "baro::pressure_bias_pa",
                                "baro::temperature_bias_c" })
  {
    CHECK(unique.contains(expected));
  }
  // And appending them keeps the host's own columns first, in order.
  const std::vector<std::string> logged = hemerion::examples::with_drawn_errors({ "f16::out.alt_m", "imu::f_x_mps2" });
  CHECK(logged.size() == 22);
  CHECK(logged[0] == "f16::out.alt_m");
  CHECK(logged[1] == "imu::f_x_mps2");
  CHECK(logged[2] == names[0]);
}

}  // namespace

int main()
{
  test_every_required_column_is_written();
  test_headers_are_well_formed();
  test_existing_columns_kept_their_positions();
  test_sidecar_keys();
  test_drawn_error_columns();

  std::puts("test_log_contract: all checks passed");
  return 0;
}
