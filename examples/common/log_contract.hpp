// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file log_contract.hpp
/// @brief The names the co-simulation examples write and Aetherion's replay
/// reads: the sensor-log headers and the run-configuration keys it needs
/// (sensor fusion step 2).
///
/// Aetherion does not depend on Hemerion. It replays the logs these examples
/// write and binds their columns **by name** (its
/// `Aetherion/Estimation/LogContract.h`), so a renamed column here does not
/// fail to compile anywhere -- it fails a replay, on someone else's machine,
/// later. Every name that crosses that boundary is therefore written from this
/// one header, by both flight computers and all three hosts, and
/// `tests/unit/test_log_contract.cpp` pins it against the names Aetherion
/// requires. `scripts/check_log_contract.py` checks the same thing against a
/// real `LogContract.h` and against the logs of a finished run.
///
/// The headers are whole lines, not column lists assembled at run time, so
/// what a test reads here is byte for byte what reaches the file.

#pragma once

#include <string_view>

/// @namespace hemerion::examples::log_contract
/// @brief Names shared between the examples' log writers and their readers.
namespace hemerion::examples::log_contract
{

/// `gps_fixes.csv`, one row per decoded NAV-PVT epoch, fix or not.
///
/// `nominal_time_s` is the fix's validity time, (`fix_index` - 1) times the
/// fix period. `altitude_m` is NAV-PVT `height`, above the ellipsoid. The NED
/// velocity and its accuracy were appended rather than placed beside speed
/// and course, so a reader that indexes by position still finds the old
/// columns where they were.
inline constexpr std::string_view kGpsHeader = "fix_index,nominal_time_s,host_time_s,latitude_deg,longitude_deg,"
                                               "altitude_m,ground_speed_mps,course_deg,"
                                               "horizontal_accuracy_m,vertical_accuracy_m,num_satellites,fix_type,vel_"
                                               "north_mps,vel_east_mps,vel_down_mps,"
                                               "speed_accuracy_mps";

/// `imu_samples.csv`, one row per decoded IMU frame.
inline constexpr std::string_view kImuHeader = "sample_index,part_time_s,host_time_s,accel_x_mps2,accel_y_mps2,accel_z_"
                                               "mps2,gyro_x_rad_s,gyro_y_rad_s,"
                                               "gyro_z_rad_s";

/// `baro_samples.csv`, one row per BMP390 conversion read. `part_time_s` is
/// SENSORTIME, which wraps every 512 s.
inline constexpr std::string_view kBaroHeader = "sample_index,part_time_s,host_time_s,pressure_pa,temperature_c";

/// @name Run-configuration sidecar keys (`<truth stem>.config`)
/// The ones Aetherion requires. `lat0_deg`/`lon0_deg`/`alt0_ft` place the
/// launch-centred inertial frame's origin; the rocket host, which takes its
/// pad height in metres, writes `alt0_ft` beside its own `alt0_m`.
/// @{
inline constexpr std::string_view kLat0Key = "lat0_deg";
inline constexpr std::string_view kLon0Key = "lon0_deg";
inline constexpr std::string_view kAlt0FtKey = "alt0_ft";
inline constexpr std::string_view kGpsLatencyKey = "gps_latency_s";
inline constexpr std::string_view kRealtimeFactorKey = "realtime_factor";
/// @}

}  // namespace hemerion::examples::log_contract
