// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_gps_driver.cpp
//
// Native unit test for GpsDriver's protocol dispatch. The NMEA and UBX wire
// formats themselves are covered by test_nmea_parser.cpp / test_ubx_parser.cpp;
// this only checks that GpsDriver routes bytes to the parser matching its
// configured GpsProtocol. Run by CTest under the test-native preset.
// ------------------------------------------------------------------------------
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <string_view>
#include <vector>

#include "Hemerion/gps/gpsDriver.hpp"

using hemerion::sensors::gps::GpsDriver;
using hemerion::sensors::gps::GpsFix;
using hemerion::sensors::gps::GpsParseError;
using hemerion::sensors::gps::GpsProtocol;

namespace
{

bool near(double a, double b, double tol = 1e-3) { return std::fabs(a - b) <= tol; }

void test_nmea_protocol_decodes_gga()
{
  GpsDriver driver(GpsProtocol::kNmea);
  assert(driver.protocol() == GpsProtocol::kNmea);

  // The sentence is decoded when the '\r' is fed; the trailing '\n' that
  // follows it is a no-op (the sentence buffer already closed), so track the
  // last *meaningful* result rather than simply the final byte's result.
  const std::string_view sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
  GpsFix fix;
  GpsParseError error = GpsParseError::kIncomplete;
  for (const char c : sentence)
  {
    const GpsParseError result = driver.feed(static_cast<std::uint8_t>(c), 42, fix);
    if (result != GpsParseError::kIncomplete)
    {
      error = result;
    }
  }

  assert(error == GpsParseError::kNone);
  assert(near(fix.latitude_deg, 48.1173));
  assert(fix.timestamp_us == 42);
}

void test_ubx_protocol_decodes_nav_pvt()
{
  GpsDriver driver(GpsProtocol::kUbx);
  assert(driver.protocol() == GpsProtocol::kUbx);

  // class=0x01 (NAV), id=0x07 (PVT), length=0 -- only the header/checksum
  // routing through UbxParser is under test here, not the payload decode
  // (that's covered by test_ubx_parser.cpp's full-payload case). A 0-length
  // NAV-PVT is below kNavPvtMinPayloadLength, so it resolves to kMalformed,
  // which is enough to prove the bytes reached UbxParser rather than
  // NmeaParser.
  const std::vector<std::uint8_t> message = { 0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19 };
  GpsFix fix;
  GpsParseError error = GpsParseError::kIncomplete;
  for (const std::uint8_t byte : message)
  {
    error = driver.feed(byte, 0, fix);
  }

  assert(error == GpsParseError::kMalformed);
}

}  // namespace

int main()
{
  test_nmea_protocol_decodes_gga();
  test_ubx_protocol_decodes_nav_pvt();

  std::puts("test_gps_driver: all checks passed");
  return 0;
}
