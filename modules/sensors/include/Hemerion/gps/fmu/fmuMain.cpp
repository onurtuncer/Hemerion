// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// Hemerion/gps/fmu/fmuMain.cpp
//
// FMI 2.0 Co-Simulation slave entry points for the GPS UBX hardware
// simulator. This FMU has no FMI output variables -- its effect is a UDP
// side channel: each fmi2DoStep() call turns the current truth-state
// inputs into a noisy GpsFix (GpsNoiseModel), encodes it as a real
// UBX-NAV-PVT frame (UbxEmitter), and sends it to a fixed UDP peer
// (UdpSender) -- typically Renode's emulated UART4 RX backend, so the real,
// unmodified GpsDriver/UbxParser on the firmware side decodes it exactly as
// it would bytes from a physical u-blox M9N.
//
// UDP destination is read once at fmi2Instantiate() from
// HEMERION_GPS_FMU_UDP_HOST / HEMERION_GPS_FMU_UDP_PORT (defaults
// 127.0.0.1:5762) rather than exposed as an FMI string parameter -- this
// keeps fmi2SetString unimplemented, which is fine: it's only mandatory for
// FMUs that declare String-typed variables in model_description.xml, and
// this one doesn't.
//
// model_description.xml's <fmiModelDescription guid="..."> must match
// kExpectedGuid below exactly; fmi2Instantiate() refuses to start otherwise
// rather than silently running against the wrong model description.
// ------------------------------------------------------------------------------
#include "Hemerion/gps/fmu/fmi2Minimal.hpp"
#include "Hemerion/gps/fmu/gpsNoiseModel.hpp"
#include "Hemerion/gps/fmu/ubxEmitter.hpp"
#include "Hemerion/gps/fmu/udpSender.hpp"

#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>

namespace {

using hemerion::sensors::gps::fmu::GpsNoiseModel;
using hemerion::sensors::gps::fmu::GpsTruthSample;
using hemerion::sensors::gps::fmu::UbxEmitter;
using hemerion::sensors::gps::fmu::UdpSender;

constexpr fmi2ValueReference kVrLatitudeDeg = 0;
constexpr fmi2ValueReference kVrLongitudeDeg = 1;
constexpr fmi2ValueReference kVrAltitudeM = 2;
constexpr fmi2ValueReference kVrGroundSpeedMps = 3;
constexpr fmi2ValueReference kVrCourseDeg = 4;

// Must match model_description.xml's <fmiModelDescription guid="...">.
constexpr char kExpectedGuid[] = "{3F2A1C9E-7B4D-4E8A-9C3F-6A1B2C3D4E5F}";

constexpr char kDefaultUdpHost[] = "127.0.0.1";
constexpr std::uint16_t kDefaultUdpPort = 5762;

struct GpsFmuInstance {
  GpsNoiseModel noise_model;
  UdpSender sender;
  GpsTruthSample truth;
  fmi2CallbackFunctions callbacks{};
  bool logging_on = false;
};

GpsFmuInstance* to_instance(fmi2Component component) {
  return static_cast<GpsFmuInstance*>(component);
}

}  // namespace

HEMERION_FMI2_EXPORT const char* fmi2GetTypesPlatform() {
  return "default";
}

HEMERION_FMI2_EXPORT const char* fmi2GetVersion() {
  return "2.0";
}

HEMERION_FMI2_EXPORT fmi2Status fmi2SetDebugLogging(fmi2Component component, fmi2Boolean loggingOn,
                                                     std::size_t nCategories, const fmi2String categories[]) {
  (void)nCategories;
  (void)categories;
  to_instance(component)->logging_on = (loggingOn != 0);
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Component fmi2Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID,
                                                    fmi2String fmuResourceLocation,
                                                    const fmi2CallbackFunctions* functions, fmi2Boolean visible,
                                                    fmi2Boolean loggingOn) {
  (void)instanceName;
  (void)fmuResourceLocation;
  (void)visible;

  if (fmuType != fmi2CoSimulation) {
    return nullptr;  // this FMU only implements the Co-Simulation interface
  }
  if (fmuGUID == nullptr || std::strcmp(fmuGUID, kExpectedGuid) != 0) {
    return nullptr;  // master is loading a model_description.xml this binary doesn't match
  }

  // std::getenv is flagged deprecated by the Windows SDK headers (in favour of _dupenv_s) purely as an MSVC CRT
  // "insecure function" nag, not a real portability issue -- std::getenv is the standard, portable way to do this.
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#elif defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
  const char* host_env = std::getenv("HEMERION_GPS_FMU_UDP_HOST");
  const char* port_env = std::getenv("HEMERION_GPS_FMU_UDP_PORT");
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(_MSC_VER)
#pragma warning(pop)
#endif
  const std::string host = (host_env != nullptr) ? host_env : kDefaultUdpHost;
  const std::uint16_t port =
      (port_env != nullptr) ? static_cast<std::uint16_t>(std::atoi(port_env)) : kDefaultUdpPort;

  auto sender = UdpSender::create(host, port);
  if (!sender.has_value()) {
    return nullptr;
  }

  auto* instance = new GpsFmuInstance{GpsNoiseModel{}, std::move(*sender), GpsTruthSample{},
                                       functions != nullptr ? *functions : fmi2CallbackFunctions{}, loggingOn != 0};
  return instance;
}

HEMERION_FMI2_EXPORT void fmi2FreeInstance(fmi2Component component) {
  delete to_instance(component);
}

HEMERION_FMI2_EXPORT fmi2Status fmi2SetupExperiment(fmi2Component component, fmi2Boolean toleranceDefined,
                                                     fmi2Real tolerance, fmi2Real startTime,
                                                     fmi2Boolean stopTimeDefined, fmi2Real stopTime) {
  (void)component;
  (void)toleranceDefined;
  (void)tolerance;
  (void)startTime;
  (void)stopTimeDefined;
  (void)stopTime;
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2EnterInitializationMode(fmi2Component component) {
  (void)component;
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2ExitInitializationMode(fmi2Component component) {
  (void)component;
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2Terminate(fmi2Component component) {
  (void)component;
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2Reset(fmi2Component component) {
  to_instance(component)->truth = GpsTruthSample{};
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2SetReal(fmi2Component component, const fmi2ValueReference vr[], std::size_t nvr,
                                             const fmi2Real value[]) {
  GpsFmuInstance* instance = to_instance(component);
  for (std::size_t i = 0; i < nvr; ++i) {
    switch (vr[i]) {
      case kVrLatitudeDeg:
        instance->truth.latitude_deg = value[i];
        break;
      case kVrLongitudeDeg:
        instance->truth.longitude_deg = value[i];
        break;
      case kVrAltitudeM:
        instance->truth.altitude_m = static_cast<float>(value[i]);
        break;
      case kVrGroundSpeedMps:
        instance->truth.ground_speed_mps = static_cast<float>(value[i]);
        break;
      case kVrCourseDeg:
        instance->truth.course_deg = static_cast<float>(value[i]);
        break;
      default:
        return fmi2Error;  // unknown value reference
    }
  }
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetReal(fmi2Component component, const fmi2ValueReference vr[], std::size_t nvr,
                                             fmi2Real value[]) {
  (void)component;
  (void)vr;
  (void)value;
  // No FMI outputs are exposed -- this FMU's effect is the UDP side channel
  // in fmi2DoStep, not FMI variable exchange.
  return (nvr == 0) ? fmi2OK : fmi2Error;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2SetInteger(fmi2Component component, const fmi2ValueReference vr[],
                                                std::size_t nvr, const fmi2Integer value[]) {
  (void)component;
  (void)vr;
  (void)value;
  return (nvr == 0) ? fmi2OK : fmi2Error;  // no Integer-typed variables
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetInteger(fmi2Component component, const fmi2ValueReference vr[],
                                                std::size_t nvr, fmi2Integer value[]) {
  (void)component;
  (void)vr;
  (void)value;
  return (nvr == 0) ? fmi2OK : fmi2Error;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2SetBoolean(fmi2Component component, const fmi2ValueReference vr[],
                                                std::size_t nvr, const fmi2Boolean value[]) {
  (void)component;
  (void)vr;
  (void)value;
  return (nvr == 0) ? fmi2OK : fmi2Error;  // no Boolean-typed variables
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetBoolean(fmi2Component component, const fmi2ValueReference vr[],
                                                std::size_t nvr, fmi2Boolean value[]) {
  (void)component;
  (void)vr;
  (void)value;
  return (nvr == 0) ? fmi2OK : fmi2Error;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2SetString(fmi2Component component, const fmi2ValueReference vr[],
                                               std::size_t nvr, const fmi2String value[]) {
  (void)component;
  (void)vr;
  (void)value;
  return (nvr == 0) ? fmi2OK : fmi2Error;  // no String-typed variables
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetString(fmi2Component component, const fmi2ValueReference vr[],
                                               std::size_t nvr, fmi2String value[]) {
  (void)component;
  (void)vr;
  (void)value;
  return (nvr == 0) ? fmi2OK : fmi2Error;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2DoStep(fmi2Component component, fmi2Real currentCommunicationPoint,
                                            fmi2Real communicationStepSize,
                                            fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
  (void)noSetFMUStatePriorToCurrentPoint;
  GpsFmuInstance* instance = to_instance(component);

  const fmi2Real step_end_time = currentCommunicationPoint + communicationStepSize;
  instance->truth.timestamp_us = static_cast<std::uint64_t>(step_end_time * 1e6);

  const auto fix = instance->noise_model.apply(instance->truth);
  const auto frame = UbxEmitter::encode_nav_pvt(fix);
  // best-effort: a dropped datagram doesn't fault the co-sim step, so the [[nodiscard]] result is intentionally
  // discarded here rather than propagated as fmi2Warning/fmi2Error.
  (void)instance->sender.send(frame.data(), frame.size());
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2CancelStep(fmi2Component component) {
  (void)component;
  return fmi2OK;  // fmi2DoStep here is always synchronous; there is never a step in flight to cancel
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetStatus(fmi2Component component, fmi2StatusKind status, fmi2Status* value) {
  (void)component;
  (void)status;
  (void)value;
  return fmi2Discard;  // fmi2DoStep never returns fmi2Pending, so a master should never call this
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetRealStatus(fmi2Component component, fmi2StatusKind status, fmi2Real* value) {
  (void)component;
  (void)status;
  (void)value;
  return fmi2Discard;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetIntegerStatus(fmi2Component component, fmi2StatusKind status,
                                                      fmi2Integer* value) {
  (void)component;
  (void)status;
  (void)value;
  return fmi2Discard;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetBooleanStatus(fmi2Component component, fmi2StatusKind status,
                                                      fmi2Boolean* value) {
  (void)component;
  (void)status;
  (void)value;
  return fmi2Discard;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetStringStatus(fmi2Component component, fmi2StatusKind status,
                                                     fmi2String* value) {
  (void)component;
  (void)status;
  (void)value;
  return fmi2Discard;
}
