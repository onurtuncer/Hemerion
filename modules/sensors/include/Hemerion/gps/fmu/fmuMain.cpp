// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file fmuMain.cpp
/// @brief FMI 2.0 Co-Simulation slave entry points for the GPS UBX hardware
/// simulator.
///
/// This FMU has no FMI output variables -- its effect is a UDP side
/// channel: each fmi2DoStep() call turns the current truth-state inputs
/// into a noisy GpsFix (GpsNoiseModel), encodes it as a real UBX-NAV-PVT
/// frame (UbxEmitter), and sends it to a fixed UDP peer (UdpSender) --
/// typically Renode's emulated UART4 RX backend, so the real, unmodified
/// GpsDriver/UbxParser on the firmware side decodes it exactly as it would
/// bytes from a physical u-blox M9N.
///
/// UDP destination is read once at fmi2Instantiate() from
/// HEMERION_GPS_FMU_UDP_HOST / HEMERION_GPS_FMU_UDP_PORT (defaults
/// 127.0.0.1:5762) rather than exposed as an FMI string parameter -- this
/// keeps fmi2SetString unimplemented, which is fine: it's only mandatory
/// for FMUs that declare String-typed variables in model_description.xml,
/// and this one doesn't.
///
/// model_description.xml's <fmiModelDescription guid="..."> must match
/// kExpectedGuid below exactly; fmi2Instantiate() refuses to start
/// otherwise rather than silently running against the wrong model
/// description.

#include "Hemerion/gps/fmu/fmi2Minimal.hpp"
#include "Hemerion/gps/fmu/gpsNoiseModel.hpp"
#include "Hemerion/gps/fmu/ubxEmitter.hpp"
#include "Hemerion/gps/fmu/udpSender.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>

namespace
{

using hemerion::sensors::gps::fmu::GpsNoiseModel;
using hemerion::sensors::gps::fmu::GpsTruthSample;
using hemerion::sensors::gps::fmu::UbxEmitter;
using hemerion::sensors::gps::fmu::UdpSender;

constexpr fmi2ValueReference kVrLatitudeDeg = 0;
constexpr fmi2ValueReference kVrLongitudeDeg = 1;
constexpr fmi2ValueReference kVrAltitudeM = 2;
constexpr fmi2ValueReference kVrGroundSpeedMps = 3;
constexpr fmi2ValueReference kVrCourseDeg = 4;
constexpr fmi2ValueReference kVrVNorthMps = 5;
constexpr fmi2ValueReference kVrVEastMps = 6;
constexpr fmi2ValueReference kVrVDownMps = 7;

// Must match model_description.xml's <fmiModelDescription guid="...">.
constexpr char kExpectedGuid[] = "{8C5D3A1F-2E9B-4D67-A4C8-5B0E9F3D7A21}";

constexpr char kDefaultUdpHost[] = "127.0.0.1";
constexpr std::uint16_t kDefaultUdpPort = 5762;

struct GpsFmuInstance
{
  GpsNoiseModel noise_model;
  UdpSender sender;
  GpsTruthSample truth;
  fmi2CallbackFunctions callbacks{};
  bool logging_on = false;
  // NED truth velocity (v_north_mps/v_east_mps/v_down_mps inputs). Once any component has been
  // written, speed-over-ground and course are derived from these at every fmi2DoStep and the
  // ground_speed_mps/course_deg inputs are ignored -- see model_description.xml.
  double v_north_mps = 0.0;
  double v_east_mps = 0.0;
  double v_down_mps = 0.0;
  bool ned_velocity_set = false;
};

GpsFmuInstance* to_instance(fmi2Component component) { return static_cast<GpsFmuInstance*>(component); }

}  // namespace

HEMERION_FMI2_EXPORT const char* fmi2GetTypesPlatform() { return "default"; }

HEMERION_FMI2_EXPORT const char* fmi2GetVersion() { return "2.0"; }

HEMERION_FMI2_EXPORT fmi2Status fmi2SetDebugLogging(fmi2Component component,
                                                    fmi2Boolean loggingOn,
                                                    std::size_t nCategories,
                                                    const fmi2String categories[])
{
  (void)nCategories;
  (void)categories;
  to_instance(component)->logging_on = (loggingOn != 0);
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Component fmi2Instantiate(fmi2String instanceName,
                                                   fmi2Type fmuType,
                                                   fmi2String fmuGUID,
                                                   fmi2String fmuResourceLocation,
                                                   const fmi2CallbackFunctions* functions,
                                                   fmi2Boolean visible,
                                                   fmi2Boolean loggingOn)
{
  (void)instanceName;
  (void)fmuResourceLocation;
  (void)visible;

  if (fmuType != fmi2CoSimulation)
  {
    return nullptr;  // this FMU only implements the Co-Simulation interface
  }
  if (fmuGUID == nullptr || std::strcmp(fmuGUID, kExpectedGuid) != 0)
  {
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
  const std::uint16_t port = (port_env != nullptr) ? static_cast<std::uint16_t>(std::atoi(port_env)) : kDefaultUdpPort;

  auto sender = UdpSender::create(host, port);
  if (!sender.has_value())
  {
    return nullptr;
  }

  auto* instance = new GpsFmuInstance{ GpsNoiseModel{},
                                       std::move(*sender),
                                       GpsTruthSample{},
                                       functions != nullptr ? *functions : fmi2CallbackFunctions{},
                                       loggingOn != 0 };
  return instance;
}

HEMERION_FMI2_EXPORT void fmi2FreeInstance(fmi2Component component) { delete to_instance(component); }

HEMERION_FMI2_EXPORT fmi2Status fmi2SetupExperiment(fmi2Component component,
                                                    fmi2Boolean toleranceDefined,
                                                    fmi2Real tolerance,
                                                    fmi2Real startTime,
                                                    fmi2Boolean stopTimeDefined,
                                                    fmi2Real stopTime)
{
  (void)component;
  (void)toleranceDefined;
  (void)tolerance;
  (void)startTime;
  (void)stopTimeDefined;
  (void)stopTime;
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2EnterInitializationMode(fmi2Component component)
{
  (void)component;
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2ExitInitializationMode(fmi2Component component)
{
  (void)component;
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2Terminate(fmi2Component component)
{
  (void)component;
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2Reset(fmi2Component component)
{
  GpsFmuInstance* instance = to_instance(component);
  instance->truth = GpsTruthSample{};
  instance->v_north_mps = 0.0;
  instance->v_east_mps = 0.0;
  instance->v_down_mps = 0.0;
  instance->ned_velocity_set = false;
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2SetReal(fmi2Component component,
                                            const fmi2ValueReference vr[],
                                            std::size_t nvr,
                                            const fmi2Real value[])
{
  GpsFmuInstance* instance = to_instance(component);
  for (std::size_t i = 0; i < nvr; ++i)
  {
    switch (vr[i])
    {
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
      case kVrVNorthMps:
        instance->v_north_mps = value[i];
        instance->ned_velocity_set = true;
        break;
      case kVrVEastMps:
        instance->v_east_mps = value[i];
        instance->ned_velocity_set = true;
        break;
      case kVrVDownMps:
        instance->v_down_mps = value[i];
        instance->ned_velocity_set = true;
        break;
      default:
        return fmi2Error;  // unknown value reference
    }
  }
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetReal(fmi2Component component,
                                            const fmi2ValueReference vr[],
                                            std::size_t nvr,
                                            fmi2Real value[])
{
  (void)component;
  (void)vr;
  (void)value;
  // No FMI outputs are exposed -- this FMU's effect is the UDP side channel
  // in fmi2DoStep, not FMI variable exchange.
  return (nvr == 0) ? fmi2OK : fmi2Error;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2SetInteger(fmi2Component component,
                                               const fmi2ValueReference vr[],
                                               std::size_t nvr,
                                               const fmi2Integer value[])
{
  (void)component;
  (void)vr;
  (void)value;
  return (nvr == 0) ? fmi2OK : fmi2Error;  // no Integer-typed variables
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetInteger(fmi2Component component,
                                               const fmi2ValueReference vr[],
                                               std::size_t nvr,
                                               fmi2Integer value[])
{
  (void)component;
  (void)vr;
  (void)value;
  return (nvr == 0) ? fmi2OK : fmi2Error;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2SetBoolean(fmi2Component component,
                                               const fmi2ValueReference vr[],
                                               std::size_t nvr,
                                               const fmi2Boolean value[])
{
  (void)component;
  (void)vr;
  (void)value;
  return (nvr == 0) ? fmi2OK : fmi2Error;  // no Boolean-typed variables
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetBoolean(fmi2Component component,
                                               const fmi2ValueReference vr[],
                                               std::size_t nvr,
                                               fmi2Boolean value[])
{
  (void)component;
  (void)vr;
  (void)value;
  return (nvr == 0) ? fmi2OK : fmi2Error;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2SetString(fmi2Component component,
                                              const fmi2ValueReference vr[],
                                              std::size_t nvr,
                                              const fmi2String value[])
{
  (void)component;
  (void)vr;
  (void)value;
  return (nvr == 0) ? fmi2OK : fmi2Error;  // no String-typed variables
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetString(fmi2Component component,
                                              const fmi2ValueReference vr[],
                                              std::size_t nvr,
                                              fmi2String value[])
{
  (void)component;
  (void)vr;
  (void)value;
  return (nvr == 0) ? fmi2OK : fmi2Error;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2DoStep(fmi2Component component,
                                           fmi2Real currentCommunicationPoint,
                                           fmi2Real communicationStepSize,
                                           fmi2Boolean noSetFMUStatePriorToCurrentPoint)
{
  (void)noSetFMUStatePriorToCurrentPoint;
  GpsFmuInstance* instance = to_instance(component);

  const fmi2Real step_end_time = currentCommunicationPoint + communicationStepSize;
  instance->truth.timestamp_us = static_cast<std::uint64_t>(step_end_time * 1e6);

  if (instance->ned_velocity_set)
  {
    // NED wiring is active: derive speed-over-ground and course from v_north/v_east, overriding
    // whatever the (unwired) ground_speed_mps/course_deg inputs hold. v_down is accepted but not
    // encoded -- NAV-PVT velD stays 0 until GpsFix grows a vertical-velocity field.
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    instance->truth.ground_speed_mps = static_cast<float>(std::hypot(instance->v_north_mps, instance->v_east_mps));
    const double course_deg = std::atan2(instance->v_east_mps, instance->v_north_mps) * kRadToDeg;
    instance->truth.course_deg = static_cast<float>(std::fmod(course_deg + 360.0, 360.0));
  }

  const auto fix = instance->noise_model.apply(instance->truth);
  const auto frame = UbxEmitter::encode_nav_pvt(fix);
  // best-effort: a dropped datagram doesn't fault the co-sim step, so the [[nodiscard]] result is intentionally
  // discarded here rather than propagated as fmi2Warning/fmi2Error.
  (void)instance->sender.send(frame.data(), frame.size());
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2CancelStep(fmi2Component component)
{
  (void)component;
  return fmi2OK;  // fmi2DoStep here is always synchronous; there is never a step in flight to cancel
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetStatus(fmi2Component component, fmi2StatusKind status, fmi2Status* value)
{
  (void)component;
  (void)status;
  (void)value;
  return fmi2Discard;  // fmi2DoStep never returns fmi2Pending, so a master should never call this
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetRealStatus(fmi2Component component, fmi2StatusKind status, fmi2Real* value)
{
  (void)component;
  (void)status;
  (void)value;
  return fmi2Discard;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetIntegerStatus(fmi2Component component, fmi2StatusKind status, fmi2Integer* value)
{
  (void)component;
  (void)status;
  (void)value;
  return fmi2Discard;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetBooleanStatus(fmi2Component component, fmi2StatusKind status, fmi2Boolean* value)
{
  (void)component;
  (void)status;
  (void)value;
  return fmi2Discard;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetStringStatus(fmi2Component component, fmi2StatusKind status, fmi2String* value)
{
  (void)component;
  (void)status;
  (void)value;
  return fmi2Discard;
}

// ---------------------------------------------------------------------------
// State-management and derivative functions. This FMU declares
// canGetAndSetFMUstate="false", canSerializeFMUstate="false" and
// providesDirectionalDerivative="false", so a conforming master never calls
// these -- but importers that resolve the complete FMI 2.0 export table up
// front (e.g. fmi4c, which Ecos uses) refuse to load a binary that omits any
// of them. Exported as plain fmi2Error stubs for that reason.
// ---------------------------------------------------------------------------

HEMERION_FMI2_EXPORT fmi2Status fmi2GetFMUstate(fmi2Component component, fmi2FMUstate* state)
{
  (void)component;
  (void)state;
  return fmi2Error;  // canGetAndSetFMUstate="false"
}

HEMERION_FMI2_EXPORT fmi2Status fmi2SetFMUstate(fmi2Component component, fmi2FMUstate state)
{
  (void)component;
  (void)state;
  return fmi2Error;  // canGetAndSetFMUstate="false"
}

HEMERION_FMI2_EXPORT fmi2Status fmi2FreeFMUstate(fmi2Component component, fmi2FMUstate* state)
{
  (void)component;
  (void)state;
  return fmi2Error;  // canGetAndSetFMUstate="false"
}

HEMERION_FMI2_EXPORT fmi2Status fmi2SerializedFMUstateSize(fmi2Component component,
                                                           fmi2FMUstate state,
                                                           std::size_t* size)
{
  (void)component;
  (void)state;
  (void)size;
  return fmi2Error;  // canSerializeFMUstate="false"
}

HEMERION_FMI2_EXPORT fmi2Status fmi2SerializeFMUstate(fmi2Component component,
                                                      fmi2FMUstate state,
                                                      fmi2Byte serialized[],
                                                      std::size_t size)
{
  (void)component;
  (void)state;
  (void)serialized;
  (void)size;
  return fmi2Error;  // canSerializeFMUstate="false"
}

HEMERION_FMI2_EXPORT fmi2Status fmi2DeSerializeFMUstate(fmi2Component component,
                                                        const fmi2Byte serialized[],
                                                        std::size_t size,
                                                        fmi2FMUstate* state)
{
  (void)component;
  (void)serialized;
  (void)size;
  (void)state;
  return fmi2Error;  // canSerializeFMUstate="false"
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetDirectionalDerivative(fmi2Component component,
                                                             const fmi2ValueReference unknowns[],
                                                             std::size_t nUnknowns,
                                                             const fmi2ValueReference knowns[],
                                                             std::size_t nKnowns,
                                                             const fmi2Real deltaKnowns[],
                                                             fmi2Real deltaUnknowns[])
{
  (void)component;
  (void)unknowns;
  (void)nUnknowns;
  (void)knowns;
  (void)nKnowns;
  (void)deltaKnowns;
  (void)deltaUnknowns;
  return fmi2Error;  // providesDirectionalDerivative="false"
}

HEMERION_FMI2_EXPORT fmi2Status fmi2SetRealInputDerivatives(fmi2Component component,
                                                            const fmi2ValueReference vr[],
                                                            std::size_t nvr,
                                                            const fmi2Integer order[],
                                                            const fmi2Real value[])
{
  (void)component;
  (void)vr;
  (void)order;
  (void)value;
  return (nvr == 0) ? fmi2OK : fmi2Error;  // canInterpolateInputs defaults to false
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetRealOutputDerivatives(fmi2Component component,
                                                             const fmi2ValueReference vr[],
                                                             std::size_t nvr,
                                                             const fmi2Integer order[],
                                                             fmi2Real value[])
{
  (void)component;
  (void)vr;
  (void)order;
  (void)value;
  return (nvr == 0) ? fmi2OK : fmi2Error;  // maxOutputDerivativeOrder defaults to 0
}
