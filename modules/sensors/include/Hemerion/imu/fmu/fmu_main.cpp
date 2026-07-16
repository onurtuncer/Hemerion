// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file fmu_main.cpp
/// @brief FMI 2.0 Co-Simulation slave entry points for the IMU hardware
/// simulator.
///
/// This FMU has no FMI output variables -- its effect is a UDP side
/// channel: each fmi2DoStep() call turns the current body-frame truth
/// inputs (specific force + angular rate) into noisy raw register counts
/// (ImuNoiseModel), encodes them as Hemerion IMU raw-sample frames
/// (ImuPacketEmitter), and sends them to a fixed UDP peer (UdpSender), so
/// the real, unmodified ImuPacketParser + convert_raw_to_si() on the
/// firmware side decodes them exactly as it would bytes from a physical
/// part.
///
/// Unlike the 10 Hz GPS FMU, a real IMU samples much faster than a co-
/// simulation communication step, so the `sample_rate_hz` parameter
/// (default 100 Hz) makes each fmi2DoStep emit round(step * rate) frames.
/// The truth inputs are zero-order-held across the step (the master only
/// exchanges variables at communication points), so the sub-step samples
/// differ in noise draw and timestamp only -- the honest equivalent of
/// oversampling a plant that is itself only resolved at the communication
/// rate.
///
/// UDP destination is read once at fmi2Instantiate() from
/// HEMERION_IMU_FMU_UDP_HOST / HEMERION_IMU_FMU_UDP_PORT (defaults
/// 127.0.0.1:5763) rather than exposed as an FMI string parameter -- this
/// keeps fmi2SetString unimplemented, which is fine: it's only mandatory
/// for FMUs that declare String-typed variables in model_description.xml,
/// and this one doesn't.
///
/// model_description.xml's <fmiModelDescription guid="..."> must match
/// kExpectedGuid below exactly; fmi2Instantiate() refuses to start
/// otherwise rather than silently running against the wrong model
/// description.
///
/// fmi2Minimal.hpp and UdpSender are reused from the GPS FMU one module
/// subtree over (Hemerion/gps/fmu/) -- both are sensor-agnostic and live in
/// the same module, so duplicating them here would only invite drift.

#include "Hemerion/gps/fmu/fmi2Minimal.hpp"
#include "Hemerion/gps/fmu/udpSender.hpp"
#include "Hemerion/imu/fmu/imu_noise_model.h"
#include "Hemerion/imu/fmu/imu_packet_emitter.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>

namespace
{

using hemerion::sensors::gps::fmu::UdpSender;
using hemerion::sensors::imu::fmu::ImuNoiseModel;
using hemerion::sensors::imu::fmu::ImuPacketEmitter;
using hemerion::sensors::imu::fmu::ImuTruthSample;

constexpr fmi2ValueReference kVrSpecificForceX = 0;
constexpr fmi2ValueReference kVrSpecificForceY = 1;
constexpr fmi2ValueReference kVrSpecificForceZ = 2;
constexpr fmi2ValueReference kVrAngularRateX = 3;
constexpr fmi2ValueReference kVrAngularRateY = 4;
constexpr fmi2ValueReference kVrAngularRateZ = 5;
constexpr fmi2ValueReference kVrSampleRateHz = 6;

// Must match model_description.xml's <fmiModelDescription guid="...">.
constexpr char kExpectedGuid[] = "{6E2B9C41-8D5F-4A07-9C63-1F7E5B8A2D94}";

constexpr char kDefaultUdpHost[] = "127.0.0.1";
constexpr std::uint16_t kDefaultUdpPort = 5763;
constexpr double kDefaultSampleRateHz = 100.0;

struct ImuFmuInstance
{
  ImuNoiseModel noise_model;
  UdpSender sender;
  ImuTruthSample truth;
  double sample_rate_hz = kDefaultSampleRateHz;
  fmi2CallbackFunctions callbacks{};
  bool logging_on = false;
};

ImuFmuInstance* to_instance(fmi2Component component) { return static_cast<ImuFmuInstance*>(component); }

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
  const char* host_env = std::getenv("HEMERION_IMU_FMU_UDP_HOST");
  const char* port_env = std::getenv("HEMERION_IMU_FMU_UDP_PORT");
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

  auto* instance = new ImuFmuInstance{ ImuNoiseModel{},
                                       std::move(*sender),
                                       ImuTruthSample{},
                                       kDefaultSampleRateHz,
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
  ImuFmuInstance* instance = to_instance(component);
  instance->truth = ImuTruthSample{};
  instance->sample_rate_hz = kDefaultSampleRateHz;
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2SetReal(fmi2Component component,
                                            const fmi2ValueReference vr[],
                                            std::size_t nvr,
                                            const fmi2Real value[])
{
  ImuFmuInstance* instance = to_instance(component);
  for (std::size_t i = 0; i < nvr; ++i)
  {
    switch (vr[i])
    {
      case kVrSpecificForceX:
        instance->truth.specific_force_x_mps2 = value[i];
        break;
      case kVrSpecificForceY:
        instance->truth.specific_force_y_mps2 = value[i];
        break;
      case kVrSpecificForceZ:
        instance->truth.specific_force_z_mps2 = value[i];
        break;
      case kVrAngularRateX:
        instance->truth.angular_rate_x_rad_s = value[i];
        break;
      case kVrAngularRateY:
        instance->truth.angular_rate_y_rad_s = value[i];
        break;
      case kVrAngularRateZ:
        instance->truth.angular_rate_z_rad_s = value[i];
        break;
      case kVrSampleRateHz:
        if (value[i] > 0.0)
        {
          instance->sample_rate_hz = value[i];
        }
        break;
      default:
        return fmi2Error;
    }
  }
  return fmi2OK;
}

HEMERION_FMI2_EXPORT fmi2Status fmi2GetReal(fmi2Component component,
                                            const fmi2ValueReference vr[],
                                            std::size_t nvr,
                                            fmi2Real value[])
{
  const ImuFmuInstance* instance = to_instance(component);
  for (std::size_t i = 0; i < nvr; ++i)
  {
    switch (vr[i])
    {
      case kVrSpecificForceX:
        value[i] = instance->truth.specific_force_x_mps2;
        break;
      case kVrSpecificForceY:
        value[i] = instance->truth.specific_force_y_mps2;
        break;
      case kVrSpecificForceZ:
        value[i] = instance->truth.specific_force_z_mps2;
        break;
      case kVrAngularRateX:
        value[i] = instance->truth.angular_rate_x_rad_s;
        break;
      case kVrAngularRateY:
        value[i] = instance->truth.angular_rate_y_rad_s;
        break;
      case kVrAngularRateZ:
        value[i] = instance->truth.angular_rate_z_rad_s;
        break;
      case kVrSampleRateHz:
        value[i] = instance->sample_rate_hz;
        break;
      default:
        return fmi2Error;
    }
  }
  return fmi2OK;
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
  ImuFmuInstance* instance = to_instance(component);

  // Truth is zero-order-held over the step; emit one frame per sensor
  // sample period that elapses within it, each with a fresh noise draw and
  // its own timestamp.
  const long samples = std::lround(communicationStepSize * instance->sample_rate_hz);
  const long count = (samples > 0) ? samples : 1;
  const double sample_period_s = communicationStepSize / static_cast<double>(count);

  for (long k = 1; k <= count; ++k)
  {
    const double sample_time_s = currentCommunicationPoint + static_cast<double>(k) * sample_period_s;
    instance->truth.timestamp_us = static_cast<std::uint64_t>(sample_time_s * 1e6);
    const auto raw = instance->noise_model.apply(instance->truth);
    const auto frame = ImuPacketEmitter::encode_raw_sample(raw);
    if (!instance->sender.send(frame.data(), frame.size()))
    {
      return fmi2Warning;  // a dropped UDP datagram is a dropped sensor byte, not a simulation error
    }
  }
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
// FMU-state and derivative entry points. model_description.xml declares all
// of these capabilities false, but fmi4c (the loader Ecos uses) looks up the
// complete FMI 2.0 export table at load time and fails the whole FMU on any
// missing symbol, so a conforming slave must still export implementations
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
  return (nvr == 0) ? fmi2OK : fmi2Error;
}
