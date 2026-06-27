// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// Hemerion/gps/fmu/fmi2Minimal.hpp
//
// Local, minimal subset of the public-domain FMI 2.0 standard headers
// (fmi2TypesPlatform.h / fmi2FunctionTypes.h), hand-written because
// vendor/fmi4c is not checked out yet and cmake/fmi4c.cmake does not exist
// (see cmake/README.md -- it's documented, not built). Only the typedefs
// and the fmi2CallbackFunctions struct this FMU's Co-Simulation slave
// actually needs are reproduced here; this is not a full copy of the FMI
// headers and must not be used as one. Once vendor/fmi4c lands, fmuMain.cpp
// should switch to its headers and this file should go away.
// ------------------------------------------------------------------------------
#pragma once

#include <cstddef>

extern "C" {

using fmi2Component = void*;
using fmi2ComponentEnvironment = void*;
using fmi2FMUstate = void*;
using fmi2ValueReference = unsigned int;
using fmi2Real = double;
using fmi2Integer = int;
using fmi2Boolean = int;
using fmi2Char = char;
using fmi2String = const fmi2Char*;
using fmi2Byte = char;

enum fmi2Status { fmi2OK = 0, fmi2Warning = 1, fmi2Discard = 2, fmi2Error = 3, fmi2Fatal = 4, fmi2Pending = 5 };

enum fmi2Type { fmi2ModelExchange = 0, fmi2CoSimulation = 1 };

enum fmi2StatusKind { fmi2DoStepStatus = 0, fmi2PendingStatus = 1, fmi2LastSuccessfulTime = 2, fmi2Terminated = 3 };

using fmi2CallbackLogger = void (*)(fmi2ComponentEnvironment componentEnvironment, fmi2String instanceName,
                                     fmi2Status status, fmi2String category, fmi2String message, ...);
using fmi2CallbackAllocateMemory = void* (*)(std::size_t nobj, std::size_t size);
using fmi2CallbackFreeMemory = void (*)(void* obj);
using fmi2StepFinished = void (*)(fmi2ComponentEnvironment componentEnvironment, fmi2Status status);

struct fmi2CallbackFunctions {
  fmi2CallbackLogger logger;
  fmi2CallbackAllocateMemory allocateMemory;
  fmi2CallbackFreeMemory freeMemory;
  fmi2StepFinished stepFinished;
  fmi2ComponentEnvironment componentEnvironment;
};

}  // extern "C"

#if defined(_WIN32)
#define HEMERION_FMI2_EXPORT extern "C" __declspec(dllexport)
#else
#define HEMERION_FMI2_EXPORT extern "C" __attribute__((visibility("default")))
#endif
