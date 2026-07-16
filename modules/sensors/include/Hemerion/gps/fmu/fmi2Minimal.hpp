// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file fmi2Minimal.hpp
/// @brief Local, minimal subset of the public-domain FMI 2.0 standard
/// headers (fmi2TypesPlatform.h / fmi2FunctionTypes.h).
///
/// Hand-written because vendor/fmi4c is not checked out yet and
/// cmake/fmi4c.cmake does not exist (see cmake/README.md -- it's
/// documented, not built). Only the typedefs and the fmi2CallbackFunctions
/// struct this FMU's Co-Simulation slave actually needs are reproduced
/// here; this is not a full copy of the FMI headers and must not be used as
/// one. Once vendor/fmi4c lands, fmuMain.cpp should switch to its headers
/// and this file should go away.

#pragma once

#include <cstddef>

extern "C" {

using fmi2Component = void*;              ///< Opaque handle to one FMU instance.
using fmi2ComponentEnvironment = void*;   ///< Opaque importer-side context passed back in callbacks.
using fmi2FMUstate = void*;               ///< Opaque serialized-state handle (unused by this FMU).
using fmi2ValueReference = unsigned int;  ///< Index identifying one scalar variable.
using fmi2Real = double;                  ///< FMI real scalar type.
using fmi2Integer = int;                  ///< FMI integer scalar type.
using fmi2Boolean = int;                  ///< FMI boolean scalar type (0/1).
using fmi2Char = char;                    ///< FMI character type.
using fmi2String = const fmi2Char*;       ///< FMI string type (null-terminated).
using fmi2Byte = char;                    ///< FMI raw byte type.

/// Status every fmi2* API function returns to the importer.
enum fmi2Status
{
  fmi2OK = 0,
  fmi2Warning = 1,
  fmi2Discard = 2,
  fmi2Error = 3,
  fmi2Fatal = 4,
  fmi2Pending = 5
};

/// Which FMI 2.0 interface an instance is created for.
enum fmi2Type
{
  fmi2ModelExchange = 0,
  fmi2CoSimulation = 1
};

/// Selector for fmi2GetStatus-family queries.
enum fmi2StatusKind
{
  fmi2DoStepStatus = 0,
  fmi2PendingStatus = 1,
  fmi2LastSuccessfulTime = 2,
  fmi2Terminated = 3
};

/// Importer-supplied logging callback.
using fmi2CallbackLogger = void (*)(fmi2ComponentEnvironment componentEnvironment,
                                    fmi2String instanceName,
                                    fmi2Status status,
                                    fmi2String category,
                                    fmi2String message,
                                    ...);
/// Importer-supplied allocator (calloc-shaped).
using fmi2CallbackAllocateMemory = void* (*)(std::size_t nobj, std::size_t size);
/// Importer-supplied deallocator for fmi2CallbackAllocateMemory memory.
using fmi2CallbackFreeMemory = void (*)(void* obj);
/// Importer notification for asynchronous fmi2DoStep completion (unused here).
using fmi2StepFinished = void (*)(fmi2ComponentEnvironment componentEnvironment, fmi2Status status);

/// Callback table the importer hands to fmi2Instantiate().
struct fmi2CallbackFunctions
{
  fmi2CallbackLogger logger;                      ///< Never null.
  fmi2CallbackAllocateMemory allocateMemory;      ///< Never null.
  fmi2CallbackFreeMemory freeMemory;              ///< Never null.
  fmi2StepFinished stepFinished;                  ///< May be null (synchronous stepping).
  fmi2ComponentEnvironment componentEnvironment;  ///< Passed back verbatim in every callback.
};

}  // extern "C"

/// Marks a function as part of the FMU's exported FMI 2.0 C API.
#if defined(_WIN32)
#define HEMERION_FMI2_EXPORT extern "C" __declspec(dllexport)
#else
#define HEMERION_FMI2_EXPORT extern "C" __attribute__((visibility("default")))
#endif
