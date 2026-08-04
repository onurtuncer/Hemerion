// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
#include "hemerion/sim/i2c_shm/i2c_peripheral_endpoint.h"

#include <cstdlib>

namespace hemerion::sim::i2c_shm
{

std::string resolve_bus_name(const std::string& env_variable, const std::string& fallback)
{
  if (env_variable.empty())
  {
    return fallback;
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
  const char* value = std::getenv(env_variable.c_str());
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(_MSC_VER)
#pragma warning(pop)
#endif

  // An empty setting means "unset" rather than "a bus with no name".
  return (value != nullptr && value[0] != '\0') ? std::string(value) : fallback;
}

}  // namespace hemerion::sim::i2c_shm
