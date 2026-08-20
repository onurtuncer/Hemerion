// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file tool_args.h
/// @brief Strict command-line number parsing shared by the sim/i2c_shm tools.
///
/// std::atoi and friends have no failure mode: they return 0 for text that is
/// not a number at all and silently wrap a value too large for the
/// destination. Both matter here, because the results are quiet rather than
/// loud -- `--port 70000` narrows to 4464 and the bridge listens somewhere the
/// far end never looks, `--port abc` becomes 0 and the kernel picks an
/// arbitrary ephemeral port. The firmware then reads 0xFF forever and reports
/// a missing part, with nothing anywhere pointing back at the argument.
///
/// Host-only, like everything under sim/.

#ifndef HEMERION_SIM_I2C_SHM_TOOLS_TOOL_ARGS_H
#define HEMERION_SIM_I2C_SHM_TOOLS_TOOL_ARGS_H

#include <cerrno>
#include <charconv>
#include <cstdlib>
#include <string_view>
#include <system_error>

namespace hemerion::sim::i2c_shm::tools
{

/// Parses an integer, rejecting trailing garbage and anything that does not
/// fit `T` -- so the destination type *is* the range check (a std::uint16_t
/// port refuses 70000 rather than truncating it).
template <typename T>
[[nodiscard]] bool parse_number(const char* text, T& out)
{
  const std::string_view view(text);
  const char* const end = view.data() + view.size();
  T value{};
  const auto result = std::from_chars(view.data(), end, value);
  if (result.ec != std::errc{} || result.ptr != end)
  {
    return false;
  }
  out = value;
  return true;
}

/// Floating-point overload. std::from_chars for double needs a libstdc++ new
/// enough that these host tools should not assume it; strtod with an endptr
/// check is the same contract without the dependency.
[[nodiscard]] inline bool parse_number(const char* text, double& out)
{
  errno = 0;
  char* end = nullptr;
  const double value = std::strtod(text, &end);
  if (end == text || *end != '\0' || errno == ERANGE)
  {
    return false;
  }
  out = value;
  return true;
}

}  // namespace hemerion::sim::i2c_shm::tools

#endif  // HEMERION_SIM_I2C_SHM_TOOLS_TOOL_ARGS_H
