// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/comms/can_frame.h
//
// CAN 2.0A/2.0B frame representation and validated builders. This is a pure
// framing layer: it has no dependency on a CAN peripheral or the HAL
// abstraction (cmake/hemerion_hal/ has no can.h yet), so it only models the
// frame shape and validates identifiers/length against the wire format. A
// later HAL-backed driver passes CanFrame values to/from the peripheral.
// ------------------------------------------------------------------------------
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace hemerion::comms {

// Standard (11-bit) and extended (29-bit) CAN identifiers share the same
// CanFrame representation; the `extended` flag selects which range applies.
inline constexpr std::uint32_t kCanStandardIdMax = 0x7FFU;
inline constexpr std::uint32_t kCanExtendedIdMax = 0x1FFFFFFFU;
inline constexpr std::uint8_t kCanMaxDlc = 8U;

enum class CanFrameError : std::uint8_t {
  kNone,
  kIdOutOfRange,
  kDlcOutOfRange,
};

struct CanFrame {
  std::uint32_t id = 0;
  std::array<std::uint8_t, kCanMaxDlc> data{};
  std::uint8_t dlc = 0;
  bool extended = false;
  bool remote_request = false;
};

// Validates `id`/`extended`/`length` and, on success, fills `out_frame` with
// a copy of the first `length` bytes of `payload`. Leaves `out_frame`
// unmodified on error. `payload` may be nullptr only when `length` is 0.
[[nodiscard]] CanFrameError make_frame(std::uint32_t id, bool extended, const std::uint8_t* payload,
                                        std::uint8_t length, bool remote_request,
                                        CanFrame& out_frame);

// Re-validates an already-constructed frame, e.g. one decoded from a wire
// buffer rather than built via make_frame().
[[nodiscard]] CanFrameError validate_frame(const CanFrame& frame);

}  // namespace hemerion::comms
