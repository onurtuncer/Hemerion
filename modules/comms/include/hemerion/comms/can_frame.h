// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file can_frame.h
/// @brief CAN 2.0A/2.0B frame representation and validated builders.
///
/// This is a pure framing layer: it has no dependency on a CAN peripheral or
/// the HAL abstraction (cmake/hemerion_hal/ has no can.h yet), so it only
/// models the frame shape and validates identifiers/length against the wire
/// format. A later HAL-backed driver passes CanFrame values to/from the
/// peripheral.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

/// @namespace hemerion::comms
/// @brief Communication-link framing and staging primitives (CAN today).
namespace hemerion::comms {

/// Highest valid standard (11-bit) CAN identifier. Standard and extended
/// identifiers share the same CanFrame representation; the
/// CanFrame::extended flag selects which range applies.
inline constexpr std::uint32_t kCanStandardIdMax = 0x7FFU;
/// Highest valid extended (29-bit) CAN identifier.
inline constexpr std::uint32_t kCanExtendedIdMax = 0x1FFFFFFFU;
/// Maximum data length code (payload bytes) of a classic CAN frame.
inline constexpr std::uint8_t kCanMaxDlc = 8U;

/// Result of validating a CanFrame against the CAN 2.0 wire format.
enum class CanFrameError : std::uint8_t {
  kNone,           ///< Frame is valid.
  kIdOutOfRange,   ///< Identifier exceeds the range selected by `extended`.
  kDlcOutOfRange,  ///< Data length code exceeds kCanMaxDlc.
};

/// One classic CAN 2.0A/2.0B frame, independent of any peripheral register
/// layout.
struct CanFrame {
  std::uint32_t id = 0;                          ///< 11-bit or 29-bit identifier, per `extended`.
  std::array<std::uint8_t, kCanMaxDlc> data{};   ///< Payload; only the first `dlc` bytes are meaningful.
  std::uint8_t dlc = 0;                          ///< Data length code, 0..kCanMaxDlc.
  bool extended = false;                         ///< True selects the 29-bit identifier range.
  bool remote_request = false;                   ///< True marks a remote transmission request (RTR) frame.
};

/// @brief Builds a validated CanFrame from raw parts.
///
/// Validates `id`/`extended`/`length` and, on success, fills `out_frame` with
/// a copy of the first `length` bytes of `payload`. Leaves `out_frame`
/// unmodified on error.
///
/// @param id             CAN identifier; checked against the range selected by `extended`.
/// @param extended       True for a 29-bit identifier, false for 11-bit.
/// @param payload        Source bytes to copy; may be nullptr only when `length` is 0.
/// @param length         Number of payload bytes, 0..kCanMaxDlc.
/// @param remote_request True to mark the frame as a remote transmission request.
/// @param out_frame      Receives the built frame on success; untouched on error.
/// @return CanFrameError::kNone on success, otherwise the first validation failure.
[[nodiscard]] CanFrameError make_frame(std::uint32_t id, bool extended, const std::uint8_t* payload,
                                        std::uint8_t length, bool remote_request,
                                        CanFrame& out_frame);

/// @brief Re-validates an already-constructed frame, e.g. one decoded from a
/// wire buffer rather than built via make_frame().
///
/// @param frame Frame to check.
/// @return CanFrameError::kNone if the frame is valid, otherwise the first
///         validation failure.
[[nodiscard]] CanFrameError validate_frame(const CanFrame& frame);

}  // namespace hemerion::comms
