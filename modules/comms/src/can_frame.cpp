// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
#include "hemerion/comms/can_frame.h"

#include <algorithm>

namespace hemerion::comms {

CanFrameError validate_frame(const CanFrame& frame) {
  const std::uint32_t id_max = frame.extended ? kCanExtendedIdMax : kCanStandardIdMax;
  if (frame.id > id_max) {
    return CanFrameError::kIdOutOfRange;
  }
  if (frame.dlc > kCanMaxDlc) {
    return CanFrameError::kDlcOutOfRange;
  }
  return CanFrameError::kNone;
}

CanFrameError make_frame(std::uint32_t id, bool extended, const std::uint8_t* payload,
                          std::uint8_t length, bool remote_request, CanFrame& out_frame) {
  CanFrame candidate;
  candidate.id = id;
  candidate.extended = extended;
  candidate.dlc = length;
  candidate.remote_request = remote_request;

  const CanFrameError error = validate_frame(candidate);
  if (error != CanFrameError::kNone) {
    return error;
  }

  if (length > 0) {
    std::copy_n(payload, length, candidate.data.begin());
  }

  out_frame = candidate;
  return CanFrameError::kNone;
}

}  // namespace hemerion::comms
