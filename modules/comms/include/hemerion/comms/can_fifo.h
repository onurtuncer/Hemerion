// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file can_fifo.h
/// @brief Fixed-capacity CanFrame queue for TX/RX staging ahead of a CAN
/// peripheral.
///
/// Wraps etl::queue so callers never see ETL's overflow/underflow assertions:
/// push()/pop() report failure via the return value instead, matching the
/// [[nodiscard]] error-code convention used across this module.

#pragma once

#include <cstddef>

#include "etl/queue.h"
#include "hemerion/comms/can_frame.h"

namespace hemerion::comms {

/// @brief Statically-allocated FIFO of CanFrame values.
///
/// @tparam Capacity Maximum number of frames the FIFO can hold.
template <std::size_t Capacity>
class CanFifo {
 public:
  /// @brief Appends `frame` to the back of the queue.
  /// @param frame Frame to enqueue (copied).
  /// @return False without modifying the queue if it is already full.
  [[nodiscard]] bool push(const CanFrame& frame) {
    if (queue_.full()) {
      return false;
    }
    queue_.push(frame);
    return true;
  }

  /// @brief Removes the front frame and copies it into `out_frame`.
  /// @param out_frame Receives the dequeued frame on success.
  /// @return False without modifying `out_frame` if the queue is empty.
  [[nodiscard]] bool pop(CanFrame& out_frame) {
    if (queue_.empty()) {
      return false;
    }
    out_frame = queue_.front();
    queue_.pop();
    return true;
  }

  /// True if the queue holds no frames.
  [[nodiscard]] bool empty() const { return queue_.empty(); }
  /// True if the queue holds Capacity frames.
  [[nodiscard]] bool full() const { return queue_.full(); }
  /// Number of frames currently queued.
  [[nodiscard]] std::size_t size() const { return queue_.size(); }
  /// Compile-time maximum number of frames.
  [[nodiscard]] static constexpr std::size_t capacity() { return Capacity; }

 private:
  etl::queue<CanFrame, Capacity> queue_;
};

}  // namespace hemerion::comms
