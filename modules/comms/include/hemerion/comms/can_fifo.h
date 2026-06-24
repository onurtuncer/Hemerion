// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/comms/can_fifo.h
//
// Fixed-capacity CanFrame queue for TX/RX staging ahead of a CAN peripheral.
// Wraps etl::queue so callers never see ETL's overflow/underflow assertions:
// push()/pop() report failure via the return value instead, matching the
// [[nodiscard]] error-code convention used across this module.
// ------------------------------------------------------------------------------
#pragma once

#include <cstddef>

#include "etl/queue.h"
#include "hemerion/comms/can_frame.h"

namespace hemerion::comms {

template <std::size_t Capacity>
class CanFifo {
 public:
  // Returns false without modifying the queue if it is already full.
  [[nodiscard]] bool push(const CanFrame& frame) {
    if (queue_.full()) {
      return false;
    }
    queue_.push(frame);
    return true;
  }

  // Returns false without modifying `out_frame` if the queue is empty.
  [[nodiscard]] bool pop(CanFrame& out_frame) {
    if (queue_.empty()) {
      return false;
    }
    out_frame = queue_.front();
    queue_.pop();
    return true;
  }

  [[nodiscard]] bool empty() const { return queue_.empty(); }
  [[nodiscard]] bool full() const { return queue_.full(); }
  [[nodiscard]] std::size_t size() const { return queue_.size(); }
  [[nodiscard]] static constexpr std::size_t capacity() { return Capacity; }

 private:
  etl::queue<CanFrame, Capacity> queue_;
};

}  // namespace hemerion::comms
