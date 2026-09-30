// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file gpsLatencyLine.hpp
/// @brief Holds a receiver's output back by its processing latency.
///
/// **What this models.** A GNSS receiver does not emit the solution for an
/// epoch at that epoch. It closes the epoch, runs its own filter, formats the
/// message and clocks it out of a UART: by the time the first byte of a
/// NAV-PVT reaches the flight computer, the position it describes is 50 to
/// 200 ms old. The receiver is not wrong about this and does not conceal it --
/// a real one publishes its own time of validity -- but a consumer that treats
/// the fix as current is fusing a measurement of where the vehicle *was* with
/// an IMU that is telling it where the vehicle *is*.
///
/// At 172 m/s, the F-16 examples' trim speed, 100 ms of latency is **17
/// metres** of position error, pointing along the velocity vector. That is an
/// order of magnitude larger than the receiver's own noise, entirely
/// systematic, and invisible to any amount of averaging. It is the reason
/// delayed-measurement handling exists in a filter, and until this existed
/// nothing in the tree produced it.
///
/// **Why a queue rather than an offset.** Subtracting a constant from a
/// timestamp would model a receiver that *reports* a stale time, which is not
/// what happens. What happens is that a correct solution for epoch *k* is
/// delivered during epoch *k+n*. So the frame is built when the epoch closes
/// and released later, which is also what makes it visible to the flight
/// computer as an arrival time later than the truth it describes -- exactly
/// the gap `host_time_s` records.
///
/// **Zero latency is the identity.** A frame pushed with a release time of
/// `now` is due immediately, so the default reproduces a receiver with no
/// latency at all, frame for frame.
///
/// Host-only, like the rest of the fmu/ subtree.

#pragma once

#include <cstddef>
#include <deque>
#include <utility>

namespace hemerion::sensors::gps::fmu
{

/// @brief A first-in, first-out delay line for receiver output.
///
/// @tparam Frame Whatever the emitter produces; copied in and out. Templated
///         so this can be tested without a UBX encoder, and so a different
///         wire format would not need a second delay line.
template <typename Frame>
class GpsLatencyLine
{
public:
  /// @brief Queues a frame for release.
  ///
  /// @param due_time_s When the frame becomes deliverable [s], on the
  ///        simulation clock: the epoch's own time plus the latency.
  /// @param frame The encoded message.
  void push(double due_time_s, Frame frame) { queue_.emplace_back(due_time_s, std::move(frame)); }

  /// @brief Takes the next frame that is due, if any.
  ///
  /// Called in a loop: a step longer than the latency can make several frames
  /// due at once, and dropping the older ones would model a receiver that
  /// discards solutions rather than one that delays them.
  ///
  /// @param now_s Current simulation time [s].
  /// @param out   Receives the frame when one is due.
  /// @return true if `out` was written.
  [[nodiscard]] bool pop_due(double now_s, Frame& out)
  {
    if (queue_.empty() || queue_.front().first > now_s)
    {
      return false;
    }
    out = std::move(queue_.front().second);
    queue_.pop_front();
    return true;
  }

  /// How many frames are waiting. A receiver at steady state holds
  /// ceil(latency / step) of them; a number that grows without bound means the
  /// caller has stopped draining.
  [[nodiscard]] std::size_t pending() const { return queue_.size(); }

  /// @brief Discards everything in flight, for an FMI reset.
  void reset() { queue_.clear(); }

private:
  std::deque<std::pair<double, Frame>> queue_;
};

}  // namespace hemerion::sensors::gps::fmu
