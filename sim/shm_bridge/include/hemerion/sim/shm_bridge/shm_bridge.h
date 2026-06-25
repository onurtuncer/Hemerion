// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/sim/shm_bridge/shm_bridge.h
//
// Lockstep handshake on top of ShmSegment + BridgeRegion. The FMI master
// (sim/fmi) owns the segment and drives StepPhase::kIdle -> kInputsPosted;
// the locally running Aetherion process attaches as the peer and drives
// kInputsPosted -> kOutputsPosted. Synchronization is a spin-wait on the
// shared phase atomic with a short sleep backoff -- acceptable for a
// same-host bridge where steps complete in microseconds to low
// milliseconds; revisit with a named semaphore/event if profiling ever shows
// the spin costing real CPU.
// ------------------------------------------------------------------------------
#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "hemerion/sim/shm_bridge/bridge_protocol.h"
#include "hemerion/sim/shm_bridge/shm_segment.h"

namespace hemerion::sim::shm_bridge {

enum class WaitResult {
  kReady,
  kShutdown,
  kTimedOut,
};

class ShmBridge {
 public:
  // Creates the segment; called once per simulation run by the FMI master.
  [[nodiscard]] static std::optional<ShmBridge> create_master(const std::string& name);

  // Attaches to a segment already created by create_master(); called by the
  // Aetherion process. Returns std::nullopt if the master hasn't created it
  // yet, or if its protocol_version doesn't match -- callers should treat
  // both as retryable rather than fatal.
  [[nodiscard]] static std::optional<ShmBridge> open_peer(const std::string& name);

  ShmBridge(const ShmBridge&) = delete;
  ShmBridge& operator=(const ShmBridge&) = delete;
  ShmBridge(ShmBridge&&) noexcept = default;
  ShmBridge& operator=(ShmBridge&&) noexcept = default;
  ~ShmBridge() = default;

  // -- master side -------------------------------------------------------
  // Posts the next step's inputs and advances kIdle -> kInputsPosted.
  void post_inputs(double sim_time_s, double dt_s, const ChannelFrame& inputs);
  // Blocks until the peer posts outputs (kOutputsPosted), a shutdown is
  // observed, or `timeout` elapses. On kReady, consumes the outputs and
  // advances kOutputsPosted -> kIdle.
  [[nodiscard]] WaitResult wait_for_outputs(ChannelFrame& outputs, std::chrono::milliseconds timeout);
  // Sets kShutdownRequested so any peer blocked in wait_for_inputs() returns
  // WaitResult::kShutdown instead of timing out.
  void request_shutdown();

  // -- peer side -----------------------------------------------------------
  // Blocks until the master posts inputs (kInputsPosted), a shutdown is
  // observed, or `timeout` elapses.
  [[nodiscard]] WaitResult wait_for_inputs(ChannelFrame& inputs, double& sim_time_s, double& dt_s,
                                            std::chrono::milliseconds timeout);
  // Posts this step's outputs and advances kInputsPosted -> kOutputsPosted.
  void post_outputs(const ChannelFrame& outputs);

  // Number of post_inputs() calls so far this run. Visible from either side.
  [[nodiscard]] std::uint64_t step_index() const;

 private:
  explicit ShmBridge(ShmSegment segment);

  [[nodiscard]] BridgeRegion& region() noexcept;
  [[nodiscard]] const BridgeRegion& region() const noexcept;

  ShmSegment segment_;
};

}  // namespace hemerion::sim::shm_bridge
