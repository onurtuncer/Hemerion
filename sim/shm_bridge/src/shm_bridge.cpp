// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
#include "hemerion/sim/shm_bridge/shm_bridge.h"

#include <new>
#include <thread>
#include <utility>

namespace hemerion::sim::shm_bridge {

namespace {

constexpr std::chrono::microseconds kSpinSleep{50};

WaitResult spin_wait_for_phase(const std::atomic<std::uint32_t>& phase, StepPhase target,
                                std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  for (;;) {
    const auto current = static_cast<StepPhase>(phase.load(std::memory_order_acquire));
    if (current == target) {
      return WaitResult::kReady;
    }
    if (current == StepPhase::kShutdownRequested) {
      return WaitResult::kShutdown;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      return WaitResult::kTimedOut;
    }
    std::this_thread::sleep_for(kSpinSleep);
  }
}

}  // namespace

ShmBridge::ShmBridge(ShmSegment segment) : segment_(std::move(segment)) {}

std::optional<ShmBridge> ShmBridge::create_master(const std::string& name) {
  std::optional<ShmSegment> segment = ShmSegment::create(name, sizeof(BridgeRegion));
  if (!segment) {
    return std::nullopt;
  }
  // The master is the only side that ever constructs the region -- the peer
  // always finds it already constructed and just reinterprets the bytes.
  new (segment->data()) BridgeRegion();
  return ShmBridge(std::move(*segment));
}

std::optional<ShmBridge> ShmBridge::open_peer(const std::string& name) {
  std::optional<ShmSegment> segment = ShmSegment::open_existing(name, sizeof(BridgeRegion));
  if (!segment) {
    return std::nullopt;
  }
  if (static_cast<const BridgeRegion*>(segment->data())->protocol_version != kProtocolVersion) {
    return std::nullopt;
  }
  return ShmBridge(std::move(*segment));
}

BridgeRegion& ShmBridge::region() noexcept { return *static_cast<BridgeRegion*>(segment_.data()); }

const BridgeRegion& ShmBridge::region() const noexcept { return *static_cast<const BridgeRegion*>(segment_.data()); }

void ShmBridge::post_inputs(double sim_time_s, double dt_s, const ChannelFrame& inputs) {
  BridgeRegion& r = region();
  r.sim_time_s = sim_time_s;
  r.dt_s = dt_s;
  r.master_to_plant = inputs;
  r.step_index.fetch_add(1, std::memory_order_relaxed);
  r.phase.store(static_cast<std::uint32_t>(StepPhase::kInputsPosted), std::memory_order_release);
}

WaitResult ShmBridge::wait_for_outputs(ChannelFrame& outputs, std::chrono::milliseconds timeout) {
  BridgeRegion& r = region();
  const WaitResult result = spin_wait_for_phase(r.phase, StepPhase::kOutputsPosted, timeout);
  if (result != WaitResult::kReady) {
    return result;
  }
  outputs = r.plant_to_master;
  r.phase.store(static_cast<std::uint32_t>(StepPhase::kIdle), std::memory_order_release);
  return WaitResult::kReady;
}

void ShmBridge::request_shutdown() {
  region().phase.store(static_cast<std::uint32_t>(StepPhase::kShutdownRequested), std::memory_order_release);
}

WaitResult ShmBridge::wait_for_inputs(ChannelFrame& inputs, double& sim_time_s, double& dt_s,
                                       std::chrono::milliseconds timeout) {
  BridgeRegion& r = region();
  const WaitResult result = spin_wait_for_phase(r.phase, StepPhase::kInputsPosted, timeout);
  if (result != WaitResult::kReady) {
    return result;
  }
  inputs = r.master_to_plant;
  sim_time_s = r.sim_time_s;
  dt_s = r.dt_s;
  return WaitResult::kReady;
}

void ShmBridge::post_outputs(const ChannelFrame& outputs) {
  BridgeRegion& r = region();
  r.plant_to_master = outputs;
  r.phase.store(static_cast<std::uint32_t>(StepPhase::kOutputsPosted), std::memory_order_release);
}

std::uint64_t ShmBridge::step_index() const { return region().step_index.load(std::memory_order_relaxed); }

}  // namespace hemerion::sim::shm_bridge
