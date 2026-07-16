// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/sim/shm_bridge/bridge_protocol.h
//
// Wire format shared between the FMI master (sim/fmi) and a locally running
// Aetherion process: both sides map this struct over the same bytes of a
// named shared-memory segment (see shm_segment.h), so it must stay a single
// stable, standard-layout block -- no virtual functions, no pointers, no
// type whose representation could differ across the two binaries. The phase
// field is the handshake: each side advances it in one direction only (see
// shm_bridge.h for the state machine), and the surrounding data fields are
// only valid to read once the reader has observed the phase transition that
// the writer made after filling them in.
// ------------------------------------------------------------------------------
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace hemerion::sim::shm_bridge
{

inline constexpr std::uint32_t kProtocolVersion = 1;
inline constexpr std::size_t kMaxChannelValues = 64;

// Step handshake. Each transition is driven by exactly one side:
//   kIdle           -> kInputsPosted        driven by the master
//   kInputsPosted   -> kOutputsPosted       driven by the plant (Aetherion)
//   kOutputsPosted  -> kIdle                driven by the master
// kShutdownRequested is set by the master at any time and is a terminal
// state for the peer's wait loops -- it never transitions back to kIdle.
enum class StepPhase : std::uint32_t
{
  kIdle = 0,
  kInputsPosted = 1,
  kOutputsPosted = 2,
  kShutdownRequested = 3,
};

// Fixed-capacity vector of FMI variable values, in the order
// sim/fmi/topology.cpp wired them up.
struct ChannelFrame
{
  std::array<double, kMaxChannelValues> values{};
  std::uint32_t count = 0;
};

static_assert(std::is_trivially_copyable_v<ChannelFrame>,
              "ChannelFrame is copied byte-for-byte across the shm boundary");

// The full shared-memory region. Constructed exactly once, in place, by the
// master (see ShmBridge::create_master); the peer only ever reinterprets an
// already-constructed region -- never reconstructs it.
struct BridgeRegion
{
  std::uint32_t protocol_version = kProtocolVersion;
  std::atomic<std::uint32_t> phase{ static_cast<std::uint32_t>(StepPhase::kIdle) };
  std::atomic<std::uint64_t> step_index{ 0 };
  double sim_time_s = 0.0;
  double dt_s = 0.0;
  ChannelFrame master_to_plant;  // FMI master -> Aetherion: actuator/command channels
  ChannelFrame plant_to_master;  // Aetherion -> FMI master: true-state output channels
};

static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
              "BridgeRegion::phase must be lock-free to be safely shared across processes");
static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
              "BridgeRegion::step_index must be lock-free to be safely shared across processes");

}  // namespace hemerion::sim::shm_bridge
