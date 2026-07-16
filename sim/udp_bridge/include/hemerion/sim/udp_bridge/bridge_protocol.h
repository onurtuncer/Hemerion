// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/sim/udp_bridge/bridge_protocol.h
//
// Wire format exchanged between the FMI master (sim/fmi) and an
// out-of-process Aetherion over UDP. Unlike shm_bridge's BridgeRegion, there
// is no shared memory to reinterpret -- each step is a standalone datagram,
// so StepPacket must be exactly as large sent as received and trivially
// copyable, with no pointers and no padding-sensitive layout assumptions
// across the two binaries. Both ends are expected to run on hosts of the
// same architecture (no byte-swapping is done here), matching the no-byte-
// swap assumption shm_bridge already makes for its shared region.
// ------------------------------------------------------------------------------
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace hemerion::sim::udp_bridge
{

inline constexpr std::uint32_t kProtocolVersion = 1;
inline constexpr std::size_t kMaxChannelValues = 64;

// Datagram kinds. Unlike shm_bridge's StepPhase, there is no single shared
// state machine -- each side just sends the packet kind appropriate to its
// role and the other side classifies what it receives.
enum class PacketType : std::uint32_t
{
  kInputs = 1,
  kOutputs = 2,
  kShutdown = 3,
};

// Fixed-capacity vector of FMI variable values, in the order
// sim/fmi/topology.cpp wired them up. Same shape as shm_bridge's
// ChannelFrame, duplicated here so this library has no dependency on
// shm_bridge.
struct ChannelFrame
{
  std::array<double, kMaxChannelValues> values{};
  std::uint32_t count = 0;
};

static_assert(std::is_trivially_copyable_v<ChannelFrame>, "ChannelFrame is sent byte-for-byte inside a UDP datagram");

// The full datagram payload. step_index is stamped by whichever side sends
// kInputs (the master) and echoed back by the side sending kOutputs (the
// peer), letting either side notice a stale or duplicate datagram if one
// ever gets re-delivered.
struct StepPacket
{
  std::uint32_t protocol_version = kProtocolVersion;
  PacketType type = PacketType::kInputs;
  std::uint64_t step_index = 0;
  double sim_time_s = 0.0;
  double dt_s = 0.0;
  ChannelFrame channel;
};

static_assert(std::is_trivially_copyable_v<StepPacket>, "StepPacket is sent byte-for-byte as a single UDP datagram");

}  // namespace hemerion::sim::udp_bridge
