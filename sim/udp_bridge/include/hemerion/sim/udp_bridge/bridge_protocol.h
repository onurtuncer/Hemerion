// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file bridge_protocol.h
/// @brief Datagram wire format for the UDP co-simulation bridge.
///
/// Wire format exchanged between the FMI master (sim/fmi) and an
/// out-of-process Aetherion over UDP. Unlike shm_bridge's BridgeRegion, there
/// is no shared memory to reinterpret -- each step is a standalone datagram,
/// so StepPacket must be exactly as large sent as received and trivially
/// copyable, with no pointers and no padding-sensitive layout assumptions
/// across the two binaries. Both ends are expected to run on hosts of the
/// same architecture (no byte-swapping is done here), matching the no-byte-
/// swap assumption shm_bridge already makes for its shared region.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace hemerion::sim::udp_bridge
{

/// Layout version stamped into StepPacket::protocol_version. Bump it on any
/// change to the packet's field order, types, or sizes; a receiver drops
/// datagrams carrying a different value.
inline constexpr std::uint32_t kProtocolVersion = 1;

/// Capacity of a ChannelFrame. Fixed rather than dynamic so a StepPacket is
/// one trivially copyable datagram of known size.
inline constexpr std::size_t kMaxChannelValues = 64;

/// Datagram kinds. Unlike shm_bridge's StepPhase, there is no single shared
/// state machine -- each side just sends the packet kind appropriate to its
/// role and the other side classifies what it receives.
enum class PacketType : std::uint32_t
{
  kInputs = 1,
  kOutputs = 2,
  kShutdown = 3,
};

/// Fixed-capacity vector of FMI variable values, in the order
/// sim/fmi/topology.cpp wired them up. Same shape as shm_bridge's
/// ChannelFrame, duplicated here so this library has no dependency on
/// shm_bridge.
struct ChannelFrame
{
  std::array<double, kMaxChannelValues> values{};  ///< Channel values; only the first `count` are meaningful.
  std::uint32_t count = 0;                         ///< Number of channels wired up, at most kMaxChannelValues.
};

static_assert(std::is_trivially_copyable_v<ChannelFrame>, "ChannelFrame is sent byte-for-byte inside a UDP datagram");

/// The full datagram payload. step_index is stamped by whichever side sends
/// kInputs (the master) and echoed back by the side sending kOutputs (the
/// peer), letting either side notice a stale or duplicate datagram if one
/// ever gets re-delivered.
struct StepPacket
{
  std::uint32_t protocol_version = kProtocolVersion;  ///< kProtocolVersion the sender was built with.
  PacketType type = PacketType::kInputs;              ///< Which side sent this and what `channel` holds.
  std::uint64_t step_index = 0;                       ///< Step this datagram belongs to; echoed by the peer.
  double sim_time_s = 0.0;                            ///< Simulation time at the start of the step, in seconds.
  double dt_s = 0.0;                                  ///< Length of the step, in seconds.
  ChannelFrame channel;                               ///< Inputs on a kInputs packet, outputs on a kOutputs one.
};

static_assert(std::is_trivially_copyable_v<StepPacket>, "StepPacket is sent byte-for-byte as a single UDP datagram");

}  // namespace hemerion::sim::udp_bridge
