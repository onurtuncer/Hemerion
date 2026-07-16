// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/sim/udp_bridge/udp_bridge.h
//
// Lockstep handshake on top of UdpSocket + StepPacket, for when Aetherion
// runs out-of-process or on a different host from the FMI master (sim/fmi)
// -- see shm_bridge for the same-host, shared-memory version of this
// protocol. There is no shared phase atomic here: each step is a kInputs
// datagram from the master followed by a kOutputs datagram from the peer.
// UDP can drop a datagram outright, so unlike ShmBridge's spin-wait (which
// only ever waits on a peer that is merely slow), a wait_*() call here can
// time out because the *datagram itself* never arrived -- callers are
// expected to retry the corresponding post_*() call after a timeout, the
// same way an FMI master already has to handle a slow/dead peer.
// ------------------------------------------------------------------------------
#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "hemerion/sim/udp_bridge/bridge_protocol.h"
#include "hemerion/sim/udp_bridge/udp_socket.h"

namespace hemerion::sim::udp_bridge
{

enum class WaitResult
{
  kReady,
  kShutdown,
  kTimedOut,
};

class UdpBridge
{
public:
  // Binds `local_address:local_port` and targets `peer_address:peer_port`;
  // called by the FMI master. Named create_master() to stay parallel to
  // ShmBridge, but unlike ShmBridge::create_master() there is nothing to
  // exclusively own here -- UDP has no equivalent of "fails because a
  // segment with this name already exists" -- so create_master() and
  // create_peer() differ only in which side of the protocol the caller
  // intends to drive afterwards.
  [[nodiscard]] static std::optional<UdpBridge> create_master(const std::string& local_address,
                                                              std::uint16_t local_port,
                                                              const std::string& peer_address,
                                                              std::uint16_t peer_port);

  // Binds `local_address:local_port` and targets `peer_address:peer_port`;
  // called by the Aetherion process. Can be called before or after the
  // master is up -- UDP has no "does the master exist yet" check the way
  // ShmBridge::open_peer() has, since there is no shared object to attach
  // to.
  [[nodiscard]] static std::optional<UdpBridge> create_peer(const std::string& local_address,
                                                            std::uint16_t local_port,
                                                            const std::string& peer_address,
                                                            std::uint16_t peer_port);

  UdpBridge(const UdpBridge&) = delete;
  UdpBridge& operator=(const UdpBridge&) = delete;
  UdpBridge(UdpBridge&&) noexcept = default;
  UdpBridge& operator=(UdpBridge&&) noexcept = default;
  ~UdpBridge() = default;

  // -- master side ---------------------------------------------------------
  // Sends this step's inputs as a kInputs datagram and increments
  // step_index(). Does not block and gives no delivery guarantee.
  void post_inputs(double sim_time_s, double dt_s, const ChannelFrame& inputs);
  // Blocks until a kOutputs datagram arrives, a kShutdown datagram is
  // observed, or `timeout` elapses. Stray or malformed datagrams (wrong
  // protocol version, unexpected type) are silently skipped rather than
  // treated as failures, since UDP delivery order/duplication is not
  // guaranteed.
  [[nodiscard]] WaitResult wait_for_outputs(ChannelFrame& outputs, std::chrono::milliseconds timeout);
  // Sends a kShutdown datagram so a peer blocked in wait_for_inputs()
  // returns WaitResult::kShutdown instead of timing out. Like any datagram
  // here, delivery isn't guaranteed -- call again if the peer doesn't react.
  void request_shutdown();

  // -- peer side -------------------------------------------------------------
  // Blocks until a kInputs datagram arrives, a kShutdown datagram is
  // observed, or `timeout` elapses.
  [[nodiscard]] WaitResult
  wait_for_inputs(ChannelFrame& inputs, double& sim_time_s, double& dt_s, std::chrono::milliseconds timeout);
  // Sends this step's outputs as a kOutputs datagram, stamped with the
  // step_index() of the most recently received inputs.
  void post_outputs(const ChannelFrame& outputs);

  // Master: number of post_inputs() calls so far this run. Peer: step_index
  // of the most recently received inputs. Local to this process -- not
  // synchronized with the other side the way ShmBridge::step_index() is.
  [[nodiscard]] std::uint64_t step_index() const noexcept { return step_index_; }

private:
  explicit UdpBridge(UdpSocket socket);

  [[nodiscard]] WaitResult wait_for_packet(PacketType expected, StepPacket& packet, std::chrono::milliseconds timeout);

  UdpSocket socket_;
  std::uint64_t step_index_ = 0;
};

}  // namespace hemerion::sim::udp_bridge
