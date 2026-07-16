// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
#include "hemerion/sim/udp_bridge/udp_bridge.h"

#include <utility>

namespace hemerion::sim::udp_bridge
{

UdpBridge::UdpBridge(UdpSocket socket) : socket_(std::move(socket)) {}

std::optional<UdpBridge> UdpBridge::create_master(const std::string& local_address,
                                                  std::uint16_t local_port,
                                                  const std::string& peer_address,
                                                  std::uint16_t peer_port)
{
  std::optional<UdpSocket> socket = UdpSocket::create(local_address, local_port, peer_address, peer_port);
  if (!socket)
  {
    return std::nullopt;
  }
  return UdpBridge(std::move(*socket));
}

std::optional<UdpBridge> UdpBridge::create_peer(const std::string& local_address,
                                                std::uint16_t local_port,
                                                const std::string& peer_address,
                                                std::uint16_t peer_port)
{
  std::optional<UdpSocket> socket = UdpSocket::create(local_address, local_port, peer_address, peer_port);
  if (!socket)
  {
    return std::nullopt;
  }
  return UdpBridge(std::move(*socket));
}

WaitResult UdpBridge::wait_for_packet(PacketType expected, StepPacket& packet, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  for (;;)
  {
    const auto remaining = deadline - std::chrono::steady_clock::now();
    if (remaining <= std::chrono::milliseconds::zero())
    {
      return WaitResult::kTimedOut;
    }
    const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
    const std::optional<std::size_t> received = socket_.receive(&packet, sizeof(packet), remaining_ms);
    if (!received)
    {
      return WaitResult::kTimedOut;
    }
    if (*received != sizeof(packet) || packet.protocol_version != kProtocolVersion)
    {
      continue;  // malformed or foreign datagram -- ignore and keep waiting
    }
    if (packet.type == PacketType::kShutdown)
    {
      return WaitResult::kShutdown;
    }
    if (packet.type == expected)
    {
      return WaitResult::kReady;
    }
    // Unexpected-but-valid packet type, e.g. a delayed retransmit from a
    // prior step -- ignore and keep waiting for `expected`.
  }
}

void UdpBridge::post_inputs(double sim_time_s, double dt_s, const ChannelFrame& inputs)
{
  StepPacket packet;
  packet.type = PacketType::kInputs;
  packet.step_index = ++step_index_;
  packet.sim_time_s = sim_time_s;
  packet.dt_s = dt_s;
  packet.channel = inputs;
  (void)socket_.send(&packet, sizeof(packet));
}

WaitResult UdpBridge::wait_for_outputs(ChannelFrame& outputs, std::chrono::milliseconds timeout)
{
  StepPacket packet;
  const WaitResult result = wait_for_packet(PacketType::kOutputs, packet, timeout);
  if (result == WaitResult::kReady)
  {
    outputs = packet.channel;
  }
  return result;
}

void UdpBridge::request_shutdown()
{
  StepPacket packet;
  packet.type = PacketType::kShutdown;
  packet.step_index = step_index_;
  (void)socket_.send(&packet, sizeof(packet));
}

WaitResult
UdpBridge::wait_for_inputs(ChannelFrame& inputs, double& sim_time_s, double& dt_s, std::chrono::milliseconds timeout)
{
  StepPacket packet;
  const WaitResult result = wait_for_packet(PacketType::kInputs, packet, timeout);
  if (result == WaitResult::kReady)
  {
    inputs = packet.channel;
    sim_time_s = packet.sim_time_s;
    dt_s = packet.dt_s;
    step_index_ = packet.step_index;
  }
  return result;
}

void UdpBridge::post_outputs(const ChannelFrame& outputs)
{
  StepPacket packet;
  packet.type = PacketType::kOutputs;
  packet.step_index = step_index_;
  packet.channel = outputs;
  (void)socket_.send(&packet, sizeof(packet));
}

}  // namespace hemerion::sim::udp_bridge
