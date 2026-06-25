// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_udp_bridge.cpp
//
// Native unit test for the lockstep handshake protocol over UDP. Run by
// CTest under the test-native preset on both Linux and Windows. Both sides
// (master and peer) bind to 127.0.0.1 in this single process -- enough to
// exercise the protocol without spawning a real Aetherion process.
// ------------------------------------------------------------------------------
#include <cassert>
#include <chrono>
#include <cstdio>
#include <optional>

#include "hemerion/sim/udp_bridge/udp_bridge.h"

using hemerion::sim::udp_bridge::ChannelFrame;
using hemerion::sim::udp_bridge::UdpBridge;
using hemerion::sim::udp_bridge::WaitResult;
using namespace std::chrono_literals;

namespace {

constexpr const char* kLoopback = "127.0.0.1";

ChannelFrame make_frame(std::initializer_list<double> values) {
  ChannelFrame frame;
  frame.count = 0;
  for (double v : values) {
    frame.values[frame.count++] = v;
  }
  return frame;
}

void test_round_trip_step() {
  std::optional<UdpBridge> master = UdpBridge::create_master(kLoopback, 58201, kLoopback, 58202);
  assert(master.has_value());
  std::optional<UdpBridge> peer = UdpBridge::create_peer(kLoopback, 58202, kLoopback, 58201);
  assert(peer.has_value());

  assert(master->step_index() == 0);

  const ChannelFrame inputs = make_frame({1.0, 2.0, 3.0});
  master->post_inputs(/*sim_time_s=*/0.5, /*dt_s=*/0.01, inputs);
  assert(master->step_index() == 1);

  ChannelFrame received_inputs;
  double sim_time_s = 0.0;
  double dt_s = 0.0;
  const WaitResult input_result = peer->wait_for_inputs(received_inputs, sim_time_s, dt_s, 1000ms);
  assert(input_result == WaitResult::kReady);
  assert(sim_time_s == 0.5);
  assert(dt_s == 0.01);
  assert(received_inputs.count == 3);
  assert(received_inputs.values[0] == 1.0);
  assert(received_inputs.values[2] == 3.0);
  assert(peer->step_index() == 1);  // mirrored from the received packet

  const ChannelFrame outputs = make_frame({9.0, 8.0});
  peer->post_outputs(outputs);

  ChannelFrame received_outputs;
  const WaitResult output_result = master->wait_for_outputs(received_outputs, 1000ms);
  assert(output_result == WaitResult::kReady);
  assert(received_outputs.count == 2);
  assert(received_outputs.values[0] == 9.0);
  assert(received_outputs.values[1] == 8.0);

  // A second step must work the same way -- nothing latches after one round
  // trip the way a shared-memory phase would.
  master->post_inputs(1.0, 0.01, inputs);
  assert(master->step_index() == 2);
}

void test_wait_for_inputs_times_out_when_nothing_posted() {
  std::optional<UdpBridge> peer = UdpBridge::create_peer(kLoopback, 58203, kLoopback, 58204);
  assert(peer.has_value());

  ChannelFrame inputs;
  double sim_time_s = 0.0;
  double dt_s = 0.0;
  const WaitResult result = peer->wait_for_inputs(inputs, sim_time_s, dt_s, 50ms);
  assert(result == WaitResult::kTimedOut);
}

void test_shutdown_unblocks_peer() {
  std::optional<UdpBridge> master = UdpBridge::create_master(kLoopback, 58205, kLoopback, 58206);
  assert(master.has_value());
  std::optional<UdpBridge> peer = UdpBridge::create_peer(kLoopback, 58206, kLoopback, 58205);
  assert(peer.has_value());

  master->request_shutdown();

  ChannelFrame inputs;
  double sim_time_s = 0.0;
  double dt_s = 0.0;
  const WaitResult result = peer->wait_for_inputs(inputs, sim_time_s, dt_s, 1000ms);
  assert(result == WaitResult::kShutdown);
}

}  // namespace

int main() {
  test_round_trip_step();
  test_wait_for_inputs_times_out_when_nothing_posted();
  test_shutdown_unblocks_peer();

  std::puts("test_udp_bridge: all checks passed");
  return 0;
}
