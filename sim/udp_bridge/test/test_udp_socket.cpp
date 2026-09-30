// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_udp_socket.cpp
//
// Native unit test for the cross-platform connected-UDP-socket RAII wrapper.
// Run by CTest under the test-native preset on both Linux and Windows. All
// sockets bind to 127.0.0.1 so this never touches a real network interface.
//
// **No hardcoded ports.** This file used to name nine of them, 58101 through
// 58109, and on 2026-09-30 every one of them fell inside a range Windows had
// reserved (58035-58134). bind() returned WSAEACCES -- "permission denied",
// which looks nothing like a port clash -- and four tests failed for a reason
// with nothing to do with what they test. Those ranges are assigned
// dynamically and move on reboot, so the suite had passed two days earlier;
// CI is Linux and never saw it at all.
//
// Port 0 asks the OS for a port it knows is free, which is the only way to
// pick one that is correct on every machine. UdpSocket::create_pair() exists
// for the cases needing two sockets that talk to each other, which port 0
// alone cannot arrange -- see its documentation.
// ------------------------------------------------------------------------------
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <utility>

#include "hemerion/sim/udp_bridge/udp_socket.h"

using hemerion::sim::udp_bridge::UdpSocket;
using namespace std::chrono_literals;

namespace
{

constexpr const char* kLoopback = "127.0.0.1";

void test_send_then_receive_round_trip()
{
  std::optional<std::pair<UdpSocket, UdpSocket>> pair = UdpSocket::create_pair(kLoopback);
  assert(pair.has_value());
  UdpSocket& a = pair->first;
  UdpSocket& b = pair->second;

  const char message[] = "hello";
  assert(a.send(message, sizeof(message)));

  char buffer[sizeof(message)] = {};
  const std::optional<std::size_t> received = b.receive(buffer, sizeof(buffer), 1000ms);
  assert(received.has_value());
  assert(*received == sizeof(message));
  assert(std::memcmp(buffer, message, sizeof(message)) == 0);
}

void test_receive_times_out_when_nothing_sent()
{
  std::optional<std::pair<UdpSocket, UdpSocket>> pair = UdpSocket::create_pair(kLoopback);
  assert(pair.has_value());

  char buffer[16];
  const std::optional<std::size_t> received = pair->first.receive(buffer, sizeof(buffer), 50ms);
  assert(!received.has_value());
}

void test_create_fails_if_local_port_already_bound()
{
  // Asking the OS for a port and then asking for that same one is a stronger
  // test than naming a number and hoping it was free: the first bind proves
  // the port was available, so the second failing can only be the clash.
  std::optional<UdpSocket> first = UdpSocket::create(kLoopback, 0, kLoopback, 1);
  assert(first.has_value());
  assert(first->local_port() != 0);

  std::optional<UdpSocket> second = UdpSocket::create(kLoopback, first->local_port(), kLoopback, 1);
  assert(!second.has_value());
}

void test_port_zero_is_resolved_to_a_real_port()
{
  std::optional<UdpSocket> socket = UdpSocket::create(kLoopback, 0, kLoopback, 1);
  assert(socket.has_value());

  // The point of local_port(): 0 means "any", and the caller needs to know
  // which one it got. A socket that reported 0 back would be useless as a
  // peer address.
  const std::uint16_t assigned = socket->local_port();
  assert(assigned != 0);

  // Stable for the socket's lifetime, and unchanged by a move.
  assert(socket->local_port() == assigned);
  UdpSocket moved(std::move(*socket));
  assert(moved.local_port() == assigned);
}

void test_a_pair_gets_two_distinct_ports()
{
  std::optional<std::pair<UdpSocket, UdpSocket>> pair = UdpSocket::create_pair(kLoopback);
  assert(pair.has_value());
  assert(pair->first.local_port() != 0);
  assert(pair->second.local_port() != 0);
  assert(pair->first.local_port() != pair->second.local_port());
}

void test_move_construct_transfers_ownership()
{
  std::optional<std::pair<UdpSocket, UdpSocket>> pair = UdpSocket::create_pair(kLoopback);
  assert(pair.has_value());

  UdpSocket moved(std::move(pair->first));

  const char message[] = "ok";
  assert(moved.send(message, sizeof(message)));

  char buffer[sizeof(message)] = {};
  const std::optional<std::size_t> received = pair->second.receive(buffer, sizeof(buffer), 1000ms);
  assert(received.has_value());
  assert(std::memcmp(buffer, message, sizeof(message)) == 0);
}

}  // namespace

int main()
{
  test_send_then_receive_round_trip();
  test_receive_times_out_when_nothing_sent();
  test_create_fails_if_local_port_already_bound();
  test_port_zero_is_resolved_to_a_real_port();
  test_a_pair_gets_two_distinct_ports();
  test_move_construct_transfers_ownership();

  std::puts("test_udp_socket: all checks passed");
  return 0;
}
