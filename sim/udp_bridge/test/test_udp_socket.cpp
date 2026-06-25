// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_udp_socket.cpp
//
// Native unit test for the cross-platform connected-UDP-socket RAII wrapper.
// Run by CTest under the test-native preset on both Linux and Windows. All
// sockets bind to 127.0.0.1 so this never touches a real network interface;
// each test uses its own port pair to avoid clashing with the others.
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

namespace {

constexpr const char* kLoopback = "127.0.0.1";

void test_send_then_receive_round_trip() {
  std::optional<UdpSocket> a = UdpSocket::create(kLoopback, 58101, kLoopback, 58102);
  assert(a.has_value());
  std::optional<UdpSocket> b = UdpSocket::create(kLoopback, 58102, kLoopback, 58101);
  assert(b.has_value());

  const char message[] = "hello";
  assert(a->send(message, sizeof(message)));

  char buffer[sizeof(message)] = {};
  const std::optional<std::size_t> received = b->receive(buffer, sizeof(buffer), 1000ms);
  assert(received.has_value());
  assert(*received == sizeof(message));
  assert(std::memcmp(buffer, message, sizeof(message)) == 0);
}

void test_receive_times_out_when_nothing_sent() {
  std::optional<UdpSocket> a = UdpSocket::create(kLoopback, 58103, kLoopback, 58104);
  assert(a.has_value());

  char buffer[16];
  const std::optional<std::size_t> received = a->receive(buffer, sizeof(buffer), 50ms);
  assert(!received.has_value());
}

void test_create_fails_if_local_port_already_bound() {
  std::optional<UdpSocket> first = UdpSocket::create(kLoopback, 58105, kLoopback, 58106);
  assert(first.has_value());

  std::optional<UdpSocket> second = UdpSocket::create(kLoopback, 58105, kLoopback, 58107);
  assert(!second.has_value());
}

void test_move_construct_transfers_ownership() {
  std::optional<UdpSocket> a = UdpSocket::create(kLoopback, 58108, kLoopback, 58109);
  assert(a.has_value());
  std::optional<UdpSocket> b = UdpSocket::create(kLoopback, 58109, kLoopback, 58108);
  assert(b.has_value());

  UdpSocket moved(std::move(*a));

  const char message[] = "ok";
  assert(moved.send(message, sizeof(message)));

  char buffer[sizeof(message)] = {};
  const std::optional<std::size_t> received = b->receive(buffer, sizeof(buffer), 1000ms);
  assert(received.has_value());
  assert(std::memcmp(buffer, message, sizeof(message)) == 0);
}

}  // namespace

int main() {
  test_send_then_receive_round_trip();
  test_receive_times_out_when_nothing_sent();
  test_create_fails_if_local_port_already_bound();
  test_move_construct_transfers_ownership();

  std::puts("test_udp_socket: all checks passed");
  return 0;
}
