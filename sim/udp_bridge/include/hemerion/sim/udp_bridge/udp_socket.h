// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file udp_socket.h
/// @brief Cross-platform connected IPv4 UDP socket, host-only.
///
/// Wraps socket()+bind()+connect() on POSIX and Winsock behind one RAII type
/// -- every other udp_bridge type only needs "send a datagram to the peer" /
/// "receive a datagram from the peer with a timeout", not the platform API
/// used to get there. connect() on a UDP socket sets a default destination
/// and (on both POSIX and Winsock) filters out datagrams from any sender
/// other than that peer, which is exactly the point-to-point channel this
/// bridge needs. The platform handles live in the .cpp; this header stays
/// free of `<winsock2.h>`/`<sys/socket.h>` so it is safe to include from
/// either side.
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <string>

namespace hemerion::sim::udp_bridge
{

/// @brief RAII handle on a bound, connected IPv4 UDP socket, host-only.
///
/// Neither copyable nor default-constructible: a handle either owns a live
/// socket or does not exist. Use create() to get one.
class UdpSocket
{
public:
  /// Binds to `local_address:local_port` and connects to
  /// `peer_address:peer_port`. Addresses must be numeric IPv4 (e.g.
  /// "127.0.0.1" or "0.0.0.0") -- no DNS resolution is performed. Fails
  /// (returns std::nullopt) if the local port is already bound by another
  /// socket on this host, or if either address fails to parse.
  ///
  /// Pass 0 as `local_port` to let the OS choose a free one, and read it back
  /// with local_port(). Prefer that to picking a number: a port free on one
  /// machine may be administratively excluded on another. For two sockets that
  /// must talk to each other, use create_pair().
  [[nodiscard]] static std::optional<UdpSocket> create(const std::string& local_address,
                                                       std::uint16_t local_port,
                                                       const std::string& peer_address,
                                                       std::uint16_t peer_port);

  /// @brief Binds a mutually connected pair on `address`, on ports the OS
  /// chooses.
  ///
  /// Two sockets that must exchange datagrams cannot both be made by
  /// create(): that binds *and* connects in one step, connect() filters out
  /// every sender but the peer, and neither socket's port is known until it is
  /// bound. This binds both first, reads back what the OS assigned, and
  /// connects each to the other -- so there is no interval in which a port is
  /// known but unowned, which is what a probe-then-reuse approach would leave
  /// open.
  ///
  /// Prefer it to a hardcoded port pair. A port that is free on one machine
  /// may be administratively excluded on another -- Windows reserves ranges
  /// dynamically for Hyper-V and WSL, and a bind inside one fails with
  /// "permission denied" rather than "in use", which looks nothing like a port
  /// clash while debugging.
  ///
  /// @param address Numeric IPv4 to bind both sockets to; loopback for a
  ///        channel that never reaches a network interface.
  /// @return The pair, or std::nullopt if the address fails to parse or the OS
  ///         refuses a socket.
  [[nodiscard]] static std::optional<std::pair<UdpSocket, UdpSocket>> create_pair(const std::string& address);

  /// @brief The port this socket is bound to.
  ///
  /// The value the OS assigned when create() was called with port 0, and the
  /// port that was asked for otherwise. Fixed for the socket's lifetime.
  [[nodiscard]] std::uint16_t local_port() const { return local_port_; }

  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;
  UdpSocket(UdpSocket&& other) noexcept;
  UdpSocket& operator=(UdpSocket&& other) noexcept;
  ~UdpSocket();

  /// Sends exactly `size` bytes to the connected peer in a single datagram.
  /// Returns false if the OS rejected the send outright (e.g. size exceeds
  /// the path MTU); true otherwise -- UDP gives no delivery confirmation
  /// beyond that, so a true return does not mean the peer received it.
  [[nodiscard]] bool send(const void* data, std::size_t size) const;

  /// Blocks until a datagram from the connected peer arrives, `timeout`
  /// elapses, or the socket errors. Returns the number of bytes received, or
  /// std::nullopt on timeout or error.
  [[nodiscard]] std::optional<std::size_t> receive(void* buffer,
                                                   std::size_t buffer_size,
                                                   std::chrono::milliseconds timeout) const;

private:
  UdpSocket() = default;
  void reset() noexcept;

  /// The bound port, read back from the OS after bind() so that port 0 --
  /// "any free port" -- is answerable.
  std::uint16_t local_port_ = 0;

  // SOCKET on Windows is an unsigned integer handle (not a file descriptor),
  // so it is stored as a plain integer here to keep this header free of
  // <winsock2.h>; see udp_socket.cpp for the cast back to SOCKET.
#if defined(_WIN32)
  std::uintptr_t handle_ = static_cast<std::uintptr_t>(~0ULL);  ///< matches INVALID_SOCKET
#else
  int handle_ = -1;
#endif
};

}  // namespace hemerion::sim::udp_bridge
