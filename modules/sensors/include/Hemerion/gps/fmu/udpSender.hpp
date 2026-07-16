// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file udpSender.hpp
/// @brief Minimal send-only UDP socket, host-only.
///
/// Winsock/Berkeley sockets behind one RAII type. This is intentionally a
/// small duplicate of sim/udp_bridge/udp_socket.h's platform-handling shape
/// rather than a dependency on it: modules/README.md forbids modules
/// depending on sim/ targets, and this is the only operation the GPS FMU
/// needs -- fire UBX frames at a fixed peer (typically Renode's emulated
/// UART4 backend), no receive path.

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace hemerion::sensors::gps::fmu
{

/// @brief RAII wrapper over one connected, send-only UDP socket.
///
/// Move-only; the socket is closed on destruction.
class UdpSender
{
public:
  /// @brief Connects a UDP socket to `peer_address:peer_port`.
  ///
  /// @param peer_address Numeric IPv4 address only, no DNS resolution.
  /// @param peer_port    Destination UDP port.
  /// @return The connected sender, or std::nullopt if socket creation or
  ///         connect failed. The local port is left to the OS to choose.
  [[nodiscard]] static std::optional<UdpSender> create(const std::string& peer_address, std::uint16_t peer_port);

  UdpSender(const UdpSender&) = delete;
  UdpSender& operator=(const UdpSender&) = delete;
  /// Takes over `other`'s socket, leaving `other` empty.
  UdpSender(UdpSender&& other) noexcept;
  /// Closes any owned socket, then takes over `other`'s.
  UdpSender& operator=(UdpSender&& other) noexcept;
  /// Closes the socket if still owned.
  ~UdpSender();

  /// @brief Sends exactly `size` bytes as one datagram.
  ///
  /// @param data Bytes to send.
  /// @param size Datagram length [bytes].
  /// @return False if the OS rejected the send outright; true otherwise --
  ///         UDP gives no delivery confirmation beyond that.
  [[nodiscard]] bool send(const void* data, std::size_t size) const;

private:
  UdpSender() = default;
  void reset() noexcept;

#if defined(_WIN32)
  std::uintptr_t handle_ = static_cast<std::uintptr_t>(~0ULL);  // matches INVALID_SOCKET
#else
  int handle_ = -1;
#endif
};

}  // namespace hemerion::sensors::gps::fmu
