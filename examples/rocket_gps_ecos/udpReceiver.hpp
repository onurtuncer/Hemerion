// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file udpReceiver.hpp
/// @brief Minimal receive-only UDP socket for the example flight computer.
///
/// Counterpart to modules/sensors' UdpSender (the GPS FMU's transmit side):
/// binds a local port and receives datagrams from any sender, since the FMU
/// transmits from an OS-chosen ephemeral port -- a connected socket like
/// sim/udp_bridge's UdpSocket would filter its datagrams out. Host-only
/// example code; on real hardware these bytes arrive on a UART, not UDP.

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace hemerion::examples::rocket_gps_ecos {

/// @brief RAII wrapper over one bound, receive-only UDP socket.
///
/// Move-only; the socket is closed on destruction.
class UdpReceiver {
 public:
  /// @brief Binds a UDP socket to 0.0.0.0:`local_port`.
  ///
  /// @param local_port Port to listen on.
  /// @return The bound receiver, or std::nullopt if socket creation or bind
  ///         failed (e.g. the port is already in use).
  [[nodiscard]] static std::optional<UdpReceiver> create(std::uint16_t local_port);

  UdpReceiver(const UdpReceiver&) = delete;
  UdpReceiver& operator=(const UdpReceiver&) = delete;
  /// Takes over `other`'s socket, leaving `other` empty.
  UdpReceiver(UdpReceiver&& other) noexcept;
  /// Closes any owned socket, then takes over `other`'s.
  UdpReceiver& operator=(UdpReceiver&& other) noexcept;
  /// Closes the socket if still owned.
  ~UdpReceiver();

  /// @brief Blocks until a datagram arrives or `timeout` elapses.
  ///
  /// @param buffer      Destination for the datagram payload.
  /// @param buffer_size Capacity of `buffer` [bytes]; longer datagrams are truncated.
  /// @param timeout     Maximum time to wait.
  /// @return Number of bytes received, or std::nullopt on timeout or error.
  [[nodiscard]] std::optional<std::size_t> receive(void* buffer, std::size_t buffer_size,
                                                    std::chrono::milliseconds timeout) const;

 private:
  UdpReceiver() = default;
  void reset() noexcept;

#if defined(_WIN32)
  std::uintptr_t handle_ = static_cast<std::uintptr_t>(~0ULL);  // matches INVALID_SOCKET
#else
  int handle_ = -1;
#endif
};

}  // namespace hemerion::examples::rocket_gps_ecos
