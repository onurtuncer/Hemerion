// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file udpReceiver.cpp
/// @brief Implements the platform-specific (Winsock/Berkeley) receive-only
/// UDP socket declared in udpReceiver.hpp.

#include "udpReceiver.hpp"

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <utility>

namespace hemerion::examples::rocket_gps_ecos
{

namespace
{

#if defined(_WIN32)
constexpr std::uintptr_t kInvalidHandle = static_cast<std::uintptr_t>(~0ULL);  // INVALID_SOCKET

SOCKET to_native(std::uintptr_t handle) { return static_cast<SOCKET>(handle); }

// One WSAStartup/WSACleanup pair for the process lifetime -- same pattern as
// modules/sensors' udpSender.cpp and sim/udp_bridge's udp_socket.cpp.
struct WinsockInit
{
  WinsockInit()
  {
    WSADATA wsa_data;
    WSAStartup(MAKEWORD(2, 2), &wsa_data);
  }
  WinsockInit(const WinsockInit&) = delete;
  WinsockInit& operator=(const WinsockInit&) = delete;
  WinsockInit(WinsockInit&&) = delete;
  WinsockInit& operator=(WinsockInit&&) = delete;
  ~WinsockInit() { WSACleanup(); }
};

void ensure_winsock_ready() { static WinsockInit init; }
#else
constexpr int kInvalidHandle = -1;
#endif

}  // namespace

std::optional<UdpReceiver> UdpReceiver::create(std::uint16_t local_port)
{
#if defined(_WIN32)
  ensure_winsock_ready();
#endif

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(local_port);
  local_addr.sin_addr.s_addr = htonl(INADDR_ANY);

#if defined(_WIN32)
  SOCKET native = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (native == INVALID_SOCKET)
  {  // NOLINT(modernize-use-integer-sign-comparison)
    return std::nullopt;
  }
  if (bind(native, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) ==  // NOLINT(*-reinterpret-cast)
      SOCKET_ERROR)
  {
    closesocket(native);
    return std::nullopt;
  }

  UdpReceiver result;
  result.handle_ = static_cast<std::uintptr_t>(native);
  return result;
#else
  int native = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (native < 0)
  {
    return std::nullopt;
  }
  if (bind(native, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) != 0)
  {
    close(native);
    return std::nullopt;
  }

  UdpReceiver result;
  result.handle_ = native;
  return result;
#endif
}

UdpReceiver::UdpReceiver(UdpReceiver&& other) noexcept { *this = std::move(other); }

UdpReceiver& UdpReceiver::operator=(UdpReceiver&& other) noexcept
{
  if (this != &other)
  {
    reset();
    handle_ = other.handle_;
    other.handle_ = kInvalidHandle;
  }
  return *this;
}

UdpReceiver::~UdpReceiver() { reset(); }

void UdpReceiver::reset() noexcept
{
  if (handle_ != kInvalidHandle)
  {
#if defined(_WIN32)
    closesocket(to_native(handle_));
#else
    close(handle_);
#endif
    handle_ = kInvalidHandle;
  }
}

std::optional<std::size_t> UdpReceiver::receive(void* buffer,
                                                std::size_t buffer_size,
                                                std::chrono::milliseconds timeout) const
{
  if (handle_ == kInvalidHandle)
  {
    return std::nullopt;
  }

  timeval tv{};
  tv.tv_sec = static_cast<long>(timeout.count() / 1000);
  tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);

  fd_set read_set;
  FD_ZERO(&read_set);
#if defined(_WIN32)
  const SOCKET native = to_native(handle_);
  FD_SET(native, &read_set);
  const int ready = select(0 /*ignored on Winsock*/, &read_set, nullptr, nullptr, &tv);
#else
  const int native = handle_;
  FD_SET(native, &read_set);
  const int ready = select(native + 1, &read_set, nullptr, nullptr, &tv);
#endif
  if (ready <= 0)
  {
    return std::nullopt;  // timeout or select error
  }

#if defined(_WIN32)
  const int received = recvfrom(native, static_cast<char*>(buffer), static_cast<int>(buffer_size), 0, nullptr, nullptr);
  if (received == SOCKET_ERROR)
  {
    return std::nullopt;
  }
#else
  const ssize_t received = recvfrom(native, buffer, buffer_size, 0, nullptr, nullptr);
  if (received < 0)
  {
    return std::nullopt;
  }
#endif
  return static_cast<std::size_t>(received);
}

}  // namespace hemerion::examples::rocket_gps_ecos
