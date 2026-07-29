// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file udpSender.cpp
/// @brief Implements the platform-specific (Winsock/Berkeley) UDP socket
/// wrapper declared in udpSender.hpp.

#include "Hemerion/gps/fmu/udpSender.hpp"

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
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <cstdlib>
#include <utility>

namespace hemerion::sensors::gps::fmu
{

namespace
{

#if defined(_WIN32)
constexpr std::uintptr_t kInvalidHandle = static_cast<std::uintptr_t>(~0ULL);  // INVALID_SOCKET

SOCKET to_native(std::uintptr_t handle) { return static_cast<SOCKET>(handle); }
std::uintptr_t from_native(SOCKET handle) { return static_cast<std::uintptr_t>(handle); }

// One WSAStartup/WSACleanup pair for the process lifetime -- see
// sim/udp_bridge/udp_socket.cpp's WinsockInit for the same pattern and
// rationale; duplicated here rather than shared because this header tree
// must not depend on sim/.
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

bool parse_ipv4(const std::string& address, std::uint16_t port, sockaddr_in& out)
{
  out = {};
  out.sin_family = AF_INET;
  out.sin_port = htons(port);
  return inet_pton(AF_INET, address.c_str(), &out.sin_addr) == 1;
}

}  // namespace

std::optional<UdpSender> UdpSender::create(const std::string& peer_address, std::uint16_t peer_port)
{
#if defined(_WIN32)
  ensure_winsock_ready();
#endif

  sockaddr_in peer_addr{};
  if (!parse_ipv4(peer_address, peer_port, peer_addr))
  {
    return std::nullopt;
  }

#if defined(_WIN32)
  SOCKET native = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (native == INVALID_SOCKET)
  {  // NOLINT(modernize-use-integer-sign-comparison)
    return std::nullopt;
  }
  if (connect(native, reinterpret_cast<sockaddr*>(&peer_addr), sizeof(peer_addr)) ==  // NOLINT(*-reinterpret-cast)
      SOCKET_ERROR)
  {
    closesocket(native);
    return std::nullopt;
  }

  UdpSender result;
  result.handle_ = from_native(native);
  return result;
#else
  int native = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (native < 0)
  {
    return std::nullopt;
  }
  if (connect(native, reinterpret_cast<sockaddr*>(&peer_addr), sizeof(peer_addr)) != 0)
  {
    close(native);
    return std::nullopt;
  }

  UdpSender result;
  result.handle_ = native;
  return result;
#endif
}

std::optional<UdpSender> UdpSender::create_from_env(const char* host_variable,
                                                   const char* port_variable,
                                                   const std::string& default_host,
                                                   std::uint16_t default_port)
{
  // std::getenv is flagged deprecated by the Windows SDK headers (in favour of _dupenv_s) purely as an MSVC CRT
  // "insecure function" nag, not a real portability issue -- std::getenv is the standard, portable way to do this.
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#elif defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
  const char* host_env = std::getenv(host_variable);
  const char* port_env = std::getenv(port_variable);
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(_MSC_VER)
#pragma warning(pop)
#endif

  std::uint16_t port = default_port;
  if (port_env != nullptr)
  {
    // A garbage or out-of-range setting falls back to the default rather than silently wrapping to some other port.
    const long parsed = std::strtol(port_env, nullptr, 10);
    if (parsed > 0 && parsed <= 65535)
    {
      port = static_cast<std::uint16_t>(parsed);
    }
  }

  return create((host_env != nullptr) ? std::string(host_env) : default_host, port);
}

UdpSender::UdpSender(UdpSender&& other) noexcept { *this = std::move(other); }

UdpSender& UdpSender::operator=(UdpSender&& other) noexcept
{
  if (this != &other)
  {
    reset();
    handle_ = other.handle_;
    other.handle_ = kInvalidHandle;
  }
  return *this;
}

UdpSender::~UdpSender() { reset(); }

void UdpSender::reset() noexcept
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

bool UdpSender::send(const void* data, std::size_t size) const
{
#if defined(_WIN32)
  const int sent = ::send(to_native(handle_), static_cast<const char*>(data), static_cast<int>(size), 0);
  return sent >= 0 && static_cast<std::size_t>(sent) == size;
#else
  const ssize_t sent = ::send(handle_, data, size, 0);
  return sent >= 0 && static_cast<std::size_t>(sent) == size;
#endif
}

}  // namespace hemerion::sensors::gps::fmu
