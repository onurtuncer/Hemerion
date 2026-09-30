// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
#include "hemerion/sim/udp_bridge/udp_socket.h"

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
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include <utility>

namespace hemerion::sim::udp_bridge
{

namespace
{

#if defined(_WIN32)
constexpr std::uintptr_t kInvalidHandle = static_cast<std::uintptr_t>(~0ULL);  // INVALID_SOCKET

SOCKET to_native(std::uintptr_t handle) { return static_cast<SOCKET>(handle); }
std::uintptr_t from_native(SOCKET handle) { return static_cast<std::uintptr_t>(handle); }

// WSAStartup/WSACleanup must bracket all Winsock use. Calling WSAStartup
// repeatedly is safe (it ref-counts internally), but every call needs a
// matching WSACleanup -- a single process-lifetime instance avoids having to
// thread that bookkeeping through every UdpSocket.
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

/// The port a socket ended up bound to. Asked of the OS rather than echoed
/// back from the request, because the request may have been 0 -- "give me any
/// free port" -- which is the whole reason this exists.
#if defined(_WIN32)
bool bound_port(SOCKET native, std::uint16_t& out)
#else
bool bound_port(int native, std::uint16_t& out)
#endif
{
  sockaddr_in bound{};
#if defined(_WIN32)
  int length = static_cast<int>(sizeof(bound));
  if (getsockname(native, reinterpret_cast<sockaddr*>(&bound), &length) == SOCKET_ERROR)  // NOLINT(*-reinterpret-cast)
  {
    return false;
  }
#else
  socklen_t length = sizeof(bound);
  if (getsockname(native, reinterpret_cast<sockaddr*>(&bound), &length) != 0)  // NOLINT(*-reinterpret-cast)
  {
    return false;
  }
#endif
  out = ntohs(bound.sin_port);
  return true;
}

bool parse_ipv4(const std::string& address, std::uint16_t port, sockaddr_in& out)
{
  out = {};
  out.sin_family = AF_INET;
  out.sin_port = htons(port);
  return inet_pton(AF_INET, address.c_str(), &out.sin_addr) == 1;
}

}  // namespace

std::optional<UdpSocket> UdpSocket::create(const std::string& local_address,
                                           std::uint16_t local_port,
                                           const std::string& peer_address,
                                           std::uint16_t peer_port)
{
#if defined(_WIN32)
  ensure_winsock_ready();
#endif

  sockaddr_in local_addr{};
  if (!parse_ipv4(local_address, local_port, local_addr))
  {
    return std::nullopt;
  }
  sockaddr_in peer_addr{};
  if (!parse_ipv4(peer_address, peer_port, peer_addr))
  {
    return std::nullopt;
  }

#if defined(_WIN32)
  SOCKET native = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  // INVALID_SOCKET is (SOCKET)(~0); the cast already makes both sides
  // unsigned, but clang-tidy's sign-comparison check still flags the literal
  // inside the macro -- there's no portable alternative to this Winsock idiom.
  if (native == INVALID_SOCKET)
  {  // NOLINT(modernize-use-integer-sign-comparison)
    return std::nullopt;
  }
  // bind()/connect() require sockaddr*; reinterpret_cast from sockaddr_in* is
  // the mandated way to call these BSD-socket-shaped APIs, on both Winsock
  // and POSIX.
  if (bind(native, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) ==  // NOLINT(*-reinterpret-cast)
      SOCKET_ERROR)
  {
    closesocket(native);
    return std::nullopt;
  }
  if (connect(native, reinterpret_cast<sockaddr*>(&peer_addr), sizeof(peer_addr)) ==  // NOLINT(*-reinterpret-cast)
      SOCKET_ERROR)
  {
    closesocket(native);
    return std::nullopt;
  }

  UdpSocket result;
  result.handle_ = from_native(native);
  if (!bound_port(native, result.local_port_))
  {
    return std::nullopt;
  }
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
  if (connect(native, reinterpret_cast<sockaddr*>(&peer_addr), sizeof(peer_addr)) != 0)
  {
    close(native);
    return std::nullopt;
  }

  UdpSocket result;
  result.handle_ = native;
  if (!bound_port(native, result.local_port_))
  {
    return std::nullopt;
  }
  return result;
#endif
}

std::optional<std::pair<UdpSocket, UdpSocket>> UdpSocket::create_pair(const std::string& address)
{
#if defined(_WIN32)
  // inet_pton is in ws2_32 and wants WSAStartup to have run. create() would
  // guarantee that by being called first; the address is validated before it
  // now, so this has to.
  ensure_winsock_ready();
#endif

  // Validated once, up front. Parsing it again after both create() calls had
  // succeeded with it would be a branch that cannot be taken.
  sockaddr_in peer{};
  if (!parse_ipv4(address, 0, peer))
  {
    return std::nullopt;
  }

  // Bind both to port 0 first and only then connect them to each other. The
  // alternative -- discovering two free ports with throwaway sockets and
  // reopening them -- leaves an interval in which the ports are known but
  // owned by nobody, and a fix written to stop a test being flaky should not
  // open a new window for it.
  //
  // The placeholder peer port on the first socket is why this cannot simply be
  // two create() calls: create() binds and connects together, and the second
  // socket's port does not exist until it is bound.
  std::optional<UdpSocket> first = create(address, 0, address, 1);
  if (!first.has_value())
  {
    return std::nullopt;
  }
  std::optional<UdpSocket> second = create(address, 0, address, first->local_port());
  if (!second.has_value())
  {
    return std::nullopt;
  }

  // Now that `second` exists, point `first` at it. Re-connecting a UDP socket
  // is defined on both stacks and leaves the binding untouched.
  peer.sin_port = htons(second->local_port());
#if defined(_WIN32)
  if (connect(to_native(first->handle_),
              reinterpret_cast<sockaddr*>(&peer),  // NOLINT(*-reinterpret-cast)
              sizeof(peer)) == SOCKET_ERROR)
#else
  if (connect(first->handle_, reinterpret_cast<sockaddr*>(&peer), sizeof(peer)) != 0)  // NOLINT(*-reinterpret-cast)
#endif
  {
    return std::nullopt;
  }

  return std::make_pair(std::move(*first), std::move(*second));
}

UdpSocket::UdpSocket(UdpSocket&& other) noexcept { *this = std::move(other); }

UdpSocket& UdpSocket::operator=(UdpSocket&& other) noexcept
{
  if (this != &other)
  {
    reset();
    handle_ = other.handle_;
    local_port_ = other.local_port_;
    other.handle_ = kInvalidHandle;
    other.local_port_ = 0;
  }
  return *this;
}

UdpSocket::~UdpSocket() { reset(); }

void UdpSocket::reset() noexcept
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

bool UdpSocket::send(const void* data, std::size_t size) const
{
#if defined(_WIN32)
  const int sent = ::send(to_native(handle_), static_cast<const char*>(data), static_cast<int>(size), 0);
  return sent >= 0 && std::cmp_equal(sent, size);
#else
  const ssize_t sent = ::send(handle_, data, size, 0);
  return sent >= 0 && std::cmp_equal(sent, size);
#endif
}

std::optional<std::size_t> UdpSocket::receive(void* buffer,
                                              std::size_t buffer_size,
                                              std::chrono::milliseconds timeout) const
{
#if defined(_WIN32)
  const auto timeout_ms = static_cast<DWORD>(timeout.count());
  // setsockopt() takes its value as a raw byte buffer regardless of the
  // option's real type; reinterpret_cast to const char* is how every
  // Winsock/BSD-socket caller does this.
  setsockopt(to_native(handle_),
             SOL_SOCKET,
             SO_RCVTIMEO,
             reinterpret_cast<const char*>(&timeout_ms),  // NOLINT(*-reinterpret-cast)
             sizeof(timeout_ms));

  const int received = recv(to_native(handle_), static_cast<char*>(buffer), static_cast<int>(buffer_size), 0);
  if (received < 0)
  {
    return std::nullopt;  // timeout (WSAETIMEDOUT) or socket error -- both are retryable to the caller
  }
  return static_cast<std::size_t>(received);
#else
  struct timeval timeout_tv;
  timeout_tv.tv_sec = static_cast<time_t>(timeout.count() / 1000);
  timeout_tv.tv_usec = static_cast<suseconds_t>((timeout.count() % 1000) * 1000);
  setsockopt(handle_, SOL_SOCKET, SO_RCVTIMEO, &timeout_tv, sizeof(timeout_tv));

  const ssize_t received = recv(handle_, buffer, buffer_size, 0);
  if (received < 0)
  {
    return std::nullopt;  // timeout (EAGAIN/EWOULDBLOCK) or socket error -- both are retryable to the caller
  }
  return static_cast<std::size_t>(received);
#endif
}

}  // namespace hemerion::sim::udp_bridge
