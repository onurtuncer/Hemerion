// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file i2c_shm_tcp_bridge.cpp
/// @brief TCP front end for the controller side of a shared-memory I2C bus.
///
/// The piece that lets an I2C *controller in another world* -- Renode's
/// emulated STM32, in another OS or VM -- drive a simulated part answering on
/// sim/i2c_shm, which is same-host shared memory by construction (see
/// sim/renode/i2c_bridge/DESIGN.md). This process owns the I2cShmController
/// end and serves whole transactions over one TCP connection:
///
///   request:  u8 target_address, u16le write_length, u16le read_length,
///             then write_length bytes
///   response: u8 result (0 = acked, 1 = address NACK, 2 = data NACK,
///             3 = bus fault), then read_length bytes when result is 0
///
/// One client at a time, served synchronously: an I2C bus has one controller,
/// and the Renode peripheral on the far end issues one transaction per
/// emulated transfer anyway. A dropped connection just returns to accept(),
/// so Renode can be restarted against a running bridge.
///
/// Host-only tool, like everything under sim/.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "hemerion/sim/i2c_shm/i2c_shm_link.h"
#include "tool_args.h"

#if defined(_WIN32)
// <winsock2.h> drags in <windows.h>, whose min/max macros collide with the
// std:: algorithms of the same name -- NOMINMAX has to be defined before it,
// not merely before the first use.
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
using SocketHandle = SOCKET;
using SockLen = int;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using SocketHandle = int;
using SockLen = socklen_t;
constexpr SocketHandle kInvalidSocket = -1;
#endif

namespace
{

using hemerion::sim::i2c_shm::I2cShmController;
using hemerion::sim::i2c_shm::tools::parse_number;
using namespace std::chrono_literals;

constexpr std::uint8_t kResultOk = 0;
constexpr std::uint8_t kResultAddressNack = 1;
constexpr std::uint8_t kResultDataNack = 2;
constexpr std::uint8_t kResultBusFault = 3;

/// A transaction a live peripheral does not answer within this window means
/// the simulated part is hung, not merely busy.
constexpr auto kTransactionTimeout = 1000ms;

void close_socket(SocketHandle socket_handle)
{
#if defined(_WIN32)
  closesocket(socket_handle);
#else
  close(socket_handle);
#endif
}

[[nodiscard]] bool recv_exact(SocketHandle socket_handle, std::uint8_t* buffer, std::size_t length)
{
  std::size_t received = 0;
  while (received < length)
  {
    const auto chunk =
        recv(socket_handle, reinterpret_cast<char*>(buffer) + received, static_cast<int>(length - received), 0);
    if (chunk <= 0)
    {
      return false;  // orderly shutdown or error: either way this client is done
    }
    received += static_cast<std::size_t>(chunk);
  }
  return true;
}

[[nodiscard]] bool send_all(SocketHandle socket_handle, const std::uint8_t* buffer, std::size_t length)
{
  std::size_t sent = 0;
  while (sent < length)
  {
    const auto chunk =
        send(socket_handle, reinterpret_cast<const char*>(buffer) + sent, static_cast<int>(length - sent), 0);
    if (chunk <= 0)
    {
      return false;
    }
    sent += static_cast<std::size_t>(chunk);
  }
  return true;
}

/// Reads and discards `length` bytes, so the stream stays framed after a
/// request this bus cannot serve.
[[nodiscard]] bool drain(SocketHandle socket_handle, std::size_t length)
{
  std::uint8_t scratch[256];
  while (length > 0)
  {
    const std::size_t chunk = std::min(length, sizeof(scratch));
    if (!recv_exact(socket_handle, scratch, chunk))
    {
      return false;
    }
    length -= chunk;
  }
  return true;
}

[[nodiscard]] std::uint8_t result_code(I2cShmController::Result result)
{
  switch (result)
  {
    case I2cShmController::Result::kOk:
      return kResultOk;
    case I2cShmController::Result::kAddressNack:
      return kResultAddressNack;
    case I2cShmController::Result::kDataNack:
      return kResultDataNack;
    default:
      return kResultBusFault;
  }
}

/// Serves one connected client until it disconnects. Returns false once the
/// shm peripheral has powered down for good, so main() can exit instead of
/// accepting clients it can only fault.
[[nodiscard]] bool serve_client(SocketHandle client, I2cShmController& controller)
{
  std::vector<std::uint8_t> write_bytes(hemerion::sim::i2c_shm::kMaxPhaseBytes);
  std::vector<std::uint8_t> read_bytes(hemerion::sim::i2c_shm::kMaxPhaseBytes);

  for (;;)
  {
    std::uint8_t header[5];
    if (!recv_exact(client, header, sizeof(header)))
    {
      return true;  // client gone; the bus may still be alive for the next one
    }
    const std::uint8_t address = header[0];
    const std::size_t write_length = static_cast<std::size_t>(header[1]) | (static_cast<std::size_t>(header[2]) << 8);
    const std::size_t read_length = static_cast<std::size_t>(header[3]) | (static_cast<std::size_t>(header[4]) << 8);

    if (write_length > write_bytes.size() || read_length > read_bytes.size())
    {
      // Too long for this bus, but not a framing error -- the header still
      // says how many bytes follow. Draining them and answering with the
      // fault code the protocol already defines keeps the link usable;
      // returning here instead left the client blocked on a reply that never
      // came and then tore down the connection, for a condition
      // I2cShmController::transaction itself reports as recoverable.
      std::fprintf(stderr,
                   "[i2c_bridge] phase too long (write %zu, read %zu, max %zu) -- answering bus fault\n",
                   write_length,
                   read_length,
                   hemerion::sim::i2c_shm::kMaxPhaseBytes);
      if (!drain(client, write_length))
      {
        return true;
      }
      const std::uint8_t fault = kResultBusFault;
      if (!send_all(client, &fault, 1))
      {
        return true;
      }
      continue;
    }
    if (write_length > 0 && !recv_exact(client, write_bytes.data(), write_length))
    {
      return true;
    }

    const I2cShmController::Result result = controller.transaction(
        address, write_bytes.data(), write_length, read_bytes.data(), read_length, kTransactionTimeout);

    const std::uint8_t code = result_code(result);
    if (!send_all(client, &code, 1))
    {
      return true;
    }
    if (code == kResultOk && read_length > 0 && !send_all(client, read_bytes.data(), read_length))
    {
      return true;
    }

    if (code == kResultBusFault && !controller.peripheral_present())
    {
      std::fprintf(stderr, "[i2c_bridge] peripheral powered down -- exiting\n");
      return false;
    }
  }
}

}  // namespace

int main(int argc, char** argv)
{
  std::string bus_name = "hemerion_bmp390_i2c";
  std::uint16_t port = 5767;
  long wait_s = 120;

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : nullptr; };
    const char* value = nullptr;
    if (arg == "--bus" && (value = next()))
    {
      bus_name = value;
    }
    else if (arg == "--port" && (value = next()))
    {
      if (!parse_number(value, port))
      {
        std::fprintf(stderr, "[i2c_bridge] --port '%s' is not a number in 0..65535\n", value);
        return EXIT_FAILURE;
      }
    }
    else if (arg == "--wait-s" && (value = next()))
    {
      if (!parse_number(value, wait_s))
      {
        std::fprintf(stderr, "[i2c_bridge] --wait-s '%s' is not a number of seconds\n", value);
        return EXIT_FAILURE;
      }
    }
    else
    {
      std::fprintf(stderr,
                   "usage: i2c_shm_tcp_bridge [--bus <name>] [--port <tcp port>] [--wait-s <s>]\n"
                   "  serves the controller end of shared-memory I2C bus <name> (default\n"
                   "  hemerion_bmp390_i2c) on 127.0.0.1:<port> (default 5767; 0 lets the\n"
                   "  kernel choose, and the port actually bound is printed below), waiting\n"
                   "  up to <s> seconds (default 120) for the peripheral to bring the bus up\n");
      return EXIT_FAILURE;
    }
  }

#if defined(_WIN32)
  WSADATA wsa_data;
  if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0)
  {
    std::fprintf(stderr, "[i2c_bridge] WSAStartup failed\n");
    return EXIT_FAILURE;
  }
#endif

  std::optional<I2cShmController> controller = I2cShmController::attach_within(bus_name, std::chrono::seconds(wait_s));
  if (!controller.has_value())
  {
    std::fprintf(stderr, "[i2c_bridge] no peripheral answered on bus '%s'\n", bus_name.c_str());
    return EXIT_FAILURE;
  }

  const SocketHandle listener = socket(AF_INET, SOCK_STREAM, 0);
  if (listener == kInvalidSocket)
  {
    std::fprintf(stderr, "[i2c_bridge] cannot create the listening socket\n");
    return EXIT_FAILURE;
  }
  const int reuse = 1;
  setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse), sizeof(reuse));

  sockaddr_in bind_address{};
  bind_address.sin_family = AF_INET;
  bind_address.sin_port = htons(port);
  inet_pton(AF_INET, "127.0.0.1", &bind_address.sin_addr);
  if (bind(listener, reinterpret_cast<const sockaddr*>(&bind_address), sizeof(bind_address)) != 0 ||
      listen(listener, 1) != 0)
  {
    std::fprintf(stderr, "[i2c_bridge] cannot listen on 127.0.0.1:%u (port in use?)\n", static_cast<unsigned>(port));
    close_socket(listener);
    return EXIT_FAILURE;
  }

  // With --port 0 the kernel chose the port, so read it back rather than
  // reporting the 0 that was asked for: callers that want the race-free
  // "bind first, tell me where" handshake parse this line.
  sockaddr_in bound{};
  SockLen bound_length = sizeof(bound);
  if (getsockname(listener, reinterpret_cast<sockaddr*>(&bound), &bound_length) == 0)
  {
    port = ntohs(bound.sin_port);
  }

  std::printf("[i2c_bridge] bus '%s' <-> 127.0.0.1:%u\n", bus_name.c_str(), static_cast<unsigned>(port));
  std::fflush(stdout);

  bool keep_serving = true;
  while (keep_serving)
  {
    const SocketHandle client = accept(listener, nullptr, nullptr);
    if (client == kInvalidSocket)
    {
      break;
    }
    keep_serving = serve_client(client, *controller);
    close_socket(client);
  }

  close_socket(listener);
#if defined(_WIN32)
  WSACleanup();
#endif
  return EXIT_SUCCESS;
}
