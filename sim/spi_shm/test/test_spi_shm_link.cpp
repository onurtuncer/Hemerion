// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_spi_shm_link.cpp
//
// Native unit test for the simulated SPI bus. Run by CTest under the
// test-native preset on both Linux and Windows. Both ends are opened in this
// single process onto the same named segment -- enough to exercise the full
// handshake, the service thread and the peripheral's byte-by-byte shifting
// without spawning a second process.
// ------------------------------------------------------------------------------
#include <array>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <optional>
#include <vector>

#include "hemerion/sim/spi_shm/spi_shm_link.h"

using hemerion::sim::spi_shm::kMaxTransferBytes;
using hemerion::sim::spi_shm::SpiPeripheral;
using hemerion::sim::spi_shm::SpiShmController;
using hemerion::sim::spi_shm::SpiShmPeripheral;
using namespace std::chrono_literals;

namespace
{

constexpr auto kTimeout = 2000ms;

// Peripheral whose answer to byte k depends on bytes 0..k-1 -- the property a
// per-transfer handshake has to preserve for a device model to be written the
// way a datasheet describes the part.
class EchoPlusIndex final : public SpiPeripheral
{
public:
  void chip_select(bool asserted) override
  {
    if (asserted)
    {
      index_ = 0;
      ++frames_;
    }
    else
    {
      last_frame_length_ = index_;
    }
  }

  std::uint8_t shift(std::uint8_t mosi) override
  {
    running_ = static_cast<std::uint8_t>(running_ + mosi);
    return static_cast<std::uint8_t>(index_++ == 0 ? 0x00 : running_);
  }

  [[nodiscard]] std::size_t frames() const { return frames_; }
  [[nodiscard]] std::size_t last_frame_length() const { return last_frame_length_; }

private:
  std::size_t index_ = 0;
  std::size_t frames_ = 0;
  std::size_t last_frame_length_ = 0;
  std::uint8_t running_ = 0;
};

void test_attach_before_create_fails()
{
  std::optional<SpiShmController> controller = SpiShmController::attach("hemerion_test_spi_never_created");
  assert(!controller.has_value());
}

void test_transfer_round_trip()
{
  auto peripheral = SpiShmPeripheral::create("hemerion_test_spi_round_trip");
  assert(peripheral != nullptr);

  EchoPlusIndex device;
  peripheral->start(device);

  std::optional<SpiShmController> controller =
      SpiShmController::attach_within("hemerion_test_spi_round_trip", kTimeout);
  assert(controller.has_value());
  assert(controller->peripheral_present());
  assert(peripheral->controller_attached());

  // Chip select frames the transfer: one assert/deassert pair per call, and
  // the peripheral sees exactly the bytes posted.
  const std::array<std::uint8_t, 4> tx{ 0x01, 0x02, 0x03, 0x04 };
  std::array<std::uint8_t, 4> rx{};
  assert(controller->transfer(tx.data(), rx.data(), tx.size(), kTimeout));
  assert(device.frames() == 1);
  assert(device.last_frame_length() == tx.size());

  // Byte 0 is the address phase; each later byte carries the running sum of
  // everything shifted in so far, so a transport that reordered or batched
  // bytes would show up here.
  assert(rx[0] == 0x00);
  assert(rx[1] == 0x03);  // 0x01 + 0x02
  assert(rx[2] == 0x06);
  assert(rx[3] == 0x0A);
  assert(controller->transfer_index() == 1);

  // A second transfer restarts the peripheral's frame state.
  assert(controller->transfer(tx.data(), rx.data(), tx.size(), kTimeout));
  assert(device.frames() == 2);
  assert(rx[0] == 0x00);
  assert(controller->transfer_index() == 2);
}

void test_longest_transfer_and_rejected_lengths()
{
  auto peripheral = SpiShmPeripheral::create("hemerion_test_spi_lengths");
  assert(peripheral != nullptr);
  EchoPlusIndex device;
  peripheral->start(device);

  std::optional<SpiShmController> controller = SpiShmController::attach_within("hemerion_test_spi_lengths", kTimeout);
  assert(controller.has_value());

  std::vector<std::uint8_t> tx(kMaxTransferBytes, 0x01);
  std::vector<std::uint8_t> rx(kMaxTransferBytes, 0x00);
  assert(controller->transfer(tx.data(), rx.data(), tx.size(), kTimeout));
  assert(device.last_frame_length() == kMaxTransferBytes);

  // Nothing to shift, and more than the bus can carry: both refused rather
  // than truncated.
  assert(!controller->transfer(tx.data(), rx.data(), 0, kTimeout));
  tx.push_back(0x01);
  rx.push_back(0x00);
  assert(!controller->transfer(tx.data(), rx.data(), tx.size(), kTimeout));
}

void test_data_ready_line()
{
  auto peripheral = SpiShmPeripheral::create("hemerion_test_spi_drdy");
  assert(peripheral != nullptr);
  EchoPlusIndex device;
  peripheral->start(device);

  std::optional<SpiShmController> controller = SpiShmController::attach_within("hemerion_test_spi_drdy", kTimeout);
  assert(controller.has_value());

  assert(!controller->data_ready());
  peripheral->set_data_ready(true);
  assert(controller->data_ready());
  peripheral->set_data_ready(false);
  assert(!controller->data_ready());
}

// A controller must not hang when the simulated part powers down mid-run --
// the FMU terminating is the normal end of every co-simulation.
void test_shutdown_unblocks_the_controller()
{
  auto peripheral = SpiShmPeripheral::create("hemerion_test_spi_shutdown");
  assert(peripheral != nullptr);
  EchoPlusIndex device;
  peripheral->start(device);

  std::optional<SpiShmController> controller = SpiShmController::attach_within("hemerion_test_spi_shutdown", kTimeout);
  assert(controller.has_value());

  const std::array<std::uint8_t, 2> tx{ 0xAA, 0x00 };
  std::array<std::uint8_t, 2> rx{};
  assert(controller->transfer(tx.data(), rx.data(), tx.size(), kTimeout));

  peripheral->stop();
  assert(!controller->peripheral_present());
  assert(!controller->transfer(tx.data(), rx.data(), tx.size(), kTimeout));
}

}  // namespace

int main()
{
  test_attach_before_create_fails();
  test_transfer_round_trip();
  test_longest_transfer_and_rejected_lengths();
  test_data_ready_line();
  test_shutdown_unblocks_the_controller();

  std::puts("test_spi_shm_link: all checks passed");
  return 0;
}
