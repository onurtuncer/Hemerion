// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// test_i2c_shm_link.cpp
//
// Native unit test for the simulated I2C bus. Run by CTest under the
// test-native preset on both Linux and Windows. Both ends are opened in this
// single process onto the same named segment -- enough to exercise the full
// handshake, the service thread and the peripheral's event-by-event bus state
// machine without spawning a second process.
// ------------------------------------------------------------------------------
#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <optional>

#include "hemerion/sim/i2c_shm/i2c_shm_link.h"

using hemerion::sim::i2c_shm::I2cPeripheral;
using hemerion::sim::i2c_shm::I2cShmController;
using hemerion::sim::i2c_shm::I2cShmPeripheral;
using namespace std::chrono_literals;

namespace
{

constexpr auto kTimeout = 2000ms;
constexpr std::uint8_t kDeviceAddress = 0x76;

// Minimal register-file part: the first write byte after START selects the
// register, further write bytes store through an auto-incrementing pointer,
// and reads auto-increment from wherever the pointer points -- the shape most
// I2C sensors share, reduced to what the bus-event contract has to carry.
class RegisterFileDevice final : public I2cPeripheral
{
public:
  bool start(std::uint8_t address, bool read) override
  {
    (void)read;
    ++starts_;
    if (address != kDeviceAddress)
    {
      return false;
    }
    awaiting_pointer_ = !read;
    return true;
  }

  bool write(std::uint8_t byte) override
  {
    if (awaiting_pointer_)
    {
      pointer_ = byte;
      awaiting_pointer_ = false;
      return true;
    }
    if (pointer_ == kNackRegister)
    {
      return false;  // a register that refuses writes, to exercise the data NACK path
    }
    registers_[pointer_++] = byte;
    return true;
  }

  std::uint8_t read(bool last) override
  {
    last_read_was_nacked_ = last;
    return registers_[pointer_++];
  }

  void stop() override { ++stops_; }

  [[nodiscard]] std::uint8_t register_value(std::uint8_t address) const { return registers_[address]; }
  [[nodiscard]] std::size_t starts() const { return starts_; }
  [[nodiscard]] std::size_t stops() const { return stops_; }
  [[nodiscard]] bool last_read_was_nacked() const { return last_read_was_nacked_; }

  static constexpr std::uint8_t kNackRegister = 0xEE;

private:
  std::array<std::uint8_t, 256> registers_{};
  std::uint8_t pointer_ = 0;
  bool awaiting_pointer_ = false;
  std::size_t starts_ = 0;
  std::size_t stops_ = 0;
  bool last_read_was_nacked_ = false;
};

void test_attach_before_create_fails()
{
  std::optional<I2cShmController> controller = I2cShmController::attach("hemerion_test_i2c_never_created");
  assert(!controller.has_value());
}

// A mem-write followed by a mem-read must land in and come back out of the
// device's register file, with the write phase and the repeated-start read
// phase each walking the auto-incrementing pointer.
void test_write_then_read_round_trip()
{
  auto peripheral = I2cShmPeripheral::create("hemerion_test_i2c_round_trip");
  assert(peripheral != nullptr);
  RegisterFileDevice device;
  peripheral->start(device);

  std::optional<I2cShmController> controller =
      I2cShmController::attach_within("hemerion_test_i2c_round_trip", kTimeout);
  assert(controller.has_value());

  // HAL_I2C_Mem_Write(0x10, {0xAA, 0xBB, 0xCC}): pointer then three data bytes.
  const std::array<std::uint8_t, 4> write_frame{ 0x10, 0xAA, 0xBB, 0xCC };
  assert(controller->transaction(kDeviceAddress, write_frame.data(), write_frame.size(), nullptr, 0, kTimeout) ==
         I2cShmController::Result::kOk);
  assert(device.register_value(0x10) == 0xAA);
  assert(device.register_value(0x11) == 0xBB);
  assert(device.register_value(0x12) == 0xCC);

  // HAL_I2C_Mem_Read(0x11, 2): pointer write, repeated START, two-byte read.
  const std::uint8_t pointer = 0x11;
  std::array<std::uint8_t, 2> read_back{};
  assert(controller->transaction(kDeviceAddress, &pointer, 1, read_back.data(), read_back.size(), kTimeout) ==
         I2cShmController::Result::kOk);
  assert(read_back[0] == 0xBB);
  assert(read_back[1] == 0xCC);
  // The controller NACKs the final byte to end the read, and the device saw it.
  assert(device.last_read_was_nacked());

  assert(controller->transaction_index() == 2);
  assert(peripheral->transactions_serviced() == 2);
}

// A part that is not at the probed address must come back as an address NACK
// -- and the empty probe transaction HAL_I2C_IsDeviceReady() sends must ack
// against the address that is there.
void test_address_probe_and_nack()
{
  auto peripheral = I2cShmPeripheral::create("hemerion_test_i2c_probe");
  assert(peripheral != nullptr);
  RegisterFileDevice device;
  peripheral->start(device);

  std::optional<I2cShmController> controller = I2cShmController::attach_within("hemerion_test_i2c_probe", kTimeout);
  assert(controller.has_value());

  assert(controller->transaction(kDeviceAddress, nullptr, 0, nullptr, 0, kTimeout) == I2cShmController::Result::kOk);
  assert(controller->transaction(0x42, nullptr, 0, nullptr, 0, kTimeout) == I2cShmController::Result::kAddressNack);

  // An address NACK on the read phase's repeated START is still an address NACK.
  std::uint8_t byte = 0;
  assert(controller->transaction(0x42, nullptr, 0, &byte, 1, kTimeout) == I2cShmController::Result::kAddressNack);
}

// A byte the part refuses mid-write must surface as a data NACK, with the
// transaction still framed by a STOP so the device is not left hanging.
void test_data_nack_mid_write()
{
  auto peripheral = I2cShmPeripheral::create("hemerion_test_i2c_data_nack");
  assert(peripheral != nullptr);
  RegisterFileDevice device;
  peripheral->start(device);

  std::optional<I2cShmController> controller = I2cShmController::attach_within("hemerion_test_i2c_data_nack", kTimeout);
  assert(controller.has_value());

  const std::size_t stops_before = device.stops();
  const std::array<std::uint8_t, 2> frame{ RegisterFileDevice::kNackRegister, 0x55 };
  assert(controller->transaction(kDeviceAddress, frame.data(), frame.size(), nullptr, 0, kTimeout) ==
         I2cShmController::Result::kDataNack);
  assert(device.stops() == stops_before + 1);
}

// The interrupt line is a level the peripheral drives and the controller
// samples -- both edges, in both directions.
void test_interrupt_line()
{
  auto peripheral = I2cShmPeripheral::create("hemerion_test_i2c_interrupt");
  assert(peripheral != nullptr);
  RegisterFileDevice device;
  peripheral->start(device);

  std::optional<I2cShmController> controller = I2cShmController::attach_within("hemerion_test_i2c_interrupt", kTimeout);
  assert(controller.has_value());

  assert(!controller->interrupt_line());
  peripheral->set_interrupt(true);
  assert(controller->interrupt_line());
  peripheral->set_interrupt(false);
  assert(!controller->interrupt_line());
}

// Powering the peripheral down mid-run must fail the controller's next
// transaction fast (terminal phase), not leave it waiting out its timeout.
void test_shutdown_fails_fast()
{
  auto peripheral = I2cShmPeripheral::create("hemerion_test_i2c_shutdown");
  assert(peripheral != nullptr);
  RegisterFileDevice device;
  peripheral->start(device);

  std::optional<I2cShmController> controller = I2cShmController::attach_within("hemerion_test_i2c_shutdown", kTimeout);
  assert(controller.has_value());
  assert(controller->peripheral_present());

  peripheral->stop();
  assert(!controller->peripheral_present());

  const std::uint8_t pointer = 0x00;
  std::uint8_t byte = 0;
  assert(controller->transaction(kDeviceAddress, &pointer, 1, &byte, 1, kTimeout) ==
         I2cShmController::Result::kBusFault);
}

}  // namespace

int main()
{
  test_attach_before_create_fails();
  test_write_then_read_round_trip();
  test_address_probe_and_nack();
  test_data_nack_mid_write();
  test_interrupt_line();
  test_shutdown_fails_fast();

  std::puts("test_i2c_shm_link: all checks passed");
  return 0;
}
