// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file bmp390_hal_i2c_bus.h
/// @brief Bmp390I2cBus over the Hemerion C HAL contract (`hemerion/hal/i2c.h`
/// + `hemerion/hal/board.h`) -- the on-hardware transport.
///
/// Header-only and deliberately not compiled into the hemerion_sensors
/// library: it includes the HAL contract headers, which only a firmware
/// target linking a BSP (e.g. bsp/stm32h743_nucleo) provides. Host builds --
/// the co-simulation, the unit tests, the FMUs -- never include this file;
/// they inject a bus over sim/i2c_shm or the device model directly. Include
/// it from the firmware app that owns the sensor task, next to where the
/// BSP's `hal_i2c_init()` is called.
///
/// The interrupt line: the NUCLEO wiring for the part's INT pin is a board
/// fact this adapter cannot guess, so it takes the GPIO sample as an optional
/// function pointer. Boards that leave INT unwired pass nullptr and the
/// driver polls `STATUS` instead, which is authoritative either way.

#pragma once

#include <cstddef>
#include <cstdint>

#include "Hemerion/baro/bmp390/bmp390_driver.h"

#include "hemerion/hal/board.h"
#include "hemerion/hal/i2c.h"

namespace hemerion::sensors::baro::bmp390
{

/// @brief Bmp390I2cBus over `hal_i2c_mem_read/write` -- what the STM32 build
/// injects into Bmp390Driver.
class Bmp390HalI2cBus final : public Bmp390I2cBus
{
public:
  /// Per-transaction timeout handed to the blocking HAL calls [ms]. A
  /// standard-mode transaction is sub-millisecond; anything near this bound
  /// is a wedged bus, not a slow one.
  static constexpr std::uint32_t kTransactionTimeoutMs = 25;

  /// @param instance       I2C instance number, as `hal_i2c_init()` numbers
  ///                       them (1 = I2C1 on the NUCLEO-H743ZI2's PB8/PB9).
  /// @param target_address 7-bit part address: kBmp390I2cAddressPrimary with
  ///                       SDO strapped to GND, ...Secondary to VDDIO.
  /// @param interrupt_read Optional sampler for the GPIO the part's INT pin
  ///                       is wired to; nullptr if the board leaves it
  ///                       unwired.
  explicit Bmp390HalI2cBus(std::uint8_t instance,
                           std::uint8_t target_address = kBmp390I2cAddressPrimary,
                           bool (*interrupt_read)() = nullptr)
    : instance_(instance), target_address_(target_address), interrupt_read_(interrupt_read)
  {
  }

  [[nodiscard]] bool write_register(std::uint8_t reg, std::uint8_t value) override
  {
    return hal_i2c_mem_write(instance_, target_address_, reg, &value, 1U, kTransactionTimeoutMs);
  }

  [[nodiscard]] bool read_registers(std::uint8_t reg, std::uint8_t* out, std::size_t count) override
  {
    return hal_i2c_mem_read(instance_, target_address_, reg, out, count, kTransactionTimeoutMs);
  }

  void delay_ms(std::uint32_t milliseconds) override { hal_delay_ms(milliseconds); }

  [[nodiscard]] bool interrupt_line() const override { return interrupt_read_ != nullptr && interrupt_read_(); }

private:
  std::uint8_t instance_;
  std::uint8_t target_address_;
  bool (*interrupt_read_)();
};

}  // namespace hemerion::sensors::baro::bmp390
