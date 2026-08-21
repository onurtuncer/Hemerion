// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file mmc5983ma_hal_i2c_bus.h
/// @brief Mmc5983maI2cBus over the Hemerion C HAL contract
/// (`hemerion/hal/i2c.h` + `hemerion/hal/board.h`) -- the on-hardware
/// transport.
///
/// Header-only and deliberately not compiled into the hemerion_sensors
/// library: it includes the HAL contract headers, which only a firmware
/// target linking a BSP (e.g. bsp/stm32h743_nucleo) provides. Host builds --
/// the co-simulation, the unit tests, the FMU -- never include this file;
/// they inject a bus over sim/i2c_shm or the device model directly. Include
/// it from the firmware app that owns the sensor task, next to where the
/// BSP's `hal_i2c_init()` is called.
///
/// Two board facts this adapter cannot guess, both taken as optional
/// function pointers:
///
/// * **the INT line.** Boards that leave the part's INT pin unwired pass
///   nullptr and the driver polls `Status` instead, which is authoritative
///   either way.
/// * **the microsecond clock.** The HAL contract has `hal_delay_ms()` but no
///   monotonic counter yet, and this part -- unlike the BMP390 with its
///   `SENSORTIME` -- has no clock of its own to stamp samples from. Pass a
///   sampler for whatever the board runs (a DWT cycle counter, a free-running
///   timer, `HAL_GetTick()` scaled to microseconds); pass nullptr and samples
///   come back stamped zero for a caller downstream to stamp instead. Once
///   cmake/hemerion_hal/ lands a `hal_monotonic_us()`, this parameter should
///   collapse into it.

#pragma once

#include <cstddef>
#include <cstdint>

#include "Hemerion/mag/mmc5983ma/mmc5983ma_driver.h"

#include "hemerion/hal/board.h"
#include "hemerion/hal/i2c.h"

namespace hemerion::sensors::mag::mmc5983ma
{

/// @brief Mmc5983maI2cBus over `hal_i2c_mem_read/write` -- what the STM32
/// build injects into Mmc5983maDriver.
class Mmc5983maHalI2cBus final : public Mmc5983maI2cBus
{
public:
  /// Per-transaction timeout handed to the blocking HAL calls [ms]. The
  /// part runs the bus at up to 400 kHz and the longest transaction here is
  /// eight bytes, so anything near this bound is a wedged bus, not a slow
  /// one.
  static constexpr std::uint32_t kTransactionTimeoutMs = 25;

  /// @param instance       I2C instance number, as `hal_i2c_init()` numbers
  ///                       them (1 = I2C1 on the NUCLEO-H743ZI2's PB8/PB9).
  /// @param interrupt_read Optional sampler for the GPIO the part's INT pin
  ///                       is wired to; nullptr if the board leaves it
  ///                       unwired.
  /// @param monotonic_us   Optional microsecond clock used to stamp samples;
  ///                       nullptr stamps them zero.
  /// @param target_address 7-bit part address. Fixed at
  ///                       @ref kMmc5983maI2cAddress in silicon -- exposed
  ///                       only so a board bridging the part behind an
  ///                       address translator can say so.
  explicit Mmc5983maHalI2cBus(std::uint8_t instance,
                              bool (*interrupt_read)() = nullptr,
                              std::uint64_t (*monotonic_us)() = nullptr,
                              std::uint8_t target_address = kMmc5983maI2cAddress)
    : instance_(instance), target_address_(target_address), interrupt_read_(interrupt_read), monotonic_us_(monotonic_us)
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

  [[nodiscard]] std::uint64_t now_us() const override { return (monotonic_us_ != nullptr) ? monotonic_us_() : 0; }

  [[nodiscard]] bool interrupt_line() const override { return interrupt_read_ != nullptr && interrupt_read_(); }

private:
  std::uint8_t instance_;
  std::uint8_t target_address_;
  bool (*interrupt_read_)();
  std::uint64_t (*monotonic_us_)();
};

}  // namespace hemerion::sensors::mag::mmc5983ma
