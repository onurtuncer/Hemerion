// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file main.cpp
/// @brief MMC5983MA logger app: the first firmware consumer of the on-target
/// magnetometer driver.
///
/// One FreeRTOS task brings an MMC5983MA up on I2C1 (PB8/PB9, the Nucleo-144's
/// Arduino D15/D14 header -- the part has no address strap, so it answers 0x30
/// and nothing else can share the bus with it) through the exact same
/// Mmc5983maDriver the co-simulation host runs, then polls it at its 50 Hz
/// continuous rate and prints one line per measurement over USART3 (the
/// ST-LINK VCP):
///
///     MAG up: MMC5983MA identified, offset cancelled, 50 Hz
///     MAG offset x=2731 y=-1204 z=3355 LSB
///     MAG t=163840 us x=22001 y=-6000 z=41002 nT
///
/// Values are printed as integers (nanotesla, microseconds from the FreeRTOS
/// tick) because newlib-nano's printf omits float support unless it is linked
/// in explicitly, and nothing here justifies pulling that in. Earth's field is
/// ~25000-65000 nT, so a long is ample.
///
/// **The offset line is the point of this app.** Bring-up runs the datasheet's
/// SET/RESET pair and prints what it recovered, because that number is the
/// difference between a working magnetometer and one reading its own bridge
/// offset: this part's null field output is specified only to +/-0.5 gauss,
/// which is as large as Earth's entire field. A line of zeroes means the
/// calibration did not happen; a line whose magnitude drifts between reboots
/// is a part that has been sitting in a disturbing field.
///
/// The board layer under the driver is Mmc5983maHalI2cBus --
/// hal_i2c_mem_read/write over the BSP's I2C1 -- with the INT pin left
/// unwired; the driver's `Status` poll is authoritative without it. The
/// microsecond clock it stamps samples with comes from the FreeRTOS tick,
/// which is the only monotonic counter the HAL contract exposes today (see
/// mmc5983ma_hal_i2c_bus.h on why that is a parameter rather than a
/// `hal_monotonic_us()`).
///
/// A probe failure is reported over the UART and retried every second rather
/// than trapped: on the bench, "MAG probe failed" repeating is a wiring
/// conversation, not a crash dump.

#include <cstdio>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

#include "hemerion/hal/board.h"
#include "hemerion/hal/i2c.h"
#include "hemerion/hal/uart.h"

#include "Hemerion/mag/mag_types.h"
#include "Hemerion/mag/mmc5983ma/mmc5983ma_driver.h"
#include "Hemerion/mag/mmc5983ma/mmc5983ma_hal_i2c_bus.h"

namespace
{

using hemerion::sensors::mag::MagSample;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maDriver;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maError;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maHalI2cBus;
using hemerion::sensors::mag::mmc5983ma::Mmc5983maReadResult;

constexpr uint8_t kUartInstance = 3;
constexpr uint32_t kUartBaudRate = 115200;
constexpr uint8_t kI2cInstance = 1;
// The driver's default config is 50 Hz; polling at half the period keeps the
// Status check ahead of every measurement without busy-waiting the bus.
constexpr TickType_t kPollPeriod = pdMS_TO_TICKS(10);
constexpr TickType_t kProbeRetryPeriod = pdMS_TO_TICKS(1000);

void uart_println(const char* message)
{
  hal_uart_write(kUartInstance, reinterpret_cast<const uint8_t*>(message), std::strlen(message));
  hal_uart_write(kUartInstance, reinterpret_cast<const uint8_t*>("\r\n"), 2);
}

/// The monotonic microsecond clock the driver stamps samples with. The tick
/// is millisecond-resolution, so the bottom three digits are always zero --
/// honest for what this board can currently offer, and enough to order
/// samples at 50 Hz.
std::uint64_t monotonic_us()
{
  return static_cast<std::uint64_t>(xTaskGetTickCount()) * (1000000ULL / configTICK_RATE_HZ);
}

void mag_task(void* /*pvParameters*/)
{
  Mmc5983maHalI2cBus bus(kI2cInstance, nullptr, monotonic_us);
  Mmc5983maDriver driver(bus);

  for (;;)
  {
    switch (driver.probe())
    {
      case Mmc5983maError::kNone:
        uart_println("MAG up: MMC5983MA identified, offset cancelled, 50 Hz");
        break;
      case Mmc5983maError::kIdentityMismatch:
        uart_println("MAG probe failed: product ID mismatch (not an MMC5983MA at 0x30?)");
        vTaskDelay(kProbeRetryPeriod);
        continue;
      case Mmc5983maError::kTransferFailed:
        uart_println("MAG probe failed: no ack on I2C1 (wiring/pull-ups?)");
        vTaskDelay(kProbeRetryPeriod);
        continue;
      case Mmc5983maError::kMeasurementTimeout:
        uart_println("MAG probe failed: SET/RESET measurement never completed");
        vTaskDelay(kProbeRetryPeriod);
        continue;
      case Mmc5983maError::kInvalidConfiguration:
        // Not reachable with the default config, and not retryable if it
        // ever were: the fix is a rebuild, not another second of waiting.
        uart_println("MAG probe failed: configuration rejected -- rebuild required");
        vTaskDelay(portMAX_DELAY);
        continue;
    }
    break;
  }

  char line[96];
  const auto& offset = driver.bridge_offset();
  std::snprintf(line,
                sizeof(line),
                "MAG offset x=%ld y=%ld z=%ld LSB",
                static_cast<long>(offset.x),
                static_cast<long>(offset.y),
                static_cast<long>(offset.z));
  uart_println(line);

  for (;;)
  {
    vTaskDelay(kPollPeriod);

    MagSample sample;
    const Mmc5983maReadResult result = driver.read_sample(sample);
    if (result == Mmc5983maReadResult::kNoNewData)
    {
      continue;
    }
    if (result == Mmc5983maReadResult::kTransferFailed)
    {
      uart_println("MAG read failed: I2C transfer error");
      vTaskDelay(kProbeRetryPeriod);
      continue;
    }

    // Nanotesla: integer-friendly, and three digits finer than the part's
    // ~6 nT quantum, so nothing real is rounded away in the print.
    std::snprintf(line,
                  sizeof(line),
                  "MAG t=%lu us x=%ld y=%ld z=%ld nT",
                  static_cast<unsigned long>(sample.timestamp_us),
                  static_cast<long>(sample.mag_x_ut * 1000.0F),
                  static_cast<long>(sample.mag_y_ut * 1000.0F),
                  static_cast<long>(sample.mag_z_ut * 1000.0F));
    uart_println(line);
  }
}

}  // namespace

int main()
{
  hal_board_init();
  hal_uart_init(kUartInstance, kUartBaudRate);
  hal_i2c_init(kI2cInstance);

  // 1 KiB of words: the driver itself is fixed-size and shallow, but the
  // snprintf above is not, and this matches apps/baro_logger's headroom.
  xTaskCreate(mag_task, "mag", 1024, nullptr, tskIDLE_PRIORITY + 1, nullptr);

  vTaskStartScheduler();

  /* vTaskStartScheduler() does not return on a successful boot. */
  for (;;)
  {
  }
}
