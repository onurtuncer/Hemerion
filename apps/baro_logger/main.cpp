// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file main.cpp
/// @brief BMP390 logger app: the first firmware consumer of the on-target
/// barometer driver.
///
/// One FreeRTOS task brings a BMP390 up on I2C1 (PB8/PB9, the Nucleo-144's
/// Arduino D15/D14 header -- SDO strapped to GND, so the part answers 0x76)
/// through the exact same Bmp390Driver + Bmp390Compensator the co-simulation
/// host runs in examples/rocket_gps_ecos, then polls it at its 50 Hz ODR and
/// prints one line per conversion over USART3 (the ST-LINK VCP):
///
///     BARO t=163840 us p=101321 Pa T=23150 mC
///
/// Values are printed as integers (Pa, milli-degrees, microseconds from the
/// part's own SENSORTIME counter) because newlib-nano's printf omits float
/// support unless it is linked in explicitly, and nothing here justifies
/// pulling that in. The board layer under the driver is
/// Bmp390HalI2cBus -- hal_i2c_mem_read/write over the BSP's I2C1 -- with the
/// INT pin left unwired; the driver's STATUS poll is authoritative without
/// it.
///
/// A probe failure is reported over the UART and retried every second rather
/// than trapped: on the bench, "BARO probe failed" repeating is a wiring
/// conversation, not a crash dump.

#include <cstdio>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

#include "hemerion/hal/board.h"
#include "hemerion/hal/i2c.h"
#include "hemerion/hal/uart.h"

#include "Hemerion/baro/baro_types.h"
#include "Hemerion/baro/bmp390/bmp390_driver.h"
#include "Hemerion/baro/bmp390/bmp390_hal_i2c_bus.h"

namespace
{

using hemerion::sensors::baro::BaroSample;
using hemerion::sensors::baro::bmp390::Bmp390Driver;
using hemerion::sensors::baro::bmp390::Bmp390Error;
using hemerion::sensors::baro::bmp390::Bmp390HalI2cBus;
using hemerion::sensors::baro::bmp390::Bmp390ReadResult;

constexpr uint8_t kUartInstance = 3;
constexpr uint32_t kUartBaudRate = 115200;
constexpr uint8_t kI2cInstance = 1;
// The driver's default config is 50 Hz; polling at half the period keeps the
// STATUS check ahead of every conversion without busy-waiting the bus.
constexpr TickType_t kPollPeriod = pdMS_TO_TICKS(10);
constexpr TickType_t kProbeRetryPeriod = pdMS_TO_TICKS(1000);

void uart_println(const char* message)
{
  hal_uart_write(kUartInstance, reinterpret_cast<const uint8_t*>(message), std::strlen(message));
  hal_uart_write(kUartInstance, reinterpret_cast<const uint8_t*>("\r\n"), 2);
}

void baro_task(void* /*pvParameters*/)
{
  Bmp390HalI2cBus bus(kI2cInstance);
  Bmp390Driver driver(bus);

  for (;;)
  {
    switch (driver.probe())
    {
      case Bmp390Error::kNone:
        uart_println("BARO up: BMP390 identified, calibrated, normal mode at 50 Hz");
        break;
      case Bmp390Error::kIdentityMismatch:
        uart_println("BARO probe failed: CHIP_ID mismatch (not a BMP390 at 0x76?)");
        vTaskDelay(kProbeRetryPeriod);
        continue;
      case Bmp390Error::kTransferFailed:
        uart_println("BARO probe failed: no ack on I2C1 (wiring/pull-ups/SDO strap?)");
        vTaskDelay(kProbeRetryPeriod);
        continue;
    }
    break;
  }

  char line[64];
  for (;;)
  {
    vTaskDelay(kPollPeriod);

    BaroSample sample;
    const Bmp390ReadResult result = driver.read_sample(sample);
    if (result == Bmp390ReadResult::kNoNewData)
    {
      continue;
    }
    if (result == Bmp390ReadResult::kTransferFailed)
    {
      uart_println("BARO read failed: I2C transfer error");
      vTaskDelay(kProbeRetryPeriod);
      continue;
    }

    const auto pressure_pa = static_cast<long>(sample.pressure_pa);
    const auto temperature_mc = static_cast<long>(sample.temperature_c * 1000.0F);
    std::snprintf(line,
                  sizeof(line),
                  "BARO t=%lu us p=%ld Pa T=%ld mC",
                  static_cast<unsigned long>(sample.timestamp_us),
                  pressure_pa,
                  temperature_mc);
    uart_println(line);
  }
}

}  // namespace

int main()
{
  hal_board_init();
  hal_uart_init(kUartInstance, kUartBaudRate);
  hal_i2c_init(kI2cInstance);

  // The driver's compensation runs double-precision polynomial math on this
  // task's stack; 1 KiB of words is comfortable headroom next to
  // configMINIMAL_STACK_SIZE.
  xTaskCreate(baro_task, "baro", 1024, nullptr, tskIDLE_PRIORITY + 1, nullptr);

  vTaskStartScheduler();

  /* vTaskStartScheduler() does not return on a successful boot. */
  for (;;)
  {
  }
}
