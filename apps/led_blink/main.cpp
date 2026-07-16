// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file main.cpp
/// @brief LED-blink example app: BSP/FreeRTOS smoke test and primary SWIL
/// example.
///
/// Minimal FreeRTOS task that toggles LD1 (GreenLED, PB0 on the Nucleo-144
/// baseboard) every 500 ms and prints "LED ON"/"LED OFF" over USART3 (the
/// ST-LINK VCP). See sim/renode/boards/nucleo_h743zi2.repl and
/// tests/swil/test_led_blink.py, which assert on this exact UART text.

#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

#include "hemerion/hal/board.h"
#include "hemerion/hal/gpio.h"
#include "hemerion/hal/uart.h"

namespace
{

constexpr uint8_t kLedPort = 'B';
constexpr uint8_t kLedPin = 0;
constexpr uint8_t kUartInstance = 3;
constexpr uint32_t kUartBaudRate = 115200;
constexpr TickType_t kBlinkPeriod = pdMS_TO_TICKS(500);

void uart_println(const char* message)
{
  hal_uart_write(kUartInstance, reinterpret_cast<const uint8_t*>(message), std::strlen(message));
  hal_uart_write(kUartInstance, reinterpret_cast<const uint8_t*>("\r\n"), 2);
}

void led_blink_task(void* /*pvParameters*/)
{
  bool led_on = false;
  for (;;)
  {
    led_on = !led_on;
    hal_gpio_write(kLedPort, kLedPin, led_on);
    uart_println(led_on ? "LED ON" : "LED OFF");
    vTaskDelay(kBlinkPeriod);
  }
}

}  // namespace

int main()
{
  hal_board_init();
  hal_uart_init(kUartInstance, kUartBaudRate);
  hal_gpio_set_mode(kLedPort, kLedPin, HAL_GPIO_MODE_OUTPUT_PUSH_PULL);

  xTaskCreate(led_blink_task, "led_blink", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1, nullptr);

  vTaskStartScheduler();

  /* vTaskStartScheduler() does not return on a successful boot. */
  for (;;)
  {
  }
}
