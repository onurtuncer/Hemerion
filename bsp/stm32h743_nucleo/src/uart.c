/* ------------------------------------------------------------------------------
 * Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
 *
 * SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
 * ------------------------------------------------------------------------------ */

#include "hemerion/hal/uart.h"
#include "stm32h7xx_hal.h"

static UART_HandleTypeDef husart3;
static bool usart3_ready = false;

void hal_uart_init(uint8_t instance, uint32_t baud_rate)
{
  if (instance != 3U)
  {
    return; /* only USART3 (ST-LINK VCP, PD8/PD9) is wired up -- see uart.h */
  }

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();

  GPIO_InitTypeDef gpio_init = {0};
  gpio_init.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  gpio_init.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &gpio_init);

  husart3.Instance = USART3;
  husart3.Init.BaudRate = baud_rate;
  husart3.Init.WordLength = UART_WORDLENGTH_8B;
  husart3.Init.StopBits = UART_STOPBITS_1;
  husart3.Init.Parity = UART_PARITY_NONE;
  husart3.Init.Mode = UART_MODE_TX_RX;
  husart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husart3.Init.OverSampling = UART_OVERSAMPLING_16;
  husart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  husart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  husart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  usart3_ready = (HAL_UART_Init(&husart3) == HAL_OK);
}

void hal_uart_write(uint8_t instance, const uint8_t *data, size_t len)
{
  if (instance != 3U || !usart3_ready)
  {
    return;
  }
  HAL_UART_Transmit(&husart3, data, (uint16_t)len, HAL_MAX_DELAY);
}

size_t hal_uart_read(uint8_t instance, uint8_t *data, size_t len, uint32_t timeout_ms)
{
  if (instance != 3U || !usart3_ready)
  {
    return 0U;
  }

  HAL_StatusTypeDef status = HAL_UART_Receive(&husart3, data, (uint16_t)len, timeout_ms);
  if (status == HAL_OK)
  {
    return len;
  }
  /* Timed out partway through -- RxXferCount is what's still missing. */
  return len - husart3.RxXferCount;
}
