/* ------------------------------------------------------------------------------
 * Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
 *
 * SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
 * ------------------------------------------------------------------------------ */

#include "hemerion/hal/gpio.h"
#include "stm32h7xx_hal.h"

/* Enables the port's peripheral clock as a side effect -- callers only need
   to call hal_gpio_set_mode() once per pin before using it. */
static GPIO_TypeDef *gpio_port_enable(uint8_t port)
{
  switch (port)
  {
    case 'A': __HAL_RCC_GPIOA_CLK_ENABLE(); return GPIOA;
    case 'B': __HAL_RCC_GPIOB_CLK_ENABLE(); return GPIOB;
    case 'C': __HAL_RCC_GPIOC_CLK_ENABLE(); return GPIOC;
    case 'D': __HAL_RCC_GPIOD_CLK_ENABLE(); return GPIOD;
    case 'E': __HAL_RCC_GPIOE_CLK_ENABLE(); return GPIOE;
    case 'F': __HAL_RCC_GPIOF_CLK_ENABLE(); return GPIOF;
    case 'G': __HAL_RCC_GPIOG_CLK_ENABLE(); return GPIOG;
    case 'H': __HAL_RCC_GPIOH_CLK_ENABLE(); return GPIOH;
    case 'I': __HAL_RCC_GPIOI_CLK_ENABLE(); return GPIOI;
    default: return NULL; /* not a port on the LQFP144 package -- programmer error */
  }
}

void hal_gpio_set_mode(uint8_t port, uint8_t pin, hal_gpio_mode_t mode)
{
  GPIO_TypeDef *gpio = gpio_port_enable(port);
  if (gpio == NULL || pin > 15U)
  {
    return;
  }

  GPIO_InitTypeDef init = {0};
  init.Pin = (uint16_t)(1U << pin);
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_LOW;

  switch (mode)
  {
    case HAL_GPIO_MODE_INPUT:
      init.Mode = GPIO_MODE_INPUT;
      break;
    case HAL_GPIO_MODE_OUTPUT_PUSH_PULL:
      init.Mode = GPIO_MODE_OUTPUT_PP;
      break;
    case HAL_GPIO_MODE_OUTPUT_OPEN_DRAIN:
      init.Mode = GPIO_MODE_OUTPUT_OD;
      break;
    case HAL_GPIO_MODE_ANALOG:
      init.Mode = GPIO_MODE_ANALOG;
      break;
    default:
      return;
  }

  HAL_GPIO_Init(gpio, &init);
}

void hal_gpio_write(uint8_t port, uint8_t pin, bool level)
{
  GPIO_TypeDef *gpio = gpio_port_enable(port);
  if (gpio == NULL || pin > 15U)
  {
    return;
  }
  HAL_GPIO_WritePin(gpio, (uint16_t)(1U << pin), level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool hal_gpio_read(uint8_t port, uint8_t pin)
{
  GPIO_TypeDef *gpio = gpio_port_enable(port);
  if (gpio == NULL || pin > 15U)
  {
    return false;
  }
  return HAL_GPIO_ReadPin(gpio, (uint16_t)(1U << pin)) == GPIO_PIN_SET;
}
