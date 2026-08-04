/* ------------------------------------------------------------------------------
 * Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
 *
 * SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
 * ------------------------------------------------------------------------------ */

#include <stdbool.h>

#include "hemerion/hal/i2c.h"
#include "stm32h7xx_hal.h"

static I2C_HandleTypeDef hi2c1;
static bool i2c1_ready = false;

void hal_i2c_init(uint8_t instance)
{
  if (instance != 1U)
  {
    return; /* only I2C1 (PB8/PB9, Arduino D15/D14) is wired up -- see i2c.h */
  }

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  GPIO_InitTypeDef gpio_init = { 0 };
  gpio_init.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  gpio_init.Mode = GPIO_MODE_AF_OD; /* I2C is open-drain by construction */
  gpio_init.Pull = GPIO_PULLUP;     /* weak assist; the bus still wants real pull-ups */
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  gpio_init.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &gpio_init);

  hi2c1.Instance = I2C1;
  /* TIMINGR for 100 kHz standard mode with the I2C1 kernel clock on
     rcc_pclk1 = 120 MHz (SystemClock_Config's 480 MHz tree, APB1 /4 --
     see system_stm32h7xx.c). CubeMX-computed; recompute if either the
     clock tree or the bus speed changes. */
  hi2c1.Init.Timing = 0x307075B1U;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  i2c1_ready = (HAL_I2C_Init(&hi2c1) == HAL_OK);
  if (i2c1_ready)
  {
    /* Same post-init trim CubeMX emits: analog filter on, digital off. */
    i2c1_ready = (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) == HAL_OK) &&
                 (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) == HAL_OK);
  }
}

bool hal_i2c_mem_write(uint8_t instance,
                       uint8_t target_address,
                       uint8_t reg,
                       const uint8_t* data,
                       size_t len,
                       uint32_t timeout_ms)
{
  if (instance != 1U || !i2c1_ready)
  {
    return false;
  }
  /* The HAL wants the address pre-shifted; the R/W bit is its business. */
  return HAL_I2C_Mem_Write(&hi2c1,
                           (uint16_t)((uint16_t)target_address << 1),
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           /* the HAL API is not const-correct; it never writes through this */
                           (uint8_t*)(uintptr_t)data,
                           (uint16_t)len,
                           timeout_ms) == HAL_OK;
}

bool hal_i2c_mem_read(uint8_t instance,
                      uint8_t target_address,
                      uint8_t reg,
                      uint8_t* data,
                      size_t len,
                      uint32_t timeout_ms)
{
  if (instance != 1U || !i2c1_ready)
  {
    return false;
  }
  return HAL_I2C_Mem_Read(&hi2c1,
                          (uint16_t)((uint16_t)target_address << 1),
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          data,
                          (uint16_t)len,
                          timeout_ms) == HAL_OK;
}
