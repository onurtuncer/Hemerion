/* ------------------------------------------------------------------------------
 * Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
 *
 * SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
 * ------------------------------------------------------------------------------ */

#include "hemerion/hal/board.h"
#include "stm32h7xx_hal.h"

/* Defined in system_stm32h7xx.c; not part of the CMSIS system_stm32h7xx.h
   contract (SystemInit/SystemCoreClockUpdate only), so declared locally here. */
extern void SystemClock_Config(void);
extern void MPU_Config(void);

void hal_board_init(void)
{
  MPU_Config();

  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();
  SystemClock_Config();
}

void hal_board_reset(void)
{
  NVIC_SystemReset();
  for (;;)
  {
  }
}
