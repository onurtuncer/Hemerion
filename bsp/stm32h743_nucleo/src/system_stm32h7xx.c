/**
  ******************************************************************************
  * @file    system_stm32h7xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-Mx Device Peripheral Access Layer System Source File,
  *          adapted for bsp/stm32h743_nucleo (single Cortex-M7 core, no
  *          DUAL_CORE branches). Provides:
  *      - ExitRun0Mode(): power supply sequencing, called from Reset_Handler
  *                        in startup_stm32h743xx.s before SystemInit().
  *      - SystemInit(): called from Reset_Handler before main().
  *      - SystemCoreClock / SystemCoreClockUpdate(): per CMSIS convention.
  *      - SystemClock_Config(): NUCLEO-H743ZI2 480 MHz (VOS0) PLL1 tree --
  *        not part of upstream CMSIS, added here per bsp/README.md's
  *        documented purpose for this file ("SystemClock_Config, MPU setup").
  *      - MPU_Config(): baseline MPU enable; extend with cache-policy regions
  *        once a DMA-using peripheral driver needs one (see hal_board_init()
  *        in board.c, which calls both of these before HAL_Init()).
  *
  *      Assumptions specific to this board, called out where they matter:
  *        - HSE is the 8 MHz clock the ST-LINK's MCO outputs on PH0 (bypass
  *          mode, no crystal) -- the NUCLEO-144 default solder-bridge state.
  *        - LDO (not SMPS) supplies the core -- Nucleo boards have no SMPS
  *          inductor populated.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "stm32h7xx.h"
#include <math.h>

#if !defined(HSE_VALUE)
#define HSE_VALUE ((uint32_t)8000000) /* NUCLEO-H743ZI2: 8 MHz from ST-LINK MCO, HSE bypass */
#endif /* HSE_VALUE */

#if !defined(CSI_VALUE)
#define CSI_VALUE ((uint32_t)4000000)
#endif /* CSI_VALUE */

#if !defined(HSI_VALUE)
#define HSI_VALUE ((uint32_t)64000000)
#endif /* HSI_VALUE */

uint32_t SystemCoreClock = 64000000;
uint32_t SystemD2Clock = 64000000;
const uint8_t D1CorePrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

/**
  * @brief  Setup the microcontroller system: FPU access and flash latency
  *         reset state. Vector table stays at its post-reset address (FLASH
  *         bank 1 base) -- this BSP does not relocate it.
  * @retval None
  */
void SystemInit(void)
{
  /* FPU settings */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2))); /* set CP10 and CP11 Full Access */
#endif

  /* Reset the RCC clock configuration to the default reset state */

  /* Increasing the CPU frequency */
  if (FLASH_LATENCY_DEFAULT > (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY)))
  {
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, (uint32_t)(FLASH_LATENCY_DEFAULT));
  }

  /* Set HSION bit */
  RCC->CR |= RCC_CR_HSION;

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000;

  /* Reset HSEON, HSECSSON, CSION, HSI48ON, CSIKERON, PLL1ON, PLL2ON and PLL3ON bits */
  RCC->CR &= 0xEAF6ED7FU;

  /* Decreasing the number of wait states because of lower CPU frequency */
  if (FLASH_LATENCY_DEFAULT < (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY)))
  {
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, (uint32_t)(FLASH_LATENCY_DEFAULT));
  }

#if defined(D3_SRAM_BASE)
  RCC->D1CFGR = 0x00000000;
  RCC->D2CFGR = 0x00000000;
  RCC->D3CFGR = 0x00000000;
#else
  RCC->CDCFGR1 = 0x00000000;
  RCC->CDCFGR2 = 0x00000000;
  RCC->SRDCFGR = 0x00000000;
#endif

  /* Reset PLLCKSELR register */
  RCC->PLLCKSELR = 0x02020200;
  /* Reset PLLCFGR register */
  RCC->PLLCFGR = 0x01FF0000;
  /* Reset PLL1DIVR register */
  RCC->PLL1DIVR = 0x01010280;
  /* Reset PLL1FRACR register */
  RCC->PLL1FRACR = 0x00000000;
  /* Reset PLL2DIVR register */
  RCC->PLL2DIVR = 0x01010280;
  /* Reset PLL2FRACR register */
  RCC->PLL2FRACR = 0x00000000;
  /* Reset PLL3DIVR register */
  RCC->PLL3DIVR = 0x01010280;
  /* Reset PLL3FRACR register */
  RCC->PLL3FRACR = 0x00000000;

  /* Reset HSEBYP bit */
  RCC->CR &= 0xFFFBFFFFU;

  /* Disable all interrupts */
  RCC->CIER = 0x00000000;

#if (STM32H7_DEV_ID == 0x450UL)
  if ((DBGMCU->IDCODE & 0xFFFF0000U) < 0x20000000U)
  {
    /* Rev Y: widen the switch matrix read issuing capability for the AXI SRAM target (Target 7) */
    *((__IO uint32_t *)0x51008108) = 0x000000001U;
  }
#endif /* STM32H7_DEV_ID */

  if (READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN) == 0U)
  {
    SET_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);
    /* Disable FMC bank1 (enabled after reset) so CPU speculation can't access it
       and block other FMC masters for the ~24us it would otherwise take */
    FMC_Bank1_R->BTCR[0] = 0x000030D2;
    CLEAR_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);
  }
}

/**
  * @brief  Update SystemCoreClock from the live RCC registers. Call after any
  *         change to the clock tree that isn't made via HAL_RCC_ClockConfig()
  *         (which already updates it).
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
  uint32_t pllp, pllsource, pllm, pllfracen, hsivalue, tmp;
  uint32_t common_system_clock;
  float_t fracn1, pllvco;

  switch (RCC->CFGR & RCC_CFGR_SWS)
  {
    case RCC_CFGR_SWS_HSI:
      common_system_clock = (uint32_t)(HSI_VALUE >> ((RCC->CR & RCC_CR_HSIDIV) >> 3));
      break;

    case RCC_CFGR_SWS_CSI:
      common_system_clock = CSI_VALUE;
      break;

    case RCC_CFGR_SWS_HSE:
      common_system_clock = HSE_VALUE;
      break;

    case RCC_CFGR_SWS_PLL1:
      /* PLL_VCO = (HSE_VALUE or HSI_VALUE or CSI_VALUE / PLLM) * PLLN
         SYSCLK = PLL_VCO / PLLP */
      pllsource = (RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC);
      pllm = ((RCC->PLLCKSELR & RCC_PLLCKSELR_DIVM1) >> 4);
      pllfracen = ((RCC->PLLCFGR & RCC_PLLCFGR_PLL1FRACEN) >> RCC_PLLCFGR_PLL1FRACEN_Pos);
      fracn1 = (float_t)(uint32_t)(pllfracen * ((RCC->PLL1FRACR & RCC_PLL1FRACR_FRACN1) >> 3));

      if (pllm != 0U)
      {
        switch (pllsource)
        {
          case RCC_PLLCKSELR_PLLSRC_HSI:
            hsivalue = (HSI_VALUE >> ((RCC->CR & RCC_CR_HSIDIV) >> 3));
            pllvco = ((float_t)hsivalue / (float_t)pllm)
                     * ((float_t)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_N1) + (fracn1 / (float_t)0x2000) + (float_t)1);
            break;

          case RCC_PLLCKSELR_PLLSRC_CSI:
            pllvco = ((float_t)CSI_VALUE / (float_t)pllm)
                     * ((float_t)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_N1) + (fracn1 / (float_t)0x2000) + (float_t)1);
            break;

          case RCC_PLLCKSELR_PLLSRC_HSE:
            pllvco = ((float_t)HSE_VALUE / (float_t)pllm)
                     * ((float_t)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_N1) + (fracn1 / (float_t)0x2000) + (float_t)1);
            break;

          default:
            hsivalue = (HSI_VALUE >> ((RCC->CR & RCC_CR_HSIDIV) >> 3));
            pllvco = ((float_t)hsivalue / (float_t)pllm)
                     * ((float_t)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_N1) + (fracn1 / (float_t)0x2000) + (float_t)1);
            break;
        }
        pllp = (((RCC->PLL1DIVR & RCC_PLL1DIVR_P1) >> 9) + 1U);
        common_system_clock = (uint32_t)(float_t)(pllvco / (float_t)pllp);
      }
      else
      {
        common_system_clock = 0U;
      }
      break;

    default:
      common_system_clock = (uint32_t)(HSI_VALUE >> ((RCC->CR & RCC_CR_HSIDIV) >> 3));
      break;
  }

#if defined(RCC_D1CFGR_D1CPRE)
  tmp = D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_D1CPRE) >> RCC_D1CFGR_D1CPRE_Pos];
  common_system_clock >>= tmp;
  SystemD2Clock = (common_system_clock >> ((D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_HPRE) >> RCC_D1CFGR_HPRE_Pos]) & 0x1FU));
#else
  tmp = D1CorePrescTable[(RCC->CDCFGR1 & RCC_CDCFGR1_CDCPRE) >> RCC_CDCFGR1_CDCPRE_Pos];
  common_system_clock >>= tmp;
  SystemD2Clock = (common_system_clock >> ((D1CorePrescTable[(RCC->CDCFGR1 & RCC_CDCFGR1_HPRE) >> RCC_CDCFGR1_HPRE_Pos]) & 0x1FU));
#endif

  SystemCoreClock = common_system_clock;
}

/**
  * @brief  Exit Run* mode and configure the system power supply for LDO --
  *         the NUCLEO-H743ZI2 has no SMPS inductor populated, so LDO is the
  *         only valid choice on this board. Called from Reset_Handler before
  *         SystemInit().
  * @retval None
  */
void ExitRun0Mode(void)
{
  /* Enable LDO mode */
  PWR->CR3 |= PWR_CR3_LDOEN;
  /* Wait till voltage level flag is set */
  while ((PWR->CSR1 & PWR_CSR1_ACTVOSRDY) == 0U)
  {
  }
}

/**
  * @brief  Program PLL1 for 480 MHz SYSCLK at VOS0 (max performance) from the
  *         8 MHz HSE supplied by the ST-LINK's MCO. Must run after HAL_Init()
  *         scales the flash latency; see hal_board_init() in board.c.
  *
  *         HSE(8) /M(4) = 2 MHz VCO input * N(480) = 960 MHz VCO * /P(2) = 480 MHz.
  *         AHB = SYSCLK/2 = 240 MHz; APBx = AHB/2 = 120 MHz.
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* VOS0 requires the supply scaling step below before raising SYSCLK past 400 MHz */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY) == RESET)
  {
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS; /* clock source is the ST-LINK MCO, not a crystal */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 480;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1; /* VCO input = 8MHz/M(4) = 2MHz -> 2-4MHz range */
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while (1)
    {
    }
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
                                 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_D3PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    while (1)
    {
    }
  }
}

/**
  * @brief  Baseline MPU setup: enable the MPU with the default background
  *         region (normal, cacheable memory everywhere) so the I/D-caches
  *         enabled in hal_board_init() behave predictably. Add an explicit
  *         non-cacheable region here once a DMA-using peripheral driver needs
  *         one -- AXI SRAM (RAM_D1, where .data/.bss live, see the linker
  *         script) is cached by default, which is unsafe for buffers a DMA
  *         engine writes without CPU-side cache maintenance.
  * @retval None
  */
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();

  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x00000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
