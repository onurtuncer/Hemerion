/**
  ******************************************************************************
  * @file    stm32h7xx_hal_conf.h
  * @brief   HAL configuration for bsp/stm32h743_nucleo -- adapted from
  *          vendor/stm32h7xx-hal-driver/Inc/stm32h7xx_hal_conf_template.h per
  *          vendor/CMakeLists.txt's documented convention (the BSP owns its
  *          own copy of the template, on its include path ahead of
  *          STM32H7xx::HAL). Only the modules this BSP currently links
  *          (board.c, gpio.c, uart.c) are enabled; add more as drivers land.
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

#ifndef STM32H7xx_HAL_CONF_H
#define STM32H7xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ########################## Module Selection ############################## */
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED /* UART_HandleTypeDef references DMA_HandleTypeDef unconditionally */
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* ########################## Oscillator Values adaptation ################## */
#if !defined(HSE_VALUE)
#define HSE_VALUE (8000000UL) /* NUCLEO-H743ZI2: ST-LINK MCO, HSE bypass -- see system_stm32h7xx.c */
#endif /* HSE_VALUE */

#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT (100UL)
#endif /* HSE_STARTUP_TIMEOUT */

#if !defined(CSI_VALUE)
#define CSI_VALUE (4000000UL)
#endif /* CSI_VALUE */

#if !defined(HSI_VALUE)
#define HSI_VALUE (64000000UL)
#endif /* HSI_VALUE */

#if !defined(LSE_VALUE)
#define LSE_VALUE (32768UL)
#endif /* LSE_VALUE */

#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT (5000UL)
#endif /* LSE_STARTUP_TIMEOUT */

#if !defined(LSI_VALUE)
#define LSI_VALUE (32000UL)
#endif /* LSI_VALUE */

#if !defined(EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE 12288000UL
#endif /* EXTERNAL_CLOCK_VALUE */

/* ########################### System Configuration ########################## */
#define VDD_VALUE (3300UL)
#define TICK_INT_PRIORITY (0x0FUL)
#define USE_RTOS 0
#define USE_SD_TRANSCEIVER 0U
#define USE_SPI_CRC 1U
#define USE_FLASH_ECC 0U
#define USE_SDIO_TRANSCEIVER 0U
#define SDIO_MAX_IO_NUMBER 7U

#define USE_HAL_UART_REGISTER_CALLBACKS 0U

/* ########################## Assert Selection ############################## */
/* #define USE_FULL_ASSERT 1 */

/* Includes -------------------------------------------------------------------*/
#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32h7xx_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32h7xx_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32h7xx_hal_dma.h"
#endif /* HAL_DMA_MODULE_ENABLED */

#ifdef HAL_EXTI_MODULE_ENABLED
#include "stm32h7xx_hal_exti.h"
#endif /* HAL_EXTI_MODULE_ENABLED */

#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32h7xx_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32h7xx_hal_flash.h"
#endif /* HAL_FLASH_MODULE_ENABLED */

#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32h7xx_hal_pwr.h"
#endif /* HAL_PWR_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
#include "stm32h7xx_hal_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

/* Exported macro ---------------------------------------------------------- */
#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

#ifdef __cplusplus
}
#endif

#endif /* STM32H7xx_HAL_CONF_H */
