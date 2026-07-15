/* ------------------------------------------------------------------------------
 * Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
 *
 * SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
 * ------------------------------------------------------------------------------ */

/**
 * @file gpio.h
 * @brief GPIO HAL for bsp/stm32h743_nucleo.
 *
 * See board.h's header comment: this is the BSP's own copy of the contract
 * documented in cmake/README.md's hemerion_hal/ snippet, since that shared
 * directory doesn't exist yet.
 *
 * `port` is the GPIOx letter ('A'..'I' on the LQFP144 package this MCU
 * ships in); `pin` is 0..15. Ports/pins outside that range are a programmer
 * error -- configASSERT()-style behavior, not a runtime error path.
 */

#ifndef HEMERION_HAL_GPIO_H
#define HEMERION_HAL_GPIO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Pin direction/output-type selected by hal_gpio_set_mode(). */
typedef enum
{
  HAL_GPIO_MODE_INPUT,             /**< High-impedance input. */
  HAL_GPIO_MODE_OUTPUT_PUSH_PULL,  /**< Push-pull output. */
  HAL_GPIO_MODE_OUTPUT_OPEN_DRAIN, /**< Open-drain output. */
  HAL_GPIO_MODE_ANALOG,            /**< Analog mode (ADC/DAC, lowest leakage). */
} hal_gpio_mode_t;

/** Configures one pin's direction/output-type; pull resistors are left floating.
 *  Also enables that port's peripheral clock, as do hal_gpio_write()/_read() below --
 *  there is no separate "enable this port" call to forget. */
void hal_gpio_set_mode(uint8_t port, uint8_t pin, hal_gpio_mode_t mode);

/** Drives an output pin high (true) or low (false). */
void hal_gpio_write(uint8_t port, uint8_t pin, bool level);

/** Reads the current input level of a pin, regardless of its configured mode. */
bool hal_gpio_read(uint8_t port, uint8_t pin);

#ifdef __cplusplus
}
#endif

#endif /* HEMERION_HAL_GPIO_H */
