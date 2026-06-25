/* ------------------------------------------------------------------------------
 * Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
 *
 * SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
 * ------------------------------------------------------------------------------
 * UART HAL for bsp/stm32h743_nucleo. See board.h's header comment: this is
 * the BSP's own copy of the contract documented in cmake/README.md's
 * hemerion_hal/ snippet, since that shared directory doesn't exist yet.
 *
 * Only `instance == 3` (USART3, the ST-LINK Virtual COM Port on PD8/PD9) is
 * wired up. Other instances are a no-op/false return -- there is no other
 * UART consumer yet to justify building out the rest of the table.
 * ------------------------------------------------------------------------------ */

#ifndef HEMERION_HAL_UART_H
#define HEMERION_HAL_UART_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Configures USARTx for 8N1, blocking (polling) transfers at baud_rate. */
void hal_uart_init(uint8_t instance, uint32_t baud_rate);

/** Blocking write of len bytes; returns once all bytes are transmitted. */
void hal_uart_write(uint8_t instance, const uint8_t *data, size_t len);

/** Blocking read of up to len bytes; returns the number of bytes actually read. */
size_t hal_uart_read(uint8_t instance, uint8_t *data, size_t len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* HEMERION_HAL_UART_H */
