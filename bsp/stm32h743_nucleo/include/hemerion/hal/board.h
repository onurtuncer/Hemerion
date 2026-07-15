/* ------------------------------------------------------------------------------
 * Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
 *
 * SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
 * ------------------------------------------------------------------------------ */

/**
 * @file board.h
 * @brief Board-level init for bsp/stm32h743_nucleo.
 *
 * This is bsp/stm32h743_nucleo's own copy of the
 * hal_board_init()/hal_board_reset() contract -- cmake/hemerion_hal/ (the
 * shared, board-agnostic declaration of this contract, see cmake/README.md)
 * does not exist yet. Once it lands, this header should shrink to just the
 * function bodies in board.c implementing that contract.
 */

#ifndef HEMERION_HAL_BOARD_H
#define HEMERION_HAL_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Brings the board to a known-good running state: power supply sequencing
 * (ExitRun0Mode), SystemClock_Config (480 MHz PLL1), MPU + I/D-cache enable,
 * and HAL_Init(). Call once, first thing in main(), before any other hal_*
 * function or FreeRTOS API.
 */
void hal_board_init(void);

/** Forces an immediate Cortex-M7 system reset (NVIC_SystemReset); does not return. */
void hal_board_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* HEMERION_HAL_BOARD_H */
