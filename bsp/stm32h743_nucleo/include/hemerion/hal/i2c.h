/* ------------------------------------------------------------------------------
 * Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
 *
 * SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
 * ------------------------------------------------------------------------------ */

/**
 * @file i2c.h
 * @brief I2C master HAL for bsp/stm32h743_nucleo.
 *
 * See board.h's header comment: this is the BSP's own copy of the contract
 * documented in cmake/README.md's hemerion_hal/ snippet, since that shared
 * directory doesn't exist yet.
 *
 * Only `instance == 1` (I2C1 on PB8/PB9, the NUCLEO-H743ZI2's Arduino
 * D15/D14 header pins) is wired up, in 100 kHz standard mode. Other
 * instances are a no-op/false return -- there is no other I2C consumer yet
 * to justify building out the rest of the table.
 *
 * The register-oriented mem_read/mem_write shape (rather than raw
 * transmit/receive) is deliberate: it is the transaction granularity every
 * register-mapped I2C sensor uses, it maps 1:1 onto HAL_I2C_Mem_Read/Write,
 * and it is what modules/sensors' bus adapters (e.g.
 * Hemerion/baro/bmp390/bmp390_hal_i2c_bus.h) consume.
 */

#ifndef HEMERION_HAL_I2C_H
#define HEMERION_HAL_I2C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Configures I2Cx as a bus master, 7-bit addressing, 100 kHz standard mode. */
void hal_i2c_init(uint8_t instance);

/**
 * Blocking register write: START, target+W, reg, data bytes, STOP.
 *
 * @param target_address 7-bit target address (unshifted).
 * @return true once every byte is acknowledged and transmitted; false on
 *         NACK, bus error, timeout, or an uninitialized instance.
 */
bool hal_i2c_mem_write(uint8_t instance,
                       uint8_t target_address,
                       uint8_t reg,
                       const uint8_t* data,
                       size_t len,
                       uint32_t timeout_ms);

/**
 * Blocking register read: START, target+W, reg, repeated START, target+R,
 * data bytes, STOP.
 *
 * @param target_address 7-bit target address (unshifted).
 * @return true once len bytes are received; false on NACK, bus error,
 *         timeout, or an uninitialized instance.
 */
bool hal_i2c_mem_read(uint8_t instance,
                      uint8_t target_address,
                      uint8_t reg,
                      uint8_t* data,
                      size_t len,
                      uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* HEMERION_HAL_I2C_H */
