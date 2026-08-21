.. ------------------------------------------------------------------------------
.. Project: Hemerion
.. Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _api-bsp-hal:

=====================================
Board support package: NUCLEO-H743ZI2
=====================================

``bsp/stm32h743_nucleo`` is the only board Hemerion currently supports, and its
hardware abstraction layer is the one piece of the API that is C rather than
C++: a flat set of ``hal_*`` functions in ``extern "C"``, with no namespace and
no types beyond the fixed-width integers. That is deliberate — it is the
boundary the firmware crosses to reach silicon, and keeping it a C ABI lets the
vendor HAL below it and the C++ modules above it each be compiled on their own
terms.

Each header repeats a contract that ``cmake/README.md`` describes for a shared
``hemerion_hal/`` directory. That directory does not exist yet, so these are
the BSP's own copies; a second board would start by copying them, and the
duplication is the signal that the shared contract still needs extracting.

Only what has a consumer is wired up. ``hal_i2c_*`` supports ``instance == 1``
(I2C1 on PB8/PB9, the Arduino D15/D14 header pins) because the BMP390 and
MMC5983MA drivers need it; other instances return ``false`` rather than
pretending. The same principle applies across the layer.

.. seealso::

   :doc:`../bsp_freertos_wiring` for how these calls are reached from a running
   FreeRTOS system, and :doc:`../swil_windows_setup` for running firmware built
   against this BSP under Renode.

Board initialisation
====================

.. doxygenfile:: hemerion/hal/board.h

GPIO
====

.. doxygenfile:: hemerion/hal/gpio.h

UART
====

.. doxygenfile:: hemerion/hal/uart.h

I2C master
==========

.. doxygenfile:: hemerion/hal/i2c.h
