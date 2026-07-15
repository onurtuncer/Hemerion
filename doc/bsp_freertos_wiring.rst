.. ------------------------------------------------------------------------------
.. Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _bsp_freertos_wiring:

BSP Internals: FreeRTOS and MPU on STM32H743
=============================================

This page documents the non-obvious wiring decisions in
``bsp/stm32h743_nucleo`` that are required to run FreeRTOS's GCC_ARM_CM7
port correctly on the NUCLEO-H743ZI2 board, and two real bugs that were
found and fixed when the BSP was first executed (during ``apps/led_blink``
bring-up). Neither was caught in code review because the BSP had never
actually run before.

.. contents:: On this page
   :local:
   :depth: 2

---

FreeRTOS handler routing: Direct vs Indirect
---------------------------------------------

The FreeRTOS GCC_ARM_CM7 port (``vendor/FreeRTOS-Kernel/portable/GCC/ARM_CM7/r0p1/``)
defines three OS-critical exception handlers under its own names:

.. list-table::
   :header-rows: 1

   * - FreeRTOS symbol
     - Exception
   * - ``vPortSVCHandler``
     - SVCall (SVC #0 — starts the first task)
   * - ``xPortPendSVHandler``
     - PendSV (context switch)
   * - ``xPortSysTickHandler``
     - SysTick (RTOS tick)

CMSIS and the ST startup file ``startup_stm32h743xx.s`` expect these to be
named ``SVC_Handler``, ``PendSV_Handler``, and ``SysTick_Handler``.

There are two ways to connect them:

**Direct Routing** — Install the FreeRTOS symbols directly in the vector table.
This requires the vector table to be in writable RAM (so the port can patch it
at scheduler startup), or that the application links the FreeRTOS symbols as
the exception handler names via ``PROVIDE`` in the linker script. The port
validates this at startup when ``configCHECK_HANDLER_INSTALLATION == 1``
(the default).

**Indirect Routing** — Install CMSIS-named wrappers that branch to the FreeRTOS
symbols. Required when the vector table lives in read-only flash (as in this
project — see ``linker/stm32h743zi_flash.ld``). Must set
``configCHECK_HANDLER_INSTALLATION 0`` to suppress the Direct-Routing
startup check.

This BSP uses **Indirect Routing** because ``.isr_vector`` is placed in
flash and cannot be patched at runtime.

---

``bsp/stm32h743_nucleo/src/freertos_hooks.c``
----------------------------------------------

The wrappers live in ``freertos_hooks.c`` alongside the FreeRTOS application
hooks (static-allocation buffers, malloc-failure trap, stack-overflow trap):

.. code-block:: c

    extern void vPortSVCHandler(void);
    extern void xPortPendSVHandler(void);
    extern void xPortSysTickHandler(void);

    void SVC_Handler(void) __attribute__((naked));
    void SVC_Handler(void) { __asm volatile("b vPortSVCHandler"); }

    void PendSV_Handler(void) __attribute__((naked));
    void PendSV_Handler(void) { __asm volatile("b xPortPendSVHandler"); }

    void SysTick_Handler(void) { xPortSysTickHandler(); }

Why ``SVC_Handler`` and ``PendSV_Handler`` are ``naked``:
``vPortSVCHandler`` and ``xPortPendSVHandler`` are themselves naked functions
that inspect the exception stack frame as if they had been entered from hardware.
A normal ``bl`` call would push an extra stack frame, corrupting that
inspection. The ``b`` (branch without link) instruction transfers control
without touching the stack, preserving the hardware-stacked frame layout the
FreeRTOS handlers expect.

``SysTick_Handler`` is a normal function call — ``xPortSysTickHandler`` is not
naked; it behaves like a regular C function called from the SysTick exception
context.

---

``configCHECK_HANDLER_INSTALLATION``
--------------------------------------

``vendor/FreeRTOS-Kernel/portable/GCC/ARM_CM7/r0p1/port.c``'s
``xPortStartScheduler()`` contains:

.. code-block:: c

    #if ( configCHECK_HANDLER_INSTALLATION == 1 )
    {
        const portISR_t * const pxVectorTable = portSCB_VTOR_REG;
        configASSERT( pxVectorTable[ portVECTOR_INDEX_SVC ] == vPortSVCHandler );
        configASSERT( pxVectorTable[ portVECTOR_INDEX_PENDSV ] == xPortPendSVHandler );
    }
    #endif

With Indirect Routing the vector table entries are ``SVC_Handler``/
``PendSV_Handler`` (the wrappers), **not** ``vPortSVCHandler``/
``xPortPendSVHandler`` themselves. The assertion fails and, because
``configASSERT`` is defined as a ``taskDISABLE_INTERRUPTS()`` + infinite loop
in ``FreeRTOSConfig.h``, the firmware silently hangs before the scheduler
ever starts.

The fix — in ``bsp/stm32h743_nucleo/FreeRTOSConfig.h``:

.. code-block:: c

    /* .isr_vector is in flash; Indirect Routing is used via freertos_hooks.c.
     * Disables the Direct-Routing vector-table assertion in xPortStartScheduler(). */
    #define configCHECK_HANDLER_INSTALLATION 0

This was a pre-existing bug first caught by ``apps/led_blink`` boot
(symptoms: firmware hung at ``xPortStartScheduler`` line 330,
PC frozen at ``0x8002620``).

---

MPU background region: ``DisableExec``
----------------------------------------

``system_stm32h7xx.c``'s ``MPU_Config()`` configures a background MPU region
covering the entire 4 GB address space (``ARM_MPU_REGION_SIZE_4GB``) as a
strongly-ordered catch-all that disallows unprivileged access. The intent is
that code regions overlay this with narrower, more permissive regions.

A pre-existing bug had:

.. code-block:: c

    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;  // WRONG

Because this region covers **all 4 GB**, disabling instruction fetch on it made
every memory address non-executable — including the firmware's own flash. The
processor took a MemManage Fault on the very first instruction fetch after
``MPU_Enable()``, which itself is called before ``main()`` inside
``SystemInit()``. The correct value:

.. code-block:: c

    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

Narrower overlay regions for specific peripherals (SRAM vs flash vs device
space) may override this with ``MPU_INSTRUCTION_ACCESS_DISABLE`` where
appropriate — the background region must permit execution so the overlay can
selectively restrict it, not the other way around.

---

Renode: ``init add:`` vs ``init:`` in ``.repl`` files
--------------------------------------------------------

This is not a BSP issue but was discovered during SWIL bring-up of the same
BSP, so it is documented here.

When a Renode ``board.repl`` file includes a CPU platform via ``using``:

.. code-block:: text

    using "platforms/cpus/stm32h743.repl"

the bundled CPU description installs sysbus Tags for peripheral regions that
Renode's model doesn't implement (including ``D3CR`` at ``0x58024818`` with a
constant ``0x2000`` so that the ``VOSRDY`` voltage-scaling ready bit appears
set). A board-level ``init:`` block that adds further Tags must use
``init add:`` **not** ``init:``:

.. code-block:: text

    sysbus:
        init add:               # ← CORRECT: appends to the inherited init list
            Tag <0x58024804, 0x58024804> "CSR1" 0x2000

.. code-block:: text

    sysbus:
        init:                   # ← WRONG: replaces the inherited init list,
            Tag <0x58024804, 0x58024804> "CSR1" 0x2000
            #   silently drops the parent's D3CR and all other sysbus tags

Using a bare ``init:`` discards the parent's entire sysbus init list. The
``D3CR`` tag disappears, the firmware's voltage-scaling busy-wait in
``SystemClock_Config()`` reads ``0x00000000`` instead of ``0x00002000``,
and the loop spins forever. This does **not** produce any Renode warning —
the register still reads as zero (untagged memory), which is a valid but
wrong response. See ``sim/renode/boards/nucleo_h743zi2.repl`` for the
corrected form.

---

HAL DMA dependency
-------------------

``stm32h7xx_hal_uart.h`` in the ST HAL vendor library references
``DMA_HandleTypeDef`` unconditionally inside ``UART_HandleTypeDef``, regardless
of whether DMA is used by the application. Without ``HAL_DMA_MODULE_ENABLED``
defined, the DMA struct definition is not included and the UART header fails
to compile.

Fix in ``bsp/stm32h743_nucleo/include/stm32h7xx_hal_conf.h``:

.. code-block:: c

    #define HAL_DMA_MODULE_ENABLED
    #include "stm32h7xx_hal_dma.h"
