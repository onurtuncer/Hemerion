/*
 * ------------------------------------------------------------------------------
 * Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
 *
 * SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
 * ------------------------------------------------------------------------------
 * Linker script for STM32H743ZIT6 (NUCLEO-H743ZI2): 2048K dual-bank flash,
 * 1MB RAM split across the H7 domain SRAMs. .data/.bss/heap live in RAM_D1
 * (AXI SRAM) so DMA peripherals can reach them; the main stack lives in
 * DTCMRAM for fast, deterministic context switches. RAM_D2/RAM_D3 are left
 * unused by this script -- claim them explicitly with PROVIDE() symbols in
 * an app's own linker fragment if a driver needs D2/D3-domain placement.
 * ------------------------------------------------------------------------------
 */

ENTRY(Reset_Handler)

/* Top of DTCMRAM -- main stack lives here, separate from .data/.bss in RAM_D1 */
_estack = 0x20020000;

_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x400;

MEMORY
{
  ITCMRAM (xrw)  : ORIGIN = 0x00000000, LENGTH = 64K
  FLASH   (rx)   : ORIGIN = 0x08000000, LENGTH = 2048K
  DTCMRAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_D1  (xrw)  : ORIGIN = 0x24000000, LENGTH = 512K
  RAM_D2  (xrw)  : ORIGIN = 0x30000000, LENGTH = 288K
  RAM_D3  (xrw)  : ORIGIN = 0x38000000, LENGTH = 64K
}

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } >FLASH

  /* Used by startup_stm32h743xx.s to copy .data from flash to RAM_D1 at boot */
  _sidata = LOADADDR(.data);

  .data : ALIGN(4)
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM_D1 AT> FLASH

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM_D1

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = ALIGN(8);
  } >RAM_D1

  /DISCARD/ :
  {
    *(.ARM.attributes)
  }
}
