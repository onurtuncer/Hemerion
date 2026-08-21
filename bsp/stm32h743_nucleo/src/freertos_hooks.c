/* ------------------------------------------------------------------------------
 * Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
 *
 * SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
 * ------------------------------------------------------------------------------
 * Application hooks FreeRTOSConfig.h's configSUPPORT_STATIC_ALLOCATION,
 * configUSE_MALLOC_FAILED_HOOK, and configCHECK_FOR_STACK_OVERFLOW commit
 * this BSP to providing: static buffers for the idle/timer tasks, and a
 * trap for malloc failure / stack overflow. Every app linking this BSP gets
 * one canonical implementation here rather than redefining these per app.
 * ------------------------------------------------------------------------------ */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "stm32h7xx_hal.h"

/* vendor/FreeRTOS-Kernel/portable/GCC/ARM_CM7/r0p1/port.c defines
   vPortSVCHandler/xPortPendSVHandler/xPortSysTickHandler, not the CMSIS
   vector-table names -- with .isr_vector in flash (read-only, see
   linker/stm32h743zi_flash.ld), the port can't patch the vector table
   itself at startup, so this BSP has to provide the CMSIS-named handlers
   and route them through by hand ("Indirect Routing" in port.c's header
   comment on xPortStartScheduler()). SVC/PendSV must branch rather than
   call: vPortSVCHandler/xPortPendSVHandler are naked and inspect the
   exception stack frame as if dispatched directly from hardware, which an
   ordinary call (extra stacked LR) would corrupt. */
extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

void SVC_Handler(void) __attribute__((naked));
void SVC_Handler(void)
{
  __asm volatile("b vPortSVCHandler");
}

void PendSV_Handler(void) __attribute__((naked));
void PendSV_Handler(void)
{
  __asm volatile("b xPortPendSVHandler");
}

/* Two tick consumers share this one interrupt, and both have to be served.

   HAL_IncTick() advances uwTick, which is the only thing HAL_GetTick() reads
   and therefore the only thing that lets HAL_Delay() terminate. ST's own
   stm32h7xx_it.c calls it from here; this BSP replaced that file wholesale to
   route SVC/PendSV/SysTick to the FreeRTOS port (see above) and did not carry
   the call over. Nothing else in the tree calls HAL_IncTick(), so uwTick sat at
   zero for the life of the program and every HAL_Delay() spun forever.

   That is not a latent defect: apps/baro_logger hung on the BMP390's 2 ms
   soft-reset wait (Bmp390Driver::probe -> hal_delay_ms -> HAL_Delay) before
   printing its first UART byte, in Renode and identically on real silicon --
   the fault was read as an emulated-I2C problem for exactly as long as nobody
   looked at the program counter. apps/led_blink never calls HAL_Delay, which is
   the whole reason it was unaffected and made the platform look healthy.

   The scheduler-state guard is the second half. HAL_Init() enables SysTick long
   before vTaskStartScheduler() does, so this handler fires while the kernel is
   still uninitialised; xPortSysTickHandler() assumes a running scheduler and
   touches its ready lists. It has been reached in that state on every boot so
   far without visible harm, which is not the same as being safe. HAL's tick
   must advance from HAL_Init() onward -- pre-scheduler HAL_Delay() calls in
   board bring-up depend on it -- so the increment is unconditional and only the
   kernel half is gated. */
void SysTick_Handler(void)
{
  HAL_IncTick();

  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    xPortSysTickHandler();
  }
}

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize)
{
  static StaticTask_t idle_task_tcb;
  static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];

  *ppxIdleTaskTCBBuffer = &idle_task_tcb;
  *ppxIdleTaskStackBuffer = idle_task_stack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize)
{
  static StaticTask_t timer_task_tcb;
  static StackType_t timer_task_stack[configTIMER_TASK_STACK_DEPTH];

  *ppxTimerTaskTCBBuffer = &timer_task_tcb;
  *ppxTimerTaskStackBuffer = timer_task_stack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void vApplicationMallocFailedHook(void)
{
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}
