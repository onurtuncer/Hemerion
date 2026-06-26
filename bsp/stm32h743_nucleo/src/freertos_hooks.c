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

void SysTick_Handler(void)
{
  xPortSysTickHandler();
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
