.. ------------------------------------------------------------------------------
.. Project: Hemerion
.. Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _api-rtos-core:

=========
RTOS core
=========

``hemerion::rtos_core`` is the static scaffolding an application declares
before FreeRTOS starts: which tasks exist, which queues connect them, and
where their payloads live. None of it calls FreeRTOS. The registries are
plain fixed-capacity tables that validate a declaration and hand it back; the
BSP is what walks a populated registry and issues the ``xTaskCreate()`` calls.

That split is what makes the scheduling topology testable. A registry can be
populated and checked in a native unit test with no scheduler present, and the
same registry drives the real system on target. It also keeps this module free
of ``FreeRTOSConfig.h``, which is why the tick conversion below takes a tick
rate as an argument rather than reading ``configTICK_RATE_HZ``.

.. seealso::

   :doc:`../bsp_freertos_wiring` describes how a populated registry becomes
   running FreeRTOS tasks on the NUCLEO-H743ZI2.

Task registry
=============

.. doxygenfile:: hemerion/rtos_core/task_registry.h

Task priorities
===============

.. doxygenfile:: hemerion/rtos_core/task_priority.h

Queue registry
==============

.. doxygenfile:: hemerion/rtos_core/queue_registry.h

Memory pools
============

.. doxygenfile:: hemerion/rtos_core/memory_pool.h

Tick conversion
===============

.. doxygenfile:: hemerion/rtos_core/tick.h
