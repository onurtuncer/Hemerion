.. ------------------------------------------------------------------------------
.. Project: Hemerion
.. Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _api-fault:

=============
Fault manager
=============

``hemerion::fault`` is bookkeeping, deliberately and completely. It records
which fault codes exist and which are currently active, and it tracks whether
each supervised deadline is still being met. It has no opinion on how a fault
is *detected* — that belongs to the module owning the condition, such as
``hemerion::power``'s battery limit evaluation — and none on what should
*happen* when one is active.

That second half is the health monitor, which is not built yet. When it is, it
will poll the registry's highest active severity and the watchdog's channel
status, and decide whether to trip the safe state and stop feeding the
hardware independent watchdog. Until then nothing in this module causes a
reset on its own, and the watchdog here is a pure software deadline supervisor
rather than a driver for the STM32's IWDG.

Both types are fixed-capacity and registered up front: codes and channels are
declared during initialisation and never allocated afterwards, so raising a
fault from a task at run time cannot fail for want of memory.

Fault registry
==============

.. doxygenfile:: hemerion/fault/fault_registry.h

Watchdog supervisor
===================

.. doxygenfile:: hemerion/fault/watchdog.h
