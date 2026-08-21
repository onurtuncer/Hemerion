.. ------------------------------------------------------------------------------
.. Project: Hemerion
.. Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _api-power:

=====
Power
=====

``hemerion::power`` holds two pure, peripheral-free pieces of the power
subsystem: a battery telemetry model that turns a pack sample plus a set of
limits into a fault bitmask, and a regulator sequencer that enforces a rail
enable order.

Neither touches an ADC, a BMS part, a GPIO or the HAL. They take values in and
produce a verdict or a next action out, which is what makes both of them
exhaustively testable natively — and it is why the faults they identify are
handed to ``hemerion::fault`` to record rather than acted on here.

Battery monitor
===============

.. doxygenfile:: hemerion/power/battery_monitor.h

Regulator sequencer
===================

.. doxygenfile:: hemerion/power/regulator_sequencer.h
