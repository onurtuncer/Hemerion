.. ------------------------------------------------------------------------------
.. Project: Hemerion
.. Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _api-sensors-radalt:

================
Radar altimeter
================

``hemerion::sensors::radalt`` measures height above ground level. It is the
simplest of the five families: sample types, conversion, wire protocol, and a
signal-level hardware simulator. No specific part is modelled yet, so there is
no register map and no on-target driver here — a radar altimeter's output on
this airframe arrives as framed packets rather than over a sensor bus.

What distinguishes it from the other families is that *no return* is a normal
operating state, not an error. Out of range, over water, or during a loss of
track, the part reports that it has no valid range rather than reporting a
wrong one. That is carried as a status word alongside the range, and both the
error model and any consumer have to treat it as a first-class case.

Sample types
============

.. doxygenfile:: Hemerion/radalt/radalt_types.h

Unit conversion
===============

.. doxygenfile:: Hemerion/radalt/radalt_conversion.h

Wire protocol
=============

.. doxygenfile:: Hemerion/radalt/radalt_packet.h

Hardware simulator
==================

Error model
-----------

.. doxygenfile:: Hemerion/radalt/fmu/radalt_noise_model.h

Packet emitter
--------------

.. doxygenfile:: Hemerion/radalt/fmu/radalt_packet_emitter.h
