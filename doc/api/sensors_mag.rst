.. ------------------------------------------------------------------------------
.. Project: Hemerion
.. Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _api-sensors-mag:

=============
Magnetometer
=============

``hemerion::sensors::mag`` follows the barometer's split: a part-independent
outer namespace, and ``hemerion::sensors::mag::mmc5983ma`` for the MEMSIC
MMC5983MA three-axis AMR part.

Magnetic field is carried in **microtesla**, not tesla — one of the two
documented departures from SI in this API, and the unit the sensor literature
and the part's own datasheet use.

The MMC5983MA has no calibration NVM and no compensation polynomial, so unlike
the BMP390 there is no compensation layer: sensitivity is a fixed scalar and
decoding is a bit unpack, a null-field subtraction and
``convert_raw_to_si()``. What the driver owns instead is the **bridge offset**.
An AMR bridge drifts, and the part provides SET and RESET commands that flip
the sensing polarity; measuring in both polarities and averaging cancels the
offset. The driver's offset calibration is the non-obvious part of driving this
part correctly.

.. important::

   The part's automatic SET/RESET (``Auto_SR_en``) and the driver's software
   SET/RESET calibration are **alternatives, not layers**. Enabling automatic
   set/reset makes the part cancel the offset internally on every measurement;
   the software sequence does the same thing under driver control. Running both
   does not cancel the offset twice — it wastes measurements and confuses the
   polarity bookkeeping. Choose one.

Sample types
============

.. doxygenfile:: Hemerion/mag/mag_types.h

Unit conversion
===============

.. doxygenfile:: Hemerion/mag/mag_conversion.h

Wire protocol
=============

.. doxygenfile:: Hemerion/mag/mag_packet.h

Generic hardware simulator
==========================

Error model
-----------

.. doxygenfile:: Hemerion/mag/fmu/mag_noise_model.h

Packet emitter
--------------

.. doxygenfile:: Hemerion/mag/fmu/mag_packet_emitter.h

Magnetic calibration
--------------------

Not to be confused with ``Mmc5983maDriver::calibrate_offset()`` further down
this page. That drives the part's SET/RESET pair and cancels the **bridge
offset**, an artefact of the sensor die that is present with no vehicle around
it at all. This solves for the **magnetic environment the part is bolted
into** — the vehicle's own ferrous structure and currents — which a
bridge-calibrated part still reads in full.

An installation turns the true field into :math:`m = S\,B + h`: a constant
hard-iron offset :math:`h` in body axes, and a near-identity soft-iron matrix
:math:`S` that redirects the ambient field rather than adding to it. Rotating
through every attitude under a field of constant magnitude traces a *sphere*
in truth and an *ellipsoid* in measurement, so fitting the ellipsoid and
mapping it back to a sphere is the calibration.

The accumulator is :math:`O(1)` in the sample count — each sample is folded
into a fixed 9×9 normal matrix as it arrives — with no allocation, no
exceptions and compile-time loop bounds, so it runs on the STM32H743 during a
calibration manoeuvre rather than only in a ground tool.

.. warning::

   An ellipsoid fit cannot recover :math:`S` uniquely. For any rotation
   :math:`R`, :math:`\|R\,S^{-1}(m-h)\| = \|S^{-1}(m-h)\|`, so the samples
   constrain the ellipsoid's *shape* but not its orientation in the body
   frame. What is returned is the symmetric positive-definite square root —
   the canonical choice, and the correct one whenever the true :math:`S` is
   itself symmetric, which is what the MMC5983MA simulator models. On hardware
   the residual rotation must come from another reference, usually the IMU.

.. doxygenfile:: Hemerion/mag/magnetic_calibration.h

MEMSIC MMC5983MA
================

Register map
------------

.. doxygenfile:: Hemerion/mag/mmc5983ma/mmc5983ma_registers.h

I2C driver
----------

.. doxygenfile:: Hemerion/mag/mmc5983ma/mmc5983ma_driver.h

On-hardware bus adapter
-----------------------

.. doxygenfile:: Hemerion/mag/mmc5983ma/mmc5983ma_hal_i2c_bus.h

MMC5983MA hardware simulator
============================

Measurement model
-----------------

.. doxygenfile:: Hemerion/mag/mmc5983ma/fmu/mmc5983ma_measurement_model.h

Simulated I2C part
------------------

.. doxygenfile:: Hemerion/mag/mmc5983ma/fmu/mmc5983ma_i2c_slave.h
