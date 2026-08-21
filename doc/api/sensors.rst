.. ------------------------------------------------------------------------------
.. Project: Hemerion
.. Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _api-sensors:

=======
Sensors
=======

``hemerion::sensors`` is the largest module and the one with the most regular
shape. Five sensor families are built — IMU, barometer, magnetometer, GPS and
radar altimeter — and, GPS aside, they are all the same six pieces in the same
arrangement. Reading one stack teaches the others.

The shared shape
================

.. list-table::
   :header-rows: 1
   :widths: 24 16 60

   * - Piece
     - Tier
     - What it is
   * - ``*_types.h``
     - both
     - ``XRawSample`` (register counts as read off the part), ``XSample``
       (the same quantity in SI units), ``XScale`` (the sensitivity relating
       them), and the conversion error enum.
   * - ``*_conversion.h``
     - both
     - ``convert_raw_to_si()``: the one function that applies a scale. Pure,
       allocation-free, and the same code on target and in the simulator.
   * - ``*_packet.h``
     - both
     - The Hemerion wire protocol for shipping raw samples off a board:
       framing constants and a streaming parser that consumes one byte at a
       time.
   * - ``*_registers.h`` / ``*_protocol.h``
     - both
     - The part's register map, transcribed from its datasheet. The single
       source of truth for the driver and the simulated part alike.
   * - ``*_driver.h``
     - on-target
     - The bring-up and read sequence, over an *injected* bus interface. This
       is the code that runs on the STM32H743.
   * - ``fmu/``
     - host-only
     - The simulator side: an error model that turns truth into realistic
       counts, and either a packet emitter or a simulated part that answers a
       bus.

Two ways a sensor is simulated
==============================

The ``fmu/`` half comes in two flavours, and the difference matters when
reading these pages.

A **signal-level** simulator (the generic barometer, magnetometer and radar
altimeter FMUs, and the GPS one) applies a noise model to truth, quantizes the
result to register counts, and emits a framed packet over UDP. What is
exercised is the wire protocol and the conversion, not the driver.

A **register-accurate** simulator (BMP390, MMC5983MA, and the IMU's SPI part)
goes further: it implements the part's actual register file and answers a
simulated bus, so the real on-target driver performs its real bring-up
sequence — chip ID check, soft reset, calibration NVM read, oversampling
configuration, data-ready polling, burst read — against it. This is what makes
a co-simulation a test of the driver rather than merely of the maths, and it is
why those parts have both a ``*_i2c_slave.h`` and a ``*_measurement_model.h``.

The measurement model of a register-accurate part is, by construction, the
inverse of the driver's compensation: it takes a truth value and produces the
raw words that would compensate back to it.

Sensor families
===============

.. toctree::
   :maxdepth: 2

   sensors_imu
   sensors_baro
   sensors_mag
   sensors_gps
   sensors_radalt

.. seealso::

   :doc:`../sensor_models` covers the physics and error models behind these
   types in narrative form; this reference covers their interfaces.
