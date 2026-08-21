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
raw words that would compensate back to it. Following the BMP390 all the way
round shows what that buys — the loop closes, and every box on the right-hand
side is the same code that runs on the STM32:

.. graphviz::
   :align: center
   :caption: A register-accurate sensor round trip. The simulator encodes
             truth into the words real silicon would have produced; the
             firmware decodes them back. What is being tested is the whole
             right-hand side, not just the arithmetic.
   :alt: A truth altitude enters the BMP390 measurement model, which applies
         the atmosphere and error model and the inverse compensation to
         produce raw conversion words. Those are served from the simulated
         part's register file over the shared-memory I2C bus to Bmp390Driver,
         which reads them and runs the datasheet compensation to recover a
         BaroSample in pascals and degrees Celsius, agreeing with the original
         truth to within the modelled error.

   digraph sensor_roundtrip {
       bgcolor="transparent"
       rankdir=LR
       node [shape=box style="rounded,filled" color="#41597a" fontname="Helvetica"
             fontsize=10 margin="0.16,0.09"]
       edge [color="#41597a" fontname="Helvetica" fontsize=9]

       subgraph cluster_host {
           label="host-only (fmu/)"
           fontname="Helvetica"
           fontsize=10
           color="#9aa7b5"
           style=dashed
           truth [label="truth altitude\nfrom the plant model" fillcolor="#eef3f8"]
           model [label="Bmp390MeasurementModel\natmosphere + error model\n+ inverse compensation" fillcolor="#eef3f8"]
           slave [label="Bmp390I2cSlave\nregister file, drdy, status" fillcolor="#eef3f8"]
       }

       subgraph cluster_target {
           label="same object code as on the STM32H743"
           fontname="Helvetica"
           fontsize=10
           color="#9aa7b5"
           style=dashed
           driver [label="Bmp390Driver\nbring-up + burst read" fillcolor="#d6e4f2"]
           comp   [label="Bmp390Compensator\ndatasheet 8.4" fillcolor="#d6e4f2"]
           sample [label="BaroSample\nPa, degrees C" fillcolor="#d6e4f2"]
       }

       bus [label="simulated I2C bus\nsim/i2c_shm" shape=box style=filled fillcolor="#e6e6e6"]

       truth  -> model  [label="  h [m]"]
       model  -> slave  [label="  raw 24-bit words"]
       slave  -> bus
       bus    -> driver [label="  register reads"]
       driver -> comp
       comp   -> sample
       sample -> truth  [label="  agrees to within\l  the modelled error\l"
                         style=dashed constraint=false color="#7a8a9a"]
   }

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
