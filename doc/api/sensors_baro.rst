.. ------------------------------------------------------------------------------
.. Project: Hemerion
.. Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _api-sensors-baro:

==========
Barometer
==========

``hemerion::sensors::baro`` splits in two. The outer namespace holds the
part-independent layer — sample types, conversion, and the Hemerion wire
protocol — while ``hemerion::sensors::baro::bmp390`` holds everything specific
to the Bosch BMP390 that the flight hardware actually carries.

The BMP390 is the most fully modelled part in the tree, and it is worth
understanding why the pieces are arranged as they are. The part ships raw
24-bit conversion words that mean nothing until they are run through the
floating-point compensation polynomial of datasheet section 8.4, using
coefficients burned into each individual part's NVM at the factory. So:

* ``bmp390_registers.h`` transcribes the register map and the NVM layout, and
  parses a raw 21-byte calibration block into ``Bmp390CalibData``.
* ``bmp390_compensation.h`` turns those words into pascals and degrees Celsius.
* ``bmp390_driver.h`` runs the bring-up and read sequence over an injected
  ``Bmp390I2cBus``, so the same driver object code runs on hardware
  (``bmp390_hal_i2c_bus.h``), in co-simulation (over ``sim/i2c_shm``), and
  under Renode.
* The ``fmu/`` half inverts all of it: given a truth altitude, an atmosphere
  and error model produce the uncompensated words that the compensation above
  would turn back into that altitude, and ``Bmp390I2cSlave`` serves them from a
  register file that answers the real driver's real sequence.

Sample types
============

.. doxygenfile:: Hemerion/baro/baro_types.h

Unit conversion
===============

.. doxygenfile:: Hemerion/baro/baro_conversion.h

Wire protocol
=============

.. doxygenfile:: Hemerion/baro/baro_packet.h

Generic hardware simulator
==========================

Error model
-----------

.. doxygenfile:: Hemerion/baro/fmu/baro_noise_model.h

Packet emitter
--------------

.. doxygenfile:: Hemerion/baro/fmu/baro_packet_emitter.h

Bosch BMP390
============

Register map
------------

.. doxygenfile:: Hemerion/baro/bmp390/bmp390_registers.h

Compensation
------------

.. doxygenfile:: Hemerion/baro/bmp390/bmp390_compensation.h

I2C driver
----------

.. doxygenfile:: Hemerion/baro/bmp390/bmp390_driver.h

On-hardware bus adapter
-----------------------

.. doxygenfile:: Hemerion/baro/bmp390/bmp390_hal_i2c_bus.h

BMP390 hardware simulator
=========================

Measurement model
-----------------

.. doxygenfile:: Hemerion/baro/bmp390/fmu/bmp390_measurement_model.h

Simulated I2C part
------------------

.. doxygenfile:: Hemerion/baro/bmp390/fmu/bmp390_i2c_slave.h

Reference calibration
---------------------

.. doxygenfile:: Hemerion/baro/bmp390/fmu/bmp390_reference_calibration.h
