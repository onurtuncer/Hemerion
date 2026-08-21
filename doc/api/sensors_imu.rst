.. ------------------------------------------------------------------------------
.. Project: Hemerion
.. Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _api-sensors-imu:

===========================
Inertial measurement unit
===========================

``hemerion::sensors::imu`` carries specific force and angular rate. The driver
here talks SPI rather than I2C — an IMU is sampled fast enough that the bus
choice is not incidental — and its part is described by ``imu_spi_protocol.h``
rather than by a vendor part number, because the register map is Hemerion's own
model of a generic FIFO-backed IMU rather than a transcription of one
datasheet.

The FIFO is the reason the driver is more than a burst read: the part
accumulates samples between polls, and the driver has to drain them and notice
overflow. ``ImuSpiSlave`` on the simulator side implements that same FIFO, so
an overflow is reproducible in co-simulation rather than only on a loaded
board.

Sample types
============

.. doxygenfile:: Hemerion/imu/imu_types.h

Unit conversion
===============

.. doxygenfile:: Hemerion/imu/imu_conversion.h

Wire protocol
=============

.. doxygenfile:: Hemerion/imu/imu_packet.h

Register map
============

.. doxygenfile:: Hemerion/imu/imu_spi_protocol.h

SPI driver
==========

.. doxygenfile:: Hemerion/imu/imu_spi_driver.h

Hardware simulator
==================

Error model
-----------

.. doxygenfile:: Hemerion/imu/fmu/imu_noise_model.h

Simulated SPI part
------------------

.. doxygenfile:: Hemerion/imu/fmu/imu_spi_slave.h

Packet emitter
--------------

.. doxygenfile:: Hemerion/imu/fmu/imu_packet_emitter.h
