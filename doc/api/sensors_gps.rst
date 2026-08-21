.. ------------------------------------------------------------------------------
.. Project: Hemerion
.. Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _api-sensors-gps:

===
GPS
===

``hemerion::sensors::gps`` is the one sensor family that does not follow the
shape described in :ref:`api-sensors`, and for a good reason: a GNSS receiver
does not present a register map. It presents a byte stream in a standardised
wire protocol, and the receiver — not the host — has already solved for
position. There is no raw-count-to-SI conversion step here because there are no
raw counts; the parsers produce a ``GpsFix`` directly.

Two protocols are decoded. NMEA 0183 is text, ubiquitous, and lossy about
precision; u-blox UBX is binary, compact, and carries a complete fix in a
single ``UBX-NAV-PVT`` message. A receiver is configured to emit one or the
other, so ``GpsDriver`` takes the protocol at construction and never
auto-detects.

Geodetic position is carried in **degrees**, the second documented departure
from SI in this API, matching both wire protocols.

The dynamics model has no counterpart in the other sensor families. Real GNSS
receivers refuse to output a fix outside an export-control envelope (the CoCom
limits) and degrade under high acceleration; a simulator that ignored this
would give a flight-dynamics model a fix where the real hardware would give
none. Modelling the envelope is what makes a loss-of-fix event testable.

.. seealso::

   :doc:`../rocket_gps_ecos_cosim` drives this stack end to end against a
   rocket trajectory.

Fix types
=========

.. doxygenfile:: Hemerion/gps/gpsTypes.hpp

Driver
======

.. doxygenfile:: Hemerion/gps/gpsDriver.hpp

NMEA 0183 parser
================

.. doxygenfile:: Hemerion/gps/nmeaParser.hpp

u-blox UBX parser
=================

.. doxygenfile:: Hemerion/gps/ubxParser.hpp

Hardware simulator
==================

Error model
-----------

.. doxygenfile:: Hemerion/gps/fmu/gpsNoiseModel.hpp

Dynamics envelope
-----------------

.. doxygenfile:: Hemerion/gps/fmu/gpsDynamicsModel.hpp

UBX emitter
-----------

.. doxygenfile:: Hemerion/gps/fmu/ubxEmitter.hpp

UDP sender
----------

.. doxygenfile:: Hemerion/gps/fmu/udpSender.hpp
