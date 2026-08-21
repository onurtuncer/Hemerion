.. ------------------------------------------------------------------------------
.. Project: Hemerion
.. Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _api-comms:

==============
Communications
==============

``hemerion::comms`` currently covers CAN framing only: a validated frame
representation and a fixed-capacity queue to stage frames on either side of a
peripheral. As ``modules/README.md`` records, this module is **partial** — the
EtherCAT slave stack and MAVLink codec named in the architecture are not
built, and EtherCAT is not being pursued at present.

Like the rest of ``modules/``, this is a pure framing layer with no dependency
on a CAN peripheral or on the HAL, so a frame can be built and validated in a
native test with no bus in the picture. The FIFO wraps ``etl::queue``
specifically so that callers never meet ETL's overflow assertions: a full
queue is a returned error here, not a trap.

CAN frames
==========

.. doxygenfile:: hemerion/comms/can_frame.h

CAN FIFO
========

.. doxygenfile:: hemerion/comms/can_fifo.h
