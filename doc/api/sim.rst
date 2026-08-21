.. ------------------------------------------------------------------------------
.. Project: Hemerion
.. Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _api-sim:

========================
Co-simulation transports
========================

``hemerion::sim`` is host-only. None of it is cross-compiled, and none of it
appears in a firmware image. It exists so that firmware code can be run against
something other than real silicon: a plant model in another process, or a
simulated sensor part answering a simulated bus.

There are two distinct jobs here, and they do not share a mechanism.

**Step bridges** (``shm_bridge``, ``udp_bridge``) carry one lockstep simulation
step between an FMI master and a plant process — inputs one way, outputs the
other, with the master driving the clock. The two are the same protocol over
different transports: shared memory when both processes are on one host, UDP
datagrams when they are not.

**Bus links** (``i2c_shm``, ``spi_shm``) carry one bus transaction between a
controller and a peripheral, at the granularity a HAL call has on the target.
They are how a hardware-simulator FMU gets to *be* a part on a bus rather than
merely publish a signal, which is what lets the on-target driver run its real
register sequence in co-simulation.

The two operate at different rates and answer to different clocks, which is
the reason they are separate mechanisms rather than one:

.. graphviz::
   :align: center
   :caption: Step bridges carry one simulation step, driven by the master's
             clock. Bus links carry one bus transaction, driven by whenever
             the firmware asks — many of them within a single step.
   :alt: A step bridge connects the FMI master to a plant process over either
         a shared-memory segment or a UDP datagram pair, carrying inputs one
         way and outputs the other, once per simulation step. Separately, a
         bus link connects firmware's driver to a simulated sensor part over
         a shared-memory I2C or SPI region, carrying one register transaction
         at a time, many per step.

   digraph sim_transports {
       bgcolor="transparent"
       rankdir=LR
       node [shape=box style="rounded,filled" fillcolor="#eef3f8" color="#41597a"
             fontname="Helvetica" fontsize=10 margin="0.16,0.09"]
       edge [color="#41597a" fontname="Helvetica" fontsize=9]

       subgraph cluster_step {
           label="step bridge — once per simulation step, master's clock"
           fontname="Helvetica"
           fontsize=10
           color="#9aa7b5"
           style=dashed
           master [label="FMI master"]
           sbr    [label="shm_bridge\nor udp_bridge" fillcolor="#e6e6e6"]
           plant  [label="plant process\n(Aetherion)"]
           master -> sbr [label="  inputs"]
           sbr -> plant
           plant -> sbr [label="  outputs"]
           sbr -> master
       }

       subgraph cluster_bus {
           label="bus link — once per transaction, firmware's clock"
           fontname="Helvetica"
           fontsize=10
           color="#9aa7b5"
           style=dashed
           drv  [label="driver\n(controller side)" fillcolor="#d6e4f2"]
           bl   [label="i2c_shm\nor spi_shm" fillcolor="#e6e6e6"]
           part [label="simulated part\n(peripheral side)"]
           drv -> bl [label="  address + write"]
           bl -> part
           part -> bl [label="  read bytes"]
           bl -> drv
       }
   }

Both bus links run their peripheral side on a service thread rather than only
when the FMU steps, and for the same reason: real silicon answers its address
whenever the controller starts a transaction, not when the part's physics model
happens to be integrating.

All four regions are standard-layout blocks mapped or sent between two separate
binaries, so each carries a ``kProtocolVersion`` that the attaching side checks.
Bump it whenever a field's order, type or size changes.

.. seealso::

   :doc:`../rocket_gps_ecos_cosim` walks an end-to-end co-simulation built on
   these transports, and :doc:`../sensor_models` covers the sensor models that
   sit on top of them.

Shared-memory step bridge
=========================

Wire format
-----------

.. doxygenfile:: hemerion/sim/shm_bridge/bridge_protocol.h

Segment
-------

.. doxygenfile:: hemerion/sim/shm_bridge/shm_segment.h

Handshake
---------

.. doxygenfile:: hemerion/sim/shm_bridge/shm_bridge.h

UDP step bridge
===============

Wire format
-----------

.. doxygenfile:: hemerion/sim/udp_bridge/bridge_protocol.h

Socket
------

.. doxygenfile:: hemerion/sim/udp_bridge/udp_socket.h

Handshake
---------

.. doxygenfile:: hemerion/sim/udp_bridge/udp_bridge.h

Simulated I2C bus
=================

Wire format
-----------

.. doxygenfile:: hemerion/sim/i2c_shm/i2c_shm_protocol.h

Endpoints
---------

.. doxygenfile:: hemerion/sim/i2c_shm/i2c_shm_link.h

Peripheral endpoint
-------------------

.. doxygenfile:: hemerion/sim/i2c_shm/i2c_peripheral_endpoint.h

Simulated SPI bus
=================

Wire format
-----------

.. doxygenfile:: hemerion/sim/spi_shm/spi_shm_protocol.h

Endpoints
---------

.. doxygenfile:: hemerion/sim/spi_shm/spi_shm_link.h

Peripheral endpoint
-------------------

.. doxygenfile:: hemerion/sim/spi_shm/spi_peripheral_endpoint.h
