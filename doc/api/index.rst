.. ------------------------------------------------------------------------------
.. Project: Hemerion
.. Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _api-reference:

=============
API reference
=============

These pages are generated from the documentation comments in the source
headers: Doxygen parses the headers into XML and Breathe renders that XML
into the pages below, so a declaration and its description here are the ones
in the tree at the commit this site was built from. Nothing on these pages is
maintained separately from the code.

What is covered
===============

The reference documents *headers*, one page per module directory. Everything
under a module's ``include/`` is published; the ``src/`` implementations are
not, and neither are the tests or the private members of a published class.
That boundary is the intended one: the headers are what another module, an
application, or a co-simulation consumer is allowed to depend on.

Four trees contribute:

.. list-table::
   :header-rows: 1
   :widths: 18 22 60

   * - Tree
     - Namespace root
     - What lives there
   * - ``modules/``
     - ``hemerion::``
     - Reusable firmware libraries — sensors, RTOS scaffolding, fault
       handling, power, CAN framing.
   * - ``bsp/``
     - *(none — C)*
     - The board's hardware abstraction layer, as a flat ``hal_*`` C API.
   * - ``sim/``
     - ``hemerion::sim::``
     - Host-only transports that let firmware code run against simulated
       peripherals.
   * - ``apps/``
     - *(none)*
     - Application entry points. Not part of the API surface, so they are
       described in :doc:`../led_blink_tutorial` rather than here.

Modules listed as **empty** in ``modules/README.md`` (``actuators/``,
``gnc/``, ``datalogger/``) have no headers yet and therefore no page here.
Their absence from this reference is the accurate signal, not an omission.

.. _api-layering:

How the code is layered
=======================

The single most important thing to know when reading a Hemerion header is
which of three tiers it belongs to, because the tiers have different rules
and the same directory can hold two of them.

**On-target firmware.** Cross-compiled to the STM32H743. No dynamic
allocation, no exceptions, no RTTI, no ``<random>``, fixed-size buffers
throughout. Failures are reported through returned ``enum class`` error
codes, never thrown. Everything in ``modules/*/include`` outside an ``fmu/``
subdirectory, plus all of ``bsp/``, is in this tier.

**Host-only simulation.** Compiled natively, only when
``HEMERION_BUILD_FMU`` is on, and never cross-compiled. This is the code that
turns a truth trajectory into the register counts a real part would have
produced — noise models, measurement models, and the simulated silicon that
answers a bus. It allocates, seeds ``<random>``, and opens sockets freely.
Everything under an ``fmu/`` subdirectory, and all of ``sim/``, is in this
tier.

**Both.** A handful of types are deliberately the same object code in either
build: the register maps, the wire-protocol constants and parsers, and the
sensor drivers. A driver achieves this by taking its bus as an injected
interface — ``Bmp390I2cBus``, ``Mmc5983maI2cBus``, ``ImuSpiBus`` — so one
driver reaches its part through whichever transport the build has, and the
register sequence is exercised unchanged in every one of them. That is why
these interfaces are pure virtual rather than template parameters: one
virtual call per bus transaction is noise against tens of microseconds of bus
time, while templating the driver on its transport would put the register
sequence in a header and recompile it per build.

The BMP390 is the fullest example. One ``Bmp390Driver`` reaches four
different things through four implementations of the same interface — real
silicon, a simulated part with no transport at all, the same part over a
shared-memory bus, and the co-simulation:

.. graphviz::
   :align: center
   :caption: One driver, four transports. Only the bottom row differs between
             builds; everything above the interface is identical object code.
   :alt: Bmp390Driver calls the abstract Bmp390I2cBus interface, which has four
         implementations: Bmp390HalI2cBus reaching BMP390 silicon over STM32
         I2C1, DirectBus reaching the simulated part with no transport, ShmBus
         reaching it over the shared-memory I2C bus, and ShmI2cBus reaching the
         BMP390 simulator FMU in the rocket_gps_ecos co-simulation.

   digraph injected_bus {
       bgcolor="transparent"
       rankdir=TB
       node [shape=box style="rounded,filled" fillcolor="#eef3f8" color="#41597a"
             fontname="Helvetica" fontsize=10 margin="0.16,0.09"]
       edge [color="#41597a" fontname="Helvetica" fontsize=9]

       driver [label="Bmp390Driver\nidentify, reset, read NVM,\nconfigure, poll, burst read"
               fillcolor="#d6e4f2"]
       iface  [label="Bmp390I2cBus\npure virtual\none call = one transaction"
               style="rounded,filled,dashed" fillcolor="#ffffff"]

       hal    [label="Bmp390HalI2cBus\nmodules/sensors"]
       direct [label="DirectBus\nunit test"]
       shmt   [label="ShmBus\nsim/i2c_shm test"]
       shme   [label="ShmI2cBus\nrocket_gps_ecos"]

       silicon [label="BMP390 silicon\nover STM32 I2C1" shape=box style=filled fillcolor="#e6e6e6"]
       slave1  [label="Bmp390I2cSlave\nno transport" shape=box style=filled fillcolor="#e6e6e6"]
       slave2  [label="Bmp390I2cSlave\nover shared memory" shape=box style=filled fillcolor="#e6e6e6"]
       fmu     [label="BMP390 simulator FMU\nover shared memory" shape=box style=filled fillcolor="#e6e6e6"]

       driver -> iface [label="  calls"]
       hal    -> iface [arrowhead=empty style=dashed]
       direct -> iface [arrowhead=empty style=dashed]
       shmt   -> iface [arrowhead=empty style=dashed]
       shme   -> iface [arrowhead=empty style=dashed]

       hal    -> silicon
       direct -> slave1
       shmt   -> slave2
       shme   -> fmu

       { rank=same; hal; direct; shmt; shme }
       { rank=same; silicon; slave1; slave2; fmu }
   }

A Renode adapter would be a fifth, wrapping the emulated I2C peripheral; it
is described in ``bmp390_driver.h`` as the shape such a thing would take, and
is not built.

Conventions
===========

The same handful of conventions hold across every module, so they are stated
once here rather than repeated on each page.

.. list-table::
   :header-rows: 1
   :widths: 32 68

   * - Convention
     - Meaning
   * - ``kSomeName``
     - An ``inline constexpr`` compile-time constant. Register addresses,
       capacities, sync bytes and scale factors all take this form.
   * - ``enum class XError : std::uint8_t``
     - How a fallible operation reports failure. ``kNone`` is always the
       success value; there are no exceptions and no ``errno``.
   * - ``[[nodiscard]]``
     - Marks the accessors and fallible calls whose result is the only thing
       the call produces. Present on essentially every non-``void`` member.
   * - ``XRawSample`` / ``XSample``
     - Register counts as read off the part, and the same quantity in SI
       units. ``convert_raw_to_si()`` maps the first to the second given an
       ``XScale``.
   * - ``PascalCase`` / ``snake_case``
     - Types and enumerators use the former, functions and variables the
       latter. Trailing underscores mark private data members.

Units are SI throughout — metres, seconds, pascals, radians per second,
metres per second squared — with two documented exceptions where the
underlying domain does not use SI: magnetic field is carried in microtesla,
and geodetic position in degrees.

Reference
=========

.. toctree::
   :maxdepth: 2

   sensors
   rtos_core
   fault
   power
   comms
   bsp_hal
   sim

Indices
=======

* :ref:`genindex`
* :ref:`search`
