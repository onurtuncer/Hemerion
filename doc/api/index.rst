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
interface — ``Bmp390I2cBus``, ``Mmc5983maI2cBus``, ``ImuSpiBus`` — so the
identical driver runs over the board's HAL on hardware, over ``sim/i2c_shm``
or ``sim/spi_shm`` in co-simulation, and over an emulated peripheral under
Renode. That the register sequence is exercised unchanged in all three is the
point of the arrangement, and it is why those interfaces are pure virtual
rather than template parameters.

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
