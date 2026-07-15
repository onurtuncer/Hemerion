.. ------------------------------------------------------------------------------
.. Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _rocket_gps_ecos_cosim:

Rocket → GPS → Flight Software Co-Simulation (``examples/rocket_gps_ecos``)
===========================================================================

``examples/rocket_gps_ecos`` couples three independently developed pieces into
one sensor-in-the-loop scenario, orchestrated by the
`Ecos <https://github.com/Ecos-platform/ecos>`_ FMI co-simulation platform:

1. **Aetherion's two-stage rocket plant** (``TwoStageRocket.fmu``) supplies the
   *truth* trajectory — a 6-DoF rocket (NASA TM-2015-218675 Scenario 17:
   DAVE-ML aero/propulsion/inertia tables, J2 gravity, stage separation)
   integrated with a Radau IIA RKMK scheme on SE(3).
2. **Hemerion's GPS hardware-simulator FMU** (``hemerion_gps_fmu.fmu``, from
   ``modules/sensors``) turns that truth into what a real u-blox M9N would
   report: Gaussian position/velocity noise plus the receiver's self-reported
   accuracies, encoded as **wire-exact UBX-NAV-PVT frames** sent over UDP —
   one frame per co-simulation step.
3. **The flight software GPS stack** (``gps_flight_computer``) decodes that
   byte stream with the *unmodified* ``GpsDriver``/``UbxParser`` from
   ``modules/sensors`` — the same code the STM32H743 firmware cross-compiles.
   Only the transport differs from the target: a UDP socket on the host, a
   UART RX line (or Renode's emulated UART, see :ref:`swil_windows_setup`) on
   hardware.

.. code-block:: text

   ┌──────────────────────┐  FMI 2.0 variables    ┌──────────────────────┐  UBX-NAV-PVT    ┌────────────────────────┐
   │  TwoStageRocket.fmu  │  (Ecos connections)   │ hemerion_gps_fmu.fmu │  over UDP       │  gps_flight_computer   │
   │  Aetherion 6-DoF     ├──────────────────────>│ u-blox M9N hardware  ├────────────────>│  GpsDriver + UbxParser │
   │  rocket plant        │  lat, lon, alt,       │ simulator: noise +   │ 127.0.0.1:5762  │  (the code the STM32   │
   │  (truth)             │  NED velocity         │ UBX encoder          │ 1 frame / step  │  H743 firmware runs)   │
   └──────────────────────┘                       └──────────────────────┘                 └────────────────────────┘
            └───────────── rocket_gps_cosim: Ecos master, fixed step 0.1 s (10 Hz GPS) ─────────────┘

.. contents:: On this page
   :local:
   :depth: 2

---

Signal wiring
-------------

``rocket_gps_cosim`` builds the coupling with Ecos' C++ API
(``simulation_structure``). The rocket reports geodetic position in
**radians**; the GPS FMU takes **degrees**, so the two conversions ride on the
Ecos connections as modifiers. Velocity is wired 1:1 into the GPS FMU's NED
inputs, from which it derives speed-over-ground and course itself:

.. list-table::
   :header-rows: 1
   :widths: 34 32 34

   * - ``TwoStageRocket`` output
     - Ecos connection modifier
     - ``hemerion_gps_fmu`` input
   * - ``out.lat_rad``
     - rad → deg
     - ``latitude_deg``
   * - ``out.lon_rad``
     - rad → deg
     - ``longitude_deg``
   * - ``out.alt_m``
     - —
     - ``altitude_m``
   * - ``out.v_north_m_s``
     - —
     - ``v_north_mps``
   * - ``out.v_east_m_s``
     - —
     - ``v_east_mps``
   * - ``out.v_down_m_s``
     - —
     - ``v_down_mps``

In code:

.. code-block:: cpp

    ecos::simulation_structure ss;
    ss.add_model("rocket", options.rocket_fmu.string());
    ss.add_model("gps", options.gps_fmu.string());

    const std::function<double(const double&)> rad2deg = [](const double& rad) {
      return rad * (180.0 / std::numbers::pi);
    };
    ss.make_connection<double>("rocket::out.lat_rad", "gps::latitude_deg", rad2deg);
    ss.make_connection<double>("rocket::out.lon_rad", "gps::longitude_deg", rad2deg);
    ss.make_connection<double>("rocket::out.alt_m", "gps::altitude_m");
    ss.make_connection<double>("rocket::out.v_north_m_s", "gps::v_north_mps");
    ss.make_connection<double>("rocket::out.v_east_m_s", "gps::v_east_mps");
    ss.make_connection<double>("rocket::out.v_down_m_s", "gps::v_down_mps");

    const auto sim = ss.load(std::make_unique<ecos::fixed_step_algorithm>(0.1));
    sim->init("launchSite");

The GPS FMU intentionally has **no FMI output variables**: its output is the
UBX byte stream, exactly like a real receiver's UART. That keeps the firmware
parser exercised at the byte level — sync characters, little-endian scaled
integers, checksums and all — rather than handing the flight software
convenient floating-point lat/lon it would never see on the vehicle.

Building
--------

Requires a native toolchain, network access at configure time (Ecos and its
FMU loader fmi4c are fetched and built from source), and an Aetherion install
for ``TwoStageRocket.fmu`` (set ``AETHERION_ROOT`` if it is not in a default
location; without it the example still builds and ``--rocket <path>`` selects
the FMU at runtime):

.. code-block:: console

   $ cmake --preset examples-native
   $ cmake --build build/examples-native

Besides the two example executables this also produces
``hemerion_gps_fmu.fmu`` — the ``hemerion_gps_fmu_package`` target zips the
GPS simulator into a proper FMI 2.0 archive (``modelDescription.xml`` at the
archive root, the shared library under ``binaries/win64/`` or
``binaries/linux64/``).

Running
-------

Two terminals, both in ``build/examples-native/examples/rocket_gps_ecos/``.
The flight computer starts first, since it binds the GPS FMU's default UDP
destination (port 5762):

.. code-block:: console

   $ ./gps_flight_computer          # terminal 1 — the "STM32" side
   $ ./rocket_gps_cosim             # terminal 2 — the Ecos master

The default flight lasts 240 s at a 0.1 s communication step — a 10 Hz GPS
navigation rate. Staging occurs at t ≈ 37 s, apogee (≈ 780 km) at t ≈ 232 s;
the Scenario 17 plant holds its state constant after apogee, so longer runs
only add a flat tail. ``--rtf 1`` paces the master to wall-clock speed to
watch the fix stream arrive live; unpaced, the run takes about 2.5 minutes.

Ecos master console
~~~~~~~~~~~~~~~~~~~

.. code-block:: text

   [cosim] rocket: C:/Program Files/Aetherion/share/Aetherion/fmu/TwoStageRocket.fmu
   [cosim] gps:    D:/Dev/Hemerion/build/examples-native/modules/sensors/include/Hemerion/gps/fmu/hemerion_gps_fmu.fmu
   [cosim] step 0.1 s (10 Hz GPS), stop 240 s
   [cosim] t=10 s  alt=1828.52 m  mach=1.52854  mass=265846 kg
   [cosim] t=20 s  alt=7244.12 m  mach=3.63514  mass=217693 kg
   [cosim] t=30 s  alt=16616.3 m  mach=6.5223  mass=169539 kg
   [cosim] t=37.4 s  stage 1 separated
   [cosim] t=40 s  alt=30913.4 m  mach=9.31213  mass=95506.8 kg
   [cosim] t=50 s  alt=48441.5 m  mach=10.0596  mass=82433.4 kg
   ...
   [cosim] t=230 s  alt=770855 m  mach=29.3356  mass=18897 kg
   [cosim] t=240 s  alt=780141 m  mach=29.3034  mass=18897 kg
   [cosim] done: 2401 steps, 2401 UBX-NAV-PVT frames emitted
   [cosim] apogee 780141 m at t=232.1 s, staging at t=37.4 s
   [cosim] rocket truth written to results/rocket_truth.csv

Flight computer console
~~~~~~~~~~~~~~~~~~~~~~~

Every 50th decoded fix is printed; note the fix data is what the *receiver*
reports — truth plus the injected noise — decoded from raw UBX bytes:

.. code-block:: text

   [fc] listening for UBX-NAV-PVT on UDP port 5762 (GpsDriver, protocol=UBX)
   [fc] fix     1  t=    0.1 s  lat= 37.8338021  lon= -75.4879018  alt=      1.7 m  vel=    0.0 m/s  crs=  0.1 deg  sats=11
   [fc] fix    50  t=    5.0 s  lat= 37.8337980  lon= -75.4836010  alt=    431.8 m  vel=  156.8 m/s  crs= 89.4 deg  sats=11
   [fc] fix   100  t=   10.0 s  lat= 37.8338208  lon= -75.4694585  alt=   1787.9 m  vel=  351.2 m/s  crs= 89.3 deg  sats=11
   [fc] fix   350  t=   35.0 s  lat= 37.8329034  lon= -75.1705319  alt=  22952.6 m  vel= 1944.9 m/s  crs= 89.4 deg  sats=11
   [fc] fix   400  t=   40.0 s  lat= 37.8322089  lon= -75.0490460  alt=  30750.4 m  vel= 2284.8 m/s  crs= 91.7 deg  sats=11
   ...
   [fc] fix  2400  t=  240.0 s  lat= 36.9722100  lon= -62.4137047  alt= 780139.9 m  vel= 6762.7 m/s  crs= 99.4 deg  sats=11
   [fc] UBX stream quiet for 3000 ms -- co-simulation finished
   [fc] summary: 2401 fixes decoded, 0 checksum errors
   [fc] max altitude 780148 m, max ground speed 7444.85 m/s
   [fc] fixes written to results/gps_fixes.csv

**2401 frames sent, 2401 fixes decoded, zero checksum errors** — the whole
chain from 6-DoF truth through the noise model, the UBX encoder, UDP, and the
on-target parser is lossless and wire-correct.

Results
-------

``plot_results.py`` (matplotlib) renders the two CSVs — Ecos' truth log and
the flight computer's fix log — into the figures below:

.. code-block:: console

   $ python plot_results.py       # reads results/, writes plots/

.. figure:: _static/rocket_gps_ecos/altitude_vs_time.png
   :width: 100%
   :alt: Altitude vs time: truth line with decoded GPS fixes overlaid, staging marker at 37.4 s, apogee 780.1 km

   Truth altitude and the fixes the flight software decoded. At this scale the
   two are indistinguishable — which is the point.

.. figure:: _static/rocket_gps_ecos/ground_track.png
   :width: 100%
   :alt: Ground track from Wallops heading east over the Atlantic, truth and GPS fixes overlaid

   Ground track: launch from Wallops Flight Facility, firing due east over the
   Atlantic. The southward curl is the J2/rotating-Earth effect on an eastward
   suborbital arc.

.. figure:: _static/rocket_gps_ecos/velocity_vs_time.png
   :width: 100%
   :alt: Speed over ground vs time, truth and UBX-reported gSpeed, staging marker

   Speed over ground: truth vs. the NAV-PVT ``gSpeed`` field the parser
   recovered. The slope change at staging and burnout (t ≈ 100 s) is visible.

.. figure:: _static/rocket_gps_ecos/gps_error.png
   :width: 100%
   :alt: Horizontal and vertical decoded-fix error vs truth, flat noise band around the 1.5 m one-sigma line

   Decoded-fix error against truth. The band is flat across three decades of
   altitude and speed and matches the configured receiver noise
   (``GpsNoiseConfig``: 1.5 m horizontal / 3 m vertical, 1-sigma) — the error
   the flight software sees is *exactly* the error that was injected, with no
   distortion added by the encode/transmit/parse chain.

One detail matters when comparing fixes against truth: the Ecos master
propagates connections *between* steps, so the fix emitted at the end of step
*k* carries the truth sampled at the end of step *k−1* — one communication
step (0.1 s) of transport delay, just like a real receiver's navigation-epoch
latency. At 7 km/s that step is ~700 m of along-track motion, so the error
plot aligns each fix with the truth one step earlier; naive equal-timestamp
alignment would show latency, not receiver error.

Implementation notes
--------------------

Ecos and fmi4c compatibility
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Two properties of the FMU loader Ecos uses (`fmi4c
<https://github.com/Ecos-platform/fmi4c>`_) shaped the GPS FMU:

* fmi4c resolves the **complete FMI 2.0 export table** up front and refuses to
  load a binary that omits any function. The GPS FMU therefore exports stub
  implementations (returning ``fmi2Error``) of the state-management and
  derivative functions its capability flags disable
  (``fmi2GetFMUstate`` … ``fmi2GetRealOutputDerivatives``).
* fmi4c's XML parser (ezxml) rejects ``--`` sequences inside XML comments —
  they are in fact illegal per the XML specification — which constrains the
  comment style used in ``model_description.xml``.

Both are good conformance pressure: any strict FMI master would be within its
rights on either count.

Ecos on Windows, cross-drive paths
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``simulation_structure::add_model`` has a ``std::filesystem::path`` overload
that internally converts the path with ``std::filesystem::relative()``. On
Windows that yields an *empty* path when the FMU lives on a different drive
than the working directory (e.g. FMU on ``C:``, build tree on ``D:``), so the
example passes plain absolute-path *strings*, which Ecos' file resolver
handles as-is.

Timing model
~~~~~~~~~~~~

One NAV-PVT frame is emitted per communication step, so the flight computer
maps fix index → simulation time when logging (``--fix-period``, default
0.1 s). When the master runs faster than real time, wall-clock arrival times
are meaningless; the index mapping is exact as long as no datagram is dropped,
and the summary line makes loss visible (decoded-fix count vs. the master's
step count). ``--rtf 1`` runs the co-simulation paced to the wall clock
instead, which makes the stream realistic enough to demo live consumers.

Next: an IMU on the same truth bus
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The rocket FMU already exposes body rates (``out.p_rad_s`` …), attitude, and
enough state to derive specific force. A future ``imu.fmu`` will sit next to
the GPS FMU on the same Ecos truth bus and feed the flight computer a second
sensor stream through ``modules/sensors``' IMU conversion path — same
pattern: truth in over FMI, sensor-realistic bytes out the side channel.
