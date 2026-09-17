.. ------------------------------------------------------------------------------
.. Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _f16_autopilot_ecos_cosim:

F-16 Autopilot Closed Loop → Five Sensor FMUs → Flight Software Co-Simulation (``examples/f16_autopilot_ecos``)
===============================================================================================================

``examples/f16_autopilot_ecos`` closes the loop the trim flyouts leave open:
Aetherion's ``F16Plant.fmu`` and ``F16Autopilot.fmu`` co-simulated as a
100 Hz sampled control loop, flying NASA TM-2015-218675's four closed-loop
autopilot check-cases — while the five Hemerion sensor FMUs ride the same
truth with the same wiring as :ref:`f16_trim_ecos_cosim`, consumed by **the
same** ``f16_flight_computer`` executable, reused unchanged.

All four cases start from check-case 11's trim condition (Kitty Hawk,
10 013 ft, 335 KTAS, heading 45°) and step exactly one command:

.. list-table::
   :header-rows: 1
   :widths: 14 30 36 20

   * - ``--case``
     - check-case
     - command
     - window
   * - ``13.1`` (default)
     - subsonic altitude change
     - +100 ft (10 013 → 10 113 ft) at t = 5 s
     - 60 s
   * - ``13.2``
     - subsonic airspeed change
     - KEAS → 277 kt (from trim ≈ 288) at t = 5 s
     - 60 s
   * - ``13.3``
     - subsonic heading change
     - course 45 → 60° at t = 15 s
     - 240 s
   * - ``13.4``
     - subsonic lateral side-step
     - 2000 ft right of the courseline at t = 20 s
     - 240 s

Why these runs matter to the sensors: a straight-and-level flyout leaves
heading, tilt and IMU-bias states only weakly observable, which is why
case 11 is a *consistency* reference rather than a fusion test. The 13.x
maneuvers — 13.3 and 13.4 especially — excite exactly those states with
every stack in band throughout. These are the runs the EKF in ``modules/gnc``
will be exercised on.

The loop, and the delay that cannot be removed
----------------------------------------------

.. code-block:: text

   ┌──────────────────────┐  out.* (11 signals)   ┌──────────────────────┐
   │     F16Plant.fmu     ├──────────────────────>│   F16Autopilot.fmu   │   cmd.altCmd_ft, keasCmd_kt,
   │  (Aetherion 6-DoF    │                       │  (DML LQR SAS +      │<─ baseChiCmd_deg, latOffset_ft
   │   plant + trim)      │<──────────────────────┤   autopilot)         │   — the host owns the schedule
   │                      │  ctrl.* (4 surfaces)  └──────────────────────┘
   │                      ├──> the five sensor FMUs, wired as in f16_trim_ecos ──> f16_flight_computer
   └──────────────────────┘

Aetherion's standalone 13.x examples evaluate their DML LQR as a
**zero-order-hold discrete controller at the integration step rate** —
feedback read from the *current* state, surfaces applied over that same
step, no transport delay — at a recommended 0.02 s step; their headers warn
that dt = 0.1 diverges ("LQR plant is stiff").

An Ecos co-simulation cannot reproduce zero delay. Ecos is a Jacobi master:
every instance steps, *then* connections transfer, so the plant flies
[t, t+h] on surfaces the autopilot computed from the state at t−h — one
communication step late, whatever the stepping order. What the host can
choose is h. A one-step delay at h costs about the loop phase of zero-delay
ZOH at 2h, so **h = 0.01 s puts the co-simulation at the standalone's own
recommended cadence**. The control law itself is stateless — a pure LQR gain
evaluation, no integrators — so cadence and delay are the *only* differences
between the two drivers of the same DML. The measured responses below land
on the published solutions.

Two-rate sensors
----------------

The sensors do not follow the loop down to 0.01 s. Ecos' fixed-step
algorithm takes a per-instance step-size hint, stepping that instance every
Nth base step with dt = N × base:

.. list-table::
   :header-rows: 1
   :widths: 30 20 50

   * - instance
     - hint
     - effective rate
   * - plant, autopilot
     - — (base)
     - 100 Hz loop
   * - GPS
     - 0.1 s
     - one NAV-PVT per step of its own = **10 Hz**, as in the trim example —
       the GPS FMU needed no change
   * - radar altimeter
     - 1/25 s
     - 25 Hz — at base rate its ``max(1, lround(dt·rate))`` framing would
       clamp to 100 Hz
   * - IMU
     - — (base)
     - exactly one frame per base step = 100 Hz
   * - BMP390, MMC5983MA
     - — (base)
     - whatever ODR the flight computer programs; stepping rate is irrelevant

Two host-computed inputs
------------------------

As in the trim example, the magnetometer's body-frame field is computed by
the host after every step (a connection modifier sees one source variable;
the field needs four). This example adds a second host-computed input:
**``cmd.latOffset_ft`` is feedback, not a setpoint**. The standalone
computes the aircraft's lateral deviation from the original courseline every
step (flat earth about the initial position, R = 6 371 000 m) and feeds the
controller ``deviation − commanded_step``; the host does the same. It is the
one signal in the loop the plant cannot publish, because it depends on where
the flight *began*. Cases 13.1–13.3 write a constant 0.

**Start-up**: the plant's ``ctrl.*`` inputs start at 0, the plant does not
publish its trim deflections, and Ecos' init rounds hand the plant whatever
the autopilot computed during its own initialisation. The ``trimPoint``
parameter set therefore seeds the autopilot's ``fb.*``/``cmd.*`` inputs with
the trim condition, so its init-time output is the LQR's own answer at trim.
The initial KEAS command is then refined after init from the plant's actual
density and airspeed — printed as ``holding trim KEAS 287.981 kt``, against
the references' 287.92–287.98.

Measured responses
------------------

Unpaced, default configuration:

.. list-table::
   :header-rows: 1
   :widths: 10 22 34 34

   * - case
     - commanded
     - response at cut-off
     - the published participants
   * - 13.1
     - 10 113 ft
     - **10 113.5 ft**, ≈3 ft overshoot at t = 10 s, settled by t = 20 s
     - end within 0.6 ft of the command
   * - 13.2
     - 277 kt
     - **277.0 kt**, altitude dip to 10 006.7 ft
     - sim_02 reaches 277.0 kt and dips to 10 006.4 ft; sim_04/05 are still
       ≈283 kt when their windows end
   * - 13.3
     - course 60°
     - **60.04°** at cut-off (55.1° five seconds into the turn, 59.9° by
       t = 30 s), altitude held to 10 013.5 ft — and 240 s of integrated
       ground track lands **1.5 ft** from ``sim_05``'s lateral deviation
       (32 071.6 vs 32 070.1 ft)
     - participants end 59.92–60.01°
   * - 13.4
     - 2000 ft right
     - **1981.5 ft at t = 60 s**, **2000.1 ft at cut-off**, course back to
       45.07°, altitude held
     - participants read 1934–1992 ft at t = 60 s

Verifying
---------

``verify_trajectory.py`` makes two kinds of check. The NESC-reference
envelope works as in :ref:`f16_trim_ecos_cosim` — participants' own
disagreement as the slack, unequal windows handled by per-sample coverage.
New here is **the commanded response**, which the envelope alone cannot
carry: the participants' altitude spread plus a sensible floor is wider than
13.1's 100 ft step, so a run that never climbed could still sit inside the
envelope. Each case asserts its own response with bounds taken from what the
participants themselves achieve — 13.2's band accepts 275–285.5 kt because
"reached 277 exactly" would fail two of the three published solutions, and
13.4 is judged at t = 60 s, past which the flat-earth deviation formula
itself drifts (``sim_05`` reads 1817 ft at 239.9 s). The hold quantities are
asserted too: an altitude case must hold heading and KEAS.

.. code-block:: console

   $ python verify_trajectory.py --truth results/f16_truth.csv --case 13.1 \
       --reference <aetherion>/data/Atmos_13p1_SubsonicAltitudeChangeF16/Atmos_13p1_sim_02.csv \
       --reference <aetherion>/data/Atmos_13p1_SubsonicAltitudeChangeF16/Atmos_13p1_sim_04.csv \
       --reference <aetherion>/data/Atmos_13p1_SubsonicAltitudeChangeF16/Atmos_13p1_sim_05.csv

Running and building
--------------------

Two terminals, from ``build/examples-native/examples/``:

.. code-block:: console

   $ ./f16_trim_ecos/f16_flight_computer            # terminal 1 — the "STM32" side, reused
   $ ./f16_autopilot_ecos/f16_autopilot_cosim --case 13.3   # terminal 2 — the closed loop

The trim example's pacing caveat applies unchanged to the polled I2C parts
(``--rtf 1`` if their sample counts matter).

Building needs the ``examples-native`` preset and an **Aetherion ≥ 0.14.1**
install for both ``F16Plant.fmu`` and ``F16Autopilot.fmu`` (``AETHERION_ROOT``
if not in a default location; ``--f16`` / ``--ap`` override at runtime).
Configure reads the ports this example binds out of both archives rather
than trusting a version string. The 0.14.1 autopilot FMU exposes **no
parameters** — ``circlePoleSW`` is baked off, which is why NESC cases 15 and
16 (the circumnavigations) are not rows in the ``--case`` table.
