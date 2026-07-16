.. ------------------------------------------------------------------------------
.. Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: GPL-3.0-only
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _sensor_models:

Sensor Models (``modules/sensors``)
===================================

``modules/sensors`` ships five *hardware-simulator FMUs* — GPS, IMU,
barometer, radar altimeter, and magnetometer — each packaged as an FMI 2.0
co-simulation slave. Every one follows the same pattern established by the
GPS FMU in :ref:`rocket_gps_ecos_cosim`:

* **Truth in over FMI.** A 6-DoF plant (Aetherion's ``TwoStageRocket.fmu``)
  drives the FMU's input variables with noiseless truth — position, body
  rates, altitude, magnetic field — through ordinary Ecos connections.
* **Sensor-realistic bytes out the side channel.** The FMU deliberately has
  **no FMI output variables**. Its output is a raw byte stream over UDP,
  exactly like a real part's UART or SPI drain: wire-exact frames carrying
  quantized register counts, checksums and all. The flight software decodes
  them with the *unmodified* on-target parser and conversion code.
* **The error model is the exact inverse of the on-target conversion.**
  Feeding the emitted counts back through the corresponding
  ``convert_raw_to_si()`` with the same scale recovers the noisy SI sample to
  quantization error, so a round-trip test pins the simulator and the flight
  code to each other.

.. code-block:: text

   ┌──────────────────┐  FMI 2.0 inputs   ┌─────────────────────────────┐  raw frames   ┌────────────────────┐
   │  6-DoF plant     │  (Ecos wiring)    │  hemerion_<sensor>_fmu.fmu  │  over UDP     │  flight software   │
   │  (truth)         ├──────────────────>│  noise model + quantizer    ├──────────────>│  on-target parser  │
   │                  │                   │  + packet emitter           │               │  + conversion      │
   └──────────────────┘                   └─────────────────────────────┘               └────────────────────┘

The model code lives in each sensor's ``fmu/`` subtree
(``modules/sensors/include/Hemerion/<sensor>/fmu/``), built only when
``HEMERION_BUILD_FMU`` is on and never cross-compiled — which is why
``<random>`` and dynamic allocation are acceptable there, unlike in the
on-target conversion code one directory up.

.. contents:: On this page
   :local:
   :depth: 2

---

Overview of the five models
---------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 20 24 16 10 8 10

   * - Sensor
     - Reference part
     - Truth inputs (FMI)
     - Wire protocol
     - Sync bytes
     - UDP port
     - Default rate
   * - GPS
     - u-blox M9N
     - lat, lon, alt, NED velocity
     - UBX-NAV-PVT (wire-exact)
     - ``B5 62``
     - 5762
     - 1 frame / step
   * - IMU
     - ADIS16470-class MEMS
     - specific force, angular rate (body)
     - Hemerion IMU frame
     - ``A5 5A``
     - 5763
     - 100 Hz
   * - Barometer
     - MS5611-class
     - geometric altitude MSL
     - Hemerion baro frame
     - ``B7 7B``
     - 5764
     - 50 Hz
   * - Radar altimeter
     - small FMCW altimeter
     - height above ground
     - Hemerion radalt frame
     - ``C9 9C``
     - 5765
     - 25 Hz
   * - Magnetometer
     - RM3100 / IIS2MDC-class
     - body-frame field [µT]
     - Hemerion mag frame
     - ``D1 1D``
     - 5766
     - 100 Hz

Every FMU takes its UDP destination from the environment
(``HEMERION_<SENSOR>_FMU_UDP_HOST`` / ``HEMERION_<SENSOR>_FMU_UDP_PORT``,
with ``<SENSOR>`` one of ``GPS``, ``IMU``, ``BARO``, ``RADALT``, ``MAG``),
defaulting to ``127.0.0.1`` and the port above. The four register-count
sensors expose a ``sample_rate_hz`` FMI parameter: each ``fmi2DoStep`` emits
``round(step · rate)`` frames, so the sensor rate is decoupled from the
co-simulation communication step. The GPS FMU instead emits exactly one
NAV-PVT frame per communication step, matching a receiver's navigation
epoch.

The shared error-model form
---------------------------

Except for the GPS (which perturbs the fix directly, see below), every model
applies the same two-term Gaussian error to each measurement channel and
then quantizes to register counts:

.. math::

   y[k] \;=\; \operatorname{sat}\!\Big(\operatorname{round}\big(
        (x_{\text{truth}}[k] + b + w[k])\, S \big)\Big),
   \qquad
   b \sim \mathcal{N}(0, \sigma_b^2), \quad
   w[k] \sim \mathcal{N}(0, \sigma_w^2)

where

* :math:`b` is a **turn-on bias**, drawn *once per run* at model
  construction (per axis/channel) — the run-to-run constant offset a real
  part exhibits after power-up;
* :math:`w[k]` is **white measurement noise**, drawn per sample;
* :math:`S` is the register sensitivity (LSB per physical unit), configured
  identically to what the consuming driver passes to
  ``convert_raw_to_si()`` — scale is *not* part of the wire format, exactly
  as on real silicon;
* :math:`\operatorname{sat}(\cdot)` saturates at the register's full-scale
  range (int16 for IMU/mag, 24-bit words for baro/radalt) the way real
  silicon clips, rather than wrapping.

Setting any :math:`\sigma` to zero disables that term cleanly (the models
never construct a ``std::normal_distribution`` with zero stddev, which would
be undefined behavior).

**Reproducibility.** Every model constructor takes an RNG seed, defaulting
to a nondeterministic ``std::random_device`` draw. Pass a fixed seed for
bit-identical runs; the turn-on biases are drawn from that same stream at
construction, so seed + config fully determine the output sequence.

GPS receiver model
------------------

:file: ``modules/sensors/include/Hemerion/gps/fmu/gpsNoiseModel.hpp``

``GpsNoiseModel`` perturbs the truth trajectory at the *fix* level — a
receiver outputs a navigation solution, not register counts — producing a
``GpsFix`` that looks like it came from a u-blox M9N in open sky, which
``UbxEmitter`` then encodes as a wire-exact UBX-NAV-PVT frame.

Horizontal noise is drawn independently in local north/east metres and
converted to degrees with a flat-Earth approximation:

.. math::

   \Delta\varphi = \frac{n_N}{111320\ \text{m/deg}}, \qquad
   \Delta\lambda = \frac{n_E}{111320 \cos\varphi\ \text{m/deg}},
   \qquad n_N, n_E \sim \mathcal{N}(0, \sigma_h^2)

adequate for a sensor noise model, not a navigation-grade datum transform.
Ground speed is floored at zero after perturbation; course is wrapped into
[0°, 360°). The receiver's *self-reported* accuracy fields
(``hAcc``/``vAcc`` in NAV-PVT) are filled with the configured 1-sigma
values — so the flight software sees an honest receiver.

.. list-table:: ``GpsNoiseConfig`` defaults (u-blox M9N, open sky)
   :header-rows: 1
   :widths: 40 20 40

   * - Parameter
     - Default
     - Meaning
   * - ``horizontal_pos_noise_m``
     - 1.5 m
     - 1-sigma, applied independently to north/east
   * - ``vertical_pos_noise_m``
     - 3.0 m
     - 1-sigma altitude noise
   * - ``speed_noise_mps``
     - 0.1 m/s
     - 1-sigma speed-over-ground noise
   * - ``course_noise_deg``
     - 1.0°
     - 1-sigma course noise
   * - ``num_satellites``
     - 11
     - constant satellite count reported
   * - ``fix_type``
     - 3D fix
     - constant fix type reported

FMI inputs (value references in parentheses): ``latitude_deg`` (0),
``longitude_deg`` (1), ``altitude_m`` (2), ``ground_speed_mps`` (3),
``course_deg`` (4), ``v_north_mps`` (5), ``v_east_mps`` (6),
``v_down_mps`` (7). When NED velocity is wired, the FMU derives
speed-over-ground and course itself. See :ref:`rocket_gps_ecos_cosim` for a
complete, verified wiring of this FMU against a 6-DoF plant.

IMU model
---------

:file: ``modules/sensors/include/Hemerion/imu/fmu/imu_noise_model.h``

``ImuNoiseModel`` turns a noiseless body-frame truth sample — specific force
and angular rate — into the raw register counts a tactical-grade MEMS IMU
(ADIS16470-class at its widest ranges: ±40 g, ±2000 °/s, 16-bit registers)
would latch. Per axis:

.. math::

   a_{\text{counts}} &= \operatorname{sat}_{16}\!\Big(\operatorname{round}\big(
       (f + b_a + w_a)\, \tfrac{S_a}{g_0}\big)\Big), &\quad S_a &= 800\ \text{LSB/g} \\
   \omega_{\text{counts}} &= \operatorname{sat}_{16}\!\Big(\operatorname{round}\big(
       (\omega + b_g + w_g)\, \tfrac{S_g}{\pi/180}\big)\Big), &\quad S_g &= 16.4\ \text{LSB/(°/s)}

with :math:`g_0 = 9.80665\ \text{m/s}^2` — deliberately the same constant as
``kStandardGravityMps2`` in ``imu_conversion.cpp`` (itself a placeholder
pending Aetherion's environment model), so the round trip through
``convert_raw_to_si()`` is exact to quantization error.

.. list-table:: ``ImuNoiseConfig`` defaults
   :header-rows: 1
   :widths: 40 20 40

   * - Parameter
     - Default
     - Meaning
   * - ``accel_noise_mps2``
     - 0.05 m/s²
     - white noise 1-sigma, per axis
   * - ``gyro_noise_rad_s``
     - 0.002 rad/s
     - white noise 1-sigma, per axis
   * - ``accel_bias_sigma_mps2``
     - 0.02 m/s²
     - turn-on bias 1-sigma, drawn once per run per axis
   * - ``gyro_bias_sigma_rad_s``
     - 0.001 rad/s
     - turn-on bias 1-sigma, drawn once per run per axis
   * - ``scale``
     - 800 LSB/g, 16.4 LSB/(°/s)
     - register sensitivity, must match the driver's ``ImuScale``

FMI inputs: ``specific_force_{x,y,z}_mps2`` (0–2),
``angular_rate_{x,y,z}_rad_s`` (3–5), parameter ``sample_rate_hz`` (6,
default 100 Hz).

Barometer model
---------------

:file: ``modules/sensors/include/Hemerion/baro/fmu/baro_noise_model.h``

The barometer model is the only one that embeds an *environment* model: the
plant supplies geometric altitude above mean sea level, and
``BaroNoiseModel`` maps it through the **ICAO Standard Atmosphere** to the
ambient pressure and temperature an MS5611-class part would sense, before
applying the standard bias + noise + quantization chain to each channel.

Two ISA layers are modeled. The gradient troposphere up to 11 km:

.. math::

   p(h) = p_0 \left(1 - \frac{L\,h}{T_0}\right)^{g_0 / (L R)}, \qquad
   T(h) = T_0 - L\,h

with :math:`p_0 = 101325\ \text{Pa}`, :math:`T_0 = 288.15\ \text{K}`,
:math:`L = 6.5\ \text{K/km}`, :math:`R = 287.05287\ \text{J/(kg·K)}` — and
above it the isothermal stratosphere at :math:`T_{11} = 216.65\ \text{K}`,
extended upward indefinitely:

.. math::

   p(h) = p_{11} \exp\!\left(-\frac{g_0 (h - 11\,\text{km})}{R\, T_{11}}\right),
   \qquad p_{11} = 22632.06\ \text{Pa}

The 20 km+ gradient layers are ignored, which understates pressure slightly
above 20 km — where a rocket's baro reading is aerodynamically marginal
anyway. Below sea level the troposphere expression extrapolates. The
reported temperature is the die temperature (a small sensor tracks
ambient). Output words saturate at the signed 24-bit conversion-word range,
as MS5611-class parts output 24-bit compensated words.

.. note::

   The local ISA layers are a placeholder: once Aetherion grows an
   environment model, the atmosphere will be sourced from it — the same
   policy as :math:`g_0` in ``imu_conversion.cpp``.

.. list-table:: ``BaroNoiseConfig`` defaults (MS5611-class)
   :header-rows: 1
   :widths: 40 20 40

   * - Parameter
     - Default
     - Meaning
   * - ``pressure_noise_pa``
     - 3 Pa
     - white noise 1-sigma
   * - ``temperature_noise_c``
     - 0.05 °C
     - white noise 1-sigma
   * - ``pressure_bias_sigma_pa``
     - 50 Pa
     - turn-on bias 1-sigma (within the ±1.5 mbar datasheet band)
   * - ``temperature_bias_sigma_c``
     - 0.5 °C
     - turn-on bias 1-sigma
   * - ``scale``
     - 1 LSB/Pa, 100 LSB/°C
     - output sensitivity, must match the driver's ``BaroScale``

FMI inputs: ``altitude_m`` (0), parameter ``sample_rate_hz`` (1, default
50 Hz).

Radar altimeter model
---------------------

:file: ``modules/sensors/include/Hemerion/radalt/fmu/radalt_noise_model.h``

``RadAltNoiseModel`` approximates a small FMCW radar altimeter:
centimeter-class resolution (100 LSB/m), decimeter-class noise, a few
kilometers of tracking range. Its distinguishing feature is the
**loss-of-track** behavior: beyond the configured maximum tracking range —
or below zero truth height — the model reports
``kRadAltStatusNoReturn`` with a *zeroed* range word, exactly as a real
part loses ground track rather than clipping at full scale. Within range,
the standard bias + noise chain applies, floored at zero (a radar range is
never negative) and saturated at the 24-bit range register.

.. list-table:: ``RadAltNoiseConfig`` defaults
   :header-rows: 1
   :widths: 40 20 40

   * - Parameter
     - Default
     - Meaning
   * - ``range_noise_m``
     - 0.1 m
     - white noise 1-sigma
   * - ``range_bias_sigma_m``
     - 0.05 m
     - turn-on bias 1-sigma
   * - ``max_range_m``
     - 6000 m
     - beyond this the model reports no return
   * - ``scale``
     - 100 LSB/m
     - register sensitivity, must match the driver's ``RadAltScale``

FMI inputs: ``height_agl_m`` (0), parameter ``sample_rate_hz`` (1, default
25 Hz).

Magnetometer model
------------------

:file: ``modules/sensors/include/Hemerion/mag/fmu/mag_noise_model.h``

``MagNoiseModel`` perturbs a body-frame truth magnetic field (a 6-DoF
plant's attitude applied to a geomagnetic reference field) the way a
compass-grade three-axis part (RM3100/IIS2MDC class) would: sub-µT white
noise plus a constant per-run **hard-iron-like** turn-on bias per axis,
quantized at 100 LSB/µT into 16-bit registers — a ±327 µT full scale,
comfortably above Earth's ~25–65 µT field plus installation offsets.

**Soft-iron distortion is deliberately out of scope** for now: a
diagonal-only error model keeps the inverse trivially equal to
``convert_raw_to_si()``, preserving the round-trip property the whole
simulator family relies on.

.. list-table:: ``MagNoiseConfig`` defaults
   :header-rows: 1
   :widths: 40 20 40

   * - Parameter
     - Default
     - Meaning
   * - ``mag_noise_ut``
     - 0.2 µT
     - white noise 1-sigma, per axis
   * - ``mag_bias_sigma_ut``
     - 1.0 µT
     - turn-on (hard-iron-like) bias 1-sigma, per axis, once per run
   * - ``scale``
     - 100 LSB/µT
     - register sensitivity, must match the driver's ``MagScale``

FMI inputs: ``mag_{x,y,z}_ut`` (0–2), parameter ``sample_rate_hz`` (3,
default 100 Hz).

The Hemerion sensor wire protocol
---------------------------------

The GPS FMU speaks real UBX; the other four share a minimal binary framing
that deliberately follows the same shape as UBX — sync bytes, message id,
little-endian length, payload, two-byte Fletcher checksum — so each parser
mirrors the proven ``UbxParser`` structure:

.. code-block:: text

   offset  size  field
        0     1  sync1
        1     1  sync2
        2     1  message id (0x01 = raw sample)
        3     2  payload length, little-endian
        5     n  payload (int32 register counts + uint64 timestamp_us)
      5+n     2  checksum ck_a, ck_b (UBX-style Fletcher over id + length + payload)

Each sensor uses **distinct sync bytes** (IMU ``A5 5A``, baro ``B7 7B``,
radalt ``C9 9C``, mag ``D1 1D``, vs. UBX's ``B5 62``), so an interleaved
multi-sensor byte stream can never desync one parser into another's frames.
Payload lengths differ per sensor: 32 bytes for the IMU (6 × int32 counts),
16 for the barometer (pressure + temperature), 16 for the radar altimeter
(range + status word), 20 for the magnetometer (3 × int32 counts) — each
followed by the uint64 sample timestamp in microseconds, carried from the
*sender's* clock.

Two properties are shared with real silicon by design:

* **Scale is not on the wire.** The driver knows the full-scale range it
  configured and applies ``convert_raw_to_si()`` with the matching scale
  struct — the same values the noise model quantized with (exposed via each
  model's ``scale()`` accessor).
* **Unknown message ids are checksummed and skipped**, not buffered, so a
  stream carrying future message types doesn't desync today's parser.

Each ``<sensor>_packet_emitter`` is the stateless inverse of the
corresponding parser; per-sensor round-trip tests
(``modules/sensors/test/``) feed the emitter's output back through the real
parser to prove the two stay in sync.

API reference
-------------

GPS
~~~

.. doxygenstruct:: hemerion::sensors::gps::fmu::GpsTruthSample
   :members:

.. doxygenstruct:: hemerion::sensors::gps::fmu::GpsNoiseConfig
   :members:

.. doxygenclass:: hemerion::sensors::gps::fmu::GpsNoiseModel
   :members:

IMU
~~~

.. doxygenstruct:: hemerion::sensors::imu::fmu::ImuTruthSample
   :members:

.. doxygenstruct:: hemerion::sensors::imu::fmu::ImuNoiseConfig
   :members:

.. doxygenclass:: hemerion::sensors::imu::fmu::ImuNoiseModel
   :members:

Barometer
~~~~~~~~~

.. doxygenstruct:: hemerion::sensors::baro::fmu::BaroTruthSample
   :members:

.. doxygenstruct:: hemerion::sensors::baro::fmu::BaroNoiseConfig
   :members:

.. doxygenclass:: hemerion::sensors::baro::fmu::BaroNoiseModel
   :members:

Radar altimeter
~~~~~~~~~~~~~~~

.. doxygenstruct:: hemerion::sensors::radalt::fmu::RadAltTruthSample
   :members:

.. doxygenstruct:: hemerion::sensors::radalt::fmu::RadAltNoiseConfig
   :members:

.. doxygenclass:: hemerion::sensors::radalt::fmu::RadAltNoiseModel
   :members:

Magnetometer
~~~~~~~~~~~~

.. doxygenstruct:: hemerion::sensors::mag::fmu::MagTruthSample
   :members:

.. doxygenstruct:: hemerion::sensors::mag::fmu::MagNoiseConfig
   :members:

.. doxygenclass:: hemerion::sensors::mag::fmu::MagNoiseModel
   :members:
