// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file magnetic_calibration.h
/// @brief Recovers hard- and soft-iron distortion from field samples taken in
/// many attitudes.
///
/// **This is not `Mmc5983maDriver::calibrate_offset()`, and the two are about
/// different things.** That routine drives the part's SET/RESET pair and
/// cancels the *bridge offset* -- an artefact of the sensor die, present with
/// no vehicle around it at all. What this file solves for is the magnetic
/// environment the part is bolted into: the vehicle's own ferrous structure
/// and its currents. A part can be perfectly bridge-calibrated and still read
/// tens of degrees of heading error from an uncalibrated installation.
///
/// **What the distortion does.**
///
/// To the accuracy anyone models it, an installation turns the true field
/// @f$B@f$ into
///
/// @f[ m = S\,B + h @f]
///
/// * @f$h@f$ is **hard iron**: permanently magnetized material on the vehicle,
///   which adds a constant offset in *body* axes. It is the larger effect and
///   the easier one.
/// * @f$S@f$ is **soft iron**: material that concentrates and re-directs the
///   ambient field rather than generating one, so its effect is proportional
///   to the field and linear in it. Near identity, but not identity.
///
/// **Why an ellipsoid.**
///
/// Turn the vehicle through every attitude under a field of constant
/// magnitude and the true samples trace a *sphere* of radius @f$|B|@f$.
/// Applying the distortion above maps that sphere to an ellipsoid: offset by
/// @f$h@f$, and stretched and sheared by @f$S@f$. So fitting an ellipsoid to
/// the samples and computing the map that takes it back to a sphere is the
/// calibration. Concretely, fitting
///
/// @f[ m^{T} A m + b^{T} m = 1 @f]
///
/// recovers the centre @f$h = -\tfrac{1}{2}A^{-1}b@f$ and, after
/// re-normalising, the shape matrix whose symmetric square root is
/// @f$S^{-1}@f$ up to scale.
///
/// **The limitation, stated rather than buried.**
///
/// **An ellipsoid fit cannot recover @f$S@f$ uniquely.** For any rotation
/// @f$R@f$, @f$\|R\,S^{-1}(m-h)\| = \|S^{-1}(m-h)\|@f$, so every
/// @f$R\,S^{-1}@f$ fits the data equally well: the samples constrain the
/// ellipsoid's shape, not its orientation relative to the body frame. This
/// returns the symmetric positive-definite square root, which is the
/// canonical choice and the one that is *correct* whenever the true @f$S@f$ is
/// itself symmetric -- which is what the MMC5983MA FMU models, so the loop
/// closes exactly in simulation. On hardware the residual rotation has to come
/// from somewhere else: another reference, usually the IMU, resolving the
/// magnetometer's axes against the airframe's. Until that exists, a heading
/// computed from this correction can carry a constant rotation that the fit
/// cannot see.
///
/// **On-target by construction.**
///
/// The sample accumulator is @f$O(1)@f$ in the number of samples, not
/// @f$O(n)@f$: each one is folded into a fixed 9x9 normal matrix as it
/// arrives, so a calibration over a hundred thousand samples costs the same
/// memory as one over ten. No allocation, no exceptions, no recursion, and
/// every loop bound is a compile-time constant -- it is meant to run on the
/// STM32H743 during a calibration manoeuvre, not only in a ground tool.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

/// @namespace hemerion::sensors::mag
/// @brief Magnetometer types, conversion and calibration.
namespace hemerion::sensors::mag
{

/// Why a calibration could not be solved.
enum class MagCalibrationError : std::uint8_t
{
  kNone = 0,          ///< Solved; the result is usable.
  kInsufficientData,  ///< Fewer samples than `kMinimumSamples`.
  kDegenerate,        ///< The normal equations are singular: the samples do
                      ///< not span enough attitudes to define an ellipsoid.
                      ///< Rotating about one axis only is the usual cause.
  kNotEllipsoid,      ///< The fitted quadric is not a bounded ellipsoid (the
                      ///< shape matrix is not positive definite). Too little
                      ///< attitude coverage, or samples dominated by noise.
  kInvalidField,      ///< The expected field magnitude was not positive.
};

/// The correction a calibration produces.
///
/// Apply it with `MagneticCalibration::correct()`, or directly:
/// @f$B = W\,(m - h)@f$.
struct MagCorrection
{
  /// Hard-iron offset @f$h@f$ in body axes [uT]: subtract it first.
  std::array<double, 3> hard_iron_ut{};

  /// Soft-iron correction @f$W \approx S^{-1}@f$, symmetric, near identity.
  /// Row-major. Scaled so a corrected sample has magnitude
  /// `expected_field_ut`.
  std::array<std::array<double, 3>, 3> soft_iron_inverse{ { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } } };

  /// @brief Applies the correction to one measured body-frame field [uT].
  [[nodiscard]] std::array<double, 3> apply(double x_ut, double y_ut, double z_ut) const;
};

/// @brief Streaming ellipsoid fit over magnetometer samples.
///
/// Feed it body-frame field measurements taken across as many attitudes as the
/// vehicle can reach, then solve. Typical use is a deliberate calibration
/// manoeuvre; the accumulator is cheap enough to leave running.
class MagneticCalibration
{
public:
  /// The fit has nine free parameters, so nine samples is the algebraic
  /// minimum and a useless one. This floor is high enough that a solve which
  /// succeeds has seen something like a real manoeuvre rather than a corner
  /// of one -- `kDegenerate` catches the rest.
  static constexpr std::size_t kMinimumSamples = 50;

  /// @brief Folds one measurement into the normal equations.
  ///
  /// @param x_ut Measured body-frame field, X axis [uT].
  /// @param y_ut Measured body-frame field, Y axis [uT].
  /// @param z_ut Measured body-frame field, Z axis [uT].
  void add_sample(double x_ut, double y_ut, double z_ut);

  /// @brief Solves for the correction.
  ///
  /// @param expected_field_ut Local field magnitude [uT], from a field model
  ///        at the vehicle's position. The fit determines the ellipsoid's
  ///        shape; this sets its scale, since samples alone cannot separate a
  ///        uniformly scaled soft iron from a weaker field.
  /// @param out Receives the correction on success, untouched otherwise.
  /// @return kNone on success, else why not.
  [[nodiscard]] MagCalibrationError solve(double expected_field_ut, MagCorrection& out) const;

  /// @brief Discards everything accumulated so far.
  void reset();

  /// How many samples have been folded in.
  [[nodiscard]] std::size_t sample_count() const { return sample_count_; }

private:
  /// Design vector length: [x^2, y^2, z^2, 2xy, 2xz, 2yz, 2x, 2y, 2z].
  static constexpr std::size_t kParameters = 9;

  /// Upper-triangular accumulation of sum(d d^T) and sum(d), which is all the
  /// least-squares problem needs -- hence O(1) in the sample count.
  std::array<std::array<double, kParameters>, kParameters> normal_{};
  std::array<double, kParameters> rhs_{};
  std::size_t sample_count_ = 0;
};

}  // namespace hemerion::sensors::mag
