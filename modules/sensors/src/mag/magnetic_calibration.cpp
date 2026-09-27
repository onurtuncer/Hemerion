// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------

/// @file magnetic_calibration.cpp
/// @brief Ellipsoid fit and its decomposition; see magnetic_calibration.h for
/// what the result means and what it cannot determine.

#include "Hemerion/mag/magnetic_calibration.h"

#include <cmath>

namespace hemerion::sensors::mag
{

namespace
{

constexpr std::size_t kN = 9;
constexpr std::size_t kDim = 3;

using Matrix9 = std::array<std::array<double, kN>, kN>;
using Vector9 = std::array<double, kN>;
using Matrix3 = std::array<std::array<double, kDim>, kDim>;
using Vector3 = std::array<double, kDim>;

/// Solves `a x = b` for a symmetric positive-definite `a`, by Cholesky.
///
/// Returns false rather than producing rubbish when the matrix is not
/// positive definite, which is how a sample set that does not span enough
/// attitudes announces itself: the normal equations go singular because the
/// data constrain fewer than nine directions.
bool solve_spd(Matrix9 a, Vector9 b, Vector9& x)
{
  // Cholesky: a = L L^T, lower triangular in place.
  for (std::size_t i = 0; i < kN; ++i)
  {
    for (std::size_t j = 0; j <= i; ++j)
    {
      double sum = a[i][j];
      for (std::size_t k = 0; k < j; ++k)
      {
        sum -= a[i][k] * a[j][k];
      }
      if (i == j)
      {
        // A tolerance rather than > 0: the normal equations of a nearly
        // degenerate sample set are positive definite only in exact
        // arithmetic, and a pivot this small means the answer would be noise
        // amplified by 1/eps whatever its sign.
        if (sum < 1e-12)
        {
          return false;
        }
        a[i][j] = std::sqrt(sum);
      }
      else
      {
        a[i][j] = sum / a[j][j];
      }
    }
  }

  // Forward substitution, L y = b.
  Vector9 y{};
  for (std::size_t i = 0; i < kN; ++i)
  {
    double sum = b[i];
    for (std::size_t k = 0; k < i; ++k)
    {
      sum -= a[i][k] * y[k];
    }
    y[i] = sum / a[i][i];
  }

  // Back substitution, L^T x = y.
  for (std::size_t i = kN; i-- > 0;)
  {
    double sum = y[i];
    for (std::size_t k = i + 1; k < kN; ++k)
    {
      sum -= a[k][i] * x[k];
    }
    x[i] = sum / a[i][i];
  }
  return true;
}

/// Solves a 3x3 system by Gaussian elimination with partial pivoting. Used
/// once, for the ellipsoid's centre, where the matrix is symmetric but its
/// definiteness is not yet established -- so not Cholesky.
bool solve_3x3(Matrix3 a, Vector3 b, Vector3& x)
{
  for (std::size_t col = 0; col < kDim; ++col)
  {
    std::size_t pivot = col;
    for (std::size_t row = col + 1; row < kDim; ++row)
    {
      if (std::fabs(a[row][col]) > std::fabs(a[pivot][col]))
      {
        pivot = row;
      }
    }
    if (std::fabs(a[pivot][col]) < 1e-14)
    {
      return false;
    }
    if (pivot != col)
    {
      a[pivot].swap(a[col]);
      const double t = b[pivot];
      b[pivot] = b[col];
      b[col] = t;
    }
    for (std::size_t row = col + 1; row < kDim; ++row)
    {
      const double factor = a[row][col] / a[col][col];
      for (std::size_t k = col; k < kDim; ++k)
      {
        a[row][k] -= factor * a[col][k];
      }
      b[row] -= factor * b[col];
    }
  }
  for (std::size_t i = kDim; i-- > 0;)
  {
    double sum = b[i];
    for (std::size_t k = i + 1; k < kDim; ++k)
    {
      sum -= a[i][k] * x[k];
    }
    x[i] = sum / a[i][i];
  }
  return true;
}

/// Symmetric 3x3 eigen-decomposition by cyclic Jacobi rotations.
///
/// Fixed iteration bound rather than "until converged": this runs on a flight
/// computer, where a loop whose length depends on the data is a loop whose
/// worst case nobody has measured. Jacobi on a 3x3 converges quadratically and
/// is comfortably done inside this bound; the sweep exits early when the
/// off-diagonal mass is negligible.
void jacobi_eigen(Matrix3 a, Matrix3& vectors, Vector3& values)
{
  vectors = { { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } } };

  for (int sweep = 0; sweep < 24; ++sweep)
  {
    const double off = (a[0][1] * a[0][1]) + (a[0][2] * a[0][2]) + (a[1][2] * a[1][2]);
    if (off < 1e-30)
    {
      break;
    }
    for (std::size_t p = 0; p < kDim - 1; ++p)
    {
      for (std::size_t q = p + 1; q < kDim; ++q)
      {
        if (std::fabs(a[p][q]) < 1e-30)
        {
          continue;
        }
        const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        const double sign = (theta >= 0.0) ? 1.0 : -1.0;
        const double t = sign / ((sign * theta) + std::sqrt((theta * theta) + 1.0));
        const double c = 1.0 / std::sqrt((t * t) + 1.0);
        const double s = t * c;

        for (std::size_t k = 0; k < kDim; ++k)
        {
          const double akp = a[k][p];
          const double akq = a[k][q];
          a[k][p] = (c * akp) - (s * akq);
          a[k][q] = (s * akp) + (c * akq);
        }
        for (std::size_t k = 0; k < kDim; ++k)
        {
          const double apk = a[p][k];
          const double aqk = a[q][k];
          a[p][k] = (c * apk) - (s * aqk);
          a[q][k] = (s * apk) + (c * aqk);
        }
        for (std::size_t k = 0; k < kDim; ++k)
        {
          const double vkp = vectors[k][p];
          const double vkq = vectors[k][q];
          vectors[k][p] = (c * vkp) - (s * vkq);
          vectors[k][q] = (s * vkp) + (c * vkq);
        }
      }
    }
  }
  values = { a[0][0], a[1][1], a[2][2] };
}

}  // namespace

std::array<double, 3> MagCorrection::apply(double x_ut, double y_ut, double z_ut) const
{
  const double dx = x_ut - hard_iron_ut[0];
  const double dy = y_ut - hard_iron_ut[1];
  const double dz = z_ut - hard_iron_ut[2];
  return { (soft_iron_inverse[0][0] * dx) + (soft_iron_inverse[0][1] * dy) + (soft_iron_inverse[0][2] * dz),
           (soft_iron_inverse[1][0] * dx) + (soft_iron_inverse[1][1] * dy) + (soft_iron_inverse[1][2] * dz),
           (soft_iron_inverse[2][0] * dx) + (soft_iron_inverse[2][1] * dy) + (soft_iron_inverse[2][2] * dz) };
}

void MagneticCalibration::add_sample(double x_ut, double y_ut, double z_ut)
{
  // The design vector of m^T A m + b^T m = 1, with the off-diagonal quadratic
  // terms carrying their factor of two so that A reads straight off the
  // solution as a symmetric matrix.
  const std::array<double, kParameters> d = { x_ut * x_ut,       y_ut * y_ut,       z_ut * z_ut,
                                              2.0 * x_ut * y_ut, 2.0 * x_ut * z_ut, 2.0 * y_ut * z_ut,
                                              2.0 * x_ut,        2.0 * y_ut,        2.0 * z_ut };

  for (std::size_t i = 0; i < kParameters; ++i)
  {
    rhs_[i] += d[i];
    for (std::size_t j = 0; j < kParameters; ++j)
    {
      normal_[i][j] += d[i] * d[j];
    }
  }
  ++sample_count_;
}

void MagneticCalibration::reset()
{
  normal_ = {};
  rhs_ = {};
  sample_count_ = 0;
}

MagCalibrationError MagneticCalibration::solve(double expected_field_ut, MagCorrection& out) const
{
  if (!(expected_field_ut > 0.0))
  {
    return MagCalibrationError::kInvalidField;
  }
  if (sample_count_ < kMinimumSamples)
  {
    return MagCalibrationError::kInsufficientData;
  }

  Vector9 parameters{};
  if (!solve_spd(normal_, rhs_, parameters))
  {
    return MagCalibrationError::kDegenerate;
  }

  // Unpack into the quadratic form. The factors of two went into the design
  // vector, so these are the matrix entries themselves.
  const Matrix3 quadratic = { { { parameters[0], parameters[3], parameters[4] },
                                { parameters[3], parameters[1], parameters[5] },
                                { parameters[4], parameters[5], parameters[2] } } };
  const Vector3 linear = { parameters[6], parameters[7], parameters[8] };

  // Centre: h = -(1/2) A^{-1} b, and the design vector already carries the 2,
  // so the halving is done.
  Vector3 centre{};
  Vector3 negated = { -linear[0], -linear[1], -linear[2] };
  if (!solve_3x3(quadratic, negated, centre))
  {
    return MagCalibrationError::kDegenerate;
  }

  // Re-normalise: fitting against a right-hand side of 1 leaves the quadratic
  // form scaled by 1/(1 - h^T M h). Undo it so the shape matrix is the one
  // whose square root is S^{-1}.
  double h_a_h = 0.0;
  for (std::size_t i = 0; i < kDim; ++i)
  {
    for (std::size_t j = 0; j < kDim; ++j)
    {
      h_a_h += centre[i] * quadratic[i][j] * centre[j];
    }
  }
  const double denominator = 1.0 + h_a_h;
  if (!(std::fabs(denominator) > 1e-15))
  {
    return MagCalibrationError::kNotEllipsoid;
  }

  Matrix3 shape{};
  for (std::size_t i = 0; i < kDim; ++i)
  {
    for (std::size_t j = 0; j < kDim; ++j)
    {
      shape[i][j] = quadratic[i][j] / denominator;
    }
  }

  // The symmetric square root, through the eigen-decomposition. A bounded
  // ellipsoid has three positive eigenvalues; anything else means the fit
  // found a hyperboloid or a cylinder, which is a sample-coverage failure
  // rather than a numerical one.
  Matrix3 vectors{};
  Vector3 values{};
  jacobi_eigen(shape, vectors, values);
  for (const double value : values)
  {
    if (!(value > 0.0))
    {
      return MagCalibrationError::kNotEllipsoid;
    }
  }

  // W = |B| * V diag(sqrt(lambda)) V^T, which is symmetric positive definite
  // by construction -- the canonical choice among the rotations that fit the
  // data equally well. See the header on why that choice is a limitation and
  // not merely a convention.
  Matrix3 correction{};
  for (std::size_t i = 0; i < kDim; ++i)
  {
    for (std::size_t j = 0; j < kDim; ++j)
    {
      double sum = 0.0;
      for (std::size_t k = 0; k < kDim; ++k)
      {
        sum += vectors[i][k] * std::sqrt(values[k]) * vectors[j][k];
      }
      correction[i][j] = expected_field_ut * sum;
    }
  }

  out.hard_iron_ut = { centre[0], centre[1], centre[2] };
  out.soft_iron_inverse = correction;
  return MagCalibrationError::kNone;
}

}  // namespace hemerion::sensors::mag
