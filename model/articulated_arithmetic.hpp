// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cmath>

#include "framework/vocabulary.hpp"
#include "model/articulated.hpp"

// MuJoCo's engine arithmetic on three-vectors, quaternions and 3x3 matrices,
// shared by the articulated dynamics, collisions and viewer, each in
// MuJoCo's order of operations so that results match it to the last bit.
// Matrices are row by row.
namespace simon::model {

using Matrix3 = std::array<double, 9>;

namespace articulated {

constexpr double MINVAL = 1e-15;  // MuJoCo's mjMINVAL.

// MuJoCo's engine arithmetic, engine_util_spatial.c and engine_util_blas.c
// (Apache-2.0), in its order of operations.

inline auto normalize4(InOut<Quaternion4> q) -> double {
  double norm = std::sqrt((*q)[0] * (*q)[0] + (*q)[1] * (*q)[1] +
                          (*q)[2] * (*q)[2] + (*q)[3] * (*q)[3]);
  if (norm < MINVAL) {
    *q = {1.0, 0.0, 0.0, 0.0};
  } else if (std::abs(norm - 1) > MINVAL) {
    double inverse = 1 / norm;
    for (double& x : *q) {
      x *= inverse;
    }
  }
  return norm;
}

inline auto normalize3(InOut<Array3> v) -> double {
  double norm =
      std::sqrt((*v)[0] * (*v)[0] + (*v)[1] * (*v)[1] + (*v)[2] * (*v)[2]);
  if (norm < MINVAL) {
    *v = {1.0, 0.0, 0.0};
  } else {
    double inverse = 1 / norm;
    for (double& x : *v) {
      x *= inverse;
    }
  }
  return norm;
}

inline auto multiply(const Quaternion4& a, const Quaternion4& b)
    -> Quaternion4 {
  return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
          a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
          a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
          a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

inline auto rotate(const Array3& v, const Quaternion4& q) -> Array3 {
  if (v[0] == 0 && v[1] == 0 && v[2] == 0) {
    return {0.0, 0.0, 0.0};
  }
  if (q[0] == 1 && q[1] == 0 && q[2] == 0 && q[3] == 0) {
    return v;
  }
  Array3 t{q[0] * v[0] + q[2] * v[2] - q[3] * v[1],
           q[0] * v[1] + q[3] * v[0] - q[1] * v[2],
           q[0] * v[2] + q[1] * v[1] - q[2] * v[0]};
  return {v[0] + 2 * (q[2] * t[2] - q[3] * t[1]),
          v[1] + 2 * (q[3] * t[0] - q[1] * t[2]),
          v[2] + 2 * (q[1] * t[1] - q[2] * t[0])};
}

inline auto convert_axis_angle(const Array3& axis, double angle)
    -> Quaternion4 {
  if (angle == 0) {
    return {1.0, 0.0, 0.0, 0.0};
  }
  double s = std::sin(angle * 0.5);
  return {std::cos(angle * 0.5), axis[0] * s, axis[1] * s, axis[2] * s};
}

inline auto convert_to_matrix(const Quaternion4& q) -> Matrix3 {
  if (q[0] == 1 && q[1] == 0 && q[2] == 0 && q[3] == 0) {
    return {1, 0, 0, 0, 1, 0, 0, 0, 1};
  }
  double q00 = q[0] * q[0];
  double q01 = q[0] * q[1];
  double q02 = q[0] * q[2];
  double q03 = q[0] * q[3];
  double q11 = q[1] * q[1];
  double q12 = q[1] * q[2];
  double q13 = q[1] * q[3];
  double q22 = q[2] * q[2];
  double q23 = q[2] * q[3];
  double q33 = q[3] * q[3];
  return {q00 + q11 - q22 - q33, 2 * (q12 - q03),       2 * (q13 + q02),
          2 * (q12 + q03),       q00 - q11 + q22 - q33, 2 * (q23 - q01),
          2 * (q13 - q02),       2 * (q23 + q01),       q00 - q11 - q22 + q33};
}

inline auto multiply(const Matrix3& m, const Array3& v) -> Array3 {
  return {m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
          m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
          m[6] * v[0] + m[7] * v[1] + m[8] * v[2]};
}

inline auto cross(const Array3& a, const Array3& b) -> Array3 {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}

inline auto dot(const Array3& a, const Array3& b) -> double {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline auto norm(const Array3& v) -> double { return std::sqrt(dot(v, v)); }

inline auto add(const Array3& a, const Array3& b) -> Array3 {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

inline auto subtract(const Array3& a, const Array3& b) -> Array3 {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

inline auto scale(const Array3& v, double s) -> Array3 {
  return {v[0] * s, v[1] * s, v[2] * s};
}

// a + s b (mji_addScl3).
inline auto add_scaled(const Array3& a, const Array3& b, double s) -> Array3 {
  return {a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]};
}

// v += w s (mji_addToScl3).
inline auto add_to_scaled(InOut<Array3> v, const Array3& w, double s) -> void {
  (*v)[0] += w[0] * s;
  (*v)[1] += w[1] * s;
  (*v)[2] += w[2] * s;
}

inline auto multiply_transposed(const Matrix3& m, const Array3& v) -> Array3 {
  return {m[0] * v[0] + m[3] * v[1] + m[6] * v[2],
          m[1] * v[0] + m[4] * v[1] + m[7] * v[2],
          m[2] * v[0] + m[5] * v[1] + m[8] * v[2]};
}

inline auto column(const Matrix3& m, int c) -> Array3 {
  return {m[c], m[c + 3], m[c + 6]};
}

inline auto transpose(const Matrix3& m) -> Matrix3 {
  return {m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]};
}

}  // namespace articulated

}  // namespace simon::model
