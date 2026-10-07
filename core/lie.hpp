// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cmath>

#include "Eigen/Geometry"
#include "core/vocabulary.hpp"

// The rotation group SO(3) and the rigid motion group SE(3), each with its
// Lie algebra: the exponential and logarithm between them, the adjoint, and
// the left and right Jacobians that relate a small change in the algebra to
// a small change on the group. Rotations are quaternions and rigid motions
// are a rotation and a translation, as the rest of simon holds them.
//
// A twist is [rotation; translation], rotation first, as Featherstone and
// Lynch and Park write it and MuJoCo stores it (see model/REFERENCES.md); a
// wrench likewise, [torque; force]. The right Jacobian relates a change on
// the right, X exp(delta): the body frame. The left Jacobian relates
// exp(delta) X: the world frame.
namespace simon {

using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

namespace so3 {

// Below this angle the series replace the closed forms, whose quotients lose
// precision.
inline constexpr double SMALL_ANGLE = 1e-6;

// The skew matrix [w]x, so that [w]x v = w x v.
inline auto hat(const Vector3& w) -> Matrix3 {
  Matrix3 w_x;
  w_x << 0.0, -w.z(), w.y(),  //
      w.z(), 0.0, -w.x(),     //
      -w.y(), w.x(), 0.0;
  return w_x;
}

// The vector of a skew matrix.
inline auto vee(const Matrix3& w_x) -> Vector3 {
  return {w_x(2, 1), w_x(0, 2), w_x(1, 0)};
}

// The rotation by the angle |w| about w (Rodrigues; as a quaternion, half
// the angle).
inline auto exp(const Vector3& w) -> Quaternion {
  double angle = w.norm();
  if (angle < SMALL_ANGLE) {
    // cos(a/2) and sin(a/2)/a to second order.
    double half = 0.5 * angle;
    return Quaternion{1.0 - 0.5 * half * half, 0.5 * w.x(), 0.5 * w.y(),
                      0.5 * w.z()}
        .normalized();
  }
  double half = 0.5 * angle;
  Vector3 axis = w / angle;
  return Quaternion{Eigen::AngleAxisd{angle, axis}};
}

// The rotation vector of `q`: the angle in [0, pi] about the axis.
inline auto log(const Quaternion& q) -> Vector3 {
  // The same rotation with a non-negative scalar, so that the angle is the
  // shorter way round.
  double w = q.w();
  Vector3 v = q.vec();
  if (w < 0.0) {
    w = -w;
    v = -v;
  }
  double sine = v.norm();
  if (sine < SMALL_ANGLE) {
    // 2 atan(s / w) / s to second order in s / w.
    double ratio = sine / w;
    return (2.0 / w) * (1.0 - ratio * ratio / 3.0) * v;
  }
  double angle = 2.0 * std::atan2(sine, w);
  return (angle / sine) * v;
}

// The left Jacobian: exp(w + delta) = exp(J_l(w) delta) exp(w), to first
// order.
inline auto left_jacobian(const Vector3& w) -> Matrix3 {
  double angle = w.norm();
  Matrix3 w_x = hat(w);
  if (angle < SMALL_ANGLE) {
    return Matrix3::Identity() + 0.5 * w_x + (1.0 / 6.0) * w_x * w_x;
  }
  double a2 = angle * angle;
  return Matrix3::Identity() + ((1.0 - std::cos(angle)) / a2) * w_x +
         ((angle - std::sin(angle)) / (a2 * angle)) * w_x * w_x;
}

// The right Jacobian: exp(w + delta) = exp(w) exp(J_r(w) delta), to first
// order.
inline auto right_jacobian(const Vector3& w) -> Matrix3 {
  return left_jacobian(-w);
}

inline auto inverse_left_jacobian(const Vector3& w) -> Matrix3 {
  double angle = w.norm();
  Matrix3 w_x = hat(w);
  if (angle < SMALL_ANGLE) {
    return Matrix3::Identity() - 0.5 * w_x + (1.0 / 12.0) * w_x * w_x;
  }
  double a2 = angle * angle;
  double cot = (1.0 + std::cos(angle)) / (2.0 * angle * std::sin(angle));
  return Matrix3::Identity() - 0.5 * w_x + (1.0 / a2 - cot) * w_x * w_x;
}

inline auto inverse_right_jacobian(const Vector3& w) -> Matrix3 {
  return inverse_left_jacobian(-w);
}

}  // namespace so3

namespace se3 {

// A rigid motion: the rotation, then the translation, so that it takes a
// point p to rotation p + translation.
struct Pose final {
  Quaternion rotation = Quaternion::Identity();
  Vector3 translation = Vector3::Zero();
};

inline auto compose(const Pose& a, const Pose& b) -> Pose {
  return {.rotation = (a.rotation * b.rotation).normalized(),
          .translation = a.rotation * b.translation + a.translation};
}

inline auto inverse(const Pose& pose) -> Pose {
  Quaternion inverted = pose.rotation.conjugate();
  return {.rotation = inverted, .translation = -(inverted * pose.translation)};
}

// The point `pose` takes `point` to.
inline auto act(const Pose& pose, const Vector3& point) -> Vector3 {
  return pose.rotation * point + pose.translation;
}

// The 4x4 matrix of a twist, [[w]x, v; 0, 0].
inline auto hat(const Vector6& twist) -> Eigen::Matrix4d {
  Eigen::Matrix4d twist_x = Eigen::Matrix4d::Zero();
  twist_x.topLeftCorner<3, 3>() = so3::hat(twist.head<3>());
  twist_x.topRightCorner<3, 1>() = twist.tail<3>();
  return twist_x;
}

inline auto vee(const Eigen::Matrix4d& twist_x) -> Vector6 {
  Vector6 twist;
  twist.head<3>() = so3::vee(twist_x.topLeftCorner<3, 3>());
  twist.tail<3>() = twist_x.topRightCorner<3, 1>();
  return twist;
}

// The Lie bracket [a, b], ad_a b: how the motion `b` changes as seen from a
// frame moving with `a` (Featherstone's motion cross product, a x b).
inline auto cross_motion(const Vector6& a, const Vector6& b) -> Vector6 {
  Vector6 r;
  r.head<3>() = a.head<3>().cross(b.head<3>());
  r.tail<3>() = a.head<3>().cross(b.tail<3>()) + a.tail<3>().cross(b.head<3>());
  return r;
}

// Its dual on wrenches, -ad_aᵀ f: how the wrench `f` changes as seen from a
// frame moving with `a` (Featherstone's force cross product, a x* f).
inline auto cross_force(const Vector6& a, const Vector6& f) -> Vector6 {
  Vector6 r;
  r.head<3>() = a.head<3>().cross(f.head<3>()) + a.tail<3>().cross(f.tail<3>());
  r.tail<3>() = a.head<3>().cross(f.tail<3>());
  return r;
}

// The motion a twist makes in unit time: the rotation by its angular part,
// and the translation its linear part sweeps while turning.
inline auto exp(const Vector6& twist) -> Pose {
  Vector3 rotation = twist.head<3>();
  return {.rotation = so3::exp(rotation),
          .translation = so3::left_jacobian(rotation) * twist.tail<3>()};
}

inline auto log(const Pose& pose) -> Vector6 {
  Vector3 rotation = so3::log(pose.rotation);
  Vector6 twist;
  twist.head<3>() = rotation;
  twist.tail<3>() = so3::inverse_left_jacobian(rotation) * pose.translation;
  return twist;
}

// The adjoint, which moves a twist between frames: exp(Ad_X twist) =
// X exp(twist) X^-1.
inline auto adjoint(const Pose& pose) -> Matrix6 {
  Matrix3 rotation = pose.rotation.toRotationMatrix();
  Matrix6 adjoint = Matrix6::Zero();
  adjoint.topLeftCorner<3, 3>() = rotation;
  adjoint.bottomLeftCorner<3, 3>() = so3::hat(pose.translation) * rotation;
  adjoint.bottomRightCorner<3, 3>() = rotation;
  return adjoint;
}

// The block of the SE(3) left Jacobian that couples the translation to the
// rotation (Barfoot's Q; see model/REFERENCES.md).
inline auto coupling(const Vector6& twist) -> Matrix3 {
  Vector3 w = twist.head<3>();
  Vector3 rho = twist.tail<3>();
  double angle = w.norm();
  Matrix3 rho_x = so3::hat(rho);
  Matrix3 w_x = so3::hat(w);
  double a2 = angle * angle;
  double a3 = a2 * angle;
  double a4 = a3 * angle;
  double a5 = a4 * angle;
  // (angle - sin) / angle^3, (1 - angle^2/2 - cos) / angle^4 and
  // (angle - sin - angle^3/6) / angle^5, each by its series when small.
  double b = 0.0;
  double c = 0.0;
  double d = 0.0;
  if (angle < so3::SMALL_ANGLE) {
    b = 1.0 / 6.0 - a2 / 120.0;
    c = 1.0 / 24.0 - a2 / 720.0;
    d = 1.0 / 120.0 - a2 / 5040.0;
  } else {
    double sine = std::sin(angle);
    double cosine = std::cos(angle);
    b = (angle - sine) / a3;
    c = (1.0 - 0.5 * a2 - cosine) / a4;
    d = (angle - sine - a3 / 6.0) / a5;
  }
  Matrix3 w_rho = w_x * rho_x;
  Matrix3 rho_w = rho_x * w_x;
  Matrix3 w_rho_w = w_rho * w_x;
  return 0.5 * rho_x + b * (w_rho + rho_w + w_rho_w) -
         c * (w_x * w_rho + rho_w * w_x - 3.0 * w_rho_w) -
         0.5 * (c - 3.0 * d) * (w_rho_w * w_x + w_x * w_rho_w);
}

// The left Jacobian: exp(twist + delta) = exp(J_l(twist) delta) exp(twist),
// to first order.
inline auto left_jacobian(const Vector6& twist) -> Matrix6 {
  Matrix3 rotation = so3::left_jacobian(twist.head<3>());
  Matrix6 jacobian = Matrix6::Zero();
  jacobian.topLeftCorner<3, 3>() = rotation;
  jacobian.bottomLeftCorner<3, 3>() = coupling(twist);
  jacobian.bottomRightCorner<3, 3>() = rotation;
  return jacobian;
}

// The right Jacobian: exp(twist + delta) = exp(twist) exp(J_r(twist) delta),
// to first order.
inline auto right_jacobian(const Vector6& twist) -> Matrix6 {
  return left_jacobian(-twist);
}

inline auto inverse_left_jacobian(const Vector6& twist) -> Matrix6 {
  Matrix3 inverted = so3::inverse_left_jacobian(twist.head<3>());
  Matrix6 jacobian = Matrix6::Zero();
  jacobian.topLeftCorner<3, 3>() = inverted;
  jacobian.bottomLeftCorner<3, 3>() = -inverted * coupling(twist) * inverted;
  jacobian.bottomRightCorner<3, 3>() = inverted;
  return jacobian;
}

inline auto inverse_right_jacobian(const Vector6& twist) -> Matrix6 {
  return inverse_left_jacobian(-twist);
}

// `pose` moved by `twist` in its own frame: pose exp(twist).
inline auto plus(const Pose& pose, const Vector6& twist) -> Pose {
  return compose(pose, exp(twist));
}

// The twist, in `from`'s frame, that takes `from` to `to`:
// log(from^-1 to).
inline auto minus(const Pose& to, const Pose& from) -> Vector6 {
  return log(compose(inverse(from), to));
}

}  // namespace se3

}  // namespace simon
