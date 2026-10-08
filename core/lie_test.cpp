// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "core/lie.hpp"

#include <cmath>
#include <numbers>

#include "base/testing.hpp"

namespace simon {
namespace {

constexpr double TIGHT = 1e-12;
constexpr double FINITE_DIFFERENCE = 1e-6;  // First order, so 1e-5 or so.

template <typename LeftType, typename RightType>
auto near(const Eigen::MatrixBase<LeftType>& a,
          const Eigen::MatrixBase<RightType>& b, double tolerance) -> bool {
  return (a - b).norm() <= tolerance;
}

const Vector3 W{0.3, -0.7, 0.5};
const Vector3 W_SMALL{1e-8, -2e-8, 3e-8};
// Rotation first.
const Vector6 TWIST{(Vector6{} << 0.3, -0.7, 0.5, 1.0, -2.0, 0.5).finished()};
const Vector6 TWIST_SMALL{
    (Vector6{} << 1e-8, -2e-8, 3e-8, 1.0, -2.0, 0.5).finished()};

TEST_CASE("SO3") {
  SECTION("ShouldRoundTripGivenExpAndLog") {
    // Postconditions.
    CHECK(near(so3::log(so3::exp(W)), W, TIGHT));
    CHECK(near(so3::log(so3::exp(W_SMALL)), W_SMALL, TIGHT));
    CHECK(near(so3::log(Quaternion::Identity()), Vector3::Zero(), TIGHT));
  }

  SECTION("ShouldTakeShorterWayGivenNegatedQuaternion") {
    // Preconditions.
    Quaternion q = so3::exp(W);
    Quaternion negated{-q.w(), -q.x(), -q.y(), -q.z()};

    // Postconditions.
    CHECK(near(so3::log(negated), W, TIGHT));
  }

  SECTION("ShouldRotateAsRodriguesGivenAxisAndAngle") {
    // Preconditions.
    Vector3 axis = Vector3{1.0, 1.0, 0.0}.normalized();
    double angle = std::numbers::pi / 3.0;
    Vector3 point{1.0, 0.0, 0.0};
    Vector3 expected = std::cos(angle) * point +
                       std::sin(angle) * axis.cross(point) +
                       (1.0 - std::cos(angle)) * axis.dot(point) * axis;

    // Postconditions.
    CHECK(near(so3::exp(angle * axis) * point, expected, TIGHT));
  }

  SECTION("ShouldMatchHatAndVee") {
    // Preconditions.
    Vector3 v{0.1, 0.2, 0.3};

    // Postconditions.
    CHECK(near(so3::vee(so3::hat(W)), W, TIGHT));
    CHECK(near(so3::hat(W) * v, W.cross(v), TIGHT));
  }

  SECTION("ShouldRelateChangeInAlgebraToChangeOnGroupGivenJacobians") {
    // Postconditions.
    for (const Vector3& w : {W, W_SMALL}) {
      Vector3 delta = FINITE_DIFFERENCE * Vector3{0.4, 0.2, -0.9};
      Quaternion moved = so3::exp(w + delta);
      // Right: exp(w + delta) = exp(w) exp(J_r delta).
      Vector3 right = so3::log(so3::exp(w).conjugate() * moved);
      CHECK(near(right, so3::right_jacobian(w) * delta, 1e-10));
      // Left: exp(w + delta) = exp(J_l delta) exp(w).
      Vector3 left = so3::log(moved * so3::exp(w).conjugate());
      CHECK(near(left, so3::left_jacobian(w) * delta, 1e-10));
    }
  }

  SECTION("ShouldInvertJacobians") {
    // Postconditions.
    for (const Vector3& w : {W, W_SMALL}) {
      CHECK(near(so3::left_jacobian(w) * so3::inverse_left_jacobian(w),
                 Matrix3::Identity(), TIGHT));
      CHECK(near(so3::right_jacobian(w) * so3::inverse_right_jacobian(w),
                 Matrix3::Identity(), TIGHT));
    }
  }
}

TEST_CASE("SE3") {
  SECTION("ShouldRoundTripGivenExpAndLog") {
    // Postconditions.
    CHECK(near(se3::log(se3::exp(TWIST)), TWIST, TIGHT));
    CHECK(near(se3::log(se3::exp(TWIST_SMALL)), TWIST_SMALL, TIGHT));
  }

  SECTION("ShouldMatchHatAndVee") {
    // Postconditions.
    CHECK(near(se3::vee(se3::hat(TWIST)), TWIST, TIGHT));
  }

  SECTION("ShouldComposeAndInvert") {
    // Preconditions.
    se3::Pose a = se3::exp(TWIST);
    se3::Pose b = se3::exp(0.5 * TWIST_SMALL + Vector6::Ones());
    Vector3 point{0.2, -0.4, 0.6};

    // Postconditions.
    CHECK(near(se3::act(se3::compose(a, b), point),
               se3::act(a, se3::act(b, point)), TIGHT));
    CHECK(
        near(se3::act(se3::compose(a, se3::inverse(a)), point), point, TIGHT));
  }

  SECTION("ShouldMatchExpAsMatrixExponentialGivenPureTranslation") {
    // Preconditions.
    Vector6 translation =
        (Vector6{} << 0.0, 0.0, 0.0, 1.0, 2.0, 3.0).finished();

    // Under Test.
    se3::Pose pose = se3::exp(translation);

    // Postconditions.
    CHECK(near(pose.translation, Vector3{1.0, 2.0, 3.0}, TIGHT));
    CHECK(near(so3::log(pose.rotation), Vector3::Zero(), TIGHT));
  }

  SECTION("ShouldMoveTwistBetweenFramesGivenAdjoint") {
    // Preconditions.
    se3::Pose x = se3::exp(TWIST);
    Vector6 twist = (Vector6{} << 0.2, 0.1, -0.3, 0.05, 0.1, -0.02).finished();

    // Under Test.
    se3::Pose left = se3::exp(se3::adjoint(x) * twist);
    se3::Pose right =
        se3::compose(se3::compose(x, se3::exp(twist)), se3::inverse(x));

    // Postconditions.
    CHECK(near(se3::log(left), se3::log(right), 1e-10));
  }

  SECTION("ShouldRelateChangeInAlgebraToChangeOnGroupGivenJacobians") {
    // Postconditions.
    for (const Vector6& twist : {TWIST, TWIST_SMALL}) {
      Vector6 delta = FINITE_DIFFERENCE *
                      (Vector6{} << 0.4, 0.2, -0.9, 0.3, -0.6, 0.8).finished();
      se3::Pose moved = se3::exp(twist + delta);
      Vector6 right = se3::minus(moved, se3::exp(twist));
      CHECK(near(right, se3::right_jacobian(twist) * delta, 1e-10));
      Vector6 left =
          se3::log(se3::compose(moved, se3::inverse(se3::exp(twist))));
      CHECK(near(left, se3::left_jacobian(twist) * delta, 1e-10));
    }
  }

  SECTION("ShouldInvertJacobians") {
    // Postconditions.
    for (const Vector6& twist : {TWIST, TWIST_SMALL}) {
      CHECK(near(se3::left_jacobian(twist) * se3::inverse_left_jacobian(twist),
                 Matrix6::Identity(), TIGHT));
      CHECK(
          near(se3::right_jacobian(twist) * se3::inverse_right_jacobian(twist),
               Matrix6::Identity(), TIGHT));
    }
  }

  SECTION("ShouldBracketAsMatricesCommuteGivenCrossMotion") {
    // Preconditions.
    Vector6 a = TWIST;
    Vector6 b = (Vector6{} << 0.2, 0.1, -0.3, 0.05, 0.1, -0.02).finished();

    // Postconditions.
    // hat([a, b]) = hat(a) hat(b) - hat(b) hat(a).
    CHECK(near(se3::hat(se3::cross_motion(a, b)),
               se3::hat(a) * se3::hat(b) - se3::hat(b) * se3::hat(a), TIGHT));
  }

  SECTION("ShouldBeDualGivenCrossForce") {
    // Preconditions.
    Vector6 a = TWIST;
    Vector6 m = (Vector6{} << 0.2, 0.1, -0.3, 0.05, 0.1, -0.02).finished();
    Vector6 f = (Vector6{} << -1.0, 0.5, 2.0, 0.3, -0.4, 0.7).finished();

    // Postconditions.
    // A wrench's power on a motion is the same seen from a moving frame:
    // (a x m) . f = -m . (a x* f).
    CHECK(std::abs(se3::cross_motion(a, m).dot(f) +
                   m.dot(se3::cross_force(a, f))) < TIGHT);
  }

  SECTION("ShouldUndoPlusGivenMinus") {
    // Preconditions.
    se3::Pose pose = se3::exp(TWIST);
    Vector6 twist = (Vector6{} << 0.2, 0.1, -0.3, 0.05, 0.1, -0.02).finished();

    // Postconditions.
    CHECK(near(se3::minus(se3::plus(pose, twist), pose), twist, TIGHT));
  }
}

}  // namespace
}  // namespace simon
