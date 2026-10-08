// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/rigid_body.hpp"
#include "core/math.hpp"

#include <chrono>

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"
#include "framework/continuous.hpp"

namespace simon::model {

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using namespace std::chrono_literals;

namespace {

constexpr Duration DT = std::chrono::microseconds{8333};

// One classic Runge-Kutta step, as framework::Continuous takes it.
template <typename RateFunction>
auto runge_kutta(const RigidBody& body, RateFunction rate_of) -> RigidBody {
  RigidBodyRate k1 = rate_of(body);
  RigidBodyRate k2 = rate_of(advance(body, 0.5 * k1, DT));
  RigidBodyRate k3 = rate_of(advance(body, 0.5 * k2, DT));
  RigidBodyRate k4 = rate_of(advance(body, k3, DT));
  return advance(
      body,
      (1.0 / 6.0) * k1 + (1.0 / 3.0) * k2 + (1.0 / 3.0) * k3 + (1.0 / 6.0) * k4,
      DT);
}

auto zero_force() -> ForceVector { return QuantityVector{} * newton; }
auto zero_moment() -> Moment { return QuantityVector{} * newton_meter; }
auto no_gravity() -> Acceleration { return meters_per_second_squared(0, 0, 0); }

}  // namespace

static_assert(framework::ContinuousState<RigidBody>,
              "Continuous can integrate a rigid body.");

TEST_CASE("RigidBody") {
  SECTION("ShouldConserveAngularMomentumGivenNoTorque") {
    // Preconditions.
    // An asymmetric body spinning about no principal axis tumbles, but its
    // angular momentum is fixed in the inertial frame, and so is its energy.
    Matrix3 inertia = Vector3{1.0, 2.0, 3.0}.asDiagonal();
    MassProperties mass = compute_mass_properties(1.0 * kilogram, inertia);
    RigidBody body{.rate = QuantityVector{1.0, 0.1, 0.5} * radian_per_second};
    auto momentum = [&](const RigidBody& b) {
      return Vector3{
          b.attitude *
          (inertia * b.rate.numerical_value_in(radian_per_second).eigen())};
    };
    auto energy = [&](const RigidBody& b) {
      Vector3 w = b.rate.numerical_value_in(radian_per_second).eigen();
      return 0.5 * w.dot(inertia * w);
    };
    Vector3 start = momentum(body);
    double start_energy = energy(body);

    // Under Test.
    for (int i = 0; i < 120 * 60; ++i) {
      body = runge_kutta(body, [&](const RigidBody& b) {
        return compute_rigid_body_rate(b, zero_force(), zero_moment(), mass,
                                       no_gravity());
      });
    }

    // Postconditions.
    CHECK((momentum(body) - start).norm() < 1e-8);
    CHECK_THAT(energy(body), WithinRel(start_energy, 1e-9));
    CHECK_THAT(body.attitude.norm(), WithinAbs(1.0, 1e-15));
  }

  SECTION("ShouldFallFreelyGivenOnlyGravity") {
    // Preconditions.
    MassProperties mass =
        compute_mass_properties(10.0 * kilogram, Matrix3::Identity());
    RigidBody body;
    Acceleration gravity = meters_per_second_squared(0.0, 0.0, -9.8);

    // Under Test.
    for (int i = 0; i < 120; ++i) {
      body = runge_kutta(body, [&](const RigidBody& b) {
        return compute_rigid_body_rate(b, zero_force(), zero_moment(), mass,
                                       gravity);
      });
    }

    // Postconditions.
    // After 120 steps of 8.333 ms.
    double t = 120 * 0.008333;
    CHECK_THAT(body.position.numerical_value_in(meter).eigen().z(),
               WithinAbs(-0.5 * 9.8 * t * t, 1e-9));
  }

  SECTION("ShouldPushAlongBodyAxesGivenAttitude") {
    // Preconditions.
    // Nose pointing along the inertial y axis: a forward force pushes along y.
    MassProperties mass =
        compute_mass_properties(2.0 * kilogram, Matrix3::Identity());
    RigidBody body{.attitude = Quaternion{
                       AngleAxis{std::numbers::pi / 2.0, Vector3::UnitZ()}}};

    // Under Test.
    RigidBodyRate rate =
        compute_rigid_body_rate(body, QuantityVector{4.0, 0.0, 0.0} * newton,
                                zero_moment(), mass, no_gravity());

    // Postconditions.
    CHECK(rate.acceleration.numerical_value_in(meter_per_second_squared)
              .is_approximately(QuantityVector{0.0, 2.0, 0.0}));
  }
}

}  // namespace simon::model
