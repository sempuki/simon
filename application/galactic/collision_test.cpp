// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

#include "application/galactic/simulation.hpp"
#include "base/testing.hpp"
#include "core/math.hpp"
#include "model/gravity/galaxy.hpp"
#include "model/gravity/gravity.hpp"

namespace simon::galactic {
namespace {

// A galaxy's bodies' total mass, center of mass, momentum per mass and spin.
struct Measure final {
  double mass = 0.0;
  Vector3 center = Vector3::Zero();
  Vector3 velocity = Vector3::Zero();
  Vector3 spin = Vector3::Zero();  // Angular momentum about the center.
};

auto measure_group(const Scenario& scenario, const BodyGroup& group)
    -> Measure {
  Measure measure;
  Vector3 moment = Vector3::Zero();
  Vector3 momentum = Vector3::Zero();
  for (std::size_t i = group.first; i < group.first + group.count; ++i) {
    const BodyStart& body = scenario.bodies[i];
    double m = body.mass.numerical_value_in(kilogram);
    measure.mass += m;
    moment += m * body.position.numerical_value_in(meter).eigen();
    momentum += m * body.velocity.numerical_value_in(meter_per_second).eigen();
  }
  measure.center = moment / measure.mass;
  measure.velocity = momentum / measure.mass;
  for (std::size_t i = group.first; i < group.first + group.count; ++i) {
    const BodyStart& body = scenario.bodies[i];
    measure.spin +=
        body.mass.numerical_value_in(kilogram) *
        (body.position.numerical_value_in(meter).eigen() - measure.center)
            .cross(body.velocity.numerical_value_in(meter_per_second).eigen() -
                   measure.velocity);
  }
  return measure;
}

}  // namespace

TEST_CASE("Collision") {
  Collision collision = make_standard_collision(500);
  Scenario scenario = make_collision_scenario(collision);
  REQUIRE(scenario.groups.size() == 4);
  REQUIRE(scenario.bodies.size() == 2 * (500 + 2000));

  SECTION("ShouldPlaceGalaxiesOnTheirOrbitGivenStandardCollision") {
    const gravity::DiskGalaxy& galaxy = collision.galaxy;
    double cut = number_of(galaxy.halo_cutoff /
                           (galaxy.halo_cutoff + galaxy.halo_scale));
    Mass mass = galaxy.disk_mass + galaxy.halo_mass * cut * cut;
    gravity::Separation separation = gravity::compute_parabolic_separation(
        gravity::ParabolicOrbit{
            .first = mass, .second = mass, .pericenter = collision.pericenter},
        -collision.before);
    Vector3 expected = separation.position.numerical_value_in(meter).eigen();
    Vector3 expected_velocity =
        separation.velocity.numerical_value_in(meter_per_second).eigen();

    // Each galaxy's disk and halo are centered on its place on the orbit.
    for (std::size_t g = 0; g < 4; ++g) {
      Measure measure = measure_group(scenario, scenario.groups[g]);
      double share = g < 2 ? -0.5 : 0.5;
      CHECK((measure.center - share * expected).norm() / expected.norm() <
            1e-12);
      CHECK((measure.velocity - share * expected_velocity).norm() /
                expected_velocity.norm() <
            1e-12);
    }
  }

  SECTION("ShouldTiltSecondDiskGivenInclination") {
    Vector3 first = measure_group(scenario, scenario.groups[0]).spin;
    Vector3 second = measure_group(scenario, scenario.groups[2]).spin;
    double angle = std::acos(first.normalized().dot(second.normalized()));
    // The first disk turns about z, as the orbit does; the second is tilted
    // 45 degrees about x, toward -y.
    CHECK(first.normalized().z() > 0.999);
    CHECK(std::abs(angle - std::numbers::pi / 4.0) < 0.02);
    CHECK(second.y() < 0.0);
  }

  SECTION("ShouldStartFarApartGivenStandardCollision") {
    Simulation simulation{scenario};
    REQUIRE(simulation.configure());
    double separation =
        (compute_group_center(simulation.world(), simulation.bodies(),
                              scenario.groups[0], 10.0 * gravity::KILOPARSEC) -
         compute_group_center(simulation.world(), simulation.bodies(),
                              scenario.groups[2], 10.0 * gravity::KILOPARSEC))
            .numerical_value_in(meter)
            .eigen()
            .norm() /
        gravity::KILOPARSEC.numerical_value_in(meter);
    CHECK(separation > 170.0);
    CHECK(separation < 180.0);
  }
}

}  // namespace simon::galactic
