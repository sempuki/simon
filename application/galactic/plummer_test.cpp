// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <vector>

#include "application/galactic/simulation.hpp"
#include "base/testing.hpp"
#include "core/random.hpp"
#include "core/vocabulary.hpp"
#include "engine/driver.hpp"
#include "model/gravity/galaxy.hpp"
#include "model/gravity/gravity.hpp"

namespace simon::galactic {
namespace {

using model::KILOPARSEC;
using model::SOLAR_MASS;

// A sphere of 10^10 solar masses with a scale radius of 1 kpc.
const model::Plummer PLUMMER{.mass = 1e10 * SOLAR_MASS, .scale = KILOPARSEC};

auto make_plummer_scenario(std::size_t bodies, double softening) -> Scenario {
  Scenario scenario{.softening = softening * PLUMMER.scale};
  Random random{7};
  model::append_plummer(PLUMMER, bodies, InOut(random), InOut(scenario.bodies));
  return scenario;
}

}  // namespace

TEST_CASE("Plummer") {
  const std::array<double, 5> fractions{0.1, 0.25, 0.5, 0.75, 0.9};

  SECTION("ShouldFollowPlummersProfileGivenSample") {
    Simulation simulation{make_plummer_scenario(4000, 0.0)};
    REQUIRE(simulation.configure());

    std::vector<Length> radii =
        compute_mass_radii(simulation.world(), fractions);
    for (std::size_t i = 0; i < fractions.size(); ++i) {
      double expected = number_of(
          radii[i] / model::compute_plummer_radius(PLUMMER, fractions[i]));
      CHECK(std::abs(expected - 1.0) < 0.05);
    }
  }

  SECTION("ShouldStartInEquilibriumGivenSample") {
    Simulation simulation{make_plummer_scenario(4000, 0.0)};
    REQUIRE(simulation.configure());

    Mechanics mechanics =
        measure_mechanics(simulation.world(), simulation.scenario().softening);
    CHECK(std::abs(mechanics.virial_ratio() - 1.0) < 0.05);
    CHECK(
        std::abs(mechanics.energy() /
                     model::compute_plummer_energy(PLUMMER).numerical_value_in(
                         units::si::joule) -
                 1.0) < 0.05);
    double scale_momentum = PLUMMER.mass.numerical_value_in(kilogram) * 100e3;
    CHECK(mechanics.momentum.norm() / scale_momentum < 1e-15);
  }

  // 256 bodies, softened by 0.05 a, for ten crossing times of 128 steps.
  SECTION("ShouldStayInEquilibriumGivenTenCrossingTimes") {
    Scenario scenario = make_plummer_scenario(256, 0.05);
    Time crossing = model::compute_crossing_time(
        PLUMMER.mass, model::compute_plummer_energy(PLUMMER));
    Year step = std::chrono::round<Year>(std::chrono::duration<double>(
        crossing.numerical_value_in(second) / 128.0));

    Simulation simulation{scenario};
    engine::BatchDriver driver{Timing{.max_step = step}, Depend(simulation)};
    Simulation start{scenario};
    REQUIRE(start.configure());
    Mechanics before = measure_mechanics(start.world(), scenario.softening);
    std::vector<Length> radii_before =
        compute_mass_radii(start.world(), fractions);

    REQUIRE(driver.run(BasicTimePoint<Year>{} + 1280 * step));

    Mechanics after = measure_mechanics(simulation.world(), scenario.softening);
    std::vector<Length> radii_after =
        compute_mass_radii(simulation.world(), fractions);
    double scale_momentum =
        PLUMMER.mass.numerical_value_in(kilogram) *
        std::sqrt(model::GRAVITATIONAL_CONSTANT *
                  PLUMMER.mass.numerical_value_in(kilogram) /
                  PLUMMER.scale.numerical_value_in(meter));
    double scale_angular =
        scale_momentum * PLUMMER.scale.numerical_value_in(meter);

    // Measured: energy to 1.5e-4, momentum and angular momentum to rounding,
    // the virial ratio at 1.036 and the half-mass radius within 2.5%.
    CHECK(std::abs(after.energy() / before.energy() - 1.0) < 3e-4);
    CHECK((after.momentum - before.momentum).norm() / scale_momentum < 1e-14);
    CHECK((after.angular_momentum - before.angular_momentum).norm() /
              scale_angular <
          1e-14);
    CHECK(std::abs(after.virial_ratio() - 1.0) < 0.1);
    CHECK(std::abs(number_of(radii_after[2] / radii_before[2]) - 1.0) < 0.05);
  }
}

}  // namespace simon::galactic
