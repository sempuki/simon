// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <cmath>
#include <cstddef>
#include <numbers>
#include <string>
#include <vector>

#include "application/galactic/simulation.hpp"
#include "application/testing.hpp"
#include "base/testing.hpp"
#include "engine/driver.hpp"
#include "framework/vocabulary.hpp"
#include "model/galaxy.hpp"
#include "model/gravity.hpp"

namespace simon::galactic {
namespace {

using model::KILOPARSEC;
using model::SOLAR_MASS;

constexpr std::string_view ENCOUNTER =
    "application/galactic/reference/rebound_encounter.csv";

// Toomre and Toomre's units: R_min = 25 kpc, 10^11 suns, 10^8 years.
const Encounter TOOMRE{.victim = 1e11 * SOLAR_MASS,
                       .companion = 1e11 * SOLAR_MASS,
                       .pericenter = 25.0 * KILOPARSEC,
                       .before = 1e9 * model::JULIAN_YEAR,
                       .softening = 0.1 * KILOPARSEC};
const Year STEP{250000};

auto read_state(const std::vector<std::string>& line) -> model::BodyStart {
  auto number = [&](std::size_t column) {
    return testing::parse_number(line[column]);
  };
  return model::BodyStart{
      .position = model::meters(number(2), number(3), number(4)),
      .velocity = model::meters_per_second(number(5), number(6), number(7))};
}

// Every body's state in `simulation`: the masses, then the test particles.
auto collect_states(const Simulation& simulation)
    -> std::vector<model::BodyStart> {
  std::vector<model::BodyStart> states;
  auto append = [&](const std::vector<Entity>& entities) {
    for (Entity entity : entities) {
      const Kinematics& kinematics =
          simulation.world().store_of<Kinematics>().component_of(entity);
      states.push_back(model::BodyStart{.position = kinematics.position,
                                        .velocity = kinematics.velocity});
    }
  };
  append(simulation.bodies());
  append(simulation.test_particles());
  return states;
}

}  // namespace

TEST_CASE("Encounter") {
  SECTION("ShouldTakeToomresPeriodGivenOuterRing") {
    // Toomre and Toomre's outermost ring, 15 kpc about 10^11 suns, turns in
    // 5.442 of their units of 10^8 years.
    double gm = model::GRAVITATIONAL_CONSTANT *
                TOOMRE.victim.numerical_value_in(model::kilogram);
    double r = (0.6 * TOOMRE.pericenter).numerical_value_in(model::meter);
    double period = 2.0 * std::numbers::pi * std::sqrt(r * r * r / gm);
    CHECK(std::abs(period / (5.442e8 * model::JULIAN_YEAR.numerical_value_in(
                                           model::second)) -
                   1.0) < 1e-4);
  }

  SECTION("ShouldStartAsReboundGivenDirectPassage") {
    Simulation simulation{make_encounter_scenario(TOOMRE)};
    REQUIRE(simulation.configure());
    std::vector<model::BodyStart> ours = collect_states(simulation);

    testing::Table table = testing::load_table(ENCOUNTER);
    std::size_t compared = 0;
    for (const std::vector<std::string>& line : table.lines) {
      if (line[0] != "0") continue;
      auto body = static_cast<std::size_t>(testing::parse_number(line[1]));
      model::BodyStart theirs = read_state(line);
      CHECK(ours[body].position == theirs.position);  // Bit for bit.
      CHECK(ours[body].velocity == theirs.velocity);
      ++compared;
    }
    CHECK(compared == 122);
  }

  SECTION("ShouldFollowReboundGivenDirectPassage") {
    Simulation simulation{make_encounter_scenario(TOOMRE)};
    engine::Driver driver{Timing{.max_step = STEP}, Depend(simulation)};
    REQUIRE(driver.start());

    testing::Table table = testing::load_table(ENCOUNTER);
    std::size_t differing = 0;
    for (const std::vector<std::string>& line : table.lines) {
      auto step = static_cast<int>(testing::parse_number(line[0]));
      auto body = static_cast<std::size_t>(testing::parse_number(line[1]));
      REQUIRE(
          driver.advance_to(framework::BasicTimePoint<Year>{} + step * STEP));
      model::BodyStart ours = collect_states(simulation)[body];
      model::BodyStart theirs = read_state(line);
      if (ours.position != theirs.position ||
          ours.velocity != theirs.velocity) {
        ++differing;
      }
    }
    CHECK(differing == 0);  // Bit for bit, 8,000 steps.
    REQUIRE(driver.finish());
  }

  // Toomre and Toomre's companion captures 28 of the victim's particles. A
  // particle belongs to the mass it is more tightly bound to, at t = +10.
  // Captured particles plunge within 1 kpc of the companion, so the count
  // needs a short step: 22 at 250,000 years, 27 at 15,625 and 7,812, and 28
  // with every ring turned half a spacing, since their rings' phase is not
  // given.
  SECTION("ShouldCaptureAsToomresGivenDirectPassage") {
    Encounter converged = TOOMRE;
    converged.softening = 0.02 * KILOPARSEC;
    const Year step{15625};
    Simulation simulation{make_encounter_scenario(converged)};
    engine::BatchDriver driver{Timing{.max_step = step}, Depend(simulation)};
    REQUIRE(driver.run(framework::BasicTimePoint<Year>{} + 128000 * step));

    std::vector<model::BodyStart> states = collect_states(simulation);
    double gm = model::GRAVITATIONAL_CONSTANT *
                converged.companion.numerical_value_in(model::kilogram);
    double e = converged.softening.numerical_value_in(model::meter);
    auto energy_about = [&](const model::BodyStart& particle,
                            const model::BodyStart& mass) {
      Vector3 d = (particle.position - mass.position)
                      .numerical_value_in(model::meter)
                      .eigen();
      Vector3 v = (particle.velocity - mass.velocity)
                      .numerical_value_in(model::meter_per_second)
                      .eigen();
      return 0.5 * v.squaredNorm() - gm / std::sqrt(d.squaredNorm() + e * e);
    };
    int captured = 0;
    for (std::size_t i = 2; i < states.size(); ++i) {
      double to_victim = energy_about(states[i], states[0]);
      double to_companion = energy_about(states[i], states[1]);
      if (to_companion < 0.0 && to_companion < to_victim) ++captured;
    }
    CHECK(captured == 27);
    CHECK(std::abs(captured - 28) <= 2);
  }
}

}  // namespace simon::galactic
