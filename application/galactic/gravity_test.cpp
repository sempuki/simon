// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <chrono>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <string>
#include <vector>

#include "application/galactic/simulation.hpp"
#include "application/testing.hpp"
#include "base/testing.hpp"
#include "core/argument.hpp"
#include "core/math.hpp"
#include "engine/driver.hpp"
#include "model/gravity/gravity.hpp"

namespace simon::galactic {
namespace {

using gravity::KILOPARSEC;
using gravity::SOLAR_MASS;

constexpr std::string_view FORCES =
    "application/galactic/reference/rebound_forces.csv";
constexpr std::string_view LEAPFROG =
    "application/galactic/reference/rebound_leapfrog.csv";

auto to_years(Time time) -> Year {
  return std::chrono::round<Year>(
      std::chrono::duration<double>(time.numerical_value_in(second)));
}

auto vector_of(const testing::Table& table,
               const std::vector<std::string>& line, std::string_view x)
    -> QuantityVector {
  std::size_t column = 0;
  while (table.header[column] != x) ++column;
  return {testing::parse_number(line[column]),
          testing::parse_number(line[column + 1]),
          testing::parse_number(line[column + 2])};
}

auto number_of(const testing::Table& table,
               const std::vector<std::string>& line, std::string_view name)
    -> double {
  std::size_t column = 0;
  while (table.header[column] != name) ++column;
  return testing::parse_number(line[column]);
}

// The bodies of one case of the forces table, and REBOUND's accelerations.
struct ForcesCase final {
  Scenario scenario;
  std::vector<QuantityVector> accelerations;
};

auto load_forces_case(std::string_view name) -> ForcesCase {
  testing::Table table = testing::load_table(FORCES);
  ForcesCase loaded;
  for (const std::vector<std::string>& line : table.lines) {
    if (line[0] != name) continue;
    loaded.scenario.softening = number_of(table, line, "softening") * meter;
    loaded.scenario.bodies.push_back(
        BodyStart{.position = vector_of(table, line, "x") * meter,
                  .velocity = vector_of(table, line, "vx") * meter_per_second,
                  .mass = number_of(table, line, "m") * kilogram});
    loaded.accelerations.push_back(vector_of(table, line, "ax"));
  }
  REQUIRE_FALSE(loaded.accelerations.empty());
  return loaded;
}

// Relative position and velocity on a Kepler orbit of semi-major axis `a`
// and eccentricity `e` under G M = `mu`, `time` after pericenter, from
// Kepler's equation solved by Newton's method (Murray and Dermott, chapter 2).
struct OrbitState final {
  Vector3 position;
  Vector3 velocity;
};

auto solve_kepler_orbit(double a, double e, double mu, double time)
    -> OrbitState {
  double n = std::sqrt(mu / (a * a * a));
  double mean = n * time;
  double eccentric = mean;
  for (int i = 0; i < 50; ++i) {
    eccentric -= (eccentric - e * std::sin(eccentric) - mean) /
                 (1.0 - e * std::cos(eccentric));
  }
  double b = std::sqrt(1.0 - e * e);
  double rate = n / (1.0 - e * std::cos(eccentric));
  return OrbitState{
      .position = Vector3{a * (std::cos(eccentric) - e),
                          a * b * std::sin(eccentric), 0.0},
      .velocity = Vector3{-a * rate * std::sin(eccentric),
                          a * b * rate * std::cos(eccentric), 0.0}};
}

// Two bodies on a Kepler orbit about their center of mass, at pericenter.
auto make_binary(double a, double e, double m1, double m2) -> Scenario {
  double mu = gravity::GRAVITATIONAL_CONSTANT * (m1 + m2);
  OrbitState relative = solve_kepler_orbit(a, e, mu, 0.0);
  double share1 = -m2 / (m1 + m2);
  double share2 = m1 / (m1 + m2);
  auto body = [&](double share, double mass) {
    return BodyStart{
        .position = QuantityVector{share * relative.position} * meter,
        .velocity =
            QuantityVector{share * relative.velocity} * meter_per_second,
        .mass = mass * kilogram};
  };
  return Scenario{.bodies = {body(share1, m1), body(share2, m2)}};
}

// The second body's position relative to the first's, and when.
struct Separation final {
  Vector3 position;
  double time = 0.0;  // Seconds.
};

// Runs `scenario` for `steps` of `dt`, rounded to whole years.
auto run_binary(const Scenario& scenario, Time dt, int steps) -> Separation {
  Simulation simulation{scenario};
  engine::BatchDriver driver{Timing{.max_step = to_years(dt)},
                             Depend(simulation)};
  auto reached = driver.run(BasicTimePoint<Year>{} + steps * to_years(dt));
  REQUIRE(reached);
  const auto& store = simulation.world().store_of<Kinematics>();
  return Separation{
      .position = (store.component_of(simulation.bodies()[1]).position -
                   store.component_of(simulation.bodies()[0]).position)
                      .numerical_value_in(meter)
                      .eigen(),
      .time = seconds(reached->time_since_epoch()).numerical_value_in(second)};
}

}  // namespace

TEST_CASE("Gravity") {
  for (std::string_view name : {"cluster", "unsoftened"}) {
    SECTION("ShouldMatchReboundGivenCase " + std::string{name}) {
      ForcesCase loaded = load_forces_case(name);
      Simulation simulation{loaded.scenario};
      REQUIRE(simulation.configure());
      REQUIRE(simulation.initialize());

      double worst = 0.0;
      for (std::size_t i = 0; i < loaded.accelerations.size(); ++i) {
        Vector3 ours =
            simulation.world()
                .store_of<Gravity>()
                .component_of(simulation.bodies()[i])
                .acceleration.numerical_value_in(meter_per_second_squared)
                .eigen();
        Vector3 theirs = loaded.accelerations[i].eigen();
        worst = std::max(worst, (ours - theirs).norm() / theirs.norm());
      }
      CHECK(worst == 0.0);  // REBOUND adds each pull in the same order.
    }
  }
}

TEST_CASE("Leapfrog") {
  SECTION("ShouldFollowReboundGivenCluster") {
    ForcesCase loaded = load_forces_case("cluster");
    Simulation simulation{loaded.scenario};
    engine::Driver driver{Timing{.max_step = Year{50000}}, Depend(simulation)};
    REQUIRE(driver.start());

    testing::Table table = testing::load_table(LEAPFROG);
    double worst_position = 0.0;
    double worst_velocity = 0.0;
    for (const std::vector<std::string>& line : table.lines) {
      auto step = static_cast<int>(number_of(table, line, "step"));
      auto body = static_cast<std::size_t>(number_of(table, line, "body"));
      REQUIRE(driver.advance_to(BasicTimePoint<Year>{} + step * Year{50000}));
      const Kinematics& ours =
          simulation.world().store_of<Kinematics>().component_of(
              simulation.bodies()[body]);
      Vector3 position = ours.position.numerical_value_in(meter).eigen();
      Vector3 velocity =
          ours.velocity.numerical_value_in(meter_per_second).eigen();
      worst_position =
          std::max(worst_position,
                   (position - vector_of(table, line, "x").eigen()).norm() /
                       KILOPARSEC.numerical_value_in(meter));
      worst_velocity = std::max(
          worst_velocity,
          (velocity - vector_of(table, line, "vx").eigen()).norm() / 100e3);
    }
    CHECK(worst_position == 0.0);  // Bit for bit, 400 steps.
    CHECK(worst_velocity == 0.0);
    REQUIRE(driver.finish());
  }

  // A 10 kpc orbit of eccentricity 0.5 about 10^11 solar masses, one period.
  const double a = 10.0 * KILOPARSEC.numerical_value_in(meter);
  const double e = 0.5;
  const double m1 = 1e11 * SOLAR_MASS.numerical_value_in(kilogram);
  const double m2 = 1e9 * SOLAR_MASS.numerical_value_in(kilogram);
  const double mu = gravity::GRAVITATIONAL_CONSTANT * (m1 + m2);
  const double period = 2.0 * std::numbers::pi * std::sqrt(a * a * a / mu);
  const Scenario binary = make_binary(a, e, m1, m2);

  SECTION("ShouldReturnToPericenterGivenKeplerOrbit") {
    int steps = 4000;
    Separation run = run_binary(binary, period / steps * second, steps);

    // 1.1 parsecs: the leapfrog's error at 4,000 steps an orbit.
    OrbitState exact = solve_kepler_orbit(a, e, mu, run.time);
    CHECK((run.position - exact.position).norm() / a < 1.2e-4);
  }

  SECTION("ShouldConvergeAtSecondOrderGivenKeplerOrbit") {
    auto error = [&](int steps) {
      Separation run = run_binary(binary, period / steps * second, steps);
      return (run.position - solve_kepler_orbit(a, e, mu, run.time).position)
          .norm();
    };
    double coarse = error(1000);
    double fine = error(2000);
    CHECK(std::abs(coarse / fine - 4.0) < 0.4);
  }
}

}  // namespace simon::galactic
