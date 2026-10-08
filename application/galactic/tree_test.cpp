// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <format>
#include <string>
#include <vector>

#include "application/galactic/simulation.hpp"
#include "application/testing.hpp"
#include "base/testing.hpp"
#include "core/argument.hpp"
#include "core/math.hpp"
#include "core/random.hpp"
#include "engine/driver.hpp"
#include "model/gravity/galaxy.hpp"
#include "model/gravity/gravity.hpp"

namespace simon::galactic {
namespace {

using gravity::KILOPARSEC;
using gravity::SOLAR_MASS;

constexpr std::string_view TREE =
    "application/galactic/reference/rebound_tree.csv";
constexpr std::array<double, 4> ANGLES{0.25, 0.5, 0.75, 1.0};

auto find_column(const testing::Table& table, std::string_view name)
    -> std::size_t {
  auto found = std::ranges::find(table.header, name);
  REQUIRE(found != table.header.end());
  return static_cast<std::size_t>(found - table.header.begin());
}

auto read_vector(const testing::Table& table,
                 const std::vector<std::string>& line, std::string_view x)
    -> Vector3 {
  std::size_t column = find_column(table, x);
  return Vector3{testing::parse_number(line[column]),
                 testing::parse_number(line[column + 1]),
                 testing::parse_number(line[column + 2])};
}

// The median and 99th percentile of `errors`.
struct Spread final {
  double median = 0.0;
  double worst = 0.0;  // The 99th percentile.
};

auto compute_spread(std::vector<double> errors) -> Spread {
  std::ranges::sort(errors);
  auto at = [&](double fraction) {
    return errors[static_cast<std::size_t>(fraction * (errors.size() - 1))];
  };
  return Spread{.median = at(0.5), .worst = at(0.99)};
}

// Every body's gravity in `scenario`, in its order.
auto compute_accelerations(const Scenario& scenario) -> std::vector<Vector3> {
  Simulation simulation{scenario};
  REQUIRE(simulation.configure());
  REQUIRE(simulation.initialize());
  std::vector<Vector3> accelerations;
  for (Entity body : simulation.bodies()) {
    accelerations.push_back(
        simulation.world()
            .store_of<Gravity>()
            .component_of(body)
            .acceleration.numerical_value_in(meter_per_second_squared)
            .eigen());
  }
  return accelerations;
}

}  // namespace

TEST_CASE("Tree") {
  testing::Table table = testing::load_table(TREE);
  Scenario scenario{
      .softening = testing::parse_number(
                       table.lines.front()[find_column(table, "softening")]) *
                   meter};
  std::vector<Vector3> direct;
  for (const std::vector<std::string>& line : table.lines) {
    scenario.bodies.push_back(gravity::BodyStart{
        .position = QuantityVector{read_vector(table, line, "x")} * meter,
        .mass =
            testing::parse_number(line[find_column(table, "m")]) * kilogram});
    direct.push_back(read_vector(table, line, "ax"));
  }

  auto errors_against_direct = [&](const std::vector<Vector3>& tree) {
    std::vector<double> errors;
    for (std::size_t i = 0; i < direct.size(); ++i) {
      errors.push_back((tree[i] - direct[i]).norm() / direct[i].norm());
    }
    return errors;
  };

  SECTION("ShouldEqualDirectSumGivenNoCellsUnopened") {
    // Preconditions.
    scenario.gravity = GravityMethod::TREE;
    scenario.opening_angle = 0.0;

    // Under Test.
    Spread spread =
        compute_spread(errors_against_direct(compute_accelerations(scenario)));

    // Postconditions.
    CHECK(spread.worst < 1e-13);  // Measured: 4e-15, the sums' order.
  }

  // Measured at 0.5: a median error of 0.29% to REBOUND's 0.25%, and a 99th
  // percentile of 1.6% to its 1.7%. The two trees' cells differ, since
  // REBOUND's root is a cube about the origin and simon's bounds the bodies.
  SECTION("ShouldErrLikeReboundGivenOpeningAngles") {
    // Preconditions.
    scenario.gravity = GravityMethod::TREE;
    Spread last;

    // Under Test.
    for (double angle : ANGLES) {
      scenario.opening_angle = angle;
      Spread ours = compute_spread(
          errors_against_direct(compute_accelerations(scenario)));

      std::vector<Vector3> rebound;
      for (const std::vector<std::string>& line : table.lines) {
        rebound.push_back(
            read_vector(table, line, std::format("tree_{}_x", angle)));
      }
      Spread theirs = compute_spread(errors_against_direct(rebound));

      INFO(std::format("angle {}: ours {} {}, REBOUND {} {}", angle,
                       ours.median, ours.worst, theirs.median, theirs.worst));
      CHECK(ours.median < 1.3 * theirs.median);
      CHECK(ours.worst < 1.3 * theirs.worst);
      CHECK(ours.median > last.median);
      last = ours;
    }
  }
}

// 256 bodies of a Plummer sphere on the tree at an opening angle of 0.5, for
// ten crossing times of 128 steps.
TEST_CASE("TreeMomentum") {
  // Preconditions.
  const gravity::Plummer plummer{.mass = 1e10 * SOLAR_MASS,
                                 .scale = KILOPARSEC};
  Scenario scenario{.softening = 0.05 * KILOPARSEC,
                    .gravity = GravityMethod::TREE};
  Random random{7};
  gravity::append_plummer(plummer, 256, InOut(random), InOut(scenario.bodies));
  Time crossing = gravity::compute_crossing_time(
      plummer.mass, gravity::compute_plummer_energy(plummer));
  Year step = std::chrono::round<Year>(std::chrono::duration<double>(
      crossing.numerical_value_in(second) / 128.0));
  Simulation start{scenario};
  REQUIRE(start.configure());
  Mechanics before = measure_mechanics(start.world(), scenario.softening);
  Simulation simulation{scenario};
  engine::BatchDriver driver{Timing{.max_step = step}, Depend(simulation)};

  // Under Test.
  REQUIRE(driver.run(BasicTimePoint<Year>{} + 1280 * step));

  // Postconditions.
  Mechanics after = measure_mechanics(simulation.world(), scenario.softening);
  double scale_momentum = plummer.mass.numerical_value_in(kilogram) *
                          std::sqrt(gravity::GRAVITATIONAL_CONSTANT *
                                    plummer.mass.numerical_value_in(kilogram) /
                                    plummer.scale.numerical_value_in(meter));
  // Measured: energy to 9.4e-4 and momentum to 9.3e-4, where direct
  // summation keeps them to 1.5e-4 and rounding.
  CHECK(std::abs(after.energy() / before.energy() - 1.0) < 2e-3);
  CHECK((after.momentum - before.momentum).norm() / scale_momentum < 2e-3);
}

}  // namespace simon::galactic
