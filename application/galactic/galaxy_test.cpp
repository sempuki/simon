// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

#include "application/galactic/simulation.hpp"
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

const gravity::DiskGalaxy GALAXY = make_standard_galaxy(2000);

auto make_galaxy_scenario() -> Scenario {
  return make_standard_disk_scenario(GALAXY.disk_bodies);
}

// The disk's bodies, the first of the scenario's.
struct Disk final {
  std::vector<Vector3> positions;
  std::vector<Vector3> velocities;
};

auto collect_disk(const Simulation& simulation) -> Disk {
  Disk disk;
  for (std::size_t i = 0; i < GALAXY.disk_bodies; ++i) {
    const Kinematics& body =
        simulation.world().store_of<Kinematics>().component_of(
            simulation.bodies()[i]);
    disk.positions.push_back(body.position.numerical_value_in(meter).eigen());
    disk.velocities.push_back(
        body.velocity.numerical_value_in(meter_per_second).eigen());
  }
  return disk;
}

auto compute_half_mass_radius(const Disk& disk) -> double {
  std::vector<double> radii;
  for (const Vector3& p : disk.positions) radii.push_back(p.head<2>().norm());
  std::ranges::sort(radii);
  return radii[radii.size() / 2];
}

auto compute_thickness(const Disk& disk) -> double {
  double sum = 0.0;
  for (const Vector3& p : disk.positions) sum += p.z() * p.z();
  return std::sqrt(sum / disk.positions.size());
}

}  // namespace

TEST_CASE("DiskGalaxy") {
  const double h = GALAXY.disk_scale.numerical_value_in(meter);

  SECTION("ShouldStartAsDesignedGivenSample") {
    Simulation simulation{make_galaxy_scenario()};
    REQUIRE(simulation.configure());
    Disk disk = collect_disk(simulation);

    // Mean rotation and radial dispersion in a ring about 2.5 h.
    double rotation = 0.0;
    double radial2 = 0.0;
    int count = 0;
    for (std::size_t i = 0; i < disk.positions.size(); ++i) {
      Vector3 p = disk.positions[i];
      double r = p.head<2>().norm();
      if (r < 2.2 * h || r > 2.8 * h) continue;
      Vector3 out = Vector3{p.x(), p.y(), 0.0} / r;
      Vector3 around = Vector3{-out.y(), out.x(), 0.0};
      rotation += disk.velocities[i].dot(around);
      radial2 += std::pow(disk.velocities[i].dot(out), 2.0);
      ++count;
    }
    rotation /= count;
    double circular =
        gravity::compute_circular_speed(GALAXY, 2.5 * GALAXY.disk_scale)
            .numerical_value_in(meter_per_second);
    Mechanics mechanics =
        measure_mechanics(simulation.world(), simulation.scenario().softening);

    // Toomre's Q from the measured dispersion: sigma_R kappa / 3.36 G Sigma.
    double r = 2.5 * h;
    auto v2 = [&](double x) {
      return std::pow(gravity::compute_circular_speed(GALAXY, x * meter)
                          .numerical_value_in(meter_per_second),
                      2.0);
    };
    double kappa =
        std::sqrt((v2(r * 1.0001) - v2(r * 0.9999)) / (2e-4 * r) / r +
                  2.0 * v2(r) / (r * r));
    double sigma = GALAXY.disk_mass.numerical_value_in(kilogram) /
                   (2.0 * std::numbers::pi * h * h) * std::exp(-2.5);
    double q = std::sqrt(radial2 / count) * kappa /
               (3.36 * gravity::GRAVITATIONAL_CONSTANT * sigma);
    // Measured: 1.661 h to the profile's 1.678 h, 0.557 kpc to a sech^2
    // layer's z_0 pi / 2 sqrt(3) = 0.544, rotation at 0.972 of circular
    // (asymmetric drift), Q 1.56 over 249 bodies, and a virial ratio of 0.953.
    double half_mass = 1.678 * h;
    double layer = GALAXY.disk_thickness.numerical_value_in(meter) *
                   std::numbers::pi / (2.0 * std::sqrt(3.0));
    CHECK(std::abs(compute_half_mass_radius(disk) / half_mass - 1.0) < 0.03);
    CHECK(std::abs(compute_thickness(disk) / layer - 1.0) < 0.05);
    CHECK(rotation / circular > 0.9);
    CHECK(rotation / circular < 1.0);
    CHECK(std::abs(q - GALAXY.stability) < 0.2);
    CHECK(std::abs(mechanics.virial_ratio() - 1.0) < 0.1);
  }

  // 100 million years, about half a turn at 2.5 h, on the tree.
  SECTION("ShouldStayADiskGivenHundredMillionYears") {
    Simulation start{make_galaxy_scenario()};
    REQUIRE(start.configure());
    Disk before = collect_disk(start);
    Mechanics mechanics_before =
        measure_mechanics(start.world(), start.scenario().softening);

    Simulation simulation{make_galaxy_scenario()};
    const Year step{2000000};
    engine::BatchDriver driver{Timing{.max_step = step}, Depend(simulation)};
    REQUIRE(driver.run(BasicTimePoint<Year>{} + 50 * step));
    Disk after = collect_disk(simulation);
    Mechanics mechanics_after =
        measure_mechanics(simulation.world(), simulation.scenario().softening);

    // Measured: the half-mass radius from 1.661 h to 1.670, the thickness
    // from 0.557 kpc to 0.555, and the energy to 9.8e-4.
    CHECK(std::abs(compute_half_mass_radius(after) /
                       compute_half_mass_radius(before) -
                   1.0) < 0.03);
    CHECK(std::abs(compute_thickness(after) / compute_thickness(before) - 1.0) <
          0.05);
    CHECK(std::abs(mechanics_after.energy() / mechanics_before.energy() - 1.0) <
          2e-3);
  }
}

}  // namespace simon::galactic
