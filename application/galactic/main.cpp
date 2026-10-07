// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Runs a galactic scenario as fast as possible and prints how it goes:
//
//   bazel run -c opt //application/galactic -- collision [disk bodies]
//       [million years] [--step=YEARS] [--start=PATH] [--track=PATH]
//   bazel run -c opt //application/galactic -- disk [disk bodies]
//       [million years]
//
// A collision prints, every 50 million years, the energy's drift, the
// distance between the galaxies' centers, and the share of their disks more
// than 30 kpc from their own galaxy's center, in tails and bridges. Steps
// are a million years unless --step gives them. --start writes the bodies'
// starting state, and --track the separation every 10 million years, both as
// CSV in SI units, for application/galactic/reference/rebound_collision.py.
// A disk galaxy alone prints its disk's half-mass radius and thickness.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <print>
#include <string>
#include <string_view>
#include <vector>

#include "application/galactic/simulation.hpp"
#include "core/argument.hpp"
#include "core/math.hpp"
#include "engine/driver.hpp"
#include "model/gravity/gravity.hpp"

namespace simon::galactic {
namespace {

using model::KILOPARSEC;

const double KPC = KILOPARSEC.numerical_value_in(meter);

auto read_position(const Simulation& simulation, std::size_t body) -> Vector3 {
  return simulation.world()
      .store_of<Kinematics>()
      .component_of(simulation.bodies()[body])
      .position.numerical_value_in(meter)
      .eigen();
}

auto write_start(const Scenario& scenario, const std::string& path) -> bool {
  std::ofstream out{path};
  if (!out) return false;
  out << "group,x,y,z,vx,vy,vz,m\n";
  for (std::size_t g = 0; g < scenario.groups.size(); ++g) {
    const BodyGroup& group = scenario.groups[g];
    for (std::size_t i = group.first; i < group.first + group.count; ++i) {
      const BodyStart& body = scenario.bodies[i];
      Vector3 p = body.position.numerical_value_in(meter).eigen();
      Vector3 v = body.velocity.numerical_value_in(meter_per_second).eigen();
      std::println(out,
                   "{},{:.17g},{:.17g},{:.17g},{:.17g},{:.17g},{:.17g},{:.17g}",
                   g, p.x(), p.y(), p.z(), v.x(), v.y(), v.z(),
                   body.mass.numerical_value_in(kilogram));
    }
  }
  return true;
}

auto run_collision(std::size_t disk_bodies, int million_years, Year step,
                   std::string_view start_path, std::string_view track_path)
    -> int {
  Collision collision = make_standard_collision(disk_bodies);
  Scenario scenario = make_collision_scenario(collision);
  if (!start_path.empty() && !write_start(scenario, std::string{start_path})) {
    std::println(stderr, "Error: cannot write {}", start_path);
    return EXIT_FAILURE;
  }
  std::ofstream track;
  if (!track_path.empty()) {
    track.open(std::string{track_path});
    track << "million_years,separation_kpc\n";
  }

  Simulation simulation{scenario};
  engine::Driver driver{Timing{.max_step = step}, Depend(simulation)};
  if (auto started = driver.start(); !started) {
    std::println(stderr, "Error: {}", started.error().message());
    return EXIT_FAILURE;
  }
  Mechanics start = measure_mechanics(simulation.world(), scenario.softening);
  std::println("{:>6} {:>12} {:>15} {:>8} {:>10}", "Myr", "energy",
               "separation kpc", "tails", "wall s");

  auto wall_start = std::chrono::steady_clock::now();
  for (int myr = 0; myr <= million_years; myr += 10) {
    if (auto reached = driver.advance_to(BasicTimePoint<Year>{} +
                                         Year{std::int64_t{myr} * 1000000});
        !reached) {
      std::println(stderr, "Error: {}", reached.error().message());
      return EXIT_FAILURE;
    }
    const World& world = simulation.world();
    std::array<Vector3, 2> centers;
    for (std::size_t galaxy : {0u, 1u}) {
      centers[galaxy] =
          compute_group_center(world, simulation.bodies(),
                               scenario.groups[2 * galaxy], 10.0 * KILOPARSEC)
              .numerical_value_in(meter)
              .eigen();
    }
    double separation = (centers[0] - centers[1]).norm() / KPC;
    if (track.is_open()) std::println(track, "{},{:.9g}", myr, separation);
    if (myr % 50 != 0) continue;

    Mechanics now = measure_mechanics(world, scenario.softening);
    std::size_t far = 0;
    std::size_t disk = 0;
    for (std::size_t galaxy : {0u, 1u}) {
      const BodyGroup& group = scenario.groups[2 * galaxy];
      for (std::size_t i = group.first; i < group.first + group.count; ++i) {
        if ((read_position(simulation, i) - centers[galaxy]).norm() >
            30.0 * KPC) {
          ++far;
        }
        ++disk;
      }
    }
    std::println("{:>6} {:>12.2e} {:>15.2f} {:>8.3f} {:>10.1f}", myr,
                 now.energy() / start.energy() - 1.0, separation,
                 static_cast<double>(far) / static_cast<double>(disk),
                 std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - wall_start)
                     .count());
  }
  return EXIT_SUCCESS;
}

auto run_disk(std::size_t disk_bodies, int million_years) -> int {
  model::DiskGalaxy galaxy = make_standard_galaxy(disk_bodies);
  Simulation simulation{make_standard_disk_scenario(disk_bodies)};
  engine::Driver driver{Timing{.max_step = Year{2000000}}, Depend(simulation)};
  if (auto started = driver.start(); !started) {
    std::println(stderr, "Error: {}", started.error().message());
    return EXIT_FAILURE;
  }
  std::println("{:>6} {:>16} {:>14}", "Myr", "half-mass kpc", "thickness kpc");
  for (int myr = 0; myr <= million_years; myr += 50) {
    if (auto reached = driver.advance_to(BasicTimePoint<Year>{} +
                                         Year{std::int64_t{myr} * 1000000});
        !reached) {
      std::println(stderr, "Error: {}", reached.error().message());
      return EXIT_FAILURE;
    }
    std::vector<double> radii;
    double z2 = 0.0;
    for (std::size_t i = 0; i < galaxy.disk_bodies; ++i) {
      Vector3 p = read_position(simulation, i);
      radii.push_back(p.head<2>().norm());
      z2 += p.z() * p.z();
    }
    std::ranges::sort(radii);
    std::println("{:>6} {:>16.3f} {:>14.3f}", myr,
                 radii[radii.size() / 2] / KPC,
                 std::sqrt(z2 / galaxy.disk_bodies) / KPC);
  }
  return EXIT_SUCCESS;
}

}  // namespace
}  // namespace simon::galactic

auto main(int argc, char** argv) -> int {
  using namespace simon::galactic;
  std::vector<std::string_view> arguments;
  std::string_view start;
  std::string_view track;
  Year step{1000000};
  for (int i = 1; i < argc; ++i) {
    std::string_view argument = argv[i];
    if (argument.starts_with("--start=")) {
      start = argument.substr(8);
    } else if (argument.starts_with("--track=")) {
      track = argument.substr(8);
    } else if (argument.starts_with("--step=")) {
      step = Year{std::atoll(argument.substr(7).data())};
    } else {
      arguments.push_back(argument);
    }
  }
  std::string_view what = arguments.empty() ? "collision" : arguments[0];
  std::size_t bodies = arguments.size() > 1
                           ? std::strtoull(arguments[1].data(), nullptr, 10)
                           : 2000;
  int million_years =
      arguments.size() > 2 ? std::atoi(arguments[2].data()) : 2000;
  if (what == "collision") {
    return run_collision(bodies, million_years, step, start, track);
  }
  if (what == "disk") return run_disk(bodies, million_years);
  std::println(stderr,
               "Usage: galactic [collision|disk] [disk bodies] "
               "[million years] [--step=YEARS] [--start=PATH] "
               "[--track=PATH]");
  return EXIT_FAILURE;
}
