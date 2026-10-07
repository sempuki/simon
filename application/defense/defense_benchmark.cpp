// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Times the defense simulation at growing populations, system by system.
//
//   bazel run -c opt //application/defense:defense_benchmark [-- --steps N]
//       [--contend[=N]] [--in-turn] [drones...]
//
// --in-turn has each site's radars scan in turn (Scenario::radars_in_turn).
//
// Each scenario spawns its drones inside radar and launcher range, so
// sensing, engagement, guidance and blasts all run from the first steps.
// Sites (each with its asset, radars, launchers and 1,000 drones) scale with
// the drones, so density stays the same. Each system's line ends with the
// bytes its loop can read per entity, from the sizes of the components it
// names (framework::bytes_per_entity_v). --contend runs one thread per spare
// core streaming over a large buffer, to compete for shared cache and memory
// bandwidth as a busy cloud host would; --contend=N runs N. Each system runs in
// its own single-system scheduler, in schedule order, against one world; that
// is the same as the full schedule, which also syncs after every system.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <expected>
#include <optional>
#include <print>
#include <string_view>
#include <vector>

#include "application/defense/simulation.hpp"
#include "base/core.hpp"
#include "core/argument.hpp"
#include "framework/benchmarking.hpp"

namespace simon::defense {
namespace {

using namespace std::chrono_literals;

constexpr Duration DT = 10ms;
constexpr int DEFAULT_STEPS = 500;    // 5 s simulated.
constexpr double WALL_BUDGET = 30.0;  // s per population, unless --steps.

// One site per 1,000 drones, each with the density of the single-site
// benchmark: 10 radars, 50 launchers and 1,000 drones spawning 3 to 4 km from
// its asset. The area grows with the population, so each radar and launcher
// sees about as many drones at any size.
auto make_scenario(int drones) -> Scenario {
  constexpr int DRONES_PER_SITE = 1000;
  int sites = std::max(1, drones / DRONES_PER_SITE);
  int per_site = drones / sites;
  return Scenario{.seed = 1,
                  .radars = std::max(3, per_site / 100),
                  .launchers = std::max(3, per_site / 20),
                  .drones = per_site,
                  .spawn_distance = 3500.0 * meter,
                  .spawn_spread = 1000.0 * meter,
                  .sites = sites};
}

auto measure(int drones, int maximum_steps, bool budgeted, bool in_turn)
    -> void {
  Scenario scenario = make_scenario(drones);
  scenario.radars_in_turn = in_turn;
  World world;
  std::expected<void, framework::Status> built =
      build_world(scenario, Out(world));
  CHECK_POSTCONDITION(built.has_value());
  std::expected<Entity, framework::Status> built_asset =
      build_scenario(scenario, InOut(world));
  CHECK_POSTCONDITION(built_asset.has_value());
  Entity asset = *built_asset;
  framework::benchmark::SystemTimerFor<World, Scheduler::FlattenedSystemList>
      timer;
  std::size_t entity_steps = 0;

  framework::benchmark::Stopwatch wall;
  int steps = 0;
  for (; steps < maximum_steps && (!budgeted || wall.seconds() < WALL_BUDGET) &&
         world.alive(asset) && world.store_of<RedDrone>().size() > 0;
       ++steps) {
    entity_steps += world.size();
    timer.step(Step{.time = TimePoint{} + steps * DT, .dt = DT}, InOut(world));
  }

  std::println(
      "\n{} drones, {} radars, {} launchers on {} site{}: {} steps, {:.0f} "
      "entities on average",
      scenario.drones * scenario.sites, scenario.radars * scenario.sites,
      scenario.launchers * scenario.sites, scenario.sites,
      scenario.sites == 1 ? "" : "s", steps,
      static_cast<double>(entity_steps) / std::max(steps, 1));
  timer.print(static_cast<double>(entity_steps));
}

}  // namespace
}  // namespace simon::defense

// defense_benchmark [--steps N] [--contend[=N]] [--in-turn] [drones...]
//
// Without --steps, each population runs up to 500 steps or 30 s of wall time,
// whichever comes first. Radars scan once a second, so compare runs only over
// the same number of steps.
auto main(int argc, char** argv) -> int {
  using simon::framework::benchmark::Contention;
  using simon::framework::benchmark::parse_count;
  auto arguments = simon::framework::benchmark::parse_arguments(argc, argv);
  if (!arguments) {
    std::println(stderr, "{}", arguments.error());
    return 1;
  }
  int steps = arguments->steps.value_or(simon::defense::DEFAULT_STEPS);
  bool budgeted = !arguments->steps;
  bool in_turn = false;
  std::vector<int> populations;
  for (std::string_view argument : arguments->rest) {
    std::optional<int> count;
    if (argument == "--in-turn") {
      in_turn = true;
    } else if ((count = parse_count(argument))) {
      populations.push_back(*count);
    } else {
      std::println(stderr, "unknown argument: {}", argument);
      return 1;
    }
  }
  if (populations.empty()) {
    populations = {1'000, 10'000, 100'000};
  }
  Contention contention{arguments->threads};
  std::println("{}", Contention::describe(arguments->threads));
  for (int drones : populations) {
    simon::defense::measure(drones, steps, budgeted, in_turn);
  }
}
