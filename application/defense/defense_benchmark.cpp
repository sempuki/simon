// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Times the defense simulation at growing populations, system by system.
//
//   bazel run -c opt //application/defense:defense_benchmark [-- --steps N]
//       [--contend[=N]] [--in-turn] [drones...]
//
// --in-turn has each site's radars scan in turn (Scenario::radars_in_turn).
// The total line reports the slowest step as well as the average.
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
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <expected>
#include <optional>
#include <print>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "application/defense/simulation.hpp"
#include "base/core.hpp"
#include "framework/benchmarking.hpp"
#include "framework/type_list.hpp"
#include "framework/vocabulary.hpp"

namespace simon::defense {
namespace {

using namespace std::chrono_literals;
using WallClock = std::chrono::steady_clock;

constexpr Duration DT = 10ms;
constexpr int DEFAULT_STEPS = 500;  // 5 s simulated.
constexpr auto WALL_BUDGET = 30s;   // Per population, unless --steps is given.

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
                  .spawn_distance = 3500.0 * model::meter,
                  .spawn_spread = 1000.0 * model::meter,
                  .sites = sites};
}

auto measure(int drones, int maximum_steps, bool budgeted, bool in_turn)
    -> void {
  using List = Scheduler::FlattenedSystemList;
  constexpr std::size_t SYSTEM_COUNT = List::size;

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
  auto schedulers = framework::benchmark::create_schedulers<World>(List{});
  std::array<double, SYSTEM_COUNT> seconds{};
  std::size_t entity_steps = 0;
  double slowest = 0.0;  // The slowest step, in seconds.

  auto wall_start = WallClock::now();
  int steps = 0;
  for (; steps < maximum_steps &&
         (!budgeted || WallClock::now() - wall_start < WALL_BUDGET) &&
         world.alive(asset) && world.store_of<RedDrone>().size() > 0;
       ++steps) {
    framework::Step step{.time = TimePoint{} + steps * DT, .dt = DT};
    entity_steps += world.size();
    std::size_t index = 0;
    double this_step = 0.0;
    std::apply(
        [&](auto&... scheduler) {
          (([&] {
             auto start = WallClock::now();
             scheduler.step(step, InOut(world));
             double elapsed =
                 std::chrono::duration<double>(WallClock::now() - start)
                     .count();
             seconds[index++] += elapsed;
             this_step += elapsed;
           }()),
           ...);
        },
        schedulers);
    slowest = std::max(slowest, this_step);
  }

  double total = 0.0;
  for (double value : seconds) total += value;
  std::println(
      "\n{} drones, {} radars, {} launchers on {} site{}: {} steps, {:.0f} "
      "entities on average",
      scenario.drones * scenario.sites, scenario.radars * scenario.sites,
      scenario.launchers * scenario.sites, scenario.sites,
      scenario.sites == 1 ? "" : "s", steps,
      static_cast<double>(entity_steps) / std::max(steps, 1));
  std::println(
      "  total {:10.3f} ms/step {:10.1f} ns/entity-step, slowest step {:.3f} "
      "ms",
      1e3 * total / std::max(steps, 1),
      1e9 * total / std::max<double>(entity_steps, 1), 1e3 * slowest);
  auto names = framework::benchmark::collect_system_names(List{});
  auto bytes = framework::benchmark::collect_bytes_per_entity(List{});
  for (std::size_t i = 0; i < SYSTEM_COUNT; ++i) {
    std::println("  {:<20} {:10.3f} ms/step {:6.1f}% {:6} B/entity", names[i],
                 1e3 * seconds[i] / std::max(steps, 1),
                 total > 0.0 ? 100.0 * seconds[i] / total : 0.0, bytes[i]);
  }
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
  int steps = simon::defense::DEFAULT_STEPS;
  bool budgeted = true;
  bool in_turn = false;
  unsigned threads = 0;
  std::vector<int> populations;
  for (int i = 1; i < argc; ++i) {
    std::string_view argument{argv[i]};
    std::optional<int> count;
    if (argument == "--in-turn") {
      in_turn = true;
    } else if (argument == "--steps" && i + 1 < argc &&
               (count = parse_count(argv[i + 1]))) {
      steps = *count;
      budgeted = false;
      ++i;
    } else if (auto asked = Contention::threads_from(argument)) {
      threads = *asked;
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
  Contention contention{threads};
  std::println("{}", Contention::describe(threads));
  for (int drones : populations) {
    simon::defense::measure(drones, steps, budgeted, in_turn);
  }
}
