// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

// Times the missile simulation at growing populations, system by system.
//
//   bazel run -c opt //application/missile:missile_benchmark [-- --steps N]
//       [--contend[=N]] [drones...]
//
// Each scenario spawns its drones inside radar and launcher range, so
// sensing, engagement, guidance and blasts all run from the first steps.
// Sites (each with its asset, radars, launchers and 1,000 drones) scale with
// the drones, so density stays the same. --contend runs one thread per spare
// core streaming over a large buffer, to compete for shared cache and memory
// bandwidth as a busy cloud host would; --contend=N runs N. Each system runs in
// its own single-system scheduler, in schedule order, against one world; that
// is the same as the full schedule, which also syncs after every system.

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <optional>
#include <print>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
#include <vector>

#include "application/missile/simulation.hpp"
#include "base/core.hpp"
#include "framework/benchmark_support.hpp"
#include "framework/type_list.hpp"

namespace simon::missile {
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
Scenario scenario_of(int drones) {
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

template <typename... SystemTypes>
auto schedulers_of(framework::TypeList<SystemTypes...>) {
  return std::tuple<framework::Scheduler<World, SystemList<SystemTypes>>...>{};
}

template <typename... SystemTypes>
std::array<std::string, sizeof...(SystemTypes)> names_of(
    framework::TypeList<SystemTypes...>) {
  auto short_name = [](std::string name) {
    std::size_t colons = name.rfind("::");
    return colons == std::string::npos ? name : name.substr(colons + 2);
  };
  return {short_name(lib::to_type_string<SystemTypes>())...};
}

void measure(int drones, int maximum_steps, bool budgeted) {
  using List = Scheduler::FlattenedSystemList;
  constexpr std::size_t SYSTEM_COUNT = List::size;

  Scenario scenario = scenario_of(drones);
  World world{world_configuration_of(scenario)};
  Entity asset = build_scenario(lib::InOut(world), scenario);
  auto schedulers = schedulers_of(List{});
  std::array<double, SYSTEM_COUNT> seconds{};
  std::size_t entity_steps = 0;

  auto wall_start = WallClock::now();
  int steps = 0;
  for (; steps < maximum_steps &&
         (!budgeted || WallClock::now() - wall_start < WALL_BUDGET) &&
         world.alive(asset) && world.store_of<RedDrone>().size() > 0;
       ++steps) {
    framework::Step step{.time = TimePoint{} + steps * DT, .dt = DT};
    entity_steps += world.size();
    std::size_t index = 0;
    std::apply(
        [&](auto&... scheduler) {
          (([&] {
             auto start = WallClock::now();
             scheduler.step(lib::InOut(world), step);
             seconds[index++] +=
                 std::chrono::duration<double>(WallClock::now() - start)
                     .count();
           }()),
           ...);
        },
        schedulers);
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
  std::println("  total {:10.3f} ms/step {:10.1f} ns/entity-step",
               1e3 * total / std::max(steps, 1),
               1e9 * total / std::max<double>(entity_steps, 1));
  auto names = names_of(List{});
  for (std::size_t i = 0; i < SYSTEM_COUNT; ++i) {
    std::println("  {:<20} {:10.3f} ms/step {:6.1f}%", names[i],
                 1e3 * seconds[i] / std::max(steps, 1),
                 total > 0.0 ? 100.0 * seconds[i] / total : 0.0);
  }
}

}  // namespace
}  // namespace simon::missile

namespace {

// A whole positive number, or nothing.
std::optional<int> count_of(std::string_view text) {
  int count = 0;
  auto [end, error] =
      std::from_chars(text.data(), text.data() + text.size(), count);
  if (error != std::errc{} || end != text.data() + text.size() || count <= 0) {
    return std::nullopt;
  }
  return count;
}

}  // namespace

// missile_benchmark [--steps N] [--contend[=N]] [drones...]
//
// Without --steps, each population runs up to 500 steps or 30 s of wall time,
// whichever comes first. Radars scan once a second, so compare runs only over
// the same number of steps.
int main(int argc, char** argv) {
  using simon::framework::benchmark::Contention;
  int steps = simon::missile::DEFAULT_STEPS;
  bool budgeted = true;
  unsigned threads = 0;
  std::vector<int> populations;
  for (int i = 1; i < argc; ++i) {
    std::string_view argument{argv[i]};
    std::optional<int> count;
    if (argument == "--steps" && i + 1 < argc &&
        (count = count_of(argv[i + 1]))) {
      steps = *count;
      budgeted = false;
      ++i;
    } else if (auto asked = Contention::threads_from(argument)) {
      threads = *asked;
    } else if ((count = count_of(argument))) {
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
    simon::missile::measure(drones, steps, budgeted);
  }
}
