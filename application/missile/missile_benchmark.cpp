// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

// Times the missile simulation at growing populations, system by system.
//
//   bazel run -c opt //application/missile:missile_benchmark
//
// Each scenario spawns its drones inside radar and launcher range, so
// sensing, engagement, guidance and blasts all run from the first steps.
// Radars and launchers scale with the drones. Each system runs in its own
// single-system scheduler, in schedule order, against one world; that is the
// same as the full schedule, which also syncs after every system.

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <print>
#include <string>
#include <tuple>

#include "application/missile/simulation.hpp"
#include "base/core.hpp"
#include "framework/type_list.hpp"

namespace simon::missile {
namespace {

using namespace std::chrono_literals;
using WallClock = std::chrono::steady_clock;

constexpr Duration DT = 10ms;
constexpr int MAXIMUM_STEPS = 500;  // 5 s simulated.
constexpr auto WALL_BUDGET = 30s;   // Per population.

Scenario scenario_of(int drones) {
  return Scenario{.seed = 1,
                  .radars = std::max(3, drones / 100),
                  .launchers = std::max(3, drones / 20),
                  .drones = drones,
                  .spawn_distance = 3500.0 * model::meter,
                  .spawn_spread = 1000.0 * model::meter};
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

void measure(int drones) {
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
  for (; steps < MAXIMUM_STEPS && WallClock::now() - wall_start < WALL_BUDGET &&
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
      "\n{} drones, {} radars, {} launchers: {} steps, {:.0f} entities "
      "on average",
      drones, scenario.radars, scenario.launchers, steps,
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

int main() {
  for (int drones : {1'000, 10'000, 100'000}) {
    simon::missile::measure(drones);
  }
}
