// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Times traffic at growing populations, system by system:
//
//   bazel run -c opt //application/automotive:automotive_benchmark
//       [-- --steps N] [--contend[=N]] [--grid] [populations...]
//
// By default, vehicles on 100 rings of 1 km radius with three lanes each
// way, 3,770 km of lanes in all, drivers wanting 30 m/s; a population is a
// vehicle count. With --grid, vehicles and pedestrians on a 20 by 20 grid of
// signalized junctions 200 m apart, with sidewalks and crosswalks, drivers
// wanting 50 km/h; a population is vehicles:pedestrians.
//
// Each step is 0.1 s. The world is driven for 60 s first, so that traffic
// has settled and drivers change lanes as they would. Each system runs in
// its own single-system scheduler, in schedule order, against one world.

#include <array>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <optional>
#include <print>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "application/automotive/simulation.hpp"
#include "base/core.hpp"
#include "core/argument.hpp"
#include "framework/benchmarking.hpp"

namespace simon::automotive {
namespace {

using namespace std::chrono_literals;
using WallClock = std::chrono::steady_clock;

constexpr Duration DT = 100ms;
constexpr int DEFAULT_STEPS = 300;  // 30 s simulated.
constexpr int SETTLING_STEPS = 600;

// Steps `world` through settling and `steps` more, each of `systems` in its
// own scheduler in turn, putting the entities back in road order every
// ROAD_ORDER_STEPS as the simulation does, and prints each one's share of
// the measured time, per step and per entity-step of `entities`.
template <typename... Systems>
auto time_systems(const Network& network, InOut<World> world, int steps,
                  double entities,
                  std::array<std::string_view, sizeof...(Systems)> names,
                  Systems... systems) -> void {
  std::tuple<framework::Scheduler<World, SystemList<Systems>>...> schedulers{
      SystemList<Systems>{std::move(systems)}...};
  std::array<double, sizeof...(Systems)> seconds{};
  double ordering = 0.0;
  keep_road_order(network, world);
  for (int i = 0; i < SETTLING_STEPS + steps; ++i) {
    Step step{.time = TimePoint{} + i * DT, .dt = DT};
    std::size_t index = 0;
    std::apply(
        [&](auto&... scheduler) {
          auto timed = [&](auto& one) {
            auto start = WallClock::now();
            one.step(step, world);
            if (i >= SETTLING_STEPS) {
              seconds[index] +=
                  std::chrono::duration<double>(WallClock::now() - start)
                      .count();
            }
            ++index;
          };
          (timed(scheduler), ...);
        },
        schedulers);
    if ((i + 1) % ROAD_ORDER_STEPS == 0) {
      auto start = WallClock::now();
      keep_road_order(network, world);
      if (i >= SETTLING_STEPS) {
        ordering +=
            std::chrono::duration<double>(WallClock::now() - start).count();
      }
    }
  }

  double total = ordering;
  for (double s : seconds) {
    total += s;
  }
  std::println("  total {:10.3f} ms/step {:10.1f} ns/entity-step",
               1e3 * total / steps, 1e9 * total / (entities * steps));
  std::array<std::size_t, sizeof...(Systems)> bytes{
      framework::bytes_per_entity_v<Systems>...};
  for (std::size_t i = 0; i < names.size(); ++i) {
    std::println("  {:<22} {:10.3f} ms/step {:6.1f}% {:6} B/entity", names[i],
                 1e3 * seconds[i] / steps, 100.0 * seconds[i] / total,
                 bytes[i]);
  }
  std::println("  {:<22} {:10.3f} ms/step {:6.1f}%", "keep_road_order",
               1e3 * ordering / steps, 100.0 * ordering / total);
}

auto measure_rings(const Network& network, int vehicles, int steps) -> void {
  Scenario scenario{.vehicles = vehicles,
                    .starting_speed = 25.0 * meter_per_second,
                    .following = {.desired_speed = 30.0 * meter_per_second}};
  World world;
  CHECK_POSTCONDITION(build_world(scenario, network, Out(world)).has_value());
  CHECK_POSTCONDITION(
      build_scenario(scenario, network, InOut(world)).has_value());
  world.sync();
  std::println("\n{} vehicles: {} steps of 0.1 s", vehicles, steps);
  time_systems(network, InOut(world), steps, vehicles,
               {"Decide", "Drive", "FollowLane"}, Decide{network},
               Drive{network}, FollowLane{network});

  double speeds = 0.0;
  world.store_of<LaneState>().for_each([&](Entity, const LaneState& state) {
    speeds += state.speed.numerical_value_in(meter_per_second);
  });
  std::println("  ending at {:.1f} m/s", speeds / vehicles);
}

auto measure_grid(const Network& network, int vehicles, int pedestrians,
                  int steps) -> void {
  Scenario scenario{
      .vehicles = vehicles,
      .starting_speed = 10.0 * meter_per_second,
      .following = {.desired_speed = 50.0 / 3.6 * meter_per_second},
      .pedestrians = pedestrians};
  World world;
  CHECK_POSTCONDITION(build_world(scenario, network, Out(world)).has_value());
  CHECK_POSTCONDITION(
      build_scenario(scenario, network, InOut(world)).has_value());
  world.sync();
  std::println("\n{} vehicles and {} pedestrians: {} steps of 0.1 s", vehicles,
               pedestrians, steps);
  time_systems(network, InOut(world), steps, vehicles + pedestrians,
               {"RunSignals", "Pace", "Decide", "Drive", "FollowLane", "Walk",
                "PlaceWalker"},
               RunSignals{}, Pace{network}, Decide{network}, Drive{network},
               FollowLane{network}, Walk{network}, PlaceWalker{network});

  double speeds = 0.0;
  world.store_of<LaneState>().for_each([&](Entity, const LaneState& state) {
    speeds += state.speed.numerical_value_in(meter_per_second);
  });
  std::uint64_t trips = 0;
  world.store_of<WalkRoute>().for_each(
      [&](Entity, const WalkRoute& route) { trips += route.trips; });
  std::println("  vehicles ending at {:.1f} m/s; pedestrians walked {} trips",
               vehicles > 0 ? speeds / vehicles : 0.0, trips);
}

}  // namespace
}  // namespace simon::automotive

// automotive_benchmark [--steps N] [--contend[=N]] [--grid] [populations...]
auto main(int argc, char** argv) -> int {
  using simon::framework::benchmark::Contention;
  using simon::framework::benchmark::parse_count;
  int steps = simon::automotive::DEFAULT_STEPS;
  unsigned threads = 0;
  bool grid = false;
  std::vector<std::pair<int, int>> populations;
  for (int i = 1; i < argc; ++i) {
    std::string_view argument{argv[i]};
    std::optional<int> count;
    std::size_t colon = argument.find(':');
    if (argument == "--steps" && i + 1 < argc &&
        (count = parse_count(argv[i + 1]))) {
      steps = *count;
      ++i;
    } else if (argument == "--grid") {
      grid = true;
    } else if (auto asked = Contention::threads_from(argument)) {
      threads = *asked;
    } else if (colon != std::string_view::npos &&
               parse_count(std::string{argument.substr(0, colon)}) &&
               parse_count(std::string{argument.substr(colon + 1)})) {
      populations.emplace_back(
          *parse_count(std::string{argument.substr(0, colon)}),
          *parse_count(std::string{argument.substr(colon + 1)}));
    } else if ((count = parse_count(argument))) {
      populations.emplace_back(*count, 0);
    } else {
      std::println(stderr, "unknown argument: {}", argument);
      return 1;
    }
  }
  if (populations.empty()) {
    populations = grid ? std::vector<std::pair<int, int>>{{1'000, 1'000},
                                                          {10'000, 10'000},
                                                          {10'000, 100'000}}
                       : std::vector<std::pair<int, int>>{
                             {1'000, 0}, {10'000, 0}, {100'000, 0}};
  }
  auto network = simon::automotive::load_network(
      grid ? "application/automotive/roads/grid.xodr"
           : "application/automotive/roads/rings.xodr");
  if (!network) {
    std::println(stderr, "{}", network.error().message());
    return 1;
  }
  Contention contention{threads};
  std::println("{}", Contention::describe(threads));
  for (auto [vehicles, pedestrians] : populations) {
    if (grid) {
      simon::automotive::measure_grid(*network, vehicles, pedestrians, steps);
    } else {
      simon::automotive::measure_rings(*network, vehicles, steps);
    }
  }
}
