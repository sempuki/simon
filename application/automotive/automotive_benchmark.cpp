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

#include <chrono>
#include <cstddef>
#include <optional>
#include <print>
#include <string_view>
#include <utility>
#include <vector>

#include "application/automotive/simulation.hpp"
#include "base/core.hpp"
#include "core/argument.hpp"
#include "framework/benchmarking.hpp"

namespace simon::automotive {
namespace {

using namespace std::chrono_literals;

constexpr Duration DT = 100ms;
constexpr int DEFAULT_STEPS = 300;  // 30 s simulated.
constexpr int SETTLING_STEPS = 600;

// Steps `world` through settling and `steps` more, each of `systems` in its
// own scheduler in turn, putting the entities back in road order every
// ROAD_ORDER_STEPS as the simulation does, and prints each one's share of
// the measured time, per step and per entity-step of `entities`.
template <typename... Systems>
auto time_systems(const Network& network, InOut<World> world, int steps,
                  double entities, Systems... systems) -> void {
  framework::benchmark::SystemTimer<World, Systems...> timer{
      std::move(systems)...};
  keep_road_order(network, world);
  for (int i = 0; i < SETTLING_STEPS + steps; ++i) {
    bool measured = i >= SETTLING_STEPS;
    timer.step(Step{.time = TimePoint{} + i * DT, .dt = DT}, world, measured);
    if ((i + 1) % ROAD_ORDER_STEPS == 0) {
      timer.time(
          "keep_road_order", [&] { keep_road_order(network, world); },
          measured);
    }
  }
  timer.print(entities * steps);
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
  time_systems(network, InOut(world), steps, vehicles, Decide{network},
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
  auto arguments = simon::framework::benchmark::parse_arguments(argc, argv);
  if (!arguments) {
    std::println(stderr, "{}", arguments.error());
    return 1;
  }
  int steps = arguments->steps.value_or(simon::automotive::DEFAULT_STEPS);
  bool grid = false;
  std::vector<std::pair<int, int>> populations;
  for (std::string_view argument : arguments->rest) {
    std::optional<int> count;
    std::size_t colon = argument.find(':');
    if (argument == "--grid") {
      grid = true;
    } else if (colon != std::string_view::npos &&
               parse_count(argument.substr(0, colon)) &&
               parse_count(argument.substr(colon + 1))) {
      populations.emplace_back(*parse_count(argument.substr(0, colon)),
                               *parse_count(argument.substr(colon + 1)));
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
  Contention contention{arguments->threads};
  std::println("{}", Contention::describe(arguments->threads));
  for (auto [vehicles, pedestrians] : populations) {
    if (grid) {
      simon::automotive::measure_grid(*network, vehicles, pedestrians, steps);
    } else {
      simon::automotive::measure_rings(*network, vehicles, steps);
    }
  }
}
