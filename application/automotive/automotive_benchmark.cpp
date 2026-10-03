// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Times traffic at growing populations, system by system, on 100 rings of
// 1 km radius with three lanes each way, 3,770 km of lanes in all:
//
//   bazel run -c opt //application/automotive:automotive_benchmark
//       [-- --steps N] [--contend[=N]] [vehicles...]
//
// Drivers want 30 m/s, and each step is 0.1 s. The world is driven for 60 s
// first, so that traffic has settled and drivers change lanes as they would.
// Each system runs in its own single-system scheduler, in schedule order,
// against one world.

#include <array>
#include <chrono>
#include <cstddef>
#include <optional>
#include <print>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "application/automotive/simulation.hpp"
#include "base/core.hpp"
#include "framework/benchmarking.hpp"
#include "framework/vocabulary.hpp"

namespace simon::automotive {
namespace {

using namespace std::chrono_literals;
using WallClock = std::chrono::steady_clock;

constexpr Duration DT = 100ms;
constexpr int DEFAULT_STEPS = 300;  // 30 s simulated.
constexpr int SETTLING_STEPS = 600;

auto measure(const Network& network, int vehicles, int steps) -> void {
  Scenario scenario{
      .vehicles = vehicles,
      .starting_speed = 25.0 * model::meter_per_second,
      .following = {.desired_speed = 30.0 * model::meter_per_second}};
  World world;
  CHECK_POSTCONDITION(build_world(scenario, Out(world)).has_value());
  CHECK_POSTCONDITION(
      build_scenario(scenario, network, InOut(world)).has_value());
  world.sync();

  framework::Scheduler<World, SystemList<Decide>> decide{
      SystemList{Decide{network}}};
  framework::Scheduler<World, SystemList<Drive>> drive{
      SystemList{Drive{network}}};
  framework::Scheduler<World, SystemList<FollowLane>> follow{
      SystemList{FollowLane{network}}};
  std::array<double, 3> seconds{};
  for (int i = 0; i < SETTLING_STEPS + steps; ++i) {
    framework::Step step{.time = TimePoint{} + i * DT, .dt = DT};
    std::size_t index = 0;
    auto timed = [&](auto& scheduler) {
      auto start = WallClock::now();
      scheduler.step(step, InOut(world));
      if (i >= SETTLING_STEPS) {
        seconds[index] +=
            std::chrono::duration<double>(WallClock::now() - start).count();
      }
      ++index;
    };
    timed(decide);
    timed(drive);
    timed(follow);
  }

  double speeds = 0.0;
  world.store_of<LaneState>().for_each([&](Entity, const LaneState& state) {
    speeds += state.speed.numerical_value_in(model::meter_per_second);
  });

  double total = seconds[0] + seconds[1] + seconds[2];
  double entity_steps = static_cast<double>(vehicles) * steps;
  std::println("\n{} vehicles: {} steps of 0.1 s, ending at {:.1f} m/s",
               vehicles, steps, speeds / vehicles);
  std::println("  total {:10.3f} ms/step {:10.1f} ns/entity-step",
               1e3 * total / steps, 1e9 * total / entity_steps);
  std::array<std::string_view, 3> names{"Decide", "Drive", "FollowLane"};
  std::array<std::size_t, 3> bytes{framework::bytes_per_entity_v<Decide>,
                                   framework::bytes_per_entity_v<Drive>,
                                   framework::bytes_per_entity_v<FollowLane>};
  for (std::size_t i = 0; i < 3; ++i) {
    std::println("  {:<22} {:10.3f} ms/step {:6.1f}% {:6} B/entity", names[i],
                 1e3 * seconds[i] / steps, 100.0 * seconds[i] / total,
                 bytes[i]);
  }
}

}  // namespace
}  // namespace simon::automotive

// automotive_benchmark [--steps N] [--contend[=N]] [vehicles...]
auto main(int argc, char** argv) -> int {
  using simon::framework::benchmark::Contention;
  using simon::framework::benchmark::parse_count;
  int steps = simon::automotive::DEFAULT_STEPS;
  unsigned threads = 0;
  std::vector<int> populations;
  for (int i = 1; i < argc; ++i) {
    std::string_view argument{argv[i]};
    std::optional<int> count;
    if (argument == "--steps" && i + 1 < argc &&
        (count = parse_count(argv[i + 1]))) {
      steps = *count;
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
  auto network = simon::automotive::load_network(
      "application/automotive/roads/rings.xodr");
  if (!network) {
    std::println(stderr, "{}", network.error().message());
    return 1;
  }
  Contention contention{threads};
  std::println("{}", Contention::describe(threads));
  for (int vehicles : populations) {
    simon::automotive::measure(*network, vehicles, steps);
  }
}
