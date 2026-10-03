// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Times rigid aircraft at growing populations, system by system, over a flat
// Earth and round a turning one: 737s converted from JSBSim, trimmed roughly
// in cruise, each flying on its own.
//
//   bazel run -c opt //application/aeronautic:rigid_benchmark [-- --steps N]
//       [aircraft...]
//
// Each system runs in its own single-system scheduler, in schedule order,
// against one world.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <expected>
#include <optional>
#include <print>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "application/aeronautic/simulation.hpp"
#include "application/aeronautic/simulation_components.hpp"
#include "application/aeronautic/simulation_systems.hpp"
#include "base/core.hpp"
#include "format/aircraft_file.hpp"
#include "framework/benchmarking.hpp"
#include "framework/vocabulary.hpp"

namespace simon::aeronautic {
namespace {

using namespace std::chrono_literals;
using WallClock = std::chrono::steady_clock;

constexpr std::string_view BOEING_737 =
    "3rd_party/jsbsim/737.aircraft";
constexpr Duration DT = 8ms;
constexpr int DEFAULT_STEPS = 250;  // 2 s simulated.

// Creates `count` 737s in cruise, 2 km apart, each at 6 km and 200 m/s.
auto populate(int count, const model::Earth& earth,
              const model::AircraftData& data, InOut<World> world) -> void {
  auto side = static_cast<int>(std::ceil(std::sqrt(count)));
  auto trim = trim_in_cruise(data, earth);
  CHECK_POSTCONDITION(trim.has_value());
  auto transaction = world->transaction();
  for (int i = 0; i < count; ++i) {
    auto built = create_rigid_aircraft(data, earth, *trim, SurfaceGains{},
                                       2000.0 * (i % side) * model::meter,
                                       2000.0 * (i / side) * model::meter,
                                       0.0 * model::radian, Route{}, world);
    CHECK_POSTCONDITION(built.has_value());
  }
  transaction.commit();
}

auto measure(int aircraft, bool round, int steps,
             const model::AircraftData& data) -> void {
  model::Earth earth = round ? model::Earth::round(model::wgs84::Geodetic{})
                             : model::Earth::flat();
  World world;
  std::expected<void, framework::Status> built =
      World::set_up()
          .numbered(1)
          .holding<archetype::RigidAircraft>(static_cast<std::size_t>(aircraft))
          .build(Out(world));
  CHECK_POSTCONDITION(built.has_value());
  populate(aircraft, earth, data, InOut(world));
  world.sync();

  std::tuple schedulers{
      framework::Scheduler<World, SystemList<RunFlightControls>>{
          SystemList{RunFlightControls{earth}}},
      framework::Scheduler<World, SystemList<RunEngines>>{
          SystemList{RunEngines{earth}}},
      framework::Scheduler<World, SystemList<Rigid>>{
          SystemList{Rigid{SystemList{RigidAircraftRates{earth}}}}},
      framework::Scheduler<World, SystemList<BurnFuel>>{},
      framework::Scheduler<World, SystemList<FollowRigidBody>>{
          SystemList{FollowRigidBody{earth}}},
  };
  constexpr std::array<std::string_view, 5> NAMES{
      "RunFlightControls", "RunEngines", "Rigid", "BurnFuel",
      "FollowRigidBody"};
  std::array<double, NAMES.size()> seconds{};
  for (int i = 0; i < steps; ++i) {
    framework::Step step{.time = TimePoint{} + i * DT, .dt = DT};
    std::size_t index = 0;
    std::apply(
        [&](auto&... scheduler) {
          (([&] {
             auto start = WallClock::now();
             scheduler.step(step, InOut(world));
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
  double entity_steps = static_cast<double>(aircraft) * std::max(steps, 1);
  std::println("\n{} rigid aircraft, {} Earth: {} steps of 8 ms", aircraft,
               round ? "round" : "flat", steps);
  std::println("  total {:10.3f} ms/step {:10.1f} ns/entity-step",
               1e3 * total / std::max(steps, 1), 1e9 * total / entity_steps);
  for (std::size_t i = 0; i < NAMES.size(); ++i) {
    std::println("  {:<18} {:10.3f} ms/step {:6.1f}%", NAMES[i],
                 1e3 * seconds[i] / std::max(steps, 1),
                 total > 0.0 ? 100.0 * seconds[i] / total : 0.0);
  }
}

}  // namespace
}  // namespace simon::aeronautic

// rigid_benchmark [--steps N] [aircraft...]
auto main(int argc, char** argv) -> int {
  using simon::framework::benchmark::parse_count;
  int steps = simon::aeronautic::DEFAULT_STEPS;
  std::vector<int> populations;
  for (int i = 1; i < argc; ++i) {
    std::string_view argument{argv[i]};
    std::optional<int> count;
    if (argument == "--steps" && i + 1 < argc &&
        (count = parse_count(argv[i + 1]))) {
      steps = *count;
      ++i;
    } else if ((count = parse_count(argument))) {
      populations.push_back(*count);
    } else {
      std::println(stderr, "unknown argument: {}", argument);
      return 1;
    }
  }
  if (populations.empty()) {
    populations = {100, 1'000, 10'000};
  }
  auto data =
      simon::format::load_aircraft(std::string{simon::aeronautic::BOEING_737});
  if (!data) {
    std::println(stderr, "{}", data.error().message());
    return 1;
  }
  for (int aircraft : populations) {
    simon::aeronautic::measure(aircraft, false, steps, *data);
    simon::aeronautic::measure(aircraft, true, steps, *data);
  }
}
