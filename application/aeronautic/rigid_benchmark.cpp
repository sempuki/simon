// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Times rigid aircraft at growing populations, system by system, over a flat
// Earth and round a turning one: 737s converted from JSBSim, trimmed roughly
// in cruise, each flying on its own.
//
//   bazel run -c opt //application/aeronautic:rigid_benchmark [-- --steps N]
//       [--contend[=N]] [aircraft...]
//
// Each system runs in its own single-system scheduler, in schedule order,
// against one world.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <expected>
#include <optional>
#include <print>
#include <string>
#include <string_view>
#include <vector>

#include "application/aeronautic/simulation.hpp"
#include "application/aeronautic/simulation_components.hpp"
#include "application/aeronautic/simulation_systems.hpp"
#include "base/core.hpp"
#include "core/argument.hpp"
#include "format/aircraft_file.hpp"
#include "framework/benchmarking.hpp"

namespace simon::aeronautic {
namespace {

using namespace std::chrono_literals;

constexpr std::string_view BOEING_737 = "3rd_party/jsbsim/737.aircraft";
constexpr Duration DT = 8ms;
constexpr int DEFAULT_STEPS = 250;  // 2 s simulated.

// Creates `count` 737s in cruise, 2 km apart, each at 6 km and 200 m/s.
auto populate(int count, const aircraft::Earth& earth,
              const aircraft::AircraftData& data, InOut<World> world) -> void {
  auto side = static_cast<int>(std::ceil(std::sqrt(count)));
  auto trim = trim_in_cruise(data, earth);
  CHECK_POSTCONDITION(trim.has_value());
  auto transaction = world->transaction();
  for (int i = 0; i < count; ++i) {
    auto built = create_rigid_aircraft(
        data, earth, *trim, SurfaceGains{}, 2000.0 * (i % side) * meter,
        2000.0 * (i / side) * meter, 0.0 * radian, Route{}, world);
    CHECK_POSTCONDITION(built.has_value());
  }
  transaction.commit();
}

auto measure(int aircraft, bool round, int steps,
             const aircraft::AircraftData& data) -> void {
  aircraft::Earth earth = round
                              ? aircraft::Earth::round(earth::wgs84::Geodetic{})
                              : aircraft::Earth::flat();
  World world;
  std::expected<void, framework::Status> built =
      World::set_up()
          .numbered(1)
          .holding<archetype::RigidAircraft>(static_cast<std::size_t>(aircraft))
          .build(Out(world));
  CHECK_POSTCONDITION(built.has_value());
  populate(aircraft, earth, data, InOut(world));
  world.sync();

  framework::benchmark::SystemTimer<World, RunFlightControls, RunEngines, Rigid,
                                    BurnFuel, FollowRigidBody>
      timer{RunFlightControls{earth}, RunEngines{earth},
            Rigid{SystemList{RigidAircraftRates{earth}}}, BurnFuel{},
            FollowRigidBody{earth}};
  for (int i = 0; i < steps; ++i) {
    timer.step(Step{.time = TimePoint{} + i * DT, .dt = DT}, InOut(world));
  }
  std::println("\n{} rigid aircraft, {} Earth: {} steps of 8 ms", aircraft,
               round ? "round" : "flat", steps);
  timer.print(static_cast<double>(aircraft) * steps);
}

}  // namespace
}  // namespace simon::aeronautic

// rigid_benchmark [--steps N] [--contend[=N]] [aircraft...]
auto main(int argc, char** argv) -> int {
  using simon::framework::benchmark::Contention;
  using simon::framework::benchmark::parse_count;
  auto arguments = simon::framework::benchmark::parse_arguments(argc, argv);
  if (!arguments) {
    std::println(stderr, "{}", arguments.error());
    return 1;
  }
  int steps = arguments->steps.value_or(simon::aeronautic::DEFAULT_STEPS);
  std::vector<int> populations;
  for (std::string_view argument : arguments->rest) {
    std::optional<int> count = parse_count(argument);
    if (!count) {
      std::println(stderr, "unknown argument: {}", argument);
      return 1;
    }
    populations.push_back(*count);
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
  Contention contention{arguments->threads};
  std::println("{}", Contention::describe(arguments->threads));
  for (int aircraft : populations) {
    simon::aeronautic::measure(aircraft, false, steps, *data);
    simon::aeronautic::measure(aircraft, true, steps, *data);
  }
}
