// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Flies one flight scenario as fast as possible and prints what it did:
//
//   bazel run //application/aeronautic -- [aircraft] [precise] [rigid]
//       [fighters] [seed]
//
// `rigid` aircraft are 737s and `fighters` F-16s, both flying as rigid
// bodies.

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <print>
#include <string_view>

#include "application/aeronautic/simulation.hpp"
#include "application/arguments.hpp"
#include "core/argument.hpp"
#include "engine/driver.hpp"

// How the program is called.
constexpr std::string_view USAGE =
    "aeronautic [aircraft] [precise] [rigid] [fighters] [seed]";

auto main(int argc, char** argv) -> int {
  using namespace simon;
  using namespace std::chrono_literals;

  aeronautic::Scenario defaults;
  application::Arguments arguments{argc, argv};
  aeronautic::Scenario scenario{
      .seed = static_cast<std::uint64_t>(
          arguments.integer(4, static_cast<std::int64_t>(defaults.seed))),
      .aircraft = static_cast<int>(arguments.integer(0, defaults.aircraft)),
      .precise = static_cast<int>(arguments.integer(1, defaults.precise)),
      .rigid = static_cast<int>(arguments.integer(2, defaults.rigid)),
      .fighters = static_cast<int>(arguments.integer(3, defaults.fighters)),
  };
  if (arguments.report_error(USAGE)) {
    return EXIT_FAILURE;
  }
  aeronautic::Simulation simulation{scenario};
  engine::BatchDriver driver{engine::Timing{.max_step = 20ms},
                             Depend(simulation)};

  auto wall_start = std::chrono::steady_clock::now();
  auto reached = driver.run(TimePoint{10min});
  if (!reached) {
    std::println(stderr, "Error: {}", reached.error().message());
    return EXIT_FAILURE;
  }
  double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                              wall_start)
                    .count();

  std::println(
      "seed {}: {} aircraft ({} precise, {} rigid 737s, {} rigid F-16s) flew "
      "{:.0f} s, reaching {} waypoints ({} by rigid aircraft), in {:.2f} s",
      scenario.seed, scenario.aircraft, scenario.precise, scenario.rigid,
      scenario.fighters,
      std::chrono::duration<double>(reached->time_since_epoch()).count(),
      simulation.waypoints_reached(), simulation.rigid_waypoints_reached(),
      wall);
  return EXIT_SUCCESS;
}
