// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "application/flight/testing.hpp"
#include "base/testing.hpp"
#include "model/aircraft_data.hpp"
#include "model/flight_control.hpp"
#include "model/units.hpp"

// The 737's flight controls, converted from JSBSim, against JSBSim's own,
// frame by frame through the sweeps, steps and extensions that
// reference/jsbsim_737_flight_control.py recorded.
namespace simon::model {

namespace {

using namespace flight::testing;

constexpr char REFERENCE[] =
    "application/flight/reference/jsbsim_737_flight_control.csv";
constexpr double DT = 1.0 / 60.0;  // The reference's frame, seconds.

constexpr std::array COMMANDS{
    FlightSignal::ELEVATOR_COMMAND,
    FlightSignal::AILERON_COMMAND,
    FlightSignal::RUDDER_COMMAND,
    FlightSignal::FLAPS_COMMAND,
    FlightSignal::GEAR_COMMAND,
    FlightSignal::SPEEDBRAKE_COMMAND,
    FlightSignal::SPOILERS_COMMAND,
    FlightSignal::PITCH_TRIM_COMMAND,
    FlightSignal::ROLL_TRIM_COMMAND,
    FlightSignal::YAW_TRIM_COMMAND,
    FlightSignal::MACH,
    FlightSignal::YAW_RATE,
};

constexpr std::array<std::string_view, 8> SURFACES{
    "elevator",   "left_aileron", "right_aileron",   "rudder",
    "flaps_norm", "gear",         "speedbrake_norm", "spoilers_norm",
};

auto name_of(FlightSignal signal) -> std::string {
  return std::string{flight_signal_name(signal)};
}

}  // namespace

TEST_CASE("FlightControl737") {
  auto aircraft = load_aircraft(AIRCRAFT);
  REQUIRE(aircraft);
  const FlightControlData& controls = aircraft->flight_controls;
  REQUIRE(controls.blocks.size() > 10);
  for (std::string_view surface : SURFACES) {
    REQUIRE(find_signal(controls, surface));
  }
  std::vector<Row> rows = load_rows(REFERENCE);
  REQUIRE(rows.size() > 800);

  SECTION("ShouldMatchJsbsimGivenRecordedCommandsAndState") {
    // Kinematic blocks start where JSBSim's first frame left them.
    FlightSignals signals;
    for (const FlightBlock& block : controls.blocks) {
      if (block.kind == FlightBlock::Kind::KINEMATIC && block.output) {
        double start = rows[0].at(controls.signals[*block.output]);
        signals.values[block.signal] = start;
        signals.values[*block.output] = start;
      }
    }

    double worst = 0.0;
    std::string where;
    for (std::size_t i = 1; i < rows.size(); ++i) {
      const Row& row = rows[i];
      for (FlightSignal command : COMMANDS) {
        signals[command] = row.at(name_of(command));
      }
      run_flight_controls(controls, InOut(signals), DT * second);
      for (std::string_view surface : SURFACES) {
        std::string name{surface};
        double error = std::abs(
            signals.values[*find_signal(controls, surface)] - row.at(name));
        if (error > worst) {
          worst = error;
          where = name + " at frame " + std::to_string(i);
        }
      }
    }
    CAPTURE(worst, where);
    CHECK(worst < 1e-12);
  }

  SECTION("ShouldSettleKinematicBlocksGivenCommands") {
    FlightSignals signals;
    signals[FlightSignal::FLAPS_COMMAND] = 0.5;
    signals[FlightSignal::GEAR_COMMAND] = 1.0;
    settle_flight_controls(controls, InOut(signals));
    CHECK(signals.values[*find_signal(controls, "flaps_norm")] == 0.5);
    CHECK(signals.values[*find_signal(controls, "gear")] == 1.0);
  }
}

}  // namespace simon::model
