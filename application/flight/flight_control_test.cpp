// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "base/testing.hpp"
#include "model/aircraft_data.hpp"
#include "model/flight_control.hpp"
#include "model/units.hpp"

// The 737's flight controls, converted from JSBSim, against JSBSim's own,
// frame by frame through the sweeps, steps and extensions that
// reference/jsbsim_737_flight_control.py recorded.
namespace simon::model {

namespace {

constexpr char AIRCRAFT[] = "application/flight/aircraft/737.aircraft";
constexpr char REFERENCE[] =
    "application/flight/reference/jsbsim_737_flight_control.csv";
constexpr double DT = 1.0 / 60.0;  // The reference's frame, seconds.

using Row = std::map<std::string, double, std::less<>>;

auto load_reference() -> std::vector<Row> {
  std::ifstream file{REFERENCE};
  REQUIRE(file);
  std::string line;
  std::getline(file, line);
  std::vector<std::string> names;
  for (std::size_t at = 0; at <= line.size();) {
    std::size_t comma = std::min(line.find(',', at), line.size());
    names.emplace_back(line.substr(at, comma - at));
    at = comma + 1;
  }

  std::vector<Row> rows;
  while (std::getline(file, line)) {
    Row row;
    const char* next = line.data();
    const char* end = line.data() + line.size();
    for (const std::string& name : names) {
      double value = 0.0;
      auto [stop, error] = std::from_chars(next, end, value);
      REQUIRE(error == std::errc{});
      row[name] = value;
      next = stop + 1;
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

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

constexpr std::array SURFACES{
    FlightSignal::ELEVATOR,      FlightSignal::LEFT_AILERON,
    FlightSignal::RIGHT_AILERON, FlightSignal::RUDDER,
    FlightSignal::FLAPS,         FlightSignal::GEAR,
    FlightSignal::SPEEDBRAKE,    FlightSignal::SPOILERS,
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
  std::vector<Row> rows = load_reference();
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
      run_flight_controls(controls, signals, DT * second);
      for (FlightSignal surface : SURFACES) {
        double error = std::abs(signals[surface] - row.at(name_of(surface)));
        if (error > worst) {
          worst = error;
          where = name_of(surface) + " at frame " + std::to_string(i);
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
    settle_flight_controls(controls, signals);
    CHECK(signals[FlightSignal::FLAPS] == 0.5);
    CHECK(signals[FlightSignal::GEAR] == 1.0);
  }
}

}  // namespace simon::model
