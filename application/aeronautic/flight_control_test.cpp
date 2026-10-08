// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "application/aeronautic/testing.hpp"
#include "base/testing.hpp"
#include "core/argument.hpp"
#include "core/units.hpp"
#include "format/aircraft_file.hpp"
#include "model/aircraft/definition.hpp"
#include "model/aircraft/flight_control.hpp"

// The 737's and the F-16's flight controls, converted from JSBSim, against
// JSBSim's own, frame by frame through the sweeps, steps and extensions that
// reference/jsbsim_737_flight_control.py and jsbsim_f16_flight_control.py
// recorded.
namespace simon::model {

namespace {

using namespace aeronautic::testing;

constexpr std::string_view REFERENCE =
    "application/aeronautic/reference/jsbsim_737_flight_control.csv";
constexpr double DT = 1.0 / 60.0;  // The reference's frame, seconds.

constexpr std::array COMMANDS{
    aircraft::FlightSignal::ELEVATOR_COMMAND,
    aircraft::FlightSignal::AILERON_COMMAND,
    aircraft::FlightSignal::RUDDER_COMMAND,
    aircraft::FlightSignal::FLAPS_COMMAND,
    aircraft::FlightSignal::GEAR_COMMAND,
    aircraft::FlightSignal::SPEEDBRAKE_COMMAND,
    aircraft::FlightSignal::SPOILERS_COMMAND,
    aircraft::FlightSignal::PITCH_TRIM_COMMAND,
    aircraft::FlightSignal::ROLL_TRIM_COMMAND,
    aircraft::FlightSignal::YAW_TRIM_COMMAND,
    aircraft::FlightSignal::MACH,
    aircraft::FlightSignal::YAW_RATE,
};

constexpr std::array<std::string_view, 8> SURFACES{
    "elevator",   "left_aileron", "right_aileron",   "rudder",
    "flaps_norm", "gear",         "speedbrake_norm", "spoilers_norm",
};

}  // namespace

TEST_CASE("FlightControl737") {
  auto aircraft = format::load_aircraft(std::string{BOEING_737});
  REQUIRE(aircraft);
  const aircraft::FlightControlData& controls = aircraft->flight_controls;
  REQUIRE(controls.blocks.size() > 10);
  for (std::string_view surface : SURFACES) {
    REQUIRE(aircraft::find_signal(controls, surface));
  }
  std::vector<Row> rows = load_rows(REFERENCE);
  REQUIRE(rows.size() > 800);

  SECTION("ShouldMatchJsbsimGivenRecordedCommandsAndState") {
    // Preconditions.
    // Kinematic blocks start where JSBSim's first frame left them.
    aircraft::FlightSignals signals;
    for (const aircraft::FlightBlock& block : controls.blocks) {
      if (block.kind == aircraft::FlightBlock::Kind::KINEMATIC &&
          block.output) {
        double start = rows[0].at(controls.signals[*block.output]);
        signals.values[block.signal] = start;
        signals.values[*block.output] = start;
      }
    }
    double worst = 0.0;
    std::string where;

    // Under Test.
    for (std::size_t i = 1; i < rows.size(); ++i) {
      const Row& row = rows[i];
      for (aircraft::FlightSignal command : COMMANDS) {
        signals[command] = row.at(flight_signal_name(command));
      }
      aircraft::run_flight_controls(controls, InOut(signals), DT * second);
      for (std::string_view surface : SURFACES) {
        std::string name{surface};
        double error =
            std::abs(signals.values[*aircraft::find_signal(controls, surface)] -
                     row.at(name));
        if (error > worst) {
          worst = error;
          where = name + " at frame " + std::to_string(i);
        }
      }
    }

    // Postconditions.
    CAPTURE(worst, where);
    CHECK(worst < 1e-12);
  }

  SECTION("ShouldSettleKinematicBlocksGivenCommands") {
    // Preconditions.
    aircraft::FlightSignals signals;
    signals[aircraft::FlightSignal::FLAPS_COMMAND] = 0.5;
    signals[aircraft::FlightSignal::GEAR_COMMAND] = 1.0;

    // Under Test.
    aircraft::settle_flight_controls(controls, InOut(signals));

    // Postconditions.
    CHECK(signals.values[*aircraft::find_signal(controls, "flaps_norm")] ==
          0.5);
    CHECK(signals.values[*find_signal(controls, "gear")] == 1.0);
  }
}

TEST_CASE("FlightControlF16") {
  auto aircraft = format::load_aircraft(std::string{F16});
  REQUIRE(aircraft);
  const aircraft::FlightControlData& controls = aircraft->flight_controls;
  std::vector<Row> rows = load_rows(
      "application/aeronautic/reference/jsbsim_f16_flight_control.csv");
  REQUIRE(rows.size() > 3000);
  constexpr double F16_DT = 1.0 / 120.0;

  // Every block's own signal and output that the reference records.
  std::vector<std::size_t> checked;
  for (const aircraft::FlightBlock& block : controls.blocks) {
    for (std::optional<std::size_t> signal :
         {std::optional{block.signal}, block.output}) {
      if (signal && rows[0].contains(controls.signals[*signal]) &&
          !std::ranges::contains(checked, *signal)) {
        checked.push_back(*signal);
      }
    }
  }
  REQUIRE(checked.size() > 60);

  SECTION("ShouldMatchJsbsimGivenRecordedCommandsAndState") {
    // Preconditions.
    aircraft::FlightSignals signals;
    double worst = 0.0;
    std::string where;

    // Under Test.
    for (std::size_t i = 0; i < rows.size(); ++i) {
      const Row& row = rows[i];
      bool first = i == 0 || row.at("run") != rows[i - 1].at("run");
      if (first) {
        // Each flight starts where JSBSim's first frame left every signal,
        // and each PID with that frame's input as its last.
        signals = aircraft::FlightSignals{};
        for (std::size_t s = 0; s < controls.signals.size(); ++s) {
          if (auto found = row.find(controls.signals[s]); found != row.end()) {
            signals.values[s] = found->second;
          }
        }
        for (const aircraft::FlightBlock& block : controls.blocks) {
          if (block.kind == aircraft::FlightBlock::Kind::PID) {
            const aircraft::FlightBlock::Input& input = block.inputs.front();
            double value = signals.values[input.signal] * input.scale;
            signals.values[block.state + 1] = input.negated ? -value : value;
          }
        }
        continue;
      }
      for (std::size_t s = 0; s < aircraft::FLIGHT_SIGNAL_COUNT; ++s) {
        if (auto found = row.find(controls.signals[s]); found != row.end()) {
          signals.values[s] = found->second;
        }
      }
      aircraft::run_flight_controls(controls, InOut(signals), F16_DT * second);
      for (std::size_t s : checked) {
        const std::string& name = controls.signals[s];
        double expected = row.at(name);
        double error = std::abs(signals.values[s] - expected) /
                       std::max(std::abs(expected), 1.0);
        if (error > worst) {
          worst = error;
          where = name + " at frame " + std::to_string(i);
        }
      }
    }

    // Postconditions.
    CAPTURE(worst, where);
    CHECK(worst < 1e-12);
  }
}

}  // namespace simon::model
