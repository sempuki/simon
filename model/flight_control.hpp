// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "model/control.hpp"
#include "model/units.hpp"

// A flight control system as data: blocks that turn the pilot's commands and
// the aircraft's state into control surface positions, run in order every
// step, as JSBSim's flight control channels are. tools/jsbsim/convert.py
// writes them from a JSBSim aircraft's <flight_control>.
//
// Blocks read and write signals, plain numbers each aircraft keeps: the
// commands and state it is given, the surfaces it sets, and whatever the
// blocks name in between. A signal keeps its value from step to step, so a
// block that reads one a later block writes reads the step before's.
namespace simon::model {

// The signals every flight control system has, in a fixed place.
enum class FlightSignal : std::uint8_t {
  // Commands, from -1 to 1 or 0 to 1.
  ELEVATOR_COMMAND,
  AILERON_COMMAND,
  RUDDER_COMMAND,
  FLAPS_COMMAND,
  GEAR_COMMAND,
  SPEEDBRAKE_COMMAND,
  SPOILERS_COMMAND,
  PITCH_TRIM_COMMAND,
  ROLL_TRIM_COMMAND,
  YAW_TRIM_COMMAND,
  // State, read at the step's start.
  MACH,
  ROLL_RATE,  // Relative to the air, rad/s.
  PITCH_RATE,
  YAW_RATE,
  ALPHA,  // rad.
  BETA,
  // Surfaces: deflections in radians, extensions from 0 to 1.
  ELEVATOR,
  LEFT_AILERON,
  RIGHT_AILERON,
  RUDDER,
  FLAPS,
  GEAR,
  SPEEDBRAKE,
  SPOILERS,
  COUNT,
};

inline constexpr std::size_t FLIGHT_SIGNAL_COUNT =
    static_cast<std::size_t>(FlightSignal::COUNT);

// The most signals an aircraft may have, its blocks' included.
inline constexpr std::size_t MAX_FLIGHT_SIGNALS = 64;

// The fixed signal `name` names, if any, and the name of each.
auto flight_signal_named(std::string_view name) -> std::optional<FlightSignal>;
auto flight_signal_name(FlightSignal signal) -> std::string_view;

inline constexpr auto index_of(FlightSignal signal) -> std::size_t {
  return static_cast<std::size_t>(signal);
}

// The value of every signal, for one aircraft.
struct FlightSignals final {
  std::array<double, MAX_FLIGHT_SIGNALS> values{};

  auto operator[](FlightSignal signal) -> double& {
    return values[index_of(signal)];
  }
  auto operator[](FlightSignal signal) const -> double {
    return values[index_of(signal)];
  }
};

// One block. Each kind reads the fields it needs:
//
//   SUMMER         the sum of its inputs, plus `bias`
//   GAIN           its input times `gain`
//   SCHEDULED_GAIN its input times `gain` times `schedule` of a signal
//   SURFACE_SCALE  its input mapped from `domain` to `range`, either
//                  linearly or, if zero-centered, each side of zero
//                  separately, times `gain`
//   KINEMATIC      moves toward its input, scaled by the last detent unless
//                  `scale` is off, through `detents`, taking
//                  `times[i]` to go from detent i - 1 to detent i
//
// Then the output is clipped to `clip`, if there is one, and written to the
// block's own signal and to `output`, if it has one.
struct FlightBlock final {
  enum class Kind : std::uint8_t {
    SUMMER,
    GAIN,
    SCHEDULED_GAIN,
    SURFACE_SCALE,
    KINEMATIC,
  };

  struct Input final {
    std::size_t signal = 0;
    bool negated = false;
  };

  Kind kind = Kind::SUMMER;
  std::string name;
  std::size_t signal = 0;  // Its own.
  std::vector<Input> inputs;
  std::optional<std::size_t> output;
  std::optional<std::pair<double, double>> clip;

  double bias = 0.0;
  double gain = 1.0;
  std::size_t schedule_signal = 0;
  std::optional<Table1<>> schedule;
  std::pair<double, double> domain{-1.0, 1.0};
  std::pair<double, double> range{-1.0, 1.0};
  std::vector<double> detents;
  std::vector<double> times;
  bool zero_centered = true;
  bool scale = true;
};

struct FlightControlData final {
  std::vector<std::string> signals;  // By index: the fixed ones first.
  std::vector<FlightBlock> blocks;
};

// Runs every block once, in order, over `dt`.
auto run_flight_controls(const FlightControlData& controls,
                         FlightSignals& signals, Time dt) -> void;

// Sets every kinematic block, and its outputs, to where its input sends it,
// as if it had all the time it needs: for an aircraft that starts with its
// flaps and gear where its commands put them.
auto settle_flight_controls(const FlightControlData& controls,
                            FlightSignals& signals) -> void;

}  // namespace simon::model
