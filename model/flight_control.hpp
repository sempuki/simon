// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <bitset>
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
// commands and state it is given, and whatever the blocks name, such as the
// surface positions the aerodynamics read. A signal keeps its value from step
// to step, so a block that reads one a later block writes reads the step
// before's.
namespace simon::model {

// The signals every flight control system has, in a fixed place: those the
// aircraft is given.
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
  THROTTLE_COMMAND_0,  // Each engine's, from 0 to 1.
  THROTTLE_COMMAND_1,
  THROTTLE_COMMAND_2,
  THROTTLE_COMMAND_3,
  // State, read at the step's start.
  MACH,
  ROLL_RATE,  // Relative to the air, rad/s.
  PITCH_RATE,
  YAW_RATE,
  ALPHA,  // rad.
  BETA,
  CALIBRATED_AIRSPEED,  // m/s.
  GROUND_SPEED,         // Over the ground, horizontally.
  BODY_VELOCITY_X,      // Along body axes.
  BODY_VELOCITY_Y,
  PITCH,  // Euler angles from the local north-east-down frame, rad.
  ROLL,
  PILOT_ACCELERATION_Y,  // At the pilot, along body axes, less gravity, in g.
  PILOT_ACCELERATION_Z,
  WEIGHT_ON_WHEELS,  // Always 0: the aircraft is always flying.
  COUNT,
};

inline constexpr std::size_t FLIGHT_SIGNAL_COUNT =
    static_cast<std::size_t>(FlightSignal::COUNT);

// The most signals an aircraft may have, its blocks' included.
inline constexpr std::size_t MAX_FLIGHT_SIGNALS = 128;

// The most values a function block's operations hold at once.
inline constexpr std::size_t FUNCTION_STACK_SIZE = 16;

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
//                  `times[i]` to go from detent i - 1 to detent i; it moves
//                  from its output, if it has one, which another block may
//                  have written
//   SWITCH         the value of the first of `tests` whose conditions hold,
//                  or `fallback` if none does
//   PID            `kp` times its input, plus its integral, plus `kd` times
//                  its input's rate; the integral grows by `ki` times the
//                  input, integrated by `integrator`, only while `trigger`
//                  is zero, and resets while `trigger` is negative
//   FUNCTION       `operations` run on a stack, the last one's result
//
// Then the output is clipped to `clip`, if there is one, and written to the
// block's own signal and, times `output_scale`, to `output`, if it has one.
//
// A block works in its JSBSim aircraft's units. Signals are in SI, so an
// input whose units differ has a `scale` that converts it back.
struct FlightBlock final {
  enum class Kind : std::uint8_t {
    SUMMER,
    GAIN,
    SCHEDULED_GAIN,
    SURFACE_SCALE,
    KINEMATIC,
    SWITCH,
    PID,
    FUNCTION,
  };

  struct Input final {
    std::size_t signal = 0;
    bool negated = false;
    double scale = 1.0;
  };

  // A constant, or an input.
  struct Operand final {
    double value = 0.0;
    std::optional<Input> input;
  };

  // A signal compared with an operand, both in SI.
  struct Condition final {
    enum class Comparison : std::uint8_t { LT, LE, GT, GE, EQ, NE };

    std::size_t signal = 0;
    Comparison comparison = Comparison::EQ;
    Operand right;
  };

  // Holds if all its conditions do, or if `any`, if one does.
  struct Test final {
    bool any = false;
    Operand value;
    std::vector<Condition> conditions;
  };

  enum class Integrator : std::uint8_t {
    NONE,
    RECTANGULAR,
    TRAPEZOIDAL,
    ADAMS_BASHFORTH_2,
    ADAMS_BASHFORTH_3,
  };

  // One step of a function: push an input or a constant, or replace the top
  // `count` values, or the top one or two, with their result.
  struct Operation final {
    enum class Kind : std::uint8_t {
      PUSH,
      CONSTANT,
      SUM,
      PRODUCT,
      DIFFERENCE,
      QUOTIENT,
      SIN,
      COS,
      TAN,
      ABS,
    };

    Kind kind = Kind::PUSH;
    Input input;
    double value = 0.0;
    std::size_t count = 0;
  };

  Kind kind = Kind::SUMMER;
  std::string name;
  std::size_t signal = 0;  // Its own.
  std::vector<Input> inputs;
  std::optional<std::size_t> output;
  double output_scale = 1.0;
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

  Operand fallback;
  std::vector<Test> tests;

  std::optional<Input> trigger;
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  Integrator integrator = Integrator::NONE;
  // The first of three signals that hold its state: the integral, and its
  // input one and two steps before.
  std::size_t state = 0;

  std::vector<Operation> operations;
};

struct FlightControlData final {
  std::vector<std::string> signals;  // By index: the fixed ones first.
  std::vector<FlightBlock> blocks;
  // Each engine's throttle, `throttle_<n>`: its command, unless a block
  // writes it.
  std::vector<std::size_t> throttles;
  // The fixed signals the blocks read, so an aircraft finds only those.
  std::bitset<FLIGHT_SIGNAL_COUNT> read;

  auto reads(FlightSignal signal) const -> bool {
    return read[index_of(signal)];
  }
};

// The index of the signal `controls` names `name`, if any.
auto find_signal(const FlightControlData& controls, std::string_view name)
    -> std::optional<std::size_t>;

// Sets each engine's throttle to its command, as JSBSim does at the start of
// a frame, then runs every block once, in order, over `dt`.
auto run_flight_controls(const FlightControlData& controls,
                         InOut<FlightSignals> signals, Time dt) -> void;

// Runs every block once, in order, as it stands when its inputs hold still:
// each kinematic block where its input sends it, as if it had all the time it
// needs, and each PID seeing no rate, its integral unchanged. Running it again
// settles blocks that read later blocks: for an aircraft that starts with its
// surfaces, flaps and gear where its commands and state put them.
auto settle_flight_controls(const FlightControlData& controls,
                            InOut<FlightSignals> signals) -> void;

}  // namespace simon::model
