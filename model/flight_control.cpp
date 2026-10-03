// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/flight_control.hpp"

#include <algorithm>
#include <cmath>

namespace simon::model {

namespace {

constexpr std::array<std::string_view, FLIGHT_SIGNAL_COUNT> SIGNAL_NAMES{
    "elevator_command",
    "aileron_command",
    "rudder_command",
    "flaps_command",
    "gear_command",
    "speedbrake_command",
    "spoilers_command",
    "pitch_trim_command",
    "roll_trim_command",
    "yaw_trim_command",
    "throttle_command_0",
    "throttle_command_1",
    "throttle_command_2",
    "throttle_command_3",
    "mach",
    "roll_rate",
    "pitch_rate",
    "yaw_rate",
    "alpha",
    "beta",
    "calibrated_airspeed",
    "ground_speed",
    "body_velocity_x",
    "body_velocity_y",
    "pitch",
    "roll",
    "pilot_acceleration_y",
    "pilot_acceleration_z",
    "weight_on_wheels",
};

auto read(const FlightBlock::Input& input, const FlightSignals& signals)
    -> double {
  double value = signals.values[input.signal] * input.scale;
  return input.negated ? -value : value;
}

auto input_of(const FlightBlock& block, const FlightSignals& signals)
    -> double {
  if (block.inputs.empty()) {
    return 0.0;
  }
  return read(block.inputs.front(), signals);
}

auto value_of(const FlightBlock::Operand& operand, const FlightSignals& signals)
    -> double {
  return operand.input ? read(*operand.input, signals) : operand.value;
}

auto holds(const FlightBlock::Condition& condition,
           const FlightSignals& signals) -> bool {
  double left = signals.values[condition.signal];
  double right = value_of(condition.right, signals);
  using enum FlightBlock::Condition::Comparison;
  switch (condition.comparison) {
    case LT:
      return left < right;
    case LE:
      return left <= right;
    case GT:
      return left > right;
    case GE:
      return left >= right;
    case EQ:
      return left == right;
    case NE:
      return left != right;
  }
  return false;
}

// The value of the first test that holds, as JSBSim's switch finds it.
auto run_switch(const FlightBlock& block, const FlightSignals& signals)
    -> double {
  for (const FlightBlock::Test& test : block.tests) {
    auto check = [&](const FlightBlock::Condition& condition) {
      return holds(condition, signals);
    };
    bool passes = test.any ? std::ranges::any_of(test.conditions, check)
                           : std::ranges::all_of(test.conditions, check);
    if (passes) {
      return value_of(test.value, signals);
    }
  }
  return value_of(block.fallback, signals);
}

// JSBSim's PID (FGPID), with its state in three of the block's signals.
auto run_pid(const FlightBlock& block, double input, double dt,
             InOut<FlightSignals> signals) -> double {
  double& integral = signals->values[block.state];
  double& previous = signals->values[block.state + 1];
  double& before_previous = signals->values[block.state + 2];
  double trigger = block.trigger ? read(*block.trigger, *signals) : 0.0;

  double increment = 0.0;
  if (std::abs(trigger) < 0.000001) {
    using enum FlightBlock::Integrator;
    switch (block.integrator) {
      case NONE:
        break;
      case RECTANGULAR:
        increment = input;
        break;
      case TRAPEZOIDAL:
        increment = 0.5 * (input + previous);
        break;
      case ADAMS_BASHFORTH_2:
        increment = 1.5 * input - 0.5 * previous;
        break;
      case ADAMS_BASHFORTH_3:
        increment =
            (23.0 * input - 16.0 * previous + 5.0 * before_previous) / 12.0;
        break;
    }
  }
  if (trigger < 0.0) {
    integral = 0.0;
  }
  integral += block.ki * dt * increment;
  // A step of no time, as when an aircraft is set up, has no rate.
  double rate = dt > 0.0 ? (input - previous) / dt : 0.0;
  double output = block.kp * input + integral + block.kd * rate;
  before_previous = trigger < 0.0 ? 0.0 : previous;
  previous = input;
  return output;
}

auto run_function(const FlightBlock& block, const FlightSignals& signals)
    -> double {
  std::array<double, FUNCTION_STACK_SIZE> stack{};
  std::size_t size = 0;
  auto pop = [&]() { return stack[--size]; };
  for (const FlightBlock::Operation& operation : block.operations) {
    using enum FlightBlock::Operation::Kind;
    switch (operation.kind) {
      case PUSH:
        stack[size++] = read(operation.input, signals);
        break;
      case CONSTANT:
        stack[size++] = operation.value;
        break;
      case SUM: {
        double sum = 0.0;
        for (std::size_t i = 0; i < operation.count; ++i) {
          sum += pop();
        }
        stack[size++] = sum;
        break;
      }
      case PRODUCT: {
        double product = 1.0;
        for (std::size_t i = 0; i < operation.count; ++i) {
          product *= pop();
        }
        stack[size++] = product;
        break;
      }
      case DIFFERENCE: {
        double right = pop();
        stack[size - 1] -= right;
        break;
      }
      case QUOTIENT: {
        double right = pop();
        stack[size - 1] /= right;
        break;
      }
      case SIN:
        stack[size - 1] = std::sin(stack[size - 1]);
        break;
      case COS:
        stack[size - 1] = std::cos(stack[size - 1]);
        break;
      case TAN:
        stack[size - 1] = std::tan(stack[size - 1]);
        break;
      case ABS:
        stack[size - 1] = std::abs(stack[size - 1]);
        break;
    }
  }
  return size > 0 ? stack[size - 1] : 0.0;
}

// Where a kinematic block starts a step: from its output, if it has one, as
// JSBSim's does, so that a block that writes the same output moves it.
auto start_of(const FlightBlock& block, const FlightSignals& signals)
    -> double {
  if (block.output) {
    return signals.values[*block.output] / block.output_scale;
  }
  return signals.values[block.signal];
}

// Moves `output` toward `input` through the detents, for `dt`, at the rate
// of each interval it crosses.
auto traverse(const FlightBlock& block, double output, double input, double dt)
    -> double {
  const std::vector<double>& detents = block.detents;
  input = std::clamp(input, detents.front(), detents.back());
  double left = dt;
  // A thousandth of a nanometer short of the target is there.
  while (left > 0.0 && std::abs(input - output) > 1e-12) {
    // The interval the output moves through next.
    std::size_t i = 1;
    while (i + 1 < detents.size() &&
           (input < output ? detents[i] < output : detents[i] <= output)) {
      ++i;
    }
    if (block.times[i] <= 0.0) {
      return input;
    }
    double rate = (detents[i] - detents[i - 1]) / block.times[i];
    double stop = std::clamp(input, detents[i - 1], detents[i]);
    double needed = std::abs(stop - output) / rate;
    if (left < needed) {
      return output + (output < input ? left * rate : -left * rate);
    }
    output = stop;
    left -= needed;
  }
  return output;
}

auto write(const FlightBlock& block, InOut<FlightSignals> signals, double value)
    -> void {
  if (block.clip) {
    value = std::clamp(value, block.clip->first, block.clip->second);
  }
  signals->values[block.signal] = value;
  if (block.output) {
    signals->values[*block.output] = value * block.output_scale;
  }
}

}  // namespace

auto find_flight_signal(std::string_view name) -> std::optional<FlightSignal> {
  auto found = std::ranges::find(SIGNAL_NAMES, name);
  if (found == SIGNAL_NAMES.end()) {
    return std::nullopt;
  }
  return static_cast<FlightSignal>(found - SIGNAL_NAMES.begin());
}

auto flight_signal_name(FlightSignal signal) -> std::string_view {
  return SIGNAL_NAMES[index_of(signal)];
}

auto find_signal(const FlightControlData& controls, std::string_view name)
    -> std::optional<std::size_t> {
  auto found = std::ranges::find(controls.signals, name);
  if (found == controls.signals.end()) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(found - controls.signals.begin());
}

namespace {

// Runs every block once, in order: over `seconds`, or, if `steady`, as each
// block stands with its inputs held, with each kinematic block at its input
// and each PID seeing no rate, its integral unchanged.
auto run_blocks(const FlightControlData& controls, InOut<FlightSignals> signals,
                double seconds, bool steady) -> void {
  for (std::size_t i = 0; i < controls.throttles.size(); ++i) {
    signals->values[controls.throttles[i]] =
        signals->values[index_of(FlightSignal::THROTTLE_COMMAND_0) + i];
  }
  for (const FlightBlock& block : controls.blocks) {
    double input = input_of(block, *signals);
    double value = 0.0;
    switch (block.kind) {
      case FlightBlock::Kind::SUMMER:
        for (const FlightBlock::Input& each : block.inputs) {
          value += read(each, *signals);
        }
        value += block.bias;
        break;
      case FlightBlock::Kind::GAIN:
        value = block.gain * input;
        break;
      case FlightBlock::Kind::SCHEDULED_GAIN:
        value = block.gain *
                (*block.schedule)(signals->values[block.schedule_signal]) *
                input;
        break;
      case FlightBlock::Kind::SURFACE_SCALE: {
        auto [in_low, in_high] = block.domain;
        auto [out_low, out_high] = block.range;
        if (block.zero_centered) {
          value = input == 0.0  ? 0.0
                  : input > 0.0 ? input / in_high * out_high
                                : input / in_low * out_low;
        } else {
          value = out_low +
                  (input - in_low) / (in_high - in_low) * (out_high - out_low);
        }
        value *= block.gain;
        break;
      }
      case FlightBlock::Kind::KINEMATIC:
        if (block.scale) {
          input *= block.detents.back();
        }
        value =
            steady
                ? std::clamp(input, block.detents.front(), block.detents.back())
                : traverse(block, start_of(block, *signals), input, seconds);
        break;
      case FlightBlock::Kind::SWITCH:
        value = run_switch(block, *signals);
        break;
      case FlightBlock::Kind::PID:
        if (steady) {
          signals->values[block.state + 1] = input;
          signals->values[block.state + 2] = input;
          value = block.kp * input + signals->values[block.state];
        } else {
          value = run_pid(block, input, seconds, signals);
        }
        break;
      case FlightBlock::Kind::FUNCTION:
        value = run_function(block, *signals);
        break;
    }
    write(block, signals, value);
  }
}

}  // namespace

auto run_flight_controls(const FlightControlData& controls,
                         InOut<FlightSignals> signals, Time dt) -> void {
  run_blocks(controls, signals, dt.numerical_value_in(second), false);
}

auto settle_flight_controls(const FlightControlData& controls,
                            InOut<FlightSignals> signals) -> void {
  run_blocks(controls, signals, 0.0, true);
}

}  // namespace simon::model
