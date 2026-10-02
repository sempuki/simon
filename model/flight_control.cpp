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
    "mach",
    "roll_rate",
    "pitch_rate",
    "yaw_rate",
    "alpha",
    "beta",
};

auto input_of(const FlightBlock& block, const FlightSignals& signals)
    -> double {
  if (block.inputs.empty()) {
    return 0.0;
  }
  const FlightBlock::Input& input = block.inputs.front();
  double value = signals.values[input.signal];
  return input.negated ? -value : value;
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
    signals->values[*block.output] = value;
  }
}

}  // namespace

auto flight_signal_named(std::string_view name) -> std::optional<FlightSignal> {
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

auto run_flight_controls(const FlightControlData& controls,
                         InOut<FlightSignals> signals, Time dt) -> void {
  double seconds = dt.numerical_value_in(second);
  for (const FlightBlock& block : controls.blocks) {
    double input = input_of(block, *signals);
    double value = 0.0;
    switch (block.kind) {
      case FlightBlock::Kind::SUMMER:
        for (const FlightBlock::Input& each : block.inputs) {
          double term = signals->values[each.signal];
          value += each.negated ? -term : term;
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
        value = traverse(block, signals->values[block.signal], input, seconds);
        break;
    }
    write(block, signals, value);
  }
}

auto settle_flight_controls(const FlightControlData& controls,
                            InOut<FlightSignals> signals) -> void {
  for (const FlightBlock& block : controls.blocks) {
    if (block.kind == FlightBlock::Kind::KINEMATIC) {
      double input = input_of(block, *signals);
      if (block.scale) {
        input *= block.detents.back();
      }
      write(block, signals,
            std::clamp(input, block.detents.front(), block.detents.back()));
    }
  }
}

}  // namespace simon::model
