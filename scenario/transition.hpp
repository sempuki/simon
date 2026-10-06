// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows esmini 3.8.2, Copyright (c) partners of Simulation Scenarios,
// MPL-2.0; translated to C++ and changed. See NOTICE.md.

#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

#include "scenario/openscenario.hpp"

// A value moving from `start` to `target` as a parameter x runs from 0 to its
// end c, by OpenSCENARIO's transition shapes (see model/REFERENCES.md), with
// b = target - start:
//
//   step         target, at once
//   linear       start + b x / c
//   cubic        start + 3 b (x / c)^2 - 2 b (x / c)^3
//   sinusoidal   start - b (cos(pi x / c) - 1) / 2
//
// The parameter is time, distance, or time at a rate, as an action's dynamics
// say. With no length to run over, every shape is a step.
namespace simon::scenario {

struct Transition final {
  // Whether the parameter has reached its end.
  auto done() const -> bool { return parameter >= end - 1e-12; }

  // Moves the parameter on by `by`, within [0, end].
  auto advance(double by) -> void {
    parameter = std::clamp(parameter + by, 0.0, end);
  }

  // The value at the parameter, by `shape`, or the transition's own.
  auto evaluate() const -> double { return evaluate(shape); }
  auto evaluate(DynamicsShape as) const -> double {
    if (as == DynamicsShape::STEP || end < 1e-12) {
      return target;
    }
    double b = target - start;
    double x = parameter / end;
    switch (as) {
      case DynamicsShape::LINEAR:
        return start + b * x;
      case DynamicsShape::CUBIC:
        return start + b * x * x * (3.0 - 2.0 * x);
      case DynamicsShape::SINUSOIDAL:
        return start - b * (std::cos(std::numbers::pi * x) - 1.0) / 2.0;
      case DynamicsShape::STEP:
        break;
    }
    return target;
  }

  // The peak of the value's rate of change over the transition, |b| / c
  // times the shape's factor: 1 linear, 1.5 cubic, pi / 2 sinusoidal.
  static auto compute_peak_factor(DynamicsShape shape) -> double {
    switch (shape) {
      case DynamicsShape::LINEAR:
        return 1.0;
      case DynamicsShape::CUBIC:
        return 1.5;
      case DynamicsShape::SINUSOIDAL:
        return 0.5 * std::numbers::pi;
      case DynamicsShape::STEP:
        break;
    }
    return 0.0;
  }

  // The parameter's end at which the value's rate peaks at `rate`.
  auto compute_end_at_peak(double rate) const -> double {
    return compute_peak_factor(shape) * std::abs(target - start) /
           std::max(std::abs(rate), 1e-12);
  }

  // Lengthens the transition so that its rate peaks at no more than
  // `limit`.
  auto stretch_to(double limit) -> void {
    if (shape == DynamicsShape::STEP || end < 1e-12) {
      return;
    }
    double peak = compute_peak_factor(shape) * std::abs(target - start) / end;
    if (peak > std::abs(limit)) {
      end = compute_end_at_peak(limit);
    }
  }

  // The value's rate of change with the parameter, at the parameter.
  auto compute_slope() const -> double {
    double b = target - start;
    if (shape == DynamicsShape::STEP || end < 1e-12) {
      return b == 0.0 ? 0.0 : std::copysign(1e10, b);
    }
    double x = parameter / end;
    switch (shape) {
      case DynamicsShape::LINEAR:
        return b / end;
      case DynamicsShape::CUBIC:
        return 6.0 * b * x * (1.0 - x) / end;
      case DynamicsShape::SINUSOIDAL:
        return std::numbers::pi * b * std::sin(std::numbers::pi * x) /
               (2.0 * end);
      case DynamicsShape::STEP:
        break;
    }
    return 0.0;
  }

  DynamicsShape shape = DynamicsShape::STEP;
  double start = 0.0;
  double target = 0.0;
  double end = 0.0;  // c.
  double parameter = 0.0;
};

}  // namespace simon::scenario
