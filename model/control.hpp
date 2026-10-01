// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

#include "base/core.hpp"
#include "model/units.hpp"

// Control blocks: the pieces flight control laws are built from, as free
// functions and small value types. Each is exact or stable at any step, so a
// simulation can take large steps.
namespace simon::model {

// Moves `value` toward `input` as a first-order lag with `time_constant`, by
// its exact solution over `dt`:
//
//   value += (input - value) * (1 - e^(-dt / time_constant))
//
// Stable at any step: it never overshoots `input`. A zero time constant
// follows the input at once.
template <typename ValueType>
auto lag(const ValueType& value, const ValueType& input, Time time_constant,
         Time dt) -> ValueType {
  if (time_constant <= 0.0 * second) {
    return input;
  }
  double fraction =
      1.0 - std::exp(-(dt / time_constant).numerical_value_in(units::one));
  return value + (input - value) * fraction;
}

// `value` moved toward `target` by at most `most`, which must not be negative:
// a rate limit, given the most the value may change this step.
template <typename ValueType>
auto approach(const ValueType& value, const ValueType& target,
              const ValueType& most) -> ValueType {
  return value + std::clamp(target - value, -most, most);
}

// A proportional-integral controller's gains and output limits. The integral
// is the caller's state.
struct PiGains final {
  double proportional = 0.0;
  double integral = 0.0;  // Per second.
  double low = 0.0;       // The lowest output.
  double high = 1.0;      // The highest output.
};

// The output of a PI controller for `error`, integrating over `dt` into
// `integral`. The integral stops growing while the output is held at a limit
// in the error's direction, so it does not wind up.
inline auto pi_control(double error, const PiGains& gains, Time dt,
                       lib::InOut<double> integral) -> double {
  double unlimited = gains.proportional * error + *integral;
  bool saturated = (unlimited >= gains.high && error > 0.0) ||
                   (unlimited <= gains.low && error < 0.0);
  if (!saturated) {
    *integral += gains.integral * error * dt.numerical_value_in(second);
    *integral = std::clamp(*integral, gains.low, gains.high);
  }
  return std::clamp(gains.proportional * error + *integral, gains.low,
                    gains.high);
}

// A function of one variable, linear between breakpoints and constant beyond
// the first and last. Breakpoints must be strictly increasing.
class Table1 final {
 public:
  // Needs at least one breakpoint, strictly increasing, and a value for each.
  Table1(std::vector<double> breakpoints, std::vector<double> values)
      : breakpoints_{std::move(breakpoints)}, values_{std::move(values)} {
    CHECK_PRECONDITION(!breakpoints_.empty());
    CHECK_PRECONDITION(values_.size() == breakpoints_.size());
    CHECK_PRECONDITION(
        std::ranges::adjacent_find(breakpoints_, std::greater_equal{}) ==
        breakpoints_.end());
  }

  auto operator()(double x) const -> double {
    auto [i, weight] = locate(breakpoints_, x);
    if (weight == 0.0) {
      return values_[i];
    }
    return values_[i] + (values_[i + 1] - values_[i]) * weight;
  }

  // Where `x` falls among `breakpoints`: the breakpoint at or below it, and
  // how far toward the next one, in [0, 1). Clamped to the ends.
  static auto locate(const std::vector<double>& breakpoints, double x)
      -> std::pair<std::size_t, double> {
    if (x <= breakpoints.front()) {
      return {0, 0.0};
    }
    if (x >= breakpoints.back()) {
      return {breakpoints.size() - 1, 0.0};
    }
    auto above = std::upper_bound(breakpoints.begin(), breakpoints.end(), x);
    auto i = static_cast<std::size_t>(above - breakpoints.begin()) - 1;
    return {i, (x - breakpoints[i]) / (breakpoints[i + 1] - breakpoints[i])};
  }

 private:
  std::vector<double> breakpoints_;
  std::vector<double> values_;
};

// A function of two variables, bilinear between breakpoints and clamped to
// the edges. `values` holds a row per row breakpoint, each with a value per
// column breakpoint.
class Table2 final {
 public:
  // Needs at least one row and column breakpoint, each strictly increasing,
  // and a value for each pair.
  Table2(std::vector<double> rows, std::vector<double> columns,
         std::vector<double> values)
      : rows_{std::move(rows)},
        columns_{std::move(columns)},
        values_{std::move(values)} {
    CHECK_PRECONDITION(!rows_.empty() && !columns_.empty());
    CHECK_PRECONDITION(values_.size() == rows_.size() * columns_.size());
    CHECK_PRECONDITION(std::ranges::adjacent_find(rows_, std::greater_equal{}) ==
                       rows_.end());
    CHECK_PRECONDITION(
        std::ranges::adjacent_find(columns_, std::greater_equal{}) ==
        columns_.end());
  }

  auto operator()(double row, double column) const -> double {
    auto [r, row_weight] = Table1::locate(rows_, row);
    auto [c, column_weight] = Table1::locate(columns_, column);
    auto at = [&](std::size_t i, std::size_t j) {
      return values_[i * columns_.size() + j];
    };
    auto along_row = [&](std::size_t i) {
      return column_weight == 0.0
                 ? at(i, c)
                 : at(i, c) + (at(i, c + 1) - at(i, c)) * column_weight;
    };
    double low = along_row(r);
    return row_weight == 0.0 ? low : low + (along_row(r + 1) - low) * row_weight;
  }

 private:
  std::vector<double> rows_;
  std::vector<double> columns_;
  std::vector<double> values_;
};

}  // namespace simon::model
