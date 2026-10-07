// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <type_traits>
#include <utility>
#include <vector>

#include "base/core.hpp"
#include "core/argument.hpp"
#include "core/units.hpp"

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
// follows the input at once. It is the lag's zero-order-hold equivalent (see
// model/REFERENCES.md).
template <typename ValueType>
auto lag(const ValueType& value, const ValueType& input, Time time_constant,
         Time dt) -> ValueType {
  if (time_constant <= 0.0 * second) {
    return input;
  }
  double fraction = 1.0 - std::exp(-number_of(dt / time_constant));
  return value + (input - value) * fraction;
}

namespace internal {

// `part / whole` as a plain number, for numbers or quantities alike.
template <typename ValueType>
auto fraction(const ValueType& part, const ValueType& whole) -> double {
  if constexpr (std::is_arithmetic_v<ValueType>) {
    return part / whole;
  } else {
    return number_of(part / whole);
  }
}

// `value` as an `OutputType`. A dimensionless quantity becomes a plain number
// when the output is one, such as a throttle.
template <typename OutputType, typename ValueType>
auto output_of(const ValueType& value) -> OutputType {
  if constexpr (std::is_arithmetic_v<OutputType> &&
                !std::is_arithmetic_v<ValueType>) {
    return number_of(value);
  } else {
    return OutputType{value};
  }
}

// `value` clamped to [low, high], for numbers or quantities alike; see
// clamp in units.hpp for why quantities do not use std::clamp.
template <typename ValueType>
auto clamp(const ValueType& value, const ValueType& low, const ValueType& high)
    -> ValueType {
  if constexpr (std::is_arithmetic_v<ValueType>) {
    return std::clamp(value, low, high);
  } else {
    return simon::clamp(value, low, high);
  }
}

// The zero of a number or a quantity.
template <typename ValueType>
constexpr auto zero_of() -> ValueType {
  if constexpr (std::is_arithmetic_v<ValueType>) {
    return ValueType{0};
  } else {
    return ValueType::zero();
  }
}

}  // namespace internal

// `value` moved toward `target` by at most `most`, which must not be negative:
// a rate limit, given the most the value may change this step.
template <typename ValueType>
auto approach(const ValueType& value, const ValueType& target,
              const ValueType& most) -> ValueType {
  return value + internal::clamp(target - value, -most, most);
}

// A proportional-integral controller's gains and output limits, for an error
// of `ErrorType` and an output of `OutputType`: a throttle (a plain number)
// for a speed error, say. The gains carry the units that turn one into the
// other. The integral is the caller's state.
template <typename ErrorType, typename OutputType = double>
struct PiGains final {
  using Proportional = decltype(OutputType{} / ErrorType{});
  using Integral = decltype(OutputType{} / (ErrorType{} * Time{}));

  Proportional proportional = internal::zero_of<Proportional>();
  Integral integral = internal::zero_of<Integral>();
  OutputType low = internal::zero_of<OutputType>();   // The lowest output.
  OutputType high = internal::zero_of<OutputType>();  // The highest output.
};

// The output of a PI controller for `error`, integrating over `dt` into
// `integral`. The integral stops growing while the output is held at a limit
// in the error's direction, so it does not wind up: conditional integration
// (Astrom and Hagglund; see model/REFERENCES.md).
template <typename ErrorType, typename OutputType>
auto pi_control(ErrorType error, const PiGains<ErrorType, OutputType>& gains,
                Time dt, InOut<OutputType> integral) -> OutputType {
  constexpr ErrorType ZERO = internal::zero_of<ErrorType>();
  auto proportional =
      internal::output_of<OutputType>(gains.proportional * error);

  OutputType unlimited = proportional + *integral;
  bool saturated = (unlimited >= gains.high && error > ZERO) ||
                   (unlimited <= gains.low && error < ZERO);
  if (!saturated) {
    *integral += internal::output_of<OutputType>(gains.integral * error * dt);
    *integral = internal::clamp(*integral, gains.low, gains.high);
  }

  return internal::clamp(proportional + *integral, gains.low, gains.high);
}

// A function of one variable, linear between breakpoints and constant beyond
// the first and last: lift coefficient by angle of attack, say. Breakpoints
// must be strictly increasing. Either type may be a plain number or a
// quantity.
template <typename BreakpointType = double, typename ValueType = double>
class Table1 final {
 public:
  // Needs at least one breakpoint, strictly increasing, and a value for each.
  Table1(std::vector<BreakpointType> breakpoints, std::vector<ValueType> values)
      : breakpoints_{std::move(breakpoints)}, values_{std::move(values)} {
    CHECK_PRECONDITION(!breakpoints_.empty());
    CHECK_PRECONDITION(values_.size() == breakpoints_.size());
    CHECK_PRECONDITION(
        std::ranges::adjacent_find(breakpoints_, std::greater_equal{}) ==
        breakpoints_.end());
  }

  auto operator()(const BreakpointType& x) const -> ValueType {
    auto [i, weight] = locate(breakpoints_, x);
    if (weight == 0.0) {
      return values_[i];
    }
    return values_[i] + (values_[i + 1] - values_[i]) * weight;
  }

  // Locates `x` among `breakpoints`: the breakpoint at or below it, and the
  // fraction of the way to the next one, in [0, 1). Clamped to the ends. NaN
  // compares false with every breakpoint, so the first test is written to
  // take it to the first breakpoint, where the rest of the search would take
  // it past the last.
  static auto locate(const std::vector<BreakpointType>& breakpoints,
                     const BreakpointType& x)
      -> std::pair<std::size_t, double> {
    if (!(x > breakpoints.front())) {
      return {0, 0.0};
    }
    if (x >= breakpoints.back()) {
      return {breakpoints.size() - 1, 0.0};
    }

    auto above = std::upper_bound(breakpoints.begin(), breakpoints.end(), x);
    auto i = static_cast<std::size_t>(above - breakpoints.begin()) - 1;
    return {i, internal::fraction(x - breakpoints[i],
                                  breakpoints[i + 1] - breakpoints[i])};
  }

 private:
  std::vector<BreakpointType> breakpoints_;
  std::vector<ValueType> values_;
};

// A function of two variables, bilinear between breakpoints and clamped to
// the edges: drag coefficient by Mach number and lift coefficient, say.
// `values` holds a row per row breakpoint, each with a value per column
// breakpoint.
template <typename RowType = double, typename ColumnType = double,
          typename ValueType = double>
class Table2 final {
 public:
  // Needs at least one row and column breakpoint, each strictly increasing,
  // and a value for each pair.
  Table2(std::vector<RowType> rows, std::vector<ColumnType> columns,
         std::vector<ValueType> values)
      : rows_{std::move(rows)},
        columns_{std::move(columns)},
        values_{std::move(values)} {
    CHECK_PRECONDITION(!rows_.empty() && !columns_.empty());
    CHECK_PRECONDITION(values_.size() == rows_.size() * columns_.size());
    CHECK_PRECONDITION(
        std::ranges::adjacent_find(rows_, std::greater_equal{}) == rows_.end());
    CHECK_PRECONDITION(std::ranges::adjacent_find(
                           columns_, std::greater_equal{}) == columns_.end());
  }

  auto operator()(const RowType& row, const ColumnType& column) const
      -> ValueType {
    auto [r, row_weight] = Table1<RowType, ValueType>::locate(rows_, row);
    auto [c, column_weight] =
        Table1<ColumnType, ValueType>::locate(columns_, column);

    auto at = [&](std::size_t i, std::size_t j) {
      return values_[i * columns_.size() + j];
    };
    auto along_row = [&](std::size_t i) {
      return column_weight == 0.0
                 ? at(i, c)
                 : at(i, c) + (at(i, c + 1) - at(i, c)) * column_weight;
    };

    ValueType low = along_row(r);
    return row_weight == 0.0 ? low
                             : low + (along_row(r + 1) - low) * row_weight;
  }

 private:
  std::vector<RowType> rows_;
  std::vector<ColumnType> columns_;
  std::vector<ValueType> values_;
};

}  // namespace simon::model
