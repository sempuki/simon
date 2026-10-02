// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "model/control.hpp"
#include "model/units.hpp"

// Aerodynamics as data: a coefficient build-up of the kind JSBSim reads from
// an aircraft's XML. Each axis sums terms, and each term is a constant times
// some of the aerodynamic state times tables of it. tools/jsbsim/convert.py
// turns a JSBSim aircraft into these terms.
//
// Everything here is a plain number in SI units, because the terms are data:
// a term in a force axis yields newtons, a term in a moment axis newton
// meters. aero_loads turns the sums into typed forces and moments.
namespace simon::model {

// The variables a term may read. Each is in SI units, or a plain number.
enum class AeroVariable : std::uint8_t {
  DYNAMIC_PRESSURE,        // Pa.
  ALPHA,                   // Angle of attack, rad.
  BETA,                    // Sideslip, rad.
  MACH,                    //
  SPAN_OVER_TWICE_SPEED,   // b / 2V, s.
  CHORD_OVER_TWICE_SPEED,  // c / 2V, s.
  ROLL_RATE,               // Relative to the air, rad/s.
  PITCH_RATE,              //
  YAW_RATE,                //
  ALPHA_RATE,              // rad/s.
  LIFT_COEFFICIENT_SQUARED,
  HEIGHT_OVER_SPAN,  // Height above the ground over the wingspan.
  ELEVATOR,          // Surface deflections, rad.
  ELEVATOR_MAGNITUDE,
  LEFT_AILERON,
  RIGHT_AILERON,
  RUDDER,
  FLAPS,  // Extensions, from 0 to 1.
  GEAR,
  SPEEDBRAKE,
  SPOILERS,
  DENSITY_ALTITUDE,  // m.
  COUNT,
};

inline constexpr std::size_t AERO_VARIABLE_COUNT =
    static_cast<std::size_t>(AeroVariable::COUNT);

// The variable a converted aircraft names `name`, if any.
auto aero_variable_named(std::string_view name) -> std::optional<AeroVariable>;

// A value for each variable.
class AeroInputs final {
 public:
  auto operator[](AeroVariable variable) -> double& {
    return values_[static_cast<std::size_t>(variable)];
  }
  auto operator[](AeroVariable variable) const -> double {
    return values_[static_cast<std::size_t>(variable)];
  }

 private:
  std::array<double, AERO_VARIABLE_COUNT> values_{};
};

// A table of one variable, or of two: a row and a column.
struct AeroTable final {
  AeroVariable row = AeroVariable::ALPHA;
  std::optional<AeroVariable> column;
  std::variant<Table1<>, Table2<>> table;

  auto operator()(const AeroInputs& inputs) const -> double;
};

// A constant, times its factors, times its tables.
struct AeroTerm final {
  std::string name;
  double constant = 1.0;
  std::vector<AeroVariable> factors;
  std::vector<AeroTable> tables;

  auto operator()(const AeroInputs& inputs) const -> double;
};

// Drag, side force and lift are in wind axes; the moments are in body axes,
// about the aerodynamic reference point.
enum class AeroAxis : std::uint8_t {
  DRAG,
  SIDE,
  LIFT,
  ROLL,
  PITCH,
  YAW,
  COUNT,
};

inline constexpr std::size_t AERO_AXIS_COUNT =
    static_cast<std::size_t>(AeroAxis::COUNT);

auto aero_axis_named(std::string_view name) -> std::optional<AeroAxis>;

// The sum on each axis: drag, side force and lift in N, and the moments in
// N m.
using AeroSums = std::array<double, AERO_AXIS_COUNT>;

struct AeroModel final {
  std::array<std::vector<AeroTerm>, AERO_AXIS_COUNT> axes;

  auto operator()(const AeroInputs& inputs) const -> AeroSums;
};

// Body-axis aerodynamic force, and moment about the center of mass.
struct AeroLoads final {
  ForceVector force;
  Moment moment;
};

// The sums as body-axis loads, at angle of attack `alpha` and sideslip
// `beta`, with the aerodynamic reference point `reference` from the center of
// mass in body axes (x forward, y right, z down).
auto aero_loads(const AeroSums& sums, Angle alpha, Angle beta,
                const Displacement& reference) -> AeroLoads;

}  // namespace simon::model
