// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "model/control.hpp"
#include "model/units.hpp"

// Aerodynamics as data: a coefficient build-up of the kind JSBSim reads from
// an aircraft's XML. Each axis sums terms, and each term is a constant times
// some of its inputs times tables of them. An input is a variable of the
// aerodynamic state, or one of the aircraft's flight control signals, such as
// a surface's deflection. tools/jsbsim/convert.py turns a JSBSim aircraft into
// these terms.
//
// Everything here is a plain number in SI units, because the terms are data:
// a term in a force axis yields newtons, a term in a moment axis newton
// meters. compute_aero_loads turns the sums into typed forces and moments.
namespace simon::model {

// The aerodynamic state's variables. Each is in SI units, or a plain number.
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
  DENSITY_ALTITUDE,  // m.
  COUNT,
};

inline constexpr std::size_t AERO_VARIABLE_COUNT =
    static_cast<std::size_t>(AeroVariable::COUNT);

// The variable a converted aircraft names `name`, if any.
auto aero_variable_named(std::string_view name) -> std::optional<AeroVariable>;

// The most flight control signals an aircraft's aerodynamics may read.
inline constexpr std::size_t MAX_AERO_SIGNALS = 16;

// A flight control signal the aerodynamics read, by its index, or its
// magnitude.
struct AeroSignal final {
  std::size_t signal = 0;
  bool magnitude = false;

  friend auto operator==(const AeroSignal&, const AeroSignal&)
      -> bool = default;
};

// An input a term reads: a variable, or the aerodynamics' signal `i` at
// AERO_VARIABLE_COUNT + i.
struct AeroInput final {
  std::uint8_t index = 0;

  friend auto operator==(AeroInput, AeroInput) -> bool = default;
};

inline constexpr auto aero_input(AeroVariable variable) -> AeroInput {
  return AeroInput{.index = static_cast<std::uint8_t>(variable)};
}

// A value for each input.
class AeroInputs final {
 public:
  auto operator[](AeroVariable variable) -> double& {
    return values_[static_cast<std::size_t>(variable)];
  }
  auto operator[](AeroVariable variable) const -> double {
    return values_[static_cast<std::size_t>(variable)];
  }
  auto operator[](AeroInput input) const -> double {
    return values_[input.index];
  }

  // Reads each of `read` from the aircraft's flight control `signals`.
  auto read(std::span<const AeroSignal> read, std::span<const double> signals)
      -> void {
    for (std::size_t i = 0; i < read.size(); ++i) {
      double value = signals[read[i].signal];
      values_[AERO_VARIABLE_COUNT + i] =
          read[i].magnitude ? std::abs(value) : value;
    }
  }

 private:
  std::array<double, AERO_VARIABLE_COUNT + MAX_AERO_SIGNALS> values_{};
};

// A table of one input, or of two: a row and a column.
struct AeroTable final {
  AeroInput row = aero_input(AeroVariable::ALPHA);
  std::optional<AeroInput> column;
  std::variant<Table1<>, Table2<>> table;

  auto operator()(const AeroInputs& inputs) const -> double;
};

// A constant, times its factors, times its tables.
struct AeroTerm final {
  std::string name;
  double constant = 1.0;
  std::vector<AeroInput> factors;
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
  std::vector<AeroSignal> signals;  // The signals the terms read, in order.
  // Whether drag, side force or lift read the rate of angle of attack, which
  // follows from the forces; found once, when the aircraft is read.
  bool forces_read_alpha_rate = false;

  auto operator()(const AeroInputs& inputs) const -> AeroSums;

  // Whether a term on `axis` reads `variable`, as a factor or in a table.
  auto reads(AeroAxis axis, AeroVariable variable) const -> bool;
};

// Body-axis aerodynamic force, and moment about the center of mass.
struct AeroLoads final {
  ForceVector force;
  Moment moment;
};

// Angle of attack and sideslip, as the turn from wind axes to body axes takes
// them: found once for the loads of a state, however often they are summed.
struct WindAngles final {
  double sin_alpha = 0.0;
  double cos_alpha = 1.0;
  double sin_beta = 0.0;
  double cos_beta = 1.0;
};

auto compute_wind_angles(Angle alpha, Angle beta) -> WindAngles;

// The same from the body's velocity through the air, in body axes, with no
// trigonometry: the angles' sines and cosines are ratios of its components.
auto compute_wind_angles(const Vector3& air_velocity) -> WindAngles;

// The sums as body-axis loads, at angle of attack `alpha` and sideslip
// `beta`, with the aerodynamic reference point `reference` from the center of
// mass in body axes (x forward, y right, z down). Forces turn from wind axes
// to body axes as Stevens and Lewis turn them (see model/REFERENCES.md), and
// move to the center of mass with the moment of the force at the reference.
auto compute_aero_loads(const AeroSums& sums, const WindAngles& wind,
                        const Displacement& reference) -> AeroLoads;
auto compute_aero_loads(const AeroSums& sums, Angle alpha, Angle beta,
                        const Displacement& reference) -> AeroLoads;

}  // namespace simon::model
