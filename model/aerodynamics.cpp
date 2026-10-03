// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/aerodynamics.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace simon::model {

namespace {

constexpr std::array<std::pair<std::string_view, AeroVariable>,
                     AERO_VARIABLE_COUNT>
    VARIABLE_NAMES{{
        {"dynamic_pressure", AeroVariable::DYNAMIC_PRESSURE},
        {"alpha", AeroVariable::ALPHA},
        {"beta", AeroVariable::BETA},
        {"mach", AeroVariable::MACH},
        {"span_over_twice_speed", AeroVariable::SPAN_OVER_TWICE_SPEED},
        {"chord_over_twice_speed", AeroVariable::CHORD_OVER_TWICE_SPEED},
        {"roll_rate", AeroVariable::ROLL_RATE},
        {"pitch_rate", AeroVariable::PITCH_RATE},
        {"yaw_rate", AeroVariable::YAW_RATE},
        {"alpha_rate", AeroVariable::ALPHA_RATE},
        {"lift_coefficient_squared", AeroVariable::LIFT_COEFFICIENT_SQUARED},
        {"height_over_span", AeroVariable::HEIGHT_OVER_SPAN},
        {"density_altitude", AeroVariable::DENSITY_ALTITUDE},
    }};

constexpr std::array<std::pair<std::string_view, AeroAxis>, AERO_AXIS_COUNT>
    AXIS_NAMES{{
        {"drag", AeroAxis::DRAG},
        {"side", AeroAxis::SIDE},
        {"lift", AeroAxis::LIFT},
        {"roll", AeroAxis::ROLL},
        {"pitch", AeroAxis::PITCH},
        {"yaw", AeroAxis::YAW},
    }};

template <typename EnumType, std::size_t Count>
auto find(const std::array<std::pair<std::string_view, EnumType>, Count>& names,
          std::string_view name) -> std::optional<EnumType> {
  auto found = std::ranges::find(names, name,
                                 &std::pair<std::string_view, EnumType>::first);
  if (found == names.end()) {
    return std::nullopt;
  }
  return found->second;
}

}  // namespace

auto find_aero_variable(std::string_view name) -> std::optional<AeroVariable> {
  return find(VARIABLE_NAMES, name);
}

auto find_aero_axis(std::string_view name) -> std::optional<AeroAxis> {
  return find(AXIS_NAMES, name);
}

auto AeroTable::operator()(const AeroInputs& inputs) const -> double {
  if (const auto* one = std::get_if<Table1<>>(&table)) {
    return (*one)(inputs[row]);
  }
  return std::get<Table2<>>(table)(inputs[row], inputs[*column]);
}

auto AeroTerm::operator()(const AeroInputs& inputs) const -> double {
  double value = constant;
  for (AeroInput factor : factors) {
    value *= inputs[factor];
  }
  for (const AeroTable& table : tables) {
    value *= table(inputs);
  }
  return value;
}

auto AeroModel::operator()(const AeroInputs& inputs) const -> AeroSums {
  AeroSums sums{};
  for (std::size_t axis = 0; axis < AERO_AXIS_COUNT; ++axis) {
    for (const AeroTerm& term : axes[axis]) {
      sums[axis] += term(inputs);
    }
  }
  return sums;
}

auto AeroModel::reads(AeroAxis axis, AeroVariable variable) const -> bool {
  auto is = [&](AeroInput input) { return input == aero_input(variable); };
  for (const AeroTerm& term : axes[static_cast<std::size_t>(axis)]) {
    if (std::ranges::any_of(term.factors, is)) {
      return true;
    }
    for (const AeroTable& table : term.tables) {
      if (is(table.row) || (table.column && is(*table.column))) {
        return true;
      }
    }
  }
  return false;
}

auto compute_wind_angles(Angle alpha, Angle beta) -> WindAngles {
  return WindAngles{.sin_alpha = sin(alpha),
                    .cos_alpha = cos(alpha),
                    .sin_beta = sin(beta),
                    .cos_beta = cos(beta)};
}

auto compute_wind_angles(const Vector3& air_velocity) -> WindAngles {
  double u = air_velocity.x();
  double v = air_velocity.y();
  double w = air_velocity.z();
  double along_and_down = std::sqrt(u * u + w * w);
  double speed = std::sqrt(u * u + v * v + w * w);
  if (along_and_down == 0.0) {
    return WindAngles{};
  }
  return WindAngles{.sin_alpha = w / along_and_down,
                    .cos_alpha = u / along_and_down,
                    .sin_beta = v / speed,
                    .cos_beta = along_and_down / speed};
}

auto compute_aero_loads(const AeroSums& sums, Angle alpha, Angle beta,
                        const Displacement& reference) -> AeroLoads {
  return compute_aero_loads(sums, compute_wind_angles(alpha, beta), reference);
}

auto compute_aero_loads(const AeroSums& sums, const WindAngles& wind,
                        const Displacement& reference) -> AeroLoads {
  double drag = sums[static_cast<std::size_t>(AeroAxis::DRAG)];
  double side = sums[static_cast<std::size_t>(AeroAxis::SIDE)];
  double lift = sums[static_cast<std::size_t>(AeroAxis::LIFT)];

  // From wind axes, where drag and lift point back and up, to body axes.
  auto [sin_alpha, cos_alpha, sin_beta, cos_beta] = wind;
  QuantityVector force{
      -cos_alpha * cos_beta * drag - cos_alpha * sin_beta * side +
          sin_alpha * lift,
      -sin_beta * drag + cos_beta * side,
      -sin_alpha * cos_beta * drag - sin_alpha * sin_beta * side -
          cos_alpha * lift,
  };

  QuantityVector about_reference{
      sums[static_cast<std::size_t>(AeroAxis::ROLL)],
      sums[static_cast<std::size_t>(AeroAxis::PITCH)],
      sums[static_cast<std::size_t>(AeroAxis::YAW)]};
  QuantityVector arm = reference.numerical_value_in(meter);
  return AeroLoads{
      .force = force * newton,
      .moment = (about_reference + cross(arm, force)) * newton_meter,
  };
}

}  // namespace simon::model
