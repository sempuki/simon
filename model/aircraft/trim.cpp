// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows JSBSim 1.3.1, Copyright (C) 2000 Jon S. Berndt and the JSBSim
// authors, LGPL-2.0-or-later; translated to C++ and changed. See NOTICE.md.

#include "model/aircraft/trim.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "Eigen/Dense"

template <>
const std::array<lib::StatusConditionEntry, simon::aircraft::TRIM_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::aircraft::TrimError,
        simon::aircraft::TRIM_ERROR_COUNT>::conditions_ = {
        lib::StatusConditionEntry{"trim diverged"},
        lib::StatusConditionEntry{"trim saturated"},
};

namespace simon::aircraft {

namespace {

using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

// The unknowns, in order, with their limits.
enum Unknown : std::size_t {
  ALPHA,
  THROTTLE,
  PITCH_TRIM,
  BANK,
  AILERON,
  RUDDER
};

constexpr std::array<std::pair<double, double>, 6> LIMITS{{
    {-0.5, 0.5},  // Angle of attack, rad.
    {0.0, 1.0},   // Throttle command.
    {-1.0, 1.0},  // Pitch trim command.
    {-1.5, 1.5},  // Bank, rad.
    {-1.0, 1.0},  // Aileron command.
    {-1.0, 1.0},  // Rudder command.
}};

// Where Newton's method starts: level, about a third of the throttle.
constexpr std::array<double, 6> START{0.05, 0.35, 0.0, 0.0, 0.0, 0.0};

// Balanced: accelerations within this, in m/s^2, angular ones times the span.
constexpr double BALANCED = 1e-9;

// The pitch at which a body at angle of attack `alpha` and bank `bank`, with
// no sideslip, climbs at `gamma`: the rate-of-climb constraint of steady
// flight (Stevens and Lewis; see model/REFERENCES.md).
auto solve_pitch(double alpha, double bank, double gamma) -> double {
  double a = std::cos(alpha);
  double b = std::sin(alpha) * std::cos(bank);
  return std::atan2(b, a) + std::asin(std::sin(gamma) / std::hypot(a, b));
}

class Trimmer final {
 public:
  Trimmer(const Definition& aircraft, const FlightCondition& condition,
          const Earth& earth, const earth::StandardAirTable& air)
      : aircraft_{aircraft},
        condition_{condition},
        earth_{earth},
        air_{air},
        mass_{compute_mass_balance(aircraft, condition.tanks)},
        span_{aircraft.wing_span.numerical_value_in(meter)} {}

  // The aircraft at the unknowns `x`, settled, and the accelerations left,
  // each paired with its unknown.
  auto evaluate(const Vector6& x, Vector6* residual) const -> Trim {
    double speed = condition_.speed.numerical_value_in(meter_per_second);
    double gamma = radians(condition_.flight_path_angle);
    Trim trim{
        .alpha = x[ALPHA] * radian,
        .pitch = solve_pitch(x[ALPHA], x[BANK], gamma) * radian,
        .bank = x[BANK] * radian,
        .throttle = x[THROTTLE],
        .pitch_trim = x[PITCH_TRIM],
        .aileron = x[AILERON],
        .rudder = x[RUDDER],
    };
    trim.mass = mass_;
    // Level round the Earth, turning as the local frame turns under it.
    auto body_at = [&](const AngularVelocity& rate) {
      return earth_.body_at(condition_.position, trim.bank, trim.pitch,
                            condition_.heading,
                            meters_per_second(speed * std::cos(x[ALPHA]), 0.0,
                                              speed * std::sin(x[ALPHA])),
                            rate, 0.0 * second);
    };
    trim.body = body_at(QuantityVector{} * radian_per_second);
    if (condition_.level) {
      trim.body = body_at(earth_.level_rate(trim.body, 0.0 * second));
    }

    trim.signals = condition_.commands;
    trim.signals[FlightSignal::PITCH_TRIM_COMMAND] = x[PITCH_TRIM];
    trim.signals[FlightSignal::AILERON_COMMAND] = x[AILERON];
    trim.signals[FlightSignal::RUDDER_COMMAND] = x[RUDDER];
    for (std::size_t i = 0; i < aircraft_.engines.size(); ++i) {
      trim.signals.values[index_of(FlightSignal::THROTTLE_COMMAND_0) + i] =
          x[THROTTLE];
    }

    // The flight controls and the airframe, each settled on the other.
    EngineAir engine_air = compute_engine_air(trim.body, earth_, air_,
                                              earth::Wind{}, 0.0 * second);
    model::RigidBodyRate rate;
    for (int pass = 0; pass < 100; ++pass) {
      sense_flight_state(trim.body, trim.felt, trim.mass, aircraft_, earth_,
                         air_, earth::Wind{}, 0.0 * second,
                         InOut(trim.signals));
      settle_flight_controls(aircraft_.flight_controls, InOut(trim.signals));
      trim.engines =
          compute_settled_engines(aircraft_, trim.signals, engine_air);
      BodyAcceleration felt;
      rate = aircraft::compute_rigid_aircraft_rate(
          trim.body, trim.signals, trim.engines, trim.mass, aircraft_, earth_,
          air_, earth::Wind{}, 0.0 * second, Out(felt));
      double change =
          magnitude((felt.specific_force - trim.felt.specific_force)
                        .numerical_value_in(meter_per_second_squared));
      trim.felt = felt;
      if (pass > 0 && change == 0.0) {
        break;
      }
    }

    if (residual) {
      Vector3 along = earth_.air_acceleration(trim.body, rate.acceleration)
                          .numerical_value_in(meter_per_second_squared)
                          .eigen();
      Vector3 about = rate.angular_acceleration
                          .numerical_value_in(radian_per_second_squared)
                          .eigen() *
                      span_;
      (*residual)[ALPHA] = along.z();
      (*residual)[THROTTLE] = along.x();
      (*residual)[PITCH_TRIM] = about.y();
      (*residual)[BANK] = along.y();
      (*residual)[AILERON] = about.x();
      (*residual)[RUDDER] = about.z();
    }
    return trim;
  }

  auto solve() const -> std::expected<Trim, lib::Status> {
    Vector6 x = Eigen::Map<const Vector6>(START.data());
    Vector6 r;
    evaluate(x, &r);
    for (int iteration = 0; iteration < 50; ++iteration) {
      if (r.lpNorm<Eigen::Infinity>() < BALANCED) {
        break;
      }
      Matrix6 jacobian;
      for (std::size_t i = 0; i < 6; ++i) {
        constexpr double STEP = 1e-6;
        Vector6 up = x;
        Vector6 down = x;
        up[i] += STEP;
        down[i] -= STEP;
        Vector6 r_up;
        Vector6 r_down;
        evaluate(up, &r_up);
        evaluate(down, &r_down);
        jacobian.col(i) = (r_up - r_down) / (2.0 * STEP);
      }
      Vector6 step = jacobian.colPivHouseholderQr().solve(-r);

      // The longest step, halved until it leaves less, within the limits.
      double length = 1.0;
      Vector6 next;
      Vector6 r_next;
      for (int halving = 0; halving < 30; ++halving, length *= 0.5) {
        next = x + length * step;
        for (std::size_t i = 0; i < 6; ++i) {
          next[i] = std::clamp(next[i], LIMITS[i].first, LIMITS[i].second);
        }
        evaluate(next, &r_next);
        if (r_next.norm() < r.norm()) {
          break;
        }
      }
      if (r_next.norm() >= r.norm()) {
        break;
      }
      x = next;
      r = r_next;
    }

    double residual = r.lpNorm<Eigen::Infinity>();
    if (residual >= BALANCED) {
      for (std::size_t i = 0; i < 6; ++i) {
        if (x[i] == LIMITS[i].first || x[i] == LIMITS[i].second) {
          return std::unexpected(lib::raise(
              TrimError::SATURATED,
              "trimming needs control " + std::to_string(i) + " at its limit"));
        }
      }
      return std::unexpected(
          lib::raise(TrimError::DIVERGED,
                     "accelerations of " + std::to_string(residual) + " left"));
    }
    Trim result = evaluate(x, nullptr);
    result.residual = residual;
    return result;
  }

 private:
  const Definition& aircraft_;
  const FlightCondition& condition_;
  const Earth& earth_;
  const earth::StandardAirTable& air_;
  MassBalance mass_;
  double span_ = 0.0;  // m.
};

}  // namespace

auto trim(const Definition& aircraft, const FlightCondition& condition,
          const Earth& earth, const earth::StandardAirTable& air)
    -> std::expected<Trim, lib::Status> {
  return Trimmer{aircraft, condition, earth, air}.solve();
}

}  // namespace simon::aircraft
