// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/earth/wind.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>

namespace simon::earth {

namespace {

constexpr double FOOT = 0.3048;           // m.
constexpr double KNOT = 1852.0 / 3600.0;  // m/s.

// MIL-F-8785C's intensities above 2,000 ft, in ft/s, at each altitude in ft,
// for the probabilities of exceedance 10^-2, 10^-3 and 10^-5.
constexpr std::array<double, 12> TABLE_ALTITUDES{
    500.0,   1750.0,  3750.0,  7500.0,  15000.0, 25000.0,
    35000.0, 45000.0, 55000.0, 65000.0, 75000.0, 80000.0};
constexpr std::array<std::array<double, 12>, 3> TABLE_INTENSITIES{{
    {6.6, 6.9, 7.4, 6.7, 4.6, 2.7, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0},
    {8.6, 9.6, 10.6, 10.1, 8.0, 6.6, 5.0, 4.2, 2.7, 0.0, 0.0, 0.0},
    {15.6, 17.6, 23.0, 23.6, 22.1, 20.0, 16.0, 15.1, 12.1, 7.9, 6.2, 5.1},
}};

// The table's intensity for `turbulence` at `feet`, in ft/s, interpolated in
// altitude and held beyond its ends.
auto table_intensity(Turbulence turbulence, double feet) -> double {
  const std::array<double, 12>& row =
      TABLE_INTENSITIES[static_cast<std::size_t>(turbulence) - 1];
  if (feet <= TABLE_ALTITUDES.front()) {
    return row.front();
  }
  for (std::size_t i = 1; i < TABLE_ALTITUDES.size(); ++i) {
    if (feet <= TABLE_ALTITUDES[i]) {
      double t = (feet - TABLE_ALTITUDES[i - 1]) /
                 (TABLE_ALTITUDES[i] - TABLE_ALTITUDES[i - 1]);
      return row[i - 1] + t * (row[i] - row[i - 1]);
    }
  }
  return row.back();
}

// The wind at 20 ft, in ft/s, that sets low-altitude intensity.
auto wind_at_20_feet(Turbulence turbulence) -> double {
  constexpr std::array<double, 3> KNOTS{15.0, 30.0, 45.0};
  return KNOTS[static_cast<std::size_t>(turbulence) - 1] * KNOT / FOOT;
}

// Scales in feet and ft/s: u, v and w's intensities, then their lengths.
using FeetScales = std::array<double, 6>;

auto low_altitude(Turbulence turbulence, double feet) -> FeetScales {
  double sigma_w = 0.1 * wind_at_20_feet(turbulence);
  double factor = 0.177 + 0.000823 * feet;
  double sigma = sigma_w / std::pow(factor, 0.4);
  double length = feet / std::pow(factor, 1.2);
  return {sigma, sigma, sigma_w, length, length, feet};
}

auto high_altitude(Turbulence turbulence, double feet) -> FeetScales {
  double sigma = table_intensity(turbulence, feet);
  // L_u = 2 L_v = 2 L_w = 1,750 ft, with the Dryden form.
  return {sigma, sigma, sigma, 1750.0, 875.0, 875.0};
}

// A uniform number in (0, 1] from the `n`th output of the stream `seed`, by
// SplitMix64: the same everywhere for the same seed, and needing no state
// but the count.
auto uniform(std::uint64_t seed, std::uint64_t n) -> double {
  std::uint64_t z = seed + (n + 1) * 0x9e3779b97f4a7c15ULL;
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  z ^= z >> 31;
  return (static_cast<double>(z >> 11) + 1.0) * 0x1.0p-53;
}

// Six standard normal numbers for a step, by Box and Muller's transform (see
// model/REFERENCES.md).
auto normals(std::uint64_t seed, std::uint64_t step) -> std::array<double, 6> {
  std::array<double, 6> result{};
  for (std::size_t i = 0; i < 3; ++i) {
    double radius = std::sqrt(-2.0 * std::log(uniform(seed, 6 * step + 2 * i)));
    double angle = 2.0 * std::numbers::pi * uniform(seed, 6 * step + 2 * i + 1);
    result[2 * i] = radius * std::cos(angle);
    result[2 * i + 1] = radius * std::sin(angle);
  }
  return result;
}

// 1 - e^-y (1 + y + ... + y^(n-1) / (n-1)!): the tail of e^y's series from
// y^n / n!, times e^-y. Summed as a series for small y, where the difference
// would cancel.
auto exponential_tail(double y, int n) -> double {
  if (y > 1.0) {
    double partial = 0.0;
    double term = 1.0;
    for (int k = 0; k < n; ++k) {
      partial += term;
      term *= y / (k + 1);
    }
    return 1.0 - std::exp(-y) * partial;
  }
  double term = 1.0;
  for (int k = 1; k <= n; ++k) {
    term *= y / k;
  }
  double sum = 0.0;
  for (int k = n + 1; term > 1e-18 * sum; ++k) {
    sum += term;
    term *= y / k;
  }
  return std::exp(-y) * sum;
}

// Advances a first-order Dryden filter, scaled to unit variance, by `x` of
// its time constants, with the normal number `noise`.
auto advance_first_order(double state, double x, double noise) -> double {
  return std::exp(-x) * state + std::sqrt(exponential_tail(2.0 * x, 1)) * noise;
}

// Advances a second-order Dryden filter, two lags in series scaled to unit
// variance, by `x` of its time constants, with the normal numbers `a` and `b`.
// Its states' steady correlation is 1 / sqrt(2), and the gust is
// sqrt(3/2) s0 + (1 - sqrt(3)) / 2 s1 in units of its intensity. Derived here:
// the step's covariance is Van Loan's integral of the filter's transition,
// e^(-a t) (1, a t), in closed form, and the gust's weights are the partial
// fractions of the Dryden filter (1 + sqrt(3) L s / V) / (1 + L s / V)^2.
auto advance_second_order(std::array<double, 2> state, double x, double a,
                          double b) -> std::array<double, 2> {
  double decay = std::exp(-x);
  double y = 2.0 * x;
  // The covariance the step adds, and its Cholesky factor.
  double q00 = exponential_tail(y, 1);
  double q01 = std::numbers::sqrt2 / 2.0 * exponential_tail(y, 2);
  double q11 = exponential_tail(y, 3);
  double l00 = std::sqrt(q00);
  double l10 = l00 > 0.0 ? q01 / l00 : 0.0;
  double l11 = std::sqrt(std::max(q11 - l10 * l10, 0.0));
  return {decay * state[0] + l00 * a,
          decay * (state[1] + std::numbers::sqrt2 * x * state[0]) + l10 * a +
              l11 * b};
}

auto second_order_gust(const std::array<double, 2>& state) -> double {
  return std::sqrt(1.5) * state[0] + (1.0 - std::sqrt(3.0)) / 2.0 * state[1];
}

// Advances a lag of time constant `tau` by `dt`, its input moving in a
// straight line from `from` to `to`: the lag's triangle-hold equivalent
// (Franklin, Powell and Workman; see model/REFERENCES.md).
auto advance_lag(double lagged, double from, double to, double dt, double tau)
    -> double {
  double x = dt / tau;
  double decayed = -std::expm1(-x);  // 1 - e^-x.
  return lagged + decayed * (from - lagged) + (to - from) * (1.0 - decayed / x);
}

}  // namespace

auto is_still(const WindField& field) -> bool {
  return field.turbulence == Turbulence::NONE &&
         is_still(Wind{.north_east_down = field.north_east_down});
}

auto find_turbulence_scales(Turbulence turbulence, Length altitude)
    -> TurbulenceScales {
  if (turbulence == Turbulence::NONE) {
    return {};
  }
  double feet = std::max(altitude.numerical_value_in(meter) / FOOT, 10.0);
  FeetScales scales;
  if (feet <= 1000.0) {
    scales = low_altitude(turbulence, feet);
  } else if (feet >= 2000.0) {
    scales = high_altitude(turbulence, feet);
  } else {
    FeetScales low = low_altitude(turbulence, 1000.0);
    FeetScales high = high_altitude(turbulence, 2000.0);
    double t = (feet - 1000.0) / 1000.0;
    for (std::size_t i = 0; i < scales.size(); ++i) {
      scales[i] = low[i] + t * (high[i] - low[i]);
    }
  }
  return TurbulenceScales{
      .sigma_u = scales[0] * FOOT * meter_per_second,
      .sigma_v = scales[1] * FOOT * meter_per_second,
      .sigma_w = scales[2] * FOOT * meter_per_second,
      .length_u = scales[3] * FOOT * meter,
      .length_v = scales[4] * FOOT * meter,
      .length_w = scales[5] * FOOT * meter,
  };
}

auto advance_gusts(Turbulence turbulence, Length altitude, Speed airspeed,
                   Length span, Time dt, InOut<Gusts> gusts) -> void {
  Gusts& g = *gusts;
  double speed = airspeed.numerical_value_in(meter_per_second);
  if (turbulence == Turbulence::NONE || speed <= 0.0) {
    g.velocity = meters_per_second(0.0, 0.0, 0.0);
    g.rotation = QuantityVector{} * radian_per_second;
    return;
  }
  TurbulenceScales scales = find_turbulence_scales(turbulence, altitude);
  double seconds = dt.numerical_value_in(second);
  double b = span.numerical_value_in(meter);
  double length_u = scales.length_u.numerical_value_in(meter);
  double length_v = scales.length_v.numerical_value_in(meter);
  double length_w = scales.length_w.numerical_value_in(meter);
  double sigma_w = scales.sigma_w.numerical_value_in(meter_per_second);
  // MIL-F-8785C's roll gust: a first-order spectrum of length 4b / pi, and
  // the variance its spectrum integrates to.
  double length_p = 4.0 * b / std::numbers::pi;
  double sigma_p =
      sigma_w *
      std::sqrt(0.8 * std::cbrt(std::numbers::pi * length_w / (4.0 * b)) *
                std::numbers::pi * std::numbers::pi / (8.0 * b * length_w));

  std::array<double, 6> n = normals(g.seed, g.draws);
  double before_v = g.velocity.numerical_value_in(meter_per_second).y();
  double before_w = g.velocity.numerical_value_in(meter_per_second).z();
  if (g.draws == 0) {
    // The filters' steady distribution.
    g.u = n[0];
    g.v = {n[1], (n[1] + n[2]) / std::numbers::sqrt2};
    g.w = {n[3], (n[3] + n[4]) / std::numbers::sqrt2};
    g.p = n[5];
  } else {
    double travelled = speed * seconds;
    g.u = advance_first_order(g.u, travelled / length_u, n[0]);
    g.v = advance_second_order(g.v, travelled / length_v, n[1], n[2]);
    g.w = advance_second_order(g.w, travelled / length_w, n[3], n[4]);
    g.p = advance_first_order(g.p, travelled / length_p, n[5]);
  }
  ++g.draws;

  double u = scales.sigma_u.numerical_value_in(meter_per_second) * g.u;
  double v = scales.sigma_v.numerical_value_in(meter_per_second) *
             second_order_gust(g.v);
  double w = sigma_w * second_order_gust(g.w);

  // The pitch and yaw gusts are the w and v gusts' gradients along the path,
  // q = -dw/dx and r = dv/dx, as the air's rotation has them, through lags of
  // 4b / (pi V) and 3b / (pi V).
  double lag_q = 4.0 * b / std::numbers::pi;
  double lag_r = 3.0 * b / std::numbers::pi;
  double lagged_w = g.lagged_w.numerical_value_in(meter_per_second);
  double lagged_v = g.lagged_v.numerical_value_in(meter_per_second);
  if (g.draws == 1) {
    lagged_w = w;
    lagged_v = v;
  } else {
    lagged_w = advance_lag(lagged_w, before_w, w, seconds, lag_q / speed);
    lagged_v = advance_lag(lagged_v, before_v, v, seconds, lag_r / speed);
  }
  g.lagged_w = lagged_w * meter_per_second;
  g.lagged_v = lagged_v * meter_per_second;
  g.velocity = meters_per_second(u, v, w);
  g.rotation = QuantityVector{sigma_p * g.p, -(w - lagged_w) / lag_q,
                              (v - lagged_v) / lag_r} *
               radian_per_second;
}

auto compute_wind(const WindField& field, const Gusts& gusts, Angle heading)
    -> Wind {
  // From the path's axes to north, east and down: a turn by the heading.
  double c = cos(heading);
  double s = sin(heading);
  auto turn = [&](const Vector3& path) {
    return Vector3{c * path.x() - s * path.y(), s * path.x() + c * path.y(),
                   path.z()};
  };
  Vector3 gust = gusts.velocity.numerical_value_in(meter_per_second).eigen();
  Vector3 rotation =
      gusts.rotation.numerical_value_in(radian_per_second).eigen();
  return Wind{
      .north_east_down =
          field.north_east_down + QuantityVector{turn(gust)} * meter_per_second,
      .rotation = QuantityVector{turn(rotation)} * radian_per_second,
  };
}

auto compute_wind(const WindField& field) -> Wind {
  return Wind{.north_east_down = field.north_east_down};
}

}  // namespace simon::earth
