// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/earth/wind.hpp"

#include <cmath>
#include <numbers>
#include <vector>

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"

namespace simon::model {

namespace {

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

constexpr double FOOT = 0.3048;

// Gusts along the path, in m/s, every `dt` for `steps` steps, at 6 km and
// 200 m/s with a 30 m span.
struct Series final {
  std::vector<double> u;
  std::vector<double> v;
  std::vector<double> w;
  std::vector<double> p;
};

auto fly(Turbulence turbulence, double dt, int steps, std::uint64_t seed)
    -> Series {
  Gusts gusts{.seed = seed};
  Series series;
  for (int i = 0; i < steps; ++i) {
    advance_gusts(turbulence, 6000.0 * meter, 200.0 * meter_per_second,
                  30.0 * meter, dt * second, InOut(gusts));
    Vector3 velocity =
        gusts.velocity.numerical_value_in(meter_per_second).eigen();
    series.u.push_back(velocity.x());
    series.v.push_back(velocity.y());
    series.w.push_back(velocity.z());
    series.p.push_back(
        gusts.rotation.numerical_value_in(radian_per_second).x());
  }
  return series;
}

// The sample covariance of `x` with itself `lag` samples later.
auto covariance(const std::vector<double>& x, std::size_t lag) -> double {
  double sum = 0.0;
  for (std::size_t i = 0; i + lag < x.size(); ++i) {
    sum += x[i] * x[i + lag];
  }
  return sum / static_cast<double>(x.size() - lag);
}

}  // namespace

TEST_CASE("Wind") {
  SECTION("ShouldReadMilF8785cTableGivenHighAltitude") {
    // 6 km is 19,685 ft, between the table's 15,000 and 25,000 ft.
    TurbulenceScales scales =
        find_turbulence_scales(Turbulence::MODERATE, 6000.0 * meter);
    double feet = 6000.0 / FOOT;
    double expected = 8.0 + (feet - 15000.0) / 10000.0 * (6.6 - 8.0);
    CHECK_THAT(scales.sigma_w.numerical_value_in(meter_per_second),
               WithinRel(expected * FOOT, 1e-12));
    CHECK(scales.sigma_u == scales.sigma_w);
    CHECK_THAT(scales.length_u.numerical_value_in(meter),
               WithinRel(1750.0 * FOOT, 1e-12));
    CHECK_THAT(scales.length_w.numerical_value_in(meter),
               WithinRel(875.0 * FOOT, 1e-12));
  }

  SECTION("ShouldFollowWindAt20FeetGivenLowAltitude") {
    // At 500 ft in light turbulence, sigma_w is a tenth of 15 knots, and w's
    // scale is the altitude.
    TurbulenceScales scales =
        find_turbulence_scales(Turbulence::LIGHT, 500.0 * FOOT * meter);
    CHECK_THAT(scales.sigma_w.numerical_value_in(meter_per_second),
               WithinRel(1.5 * 1852.0 / 3600.0, 1e-12));
    CHECK_THAT(scales.length_w.numerical_value_in(meter),
               WithinRel(500.0 * FOOT, 1e-12));
    double factor = 0.177 + 0.000823 * 500.0;
    CHECK_THAT(scales.sigma_u.numerical_value_in(meter_per_second),
               WithinRel(1.5 * 1852.0 / 3600.0 / std::pow(factor, 0.4), 1e-12));
  }

  SECTION("ShouldBeContinuousGivenAltitudesBetweenModels") {
    for (double feet : {1000.0, 2000.0}) {
      TurbulenceScales below = find_turbulence_scales(
          Turbulence::SEVERE, (feet - 1e-6) * FOOT * meter);
      TurbulenceScales above = find_turbulence_scales(
          Turbulence::SEVERE, (feet + 1e-6) * FOOT * meter);
      CHECK_THAT(
          below.sigma_u.numerical_value_in(meter_per_second),
          WithinAbs(above.sigma_u.numerical_value_in(meter_per_second), 1e-6));
      CHECK_THAT(below.length_v.numerical_value_in(meter),
                 WithinAbs(above.length_v.numerical_value_in(meter), 1e-6));
    }
  }

  SECTION("ShouldHaveDrydenStatisticsGivenAnyStep") {
    TurbulenceScales scales =
        find_turbulence_scales(Turbulence::SEVERE, 6000.0 * meter);
    double sigma = scales.sigma_w.numerical_value_in(meter_per_second);
    double length_u = scales.length_u.numerical_value_in(meter);
    double length_w = scales.length_w.numerical_value_in(meter);
    // From simon's step to steps a quarter of w's scale long: the filters
    // are sampled exactly, so the statistics do not change with the step.
    for (double dt : {0.02, 0.25}) {
      CAPTURE(dt);
      int steps = static_cast<int>(40000.0 / dt);
      Series series = fly(Turbulence::SEVERE, dt, steps, 11);
      // About 0.2 s apart in travel: 40 m.
      auto lag = static_cast<std::size_t>(std::lround(0.2 / dt));
      double travelled = 200.0 * dt * static_cast<double>(lag);
      double variance = sigma * sigma;
      CHECK_THAT(covariance(series.u, 0), WithinRel(variance, 0.05));
      CHECK_THAT(covariance(series.v, 0), WithinRel(variance, 0.05));
      CHECK_THAT(covariance(series.w, 0), WithinRel(variance, 0.05));
      // Dryden's correlations: e^-x along the path, and e^-x (1 - x / 2)
      // across it.
      CHECK_THAT(covariance(series.u, lag) / covariance(series.u, 0),
                 WithinAbs(std::exp(-travelled / length_u), 0.02));
      double x = travelled / length_w;
      CHECK_THAT(covariance(series.w, lag) / covariance(series.w, 0),
                 WithinAbs(std::exp(-x) * (1.0 - x / 2.0), 0.02));
      double far = 3.0;  // Scale lengths, where w's correlation is negative.
      auto far_lag =
          static_cast<std::size_t>(std::lround(far * length_w / (200.0 * dt)));
      double y = 200.0 * dt * static_cast<double>(far_lag) / length_w;
      CHECK_THAT(covariance(series.w, far_lag) / covariance(series.w, 0),
                 WithinAbs(std::exp(-y) * (1.0 - y / 2.0), 0.02));
    }
  }

  SECTION("ShouldHaveMilF8785cRollVarianceGivenSpan") {
    Series series = fly(Turbulence::SEVERE, 0.02, 1'000'000, 5);
    TurbulenceScales scales =
        find_turbulence_scales(Turbulence::SEVERE, 6000.0 * meter);
    double sigma_w = scales.sigma_w.numerical_value_in(meter_per_second);
    double length_w = scales.length_w.numerical_value_in(meter);
    double b = 30.0;
    double expected = sigma_w * sigma_w * 0.8 *
                      std::cbrt(std::numbers::pi * length_w / (4.0 * b)) *
                      std::numbers::pi * std::numbers::pi /
                      (8.0 * b * length_w);
    CHECK_THAT(covariance(series.p, 0), WithinRel(expected, 0.05));
  }

  SECTION("ShouldRepeatGivenSameSeed") {
    Series first = fly(Turbulence::MODERATE, 0.02, 100, 3);
    Series again = fly(Turbulence::MODERATE, 0.02, 100, 3);
    Series other = fly(Turbulence::MODERATE, 0.02, 100, 4);
    CHECK(first.w == again.w);
    CHECK(first.w != other.w);
  }

  SECTION("ShouldBeCalmGivenNoTurbulence") {
    Series series = fly(Turbulence::NONE, 0.02, 10, 1);
    for (double w : series.w) {
      CHECK(w == 0.0);
    }
  }

  SECTION("ShouldTurnGustsByHeadingGivenWind") {
    Gusts gusts{.velocity = meters_per_second(3.0, 1.0, -2.0)};
    WindField field{.north_east_down = meters_per_second(5.0, 0.0, 0.0)};
    // Heading east: along the path is east, and to the right is south.
    Wind wind = compute_wind(field, gusts, std::numbers::pi / 2.0 * radian);
    Vector3 ned =
        wind.north_east_down.numerical_value_in(meter_per_second).eigen();
    CHECK_THAT(ned.x(), WithinAbs(5.0 - 1.0, 1e-12));
    CHECK_THAT(ned.y(), WithinAbs(3.0, 1e-12));
    CHECK_THAT(ned.z(), WithinAbs(-2.0, 1e-12));
  }
}

}  // namespace simon::model
