// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows the nuPlan devkit 1.2.2 (Copyright 2021 Motional, Apache-2.0) and
// SciPy 1.18 (BSD-3-Clause); translated to C++ and changed. See NOTICE.md.

#include "model/driving_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace simon::model {

namespace {

constexpr double STOPPED = 5e-3;  // m/s, nuPlan's stopped speed.

// The least-squares polynomial of `order` through `points` samples at 0, 1,
// ..., its `derivative`th derivative at `at` as weights on the samples:
// row `derivative` of (A^T A)^-1 A^T, A the Vandermonde matrix in (t - at),
// times derivative!.
auto compute_weights(int points, int order, int derivative, double at)
    -> std::vector<double> {
  int n = order + 1;
  std::vector<double> normal(static_cast<std::size_t>(n * n), 0.0);
  std::vector<std::vector<double>> powers(static_cast<std::size_t>(points),
                                          std::vector<double>(n));
  for (int i = 0; i < points; ++i) {
    double t = i - at;
    double p = 1.0;
    for (int k = 0; k < n; ++k) {
      powers[i][k] = p;
      p *= t;
    }
  }
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      double sum = 0.0;
      for (int i = 0; i < points; ++i) {
        sum += powers[i][r] * powers[i][c];
      }
      normal[r * n + c] = sum;
    }
  }
  // Solve normal x = e_derivative by Gaussian elimination with pivoting;
  // the weights are then A x.
  std::vector<double> rhs(static_cast<std::size_t>(n), 0.0);
  rhs[derivative] = 1.0;
  for (int col = 0; col < n; ++col) {
    int pivot = col;
    for (int r = col + 1; r < n; ++r) {
      if (std::abs(normal[r * n + col]) > std::abs(normal[pivot * n + col])) {
        pivot = r;
      }
    }
    for (int c = 0; c < n; ++c) {
      std::swap(normal[col * n + c], normal[pivot * n + c]);
    }
    std::swap(rhs[col], rhs[pivot]);
    for (int r = 0; r < n; ++r) {
      if (r == col) {
        continue;
      }
      double factor = normal[r * n + col] / normal[col * n + col];
      for (int c = col; c < n; ++c) {
        normal[r * n + c] -= factor * normal[col * n + c];
      }
      rhs[r] -= factor * rhs[col];
    }
  }
  std::vector<double> solution(static_cast<std::size_t>(n));
  for (int r = 0; r < n; ++r) {
    solution[r] = rhs[r] / normal[r * n + r];
  }
  double factorial = std::tgamma(derivative + 1.0);
  std::vector<double> weights(static_cast<std::size_t>(points), 0.0);
  for (int i = 0; i < points; ++i) {
    double sum = 0.0;
    for (int k = 0; k < n; ++k) {
      sum += powers[i][k] * solution[k];
    }
    weights[i] = sum * factorial;
  }
  return weights;
}

// x rounded to 8 decimals, halves to even, as NumPy's round.
auto round8(double x) -> double { return std::nearbyint(x * 1e8) / 1e8; }

auto round_all(std::vector<double> values) -> std::vector<double> {
  for (double& value : values) {
    value = round8(value);
  }
  return values;
}

// The derivative of `samples` over the samples' times, as nuPlan's
// approximate_derivatives takes it: the mean step for delta.
auto differentiate(std::span<const double> samples,
                   std::span<const DrivingSample> run, int derivative,
                   int order, int window) -> std::vector<double> {
  double delta = (run.back().time - run.front().time) /
                 static_cast<double>(run.size() - 1);
  return round_all(filter_savitzky_golay(
      samples, std::min<int>(window, static_cast<int>(samples.size())), order,
      derivative, delta));
}

}  // namespace

auto filter_savitzky_golay(std::span<const double> samples, int window,
                           int order, int derivative, double delta)
    -> std::vector<double> {
  auto n = static_cast<int>(samples.size());
  std::vector<double> out(samples.size(), 0.0);
  if (n == 0 || derivative > order) {
    return out;
  }
  int half = window / 2;
  double scale = std::pow(delta, derivative);
  // Inside: each window's polynomial at its middle, half a sample on for an
  // even window.
  double middle = window % 2 == 0 ? half - 0.5 : half;
  std::vector<double> weights =
      compute_weights(window, order, derivative, middle);
  int first = (window - 1) / 2;
  for (int i = half; i < n - half; ++i) {
    double sum = 0.0;
    for (int k = 0; k < window; ++k) {
      sum += weights[k] * samples[i - first + k];
    }
    out[i] = sum / scale;
  }
  // The edges: the first and last windows' polynomials.
  for (int i = 0; i < half && i < n; ++i) {
    std::vector<double> head = compute_weights(window, order, derivative, i);
    std::vector<double> tail =
        compute_weights(window, order, derivative, window - half + i);
    double front = 0.0;
    double back = 0.0;
    for (int k = 0; k < window; ++k) {
      front += head[k] * samples[k];
      back += tail[k] * samples[n - window + k];
    }
    out[i] = front / scale;
    out[n - half + i] = back / scale;
  }
  return out;
}

auto compute_comfort_signals(std::span<const DrivingSample> run)
    -> ComfortSignals {
  std::vector<double> along;
  std::vector<double> across;
  std::vector<double> magnitude;
  std::vector<double> heading;
  for (const DrivingSample& sample : run) {
    along.push_back(sample.acceleration_x);
    across.push_back(sample.acceleration_y);
    magnitude.push_back(
        std::hypot(sample.acceleration_x, sample.acceleration_y));
    heading.push_back(sample.heading);
  }
  // Headings unwrapped: each step's turn within half a turn.
  for (std::size_t i = 1; i < heading.size(); ++i) {
    double turns = std::nearbyint((run[i].heading - run[i - 1].heading) /
                                  (2.0 * std::numbers::pi));
    heading[i] = heading[i - 1] + (run[i].heading - run[i - 1].heading) -
                 2.0 * std::numbers::pi * turns;
  }
  int count = static_cast<int>(run.size());
  auto smooth = [&](std::span<const double> values) {
    return round_all(filter_savitzky_golay(values, std::min(8, count), 2));
  };
  ComfortSignals signals;
  signals.lon_acceleration = smooth(along);
  signals.lat_acceleration = smooth(across);
  std::vector<double> smooth_magnitude = smooth(magnitude);
  signals.lon_jerk = differentiate(signals.lon_acceleration, run, 1, 2, 15);
  signals.jerk = differentiate(smooth_magnitude, run, 1, 2, 15);
  signals.yaw_rate = differentiate(heading, run, 1, 2, 5);
  signals.yaw_acceleration = differentiate(heading, run, 2, 3, 5);
  return signals;
}

auto check_comfort(const ComfortSignals& signals, const ComfortLimits& limits)
    -> bool {
  auto within = [](const std::vector<double>& values, double least,
                   double most) {
    return std::ranges::all_of(values,
                               [&](double v) { return v > least && v < most; });
  };
  return within(signals.lon_acceleration, limits.min_lon_acceleration,
                limits.max_lon_acceleration) &&
         within(signals.lat_acceleration, -limits.max_lat_acceleration,
                limits.max_lat_acceleration) &&
         within(signals.lon_jerk, -limits.max_lon_jerk, limits.max_lon_jerk) &&
         within(signals.jerk, -limits.max_jerk, limits.max_jerk) &&
         within(signals.yaw_rate, -limits.max_yaw_rate, limits.max_yaw_rate) &&
         within(signals.yaw_acceleration, -limits.max_yaw_acceleration,
                limits.max_yaw_acceleration);
}

auto check_ahead(double x, double y, double heading, Point2 to,
                 double tolerance) -> bool {
  double dx = to.x - x;
  double dy = to.y - y;
  double length = std::hypot(dx, dy);
  double dot =
      std::cos(heading) * dx / length + std::sin(heading) * dy / length;
  return std::acos(dot) < tolerance;
}

// nuPlan's _compute_time_to_collision_at_timestamp: the boxes stretched over
// the horizon pick the tracks to follow, then every box moves on a step at a
// time, the times on NumPy's arange.
auto compute_time_to_collision(const MovingBox& ego,
                               std::span<const MovingBox> tracks, double step,
                               double horizon) -> std::optional<double> {
  if (tracks.empty() || ego.speed <= STOPPED) {
    return std::nullopt;
  }
  auto displacement = [&](const MovingBox& moving) {
    return Point2{.x = std::cos(moving.box.heading) * moving.speed * step,
                  .y = std::sin(moving.box.heading) * moving.speed * step};
  };
  auto stretch = [&](const MovingBox& moving, Point2 d) {
    OrientedBox box = moving.box;
    box.x = (horizon / step) / 2.0 * d.x + moving.box.x;
    box.y = (horizon / step) / 2.0 * d.y + moving.box.y;
    box.length = moving.box.length +
                 std::hypot(d.x * horizon / step, d.y * horizon / step);
    return box;
  };
  Point2 ego_step = displacement(ego);
  OrientedBox ego_reach = stretch(ego, ego_step);
  std::vector<Point2> steps;
  std::vector<std::size_t> followed;
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    steps.push_back(displacement(tracks[i]));
    if (detect_overlap(ego_reach, stretch(tracks[i], steps[i]))) {
      followed.push_back(i);
    }
  }
  if (followed.empty()) {
    return std::nullopt;
  }
  OrientedBox moved_ego = ego.box;
  std::vector<OrientedBox> moved;
  for (const MovingBox& track : tracks) {
    moved.push_back(track.box);
  }
  auto count = static_cast<std::size_t>(std::ceil((horizon - step) / step));
  for (std::size_t k = 0; k < count; ++k) {
    double time = step + static_cast<double>(k) * step;
    moved_ego.x += ego_step.x;
    moved_ego.y += ego_step.y;
    for (std::size_t i = 0; i < moved.size(); ++i) {
      moved[i].x += steps[i].x;
      moved[i].y += steps[i].y;
    }
    for (std::size_t i : followed) {
      if (detect_overlap(moved_ego, moved[i])) {
        return time;
      }
    }
  }
  return std::nullopt;
}

}  // namespace simon::model
