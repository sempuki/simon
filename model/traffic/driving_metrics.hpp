// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows the nuPlan devkit 1.2.2 (Copyright 2021 Motional, Apache-2.0) and
// SciPy 1.18 (BSD-3-Clause); translated to C++ and changed. See NOTICE.md.

#pragma once

#include <optional>
#include <span>
#include <vector>

#include "model/collision.hpp"

// Measures of a vehicle's run, as nuPlan defines them (see
// model/REFERENCES.md): how comfortable its ride is, and its time to
// collision with the vehicles ahead. Plain SI numbers.
namespace simon::model {

// A Savitzky-Golay filter, as SciPy's savgol_filter computes it with its
// default edges: each sample the `derivative`th derivative, over `delta`
// between samples, of the least-squares polynomial of `order` through the
// `window` samples about it; the window's first and last samples' from the
// polynomials through the first and last windows. An even window evaluates
// its polynomial half a sample on. Savitzky and Golay, "Smoothing and
// differentiation of data by simplified least squares procedures", 1964.
auto filter_savitzky_golay(std::span<const double> samples, int window,
                           int order, int derivative = 0, double delta = 1.0)
    -> std::vector<double>;

// One sample of a vehicle's run, as nuPlan reads it: when, its rear axle's
// position and heading, its speed, and its acceleration along and across
// its heading.
struct DrivingSample final {
  double time = 0.0;  // s.
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double speed = 0.0;
  double acceleration_x = 0.0;
  double acceleration_y = 0.0;
};

// The signals nuPlan's comfort reads, each a value a sample, as its state
// extractors make them: the accelerations smoothed over 8 samples, the jerks
// their derivatives over 15, the yaw rate and acceleration the heading's
// first and second derivatives over 5, all rounded to 8 decimals.
struct ComfortSignals final {
  std::vector<double> lon_acceleration;
  std::vector<double> lat_acceleration;
  std::vector<double> lon_jerk;
  std::vector<double> jerk;  // Of the acceleration's magnitude.
  std::vector<double> yaw_rate;
  std::vector<double> yaw_acceleration;
};

auto compute_comfort_signals(std::span<const DrivingSample> samples)
    -> ComfortSignals;

// nuPlan's bounds on a comfortable ride.
struct ComfortLimits final {
  double min_lon_acceleration = -4.05;  // m/s^2.
  double max_lon_acceleration = 2.40;
  double max_lat_acceleration = 4.89;
  double max_lon_jerk = 4.13;  // m/s^3.
  double max_jerk = 8.37;
  double max_yaw_rate = 0.95;          // rad/s.
  double max_yaw_acceleration = 1.93;  // rad/s^2.
};

// Whether every signal stays strictly within its bounds at every sample,
// as nuPlan's within-bound metrics check them.
auto check_comfort(const ComfortSignals& signals,
                   const ComfortLimits& limits = {}) -> bool;

// Whether `to` lies within `tolerance` radians either side of the heading
// from (`x`, `y`): ahead of it, as nuPlan's TTC counts a vehicle ahead.
auto check_ahead(double x, double y, double heading, Point2 to,
                 double tolerance) -> bool;

// A box moving along its heading at `speed`.
struct MovingBox final {
  OrientedBox box;
  double speed = 0.0;
};

// The time until the ego's box first overlaps a track's, each carried at
// its speed along its heading in steps of `step`, up to `horizon`; none if
// they never do or the ego is standing. Only the tracks whose boxes,
// stretched over the horizon, overlap the ego's so stretched are followed.
auto compute_time_to_collision(const MovingBox& ego,
                               std::span<const MovingBox> tracks,
                               double step = 0.1, double horizon = 3.0)
    -> std::optional<double>;

}  // namespace simon::model
