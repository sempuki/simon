// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include "application/automotive/testing.hpp"
#include "base/testing.hpp"
#include "model/traffic/driving_metrics.hpp"

// nuPlan's metrics on esmini's runs, from the table
// reference/nuplan_metrics.py recorded with nuPlan's own code: the comfort
// signals at every sample, and the time to collision with the vehicles
// ahead.
namespace simon::automotive {

namespace {

using namespace testing;

constexpr double CENTER = 1.4;  // m, the box's center ahead of the rear axle.

}  // namespace

TEST_CASE("MetricsAgainstNuPlan") {
  std::map<std::string, std::vector<Row>> runs;
  for (const Row& row : load_rows("nuplan_metrics.csv")) {
    runs[row.find("scenario")->second].push_back(row);
  }
  REQUIRE(runs.size() == 3);

  SECTION("ShouldMatchComfortSignals") {
    // Preconditions.
    double largest = 0.0;

    // Under Test.
    for (const auto& [scenario, rows] : runs) {
      std::vector<traffic::DrivingSample> samples;
      for (const Row& row : rows) {
        samples.push_back({.time = number(row, "time"),
                           .x = number(row, "x"),
                           .y = number(row, "y"),
                           .heading = number(row, "heading"),
                           .speed = number(row, "speed"),
                           .acceleration_x = number(row, "acceleration_x"),
                           .acceleration_y = number(row, "acceleration_y")});
      }
      traffic::ComfortSignals signals =
          traffic::compute_comfort_signals(samples);
      for (std::size_t i = 0; i < rows.size(); ++i) {
        for (auto [ours, column] :
             {std::pair{signals.lon_acceleration[i], "lon_acceleration"},
              {signals.lat_acceleration[i], "lat_acceleration"},
              {signals.lon_jerk[i], "lon_jerk"},
              {signals.jerk[i], "jerk"},
              {signals.yaw_rate[i], "yaw_rate"},
              {signals.yaw_acceleration[i], "yaw_acceleration"}}) {
          largest = std::max(largest, std::abs(ours - number(rows[i], column)));
        }
      }
    }

    // Postconditions.
    CAPTURE(largest);
    // nuPlan rounds each signal to 8 decimals: a value on a rounding's edge
    // can land a unit either side, and a jerk carries it through its
    // derivative.
    CHECK(largest <= 5e-8);
  }

  SECTION("ShouldMatchTimeToCollision") {
    // Preconditions.
    int compared = 0;
    int agree = 0;

    // Under Test.
    for (const auto& [scenario, rows] : runs) {
      // The other vehicle's place at each time, from esmini's runs.
      std::map<std::string, std::vector<Row>> others;
      for (const Row& row : load_rows("esmini_scenarios.csv")) {
        if (row.find("scenario")->second == scenario &&
            row.find("entity")->second != "Ego") {
          others[row.find("time")->second].push_back(row);
        }
      }
      for (const Row& row : rows) {
        double x = number(row, "x");
        double y = number(row, "y");
        double heading = number(row, "heading");
        auto center = [](double cx, double cy, double h) {
          return model::Point2{.x = cx + CENTER * std::cos(h),
                               .y = cy + CENTER * std::sin(h)};
        };
        model::Point2 ego_center = center(x, y, heading);
        traffic::MovingBox ego{.box = {.x = ego_center.x,
                                       .y = ego_center.y,
                                       .heading = heading,
                                       .length = 5.04,
                                       .width = 2.0},
                               .speed = number(row, "speed")};
        std::vector<traffic::MovingBox> tracks;
        for (const Row& other : others[row.find("time")->second]) {
          double h = number(other, "heading");
          model::Point2 c = center(number(other, "x"), number(other, "y"), h);
          if (traffic::check_ahead(x, y, heading, c,
                                   30.0 * std::numbers::pi / 180.0)) {
            tracks.push_back({.box = {.x = c.x,
                                      .y = c.y,
                                      .heading = h,
                                      .length = 5.04,
                                      .width = 2.0},
                              .speed = number(other, "speed")});
          }
        }
        std::optional<double> ours =
            tracks.empty() ? std::nullopt
                           : traffic::compute_time_to_collision(ego, tracks);
        const std::string& theirs = row.find("ttc")->second;
        ++compared;
        if ((!ours && theirs.empty()) ||
            (ours && !theirs.empty() && *ours == number(row, "ttc"))) {
          ++agree;
        }
      }
    }

    // Postconditions.
    CAPTURE(compared, agree);
    CHECK(agree == compared);
  }
}

}  // namespace simon::automotive
