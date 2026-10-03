// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <cmath>

#include "application/automotive/testing.hpp"
#include "base/testing.hpp"
#include "model/collision.hpp"

// Boxes against Shapely's GEOS, from the table reference/shapely_boxes.py
// recorded: whether pairs overlap, a quarter of them a nanometer from
// touching, and how far apart they are.
namespace simon::automotive {

namespace {

using namespace testing;

auto box_of(const Row& row, std::string_view which) -> model::OrientedBox {
  std::string prefix{which};
  return {.x = number(row, prefix + "x"),
          .y = number(row, prefix + "y"),
          .heading = number(row, prefix + "h"),
          .length = number(row, prefix + "l"),
          .width = number(row, prefix + "w")};
}

}  // namespace

TEST_CASE("CollisionAgainstShapely") {
  std::vector<Row> rows = load_rows("shapely_boxes.csv");
  REQUIRE(rows.size() == 2000);
  int agree = 0;
  int overlapping = 0;
  double distance = 0.0;
  for (const Row& row : rows) {
    model::OrientedBox a = box_of(row, "a");
    model::OrientedBox b = box_of(row, "b");
    bool theirs = number(row, "intersects") != 0.0;
    agree += model::detect_overlap(a, b) == theirs ? 1 : 0;
    overlapping += theirs ? 1 : 0;
    distance = std::max(
        distance, std::abs(model::compute_gap(a, b) - number(row, "distance")));
  }
  CAPTURE(agree, overlapping, distance);
  CHECK(agree == 2000);
  CHECK(overlapping == 669);
  CHECK(distance < 1e-12);
}

}  // namespace simon::automotive
