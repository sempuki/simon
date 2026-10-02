// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "framework/spatial_index.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <vector>

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"

namespace simon::framework {

namespace {

auto squared_distance(const Coordinates& a, const Coordinates& b) -> double {
  double x = a[0] - b[0];
  double y = a[1] - b[1];
  double z = a[2] - b[2];
  return x * x + y * y + z * z;
}

// Points spread over [-extent, extent) in x and y, and a thin band in z.
auto random_points(std::size_t count, double extent, std::uint64_t seed)
    -> std::vector<Coordinates> {
  std::mt19937_64 engine{seed};
  auto uniform = [&](double low, double high) {
    return low + (high - low) * static_cast<double>(engine() >> 11) * 0x1.0p-53;
  };
  std::vector<Coordinates> points;
  for (std::size_t i = 0; i < count; ++i) {
    points.push_back(Coordinates{uniform(-extent, extent),
                                 uniform(-extent, extent), uniform(0.0, 5.0)});
  }
  return points;
}

auto build_index(const std::vector<Coordinates>& points, std::size_t capacity,
                 double cell_size) -> SpatialIndex {
  SpatialIndex index{capacity, cell_size};
  index.rebuild(points.size(), [&](std::size_t slot) { return points[slot]; });
  return index;
}

auto within(const SpatialIndex& index, const Coordinates& center, double radius)
    -> std::vector<std::uint32_t> {
  std::vector<std::uint32_t> slots;
  index.within(center, radius,
               [&](std::uint32_t slot) { slots.push_back(slot); });
  std::ranges::sort(slots);
  return slots;
}

auto brute_within(const std::vector<Coordinates>& points,
                  const Coordinates& center, double radius)
    -> std::vector<std::uint32_t> {
  std::vector<std::uint32_t> slots;
  for (std::uint32_t slot = 0; slot < points.size(); ++slot) {
    if (squared_distance(points[slot], center) <= radius * radius) {
      slots.push_back(slot);
    }
  }
  return slots;
}

template <typename AcceptType>
auto brute_nearest(const std::vector<Coordinates>& points,
                   const Coordinates& center, double radius, AcceptType accept)
    -> std::optional<std::uint32_t> {
  std::optional<std::uint32_t> best;
  double best_distance = radius * radius;
  for (std::uint32_t slot = 0; slot < points.size(); ++slot) {
    double distance = squared_distance(points[slot], center);
    if ((distance < best_distance || (distance == best_distance && !best)) &&
        accept(slot)) {
      best = slot;
      best_distance = distance;
    }
  }
  return best;
}

}  // namespace

TEST_CASE("SpatialIndex") {
  SECTION("ShouldMatchBruteForceGivenRandomPointsAndRadii") {
    std::vector<Coordinates> points = random_points(2000, 500.0, 1);
    SpatialIndex index = build_index(points, points.size(), 10.0);
    std::vector<Coordinates> centers = random_points(50, 600.0, 2);
    auto odd = [](std::uint32_t slot) { return slot % 2 == 1; };

    // Small radii search the grid; large ones read every point.
    for (double radius : {0.0, 3.0, 10.0, 25.0, 80.0, 2000.0}) {
      for (const Coordinates& center : centers) {
        CHECK(within(index, center, radius) ==
              brute_within(points, center, radius));
        CHECK(index.nearest(center, radius, odd) ==
              brute_nearest(points, center, radius, odd));
      }
    }
  }

  SECTION("ShouldVisitEachPointOnceGivenMoreCellsThanBuckets") {
    // A fine grid: a query covers thousands of cells, but there are only 512
    // buckets, so many cells share one.
    std::vector<Coordinates> points = random_points(500, 200.0, 3);
    SpatialIndex index = build_index(points, points.size(), 1.0);
    auto any = [](std::uint32_t) { return true; };

    for (const Coordinates& center : random_points(20, 200.0, 4)) {
      CHECK(within(index, center, 20.0) == brute_within(points, center, 20.0));
      CHECK(index.nearest(center, 20.0, any) ==
            brute_nearest(points, center, 20.0, any));
    }
  }

  SECTION("ShouldVisitEveryPointGivenInfiniteOrHugeRadius") {
    std::vector<Coordinates> points = random_points(50, 100.0, 5);
    SpatialIndex index = build_index(points, points.size(), 10.0);
    auto any = [](std::uint32_t) { return true; };
    Coordinates center{0.0, 0.0, 0.0};

    for (double radius : {std::numeric_limits<double>::infinity(), 1e30}) {
      CHECK(within(index, center, radius).size() == points.size());
      CHECK(index.nearest(center, radius, any) ==
            brute_nearest(points, center, radius, any));
    }
  }

  SECTION("ShouldGiveTiesToLowestSlotGivenEqualDistances") {
    std::vector<Coordinates> points{{3.0, 0.0, 0.0}, {-3.0, 0.0, 0.0}};
    SpatialIndex index = build_index(points, points.size(), 1.0);
    auto any = [](std::uint32_t) { return true; };

    CHECK(index.nearest({0.0, 0.0, 0.0}, 10.0, any) == 0u);
  }

  SECTION("ShouldFindNothingGivenEmptyIndexOrNegativeRadius") {
    SpatialIndex empty{8, 1.0};
    std::vector<Coordinates> points{{0.0, 0.0, 0.0}};
    SpatialIndex index = build_index(points, 8, 1.0);
    auto any = [](std::uint32_t) { return true; };

    CHECK(within(empty, {0.0, 0.0, 0.0}, 5.0).empty());
    CHECK(empty.nearest({0.0, 0.0, 0.0}, 5.0, any) == std::nullopt);
    CHECK(within(index, {0.0, 0.0, 0.0}, -1.0).empty());
  }

  SECTION("ShouldReplaceContentsGivenRebuild") {
    std::vector<Coordinates> before{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    std::vector<Coordinates> after{{100.0, 0.0, 0.0}};
    SpatialIndex index = build_index(before, 4, 1.0);
    index.rebuild(after.size(), [&](std::size_t slot) { return after[slot]; });

    CHECK(index.size() == 1u);
    CHECK(within(index, {0.0, 0.0, 0.0}, 5.0).empty());
    CHECK(within(index, {100.0, 0.0, 0.0}, 5.0) ==
          std::vector<std::uint32_t>{0});
  }
}

TEST_CASE("SpatialIndexSizedToPoints") {
  using Catch::Matchers::WithinRel;
  auto sized = [](const std::vector<Coordinates>& points) {
    SpatialIndex index{points.size()};
    index.rebuild(points.size(),
                  [&](std::size_t slot) { return points[slot]; });
    return index;
  };

  SECTION("ShouldMatchBruteForceGivenRandomPointsAndRadii") {
    std::vector<Coordinates> points = random_points(2000, 500.0, 1);
    SpatialIndex index = sized(points);
    std::vector<Coordinates> centers = random_points(50, 600.0, 2);
    auto odd = [](std::uint32_t slot) { return slot % 2 == 1; };

    for (double radius : {0.0, 3.0, 10.0, 25.0, 80.0, 2000.0}) {
      for (const Coordinates& center : centers) {
        CHECK(within(index, center, radius) ==
              brute_within(points, center, radius));
        CHECK(index.nearest(center, radius, odd) ==
              brute_nearest(points, center, radius, odd));
      }
    }
  }

  SECTION("ShouldSizeCellsToSpacingGivenPointsOnAPlane") {
    // 100 by 100 points 10 apart, all at height 0: a plane, so height is
    // ignored, and one point per cell means cells about 10 across.
    std::vector<Coordinates> points;
    for (int x = 0; x < 100; ++x) {
      for (int y = 0; y < 100; ++y) {
        points.push_back({10.0 * x, 10.0 * y, 0.0});
      }
    }

    CHECK_THAT(sized(points).cell_size(), WithinRel(9.9, 1e-12));
  }

  SECTION("ShouldResizeCellsGivenRebuildWithSparserPoints") {
    std::vector<Coordinates> dense = random_points(1000, 100.0, 3);
    std::vector<Coordinates> sparse = random_points(1000, 1000.0, 4);
    SpatialIndex index = sized(dense);
    double dense_size = index.cell_size();

    index.rebuild(sparse.size(),
                  [&](std::size_t slot) { return sparse[slot]; });

    CHECK(index.cell_size() > dense_size);
  }

  SECTION("ShouldKeepCellSizeGivenPointsThatSpanNothing") {
    std::vector<Coordinates> same(5, Coordinates{3.0, 3.0, 3.0});
    std::vector<Coordinates> one{{7.0, 7.0, 7.0}};

    CHECK(sized(same).cell_size() == 1.0);
    CHECK(sized(one).cell_size() == 1.0);
  }

  SECTION("ShouldIgnoreInfinitePointsGivenSizingCells") {
    std::vector<Coordinates> points;
    for (int x = 0; x < 10; ++x) {
      points.push_back({10.0 * x, 0.0, 0.0});
    }
    points.push_back({std::numeric_limits<double>::infinity(), 0.0, 0.0});
    SpatialIndex index = sized(points);

    CHECK_THAT(index.cell_size(), WithinRel(9.0, 1e-12));
    CHECK(within(index, {0.0, 0.0, 0.0}, 15.0) ==
          std::vector<std::uint32_t>{0, 1});
  }
}

}  // namespace simon::framework
