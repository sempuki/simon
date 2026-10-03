// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <vector>

#include "base/core.hpp"

namespace simon::framework {

// A point in a spatial model's own coordinates, as plain numbers.
using Coordinates = std::array<double, 3>;

// A uniform grid of cubic cells over a set of points, each point known by its
// slot (its position in the caller's array).
//
// Cells hash into a fixed table of buckets, so the grid is unbounded and costs
// memory in proportion to its capacity, not to the space it covers. A rebuild
// is a counting sort: every point lands in one flat array, grouped by bucket,
// with its coordinates beside its slot. A query reads a few runs of that array
// and never touches the caller's data for points it rejects. Nothing allocates
// after construction.
//
// Queries are deterministic: they visit cells in a fixed order, and within a
// bucket, points in the order the rebuild gave them.
//
// Cells cost a query time whether or not they hold points, and points cost it
// time whether or not they match, so the best cell size depends on how densely
// the points lie. An index built without a cell size picks one at every
// rebuild: about one point per cell over the box the points span, counting
// only the axes they spread along.
class SpatialIndex final {
 public:
  DECLARE_COPY_DELETE(SpatialIndex);
  DECLARE_MOVE_DEFAULT(SpatialIndex);

  // Room for `capacity` points, in cells sized at every rebuild to the points.
  explicit SpatialIndex(std::size_t capacity);
  // Room for `capacity` points in cells always `cell_size` across.
  SpatialIndex(std::size_t capacity, double cell_size);
  ~SpatialIndex() = default;

  auto size() const -> std::size_t { return entries_.size(); }
  auto capacity() const -> std::size_t { return entries_.capacity(); }
  auto cell_size() const -> double { return cell_size_; }

  // Replaces the contents with `count` points, where `coordinates_of(slot)`
  // gives the point in each slot.
  template <typename CoordinatesOfType>
  auto rebuild(std::size_t count, CoordinatesOfType&& coordinates_of) -> void {
    rebuild([&](auto&& insert) {
      for (std::size_t slot = 0; slot < count; ++slot) {
        insert(static_cast<std::uint32_t>(slot), coordinates_of(slot));
      }
    });
  }

  // Replaces the contents with the points `for_each_point(insert)` gives, by
  // calling `insert(slot, coordinates)` once for each, with distinct slots.
  template <typename ForEachPointType>
  auto rebuild(ForEachPointType&& for_each_point) -> void {
    unsorted_.clear();
    buckets_.clear();
    std::ranges::fill(starts_, 0);
    lowest_ = {0, 0, 0};
    highest_ = {-1, -1, -1};
    for_each_point([&](std::uint32_t slot, const Coordinates& point) {
      CHECK_PRECONDITION(unsorted_.size() < capacity());
      unsorted_.push_back(Entry{.point = point, .slot = slot});
    });
    if (adaptive_) {
      fit_cells();
    }
    for (const Entry& entry : unsorted_) {
      Cell cell = cell_of(entry.point);
      bool first = buckets_.empty();
      for (std::size_t axis = 0; axis < 3; ++axis) {
        lowest_[axis] =
            first ? cell[axis] : std::min(lowest_[axis], cell[axis]);
        highest_[axis] =
            first ? cell[axis] : std::max(highest_[axis], cell[axis]);
      }
      std::uint32_t bucket = hash_cell(cell);
      buckets_.push_back(bucket);
      ++starts_[bucket + 1];
    }
    for (std::size_t bucket = 1; bucket < starts_.size(); ++bucket) {
      starts_[bucket] += starts_[bucket - 1];
    }
    // starts_[b] is now where bucket b begins. Scatters in the order given,
    // using it as each bucket's cursor, so each bucket keeps that order.
    // Afterwards starts_[b] is where bucket b + 1 begins, so shift back.
    entries_.resize(unsorted_.size());
    for (std::size_t i = 0; i < unsorted_.size(); ++i) {
      entries_[starts_[buckets_[i]]++] = unsorted_[i];
    }
    for (std::size_t bucket = starts_.size() - 1; bucket > 0; --bucket) {
      starts_[bucket] = starts_[bucket - 1];
    }
    starts_[0] = 0;
  }

  // Visits the slot of every point within `radius` of `center`, as
  // `visit(slot)`.
  template <typename VisitorType>
  auto within(const Coordinates& center, double radius,
              VisitorType&& visit) const -> void {
    std::optional<Box> box = find_box(center, radius);
    if (!box) {
      return;
    }
    double limit = radius * radius;
    auto consider = [&](const Entry& entry) {
      if (squared_distance(entry.point, center) <= limit) {
        visit(entry.slot);
      }
    };
    if (box->cells() > static_cast<double>(entries_.size())) {
      // With more cells than points, reading every point is cheaper.
      std::ranges::for_each(entries_, consider);
      return;
    }
    for (std::int64_t x = box->low[0]; x <= box->high[0]; ++x) {
      for (std::int64_t y = box->low[1]; y <= box->high[1]; ++y) {
        for (std::int64_t z = box->low[2]; z <= box->high[2]; ++z) {
          visit_cell(Cell{x, y, z}, center, limit, consider);
        }
      }
    }
  }

  // The slot of the nearest point within `radius` of `center` for which
  // `accept(slot)` is true. Ties go to the lowest slot. `accept` is only asked
  // about points nearer than the best so far.
  template <typename AcceptType>
  auto nearest(const Coordinates& center, double radius,
               AcceptType&& accept) const -> std::optional<std::uint32_t> {
    std::optional<Box> box = find_box(center, radius);
    if (!box) {
      return std::nullopt;
    }
    std::optional<std::uint32_t> best;
    double best_distance = radius * radius;
    auto consider = [&](const Entry& entry) {
      double distance = squared_distance(entry.point, center);
      bool nearer = distance < best_distance || (distance == best_distance &&
                                                 (!best || entry.slot < *best));
      if (nearer && accept(entry.slot)) {
        best = entry.slot;
        best_distance = distance;
      }
    };
    if (box->cells() > static_cast<double>(entries_.size())) {
      std::ranges::for_each(entries_, consider);
      return best;
    }
    // Searches outward in rings of cells. Every point beyond ring k is at
    // least k cells from the center, so once the best is nearer, stop.
    Cell middle = cell_of(center);
    std::int64_t last = 0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      last = std::max({last, middle[axis] - box->low[axis],
                       box->high[axis] - middle[axis]});
    }
    for (std::int64_t ring = 0; ring <= last; ++ring) {
      visit_ring(*box, middle, ring, center, best_distance, consider);
      double reach = static_cast<double>(ring) * cell_size_;
      if (best && best_distance < reach * reach) {
        break;
      }
    }
    return best;
  }

 private:
  using Cell = std::array<std::int64_t, 3>;

  struct Entry final {
    Coordinates point{};
    std::uint32_t slot = 0;
  };

  // An inclusive range of cells, clipped to the cells that hold points.
  struct Box final {
    // In floating point, since a box can span more cells than fit in 64 bits.
    auto cells() const -> double {
      double count = 1.0;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        count *= static_cast<double>(high[axis] - low[axis] + 1);
      }
      return count;
    }

    Cell low{};
    Cell high{};
  };

  // Sizes cells for about one of the unsorted points per cell over the box
  // they span, counting only the axes they spread along. Keeps the current
  // size when they span nothing. Defined in spatial_index.cpp.
  auto fit_cells() -> void;

  auto cell_of(const Coordinates& point) const -> Cell {
    return Cell{cell_index(point[0]), cell_index(point[1]),
                cell_index(point[2])};
  }

  // The cell along one axis, clamped well inside int64, so an infinite or
  // huge coordinate (such as the edge of an infinite radius) still converts
  // and stays clear of overflow in ring arithmetic.
  auto cell_index(double coordinate) const -> std::int64_t {
    constexpr double LIMIT = 4.0e18;
    double scaled = std::floor(coordinate * inverse_cell_size_);
    if (std::isnan(scaled)) {
      return 0;
    }
    return static_cast<std::int64_t>(std::clamp(scaled, -LIMIT, LIMIT));
  }

  auto hash_cell(const Cell& cell) const -> std::uint32_t {
    std::uint64_t hash =
        static_cast<std::uint64_t>(cell[0]) * 0x9E3779B97F4A7C15ULL ^
        static_cast<std::uint64_t>(cell[1]) * 0xC2B2AE3D27D4EB4FULL ^
        static_cast<std::uint64_t>(cell[2]) * 0x165667B19E3779F9ULL;
    hash ^= hash >> 29;
    return static_cast<std::uint32_t>(hash & mask_);
  }

  static auto squared_distance(const Coordinates& a, const Coordinates& b)
      -> double {
    double x = a[0] - b[0];
    double y = a[1] - b[1];
    double z = a[2] - b[2];
    return x * x + y * y + z * z;
  }

  auto find_box(const Coordinates& center, double radius) const
      -> std::optional<Box> {
    if (entries_.empty() || !(radius >= 0.0)) {  // Also refuses NaN.
      return std::nullopt;
    }
    Cell low =
        cell_of({center[0] - radius, center[1] - radius, center[2] - radius});
    Cell high =
        cell_of({center[0] + radius, center[1] + radius, center[2] + radius});
    Box box;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      box.low[axis] = std::max(low[axis], lowest_[axis]);
      box.high[axis] = std::min(high[axis], highest_[axis]);
      if (box.low[axis] > box.high[axis]) {
        return std::nullopt;
      }
    }
    return box;
  }

  // A lower bound on the squared distance from `point` to any point in
  // `cell`. The slack keeps it a lower bound despite rounding in cell_of.
  auto squared_distance_to(const Cell& cell, const Coordinates& point) const
      -> double {
    double slack = cell_size_ * 1e-9;
    double total = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      double low = static_cast<double>(cell[axis]) * cell_size_;
      double gap = std::max({low - point[axis] - slack, 0.0,
                             point[axis] - (low + cell_size_) - slack});
      total += gap * gap;
    }
    return total;
  }

  // Considers the points in `cell`, skipping others that share its bucket,
  // unless every point in it is farther than `limit` (squared) from `center`.
  // `limit` is a reference, so a nearest search prunes by its best so far.
  template <typename ConsiderType>
  [[gnu::always_inline]] auto visit_cell(const Cell& cell,
                                         const Coordinates& center,
                                         const double& limit,
                                         ConsiderType& consider) const -> void {
    if (squared_distance_to(cell, center) > limit) {
      return;
    }
    std::uint32_t bucket = hash_cell(cell);
    for (std::uint32_t i = starts_[bucket]; i < starts_[bucket + 1]; ++i) {
      if (cell_of(entries_[i].point) == cell) {
        consider(entries_[i]);
      }
    }
  }

  // Considers the cells exactly `ring` cells from `middle` (in the largest
  // axis) that lie in `box`.
  template <typename ConsiderType>
  auto visit_ring(const Box& box, const Cell& middle, std::int64_t ring,
                  const Coordinates& center, const double& limit,
                  ConsiderType& consider) const -> void {
    std::int64_t x_low = std::max(box.low[0], middle[0] - ring);
    std::int64_t x_high = std::min(box.high[0], middle[0] + ring);
    std::int64_t y_low = std::max(box.low[1], middle[1] - ring);
    std::int64_t y_high = std::min(box.high[1], middle[1] + ring);
    std::int64_t z_low = std::max(box.low[2], middle[2] - ring);
    std::int64_t z_high = std::min(box.high[2], middle[2] + ring);
    for (std::int64_t x = x_low; x <= x_high; ++x) {
      for (std::int64_t y = y_low; y <= y_high; ++y) {
        if (std::abs(x - middle[0]) == ring ||
            std::abs(y - middle[1]) == ring) {
          for (std::int64_t z = z_low; z <= z_high; ++z) {
            visit_cell(Cell{x, y, z}, center, limit, consider);
          }
          continue;
        }
        // Inside the ring's walls, only its floor and ceiling.
        if (middle[2] - ring >= z_low) {
          visit_cell(Cell{x, y, middle[2] - ring}, center, limit, consider);
        }
        if (ring > 0 && middle[2] + ring <= z_high) {
          visit_cell(Cell{x, y, middle[2] + ring}, center, limit, consider);
        }
      }
    }
  }

  double cell_size_ = 1.0;
  double inverse_cell_size_ = 1.0;
  std::size_t mask_ = 0;
  // starts_[b] is where bucket b begins in entries_; starts_[b + 1] where it
  // ends.
  std::vector<std::uint32_t> starts_;
  std::vector<Entry> entries_;
  // Rebuild scratch: points in the order given, and each one's bucket.
  std::vector<Entry> unsorted_;
  std::vector<std::uint32_t> buckets_;
  // The range of cells that hold points, per axis.
  Cell lowest_{0, 0, 0};
  Cell highest_{-1, -1, -1};
  bool adaptive_ = false;  // Whether rebuilds size the cells.
};

}  // namespace simon::framework
