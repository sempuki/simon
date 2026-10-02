// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "framework/spatial_index.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <limits>

namespace simon::framework {

SpatialIndex::SpatialIndex(std::size_t capacity) : SpatialIndex{capacity, 1.0} {
  adaptive_ = true;
}

SpatialIndex::SpatialIndex(std::size_t capacity, double cell_size)
    : cell_size_{cell_size},
      inverse_cell_size_{1.0 / cell_size},
      mask_{std::bit_ceil(std::max<std::size_t>(capacity, 1)) - 1},
      starts_(mask_ + 2, 0) {
  CHECK_PRECONDITION(cell_size > 0.0);
  entries_.reserve(capacity);
  unsorted_.reserve(capacity);
  buckets_.reserve(capacity);
}

auto SpatialIndex::fit_cells() -> void {
  constexpr double INFINITE = std::numeric_limits<double>::infinity();
  Coordinates low{INFINITE, INFINITE, INFINITE};
  Coordinates high{-INFINITE, -INFINITE, -INFINITE};
  std::size_t counted = 0;
  for (const Entry& entry : unsorted_) {
    if (!std::isfinite(entry.point[0]) || !std::isfinite(entry.point[1]) ||
        !std::isfinite(entry.point[2])) {
      continue;
    }
    ++counted;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      low[axis] = std::min(low[axis], entry.point[axis]);
      high[axis] = std::max(high[axis], entry.point[axis]);
    }
  }
  if (counted < 2) {
    return;
  }
  double widest = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    widest = std::max(widest, high[axis] - low[axis]);
  }
  // An axis the points barely spread along, such as height over a plane,
  // would make the cells needlessly small.
  double volume = 1.0;
  int dimensions = 0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    double extent = high[axis] - low[axis];
    if (extent > widest * 1e-3) {
      volume *= extent;
      ++dimensions;
    }
  }
  if (dimensions == 0) {
    return;
  }
  double size =
      std::pow(volume / static_cast<double>(counted), 1.0 / dimensions);
  if (size > 0.0 && std::isfinite(size)) {
    cell_size_ = size;
    inverse_cell_size_ = 1.0 / size;
  }
}

}  // namespace simon::framework
