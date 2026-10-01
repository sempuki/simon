// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/spatial_index.hpp"

#include <algorithm>
#include <bit>
#include <cstddef>

namespace simon::framework {

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

}  // namespace simon::framework
