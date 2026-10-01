// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <concepts>
#include <utility>

#include "framework/spatial_index.hpp"

namespace simon::framework {

// What a world needs to know about space: a distance, a pose, and, for its
// spatial index, plain coordinates. The world indexes every entity-component
// of its spatial type. Distances may carry units; `coordinate_length` gives
// one as a plain number in the same unit as `coordinates`.
template <typename Type>
concept Spatial = requires(const Type& a, const Type& b) {
  { coordinates(a) } -> std::same_as<Coordinates>;
  { coordinate_length(a, distance(a, b)) } -> std::same_as<double>;
  { distance(a, b) < distance(a, b) } -> std::convertible_to<bool>;
  pose(a);
};

template <Spatial Type>
using distance_of_t = decltype(distance(std::declval<const Type&>(),
                                        std::declval<const Type&>()));

}  // namespace simon::framework
