// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>

namespace simon {

// A point in plain numbers, in whatever unit a spatial component chooses, for
// the spatial index and anything else that asks where things are.
using Coordinates = std::array<double, 3>;

}  // namespace simon
