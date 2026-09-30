// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cmath>

#include "core/archetype.hpp"
#include "core/world.hpp"

// A small world for core tests: a one-dimensional position and two components.
namespace simon::core::testing {

struct Position {
  double x = 0.0;
};
inline double distance(const Position& a, const Position& b) {
  return std::abs(a.x - b.x);
}
inline Position pose(const Position& a) { return a; }

struct Velocity {
  double x = 0.0;
};

struct Health {
  double points = 0.0;
};

using TestWorld = World<Position, Velocity, Health>;

// A body may have any of the test components.
struct Body
    : Archetype<"body", Requires<>, Allows<Position, Velocity, Health>> {};

struct Launcher : Archetype<"launcher", Requires<Position>> {};
struct Interceptor : Archetype<"interceptor", Requires<Position, Velocity>> {};

inline WorldConfiguration small_world() {
  return WorldConfiguration{.number = 1, .entities = 16, .components = 16};
}

}  // namespace simon::core::testing
