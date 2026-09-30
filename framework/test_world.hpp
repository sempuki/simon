// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cmath>

#include "framework/archetype.hpp"
#include "framework/world.hpp"

// A small world for framework tests: a one-dimensional position and two
// components.
namespace simon::framework::testing {

struct Position final {
  double x = 0.0;
};
inline double distance(const Position& a, const Position& b) {
  return std::abs(a.x - b.x);
}
inline Position pose(const Position& a) { return a; }
inline Coordinates coordinates(const Position& a) { return {a.x, 0.0, 0.0}; }
inline double coordinate_length(const Position&, double length) {
  return length;
}

struct Velocity final {
  double x = 0.0;
};

struct Health final {
  double points = 0.0;
};

// A body may have any of the test components.
struct Body final
    : Archetype<"body", Requires<>, Allows<Position, Velocity, Health>> {};

struct Launcher final : Archetype<"launcher", Requires<Position>> {};
struct Interceptor final
    : Archetype<"interceptor", Requires<Position, Velocity>> {};

using TestWorld = World<Position, TypeList<Velocity, Health>,
                        TypeList<Body, Launcher, Interceptor>>;

inline WorldConfiguration small_world() {
  return WorldConfiguration{.number = 1, .entities = 16, .components = 16};
}

}  // namespace simon::framework::testing
