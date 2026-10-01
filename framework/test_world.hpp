// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cmath>
#include <expected>
#include <utility>

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

// Room for 16 entities: 8 bodies, 4 launchers and 4 interceptors.
inline TestWorld small_world() {
  std::expected<TestWorld, Status> world = TestWorld::set_up()
                                               .numbered(1)
                                               .room_for<Body>(8)
                                               .room_for<Launcher>(4)
                                               .room_for<Interceptor>(4)
                                               .build();
  CHECK_POSTCONDITION(world.has_value());
  return *std::move(world);
}

}  // namespace simon::framework::testing
