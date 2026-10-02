// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cmath>
#include <expected>

#include "framework/archetype.hpp"
#include "framework/vocabulary.hpp"
#include "framework/world.hpp"

// A small world for framework tests: a one-dimensional position and two
// components.
namespace simon::framework::testing {

struct Position final {
  double x = 0.0;
};
inline auto distance(const Position& a, const Position& b) -> double {
  return std::abs(a.x - b.x);
}
inline auto pose(const Position& a) -> Position { return a; }
inline auto coordinates(const Position& a) -> Coordinates {
  return {a.x, 0.0, 0.0};
}
inline auto coordinate_length(const Position&, double length) -> double {
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

// Builds a world holding 16 entities: 8 bodies, 4 launchers and 4
// interceptors.
inline auto build_small_world(Out<TestWorld> world) -> void {
  std::expected<void, Status> built = TestWorld::set_up()
                                          .numbered(1)
                                          .holding<Body>(8)
                                          .holding<Launcher>(4)
                                          .holding<Interceptor>(4)
                                          .build(world);
  CHECK_POSTCONDITION(built.has_value());
}

}  // namespace simon::framework::testing
