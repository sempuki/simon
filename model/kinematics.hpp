// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "Eigen/Geometry"
#include "base/core.hpp"
#include "core/coordinates.hpp"
#include "core/units.hpp"
#include "core/vocabulary.hpp"

namespace simon::model {

struct Pose final {
  Position position = meters(0.0, 0.0, 0.0);
  Quaternion orientation = Quaternion::Identity();
};

// The toolkit's default spatial model: a point mass in local Cartesian 3D, SI
// units. It holds only what motion and spatial queries read every step (48
// bytes). An entity that needs an attitude also has an Orientation; the
// commanded acceleration is in Control.
struct Kinematics final {
  Position position = meters(0.0, 0.0, 0.0);
  Velocity velocity = meters_per_second(0.0, 0.0, 0.0);
};

// The attitude of an entity that has one, such as a sensor that points.
struct Orientation final {
  Quaternion orientation = Quaternion::Identity();
};

inline auto distance(const Kinematics& a, const Kinematics& b) -> Length {
  return norm(a.position - b.position);
}

// Whether `a` and `b` are at most `reach` apart. Compares squares, so it takes
// no square root: cheaper than distance() in a check most entities fail.
inline auto within_distance(const Kinematics& a, const Kinematics& b,
                            Length reach) -> bool {
  auto apart = a.position - b.position;
  return dot(apart, apart) <= reach * reach;
}

// Coordinates for a spatial index, in meters.
inline auto coordinates(const Position& position) -> Coordinates {
  const QuantityVector& meters = position.numerical_value_ref_in(meter);
  return {meters.x(), meters.y(), meters.z()};
}
inline auto coordinates(const Kinematics& kinematics) -> Coordinates {
  return coordinates(kinematics.position);
}
inline auto coordinate_length(const Kinematics&, Length length) -> double {
  return length.numerical_value_in(meter);
}

// A point mass has no attitude of its own, so its orientation is the
// identity.
inline auto pose(const Kinematics& kinematics) -> Pose {
  return Pose{.position = kinematics.position};
}
inline auto pose(const Kinematics& kinematics, const Orientation& orientation)
    -> Pose {
  return Pose{.position = kinematics.position,
              .orientation = orientation.orientation};
}

// The commanded acceleration. Guidance and steering write it; Integrate reads
// it.
struct Control final {
  Acceleration acceleration = meters_per_second_squared(0.0, 0.0, 0.0);
};

// Advances `kinematics` by `dt` under constant `acceleration`, using the
// midpoint method (exact for constant acceleration; Hairer, Norsett and
// Wanner; see model/REFERENCES.md).
inline auto integrate_midpoint(const Acceleration& acceleration, Time dt,
                               InOut<Kinematics> kinematics) -> void {
  Velocity mid_velocity = kinematics->velocity + acceleration * (dt * 0.5);
  kinematics->position += mid_velocity * dt;
  kinematics->velocity += acceleration * dt;
}

}  // namespace simon::model
