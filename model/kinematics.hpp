// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "Eigen/Geometry"
#include "base/core.hpp"
#include "framework/spatial_index.hpp"
#include "model/units.hpp"

namespace simon::model {

using Quaternion = Eigen::Quaterniond;

struct Pose final {
  Position position = meters(0.0, 0.0, 0.0);
  Quaternion orientation = Quaternion::Identity();
};

// The toolkit's default spatial model: local Cartesian 3D, SI units.
struct Kinematics final {
  Position position = meters(0.0, 0.0, 0.0);
  Quaternion orientation = Quaternion::Identity();
  Velocity velocity = meters_per_second(0.0, 0.0, 0.0);
  Acceleration acceleration = meters_per_second_squared(0.0, 0.0, 0.0);
};

inline Length distance(const Kinematics& a, const Kinematics& b) {
  return norm(a.position - b.position);
}

// Coordinates for a spatial index, in meters.
inline framework::Coordinates coordinates(const Position& position) {
  const Vector3d& meters = position.numerical_value_ref_in(meter);
  return {meters.x(), meters.y(), meters.z()};
}
inline framework::Coordinates coordinates(const Kinematics& kinematics) {
  return coordinates(kinematics.position);
}
inline double coordinate_length(const Kinematics&, Length length) {
  return length.numerical_value_in(meter);
}

inline Pose pose(const Kinematics& kinematics) {
  return Pose{.position = kinematics.position,
              .orientation = kinematics.orientation};
}

// The commanded acceleration. Guidance and steering write it; Integrate reads
// it.
struct Control final {
  Acceleration acceleration = meters_per_second_squared(0.0, 0.0, 0.0);
};

// Advances `kinematics` by `dt` under constant `acceleration`, using the
// midpoint method (exact for constant acceleration).
inline void integrate_midpoint(lib::InOut<Kinematics> kinematics,
                               const Acceleration& acceleration, Time dt) {
  Velocity mid_velocity = kinematics->velocity + acceleration * (dt * 0.5);
  kinematics->position += mid_velocity * dt;
  kinematics->velocity += acceleration * dt;
  kinematics->acceleration = acceleration;
}

}  // namespace simon::model
