// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "Eigen/Geometry"
#include "base/core.hpp"
#include "model/units.hpp"

namespace simon::model {

using Quaternion = Eigen::Quaterniond;

struct Pose {
  Position position = metres(0.0, 0.0, 0.0);
  Quaternion orientation = Quaternion::Identity();
};

// The toolkit's default spatial model: local Cartesian 3D, SI units.
struct Kinematics {
  Position position = metres(0.0, 0.0, 0.0);
  Quaternion orientation = Quaternion::Identity();
  Velocity velocity = metres_per_second(0.0, 0.0, 0.0);
  Acceleration acceleration = metres_per_second_squared(0.0, 0.0, 0.0);
};

inline Length distance(const Kinematics& a, const Kinematics& b) {
  return norm(a.position - b.position);
}

inline Pose pose(const Kinematics& kinematics) {
  return Pose{.position = kinematics.position,
              .orientation = kinematics.orientation};
}

// The commanded acceleration. Guidance and steering write it; Integrate reads
// it.
struct Control {
  Acceleration acceleration = metres_per_second_squared(0.0, 0.0, 0.0);
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
