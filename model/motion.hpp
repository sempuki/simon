// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>

#include "framework/system.hpp"
#include "model/kinematics.hpp"

namespace simon::model {

// Integrates each entity's Control into its Kinematics. Entities without a
// Control coast.
struct Integrate : framework::System<Kinematics, const Control> {
  void operator()(framework::Entity, Kinematics& kinematics,
                  const Control* control, auto& context) const {
    integrate_midpoint(
        lib::InOut(kinematics),
        control ? control->acceleration : metres_per_second_squared(0, 0, 0),
        seconds(context.step().dt));
  }
};

using Motion = framework::Systems<Integrate>;

}  // namespace simon::model
