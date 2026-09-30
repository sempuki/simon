// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>

#include "core/system.hpp"
#include "model/kinematics.hpp"

namespace simon::model {

// Integrates each entity's Control into its Kinematics. Entities without a
// Control coast.
struct Integrate : core::System<Kinematics, const Control> {
  void operator()(core::Entity, Kinematics& kinematics, const Control* control,
                  auto& context) const {
    integrate_midpoint(kinematics,
                       control ? control->acceleration : metres_per_second_squared(0, 0, 0),
                       seconds(context.step().dt));
  }
};

using Motion = core::Systems<Integrate>;

}  // namespace simon::model
