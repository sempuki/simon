// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>

#include "framework/system.hpp"
#include "model/kinematics.hpp"

namespace simon::model {

// Integrates each entity's Control into its Kinematics. Entities without a
// Control coast.
struct Integrate final : framework::System<Kinematics, const Control> {
  void operator()(auto&, framework::Entity, Kinematics& kinematics,
                  const Control* control, framework::Step step) const {
    integrate_midpoint(
        control ? control->acceleration : meters_per_second_squared(0, 0, 0),
        seconds(step.dt), lib::InOut(kinematics));
  }
};

using Motion = framework::SystemList<Integrate>;

}  // namespace simon::model
