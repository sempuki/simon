// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>

#include "core/vocabulary.hpp"
#include "framework/system.hpp"
#include "model/kinematics.hpp"

namespace simon::model {

// Integrates each entity's Control into its Kinematics. Entities without a
// Control coast.
struct Integrate final               //
    : framework::System<Kinematics,  //
                        const Control> {
  auto operator()(auto&, framework::Entity,  //
                  Kinematics& kinematics,    //
                  const Control* control,    //
                  Step step) const -> void {
    integrate_midpoint(
        control ? control->acceleration : meters_per_second_squared(0, 0, 0),
        seconds(step.dt), InOut(kinematics));
  }
};

using Motion = framework::SystemList<Integrate>;

}  // namespace simon::model
