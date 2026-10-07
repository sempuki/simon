// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "core/vocabulary.hpp"
#include "framework/system.hpp"
#include "model/gravity.hpp"
#include "model/kinematics.hpp"

// The leapfrog in its kick-drift-kick form, as galaxy codes such as GADGET-2
// step gravity (see model/REFERENCES.md): half a step's kick from the
// acceleration at the start, a whole step's drift, the acceleration at the
// new positions, and the other half kick. It is symplectic and
// time-reversible, so an orbit's energy oscillates without drifting, and it
// takes one force evaluation a step, since the closing kick's acceleration
// opens the next step. The acceleration must be computed once before the
// first step.
namespace simon::model {

struct OpenKick final                //
    : framework::System<Kinematics,  //
                        const Gravity> {
  auto operator()(auto&, framework::Entity,  //
                  Kinematics& kinematics,    //
                  const Gravity* gravity,    //
                  auto step) const -> void {
    if (!gravity) return;
    kinematics.velocity += gravity->acceleration * (0.5 * seconds(step.dt));
  }
};

struct Drift final  //
    : framework::System<Kinematics> {
  using SequenceAfterSystemList = framework::SystemList<OpenKick>;

  auto operator()(auto&, framework::Entity,  //
                  Kinematics& kinematics,    //
                  auto step) const -> void {
    kinematics.position += kinematics.velocity * seconds(step.dt);
  }
};

struct CloseKick final               //
    : framework::System<Kinematics,  //
                        const Gravity> {
  using SequenceAfterSystemList = framework::SystemList<Drift>;

  auto operator()(auto&, framework::Entity,  //
                  Kinematics& kinematics,    //
                  const Gravity* gravity,    //
                  auto step) const -> void {
    if (!gravity) return;
    kinematics.velocity += gravity->acceleration * (0.5 * seconds(step.dt));
  }
};

// One leapfrog step, its acceleration computed by `GravityType`.
template <typename GravityType>
using Leapfrog = framework::SystemList<OpenKick, Drift, GravityType, CloseKick>;

}  // namespace simon::model
