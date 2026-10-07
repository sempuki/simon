// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <vector>

#include "application/galactic/simulation_components.hpp"
#include "core/argument.hpp"
#include "core/units.hpp"
#include "framework/component_store.hpp"
#include "framework/entity.hpp"
#include "framework/system.hpp"
#include "model/gravity/gravity.hpp"
#include "model/kinematics.hpp"

namespace simon::galactic {

using framework::Entity;
using model::Kinematics;

template <typename SystemType>
using ProjectedWorld = framework::ProjectedWorld<SystemType, World>;

//-- Gravity -------------------------------------------------------------------

// Gathers into `sources` every body with a PointMass in `points` and a
// Kinematics in `motions`, in the PointMass store's order.
inline auto gather_sources(const framework::ComponentStore<PointMass>& points,
                           const framework::ComponentStore<Kinematics>& motions,
                           InOut<std::vector<gravity::Source>> sources)
    -> void {
  sources->clear();
  points.for_each([&](Entity entity, const PointMass& point) {
    const Kinematics* kinematics = motions.maybe_component_of(entity);
    if (!kinematics) return;
    sources->push_back(gravity::Source{
        .position = kinematics->position.numerical_value_ref_in(meter).eigen(),
        .mass = point.mass.numerical_value_in(kilogram),
        .id = entity.index});
  });
}

// Sums every source's pull on each body directly. It costs N^2 and is exact
// to rounding: the reference for faster methods. Each body writes only its
// own Gravity, from sources gathered once a step, so bodies run in any order.
// It does nothing unless enabled.
struct SumGravity final           //
    : framework::System<Gravity,  //
                        const Kinematics> {
  using SystemWorld = ProjectedWorld<SumGravity>;
  using AllowComponentList = framework::TypeList<Kinematics, PointMass>;

  auto prepare(SystemWorld& world) -> bool {
    if (!enabled) return false;
    gather_sources(world.store_of<PointMass>(), world.store_of<Kinematics>(),
                   InOut(sources));
    return true;
  }

  auto operator()(SystemWorld&, Entity self,  //
                  Gravity& gravity,           //
                  const Kinematics* kinematics) const -> void {
    if (!kinematics) return;
    gravity.acceleration =
        QuantityVector{gravity::sum_acceleration(
            sources, self.index,
            kinematics->position.numerical_value_ref_in(meter).eigen(),
            softening.numerical_value_in(meter))} *
        meter_per_second_squared;
  }

  Length softening = 0.0 * meter;
  std::vector<gravity::Source> sources;
  bool enabled = true;
};

// Computes each body's gravity from a Barnes and Hut tree of every source,
// built once a step. Each body walks the tree on its own and writes only its
// own Gravity. Its forces are not exactly equal and opposite, so momentum
// drifts by the tree's error. It does nothing unless enabled.
struct TreeGravity final          //
    : framework::System<Gravity,  //
                        const Kinematics> {
  using SystemWorld = ProjectedWorld<TreeGravity>;
  using AllowComponentList = framework::TypeList<Kinematics, PointMass>;

  auto prepare(SystemWorld& world) -> bool {
    if (!enabled) return false;
    gather_sources(world.store_of<PointMass>(), world.store_of<Kinematics>(),
                   InOut(sources));
    tree.build(sources);
    return true;
  }

  auto operator()(SystemWorld&, Entity self,  //
                  Gravity& gravity,           //
                  const Kinematics* kinematics) const -> void {
    if (!kinematics) return;
    gravity.acceleration =
        QuantityVector{tree.compute_acceleration(
            self.index,
            kinematics->position.numerical_value_ref_in(meter).eigen(),
            opening_angle, softening.numerical_value_in(meter))} *
        meter_per_second_squared;
  }

  double opening_angle = 0.5;
  Length softening = 0.0 * meter;
  std::vector<gravity::Source> sources;
  gravity::BarnesHutTree tree;
  bool enabled = false;
};

//-- The leapfrog --------------------------------------------------------------

// The leapfrog in its kick-drift-kick form, as galaxy codes such as GADGET-2
// step gravity (see model/REFERENCES.md): half a step's kick from the
// acceleration at the start, a whole step's drift, the acceleration at the
// new positions, and the other half kick. It is symplectic and
// time-reversible, so an orbit's energy oscillates without drifting, and it
// takes one force evaluation a step, since the closing kick's acceleration
// opens the next step. The acceleration must be computed once before the
// first step.
struct OpenKick final                //
    : framework::System<Kinematics,  //
                        const Gravity> {
  using SystemWorld = ProjectedWorld<OpenKick>;

  auto operator()(SystemWorld&, Entity,    //
                  Kinematics& kinematics,  //
                  const Gravity* gravity,  //
                  auto step) const -> void {
    if (!gravity) return;
    kinematics.velocity += gravity->acceleration * (0.5 * seconds(step.dt));
  }
};

struct Drift final  //
    : framework::System<Kinematics> {
  using SystemWorld = ProjectedWorld<Drift>;
  using SequenceAfterSystemList = framework::SystemList<OpenKick>;

  auto operator()(SystemWorld&, Entity,    //
                  Kinematics& kinematics,  //
                  auto step) const -> void {
    kinematics.position += kinematics.velocity * seconds(step.dt);
  }
};

struct CloseKick final               //
    : framework::System<Kinematics,  //
                        const Gravity> {
  using SystemWorld = ProjectedWorld<CloseKick>;
  using SequenceAfterSystemList = framework::SystemList<Drift>;

  auto operator()(SystemWorld&, Entity,    //
                  Kinematics& kinematics,  //
                  const Gravity* gravity,  //
                  auto step) const -> void {
    if (!gravity) return;
    kinematics.velocity += gravity->acceleration * (0.5 * seconds(step.dt));
  }
};

// One leapfrog step, its acceleration computed by `GravityType`.
template <typename GravityType>
using Leapfrog = framework::SystemList<OpenKick, Drift, GravityType, CloseKick>;

//-- Schedules -----------------------------------------------------------------

// Gravity by direct summation or by tree, whichever the scenario enables.
using Gravities = framework::SystemList<SumGravity, TreeGravity>;

// Computes every body's gravity before the first step.
using StartScheduler = framework::Scheduler<World, Gravities>;

using Schedule = Leapfrog<Gravities>;
using Scheduler = framework::Scheduler<World, Schedule>;

}  // namespace simon::galactic
