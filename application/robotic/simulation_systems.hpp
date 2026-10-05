// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <span>

#include "application/robotic/simulation_components.hpp"
#include "framework/system.hpp"
#include "framework/vocabulary.hpp"

// The robotic simulation's systems, each once per capacity: Control sets
// each tree's controls, Forward computes its poses, mass matrix and smooth
// accelerations, Bound its sphere, and Integrate steps its state.
namespace simon::robotic {

using framework::Entity;
using framework::Step;
using framework::System;
using framework::SystemList;

template <typename SystemType>
using ProjectedWorld = framework::ProjectedWorld<SystemType, World>;

// Each step, a tree's controls by the feedback, if there is one, from its
// state as the last step left it.
template <typename Capacity>
struct Control final                     //
    : System<TreeControl<Capacity>,      //
             const TreeState<Capacity>,  //
             const Mechanism> {
  using SystemWorld = ProjectedWorld<Control>;

  Control(const Mechanics& mechanics, const Feedback& feedback)
      : mechanics_{&mechanics}, feedback_{&feedback} {}

  auto operator()(SystemWorld&, Entity,              //
                  TreeControl<Capacity>& control,    //
                  const TreeState<Capacity>* state,  //
                  const Mechanism* mechanism) const -> void;

 private:
  const Mechanics* mechanics_ = nullptr;
  const Feedback* feedback_ = nullptr;
};

// Each step, a tree's dynamics without constraints, from its state.
template <typename Capacity>
struct Forward final                       //
    : System<TreeDynamics<Capacity>,       //
             const TreeState<Capacity>,    //
             const TreeControl<Capacity>,  //
             const Mechanism> {
  using SystemWorld = ProjectedWorld<Forward>;
  using SequenceAfterSystemList = SystemList<Control<Capacity>>;

  explicit Forward(const Mechanics& mechanics) : mechanics_{&mechanics} {}

  auto operator()(SystemWorld&, Entity,                  //
                  TreeDynamics<Capacity>& dynamics,      //
                  const TreeState<Capacity>* state,      //
                  const TreeControl<Capacity>* control,  //
                  const Mechanism* mechanism) const -> void {
    if (state && control && mechanism) {
      mechanics_->kernel<Capacity>(mechanism->tree)
          .forward(*state, *control, dynamics);
    }
  }

 private:
  const Mechanics* mechanics_ = nullptr;
};

// After Forward, a tree's sphere: about its center of mass, out to the
// farthest of its geoms' bounding spheres.
template <typename Capacity>
struct Bound final                          //
    : System<TreeBound,                     //
             const TreeDynamics<Capacity>,  //
             const Mechanism> {
  using SystemWorld = ProjectedWorld<Bound>;
  using SequenceAfterSystemList = SystemList<Forward<Capacity>>;

  explicit Bound(const Mechanics& mechanics) : mechanics_{&mechanics} {}

  auto operator()(SystemWorld&, Entity,                    //
                  TreeBound& bound,                        //
                  const TreeDynamics<Capacity>* dynamics,  //
                  const Mechanism* mechanism) const -> void;

 private:
  const Mechanics* mechanics_ = nullptr;
};

// After Bound, a step of semi-implicit Euler.
template <typename Capacity>
struct Integrate final                      //
    : System<TreeState<Capacity>,           //
             const TreeDynamics<Capacity>,  //
             const Mechanism> {
  using SystemWorld = ProjectedWorld<Integrate>;
  using SequenceAfterSystemList = SystemList<Bound<Capacity>>;

  explicit Integrate(const Mechanics& mechanics) : mechanics_{&mechanics} {}

  auto operator()(SystemWorld&, Entity,                    //
                  TreeState<Capacity>& state,              //
                  const TreeDynamics<Capacity>* dynamics,  //
                  const Mechanism* mechanism) const -> void {
    if (dynamics && mechanism) {
      mechanics_->kernel<Capacity>(mechanism->tree)
          .advance(*dynamics, dynamics->smooth, dynamics->acceleration, state);
    }
  }

 private:
  const Mechanics* mechanics_ = nullptr;
};

using Schedule = SystemList<Control<SmallCapacity>, Control<LargeCapacity>,
                            Forward<SmallCapacity>, Forward<LargeCapacity>,
                            Bound<SmallCapacity>, Bound<LargeCapacity>,
                            Integrate<SmallCapacity>, Integrate<LargeCapacity>>;

inline auto make_schedule(const Mechanics& mechanics, const Feedback& feedback)
    -> Schedule {
  return Schedule{Control<SmallCapacity>{mechanics, feedback},
                  Control<LargeCapacity>{mechanics, feedback},
                  Forward<SmallCapacity>{mechanics},
                  Forward<LargeCapacity>{mechanics},
                  Bound<SmallCapacity>{mechanics},
                  Bound<LargeCapacity>{mechanics},
                  Integrate<SmallCapacity>{mechanics},
                  Integrate<LargeCapacity>{mechanics}};
}
using Scheduler = framework::Scheduler<World, Schedule>;

template <typename Capacity>
auto Control<Capacity>::operator()(SystemWorld&, Entity,              //
                                   TreeControl<Capacity>& control,    //
                                   const TreeState<Capacity>* state,  //
                                   const Mechanism* mechanism) const -> void {
  const Feedback& law = *feedback_;
  if (law.gains.empty() || !state || !mechanism) {
    return;
  }
  const model::ArticulatedModel& m = mechanics_->model();
  const model::Tree& tree = mechanics_->trees()[mechanism->tree];
  std::size_t nq = m.qpos0.size();
  std::size_t width = nq + m.dofs.size();
  for (std::size_t a = 0; a < tree.actuators.size(); ++a) {
    std::uint32_t row = tree.actuators[a];
    double u = law.offset[row];
    for (std::uint32_t q = 0; q < tree.qpos; ++q) {
      std::size_t column = tree.first_qpos + q;
      u -= law.gains[row * width + column] *
           (state->qpos[q] - law.reference[column]);
    }
    for (std::uint32_t v = 0; v < tree.dofs; ++v) {
      std::size_t column = nq + tree.first_dof + v;
      u -= law.gains[row * width + column] *
           (state->qvel[v] - law.reference[column]);
    }
    control.control[a] = u;
  }
}

template <typename Capacity>
auto Bound<Capacity>::operator()(SystemWorld&, Entity,                    //
                                 TreeBound& bound,                        //
                                 const TreeDynamics<Capacity>* dynamics,  //
                                 const Mechanism* mechanism) const -> void {
  if (!dynamics || !mechanism) {
    return;
  }
  const model::ArticulatedModel& m = mechanics_->model();
  const model::Tree& tree = mechanics_->trees()[mechanism->tree];
  bound.center = dynamics->com;
  bound.radius = 0.0;
  for (std::uint32_t g = tree.first_geom; g < tree.first_geom + tree.geoms;
       ++g) {
    const model::Geom& geom = m.geoms[g];
    std::uint32_t b = geom.body - tree.first_body;
    model::Array3 at =
        model::articulated::multiply(dynamics->xmat[b], geom.pos);
    double reach = 0.0;
    const model::Array3& s = geom.size;
    switch (geom.type) {
      case model::GeomType::SPHERE:
        reach = s[0];
        break;
      case model::GeomType::CAPSULE:
        reach = s[0] + s[1];
        break;
      case model::GeomType::CYLINDER:
        reach = std::hypot(s[0], s[1]);
        break;
      default:
        reach = std::hypot(s[0], s[1], s[2]);
        break;
    }
    double apart = std::hypot(dynamics->xpos[b][0] + at[0] - bound.center[0],
                              dynamics->xpos[b][1] + at[1] - bound.center[1],
                              dynamics->xpos[b][2] + at[2] - bound.center[2]);
    bound.radius = std::max(bound.radius, apart + reach);
  }
}

}  // namespace simon::robotic
