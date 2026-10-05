// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "framework/archetype.hpp"
#include "framework/spatial.hpp"
#include "framework/world.hpp"
#include "model/articulated.hpp"
#include "model/articulated_collision.hpp"
#include "model/articulated_dynamics.hpp"
#include "model/kinematics.hpp"
#include "model/units.hpp"

// The robotic simulation's components: each entity is one kinematic tree of
// a model, its state and its dynamics sized by its archetype's capacity.
namespace simon::robotic {

using SmallCapacity = model::TreeCapacity<4, 8>;
using LargeCapacity = model::TreeCapacity<16, 32>;

template <typename Capacity>
using TreeState = model::TreeState<Capacity>;
template <typename Capacity>
using TreeControl = model::TreeControl<Capacity>;
template <typename Capacity>
using TreeDynamics = model::TreeDynamics<Capacity>;

// The model and its trees, each with its kernel, read once and shared by
// the systems. It outlives the world and never moves.
class Mechanics final {
 public:
  explicit Mechanics(model::ArticulatedModel model);

  auto model() const -> const model::ArticulatedModel& { return model_; }
  auto trees() const -> const std::vector<model::Tree>& { return trees_; }

  // Whether tree `tree` fits a capacity.
  template <typename Capacity>
  auto fits(std::uint32_t tree) const -> bool {
    const model::Tree& t = trees_[tree];
    return t.bodies <= Capacity::bodies && t.dofs <= Capacity::dofs &&
           t.qpos <= Capacity::qpos;
  }

  // Which bodies may touch, each geom's bounding radius, and the geoms no
  // sphere bounds: planes, on the world or on bodies that cannot move.
  auto filter() const -> const model::BodyFilter& { return filter_; }
  auto radii() const -> const std::vector<double>& { return radii_; }
  auto unbounded() const -> const std::vector<std::uint32_t>& {
    return unbounded_;
  }

  template <typename Capacity>
  auto kernel(std::uint32_t tree) const -> const model::TreeKernel<Capacity>& {
    if constexpr (std::is_same_v<Capacity, SmallCapacity>) {
      return *small_[tree];
    } else {
      return *large_[tree];
    }
  }

 private:
  model::ArticulatedModel model_;
  std::vector<model::Tree> trees_;
  model::BodyFilter filter_;
  std::vector<double> radii_;
  std::vector<std::uint32_t> unbounded_;
  // Each tree's kernel at each capacity it fits, else none.
  std::vector<std::unique_ptr<model::TreeKernel<SmallCapacity>>> small_;
  std::vector<std::unique_ptr<model::TreeKernel<LargeCapacity>>> large_;
};

// A linear state feedback on the model's actuators: each control
// u = u0 - K (x - x0), x the model's positions then velocities, K by
// actuator, row by row, each tree's controls from its own state alone. None
// if the gains are empty.
struct Feedback final {
  std::vector<double> gains;
  std::vector<double> reference;  // x0.
  std::vector<double> offset;     // u0, by actuator.
};

// The contacts a step finds, ordered by their bodies, shared by the systems
// that find and resolve them. It outlives the world and never moves.
struct ContactSet final {
  std::vector<model::Contact> contacts;
};

// Which of the model's trees an entity is.
struct Mechanism final {
  std::uint32_t tree = 0;
};

// How many contacts a tree is in.
struct Touching final {
  std::uint32_t contacts = 0;
};

// Where a tree is, the world's spatial component: the sphere about its
// center of mass that holds every geom but planes, with its margin and gap.
struct TreeBound final {
  model::Array3 center{};
  double radius = 0.0;  // m.
};

inline auto distance(const TreeBound& a, const TreeBound& b) -> double {
  return std::hypot(a.center[0] - b.center[0], a.center[1] - b.center[1],
                    a.center[2] - b.center[2]);
}
inline auto coordinates(const TreeBound& bound) -> framework::Coordinates {
  return {bound.center[0], bound.center[1], bound.center[2]};
}
inline auto coordinate_length(const TreeBound&, double length) -> double {
  return length;
}
inline auto pose(const TreeBound& bound) -> model::Pose {
  return model::Pose{.position = model::meters(bound.center[0], bound.center[1],
                                               bound.center[2])};
}

namespace archetype {

using framework::Archetype;
using framework::Requires;

// A tree of at most 4 bodies and 8 degrees of freedom: a loose body, a
// pendulum, a cart-pole.
struct SmallTree final                                      //
    : Archetype<"small tree",                               //
                Requires<TreeBound, Mechanism, Touching,    //
                         TreeState<SmallCapacity>,          //
                         TreeControl<SmallCapacity>,        //
                         TreeDynamics<SmallCapacity>>> {};  //

// A tree of at most 16 bodies and 32 degrees of freedom: an arm, a
// humanoid.
struct LargeTree final                                      //
    : Archetype<"large tree",                               //
                Requires<TreeBound, Mechanism, Touching,    //
                         TreeState<LargeCapacity>,          //
                         TreeControl<LargeCapacity>,        //
                         TreeDynamics<LargeCapacity>>> {};  //

}  // namespace archetype

using World = framework::World<
    TreeBound,
    framework::TypeList<Mechanism, Touching, TreeState<SmallCapacity>,
                        TreeControl<SmallCapacity>, TreeDynamics<SmallCapacity>,
                        TreeState<LargeCapacity>, TreeControl<LargeCapacity>,
                        TreeDynamics<LargeCapacity>>,
    framework::TypeList<archetype::SmallTree, archetype::LargeTree>>;

}  // namespace simon::robotic
