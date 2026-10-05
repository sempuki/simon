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
  // Each tree's kernel at each capacity it fits, else none.
  std::vector<std::unique_ptr<model::TreeKernel<SmallCapacity>>> small_;
  std::vector<std::unique_ptr<model::TreeKernel<LargeCapacity>>> large_;
};

// Which of the model's trees an entity is.
struct Mechanism final {
  std::uint32_t tree = 0;
};

// Where a tree is, the world's spatial component: the sphere about its
// center of mass that holds every geom.
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
                Requires<TreeBound, Mechanism,              //
                         TreeState<SmallCapacity>,          //
                         TreeDynamics<SmallCapacity>>> {};  //

// A tree of at most 16 bodies and 32 degrees of freedom: an arm, a
// humanoid.
struct LargeTree final                                      //
    : Archetype<"large tree",                               //
                Requires<TreeBound, Mechanism,              //
                         TreeState<LargeCapacity>,          //
                         TreeDynamics<LargeCapacity>>> {};  //

}  // namespace archetype

using World = framework::World<
    TreeBound,
    framework::TypeList<Mechanism, TreeState<SmallCapacity>,
                        TreeDynamics<SmallCapacity>, TreeState<LargeCapacity>,
                        TreeDynamics<LargeCapacity>>,
    framework::TypeList<archetype::SmallTree, archetype::LargeTree>>;

}  // namespace simon::robotic
