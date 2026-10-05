// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <span>
#include <vector>

#include "application/robotic/simulation_components.hpp"
#include "framework/system.hpp"
#include "framework/vocabulary.hpp"
#include "model/articulated_constraint.hpp"

// The robotic simulation's systems, each once per capacity but Collide:
// Control sets each tree's controls, Forward computes its poses, mass matrix
// and smooth accelerations, Bound its sphere, Collide finds every contact,
// and Integrate steps its state.
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
          .forward(*state, *control, Out(dynamics));
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

// After Bound, every tree's geom poses, then the contacts of the geoms that
// may touch: the world's and the planes' with every body, a tree's own
// bodies' with each other, and those of trees whose spheres overlap, found
// in the world's spatial index. Each tree's count of them.
struct Collide final    //
    : System<Touching,  //
             const Mechanism> {
  using SystemWorld = ProjectedWorld<Collide>;
  using AllowComponentList =
      framework::TypeList<TreeBound, Mechanism, TreeDynamics<SingleCapacity>,
                          TreeDynamics<SmallCapacity>,
                          TreeDynamics<LargeCapacity>,
                          TreeDynamics<HugeCapacity>>;
  using SequenceAfterSystemList =
      SystemList<Bound<SingleCapacity>, Bound<SmallCapacity>,
                 Bound<LargeCapacity>, Bound<HugeCapacity>>;

  Collide(const Mechanics& mechanics, Depend<ContactSet> contacts)
      : mechanics_{&mechanics}, contacts_{contacts.get()} {}

  auto prepare(SystemWorld& world) -> bool;

  auto operator()(SystemWorld&, Entity,  //
                  Touching& touching,    //
                  const Mechanism* mechanism) const -> void {
    touching.contacts = mechanism ? touching_[mechanism->tree] : 0;
  }

 private:
  template <typename Capacity>
  auto place_geoms(SystemWorld& world) -> void;
  auto collide_bodies(std::uint32_t first, std::uint32_t second) -> void;
  auto collide_trees(std::uint32_t first, std::uint32_t second) -> void;

  static constexpr std::uint32_t NO_TREE = ~std::uint32_t{0};

  const Mechanics* mechanics_ = nullptr;
  ContactSet* contacts_ = nullptr;
  std::vector<model::GeomFrame> frames_;  // By geom.
  std::vector<std::uint8_t> unbounded_;   // By geom.
  std::vector<std::uint32_t> tree_of_;    // By body, none for the world.
  std::vector<std::uint32_t> touching_;   // By tree.
};

// After Collide, the constraints of every tree, as rows: its dofs' and
// tendons' dry friction, its joints' and tendons' limits, and the contacts'
// pyramidal friction cones. Trees that contacts and tendons join form an
// island, and each island is solved on
// its own, by Newton's method or projected Gauss–Seidel as the model says;
// each tree is told its island.
struct Solve final    //
    : System<Island,  //
             const Mechanism> {
  using SystemWorld = ProjectedWorld<Solve>;
  using AllowComponentList =
      framework::TypeList<Mechanism, TreeState<SingleCapacity>,
                          TreeDynamics<SingleCapacity>,
                          TreeState<SmallCapacity>, TreeDynamics<SmallCapacity>,
                          TreeState<LargeCapacity>, TreeDynamics<LargeCapacity>,
                          TreeState<HugeCapacity>, TreeDynamics<HugeCapacity>>;
  using SequenceAfterSystemList = SystemList<Collide>;

  Solve(const Mechanics& mechanics, const ContactSet& contacts,
        Depend<ConstraintSolution> solution, bool enabled)
      : mechanics_{&mechanics},
        contacts_{&contacts},
        solution_{solution.get()},
        enabled_{enabled} {}

  auto prepare(SystemWorld& world) -> bool;

  auto operator()(SystemWorld&, Entity,  //
                  Island& island,        //
                  const Mechanism* mechanism) const -> void {
    island = mechanism ? islands_[mechanism->tree] : Island{};
  }

 private:
  // Where a tree's dynamics and state lie in their stores this step.
  struct TreeData final {
    const double* mass = nullptr;  // Rows `stride` long, the lower triangle.
    std::size_t stride = 0;
    const model::Spatial* cdof = nullptr;
    const model::Array3* com = nullptr;
    const double* qacc_smooth = nullptr;
    const double* qfrc_smooth = nullptr;
    const double* qpos = nullptr;
    const double* qvel = nullptr;
    const double* warmstart = nullptr;
  };

  template <typename Capacity>
  auto gather(SystemWorld& world) -> void;
  auto find_root(std::uint32_t tree) -> std::uint32_t;
  auto tree_of_geom(std::uint32_t geom) const -> std::uint32_t;
  auto tree_of_joint(std::uint32_t joint) const -> std::uint32_t;
  auto tendon_length(std::uint32_t tendon) const -> double;
  auto tendon_reached(std::uint32_t tendon) const -> bool;
  auto solve_island(std::span<const std::uint32_t> trees,
                    std::span<const std::uint32_t> contacts) -> std::uint32_t;

  static constexpr std::uint32_t NO_TREE = ~std::uint32_t{0};

  const Mechanics* mechanics_ = nullptr;
  const ContactSet* contacts_ = nullptr;
  ConstraintSolution* solution_ = nullptr;
  bool enabled_ = true;
  std::vector<TreeData> data_;             // By tree.
  std::vector<std::uint8_t> rubs_;         // By tree: has dry friction.
  std::vector<std::uint8_t> limited_;      // By tree: has a limited joint.
  std::vector<std::uint32_t> offset_;      // By tree: its first dof's column.
  std::vector<std::uint32_t> joint_tree_;  // By joint.
  // By tree: the tendons whose first joint is on it.
  std::vector<std::vector<std::uint32_t>> tree_tendons_;
  std::vector<Island> islands_;        // By tree.
  std::vector<std::uint32_t> parent_;  // By tree, for union and find.
  model::ConstraintProblem problem_;
  model::ConstraintSolution answer_;
  // Vectors an island's rows are built in, kept from island to island.
  struct Scratch final {
    std::vector<double> qvel;
    std::vector<double> row;
    std::vector<std::uint32_t> tendons;
    std::vector<double> translation;
    std::vector<double> rotation;
    std::vector<double> jp;
    std::vector<double> jr;
    std::vector<double> framed;
  } scratch_;
};

// After Solve, a step of semi-implicit Euler, at the accelerations the
// constraints leave, with the dampers taken implicitly at the forces.
template <typename Capacity>
struct Integrate final                      //
    : System<TreeState<Capacity>,           //
             const TreeDynamics<Capacity>,  //
             const Mechanism> {
  using SystemWorld = ProjectedWorld<Integrate>;
  using SequenceAfterSystemList = SystemList<Solve>;

  Integrate(const Mechanics& mechanics, const ConstraintSolution& solution)
      : mechanics_{&mechanics}, solution_{&solution} {}

  auto operator()(SystemWorld&, Entity,                    //
                  TreeState<Capacity>& state,              //
                  const TreeDynamics<Capacity>* dynamics,  //
                  const Mechanism* mechanism) const -> void {
    if (!dynamics || !mechanism) {
      return;
    }
    const model::Tree& tree = mechanics_->trees()[mechanism->tree];
    const auto& kernel = mechanics_->kernel<Capacity>(mechanism->tree);
    if (solution_->constrained.empty() ||
        solution_->constrained[mechanism->tree] == 0) {
      state.warmstart = dynamics->acceleration;
      kernel.advance(*dynamics, dynamics->smooth, dynamics->acceleration,
                     InOut(state));
      return;
    }
    std::array<double, Capacity::dofs> force{};
    for (std::uint32_t i = 0; i < tree.dofs; ++i) {
      force[i] =
          dynamics->smooth[i] + solution_->qfrc_constraint[tree.first_dof + i];
      state.warmstart[i] = solution_->qacc[tree.first_dof + i];
    }
    kernel.advance(*dynamics, force, state.warmstart, InOut(state));
  }

 private:
  const Mechanics* mechanics_ = nullptr;
  const ConstraintSolution* solution_ = nullptr;
};

using Schedule = SystemList<
    Control<SingleCapacity>, Control<SmallCapacity>, Control<LargeCapacity>,
    Control<HugeCapacity>, Forward<SingleCapacity>, Forward<SmallCapacity>,
    Forward<LargeCapacity>, Forward<HugeCapacity>, Bound<SingleCapacity>,
    Bound<SmallCapacity>, Bound<LargeCapacity>, Bound<HugeCapacity>, Collide,
    Solve, Integrate<SingleCapacity>, Integrate<SmallCapacity>,
    Integrate<LargeCapacity>, Integrate<HugeCapacity>>;

inline auto make_schedule(const Mechanics& mechanics, const Feedback& feedback,
                          Depend<ContactSet> contacts,
                          Depend<ConstraintSolution> solution, bool constrained)
    -> Schedule {
  return Schedule{Control<SingleCapacity>{mechanics, feedback},
                  Control<SmallCapacity>{mechanics, feedback},
                  Control<LargeCapacity>{mechanics, feedback},
                  Control<HugeCapacity>{mechanics, feedback},
                  Forward<SingleCapacity>{mechanics},
                  Forward<SmallCapacity>{mechanics},
                  Forward<LargeCapacity>{mechanics},
                  Forward<HugeCapacity>{mechanics},
                  Bound<SingleCapacity>{mechanics},
                  Bound<SmallCapacity>{mechanics},
                  Bound<LargeCapacity>{mechanics},
                  Bound<HugeCapacity>{mechanics},
                  Collide{mechanics, contacts},
                  Solve{mechanics, *contacts, solution, constrained},
                  Integrate<SingleCapacity>{mechanics, *solution},
                  Integrate<SmallCapacity>{mechanics, *solution},
                  Integrate<LargeCapacity>{mechanics, *solution},
                  Integrate<HugeCapacity>{mechanics, *solution}};
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
    if (geom.type == model::GeomType::PLANE) {
      continue;
    }
    std::uint32_t b = geom.body - tree.first_body;
    model::Array3 at = dynamics->xipos[b];
    if (geom.frame != model::SameFrame::INERTIA) {
      at = model::articulated::multiply(dynamics->xmat[b], geom.pos);
      for (int k = 0; k < 3; ++k) {
        at[k] += dynamics->xpos[b][k];
      }
    }
    double reach = mechanics_->radii()[g] + geom.margin + geom.gap;
    double apart = std::hypot(at[0] - bound.center[0], at[1] - bound.center[1],
                              at[2] - bound.center[2]);
    bound.radius = std::max(bound.radius, apart + reach);
  }
}

template <typename Capacity>
auto Solve::gather(SystemWorld& world) -> void {
  constexpr std::size_t V = Capacity::dofs;
  const auto& mechanisms = world.template store_of<Mechanism>();
  const auto& states = world.template store_of<TreeState<Capacity>>();
  world.template store_of<TreeDynamics<Capacity>>().for_each(
      [&](Entity owner, const TreeDynamics<Capacity>& dynamics) {
        std::uint32_t t = mechanisms.component_of(owner).tree;
        const TreeState<Capacity>& state = states.component_of(owner);
        data_[t] = TreeData{.mass = dynamics.mass.data(),
                            .stride = V,
                            .cdof = dynamics.cdof.data(),
                            .com = &dynamics.com,
                            .qacc_smooth = dynamics.acceleration.data(),
                            .qfrc_smooth = dynamics.smooth.data(),
                            .qpos = state.qpos.data(),
                            .qvel = state.qvel.data(),
                            .warmstart = state.warmstart.data()};
      });
}

template <typename Capacity>
auto Collide::place_geoms(SystemWorld& world) -> void {
  const model::ArticulatedModel& m = mechanics_->model();
  const auto& mechanisms = world.template store_of<Mechanism>();
  world.template store_of<TreeDynamics<Capacity>>().for_each(
      [&](Entity owner, const TreeDynamics<Capacity>& dynamics) {
        const model::Tree& tree =
            mechanics_->trees()[mechanisms.component_of(owner).tree];
        for (std::uint32_t b = 0; b < tree.bodies; ++b) {
          const model::ArticulatedBody& body = m.bodies[tree.first_body + b];
          model::BodyFrame frame{
              .xpos = dynamics.xpos[b],
              .xquat = dynamics.xquat[b],
              .xmat = dynamics.xmat[b],
              .xipos = dynamics.xipos[b],
              .ximat = dynamics.ximat[b],
          };
          for (std::uint32_t g = body.first_geom;
               g < body.first_geom + body.geoms; ++g) {
            frames_[g] = model::compute_geom_frame(m.geoms[g], frame);
          }
        }
      });
}

}  // namespace simon::robotic
