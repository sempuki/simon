// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/robotic/simulation.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "core/argument.hpp"
#include "format/mjcf.hpp"

namespace simon::robotic {

namespace {

// Whether MuJoCo takes dampers implicitly: if any dof is damped or
// actuated, every dof's (mj_EulerSkip).
auto is_implicit(const articulated::Scene& m) -> bool {
  return !m.actuators.empty() ||
         std::ranges::any_of(
             m.dofs, [](const articulated::Dof& d) { return d.damping > 0; });
}

// The inverse inertia each body and dof of a tree sees at its rest
// positions, averaged over translation and rotation, and the sum of its
// mass matrix's diagonal (mj_setConst).
template <typename Capacity>
auto weigh(const articulated::Scene& m, const articulated::Tree& tree,
           const articulated::TreeKernel<Capacity>& kernel,
           InOut<std::vector<double>> body_weight,
           InOut<std::vector<double>> dof_weight,
           InOut<std::vector<double>> tendon_weight, InOut<double> inertia)
    -> void {
  constexpr std::size_t V = Capacity::dofs;
  articulated::TreeState<Capacity> state;
  for (std::uint32_t q = 0; q < tree.qpos; ++q) {
    state.qpos[q] = m.qpos0[tree.first_qpos + q];
  }
  auto dynamics = std::make_unique<articulated::TreeDynamics<Capacity>>();
  kernel.forward(state, articulated::TreeControl<Capacity>{}, Out(*dynamics));
  std::uint32_t n = tree.dofs;
  if (n == 0) {
    return;  // A tree that cannot move weighs nothing.
  }
  for (std::uint32_t i = 0; i < n; ++i) {
    *inertia += dynamics->mass[i * V + i];
  }
  // x M⁻¹ x for a row of the tree's dofs.
  auto inverse = [&](std::span<const double> row) {
    std::array<double, V> x{};
    std::ranges::copy(row, x.begin());
    kernel.solve(dynamics->factor, dynamics->inverse_diagonal, x);
    double sum = 0.0;
    for (std::uint32_t i = 0; i < n; ++i) {
      sum += row[i] * x[i];
    }
    return sum;
  };
  std::vector<double> translation(3 * std::size_t{n});
  std::vector<double> rotation(3 * std::size_t{n});
  std::span<const Vector6> cdof{dynamics->cdof.data(), n};
  for (std::uint32_t b = 0; b < tree.bodies; ++b) {
    std::uint32_t body = tree.first_body + b;
    articulated::compute_point_jacobian(m, tree, cdof, dynamics->com, body,
                                        dynamics->xipos[b], translation,
                                        rotation);
    double moved = 0.0;
    double turned = 0.0;
    for (std::uint32_t k = 0; k < 3; ++k) {
      moved += inverse({&translation[k * n], n});
      turned += inverse({&rotation[k * n], n});
    }
    (*body_weight)[2 * body] = moved / 3;
    (*body_weight)[2 * body + 1] = turned / 3;
  }
  // A tendon's share on this tree; trees' inertia is block diagonal.
  for (std::uint32_t k = 0; k < m.tendons.size(); ++k) {
    const articulated::Tendon& tendon = m.tendons[k];
    std::vector<double> row(n, 0.0);
    bool touches = false;
    for (std::size_t i = 0; i < tendon.joints.size(); ++i) {
      std::uint32_t d = m.joints[tendon.joints[i]].dof;
      if (d >= tree.first_dof && d < tree.first_dof + n) {
        row[d - tree.first_dof] += tendon.coefficients[i];
        touches = true;
      }
    }
    if (touches) {
      (*tendon_weight)[k] += inverse(row);
    }
  }
  std::vector<double> unit(n);
  auto diagonal = [&](std::uint32_t c) {
    std::ranges::fill(unit, 0.0);
    unit[c] = 1;
    return inverse(unit);
  };
  for (std::uint32_t j = tree.first_joint; j < tree.first_joint + tree.joints;
       ++j) {
    const articulated::Joint& joint = m.joints[j];
    std::uint32_t d = joint.dof;
    std::uint32_t c = d - tree.first_dof;
    switch (joint.type) {
      case articulated::JointType::FREE: {
        double moved = (diagonal(c) + diagonal(c + 1) + diagonal(c + 2)) / 3;
        double turned =
            (diagonal(c + 3) + diagonal(c + 4) + diagonal(c + 5)) / 3;
        for (std::uint32_t k = 0; k < 3; ++k) {
          (*dof_weight)[d + k] = moved;
          (*dof_weight)[d + 3 + k] = turned;
        }
        break;
      }
      case articulated::JointType::BALL: {
        double turned = (diagonal(c) + diagonal(c + 1) + diagonal(c + 2)) / 3;
        for (std::uint32_t k = 0; k < 3; ++k) {
          (*dof_weight)[d + k] = turned;
        }
        break;
      }
      default:
        (*dof_weight)[d] = diagonal(c);
        break;
    }
  }
}

}  // namespace

Mechanics::Mechanics(articulated::Scene model)
    : model_{std::move(model)},
      trees_{articulated::find_trees(model_)},
      filter_{model_} {
  for (std::uint32_t g = 0; g < model_.geoms.size(); ++g) {
    const articulated::Geometry& geom = model_.geoms[g];
    radii_.push_back(articulated::compute_bounding_radius(geom));
    if (geom.body == 0 || geom.type == articulated::GeometryType::PLANE) {
      unbounded_.push_back(g);
    }
  }
  bool implicit = is_implicit(model_);
  body_weight_.assign(2 * model_.bodies.size(), 0.0);
  dof_weight_.assign(model_.dofs.size(), 0.0);
  tendon_weight_.assign(model_.tendons.size(), 0.0);
  double inertia = 0.0;
  for (std::uint32_t t = 0; t < trees_.size(); ++t) {
    single_.push_back(
        fits<SingleCapacity>(t)
            ? std::make_unique<articulated::TreeKernel<SingleCapacity>>(
                  model_, trees_[t], implicit)
            : nullptr);
    small_.push_back(
        fits<SmallCapacity>(t)
            ? std::make_unique<articulated::TreeKernel<SmallCapacity>>(
                  model_, trees_[t], implicit)
            : nullptr);
    large_.push_back(
        fits<LargeCapacity>(t)
            ? std::make_unique<articulated::TreeKernel<LargeCapacity>>(
                  model_, trees_[t], implicit)
            : nullptr);
    huge_.push_back(
        fits<HugeCapacity>(t)
            ? std::make_unique<articulated::TreeKernel<HugeCapacity>>(
                  model_, trees_[t], implicit)
            : nullptr);
    if (single_.back()) {
      weigh(model_, trees_[t], *single_.back(), InOut(body_weight_),
            InOut(dof_weight_), InOut(tendon_weight_), InOut(inertia));
    } else if (small_.back()) {
      weigh(model_, trees_[t], *small_.back(), InOut(body_weight_),
            InOut(dof_weight_), InOut(tendon_weight_), InOut(inertia));
    } else if (large_.back()) {
      weigh(model_, trees_[t], *large_.back(), InOut(body_weight_),
            InOut(dof_weight_), InOut(tendon_weight_), InOut(inertia));
    } else if (huge_.back()) {
      weigh(model_, trees_[t], *huge_.back(), InOut(body_weight_),
            InOut(dof_weight_), InOut(tendon_weight_), InOut(inertia));
    }
  }
  mean_inertia_ = model_.dofs.empty()
                      ? 1.0
                      : inertia / static_cast<double>(model_.dofs.size());
}

Simulation::Simulation(Scenario scenario) : scenario_{std::move(scenario)} {}

namespace {

// A tree's starting state, from the model's positions and the scenario's.
template <typename Capacity>
auto start(const Mechanics& mechanics, std::uint32_t t,
           const Scenario& scenario) -> TreeState<Capacity> {
  const articulated::Scene& m = mechanics.model();
  const articulated::Tree& tree = mechanics.trees()[t];
  TreeState<Capacity> state;
  for (std::uint32_t q = 0; q < tree.qpos; ++q) {
    state.qpos[q] = m.qpos0[tree.first_qpos + q];
  }
  for (auto [address, value] : scenario.qpos) {
    if (address >= tree.first_qpos && address < tree.first_qpos + tree.qpos) {
      state.qpos[address - tree.first_qpos] = value;
    }
  }
  for (auto [address, value] : scenario.qvel) {
    if (address >= tree.first_dof && address < tree.first_dof + tree.dofs) {
      state.qvel[address - tree.first_dof] = value;
    }
  }
  return state;
}

template <typename Capacity>
auto start_control(const Mechanics& mechanics, std::uint32_t t,
                   const Scenario& scenario) -> TreeControl<Capacity> {
  const articulated::Tree& tree = mechanics.trees()[t];
  TreeControl<Capacity> control;
  for (std::size_t a = 0; a < tree.actuators.size(); ++a) {
    if (tree.actuators[a] < scenario.control.size()) {
      control.control[a] = scenario.control[tree.actuators[a]];
    }
  }
  return control;
}

}  // namespace

auto Simulation::configure() -> engine::PhaseResult {
  RETURN_OR_ASSIGN_TO(articulated::Scene model,
                      format::load_mjcf(scenario_.model));
  if (scenario_.solver) {
    model.physics.solver = *scenario_.solver;
  }
  if (scenario_.cone) {
    model.physics.cone = *scenario_.cone;
  }
  if (scenario_.integrator) {
    model.physics.integrator = *scenario_.integrator;
  }
  if (model.physics.integrator != articulated::Physics::Integrator::EULER &&
      model.physics.integrator !=
          articulated::Physics::Integrator::IMPLICIT_FAST) {
    return std::unexpected(
        lib::raise(format::FormatError::UNSUPPORTED,
                   "integrators other than Euler and implicitfast"));
  }
  if (scenario_.constrained &&
      model.physics.solver == articulated::Physics::Solver::CG) {
    return std::unexpected(
        lib::raise(format::FormatError::UNSUPPORTED, "the CG solver"));
  }
  mechanics_ = std::make_unique<Mechanics>(std::move(model));
  scheduler_ = std::make_unique<Scheduler>(
      make_schedule(*mechanics_, scenario_.feedback, Depend(*contacts_),
                    Depend(*solution_), scenario_.constrained));
  std::size_t single = 0;
  std::size_t small = 0;
  std::size_t large = 0;
  std::size_t huge = 0;
  for (std::uint32_t t = 0; t < mechanics_->trees().size(); ++t) {
    if (mechanics_->fits<SingleCapacity>(t)) {
      ++single;
    } else if (mechanics_->fits<SmallCapacity>(t)) {
      ++small;
    } else if (mechanics_->fits<LargeCapacity>(t)) {
      ++large;
    } else if (mechanics_->fits<HugeCapacity>(t)) {
      ++huge;
    } else {
      return std::unexpected(
          lib::raise(format::FormatError::UNSUPPORTED,
                     "a tree of more than 32 bodies or 64 degrees of freedom"));
    }
  }
  RETURN_IF_UNEXPECTED(World::set_up()
                           .numbered(1)
                           .holding<archetype::SingleBody>(single)
                           .holding<archetype::SmallTree>(small)
                           .holding<archetype::LargeTree>(large)
                           .holding<archetype::HugeTree>(huge)
                           .build(Out(world_)));
  auto transaction = world_.transaction();
  // Each tree in the smallest archetype it fits.
  auto create = [&]<typename ArchetypeType, typename Capacity>(
                    std::uint32_t t) -> std::expected<void, lib::Status> {
    RETURN_IF_UNEXPECTED(
        world_.create<ArchetypeType>()
            .with(TreeBound{})
            .with(Mechanism{.tree = t})
            .with(Touching{})
            .with(Island{})
            .with(start<Capacity>(*mechanics_, t, scenario_))
            .with(start_control<Capacity>(*mechanics_, t, scenario_))
            .with(TreeDynamics<Capacity>{})
            .build());
    return {};
  };
  for (std::uint32_t t = 0; t < mechanics_->trees().size(); ++t) {
    if (mechanics_->fits<SingleCapacity>(t)) {
      RETURN_IF_UNEXPECTED(
          (create.operator()<archetype::SingleBody, SingleCapacity>(t)));
    } else if (mechanics_->fits<SmallCapacity>(t)) {
      RETURN_IF_UNEXPECTED(
          (create.operator()<archetype::SmallTree, SmallCapacity>(t)));
    } else if (mechanics_->fits<LargeCapacity>(t)) {
      RETURN_IF_UNEXPECTED(
          (create.operator()<archetype::LargeTree, LargeCapacity>(t)));
    } else {
      RETURN_IF_UNEXPECTED(
          (create.operator()<archetype::HugeTree, HugeCapacity>(t)));
    }
  }
  transaction.commit();
  world_.sync();
  return engine::Flow::CONTINUE;
}

auto Simulation::step(const Step& step) -> engine::PhaseResult {
  // As many of the model's steps as the driver's step holds, the rest
  // carried to the next.
  auto h = std::chrono::nanoseconds{
      std::llround(mechanics_->model().physics.timestep * 1e9)};
  pending_ += step.dt;
  TimePoint time = step.time;
  while (pending_ >= h) {
    scheduler_->step(Step{.time = time, .dt = h}, InOut(world_));
    pending_ -= h;
    time += h;
  }
  return engine::Flow::CONTINUE;
}

namespace {

template <typename Capacity, typename Field>
auto gather(const World& world, const Mechanics& mechanics, Field field,
            bool velocities, InOut<std::vector<double>> into) -> void {
  const auto& mechanisms = world.store_of<Mechanism>();
  world.store_of<TreeState<Capacity>>().for_each(
      [&](Entity owner, const TreeState<Capacity>& state) {
        const articulated::Tree& tree =
            mechanics.trees()[mechanisms.component_of(owner).tree];
        std::uint32_t first = velocities ? tree.first_dof : tree.first_qpos;
        std::uint32_t count = velocities ? tree.dofs : tree.qpos;
        for (std::uint32_t k = 0; k < count; ++k) {
          (*into)[first + k] = (state.*field)[k];
        }
      });
}

template <typename Capacity>
auto place(const World& world, const Mechanics& mechanics,
           InOut<std::vector<articulated::GeometryFrame>> frames) -> void {
  const articulated::Scene& m = mechanics.model();
  const auto& mechanisms = world.store_of<Mechanism>();
  world.store_of<TreeDynamics<Capacity>>().for_each(
      [&](Entity owner, const TreeDynamics<Capacity>& dynamics) {
        const articulated::Tree& tree =
            mechanics.trees()[mechanisms.component_of(owner).tree];
        for (std::uint32_t b = 0; b < tree.bodies; ++b) {
          const articulated::Body& body = m.bodies[tree.first_body + b];
          articulated::BodyFrame frame{.xpos = dynamics.xpos[b],
                                       .xquat = dynamics.xquat[b],
                                       .xmat = dynamics.xmat[b],
                                       .xipos = dynamics.xipos[b],
                                       .ximat = dynamics.ximat[b]};
          for (std::uint32_t g = body.first_geom;
               g < body.first_geom + body.geoms; ++g) {
            (*frames)[g] = articulated::compute_geom_frame(m.geoms[g], frame);
          }
        }
      });
}

}  // namespace

auto Simulation::read_geom_frames() const
    -> std::vector<articulated::GeometryFrame> {
  const articulated::Scene& m = mechanics_->model();
  std::vector<articulated::GeometryFrame> frames(m.geoms.size());
  const articulated::Body& ground = m.bodies[0];
  for (std::uint32_t g = ground.first_geom;
       g < ground.first_geom + ground.geoms; ++g) {
    frames[g] =
        articulated::compute_geom_frame(m.geoms[g], articulated::BodyFrame{});
  }
  place<SingleCapacity>(world_, *mechanics_, InOut(frames));
  place<SmallCapacity>(world_, *mechanics_, InOut(frames));
  place<LargeCapacity>(world_, *mechanics_, InOut(frames));
  place<HugeCapacity>(world_, *mechanics_, InOut(frames));
  return frames;
}

auto Simulation::read_qpos() const -> std::vector<double> {
  std::vector<double> qpos(mechanics_->model().qpos0.size());
  gather<SingleCapacity>(world_, *mechanics_, &TreeState<SingleCapacity>::qpos,
                         false, InOut(qpos));
  gather<SmallCapacity>(world_, *mechanics_, &TreeState<SmallCapacity>::qpos,
                        false, InOut(qpos));
  gather<LargeCapacity>(world_, *mechanics_, &TreeState<LargeCapacity>::qpos,
                        false, InOut(qpos));
  gather<HugeCapacity>(world_, *mechanics_, &TreeState<HugeCapacity>::qpos,
                       false, InOut(qpos));
  return qpos;
}

auto Simulation::read_qvel() const -> std::vector<double> {
  std::vector<double> qvel(mechanics_->model().dofs.size());
  gather<SingleCapacity>(world_, *mechanics_, &TreeState<SingleCapacity>::qvel,
                         true, InOut(qvel));
  gather<SmallCapacity>(world_, *mechanics_, &TreeState<SmallCapacity>::qvel,
                        true, InOut(qvel));
  gather<LargeCapacity>(world_, *mechanics_, &TreeState<LargeCapacity>::qvel,
                        true, InOut(qvel));
  gather<HugeCapacity>(world_, *mechanics_, &TreeState<HugeCapacity>::qvel,
                       true, InOut(qvel));
  return qvel;
}

}  // namespace simon::robotic
