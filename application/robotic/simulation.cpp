// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/robotic/simulation.hpp"

#include <utility>

#include "format/mjcf.hpp"

namespace simon::robotic {

Mechanics::Mechanics(model::ArticulatedModel model)
    : model_{std::move(model)}, trees_{model::find_trees(model_)} {
  for (std::uint32_t t = 0; t < trees_.size(); ++t) {
    small_.push_back(fits<SmallCapacity>(t)
                         ? std::make_unique<model::TreeKernel<SmallCapacity>>(
                               model_, trees_[t])
                         : nullptr);
    large_.push_back(fits<LargeCapacity>(t)
                         ? std::make_unique<model::TreeKernel<LargeCapacity>>(
                               model_, trees_[t])
                         : nullptr);
  }
}

Simulation::Simulation(Scenario scenario) : scenario_{std::move(scenario)} {}

namespace {

// A tree's starting state, from the model's positions and the scenario's.
template <typename Capacity>
auto start(const Mechanics& mechanics, std::uint32_t t,
           const Scenario& scenario) -> TreeState<Capacity> {
  const model::ArticulatedModel& m = mechanics.model();
  const model::Tree& tree = mechanics.trees()[t];
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
  for (std::size_t a = 0; a < tree.actuators.size(); ++a) {
    if (tree.actuators[a] < scenario.control.size()) {
      state.control[a] = scenario.control[tree.actuators[a]];
    }
  }
  return state;
}

}  // namespace

auto Simulation::configure() -> engine::PhaseResult {
  RETURN_OR_ASSIGN(model::ArticulatedModel model,
                   format::load_mjcf(scenario_.model));
  mechanics_ = std::make_unique<Mechanics>(std::move(model));
  scheduler_ = std::make_unique<Scheduler>(make_schedule(*mechanics_));
  std::size_t small = 0;
  std::size_t large = 0;
  for (std::uint32_t t = 0; t < mechanics_->trees().size(); ++t) {
    if (mechanics_->fits<SmallCapacity>(t)) {
      ++small;
    } else if (mechanics_->fits<LargeCapacity>(t)) {
      ++large;
    } else {
      return std::unexpected(
          lib::raise(format::MjcfError::UNSUPPORTED,
                     "a tree of more than 16 bodies or 32 degrees of freedom"));
    }
  }
  RETURN_IF_UNEXPECTED(World::set_up()
                           .numbered(1)
                           .holding<archetype::SmallTree>(small)
                           .holding<archetype::LargeTree>(large)
                           .build(Out(world_)));
  auto transaction = world_.transaction();
  for (std::uint32_t t = 0; t < mechanics_->trees().size(); ++t) {
    if (mechanics_->fits<SmallCapacity>(t)) {
      RETURN_IF_UNEXPECTED(
          world_.create<archetype::SmallTree>()
              .with(TreeBound{})
              .with(Mechanism{.tree = t})
              .with(start<SmallCapacity>(*mechanics_, t, scenario_))
              .with(TreeDynamics<SmallCapacity>{})
              .build());
    } else {
      RETURN_IF_UNEXPECTED(
          world_.create<archetype::LargeTree>()
              .with(TreeBound{})
              .with(Mechanism{.tree = t})
              .with(start<LargeCapacity>(*mechanics_, t, scenario_))
              .with(TreeDynamics<LargeCapacity>{})
              .build());
    }
  }
  transaction.commit();
  world_.sync();
  return engine::Flow::CONTINUE;
}

auto Simulation::step(const framework::Step& step) -> engine::PhaseResult {
  scheduler_->step(step, InOut(world_));
  return engine::Flow::CONTINUE;
}

namespace {

template <typename Capacity, typename Field>
auto gather(const World& world, const Mechanics& mechanics, Field field,
            bool velocities, std::vector<double>& into) -> void {
  const auto& mechanisms = world.store_of<Mechanism>();
  world.store_of<TreeState<Capacity>>().for_each(
      [&](Entity owner, const TreeState<Capacity>& state) {
        const model::Tree& tree =
            mechanics.trees()[mechanisms.component_of(owner).tree];
        std::uint32_t first = velocities ? tree.first_dof : tree.first_qpos;
        std::uint32_t count = velocities ? tree.dofs : tree.qpos;
        for (std::uint32_t k = 0; k < count; ++k) {
          into[first + k] = (state.*field)[k];
        }
      });
}

}  // namespace

auto Simulation::read_qpos() const -> std::vector<double> {
  std::vector<double> qpos(mechanics_->model().qpos0.size());
  gather<SmallCapacity>(world_, *mechanics_, &TreeState<SmallCapacity>::qpos,
                        false, qpos);
  gather<LargeCapacity>(world_, *mechanics_, &TreeState<LargeCapacity>::qpos,
                        false, qpos);
  return qpos;
}

auto Simulation::read_qvel() const -> std::vector<double> {
  std::vector<double> qvel(mechanics_->model().dofs.size());
  gather<SmallCapacity>(world_, *mechanics_, &TreeState<SmallCapacity>::qvel,
                        true, qvel);
  gather<LargeCapacity>(world_, *mechanics_, &TreeState<LargeCapacity>::qvel,
                        true, qvel);
  return qvel;
}

}  // namespace simon::robotic
