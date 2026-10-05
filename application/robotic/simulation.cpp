// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/robotic/simulation.hpp"

#include <algorithm>
#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "format/mjcf.hpp"

namespace simon::robotic {

Mechanics::Mechanics(model::ArticulatedModel model)
    : model_{std::move(model)},
      trees_{model::find_trees(model_)},
      filter_{model_} {
  for (std::uint32_t g = 0; g < model_.geoms.size(); ++g) {
    const model::Geom& geom = model_.geoms[g];
    radii_.push_back(model::compute_bounding_radius(geom));
    if (geom.body == 0 || geom.type == model::GeomType::PLANE) {
      unbounded_.push_back(g);
    }
  }
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
  return state;
}

template <typename Capacity>
auto start_control(const Mechanics& mechanics, std::uint32_t t,
                   const Scenario& scenario) -> TreeControl<Capacity> {
  const model::Tree& tree = mechanics.trees()[t];
  TreeControl<Capacity> control;
  for (std::size_t a = 0; a < tree.actuators.size(); ++a) {
    if (tree.actuators[a] < scenario.control.size()) {
      control.control[a] = scenario.control[tree.actuators[a]];
    }
  }
  return control;
}

// Two geoms that may touch, by their bodies, contact types and affinities,
// and that only MuJoCo's general convex collider handles: an ellipsoid, or
// a cylinder with anything but a plane or a sphere.
auto find_convex_pair(const model::ArticulatedModel& m,
                      const model::BodyFilter& filter)
    -> std::optional<std::string> {
  constexpr std::array<std::string_view, 8> NAMES{
      "plane",     "hfield",   "sphere", "capsule",
      "ellipsoid", "cylinder", "box",    "mesh"};
  for (std::uint32_t c = 0; c < m.geoms.size(); ++c) {
    const model::Geom& convex = m.geoms[c];
    if (convex.type != model::GeomType::ELLIPSOID &&
        convex.type != model::GeomType::CYLINDER) {
      continue;
    }
    for (std::uint32_t g = 0; g < m.geoms.size(); ++g) {
      const model::Geom& other = m.geoms[g];
      auto [low, high] = std::minmax(convex.type, other.type);
      bool touch = (convex.contype & other.conaffinity) != 0 ||
                   (other.contype & convex.conaffinity) != 0;
      if (g != c && touch && !model::has_collider(low, high) &&
          !filter.discards(convex.body, other.body)) {
        return std::string{NAMES[static_cast<std::size_t>(low)]} + " with " +
               std::string{NAMES[static_cast<std::size_t>(high)]};
      }
    }
  }
  return std::nullopt;
}

}  // namespace

auto Simulation::configure() -> engine::PhaseResult {
  RETURN_OR_ASSIGN(model::ArticulatedModel model,
                   format::load_mjcf(scenario_.model));
  mechanics_ = std::make_unique<Mechanics>(std::move(model));
  if (std::optional<std::string> pair =
          find_convex_pair(mechanics_->model(), mechanics_->filter())) {
    return std::unexpected(lib::raise(
        format::MjcfError::UNSUPPORTED,
        "contacts of " + *pair + ", which need a general convex collider"));
  }
  scheduler_ = std::make_unique<Scheduler>(
      make_schedule(*mechanics_, scenario_.feedback, Depend(*contacts_)));
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
              .with(Touching{})
              .with(start<SmallCapacity>(*mechanics_, t, scenario_))
              .with(start_control<SmallCapacity>(*mechanics_, t, scenario_))
              .with(TreeDynamics<SmallCapacity>{})
              .build());
    } else {
      RETURN_IF_UNEXPECTED(
          world_.create<archetype::LargeTree>()
              .with(TreeBound{})
              .with(Mechanism{.tree = t})
              .with(Touching{})
              .with(start<LargeCapacity>(*mechanics_, t, scenario_))
              .with(start_control<LargeCapacity>(*mechanics_, t, scenario_))
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
