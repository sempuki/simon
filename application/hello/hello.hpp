// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>

#include "base/core.hpp"
#include "core/vocabulary.hpp"
#include "engine/lifecycle.hpp"
#include "framework/archetype.hpp"
#include "framework/system.hpp"
#include "framework/world.hpp"
#include "model/kinematics.hpp"

// Balls bouncing off each other and the walls of a box, under gravity: the
// smallest complete use of the architecture.
namespace simon::hello {

using framework::Entity;
using model::Control;
using model::Kinematics;

//-- Components ----------------------------------------------------------------

// A ball's size and mass. Collisions read both.
struct Body final {
  Length radius = 0.0 * meter;
  Mass mass = 0.0 * kilogram;
};

// The force a ball's contacts with other balls put on it this step. Written
// by DetectContacts and applied by ApplyContacts.
struct Contact final {
  ForceVector force = meters_per_second_squared(0.0, 0.0, 0.0) * kilogram;
};

// A ball falls under the gravity its Control holds, which nothing changes.
struct Ball final
    : framework::Archetype<
          "ball", framework::Requires<Kinematics, Control, Body, Contact>> {};

using World =
    framework::World<Kinematics, framework::TypeList<Control, Body, Contact>,
                     framework::TypeList<Ball>>;

//-- Systems ------------------------------------------------------------------

// The box the balls bounce in, from the origin to `width` along x and to
// `height` along y. Gravity points down y.
struct Box final {
  Length width = 80.0 * meter;
  Length height = 45.0 * meter;
};

// How a ball pushes off what it touches: each contact lasts `duration`, and
// the ball leaves it `restitution` times as fast as it met it. Restitution is
// more than 0 and at most 1.
struct Springiness final {
  Time duration = 0.05 * second;
  double restitution = 1.0;
};

// Adds to `contact` the force `other` puts on a ball it overlaps.
auto append_ball_contact(const Springiness& springiness, const Body& body,
                         const Kinematics& kinematics, const Body& other_body,
                         const Kinematics& other, InOut<Contact> contact)
    -> void;

// Adds to `contact` the force each wall of `box` puts on a ball that
// overlaps it.
auto append_wall_contacts(const Springiness& springiness, const Box& box,
                          const Body& body, const Kinematics& kinematics,
                          InOut<Contact> contact) -> void;

// Moves each entity under its Control by the midpoint rule. Entities
// without a Control coast.
struct Integrate final               //
    : framework::System<Kinematics,  //
                        const Control> {
  auto operator()(auto&, Entity,           //
                  Kinematics& kinematics,  //
                  const Control* control,  //
                  Step step) const -> void {
    model::integrate_midpoint(
        control ? control->acceleration : meters_per_second_squared(0, 0, 0),
        seconds(step.dt), InOut(kinematics));
  }
};

// Each ball finds the balls it touches through a spatial query, and sums
// their forces and the walls' on it into its own Contact. It writes nothing
// else, and every ball reads the same positions and velocities, so each pair's
// two forces are equal and opposite whatever order the balls run in.
struct DetectContacts final          //
    : framework::System<Contact,     //
                        const Body,  //
                        const Kinematics> {
  using AllowComponentList = framework::TypeList<Kinematics, Body>;
  using SequenceAfterSystemList = framework::SystemList<Integrate>;

  auto prepare(auto& world) -> void {
    largest_radius = 0.0 * meter;
    store_of<Body>(world).for_each([&](Entity, const Body& body) {
      largest_radius = max(largest_radius, body.radius);
    });
  }

  auto operator()(auto& world, Entity self,  //
                  Contact& contact,          //
                  const Body* body,          //
                  const Kinematics* kinematics) const -> void {
    contact = Contact{};
    if (!body || !kinematics) return;
    world.within(
        *kinematics, body->radius + largest_radius,
        [&](Entity other, const Kinematics& other_kinematics) {
          const Body* other_body = maybe_component_of<Body>(world, other);
          if (other == self || !other_body) return;
          append_ball_contact(springiness, *body, *kinematics, *other_body,
                              other_kinematics, InOut(contact));
        });
    append_wall_contacts(springiness, box, *body, *kinematics, InOut(contact));
  }

  Springiness springiness;
  Box box;
  Length largest_radius = 0.0 * meter;
};

// Each ball takes its contacts' force over the step. Integrate has already
// moved it, so it takes the force at its new place: semi-implicit Euler
// (Hairer, Lubich and Wanner), which keeps an elastic contact's energy from
// drifting as long as the step is short against the contact.
struct ApplyContacts final              //
    : framework::System<Kinematics,     //
                        const Contact,  //
                        const Body> {
  using SequenceAfterSystemList = framework::SystemList<DetectContacts>;

  auto operator()(auto&, Entity,           //
                  Kinematics& kinematics,  //
                  const Contact* contact,  //
                  const Body* body,        //
                  Step step) const -> void {
    if (!contact || !body) return;
    kinematics.velocity += contact->force / body->mass * seconds(step.dt);
  }
};

using Schedule =
    framework::SystemList<Integrate, DetectContacts, ApplyContacts>;
using Scheduler = framework::Scheduler<World, Schedule>;

//-- Scenario ------------------------------------------------------------------

// The step to run the balls at: a tenth of a contact, short enough that an
// elastic contact gives back the energy it took.
inline constexpr Duration STEP = std::chrono::milliseconds{5};

// Everything a run depends on. The same scenario gives the same run.
struct Scenario final {
  std::uint64_t seed = 1;
  std::size_t balls = 1500;

  Box box;
  Length smallest = 0.35 * meter;
  Length largest = 0.5 * meter;
  Density density = 1000.0 * kilogram_per_cubic_meter;
  Speed fastest = 8.0 * meter_per_second;  // Each ball starts slower.

  AccelerationMagnitude gravity = 9.8 * meter_per_second_squared;
  Springiness springiness;
};

// Builds in `world` a world holding `balls` balls.
auto build_world(std::size_t balls, Out<World> world)
    -> std::expected<void, framework::Status>;

// Computes the most balls the scenario's box holds: one to a cell.
auto compute_capacity(const Scenario& scenario) -> std::size_t;

// Builds the scenario's balls on a grid of cells over its box, spread evenly
// over the cells, each at a random place in its cell with a random radius and
// velocity. Fails if the cells are fewer than the balls.
auto build_balls(const Scenario& scenario, InOut<World> world)
    -> std::expected<void, framework::Status>;

using Momentum = units::quantity<kilogram * meter_per_second, QuantityVector>;
using Energy = units::quantity<units::si::joule, double>;

// Computes the balls' total momentum.
auto compute_momentum(const World& world) -> Momentum;

// Computes the balls' total energy, kinetic and potential above y = 0.
auto compute_energy(const World& world, AccelerationMagnitude gravity)
    -> Energy;

// The hello simulation. Any driver can run it.
class Simulation final {
 public:
  explicit Simulation(Scenario scenario = {}) : scenario_{scenario} {}

  // Builds the world and the balls in it.
  auto configure() -> engine::PhaseResult;
  auto step(const Step& step) -> engine::PhaseResult;

  auto scenario() const -> const Scenario& { return scenario_; }

  // The world: empty until configured.
  auto world() const -> const World& { return world_; }

 private:
  Scenario scenario_;
  World world_;
  Scheduler scheduler_;
};

}  // namespace simon::hello
