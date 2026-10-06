// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/hello/hello.hpp"

#include <cmath>
#include <format>
#include <numbers>

#include "framework/vocabulary.hpp"
#include "model/random.hpp"

namespace simon::hello {

namespace {

// The edge of a cell on the starting grid: wide enough for the largest ball.
auto compute_cell(const Scenario& scenario) -> Length {
  return 2.0 * scenario.largest;
}

// Computes the push between two bodies of reduced mass `reduced` that overlap
// by `overlap` and close at `closing`.
auto compute_push(const Springiness& springiness, Mass reduced, Length overlap,
                  Speed closing) -> model::Force {
  model::Rate frequency = std::numbers::pi / springiness.duration;
  model::Rate decay = -std::log(springiness.restitution) / springiness.duration;
  model::Stiffness stiffness =
      reduced * (frequency * frequency + decay * decay);
  model::Damping damping = 2.0 * reduced * decay;
  return stiffness * overlap + damping * closing;
}

// Adds to `contact` the push of a wall at `wall` along `axis` (0 for x, 1 for
// y), with the inside on the side `inside` (+1 or -1) points to, on a ball of
// `mass` and `radius` at `center` moving at `velocity`.
auto push_off_wall(const Springiness& springiness, Mass mass, double wall,
                   int axis, double inside, const model::QuantityVector& center,
                   const model::QuantityVector& velocity, double radius,
                   InOut<Contact> contact) -> void {
  double overlap = radius - inside * (center.eigen()[axis] - wall);
  if (!(overlap > 0.0)) return;
  Speed closing = -inside * velocity.eigen()[axis] * meter_per_second;
  model::QuantityVector normal{axis == 0 ? inside : 0.0,
                               axis == 1 ? inside : 0.0, 0.0};
  contact->force += ForceVector{
      normal * compute_push(springiness, mass, overlap * meter, closing)};
}

}  // namespace

// Balls that overlap by x push each other apart along the line between their
// centers with a linear spring and dashpot, k x + c dx/dt, as the discrete
// element method has it (Cundall and Strack). The pair moves as one body of
// the reduced mass m = m_i m_j / (m_i + m_j), so m x'' + c x' + k x = 0, and
// the overlap is x(t) = v / w exp(-b t) sin(w t), with b = c / 2m and
// w^2 = k / m - b^2 (Schwager and Pöschel). The contact ends when the overlap
// does, at t = pi / w, with the balls parting at exp(-b pi / w) times the
// speed they met at. A contact that lasts T with restitution e therefore has
// w = pi / T and b = -ln(e) / T, so k = m (w^2 + b^2) and c = 2 m b.
//
// The dashpot pulls a little as the balls part, which is what makes the
// restitution e. Stepped ten times a contact, an elastic contact keeps its
// energy to about 1%, and one with e = 0.5 parts about 7% slower. Both balls
// compute the pair's force from the same numbers in the same order, so the two
// are exactly equal and opposite. A wall is a ball of infinite mass, whose
// reduced mass is the ball's own.
auto append_ball_contact(const Springiness& springiness, const Body& body,
                         const Kinematics& kinematics, const Body& other_body,
                         const Kinematics& other, InOut<Contact> contact)
    -> void {
  Displacement apart = kinematics.position - other.position;
  Length distance = norm(apart);
  Length overlap = body.radius + other_body.radius - distance;
  if (!(overlap > 0.0 * meter) || !(distance > 0.0 * meter)) return;

  Mass reduced = body.mass * other_body.mass / (body.mass + other_body.mass);
  auto normal = apart / distance;
  Speed closing = -dot(kinematics.velocity - other.velocity, normal);
  contact->force += ForceVector{
      normal * compute_push(springiness, reduced, overlap, closing)};
}

auto append_wall_contacts(const Springiness& springiness, const Box& box,
                          const Body& body, const Kinematics& kinematics,
                          InOut<Contact> contact) -> void {
  model::QuantityVector center = kinematics.position.numerical_value_in(meter);
  model::QuantityVector velocity =
      kinematics.velocity.numerical_value_in(meter_per_second);
  double radius = body.radius.numerical_value_in(meter);
  double width = box.width.numerical_value_in(meter);
  double height = box.height.numerical_value_in(meter);

  push_off_wall(springiness, body.mass, 0.0, 0, 1.0, center, velocity, radius,
                contact);
  push_off_wall(springiness, body.mass, width, 0, -1.0, center, velocity,
                radius, contact);
  push_off_wall(springiness, body.mass, 0.0, 1, 1.0, center, velocity, radius,
                contact);
  push_off_wall(springiness, body.mass, height, 1, -1.0, center, velocity,
                radius, contact);
}

auto build_world(std::size_t balls, Out<World> world)
    -> std::expected<void, framework::Status> {
  return World::set_up().numbered(1).holding<Ball>(balls).build(world);
}

auto compute_capacity(const Scenario& scenario) -> std::size_t {
  double cell = compute_cell(scenario).numerical_value_in(meter);
  auto columns = static_cast<std::size_t>(
      std::floor(scenario.box.width.numerical_value_in(meter) / cell));
  auto rows = static_cast<std::size_t>(
      std::floor(scenario.box.height.numerical_value_in(meter) / cell));
  return columns * rows;
}

auto build_balls(const Scenario& scenario, InOut<World> world)
    -> std::expected<void, framework::Status> {
  std::size_t cells = compute_capacity(scenario);
  if (scenario.balls > cells) {
    return std::unexpected(
        lib::raise(framework::BuildError::ENTITY_CAPACITY_EXHAUSTED,
                   std::format("The box holds at most {} balls", cells)));
  }

  double cell = compute_cell(scenario).numerical_value_in(meter);
  auto columns = static_cast<std::size_t>(
      std::floor(scenario.box.width.numerical_value_in(meter) / cell));
  double smallest = scenario.smallest.numerical_value_in(meter);
  double largest = scenario.largest.numerical_value_in(meter);
  double fastest = scenario.fastest.numerical_value_in(meter_per_second);
  Control gravity{
      .acceleration = model::meters_per_second_squared(
          0.0, -scenario.gravity.numerical_value_in(meter_per_second_squared),
          0.0)};

  model::Random random{scenario.seed};
  for (std::size_t ball = 0; ball < scenario.balls; ++ball) {
    std::size_t at = ball * cells / scenario.balls;
    double radius = random.uniform(smallest, largest);
    double x = (at % columns) * cell + random.uniform(radius, cell - radius);
    double y = (at / columns) * cell + random.uniform(radius, cell - radius);
    double speed = random.uniform(0.0, fastest);
    double heading = random.uniform(0.0, 2.0 * std::numbers::pi);
    Mass mass = scenario.density * (4.0 / 3.0 * std::numbers::pi * radius *
                                    radius * radius * meter * meter * meter);

    RETURN_IF_UNEXPECTED(
        world->create<Ball>()
            .with(Kinematics{
                .position = meters(x, y, 0.0),
                .velocity = meters_per_second(speed * std::cos(heading),
                                              speed * std::sin(heading), 0.0)})
            .with(gravity)
            .with(Body{.radius = radius * meter, .mass = mass})
            .with(Contact{})
            .build());
  }
  world->sync();
  return {};
}

auto compute_momentum(const World& world) -> Momentum {
  Momentum momentum = model::meters_per_second(0.0, 0.0, 0.0) * model::kilogram;
  world.store_of<Body>().for_each([&](Entity entity, const Body& body) {
    momentum +=
        body.mass * world.store_of<Kinematics>().component_of(entity).velocity;
  });
  return momentum;
}

auto compute_energy(const World& world, AccelerationMagnitude gravity)
    -> Energy {
  Energy energy = 0.0 * model::units::si::joule;
  world.store_of<Body>().for_each([&](Entity entity, const Body& body) {
    const Kinematics& kinematics =
        world.store_of<Kinematics>().component_of(entity);
    Length height = kinematics.position.numerical_value_in(meter).y() * meter;
    energy += 0.5 * body.mass * dot(kinematics.velocity, kinematics.velocity) +
              body.mass * gravity * height;
  });
  return energy;
}

auto Simulation::configure() -> engine::PhaseResult {
  RETURN_IF_UNEXPECTED(build_world(scenario_.balls, Out(world_)));
  RETURN_IF_UNEXPECTED(build_balls(scenario_, InOut(world_)));
  scheduler_.system<DetectContacts>().springiness = scenario_.springiness;
  scheduler_.system<DetectContacts>().box = scenario_.box;
  return engine::Flow::CONTINUE;
}

auto Simulation::step(const framework::Step& step) -> engine::PhaseResult {
  scheduler_.step(step, InOut(world_));
  return engine::Flow::CONTINUE;
}

}  // namespace simon::hello
