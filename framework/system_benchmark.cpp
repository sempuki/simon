// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

// What a system pays to reach a sister entity-component: another component of
// the same entity that is structurally always there. One system integrates a
// 72-byte driving component using a 24-byte sister, four ways:
//
//   framework   the scheduler's loop: try_component_of on the sister's store
//               for every entity (the current design).
//   handle      a pointer resolved when the entity was built and dereferenced
//               unconditionally (the old Handle design, without its upkeep).
//   structural  the sister at the same dense position, so both arrays are
//               walked together (what an archetype-table ECS gets).
//   baseline    the driving component alone, with no sister.
//
// "aligned" attaches the sister in the same order as the driving component, so
// both stores share an order. "shuffled" attaches it in a random order, as when
// components arrive at different times, so reaching it is a random access.
// "structural" needs a shared order, so it only runs aligned. See
// churn_benchmark.cpp for layouts that keep stores in a shared order.
//
// --contend runs one thread per spare core streaming over a large buffer, to
// compete for shared cache and memory bandwidth as a busy cloud host would.
//
//   bazel run -c opt //framework:system_benchmark [-- --contend]

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <print>
#include <random>
#include <string_view>
#include <vector>

#include "base/core.hpp"
#include "framework/archetype.hpp"
#include "framework/benchmark_support.hpp"
#include "framework/system.hpp"
#include "framework/world.hpp"

namespace simon::framework {
namespace {

// The world's spatial component, unused here.
struct Point final {
  double x = 0.0;
};
inline double distance(const Point& a, const Point& b) { return a.x - b.x; }
inline Point pose(const Point& a) { return a; }
inline Coordinates coordinates(const Point& a) { return {a.x, 0.0, 0.0}; }
inline double coordinate_length(const Point&, double length) { return length; }

// About the size of Kinematics: nine doubles, 72 bytes.
struct Body final {
  double position[3] = {};
  double velocity[3] = {1.0, 2.0, 3.0};
  double acceleration[3] = {};
};

// About the size of Control: three doubles, 24 bytes.
struct Thrust final {
  double acceleration[3] = {0.1, 0.2, 0.3};
};

using BenchmarkWorld = World<Point, Body, Thrust>;
struct Craft final : Archetype<"craft", Requires<Body>, Allows<Thrust>> {};

constexpr double DT = 0.01;

inline void integrate(Body& body, const Thrust& thrust) {
  for (int axis = 0; axis < 3; ++axis) {
    body.velocity[axis] += thrust.acceleration[axis] * DT;
    body.position[axis] += body.velocity[axis] * DT;
  }
}

inline void integrate(Body& body) {
  for (int axis = 0; axis < 3; ++axis) {
    body.position[axis] += body.velocity[axis] * DT;
  }
}

struct Integrate final : System<Body, const Thrust> {
  void operator()(auto&, Entity, Body& body, const Thrust* thrust) const {
    if (thrust) {
      integrate(body, *thrust);
    }
  }
};

template <typename FunctionType>
double median_nanoseconds(FunctionType&& function, int repetitions) {
  std::vector<double> samples;
  for (int i = 0; i < repetitions; ++i) {
    auto start = std::chrono::steady_clock::now();
    function();
    auto stop = std::chrono::steady_clock::now();
    samples.push_back(
        std::chrono::duration<double, std::nano>(stop - start).count());
  }
  std::ranges::sort(samples);
  return samples[samples.size() / 2];
}

struct Result final {
  double framework = 0.0;  // Nanoseconds per entity, for each way.
  double handle = 0.0;
  double structural = 0.0;
  double baseline = 0.0;
};

Result measure(std::size_t count, bool shuffled,
               lib::InOut<std::mt19937> random) {
  BenchmarkWorld world{
      WorldConfiguration{.number = 1, .entities = count, .components = count}};
  std::vector<Entity> entities;
  for (std::size_t i = 0; i < count; ++i) {
    auto entity = world.create<Craft>().with(Body{}).build();
    CHECK_POSTCONDITION(entity.has_value());
    entities.push_back(*entity);
  }
  world.sync();
  std::vector<Entity> order = entities;
  if (shuffled) {
    std::ranges::shuffle(order, *random);
  }
  for (Entity entity : order) {
    auto attached = world.change(entity).attach(Thrust{}).build();
    CHECK_POSTCONDITION(attached.has_value());
  }
  world.sync();

  const Store<Body>& bodies = world.store_of<Body>();
  const Store<Thrust>& thrusts = world.store_of<Thrust>();
  // Handles, in the body store's order, as if each Body held one.
  std::vector<const Thrust*> handles;
  for (std::size_t i = 0; i < bodies.size(); ++i) {
    handles.push_back(&thrusts.component_of(bodies.owner(i)));
  }
  // Writable spans for the hand-written loops. Only the scheduler may ask the
  // world for them, so take them from the stores the world owns.
  auto body_data = const_cast<Store<Body>&>(bodies).values();
  auto thrust_data = thrusts.values();

  int repetitions = count < 50'000 ? 200 : 40;
  auto per_entity = [&](auto&& function) {
    return median_nanoseconds(function, repetitions) /
           static_cast<double>(count);
  };
  Scheduler<BenchmarkWorld, SystemList<Integrate>> scheduler;
  Step step{.time = TimePoint{}, .dt = std::chrono::milliseconds{10}};

  Result result;
  result.framework =
      per_entity([&] { scheduler.step(lib::InOut(world), step); });
  result.handle = per_entity([&] {
    for (std::size_t i = 0; i < body_data.size(); ++i) {
      integrate(body_data[i], *handles[i]);
    }
  });
  if (!shuffled) {
    result.structural = per_entity([&] {
      for (std::size_t i = 0; i < body_data.size(); ++i) {
        integrate(body_data[i], thrust_data[i]);
      }
    });
  }
  result.baseline = per_entity([&] {
    for (Body& body : body_data) {
      integrate(body);
    }
  });
  return result;
}

}  // namespace
}  // namespace simon::framework

int main(int argc, char** argv) {
  using namespace simon::framework;
  bool contend = argc > 1 && std::string_view{argv[1]} == "--contend";
  unsigned spare = benchmark::Contention::spare_cores();
  benchmark::Contention contention{contend ? spare : 0u};
  std::println("{}", contend ? std::format("contended by {} threads", spare)
                             : std::string{"uncontended"});

  std::mt19937 random{42};
  std::println("{:>8} {:>9} | {:>10} {:>10} {:>10} {:>10}", "entities", "order",
               "framework", "handle", "structural", "baseline");
  std::println("{:>8} {:>9} | {:>43}", "", "", "ns/entity");
  for (std::size_t count : {1'000uz, 10'000uz, 100'000uz, 1'000'000uz}) {
    for (bool shuffled : {false, true}) {
      Result result = measure(count, shuffled, lib::InOut(random));
      std::println("{:>8} {:>9} | {:>10.2f} {:>10.2f} {:>10} {:>10.2f}", count,
                   shuffled ? "shuffled" : "aligned", result.framework,
                   result.handle,
                   shuffled ? std::string{"-"}
                            : std::format("{:.2f}", result.structural),
                   result.baseline);
    }
  }
}
