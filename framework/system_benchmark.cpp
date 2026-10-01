// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

// What a system pays to reach a sibling: another component of the same entity.
// One system integrates a 72-byte driving component using a 24-byte sibling,
// five ways:
//
//   allowed     the scheduler's loop, where the archetype only allows the
//               sibling: it was attached after creation and is found by
//               try_component_of.
//   required    the scheduler's loop, where the archetype requires the
//               sibling: it sits in the archetype's segment of its store, at
//               the entity's own local index.
//   handle      plain arrays, and a pointer to the sibling resolved when the
//               entity was built (the older simulator's handle, without its
//               upkeep).
//   structural  plain arrays walked together.
//   baseline    the driving component alone, with no sibling.
//
// "aligned" attaches allowed siblings, and resolves handles, in the driving
// component's order. "shuffled" uses a random order, as when components arrive
// at different times, so reaching the sibling is a random access. Required
// siblings and structural arrays share an order by construction, so they only
// run aligned.
//
// --contend runs one thread per spare core streaming over a large buffer, to
// compete for shared cache and memory bandwidth as a busy cloud host would;
// --contend=N runs N.
//
//   bazel run -c opt //framework:system_benchmark [-- --contend[=N]]

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <numeric>
#include <print>
#include <random>
#include <string>
#include <string_view>
#include <vector>

#include "base/core.hpp"
#include "framework/archetype.hpp"
#include "framework/benchmark_support.hpp"
#include "framework/system.hpp"
#include "framework/world.hpp"

namespace simon::framework {
namespace {

// The world's spatial component, unused here beyond satisfying Spatial.
struct Point final {
  double x = 0.0;
};
[[maybe_unused]] double distance(const Point& a, const Point& b) {
  return a.x - b.x;
}
[[maybe_unused]] Point pose(const Point& a) { return a; }
[[maybe_unused]] Coordinates coordinates(const Point& a) {
  return {a.x, 0.0, 0.0};
}
[[maybe_unused]] double coordinate_length(const Point&, double length) {
  return length;
}

// About the size of Kinematics: nine doubles, 72 bytes.
struct Body final {
  std::array<double, 3> position{};
  std::array<double, 3> velocity{1.0, 2.0, 3.0};
  std::array<double, 3> acceleration{};
};

// About the size of Control: three doubles, 24 bytes.
struct Thrust final {
  std::array<double, 3> acceleration{0.1, 0.2, 0.3};
};

struct Craft final : Archetype<"craft", Requires<Body>, Allows<Thrust>> {};
struct Rocket final : Archetype<"rocket", Requires<Body, Thrust>> {};
using BenchmarkWorld =
    World<Point, TypeList<Body, Thrust>, TypeList<Craft, Rocket>>;

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
  double allowed = 0.0;  // Nanoseconds per entity, for each way.
  double required = 0.0;
  double handle = 0.0;
  double structural = 0.0;
  double baseline = 0.0;
};

Result measure(std::size_t count, bool shuffled,
               lib::InOut<std::mt19937> random) {
  std::vector<std::size_t> order(count);
  std::iota(order.begin(), order.end(), 0);
  if (shuffled) {
    std::ranges::shuffle(order, *random);
  }

  // The framework: one world whose sibling is allowed, attached in `order`,
  // and one whose sibling is required.
  BenchmarkWorld allowed;
  BenchmarkWorld required;
  auto allowed_built =
      BenchmarkWorld::set_up().holding<Craft>(count).build(lib::Out(allowed));
  auto required_built =
      BenchmarkWorld::set_up().holding<Rocket>(count).build(lib::Out(required));
  CHECK_POSTCONDITION(allowed_built.has_value() && required_built.has_value());
  std::vector<Entity> crafts;
  for (std::size_t i = 0; i < count; ++i) {
    auto craft = allowed.create<Craft>().with(Body{}).build();
    auto rocket = required.create<Rocket>().with(Body{}).with(Thrust{}).build();
    CHECK_POSTCONDITION(craft.has_value() && rocket.has_value());
    crafts.push_back(*craft);
  }
  allowed.sync();
  required.sync();
  for (std::size_t i : order) {
    auto attached = allowed.change(crafts[i]).attach(Thrust{}).build();
    CHECK_POSTCONDITION(attached.has_value());
  }
  allowed.sync();

  // Plain arrays, with handles resolved in `order`.
  std::vector<Body> bodies(count);
  std::vector<Thrust> thrusts(count);
  std::vector<const Thrust*> handles(count);
  for (std::size_t i = 0; i < count; ++i) {
    handles[i] = &thrusts[order[i]];
  }

  int repetitions = count < 50'000 ? 200 : 40;
  auto per_entity = [&](auto&& function) {
    return median_nanoseconds(function, repetitions) /
           static_cast<double>(count);
  };
  Scheduler<BenchmarkWorld, SystemList<Integrate>> allowed_scheduler;
  Scheduler<BenchmarkWorld, SystemList<Integrate>> required_scheduler;
  Step step{.time = TimePoint{}, .dt = std::chrono::milliseconds{10}};

  Result result;
  result.allowed =
      per_entity([&] { allowed_scheduler.step(step, lib::InOut(allowed)); });
  result.handle = per_entity([&] {
    for (std::size_t i = 0; i < count; ++i) {
      integrate(bodies[i], *handles[i]);
    }
  });
  if (!shuffled) {
    result.required = per_entity(
        [&] { required_scheduler.step(step, lib::InOut(required)); });
    result.structural = per_entity([&] {
      for (std::size_t i = 0; i < count; ++i) {
        integrate(bodies[i], thrusts[i]);
      }
    });
  }
  result.baseline = per_entity([&] {
    for (Body& body : bodies) {
      integrate(body);
    }
  });
  return result;
}

}  // namespace
}  // namespace simon::framework

int main(int argc, char** argv) {
  using namespace simon::framework;
  unsigned threads = 0;
  for (int i = 1; i < argc; ++i) {
    std::string_view argument{argv[i]};
    if (auto asked = benchmark::Contention::threads_from(argument)) {
      threads = *asked;
    } else {
      std::println(stderr, "unknown argument: {}", argument);
      return 1;
    }
  }
  benchmark::Contention contention{threads};
  std::println("{}", benchmark::Contention::describe(threads));

  std::mt19937 random{42};
  std::println("{:>8} {:>9} | {:>10} {:>10} {:>10} {:>10} {:>10}", "entities",
               "order", "allowed", "required", "handle", "structural",
               "baseline");
  std::println("{:>8} {:>9} | {:>54}", "", "", "ns/entity");
  auto aligned_only = [](bool shuffled, double value) {
    return shuffled ? std::string{"-"} : std::format("{:.2f}", value);
  };
  for (std::size_t count : {1'000uz, 10'000uz, 100'000uz, 1'000'000uz}) {
    for (bool shuffled : {false, true}) {
      Result result = measure(count, shuffled, lib::InOut(random));
      std::println("{:>8} {:>9} | {:>10.2f} {:>10} {:>10.2f} {:>10} {:>10.2f}",
                   count, shuffled ? "shuffled" : "aligned", result.allowed,
                   aligned_only(shuffled, result.required), result.handle,
                   aligned_only(shuffled, result.structural), result.baseline);
    }
  }
}
