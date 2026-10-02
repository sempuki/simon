// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Compares framework::ComponentStore (dense arrays with an entity index)
// against a stable-slot store at several populations and churn levels. Churn
// destroys a random fraction of the population, as when drones are shot down,
// so the stable-slot store is left with holes.
//
//   bazel run -c opt //framework:component_store_benchmark

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <print>
#include <random>
#include <vector>

#include "base/core.hpp"
#include "framework/component_store.hpp"
#include "framework/entity.hpp"
#include "framework/vocabulary.hpp"

namespace simon::framework {
namespace {

// The stable-slot alternative to ComponentStore, for comparison only. Each
// entity index has a fixed slot, so a lookup is one load and nothing ever
// moves, but destroyed entities leave holes that iteration must skip.
template <typename ComponentType>
class StableSlotStore final {
 public:
  explicit StableSlotStore(std::size_t entity_capacity)
      : slots_(entity_capacity) {}

  auto maybe_component_of(Entity entity) -> ComponentType* {
    Slot& slot = slots_[entity.index];
    return slot.generation == entity.generation ? &slot.value : nullptr;
  }

  auto append(Entity entity, ComponentType component) -> void {
    Slot& slot = slots_[entity.index];
    CHECK_PRECONDITION(slot.generation == 0);
    slot = Slot{.value = std::move(component), .generation = entity.generation};
  }

  auto erase(Entity entity) -> void {
    Slot& slot = slots_[entity.index];
    CHECK_PRECONDITION(slot.generation == entity.generation);
    slot.generation = 0;
  }

  template <typename VisitorType>
  auto for_each(VisitorType&& visit) -> void {
    for (Slot& slot : slots_) {
      if (slot.generation != 0) {
        visit(slot.value);
      }
    }
  }

 private:
  struct Slot final {
    ComponentType value{};
    std::uint32_t generation = 0;
  };

  std::vector<Slot> slots_;
};

// About the size of Kinematics: nine doubles, 72 bytes.
struct Body final {
  std::array<double, 3> position{};
  std::array<double, 3> velocity{1.0, 2.0, 3.0};
  std::array<double, 3> acceleration{};
};

template <typename Type>
auto keep(const Type& value) -> void {
  asm volatile("" : : "g"(&value) : "memory");
}

template <typename FunctionType>
auto median_nanoseconds(FunctionType&& function, int repetitions) -> double {
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
  double iterate = 0.0;  // Nanoseconds per live entity.
  double lookup = 0.0;   // Nanoseconds per random lookup.
};

struct Population final {
  std::vector<Entity> live;
  std::vector<Entity> destroyed;
};

auto make_population(std::size_t count, double churn, InOut<EntityTable> table,
                     InOut<std::mt19937> random) -> Population {
  Population population;
  for (std::size_t i = 0; i < count; ++i) {
    population.live.push_back(table->create());
  }
  std::ranges::shuffle(population.live, *random);
  auto destroyed = static_cast<std::size_t>(churn * static_cast<double>(count));
  population.destroyed.assign(population.live.end() - destroyed,
                              population.live.end());
  population.live.resize(count - destroyed);
  return population;
}

template <typename StoreType, typename IterateType>
auto measure(const Population& population, IterateType&& iterate,
             InOut<StoreType> store, InOut<std::mt19937> random) -> Result {
  constexpr double DT = 0.01;
  int repetitions = population.live.size() < 50'000 ? 200 : 40;

  Result result;
  result.iterate = median_nanoseconds(
                       [&] {
                         iterate(*store, [&](Body& body) {
                           for (int axis = 0; axis < 3; ++axis) {
                             body.position[axis] += body.velocity[axis] * DT;
                           }
                         });
                       },
                       repetitions) /
                   static_cast<double>(population.live.size());

  std::vector<Entity> order = population.live;
  std::ranges::shuffle(order, *random);
  result.lookup = median_nanoseconds(
                      [&] {
                        double sum = 0.0;
                        for (Entity entity : order) {
                          sum += store->maybe_component_of(entity)->position[0];
                        }
                        keep(sum);
                      },
                      repetitions) /
                  static_cast<double>(order.size());
  return result;
}

// Appends every entity in creation order, then erases the destroyed ones.
template <typename StoreType>
auto populate(const Population& population, InOut<StoreType> store) -> void {
  std::vector<Entity> all = population.live;
  all.insert(all.end(), population.destroyed.begin(),
             population.destroyed.end());
  std::ranges::sort(all);
  for (Entity entity : all) {
    store->append(entity, Body{});
  }
  for (Entity entity : population.destroyed) {
    store->erase(entity);
  }
}

auto measure_dense(std::size_t count, double churn, InOut<std::mt19937> random)
    -> Result {
  EntityTable table{count};
  Population population = make_population(count, churn, InOut(table), random);
  ComponentStore<Body> store{count, count};
  populate(population, InOut(store));
  return measure(
      population,
      [](auto& dense, auto&& visit) {
        dense.for_each([&](Entity, Body& body) { visit(body); });
      },
      InOut(store), random);
}

auto measure_stable(std::size_t count, double churn, InOut<std::mt19937> random)
    -> Result {
  EntityTable table{count};
  Population population = make_population(count, churn, InOut(table), random);
  StableSlotStore<Body> store{count};
  populate(population, InOut(store));
  return measure(
      population, [](auto& stable, auto&& visit) { stable.for_each(visit); },
      InOut(store), random);
}

}  // namespace
}  // namespace simon::framework

auto main() -> int {
  using namespace simon::framework;
  std::mt19937 random{42};
  std::println("{:>8} {:>6} | {:>14} {:>14} | {:>14} {:>14}", "entities",
               "churn", "dense iterate", "stable iterate", "dense lookup",
               "stable lookup");
  std::println("{:>8} {:>6} | {:>14} {:>14} | {:>14} {:>14}", "", "",
               "ns/entity", "ns/entity", "ns/lookup", "ns/lookup");
  for (std::size_t count : {1'000uz, 10'000uz, 100'000uz, 1'000'000uz}) {
    for (double churn : {0.0, 0.25, 0.5, 0.75}) {
      Result dense = measure_dense(count, churn, simon::InOut(random));
      Result stable = measure_stable(count, churn, simon::InOut(random));
      std::println(
          "{:>8} {:>5.0f}% | {:>14.2f} {:>14.2f} | {:>14.2f} {:>14.2f}", count,
          churn * 100.0, dense.iterate, stable.iterate, dense.lookup,
          stable.lookup);
    }
  }
}
