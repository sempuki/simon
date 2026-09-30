// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

// Compares core::Store (dense arrays with an entity index) against a
// stable-slot store at several populations and churn levels. Churn destroys a
// random fraction of the population, as when drones are shot down, so the
// stable-slot store is left with holes.
//
//   bazel run -c opt //core:store_benchmark

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <print>
#include <random>
#include <vector>

#include "base/core.hpp"
#include "core/entity.hpp"
#include "core/store.hpp"

namespace simon::core {
namespace {

// The stable-slot alternative to Store, for comparison only. Each entity index
// has a fixed slot, so a lookup is one load and nothing ever moves, but
// destroyed entities leave holes that iteration must skip.
template <typename Component>
class StableSlotStore final {
 public:
  explicit StableSlotStore(std::size_t entity_capacity) : slots_(entity_capacity) {}

  Component* try_get(Entity entity) {
    Slot& slot = slots_[entity.index];
    return slot.generation == entity.generation ? &slot.value : nullptr;
  }

  void append(Entity entity, Component component) {
    Slot& slot = slots_[entity.index];
    CHECK_PRECONDITION(slot.generation == 0);
    slot = Slot{.value = std::move(component), .generation = entity.generation};
  }

  void erase(Entity entity) {
    Slot& slot = slots_[entity.index];
    CHECK_PRECONDITION(slot.generation == entity.generation);
    slot.generation = 0;
  }

  template <typename Visitor>
  void for_each(Visitor&& visit) {
    for (Slot& slot : slots_) {
      if (slot.generation != 0) {
        visit(slot.value);
      }
    }
  }

 private:
  struct Slot {
    Component value{};
    std::uint32_t generation = 0;
  };

  std::vector<Slot> slots_;
};

// About the size of Kinematics: nine doubles, 72 bytes.
struct Body {
  double position[3] = {};
  double velocity[3] = {1.0, 2.0, 3.0};
  double acceleration[3] = {};
};

template <typename Type>
void keep(const Type& value) {
  asm volatile("" : : "g"(&value) : "memory");
}

template <typename Function>
double median_nanoseconds(Function&& function, int repetitions) {
  std::vector<double> samples;
  for (int i = 0; i < repetitions; ++i) {
    auto start = std::chrono::steady_clock::now();
    function();
    auto stop = std::chrono::steady_clock::now();
    samples.push_back(std::chrono::duration<double, std::nano>(stop - start).count());
  }
  std::ranges::sort(samples);
  return samples[samples.size() / 2];
}

struct Result {
  double iterate = 0.0;  // Nanoseconds per live entity.
  double lookup = 0.0;   // Nanoseconds per random lookup.
};

struct Population {
  std::vector<Entity> live;
  std::vector<Entity> destroyed;
};

Population make_population(lib::InOut<EntityTable> table, std::size_t count, double churn,
                           lib::InOut<std::mt19937> random) {
  Population population;
  for (std::size_t i = 0; i < count; ++i) {
    population.live.push_back(table->create());
  }
  std::ranges::shuffle(population.live, *random);
  auto destroyed = static_cast<std::size_t>(churn * static_cast<double>(count));
  population.destroyed.assign(population.live.end() - destroyed, population.live.end());
  population.live.resize(count - destroyed);
  return population;
}

template <typename StoreType, typename Iterate>
Result measure(lib::InOut<StoreType> store, const Population& population, Iterate&& iterate,
               lib::InOut<std::mt19937> random) {
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
                          sum += store->try_get(entity)->position[0];
                        }
                        keep(sum);
                      },
                      repetitions) /
                  static_cast<double>(order.size());
  return result;
}

// Appends every entity in creation order, then erases the destroyed ones.
template <typename StoreType>
void populate(lib::InOut<StoreType> store, const Population& population) {
  std::vector<Entity> all = population.live;
  all.insert(all.end(), population.destroyed.begin(), population.destroyed.end());
  std::ranges::sort(all);
  for (Entity entity : all) {
    store->append(entity, Body{});
  }
  for (Entity entity : population.destroyed) {
    store->erase(entity);
  }
}

Result measure_dense(std::size_t count, double churn, lib::InOut<std::mt19937> random) {
  EntityTable table{count};
  Population population = make_population(lib::InOut(table), count, churn, random);
  Store<Body> store{count, count};
  populate(lib::InOut(store), population);
  return measure(lib::InOut(store), population,
                 [](auto& dense, auto&& visit) {
                   for (Body& body : dense.values()) {
                     visit(body);
                   }
                 },
                 random);
}

Result measure_stable(std::size_t count, double churn, lib::InOut<std::mt19937> random) {
  EntityTable table{count};
  Population population = make_population(lib::InOut(table), count, churn, random);
  StableSlotStore<Body> store{count};
  populate(lib::InOut(store), population);
  return measure(lib::InOut(store), population,
                 [](auto& stable, auto&& visit) { stable.for_each(visit); }, random);
}

}  // namespace
}  // namespace simon::core

int main() {
  using namespace simon::core;
  std::mt19937 random{42};
  std::println("{:>8} {:>6} | {:>14} {:>14} | {:>14} {:>14}", "entities", "churn",
               "dense iterate", "stable iterate", "dense lookup", "stable lookup");
  std::println("{:>8} {:>6} | {:>14} {:>14} | {:>14} {:>14}", "", "", "ns/entity",
               "ns/entity", "ns/lookup", "ns/lookup");
  for (std::size_t count : {1'000uz, 10'000uz, 100'000uz, 1'000'000uz}) {
    for (double churn : {0.0, 0.25, 0.5, 0.75}) {
      Result dense = measure_dense(count, churn, lib::InOut(random));
      Result stable = measure_stable(count, churn, lib::InOut(random));
      std::println("{:>8} {:>5.0f}% | {:>14.2f} {:>14.2f} | {:>14.2f} {:>14.2f}", count,
                   churn * 100.0, dense.iterate, stable.iterate, dense.lookup,
                   stable.lookup);
    }
  }
}
