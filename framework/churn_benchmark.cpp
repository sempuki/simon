// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

// Compares store layouts under churn: entities are born and die every step,
// some within a few steps and some after hundreds, and some gain or lose a
// component partway through life. An entity's archetype is the set of siblings
// it is created with; those it Requires and never loses. It may gain and lose
// only the siblings its archetype Allows (those it was created without). Every
// entity has a Body (the driving component, 72 bytes); most also have a Sibling
// (24, 256 or 1024 bytes), which one system reads beside the Body each step.
//
//   dense         one array per component with swap-erase, and no
//                 reordering (the framework's store before archetype
//                 segments). The sibling is found by try_component_of.
//   sorted        the same, and each store is sorted by entity index at the
//                 step's sync point once 1 in 8 of it is out of order. The
//                 framework does not sort, so this uses DenseStore below.
//   group         an EnTT-style owning group: every entity with both
//                 components sits at the same position at the front of both
//                 stores, kept there by swaps on each structural change. The
//                 sibling needs no lookup.
//   segmented     each store is divided into one segment per archetype, and
//                 an archetype's segments line up across its stores, so a
//                 sibling the archetype Requires sits at the entity's own local
//                 index. Segments are lists of fixed-size chunks drawn from
//                 each store's pool, so they grow and shrink without moving
//                 other segments. Siblings an entity was not created with are
//                 Allowed, and live in a sparse store with lookups.
//   generational  each store is a settled region in entity order, then a
//                 nursery in append order. Young entity-components are never
//                 reordered; ones that survive `MINIMUM_AGE` steps are merged
//                 into the settled region. Settled erases leave tombstones that
//                 the next merge removes.
//
// The competing case adds a second sibling and a second system that walks the
// Body with it, as Integrate (Kinematics, Control) and ApplyBlasts (Kinematics,
// Health) both want Kinematics. A store can belong to only one group, so there
// the group owns (Body, first sibling) and the second system looks its sibling
// up in a dense store, walking the Body in the group's order. The hybrid
// layout also keeps that store sorted into the Body's current order, once
// group swaps and its own churn have moved 1 in 8 of it out of that order.
//
// Every layout replays the same schedule of operations, generated once per
// workload, and must visit the same entities. Each step is timed in three
// parts: applying its structural changes, maintaining the layout at the sync
// point, and iterating. Warm-up steps run first, so the stores reach the
// disorder of a long run.
//
//   bazel run -c opt //framework:churn_benchmark [-- --contend[=N]]
//       [--competing]
//
// --contend runs one thread per spare core streaming over a large buffer, to
// compete for shared cache and memory bandwidth; --contend=N runs N.
// --competing runs only the competing case.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <print>
#include <random>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "base/core.hpp"
#include "framework/benchmark_support.hpp"
#include "framework/entity.hpp"
#include "framework/store.hpp"

namespace simon::framework {
namespace {

using Clock = std::chrono::steady_clock;

constexpr double DT = 0.01;
constexpr std::uint32_t ABSENT = std::numeric_limits<std::uint32_t>::max();

// About the size of Kinematics: nine doubles, 72 bytes.
struct Body final {
  double position[3] = {};
  double velocity[3] = {1.0, 2.0, 3.0};
  double acceleration[3] = {};
};

// A sibling of `BYTES` bytes, of which the system reads the first three
// doubles, as a system reads a few fields of a larger component.
template <std::size_t BYTES>
struct Sibling final {
  static_assert(BYTES >= 24 && BYTES % 8 == 0);
  std::array<double, BYTES / 8> values{0.1, 0.2, 0.3};
};

template <typename SiblingType>
void integrate(Body& body, const SiblingType& sibling) {
  for (int axis = 0; axis < 3; ++axis) {
    body.velocity[axis] += sibling.values[axis] * DT;
    body.position[axis] += body.velocity[axis] * DT;
  }
}

void integrate(Body& body) {
  for (int axis = 0; axis < 3; ++axis) {
    body.position[axis] += body.velocity[axis] * DT;
  }
}

// How many entities a step visited, with and without a sibling. Every layout
// must agree.
struct Visits final {
  std::uint64_t with_sibling = 0;
  std::uint64_t without_sibling = 0;
  std::uint64_t with_second = 0;  // For the second system, if any.
  std::uint64_t without_second = 0;
  bool operator==(const Visits&) const = default;
  Visits& operator+=(const Visits& that) {
    with_sibling += that.with_sibling;
    without_sibling += that.without_sibling;
    with_second += that.with_second;
    without_second += that.without_second;
    return *this;
  }
};

//-- The workload -------------------------------------------------------------

struct Workload final {
  std::size_t population = 0;
  double sibling_share = 0.8;    // Of births that carry a sibling.
  double second_share = 0.0;     // Of births that carry a second sibling.
  double short_share = 0.5;      // Of births that live 1 to 4 steps.
  double long_lifetime = 500.0;  // Mean steps, for the rest.
  double toggle_share = 0.001;   // Of the population, per step.
  int warm_up = 500;             // Steps before measuring.
  int measured = 200;            // Steps measured.
};

// Which siblings, as bits.
constexpr std::uint8_t FIRST = 1;
constexpr std::uint8_t SECOND = 2;

struct Operation final {
  enum class Kind : std::uint8_t { CREATE, DESTROY, ATTACH, DETACH };
  Kind kind;
  std::uint8_t siblings;  // Created with, or attached or detached.
  Entity entity;
};

// Every step's operations, in order. Step 0 creates the population.
using Schedule = std::vector<std::vector<Operation>>;

Schedule schedule_of(const Workload& workload) {
  std::mt19937_64 engine{7};
  auto unit = [&] { return static_cast<double>(engine() >> 11) * 0x1.0p-53; };
  int steps = 1 + workload.warm_up + workload.measured;

  EntityTable table{workload.population};
  std::vector<std::vector<Entity>> dying(static_cast<std::size_t>(steps));
  std::vector<std::uint8_t> siblings_of(workload.population, 0);
  std::vector<std::uint8_t> created_with(workload.population, 0);
  std::vector<Entity> live;  // For choosing entities to toggle.
  std::vector<std::uint32_t> live_position(workload.population, ABSENT);
  Schedule schedule(static_cast<std::size_t>(steps));

  auto bear = [&](int step) {
    Entity entity = table.create();
    std::uint8_t siblings = static_cast<std::uint8_t>(
        (unit() < workload.sibling_share ? FIRST : 0) |
        (unit() < workload.second_share ? SECOND : 0));
    siblings_of[entity.index] = siblings;
    created_with[entity.index] = siblings;
    live_position[entity.index] = static_cast<std::uint32_t>(live.size());
    live.push_back(entity);
    double lifetime = unit() < workload.short_share
                          ? 1.0 + std::floor(unit() * 4.0)
                          : std::max(1.0, std::round(-workload.long_lifetime *
                                                     std::log(1.0 - unit())));
    if (double death = step + lifetime; death < steps) {
      dying[static_cast<std::size_t>(death)].push_back(entity);
    }
    schedule[static_cast<std::size_t>(step)].push_back(
        Operation{Operation::Kind::CREATE, siblings, entity});
  };

  for (std::size_t i = 0; i < workload.population; ++i) {
    bear(0);
  }
  auto toggles = static_cast<std::size_t>(
      workload.toggle_share * static_cast<double>(workload.population));
  for (int step = 1; step < steps; ++step) {
    std::vector<Operation>& operations =
        schedule[static_cast<std::size_t>(step)];
    std::size_t deaths = 0;
    for (Entity entity : dying[static_cast<std::size_t>(step)]) {
      operations.push_back(Operation{Operation::Kind::DESTROY, 0, entity});
      table.destroy(entity);
      std::uint32_t position = live_position[entity.index];
      live[position] = live.back();
      live_position[live[position].index] = position;
      live.pop_back();
      live_position[entity.index] = ABSENT;
      ++deaths;
    }
    for (std::size_t i = 0; i < deaths; ++i) {
      bear(step);
    }
    for (std::size_t i = 0; i < toggles; ++i) {
      Entity entity = live[engine() % live.size()];
      std::uint8_t which =
          workload.second_share > 0.0 && engine() % 2 == 1 ? SECOND : FIRST;
      if (created_with[entity.index] & which) {
        continue;  // Required by its archetype.
      }
      bool has = (siblings_of[entity.index] & which) != 0;
      operations.push_back(
          Operation{has ? Operation::Kind::DETACH : Operation::Kind::ATTACH,
                    which, entity});
      siblings_of[entity.index] ^= which;
    }
  }
  return schedule;
}

//-- Layouts ------------------------------------------------------------------

// The framework's store before archetype segments: one dense array with
// swap-erase, plus what the sorted and hybrid layouts need, a count of appends
// and erases that break entity order and an in-place sort. The framework chose
// archetype segments instead; see documents/design.md.
template <typename ComponentType>
class DenseStore final {
 public:
  DenseStore(std::size_t capacity, std::size_t entity_capacity)
      : index_(entity_capacity) {
    owner_.reserve(capacity);
    data_.reserve(capacity);
  }

  std::size_t size() const { return data_.size(); }
  bool contains(Entity entity) const { return position_of(entity) != ABSENT; }
  ComponentType* try_component_of(Entity entity) {
    std::uint32_t position = position_of(entity);
    return position != ABSENT ? &data_[position] : nullptr;
  }
  Entity owner(std::size_t position) const { return owner_[position]; }
  std::span<ComponentType> values() { return data_; }

  void append(Entity entity, ComponentType component) {
    if (!owner_.empty() && entity.index < owner_.back().index) {
      ++disorder_;
    }
    index_[entity.index] =
        Slot{.position = static_cast<std::uint32_t>(data_.size()),
             .generation = entity.generation};
    owner_.push_back(entity);
    data_.push_back(std::move(component));
  }

  void erase(Entity entity) {
    std::uint32_t position = position_of(entity);
    std::uint32_t last = static_cast<std::uint32_t>(data_.size() - 1);
    if (position != last) {
      ++disorder_;
      data_[position] = std::move(data_[last]);
      owner_[position] = owner_[last];
      index_[owner_[position].index].position = position;
    }
    data_.pop_back();
    owner_.pop_back();
    index_[entity.index] = Slot{};
  }

  std::size_t disorder() const { return disorder_; }

  void sort_by_entity() {
    sort_by([](Entity entity) { return entity.index; });
  }

  // Sorts by `key_of(Entity)`, which must give distinct keys, moving each
  // entity-component as the comparison sort requires.
  template <typename KeyOfType>
  void sort_by(KeyOfType&& key_of) {
    std::ranges::sort(
        std::views::zip(owner_, data_), std::less<>{},
        [&](const auto& pair) { return key_of(std::get<0>(pair)); });
    for (std::size_t position = 0; position < owner_.size(); ++position) {
      index_[owner_[position].index].position =
          static_cast<std::uint32_t>(position);
    }
    disorder_ = 0;
  }

 private:
  struct Slot final {
    std::uint32_t position = ABSENT;
    std::uint32_t generation = 0;
  };

  std::uint32_t position_of(Entity entity) const {
    const Slot& slot = index_[entity.index];
    return slot.generation == entity.generation ? slot.position : ABSENT;
  }

  std::vector<Slot> index_;
  std::vector<Entity> owner_;
  std::vector<ComponentType> data_;
  std::size_t disorder_ = 0;
};

// Whether a layout sorts its dense stores is up to the layout.
template <typename ComponentType, bool SORTED>
using StoreOf = DenseStore<ComponentType>;

// The dense store, sorted by entity index at sync points or not.
template <typename SiblingType, bool SORTED>
class StoreLayout final {
 public:
  explicit StoreLayout(std::size_t capacity)
      : bodies_{capacity, capacity}, siblings_{capacity, capacity} {}

  void create(Entity entity, bool sibling) {
    bodies_.append(entity, Body{});
    if (sibling) {
      siblings_.append(entity, SiblingType{});
    }
  }
  void destroy(Entity entity) {
    bodies_.erase(entity);
    if (siblings_.contains(entity)) {
      siblings_.erase(entity);
    }
  }
  void attach(Entity entity) { siblings_.append(entity, SiblingType{}); }
  void detach(Entity entity) { siblings_.erase(entity); }

  void maintain(int) {
    if constexpr (SORTED) {
      constexpr std::size_t DISORDER_SHARE = 8;
      if (bodies_.disorder() * DISORDER_SHARE > bodies_.size()) {
        bodies_.sort_by_entity();
      }
      if (siblings_.disorder() * DISORDER_SHARE > siblings_.size()) {
        siblings_.sort_by_entity();
      }
    }
  }

  Visits iterate() {
    Visits visits;
    auto bodies = bodies_.values();
    for (std::size_t i = 0; i < bodies.size(); ++i) {
      if (const SiblingType* sibling =
              siblings_.try_component_of(bodies_.owner(i))) {
        integrate(bodies[i], *sibling);
        ++visits.with_sibling;
      } else {
        integrate(bodies[i]);
        ++visits.without_sibling;
      }
    }
    return visits;
  }

  // An entity index slot, an owner and a value per entity, in each store.
  static constexpr std::size_t bytes_per_entity() {
    return 2 * 8 + sizeof(Entity) + sizeof(Body) + sizeof(Entity) +
           sizeof(SiblingType);
  }

 private:
  StoreOf<Body, SORTED> bodies_;
  StoreOf<SiblingType, SORTED> siblings_;
};

// An owning group over (Body, Sibling). Entities with both occupy positions
// [0, group_) of both arrays, in the same order; the rest of the bodies follow.
template <typename SiblingType>
class GroupLayout final {
 public:
  explicit GroupLayout(std::size_t capacity) : position_(capacity, ABSENT) {
    owners_.reserve(capacity);
    bodies_.reserve(capacity);
    siblings_.reserve(capacity);
  }

  void create(Entity entity, bool sibling) {
    position_[entity.index] = static_cast<std::uint32_t>(bodies_.size());
    owners_.push_back(entity);
    bodies_.push_back(Body{});
    if (sibling) {
      attach(entity);
    }
  }
  void destroy(Entity entity) {
    if (position_[entity.index] < group_) {
      detach(entity);
    }
    std::uint32_t last = static_cast<std::uint32_t>(bodies_.size() - 1);
    swap_bodies(position_[entity.index], last);
    owners_.pop_back();
    bodies_.pop_back();
    position_[entity.index] = ABSENT;
  }
  // Moves the body to the group's end, and appends the sibling beside it.
  void attach(Entity entity) {
    swap_bodies(position_[entity.index], group_);
    siblings_.push_back(SiblingType{});
    ++group_;
  }
  // Moves the body and sibling to the group's last position, then shrinks the
  // group past them.
  void detach(Entity entity) {
    std::uint32_t position = position_[entity.index];
    std::uint32_t last = group_ - 1;
    if (position != last) {
      std::swap(siblings_[position], siblings_[last]);
    }
    swap_bodies(position, last);
    siblings_.pop_back();
    --group_;
  }

  void maintain(int) {}

  Visits iterate() {
    Visits visits;
    for (std::uint32_t i = 0; i < group_; ++i) {
      integrate(bodies_[i], siblings_[i]);
    }
    for (std::size_t i = group_; i < bodies_.size(); ++i) {
      integrate(bodies_[i]);
    }
    visits.with_sibling = group_;
    visits.without_sibling = bodies_.size() - group_;
    return visits;
  }

  static constexpr std::size_t bytes_per_entity() {
    return 4 + sizeof(Entity) + sizeof(Body) + sizeof(SiblingType);
  }

  // Every body, group first, for another system to walk.
  std::size_t size() const { return bodies_.size(); }
  std::uint32_t position_of(Entity entity) const {
    return position_[entity.index];
  }
  // How many bodies swaps have moved, ever.
  std::uint64_t moves() const { return moves_; }
  Entity owner(std::size_t position) const { return owners_[position]; }
  Body& body(std::size_t position) { return bodies_[position]; }

 private:
  void swap_bodies(std::uint32_t a, std::uint32_t b) {
    if (a == b) {
      return;
    }
    std::swap(bodies_[a], bodies_[b]);
    std::swap(owners_[a], owners_[b]);
    position_[owners_[a].index] = a;
    position_[owners_[b].index] = b;
    moves_ += 2;
  }

  std::vector<std::uint32_t> position_;  // By entity index.
  std::vector<Entity> owners_;
  std::vector<Body> bodies_;
  std::vector<SiblingType> siblings_;
  std::uint32_t group_ = 0;
  std::uint64_t moves_ = 0;
};

// One store: a settled region in entity order, then a nursery in append order.
template <typename ComponentType>
class GenerationalStore final {
 public:
  GenerationalStore(std::size_t capacity, std::size_t entity_capacity)
      : index_(entity_capacity, ABSENT) {
    for (Arrays* arrays : {&main_, &scratch_}) {
      arrays->owner.reserve(capacity);
      arrays->data.reserve(capacity);
      arrays->born.reserve(capacity);
      arrays->live.reserve(capacity);
    }
    candidates_.reserve(capacity);
  }

  std::size_t size() const { return main_.data.size(); }
  bool live(std::size_t position) const { return main_.live[position]; }
  Entity owner(std::size_t position) const { return main_.owner[position]; }
  ComponentType& data(std::size_t position) { return main_.data[position]; }

  ComponentType* try_component_of(Entity entity) {
    std::uint32_t position = index_[entity.index];
    return position != ABSENT && main_.owner[position] == entity
               ? &main_.data[position]
               : nullptr;
  }
  bool contains(Entity entity) { return try_component_of(entity) != nullptr; }

  void append(Entity entity, ComponentType component, int step) {
    index_[entity.index] = static_cast<std::uint32_t>(size());
    main_.owner.push_back(entity);
    main_.data.push_back(std::move(component));
    main_.born.push_back(step);
    main_.live.push_back(true);
  }

  // Swap-erases within the nursery; leaves a tombstone in the settled region.
  void erase(Entity entity) {
    std::uint32_t position = index_[entity.index];
    index_[entity.index] = ABSENT;
    if (position < settled_) {
      main_.live[position] = false;
      ++tombstones_;
      return;
    }
    std::size_t last = size() - 1;
    if (position != last) {
      main_.owner[position] = main_.owner[last];
      main_.data[position] = std::move(main_.data[last]);
      main_.born[position] = main_.born[last];
      index_[main_.owner[position].index] = position;
    }
    main_.owner.pop_back();
    main_.data.pop_back();
    main_.born.pop_back();
    main_.live.pop_back();
  }

  // Merges nursery survivors at least `minimum_age` steps old into the settled
  // region, removing tombstones, once they amount to 1 in 8 of it.
  void maintain(int step, int minimum_age) {
    constexpr std::size_t DISORDER_SHARE = 8;
    candidates_.clear();
    for (std::size_t i = settled_; i < size(); ++i) {
      if (main_.born[i] + minimum_age <= step) {
        candidates_.push_back(
            Candidate{.entity_index = main_.owner[i].index,
                      .position = static_cast<std::uint32_t>(i)});
      }
    }
    if ((candidates_.size() + tombstones_) * DISORDER_SHARE <= settled_) {
      return;
    }
    std::ranges::sort(candidates_, {}, &Candidate::entity_index);

    scratch_.clear();
    auto take = [&](std::size_t position) {
      scratch_.owner.push_back(main_.owner[position]);
      scratch_.data.push_back(std::move(main_.data[position]));
      scratch_.born.push_back(main_.born[position]);
      scratch_.live.push_back(true);
    };
    std::size_t settled = 0;
    std::size_t candidate = 0;
    while (settled < settled_ || candidate < candidates_.size()) {
      if (settled < settled_ && !main_.live[settled]) {
        ++settled;
      } else if (candidate == candidates_.size() ||
                 (settled < settled_ &&
                  main_.owner[settled].index <
                      candidates_[candidate].entity_index)) {
        take(settled++);
      } else {
        take(candidates_[candidate++].position);
      }
    }
    std::size_t promoted_end = scratch_.data.size();
    // The remaining nursery keeps its order. Promoted entries were moved out;
    // they are the ones old enough.
    for (std::size_t i = settled_; i < size(); ++i) {
      if (main_.born[i] + minimum_age > step) {
        take(i);
      }
    }
    std::swap(main_, scratch_);
    for (std::size_t i = 0; i < size(); ++i) {
      index_[main_.owner[i].index] = static_cast<std::uint32_t>(i);
    }
    settled_ = promoted_end;
    tombstones_ = 0;
  }

  // Both copies of the arrays, and an index slot.
  static constexpr std::size_t bytes_per_entity() {
    return 4 + 2 * (sizeof(Entity) + sizeof(ComponentType) + 4 + 1);
  }

 private:
  struct Arrays final {
    std::vector<Entity> owner;
    std::vector<ComponentType> data;
    std::vector<int> born;
    std::vector<bool> live;
    void clear() {
      owner.clear();
      data.clear();
      born.clear();
      live.clear();
    }
  };
  struct Candidate final {
    std::uint32_t entity_index = 0;
    std::uint32_t position = 0;
  };

  std::vector<std::uint32_t> index_;
  Arrays main_;
  Arrays scratch_;  // Merges build here, then swap.
  std::vector<Candidate> candidates_;
  std::size_t settled_ = 0;
  std::size_t tombstones_ = 0;
};

template <typename SiblingType>
class GenerationalLayout final {
 public:
  static constexpr int MINIMUM_AGE = 16;  // Steps.

  explicit GenerationalLayout(std::size_t capacity)
      : bodies_{capacity, capacity}, siblings_{capacity, capacity} {}

  void set_step(int step) { step_ = step; }

  void create(Entity entity, bool sibling) {
    bodies_.append(entity, Body{}, step_);
    if (sibling) {
      siblings_.append(entity, SiblingType{}, step_);
    }
  }
  void destroy(Entity entity) {
    bodies_.erase(entity);
    if (siblings_.contains(entity)) {
      siblings_.erase(entity);
    }
  }
  void attach(Entity entity) { siblings_.append(entity, SiblingType{}, step_); }
  void detach(Entity entity) { siblings_.erase(entity); }

  void maintain(int step) {
    bodies_.maintain(step, MINIMUM_AGE);
    siblings_.maintain(step, MINIMUM_AGE);
  }

  Visits iterate() {
    Visits visits;
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
      if (!bodies_.live(i)) {
        continue;
      }
      if (const SiblingType* sibling =
              siblings_.try_component_of(bodies_.owner(i))) {
        integrate(bodies_.data(i), *sibling);
        ++visits.with_sibling;
      } else {
        integrate(bodies_.data(i));
        ++visits.without_sibling;
      }
    }
    return visits;
  }

  static constexpr std::size_t bytes_per_entity() {
    return GenerationalStore<Body>::bytes_per_entity() +
           GenerationalStore<SiblingType>::bytes_per_entity();
  }

 private:
  GenerationalStore<Body> bodies_;
  GenerationalStore<SiblingType> siblings_;
  int step_ = 0;
};

//-- Competing layouts --------------------------------------------------------

// Walks `size` bodies and looks each one's sibling up in `siblings`.
template <typename StoreType, typename BodyOfType, typename OwnerOfType>
void walk(std::size_t size, BodyOfType&& body_of, OwnerOfType&& owner_of,
          StoreType& siblings, lib::Out<std::uint64_t> with,
          lib::Out<std::uint64_t> without) {
  for (std::size_t i = 0; i < size; ++i) {
    if (const auto* sibling = siblings.try_component_of(owner_of(i))) {
      integrate(body_of(i), *sibling);
      ++*with;
    } else {
      integrate(body_of(i));
      ++*without;
    }
  }
}

// Two siblings in framework stores, looked up from the Body store; sorted at
// sync points or not.
template <typename FirstType, typename SecondType, bool SORTED>
class CompetingStoreLayout final {
 public:
  explicit CompetingStoreLayout(std::size_t capacity)
      : bodies_{capacity, capacity},
        first_{capacity, capacity},
        second_{capacity, capacity} {}

  void create(Entity entity, std::uint8_t siblings) {
    bodies_.append(entity, Body{});
    if (siblings & FIRST) first_.append(entity, FirstType{});
    if (siblings & SECOND) second_.append(entity, SecondType{});
  }
  void destroy(Entity entity) {
    bodies_.erase(entity);
    if (first_.contains(entity)) first_.erase(entity);
    if (second_.contains(entity)) second_.erase(entity);
  }
  void attach(Entity entity, std::uint8_t which) {
    which == FIRST ? first_.append(entity, FirstType{})
                   : second_.append(entity, SecondType{});
  }
  void detach(Entity entity, std::uint8_t which) {
    which == FIRST ? first_.erase(entity) : second_.erase(entity);
  }

  void maintain(int) {
    if constexpr (SORTED) {
      sort_if_disordered(lib::InOut(bodies_));
      sort_if_disordered(lib::InOut(first_));
      sort_if_disordered(lib::InOut(second_));
    }
  }

  Visits iterate_first() { return iterate(lib::InOut(first_), true); }
  Visits iterate_second() { return iterate(lib::InOut(second_), false); }

  static constexpr std::size_t bytes_per_entity() {
    return 3 * (8 + sizeof(Entity)) + sizeof(Body) + sizeof(FirstType) +
           sizeof(SecondType);
  }

 private:
  template <typename StoreType>
  static void sort_if_disordered(lib::InOut<StoreType> store) {
    constexpr std::size_t DISORDER_SHARE = 8;
    if (store->disorder() * DISORDER_SHARE > store->size()) {
      store->sort_by_entity();
    }
  }

  template <typename StoreType>
  Visits iterate(lib::InOut<StoreType> siblings, bool first) {
    Visits visits;
    auto bodies = bodies_.values();
    walk(
        bodies.size(), [&](std::size_t i) -> Body& { return bodies[i]; },
        [&](std::size_t i) { return bodies_.owner(i); }, *siblings,
        lib::Out(first ? visits.with_sibling : visits.with_second),
        lib::Out(first ? visits.without_sibling : visits.without_second));
    return visits;
  }

  StoreOf<Body, SORTED> bodies_;
  StoreOf<FirstType, SORTED> first_;
  StoreOf<SecondType, SORTED> second_;
};

// The group owns (Body, first sibling). The second sibling is in a framework
// store, looked up while walking the bodies in the group's order. HYBRID keeps
// that store sorted by its owners' Body positions.
template <typename FirstType, typename SecondType, bool HYBRID>
class CompetingGroupLayout final {
 public:
  explicit CompetingGroupLayout(std::size_t capacity)
      : group_{capacity}, second_{capacity, capacity} {}

  void create(Entity entity, std::uint8_t siblings) {
    group_.create(entity, (siblings & FIRST) != 0);
    if (siblings & SECOND) {
      second_.append(entity, SecondType{});
      ++churn_;
    }
  }
  void destroy(Entity entity) {
    group_.destroy(entity);
    if (second_.contains(entity)) {
      second_.erase(entity);
      ++churn_;
    }
  }
  void attach(Entity entity, std::uint8_t which) {
    which == FIRST ? group_.attach(entity)
                   : second_.append(entity, SecondType{});
    churn_ += which == SECOND;
  }
  void detach(Entity entity, std::uint8_t which) {
    which == FIRST ? group_.detach(entity) : second_.erase(entity);
    churn_ += which == SECOND;
  }

  // Counts both the second store's own churn and bodies moved by the group,
  // since either puts the store out of the order the second system walks.
  void maintain(int) {
    if constexpr (HYBRID) {
      constexpr std::size_t DISORDER_SHARE = 8;
      std::uint64_t disorder = churn_ + group_.moves() - moves_at_sort_;
      if (disorder * DISORDER_SHARE > second_.size()) {
        second_.sort_by(
            [&](Entity entity) { return group_.position_of(entity); });
        churn_ = 0;
        moves_at_sort_ = group_.moves();
      }
    }
  }

  Visits iterate_first() { return group_.iterate(); }
  Visits iterate_second() {
    Visits visits;
    walk(
        group_.size(), [&](std::size_t i) -> Body& { return group_.body(i); },
        [&](std::size_t i) { return group_.owner(i); }, second_,
        lib::Out(visits.with_second), lib::Out(visits.without_second));
    return visits;
  }

  static constexpr std::size_t bytes_per_entity() {
    return GroupLayout<FirstType>::bytes_per_entity() + 8 + sizeof(Entity) +
           sizeof(SecondType);
  }

 private:
  GroupLayout<FirstType> group_;
  StoreOf<SecondType, HYBRID> second_;
  std::uint64_t churn_ = 0;  // Second-store appends and erases since a sort.
  std::uint64_t moves_at_sort_ = 0;
};

//-- Segmented layout ---------------------------------------------------------

constexpr std::size_t CHUNK = 1024;  // Entries per chunk, in every column.

// One component's column: a pool of chunks that archetype segments draw from.
// Allocated once; segments grow by taking chunks, never by moving data.
template <typename ComponentType>
class ChunkPool final {
 public:
  explicit ChunkPool(std::size_t chunks) : data_(chunks * CHUNK) {
    for (std::size_t chunk = chunks; chunk > 0; --chunk) {
      free_.push_back(static_cast<std::uint32_t>(chunk - 1));
    }
  }

  std::uint32_t take() {
    CHECK_PRECONDITION(!free_.empty());
    std::uint32_t chunk = free_.back();
    free_.pop_back();
    return chunk;
  }
  void give(std::uint32_t chunk) { free_.push_back(chunk); }
  ComponentType* chunk(std::uint32_t chunk) {
    return data_.data() + std::size_t{chunk} * CHUNK;
  }

 private:
  std::vector<ComponentType> data_;
  std::vector<std::uint32_t> free_;
};

// One archetype's segment of one column: its chunks, in local index order.
template <typename ComponentType>
class Segment final {
 public:
  explicit Segment(std::size_t chunks) { chunks_.reserve(chunks); }

  ComponentType& at(lib::InOut<ChunkPool<ComponentType>> pool,
                    std::uint32_t local) {
    return pool->chunk(chunks_[local / CHUNK])[local % CHUNK];
  }
  // Makes room for local index `count` before it is used.
  void grow(lib::InOut<ChunkPool<ComponentType>> pool, std::uint32_t count) {
    if (count % CHUNK == 0) {
      chunks_.push_back(pool->take());
    }
  }
  // Returns the last chunk once `count` entries no longer reach it.
  void shrink(lib::InOut<ChunkPool<ComponentType>> pool, std::uint32_t count) {
    if (count % CHUNK == 0) {
      pool->give(chunks_.back());
      chunks_.pop_back();
    }
  }
  std::uint32_t chunk(std::size_t i) const { return chunks_[i]; }
  std::size_t chunks() const { return chunks_.size(); }

 private:
  std::vector<std::uint32_t> chunks_;
};

// Archetypes are the sets of siblings an entity is created with, as bits.
// Single-sibling workloads use two of them; the competing workload, four.
template <typename FirstType, typename SecondType, bool COMPETING>
class SegmentedLayout final {
 public:
  static constexpr std::uint8_t ARCHETYPES = COMPETING ? 4 : 2;

  explicit SegmentedLayout(std::size_t capacity)
      : location_(capacity),
        bodies_{chunks_for(capacity)},
        owners_{chunks_for(capacity)},
        firsts_{chunks_for(capacity)},
        seconds_{COMPETING ? chunks_for(capacity) : 0},
        allowed_first_{capacity, capacity},
        allowed_second_{COMPETING ? capacity : 0, capacity} {
    for (std::uint8_t archetype = 0; archetype < ARCHETYPES; ++archetype) {
      archetypes_.push_back(Archetype{chunks_for(capacity)});
    }
  }

  // The single-sibling interface.
  void create(Entity entity, bool sibling) {
    create(entity, static_cast<std::uint8_t>(sibling ? FIRST : 0));
  }
  void attach(Entity entity) { attach(entity, FIRST); }
  void detach(Entity entity) { detach(entity, FIRST); }
  Visits iterate() { return iterate_first(); }

  void create(Entity entity, std::uint8_t siblings) {
    Archetype& archetype = archetypes_[siblings];
    std::uint32_t local = archetype.count;
    archetype.body.grow(lib::InOut(bodies_), local);
    archetype.owner.grow(lib::InOut(owners_), local);
    archetype.body.at(lib::InOut(bodies_), local) = Body{};
    archetype.owner.at(lib::InOut(owners_), local) = entity;
    if (siblings & FIRST) {
      archetype.first.grow(lib::InOut(firsts_), local);
      archetype.first.at(lib::InOut(firsts_), local) = FirstType{};
    }
    if (siblings & SECOND) {
      archetype.second.grow(lib::InOut(seconds_), local);
      archetype.second.at(lib::InOut(seconds_), local) = SecondType{};
    }
    ++archetype.count;
    location_[entity.index] = Location{.archetype = siblings, .local = local};
  }

  // Moves the segment's last entity into the gap, in every column.
  void destroy(Entity entity) {
    Location location = location_[entity.index];
    Archetype& archetype = archetypes_[location.archetype];
    std::uint32_t last = --archetype.count;
    if (location.local != last) {
      auto move = [&](auto& segment, auto& pool) {
        segment.at(lib::InOut(pool), location.local) =
            std::move(segment.at(lib::InOut(pool), last));
      };
      move(archetype.body, bodies_);
      move(archetype.owner, owners_);
      if (location.archetype & FIRST) move(archetype.first, firsts_);
      if (location.archetype & SECOND) move(archetype.second, seconds_);
      Entity moved = archetype.owner.at(lib::InOut(owners_), location.local);
      location_[moved.index].local = location.local;
    }
    archetype.body.shrink(lib::InOut(bodies_), last);
    archetype.owner.shrink(lib::InOut(owners_), last);
    if (location.archetype & FIRST) {
      archetype.first.shrink(lib::InOut(firsts_), last);
    }
    if (location.archetype & SECOND) {
      archetype.second.shrink(lib::InOut(seconds_), last);
    }
    if (allowed_first_.contains(entity)) allowed_first_.erase(entity);
    if (allowed_second_.contains(entity)) allowed_second_.erase(entity);
  }

  void attach(Entity entity, std::uint8_t which) {
    which == FIRST ? allowed_first_.append(entity, FirstType{})
                   : allowed_second_.append(entity, SecondType{});
  }
  void detach(Entity entity, std::uint8_t which) {
    which == FIRST ? allowed_first_.erase(entity)
                   : allowed_second_.erase(entity);
  }

  void maintain(int) {}

  Visits iterate_first() {
    Visits visits;
    walk<FIRST>(lib::InOut(firsts_), allowed_first_, &Archetype::first,
                lib::Out(visits.with_sibling),
                lib::Out(visits.without_sibling));
    return visits;
  }
  Visits iterate_second()
    requires COMPETING
  {
    Visits visits;
    walk<SECOND>(lib::InOut(seconds_), allowed_second_, &Archetype::second,
                 lib::Out(visits.with_second), lib::Out(visits.without_second));
    return visits;
  }

  // Columns, owners and a location per entity, plus the sparse stores for
  // Allowed siblings.
  static constexpr std::size_t bytes_per_entity() {
    return 8 + sizeof(Entity) + sizeof(Body) + sizeof(FirstType) +
           (8 + sizeof(Entity) + sizeof(FirstType)) +
           (COMPETING
                ? sizeof(SecondType) + 8 + sizeof(Entity) + sizeof(SecondType)
                : 0);
  }

 private:
  struct Location final {
    std::uint8_t archetype = 0;
    std::uint32_t local = 0;
  };

  struct Archetype final {
    explicit Archetype(std::size_t chunks)
        : body{chunks}, owner{chunks}, first{chunks}, second{chunks} {}
    Segment<Body> body;
    Segment<Entity> owner;
    Segment<FirstType> first;
    Segment<SecondType> second;
    std::uint32_t count = 0;
  };

  // Enough chunks for every entity, plus a partly filled one per archetype.
  static std::size_t chunks_for(std::size_t capacity) {
    return (capacity + CHUNK - 1) / CHUNK + ARCHETYPES;
  }

  // Walks every archetype's bodies chunk by chunk. Where the archetype
  // Requires the sibling, it sits at the same slot of the matching chunk;
  // otherwise it is Allowed, and looked up in the sparse store.
  template <std::uint8_t WHICH, typename SiblingType>
  void walk(lib::InOut<ChunkPool<SiblingType>> pool,
            Store<SiblingType>& allowed,
            Segment<SiblingType> Archetype::* sibling_segment,
            lib::Out<std::uint64_t> with, lib::Out<std::uint64_t> without) {
    for (std::uint8_t id = 0; id < ARCHETYPES; ++id) {
      Archetype& archetype = archetypes_[id];
      for (std::size_t chunk = 0; chunk < archetype.body.chunks(); ++chunk) {
        std::size_t count =
            std::min<std::size_t>(CHUNK, archetype.count - chunk * CHUNK);
        Body* bodies = bodies_.chunk(archetype.body.chunk(chunk));
        if (id & WHICH) {
          SiblingType* siblings =
              pool->chunk((archetype.*sibling_segment).chunk(chunk));
          for (std::size_t i = 0; i < count; ++i) {
            integrate(bodies[i], siblings[i]);
          }
          *with += count;
          continue;
        }
        Entity* owners = owners_.chunk(archetype.owner.chunk(chunk));
        for (std::size_t i = 0; i < count; ++i) {
          if (const SiblingType* sibling =
                  allowed.try_component_of(owners[i])) {
            integrate(bodies[i], *sibling);
            ++*with;
          } else {
            integrate(bodies[i]);
            ++*without;
          }
        }
      }
    }
  }

  std::vector<Location> location_;  // By entity index.
  ChunkPool<Body> bodies_;
  ChunkPool<Entity> owners_;
  ChunkPool<FirstType> firsts_;
  ChunkPool<SecondType> seconds_;
  Store<FirstType> allowed_first_;
  Store<SecondType> allowed_second_;
  std::vector<Archetype> archetypes_;
};

//-- Measuring ----------------------------------------------------------------

struct Result final {
  double structural = 0.0;  // Nanoseconds per entity-step, for each part.
  double maintain = 0.0;
  double iterate = 0.0;  // Both systems, in the competing case.
  double first = 0.0;    // Each system, in the competing case.
  double second = 0.0;
  double worst_step = 0.0;  // Milliseconds, the slowest measured step.
  std::size_t bytes_per_entity = 0;
  Visits visits;
};

template <typename LayoutType>
Result measure(const Workload& workload, const Schedule& schedule) {
  LayoutType layout{workload.population};
  Result result;
  result.bytes_per_entity = LayoutType::bytes_per_entity();
  constexpr bool COMPETING = requires { layout.iterate_second(); };
  double seconds[4] = {};
  for (std::size_t step = 0; step < schedule.size(); ++step) {
    if constexpr (requires { layout.set_step(0); }) {
      layout.set_step(static_cast<int>(step));
    }
    auto start = Clock::now();
    for (const Operation& operation : schedule[step]) {
      switch (operation.kind) {
        case Operation::Kind::CREATE:
          if constexpr (COMPETING) {
            layout.create(operation.entity, operation.siblings);
          } else {
            layout.create(operation.entity, (operation.siblings & FIRST) != 0);
          }
          break;
        case Operation::Kind::DESTROY:
          layout.destroy(operation.entity);
          break;
        case Operation::Kind::ATTACH:
          if constexpr (COMPETING) {
            layout.attach(operation.entity, operation.siblings);
          } else {
            layout.attach(operation.entity);
          }
          break;
        case Operation::Kind::DETACH:
          if constexpr (COMPETING) {
            layout.detach(operation.entity, operation.siblings);
          } else {
            layout.detach(operation.entity);
          }
          break;
      }
    }
    auto applied = Clock::now();
    layout.maintain(static_cast<int>(step));
    auto maintained = Clock::now();
    Visits visits;
    auto halfway = maintained;
    if constexpr (COMPETING) {
      visits += layout.iterate_first();
      halfway = Clock::now();
      visits += layout.iterate_second();
    } else {
      visits += layout.iterate();
    }
    auto iterated = Clock::now();

    if (step + workload.measured >= schedule.size()) {
      auto count = [](auto from, auto to) {
        return std::chrono::duration<double>(to - from).count();
      };
      seconds[0] += count(start, applied);
      seconds[1] += count(applied, maintained);
      seconds[2] += count(maintained, halfway);
      seconds[3] += count(halfway, iterated);
      result.worst_step =
          std::max(result.worst_step, 1e3 * count(start, iterated));
      result.visits += visits;
    }
  }
  double entity_steps = static_cast<double>(workload.population) *
                        static_cast<double>(workload.measured);
  result.structural = 1e9 * seconds[0] / entity_steps;
  result.maintain = 1e9 * seconds[1] / entity_steps;
  result.first = 1e9 * seconds[2] / entity_steps;
  result.second = 1e9 * seconds[3] / entity_steps;
  result.iterate = COMPETING ? result.first + result.second
                             : 1e9 * (seconds[2] + seconds[3]) / entity_steps;
  return result;
}

template <std::size_t BYTES>
void compare(const Workload& workload) {
  using SiblingType = Sibling<BYTES>;
  Schedule schedule = schedule_of(workload);
  std::size_t operations = 0;
  for (std::size_t step = 1; step < schedule.size(); ++step) {
    operations += schedule[step].size();
  }
  std::println(
      "\n{} entities, {}-byte sibling: {:.0f} structural operations per step",
      workload.population, BYTES,
      static_cast<double>(operations) /
          static_cast<double>(schedule.size() - 1));
  std::println("  {:<13} {:>10} {:>10} {:>10} {:>10} {:>11} {:>8}", "layout",
               "structural", "maintain", "iterate", "total", "worst step",
               "memory");
  std::println("  {:<13} {:>43} {:>11} {:>8}", "", "ns/entity-step", "ms",
               "B/entity");

  Visits expected;
  auto report = [&](std::string_view name, const Result& result) {
    if (expected == Visits{}) {
      expected = result.visits;
    }
    std::println(
        "  {:<13} {:>10.2f} {:>10.2f} {:>10.2f} {:>10.2f} {:>11.2f} {:>8}{}",
        name, result.structural, result.maintain, result.iterate,
        result.structural + result.maintain + result.iterate, result.worst_step,
        result.bytes_per_entity,
        result.visits == expected ? "" : "  VISITS DIFFER");
  };
  report("dense", measure<StoreLayout<SiblingType, false>>(workload, schedule));
  report("sorted", measure<StoreLayout<SiblingType, true>>(workload, schedule));
  report("group", measure<GroupLayout<SiblingType>>(workload, schedule));
  report("generational",
         measure<GenerationalLayout<SiblingType>>(workload, schedule));
  report("segmented", measure<SegmentedLayout<SiblingType, SiblingType, false>>(
                          workload, schedule));
}

template <std::size_t BYTES>
void compare_competing(Workload workload) {
  using FirstType = Sibling<BYTES>;
  using SecondType = Sibling<BYTES>;
  workload.second_share = 0.5;
  Schedule schedule = schedule_of(workload);
  std::println(
      "\n{} entities, two {}-byte siblings on {:.0f}% and {:.0f}% of entities, "
      "two systems",
      workload.population, BYTES, 100.0 * workload.sibling_share,
      100.0 * workload.second_share);
  std::println("  {:<13} {:>10} {:>10} {:>10} {:>10} {:>10} {:>11}", "layout",
               "structural", "maintain", "system 1", "system 2", "total",
               "worst step");
  std::println("  {:<13} {:>54} {:>11}", "", "ns/entity-step", "ms");

  Visits expected;
  auto report = [&](std::string_view name, const Result& result) {
    if (expected == Visits{}) {
      expected = result.visits;
    }
    std::println(
        "  {:<13} {:>10.2f} {:>10.2f} {:>10.2f} {:>10.2f} {:>10.2f} "
        "{:>11.2f}{}",
        name, result.structural, result.maintain, result.first, result.second,
        result.structural + result.maintain + result.iterate, result.worst_step,
        result.visits == expected ? "" : "  VISITS DIFFER");
  };
  report("dense", measure<CompetingStoreLayout<FirstType, SecondType, false>>(
                      workload, schedule));
  report("sorted", measure<CompetingStoreLayout<FirstType, SecondType, true>>(
                       workload, schedule));
  report("group", measure<CompetingGroupLayout<FirstType, SecondType, false>>(
                      workload, schedule));
  report("hybrid", measure<CompetingGroupLayout<FirstType, SecondType, true>>(
                       workload, schedule));
  report("segmented", measure<SegmentedLayout<FirstType, SecondType, true>>(
                          workload, schedule));
}

}  // namespace
}  // namespace simon::framework

int main(int argc, char** argv) {
  using namespace simon::framework;
  unsigned threads = 0;
  bool competing_only = false;
  for (int i = 1; i < argc; ++i) {
    std::string_view argument{argv[i]};
    if (auto asked = benchmark::Contention::threads_from(argument)) {
      threads = *asked;
    } else if (argument == "--competing") {
      competing_only = true;
    } else {
      std::println(stderr, "unknown argument: {}", argument);
      return 1;
    }
  }
  benchmark::Contention contention{threads};
  std::println("{}", benchmark::Contention::describe(threads));

  for (std::size_t population : {100'000uz, 1'000'000uz}) {
    Workload workload{.population = population};
    compare_competing<24>(workload);
    compare_competing<256>(workload);
    if (competing_only) {
      continue;
    }
    compare<24>(workload);
    compare<256>(workload);
    // A million 1 KB siblings, twice over for the generational layout, is more
    // memory than a benchmark should take.
    if (population <= 100'000) {
      compare<1024>(workload);
    }
  }
}
