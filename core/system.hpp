// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <concepts>
#include <cstddef>
#include <format>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "base/core.hpp"
#include "core/entity.hpp"
#include "core/name.hpp"
#include "core/step.hpp"
#include "core/type_list.hpp"
#include "core/world.hpp"

namespace simon::core {

// A schedule: systems in execution order. Schedules nest and flatten. A
// schedule is a type (for ordering checks) and a value (holding each system,
// so systems built from lambdas with captures can be scheduled).
template <typename... SystemTypes>
struct Systems {
  std::tuple<SystemTypes...> systems;

  Systems()
    requires(std::is_default_constructible_v<SystemTypes> && ...)
  = default;
  explicit Systems(SystemTypes... given) : systems{std::move(given)...} {}
};

template <typename... SystemTypes>
Systems(SystemTypes...) -> Systems<SystemTypes...>;

// The stores a system may read by entity, for following references to other
// entities. Lookups are always read-only.
template <typename... Components>
struct Stores {};

// A system's structure. The first component drives the loop and comes in by
// reference; every following component is optional, belongs to the same
// entity, and comes in by pointer (null when absent). Declare a component
// `const` to read it without writing it.
//
//   struct Integrate : System<Kinematics, const Control> {
//     void operator()(Entity, Kinematics&, const Control*, auto& context);
//   };
template <typename Drive, typename... Optional>
struct System {
  using DriveType = Drive;
  using OptionalTypes = TypeList<Optional...>;
  using Lookups = Stores<>;
  using After = Systems<>;
};

// A system made from any callable, such as a lambda. The callable keeps its
// own state (its captures) between steps.
//
//   auto integrate = system<Kinematics, const Control>(
//       [](Entity, Kinematics&, const Control*, auto& context) { ... });
//   auto guide = system<const Interceptor, Control>(
//       Stores<Kinematics>{},  // Lookups.
//       [](Entity, const Interceptor&, Control*, auto& context) { ... });
template <typename Callable, typename LookupStores, typename Drive,
          typename... Optional>
class CallableSystem final : public System<Drive, Optional...> {
 public:
  using Lookups = LookupStores;

  explicit CallableSystem(Callable callable) : callable_{std::move(callable)} {}

  template <typename... Arguments>
    requires std::invocable<Callable&, Arguments...>
  void operator()(Arguments&&... arguments) {
    std::invoke(callable_, std::forward<Arguments>(arguments)...);
  }

  Callable& callable() { return callable_; }

 private:
  Callable callable_;
};

template <typename Drive, typename... Optional, typename Callable>
auto system(Callable callable) {
  return CallableSystem<Callable, Stores<>, Drive, Optional...>{
      std::move(callable)};
}

template <typename Drive, typename... Optional, typename... LookupComponents,
          typename Callable>
auto system(Stores<LookupComponents...>, Callable callable) {
  return CallableSystem<Callable, Stores<LookupComponents...>, Drive,
                        Optional...>{std::move(callable)};
}

//-- Schedule metafunctions ----------------------------------------------------

template <typename Type>
struct Flatten {
  using type = TypeList<Type>;
};

template <typename... SystemTypes>
struct Flatten<Systems<SystemTypes...>> {
  using type = concatenate_t<typename Flatten<SystemTypes>::type...>;
};

template <typename Schedule>
using flatten_t = typename Flatten<Schedule>::type;

template <typename StoresType>
struct LookupList;

template <typename... Components>
struct LookupList<Stores<Components...>> {
  using type = TypeList<Components...>;
};

template <typename SystemType>
using lookups_of_t = typename LookupList<typename SystemType::Lookups>::type;

template <typename SystemType>
using components_of_t =
    map_t<concatenate_t<TypeList<typename SystemType::DriveType>,
                        typename SystemType::OptionalTypes>,
          std::remove_const_t>;

template <typename List>
struct WritesOf;

template <typename... Types>
struct WritesOf<TypeList<Types...>> {
  using type =
      concatenate_t<std::conditional_t<std::is_const_v<Types>, TypeList<>,
                                       TypeList<Types>>...>;
};

template <typename List>
struct ReadsOf;

template <typename... Types>
struct ReadsOf<TypeList<Types...>> {
  using type = concatenate_t<
      std::conditional_t<std::is_const_v<Types>,
                         TypeList<std::remove_const_t<Types>>, TypeList<>>...>;
};

template <typename SystemType>
using declared_of_t = concatenate_t<TypeList<typename SystemType::DriveType>,
                                    typename SystemType::OptionalTypes>;

// The components a system writes: those it declares without const.
template <typename SystemType>
using writes_of_t = typename WritesOf<declared_of_t<SystemType>>::type;

// The components a system reads without writing: those it declares const.
template <typename SystemType>
using reads_of_t = typename ReadsOf<declared_of_t<SystemType>>::type;

// Whether every system `SystemType` must run after comes earlier in `List`.
template <typename List, typename SystemType>
constexpr bool after_satisfied() {
  using AfterList = flatten_t<typename SystemType::After>;
  bool satisfied = true;
  for_each_type(AfterList{}, [&]<typename Before>() {
    satisfied = satisfied && contains_v<List, Before> &&
                index_of_v<List, Before> < index_of_v<List, SystemType>;
  });
  return satisfied;
}

template <typename List>
struct ScheduleCheck;

template <typename... SystemTypes>
struct ScheduleCheck<TypeList<SystemTypes...>> {
  static constexpr bool unique = is_unique_v<TypeList<SystemTypes...>>;
  static constexpr bool ordered =
      (after_satisfied<TypeList<SystemTypes...>, SystemTypes>() && ...);
};

// Whether a schedule lists each system once and satisfies every `After`.
template <typename Schedule>
inline constexpr bool is_valid_schedule_v =
    ScheduleCheck<flatten_t<Schedule>>::unique &&
    ScheduleCheck<flatten_t<Schedule>>::ordered;

//-- Context -------------------------------------------------------------------

// What a system may use besides its own entity's components: the step, the
// lookups it declared, spatial and name queries, and builders.
template <typename SystemType, typename WorldType>
class Context final {
 public:
  using World = WorldType;
  using SpatialComponent = typename World::SpatialComponent;

  // Keeps a reference to `world` for as long as the context lives.
  Context(lib::Depend<World> world, Step step)
      : world_{world.get()}, step_{step} {}

  const Step& step() const { return step_; }

  template <typename Component>
  const Component* lookup(Entity entity) const {
    static_assert(
        contains_v<lookups_of_t<SystemType>, Component>,
        "Declare this component in the system's Lookups to look it up.");
    return world_->template store<Component>().try_get(entity);
  }

  // A declared lookup store, for whole-store reads such as in `prepare`.
  template <typename Component>
  const Store<Component>& store() const {
    static_assert(
        contains_v<lookups_of_t<SystemType>, Component>,
        "Declare this component in the system's Lookups to read its store.");
    return world_->template store<Component>();
  }

  template <typename Visitor>
  void within(const SpatialComponent& center,
              distance_of_t<SpatialComponent> radius, Visitor&& visit) const {
    static_assert(contains_v<lookups_of_t<SystemType>, SpatialComponent>,
                  "Declare the spatial component in the system's Lookups to "
                  "query space.");
    world_->within(center, radius, std::forward<Visitor>(visit));
  }

  bool alive(Entity entity) const { return world_->alive(entity); }
  Name name_of(Entity entity) const { return world_->name_of(entity); }
  Name archetype_of(Entity entity) const {
    return world_->archetype_of(entity);
  }
  std::optional<Entity> entity_of(Name name) const {
    return world_->entity_of(name);
  }
  std::optional<Name> find(std::string_view identity) const {
    return world_->find(identity);
  }
  std::vector<Name> find_alias(std::string_view alias) const {
    return world_->find_alias(alias);
  }

  template <ArchetypeType Archetype>
  auto create(std::string_view alias = {}) {
    return world_->template create<Archetype>(alias);
  }
  auto change(Entity entity) { return world_->change(entity); }
  auto destroy(Entity entity) { return world_->destroy(entity); }

 private:
  World* world_;  // Never null; checked once by Depend at construction.
  Step step_;
};

// Free-function forms of the Context member templates, so a system whose
// context parameter is `auto&` can write `lookup<Collider>(context, other)`
// instead of `context.template lookup<Collider>(other)`. Found by
// argument-dependent lookup.
template <typename Component, typename SystemType, typename World>
const Component* lookup(const Context<SystemType, World>& context,
                        Entity entity) {
  return context.template lookup<Component>(entity);
}

template <typename Component, typename SystemType, typename World>
const Store<Component>& store(const Context<SystemType, World>& context) {
  return context.template store<Component>();
}

template <ArchetypeType Archetype, typename SystemType, typename World>
auto create(lib::InOut<Context<SystemType, World>> context,
            std::string_view alias = {}) {
  return context->template create<Archetype>(alias);
}

//-- Running systems -----------------------------------------------------------

struct SystemRunner final {
  template <typename SystemType, typename World>
  static void run(lib::InOut<SystemType> system, lib::InOut<World> world,
                  const Step& step) {
    using Components = components_of_t<SystemType>;
    using Lookups = lookups_of_t<SystemType>;
    using Writes = writes_of_t<SystemType>;
    static_assert(is_unique_v<Components>,
                  "A system may name each component once.");
    static_assert(is_subset_v<Components, typename World::ComponentList>,
                  "A system names a component that is not in the world.");
    static_assert(is_subset_v<Lookups, typename World::ComponentList>,
                  "A system looks up a component that is not in the world.");
    static_assert(!intersects_v<Writes, Lookups>,
                  "A system cannot both write a component and look it up; "
                  "reading an array partway through writing it depends on "
                  "iteration order.");

    Context<SystemType, World> context{lib::Depend<World>{*world}, step};
    if constexpr (requires { system->prepare(context); }) {
      system->prepare(context);
    }
    loop(system, world, lib::InOut(context),
         typename SystemType::OptionalTypes{});
    if constexpr (requires { system->resolve(context); }) {
      system->resolve(context);
    }
  }

 private:
  template <typename Component, typename World>
  static decltype(auto) store_for(lib::InOut<World> world) {
    if constexpr (std::is_const_v<Component>) {
      return std::as_const(*world)
          .template store<std::remove_const_t<Component>>();
    } else {
      return world->template mutable_store<Component>(SystemAccess{});
    }
  }

  template <typename SystemType, typename World, typename ContextType,
            typename... Optional>
  static void loop(lib::InOut<SystemType> system, lib::InOut<World> world,
                   lib::InOut<ContextType> context, TypeList<Optional...>) {
    using Drive = typename SystemType::DriveType;
    static_assert(
        std::is_invocable_v<SystemType&, Entity, Drive&, Optional*...,
                            ContextType&>,
        "A system's call operator must accept (Entity, Drive&, Optional*..., "
        "Context&), with const exactly where the System declares it.");

    auto&& drive = store_for<Drive>(world);
    auto optional = std::forward_as_tuple(store_for<Optional>(world)...);
    // Unwrapped once, outside the loop, so the hot path has no pointer checks.
    SystemType& call = *system;
    ContextType& shared = *context;
    std::apply(
        [&](auto&... optional_store) {
          for (std::size_t i = 0; i < drive.size(); ++i) {
            Entity entity = drive.owner(i);
            call(entity, drive.data(i), optional_store.try_get(entity)...,
                 shared);
          }
        },
        optional);
  }
};

//-- Scheduler -----------------------------------------------------------------

template <typename List>
struct TupleOf;

template <typename... Types>
struct TupleOf<TypeList<Types...>> {
  using type = std::tuple<Types...>;
};

template <typename Type>
inline constexpr bool is_systems_v = false;

template <typename... SystemTypes>
inline constexpr bool is_systems_v<Systems<SystemTypes...>> = true;

// Flattens a schedule value into a tuple of its systems, in order.
template <typename Schedule>
auto flatten_systems(Schedule&& schedule) {
  if constexpr (is_systems_v<std::remove_cvref_t<Schedule>>) {
    return std::apply(
        [](auto&&... nested) {
          return std::tuple_cat(
              flatten_systems(std::forward<decltype(nested)>(nested))...);
        },
        std::forward<Schedule>(schedule).systems);
  } else {
    return std::tuple<std::remove_cvref_t<Schedule>>{
        std::forward<Schedule>(schedule)};
  }
}

// Runs a schedule's systems in order, applying each system's commands before
// the next one runs. Holds one instance of each system, so systems may keep
// state between stages and steps.
template <typename World, typename Schedule>
class Scheduler final {
 public:
  using SystemList = flatten_t<Schedule>;

  static_assert(ScheduleCheck<SystemList>::unique,
                "A schedule may list each system once.");
  static_assert(ScheduleCheck<SystemList>::ordered,
                "A system is scheduled before a system it must run after.");

  Scheduler()
    requires std::is_default_constructible_v<typename TupleOf<SystemList>::type>
  = default;
  explicit Scheduler(Schedule schedule)
      : systems_{flatten_systems(std::move(schedule))} {}

  void step(lib::InOut<World> world, const Step& step) {
    std::apply(
        [&](auto&... system) {
          ((SystemRunner::run(lib::InOut(system), world, step), world->sync()),
           ...);
        },
        systems_);
  }

  template <typename SystemType>
  SystemType& system() {
    return std::get<SystemType>(systems_);
  }

  // A system's name: its position in the flattened schedule.
  template <typename SystemType>
  static constexpr Name name_of() {
    static_assert(contains_v<SystemList, SystemType>,
                  "This system is not scheduled.");
    return Name{Kind::SYSTEM,
                static_cast<std::uint32_t>(index_of_v<SystemList, SystemType>)};
  }

  // The flattened schedule, one system per entry with its identity in world
  // number `world`, reads, writes and lookups. Generated from the types, so it
  // cannot go out of date.
  static std::string describe(std::uint32_t world = 0) {
    std::string text;
    for_each_type(SystemList{}, [&]<typename SystemType>() {
      text += std::format(
          "{} {}\n  writes: {}\n  reads: {}\n  lookups: {}\n",
          identity_of(world, name_of<SystemType>()),
          lib::to_type_string<SystemType>(), names(writes_of_t<SystemType>{}),
          names(reads_of_t<SystemType>{}), names(lookups_of_t<SystemType>{}));
    });
    return text;
  }

 private:
  template <typename... Types>
  static std::string names(TypeList<Types...>) {
    std::string text;
    ((text += (text.empty() ? "" : ", ") + lib::to_type_string<Types>()), ...);
    return text.empty() ? "-" : text;
  }

  typename TupleOf<SystemList>::type systems_;
};

}  // namespace simon::core
