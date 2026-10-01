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
#include "framework/entity.hpp"
#include "framework/name.hpp"
#include "framework/step.hpp"
#include "framework/type_list.hpp"
#include "framework/world.hpp"

namespace simon::framework {

// A schedule: systems in execution order. Schedules nest and flatten. A
// schedule is a type (for ordering checks) and a value (holding each system,
// so systems built from lambdas with captures can be scheduled).
template <typename... SystemTypes>
struct SystemList final {
  std::tuple<SystemTypes...> systems;

  SystemList()
    requires(std::is_default_constructible_v<SystemTypes> && ...)
  = default;
  explicit SystemList(SystemTypes... given) : systems{std::move(given)...} {}
};

template <typename... SystemTypes>
SystemList(SystemTypes...) -> SystemList<SystemTypes...>;

// A system's structure. The first component drives the loop and comes in by
// reference; every following component is optional, belongs to the same
// entity, and comes in by pointer (null when absent). Declare a component
// `const` to read it without writing it. A system may also declare an
// AllowComponentList (components it may read, read-only, from other entities),
// an ExcludeComponentList (components whose owners it does not run for) and a
// SequenceAfterSystemList (systems it must be scheduled after).
//
//   struct Integrate final  //
//       : System<Kinematics,  //
//                const Control> {
//     auto operator()(auto& world, Entity,  //
//                     Kinematics& kinematics,  //
//                     const Control* control,  //
//                     Step step) -> void;
//   };
template <typename DrivingComponentType, typename... OtherComponentTypes>
struct System {
  using DrivingComponent = DrivingComponentType;
  using OtherComponentList = TypeList<OtherComponentTypes...>;
  using AllowComponentList = TypeList<>;
  using ExcludeComponentList = TypeList<>;
  using SequenceAfterSystemList = SystemList<>;
};

// A system made from any callable, such as a lambda. The callable keeps its
// own state (its captures) between steps.
//
//   auto integrate = system<Kinematics, const Control>(
//       [](auto& world, Entity, Kinematics&, const Control*, Step) { ... });
//   auto guide = system<const Interceptor, Control>(
//       TypeList<Kinematics>{},  // AllowComponentList.
//       [](auto& world, Entity, const Interceptor&, Control*) { ... });
template <typename CallableType, typename AllowComponentListType,
          typename DrivingComponentType, typename... OtherComponentTypes>
class CallableSystem final
    : public System<DrivingComponentType, OtherComponentTypes...> {
 public:
  using AllowComponentList = AllowComponentListType;

  explicit CallableSystem(CallableType callable)
      : callable_{std::move(callable)} {}

  template <typename... ArgumentTypes>
    requires std::invocable<CallableType&, ArgumentTypes...>
  auto operator()(ArgumentTypes&&... arguments) -> void {
    std::invoke(callable_, std::forward<ArgumentTypes>(arguments)...);
  }

  auto callable() -> CallableType& { return callable_; }

 private:
  CallableType callable_;
};

template <typename DrivingComponentType, typename... OtherComponentTypes,
          typename CallableType>
auto system(CallableType callable) {
  return CallableSystem<CallableType, TypeList<>, DrivingComponentType,
                        OtherComponentTypes...>{std::move(callable)};
}

template <typename DrivingComponentType, typename... OtherComponentTypes,
          typename... AllowedComponentTypes, typename CallableType>
auto system(TypeList<AllowedComponentTypes...>, CallableType callable) {
  return CallableSystem<CallableType, TypeList<AllowedComponentTypes...>,
                        DrivingComponentType, OtherComponentTypes...>{
      std::move(callable)};
}

//-- ScheduleType metafunctions
//----------------------------------------------------

template <typename Type>
struct Flatten final {
  using type = TypeList<Type>;
};

template <typename... SystemTypes>
struct Flatten<SystemList<SystemTypes...>> final {
  using type = concatenate_t<typename Flatten<SystemTypes>::type...>;
};

template <typename ScheduleType>
using flattened_list_t = typename Flatten<ScheduleType>::type;

// The components a system may read by entity (its AllowComponentList).
template <typename SystemType>
using allow_component_list_of_t = typename SystemType::AllowComponentList;

// The components whose owners a system does not run for (its
// ExcludeComponentList).
template <typename SystemType>
using exclude_component_list_of_t = typename SystemType::ExcludeComponentList;

template <typename SystemType>
using component_list_of_t =
    map_t<concatenate_t<TypeList<typename SystemType::DrivingComponent>,
                        typename SystemType::OtherComponentList>,
          std::remove_const_t>;

template <typename ListType>
struct WriteListOf;

template <typename... Types>
struct WriteListOf<TypeList<Types...>> final {
  using type =
      concatenate_t<std::conditional_t<std::is_const_v<Types>, TypeList<>,
                                       TypeList<Types>>...>;
};

template <typename ListType>
struct ReadListOf;

template <typename... Types>
struct ReadListOf<TypeList<Types...>> final {
  using type = concatenate_t<
      std::conditional_t<std::is_const_v<Types>,
                         TypeList<std::remove_const_t<Types>>, TypeList<>>...>;
};

template <typename SystemType>
using declared_list_of_t =
    concatenate_t<TypeList<typename SystemType::DrivingComponent>,
                  typename SystemType::OtherComponentList>;

// The components a system writes: those it declares without const.
template <typename SystemType>
using write_list_of_t =
    typename WriteListOf<declared_list_of_t<SystemType>>::type;

// The components a system reads without writing: those it declares const.
template <typename SystemType>
using read_list_of_t =
    typename ReadListOf<declared_list_of_t<SystemType>>::type;

template <typename ListType>
struct BytesOf;

template <typename... Types>
struct BytesOf<TypeList<Types...>> final {
  static constexpr std::size_t value = (sizeof(Types) + ... + 0);
};

// The bytes a system's per-entity loop can read for each entity: the owner,
// the driving component and every other component it names. An upper bound:
// a sibling the entity's archetype cannot have costs nothing, and one it only
// allows also costs an index entry. Reads of other entities through
// ProjectedWorld come on top.
template <typename SystemType>
inline constexpr std::size_t bytes_per_entity_v =
    sizeof(Entity) + BytesOf<declared_list_of_t<SystemType>>::value;

// Whether every system `SystemType` must run after that is in `ListType` comes
// earlier in it.
template <typename ListType, typename SystemType>
constexpr auto after_satisfied() -> bool {
  using AfterList =
      flattened_list_t<typename SystemType::SequenceAfterSystemList>;
  bool satisfied = true;
  for_each_type(AfterList{}, [&]<typename BeforeType>() {
    // Ordering only: a system that is not scheduled imposes nothing, so
    // sub-schedules can run alone, e.g. in tests.
    satisfied = satisfied && (!contains_v<ListType, BeforeType> ||
                              index_of_v<ListType, BeforeType> <
                                  index_of_v<ListType, SystemType>);
  });
  return satisfied;
}

template <typename ListType>
struct ScheduleCheck;

template <typename... SystemTypes>
struct ScheduleCheck<TypeList<SystemTypes...>> final {
  static constexpr bool unique = is_unique_v<TypeList<SystemTypes...>>;
  static constexpr bool ordered =
      (after_satisfied<TypeList<SystemTypes...>, SystemTypes>() && ...);
};

// Whether a schedule lists each system once and satisfies every
// `SequenceAfterSystemList`.
template <typename ScheduleType>
inline constexpr bool is_valid_schedule_v =
    ScheduleCheck<flattened_list_t<ScheduleType>>::unique &&
    ScheduleCheck<flattened_list_t<ScheduleType>>::ordered;

//-- ProjectedWorld
//-------------------------------------------------------------------

// The world projected onto what a system declares, as a database projects a
// table onto some of its columns: the components its AllowComponentList
// declares (read-only), besides its own entity's, with spatial and name
// queries and builders. Projection picks stores; it never filters entities.
template <typename SystemType, typename WorldType>
class ProjectedWorld final {
 public:
  using SpatialComponent = typename WorldType::SpatialComponent;

  // Keeps a reference to `world` for as long as the access lives.
  explicit ProjectedWorld(lib::Depend<WorldType> world) : world_{world.get()} {}

  // Another entity's `ComponentType`, or null if it is gone or lacks one.
  template <typename ComponentType>
  auto maybe_component_of(Entity entity) const -> const ComponentType* {
    static_assert(
        contains_v<allow_component_list_of_t<SystemType>, ComponentType>,
        "Declare this component in the system's AllowComponentList to read "
        "it.");
    return world_->template store_of<ComponentType>().maybe_component_of(
        entity);
  }

  // Another entity's `ComponentType`. Fails a contract check if it has none.
  template <typename ComponentType>
  auto component_of(Entity entity) const -> const ComponentType& {
    const ComponentType* component = maybe_component_of<ComponentType>(entity);
    CHECK_PRECONDITION(component);
    return *component;
  }

  // A store in the system's AllowComponentList, for whole-store reads.
  template <typename ComponentType>
  auto store_of() const -> const ComponentStore<ComponentType>& {
    static_assert(
        contains_v<allow_component_list_of_t<SystemType>, ComponentType>,
        "Declare this component in the system's AllowComponentList to read its "
        "store.");
    return world_->template store_of<ComponentType>();
  }

  template <typename VisitorType>
  auto within(const SpatialComponent& center,
              distance_of_t<SpatialComponent> radius, VisitorType&& visit) const
      -> void {
    static_assert(
        contains_v<allow_component_list_of_t<SystemType>, SpatialComponent>,
        "Declare the spatial component in the system's AllowComponentList to "
        "query space.");
    world_->within(center, radius, std::forward<VisitorType>(visit));
  }

  template <typename AcceptType>
  auto nearest(const SpatialComponent& center,
               distance_of_t<SpatialComponent> radius,
               AcceptType&& accept) const -> std::optional<Entity> {
    static_assert(
        contains_v<allow_component_list_of_t<SystemType>, SpatialComponent>,
        "Declare the spatial component in the system's AllowComponentList to "
        "query space.");
    return world_->nearest(center, radius, std::forward<AcceptType>(accept));
  }

  auto alive(Entity entity) const -> bool { return world_->alive(entity); }
  auto name_of(Entity entity) const -> Name { return world_->name_of(entity); }
  auto archetype_of(Entity entity) const -> Name {
    return world_->archetype_of(entity);
  }
  auto entity_of(Name name) const -> std::optional<Entity> {
    return world_->entity_of(name);
  }
  auto find_name_of(const Identity& identity) const -> std::optional<Name> {
    return world_->find_name_of(identity);
  }
  auto find_name_of(const Alias& alias) const -> std::vector<Name> {
    return world_->find_name_of(alias);
  }

  template <Archetypal ArchetypeType>
  auto create(Alias alias = {}) {
    return world_->template create<ArchetypeType>(alias);
  }
  auto change(Entity entity) { return world_->change(entity); }
  auto destroy(Entity entity) { return world_->destroy(entity); }

  // Query forms, which select only by what the system's AllowComponentList
  // declares. Use them from prepare or resolve, which run once per step, not
  // from the per-entity call.
  auto change() {
    return ChangeQueryBuilder<WorldType, ReadAllowed, void, TypeList<>,
                              TypeList<>, false>{lib::Depend(*world_)};
  }
  auto destroy() {
    return DestroyQueryBuilder<WorldType, ReadAllowed, void>{
        lib::Depend(*world_)};
  }

 private:
  struct ReadAllowed final {
    template <typename ComponentType>
    static constexpr bool can_read =
        contains_v<allow_component_list_of_t<SystemType>, ComponentType>;
  };

  // Never null once constructed; Depend checks it.
  WorldType* world_ = nullptr;
};

// Free-function forms of the ProjectedWorld member templates, so a system whose
// access parameter is `auto&` can write
// `maybe_component_of<Collider>(access, other)` instead of
// `access.template maybe_component_of<Collider>(other)`. Found by
// argument-dependent lookup.
template <typename ComponentType, typename SystemType, typename WorldType>
auto maybe_component_of(const ProjectedWorld<SystemType, WorldType>& access,
                        Entity entity) -> const ComponentType* {
  return access.template maybe_component_of<ComponentType>(entity);
}

template <typename ComponentType, typename SystemType, typename WorldType>
auto component_of(const ProjectedWorld<SystemType, WorldType>& access,
                  Entity entity) -> const ComponentType& {
  return access.template component_of<ComponentType>(entity);
}

template <typename ComponentType, typename SystemType, typename WorldType>
auto store_of(const ProjectedWorld<SystemType, WorldType>& access)
    -> const ComponentStore<ComponentType>& {
  return access.template store_of<ComponentType>();
}

template <Archetypal ArchetypeType, typename SystemType, typename WorldType>
auto create(Alias alias,
            lib::InOut<ProjectedWorld<SystemType, WorldType>> access) {
  return access->template create<ArchetypeType>(std::move(alias));
}
template <Archetypal ArchetypeType, typename SystemType, typename WorldType>
auto create(lib::InOut<ProjectedWorld<SystemType, WorldType>> access) {
  return access->template create<ArchetypeType>();
}

//-- Running systems -----------------------------------------------------------

struct SystemRunner final {
  template <typename SystemType, typename WorldType>
  static auto run(const Step& step, lib::InOut<SystemType> system,
                  lib::InOut<WorldType> world) -> void {
    using ComponentList = component_list_of_t<SystemType>;
    using AllowComponentList = allow_component_list_of_t<SystemType>;
    using WriteList = write_list_of_t<SystemType>;
    static_assert(is_unique_v<ComponentList>,
                  "A system may name each component once.");
    static_assert(is_subset_v<ComponentList, typename WorldType::ComponentList>,
                  "A system names a component that is not in the world.");
    static_assert(
        is_subset_v<AllowComponentList, typename WorldType::ComponentList>,
        "A system allows a component that is not in the world.");
    static_assert(!intersects_v<WriteList, AllowComponentList>,
                  "A system cannot both write a component and allow reading "
                  "it by entity; "
                  "reading an array partway through writing it depends on "
                  "iteration order.");

    using ExcludeComponentList = exclude_component_list_of_t<SystemType>;
    static_assert(
        is_subset_v<ExcludeComponentList, typename WorldType::ComponentList>,
        "A system excludes a component that is not in the world.");
    static_assert(!intersects_v<ComponentList, ExcludeComponentList>,
                  "A system cannot both name a component and exclude its "
                  "owners; the component would never be there.");

    ProjectedWorld<SystemType, WorldType> access{lib::Depend(*world)};
    // A prepare stage that returns false skips the per-entity loop, for steps
    // with nothing to do.
    bool proceed = stage(
        step,
        [](auto& target,
           auto&... arguments) -> decltype(target.prepare(arguments...)) {
          return target.prepare(arguments...);
        },
        system, lib::InOut(access));
    if (proceed) {
      loop(step, typename SystemType::OtherComponentList{}, system, world,
           lib::InOut(access));
    }
    stage(
        step,
        [](auto& target,
           auto&... arguments) -> decltype(target.resolve(arguments...)) {
          return target.resolve(arguments...);
        },
        system, lib::InOut(access));
  }

 private:
  template <typename ComponentType, typename WorldType>
  static auto store_for(lib::InOut<WorldType> world) -> decltype(auto) {
    if constexpr (std::is_const_v<ComponentType>) {
      return std::as_const(*world)
          .template store_of<std::remove_const_t<ComponentType>>();
    } else {
      return world->template mutable_store_of<ComponentType>(SchedulerKey{});
    }
  }

  // Calls an optional stage (prepare or resolve) as stage(world, step) or
  // stage(world), whichever the system declares. Returns what a stage that
  // returns bool returned, and true otherwise.
  template <typename SystemType, typename ProjectedWorldType, typename CallType>
  static auto stage(const Step& step, CallType call,
                    lib::InOut<SystemType> system,
                    lib::InOut<ProjectedWorldType> access) -> bool {
    Step copy = step;
    auto outcome = [](auto&& invoke) {
      if constexpr (std::is_same_v<decltype(invoke()), bool>) {
        return invoke();
      } else {
        invoke();
        return true;
      }
    };
    if constexpr (std::is_invocable_v<CallType, SystemType&,
                                      ProjectedWorldType&, Step&>) {
      return outcome([&] { return call(*system, *access, copy); });
    } else if constexpr (std::is_invocable_v<CallType, SystemType&,
                                             ProjectedWorldType&>) {
      return outcome([&] { return call(*system, *access); });
    } else {
      return true;
    }
  }

  template <typename SystemType, typename WorldType,
            typename ProjectedWorldType, typename... OtherComponentTypes>
  static auto loop(const Step& step, TypeList<OtherComponentTypes...>,
                   lib::InOut<SystemType> system, lib::InOut<WorldType> world,
                   lib::InOut<ProjectedWorldType> access) -> void {
    using DrivingComponentType = typename SystemType::DrivingComponent;
    constexpr bool TAKES_STEP =
        std::is_invocable_v<SystemType&, ProjectedWorldType&, Entity,
                            DrivingComponentType&, OtherComponentTypes*...,
                            Step>;
    static_assert(
        TAKES_STEP ||
            std::is_invocable_v<SystemType&, ProjectedWorldType&, Entity,
                                DrivingComponentType&, OtherComponentTypes*...>,
        "A system's call operator must accept (ProjectedWorld&, Entity, "
        "DrivingComponentType&, OtherComponentTypes*...[, Step]), with const "
        "exactly where the System declares it.");

    auto&& drive = store_for<DrivingComponentType>(world);
    auto optional =
        std::forward_as_tuple(store_for<OtherComponentTypes>(world)...);

    // Owners of an excluded component are skipped: a whole segment when its
    // archetype requires one, by lookup when it only allows one.
    using ExcludeList = exclude_component_list_of_t<SystemType>;
    auto excluded = excluded_stores(ExcludeList{}, world);
    auto is_excluded = [&](Entity entity) {
      return std::apply(
          [&](const auto&... store) {
            return (store.contains(entity) || ... || false);
          },
          excluded);
    };

    // Unwrapped once, outside the loop, so the hot path has no pointer checks.
    SystemType& call = *system;
    ProjectedWorldType& shared = *access;
    auto invoke = [&](Entity entity, DrivingComponentType& driving,
                      OtherComponentTypes*... others) {
      if constexpr (TAKES_STEP) {
        call(shared, entity, driving, others..., step);
      } else {
        call(shared, entity, driving, others...);
      }
    };
    std::apply(
        [&](auto&... optional_store) {
          using Driving = std::remove_const_t<DrivingComponentType>;
          // Each archetype that requires the driving component has a segment
          // of its own, where each other component is known, at compile time,
          // to be at the same slot of the matching chunk, absent, or allowed.
          auto walk_archetype = [&]<std::size_t ARCHETYPE>() {
            if constexpr (WorldType::template archetype_requires<Driving>(
                              ARCHETYPE) &&
                          !excludes_all<WorldType, ARCHETYPE>(ExcludeList{})) {
              constexpr bool MAY_EXCLUDE =
                  excludes_some<WorldType, ARCHETYPE>(ExcludeList{});
              constexpr std::size_t SEGMENT =
                  WorldType::template segment_of<Driving>(ARCHETYPE);
              CHECK_INVARIANT(
                  ((access_of<WorldType, OtherComponentTypes, ARCHETYPE>() !=
                        Access::REQUIRED ||
                    optional_store.segment_size(
                        segment_of<WorldType, OtherComponentTypes,
                                   ARCHETYPE>()) ==
                        drive.segment_size(SEGMENT)) &&
                   ...));
              for (std::size_t ordinal = 0; ordinal < drive.chunks_in(SEGMENT);
                   ++ordinal) {
                auto chunk = drive.chunk(SEGMENT, ordinal);
                auto bases = std::make_tuple(
                    base_of<WorldType, OtherComponentTypes, ARCHETYPE>(
                        optional_store, ordinal)...);
                for (std::size_t i = 0; i < chunk.size; ++i) {
                  if constexpr (MAY_EXCLUDE) {
                    if (is_excluded(chunk.owners[i])) {
                      continue;
                    }
                  }
                  std::apply(
                      [&](auto... base) {
                        invoke(chunk.owners[i], chunk.components[i],
                               sibling_of<WorldType, OtherComponentTypes,
                                          ARCHETYPE>(optional_store, base,
                                                     chunk.owners[i], i)...);
                      },
                      bases);
                }
              }
            }
          };
          [&]<std::size_t... ARCHETYPES>(std::index_sequence<ARCHETYPES...>) {
            (walk_archetype.template operator()<ARCHETYPES>(), ...);
          }(std::make_index_sequence<WorldType::ArchetypeList::size>{});

          // The last segment holds entities whose archetype only allows the
          // driving component, so every other component is looked up.
          std::size_t allowed = drive.segments() - 1;
          for (std::size_t ordinal = 0; ordinal < drive.chunks_in(allowed);
               ++ordinal) {
            auto chunk = drive.chunk(allowed, ordinal);
            for (std::size_t i = 0; i < chunk.size; ++i) {
              if constexpr (ExcludeList::size > 0) {
                if (is_excluded(chunk.owners[i])) {
                  continue;
                }
              }
              invoke(chunk.owners[i], chunk.components[i],
                     optional_store.maybe_component_of(chunk.owners[i])...);
            }
          }
        },
        optional);
  }

  // The stores of the excluded components, read-only. A system that excludes
  // nothing reads none of the world.
  template <typename WorldType, typename... ExcludedTypes>
  static auto excluded_stores(TypeList<ExcludedTypes...>,
                              [[maybe_unused]] lib::InOut<WorldType> world) {
    return std::forward_as_tuple(
        std::as_const(*world).template store_of<ExcludedTypes>()...);
  }

  // Whether every entity of an archetype has an excluded component, because
  // the archetype requires one.
  template <typename WorldType, std::size_t ARCHETYPE,
            typename... ExcludedTypes>
  static constexpr auto excludes_all(TypeList<ExcludedTypes...>) -> bool {
    return (WorldType::template archetype_requires<ExcludedTypes>(ARCHETYPE) ||
            ... || false);
  }

  // Whether some entities of an archetype may have an excluded component,
  // because the archetype allows one.
  template <typename WorldType, std::size_t ARCHETYPE,
            typename... ExcludedTypes>
  static constexpr auto excludes_some(TypeList<ExcludedTypes...>) -> bool {
    return (WorldType::template archetype_permits<ExcludedTypes>(ARCHETYPE) ||
            ... || false);
  }

  // How a system reaches another component of an entity whose archetype
  // requires the driving component.
  enum class Access { REQUIRED, ABSENT, ALLOWED };

  template <typename WorldType, typename OtherType, std::size_t ARCHETYPE>
  static constexpr auto access_of() -> Access {
    using Other = std::remove_const_t<OtherType>;
    if constexpr (WorldType::template archetype_requires<Other>(ARCHETYPE)) {
      return Access::REQUIRED;
    } else if constexpr (!WorldType::template archetype_permits<Other>(
                             ARCHETYPE)) {
      return Access::ABSENT;
    } else {
      return Access::ALLOWED;
    }
  }

  template <typename WorldType, typename OtherType, std::size_t ARCHETYPE>
  static constexpr auto segment_of() -> std::size_t {
    return WorldType::template segment_of<std::remove_const_t<OtherType>>(
        ARCHETYPE);
  }

  // The start of the matching chunk of a required component's segment, or
  // null.
  template <typename WorldType, typename OtherType, std::size_t ARCHETYPE,
            typename StoreType>
  static auto base_of(StoreType& store, std::size_t ordinal) -> OtherType* {
    if constexpr (access_of<WorldType, OtherType, ARCHETYPE>() ==
                  Access::REQUIRED) {
      return store.chunk(segment_of<WorldType, OtherType, ARCHETYPE>(), ordinal)
          .components;
    } else {
      return nullptr;
    }
  }

  template <typename WorldType, typename OtherType, std::size_t ARCHETYPE,
            typename StoreType>
  static auto sibling_of(StoreType& store, OtherType* base, Entity entity,
                         std::size_t i) -> OtherType* {
    constexpr Access ACCESS = access_of<WorldType, OtherType, ARCHETYPE>();
    if constexpr (ACCESS == Access::REQUIRED) {
      return base + i;
    } else if constexpr (ACCESS == Access::ABSENT) {
      return nullptr;
    } else {
      return store.maybe_component_of(entity);
    }
  }
};

//-- Scheduler -----------------------------------------------------------------

template <typename ListType>
struct TupleOf;

template <typename... Types>
struct TupleOf<TypeList<Types...>> final {
  using type = std::tuple<Types...>;
};

template <typename Type>
inline constexpr bool is_systems_v = false;

template <typename... SystemTypes>
inline constexpr bool is_systems_v<SystemList<SystemTypes...>> = true;

// Flattens a schedule value into a tuple of its systems, in order.
template <typename ScheduleType>
auto flatten_systems(ScheduleType&& schedule) {
  if constexpr (is_systems_v<std::remove_cvref_t<ScheduleType>>) {
    return std::apply(
        [](auto&&... nested) {
          return std::tuple_cat(
              flatten_systems(std::forward<decltype(nested)>(nested))...);
        },
        std::forward<ScheduleType>(schedule).systems);
  } else {
    return std::tuple<std::remove_cvref_t<ScheduleType>>{
        std::forward<ScheduleType>(schedule)};
  }
}

// A schedule element that runs itself instead of being run per entity, such
// as Continuous. It declares what it reads and writes as a system does.
template <typename Type, typename WorldType>
concept RunsItself =
    requires(Type& element, const Step& step, lib::InOut<WorldType> world) {
      element.run(step, world);
    };

// Runs a schedule's systems in order, applying each system's commands before
// the next one runs. Holds one instance of each system, so systems may keep
// state between stages and steps.
template <typename WorldType, typename ScheduleType>
class Scheduler final {
 public:
  using FlattenedSystemList = flattened_list_t<ScheduleType>;

  static_assert(ScheduleCheck<FlattenedSystemList>::unique,
                "A schedule may list each system once.");
  static_assert(ScheduleCheck<FlattenedSystemList>::ordered,
                "A system is scheduled before a system it must run after.");

  Scheduler()
    requires std::is_default_constructible_v<
                 typename TupleOf<FlattenedSystemList>::type>
  = default;
  explicit Scheduler(ScheduleType schedule)
      : systems_{flatten_systems(std::move(schedule))} {}

  auto step(const Step& step, lib::InOut<WorldType> world) -> void {
    std::apply(
        [&](auto&... system) {
          ((run(step, system, world), world->sync()), ...);
        },
        systems_);
  }

  template <typename SystemType>
  auto system() -> SystemType& {
    return std::get<SystemType>(systems_);
  }

  // A system's name: its position in the flattened schedule.
  template <typename SystemType>
  static constexpr auto name_of() -> Name {
    static_assert(contains_v<FlattenedSystemList, SystemType>,
                  "This system is not scheduled.");
    return Name{Kind::SYSTEM, static_cast<std::uint32_t>(
                                  index_of_v<FlattenedSystemList, SystemType>)};
  }

  // The flattened schedule, one system per entry with its identity in world
  // number `world`, reads, writes and allow list. Generated from the types, so
  // it cannot go out of date.
  static auto describe(std::uint32_t world = 0) -> std::string {
    std::string text;
    for_each_type(FlattenedSystemList{}, [&]<typename SystemType>() {
      text += std::format(
          "{} {}\n  writes: {}\n  reads: {}\n  allowed: {}\n  excludes: {}\n",
          identity_of(world, name_of<SystemType>()),
          lib::to_type_string<SystemType>(),
          names(write_list_of_t<SystemType>{}),
          names(read_list_of_t<SystemType>{}),
          names(allow_component_list_of_t<SystemType>{}),
          names(exclude_component_list_of_t<SystemType>{}));
    });
    return text;
  }

 private:
  // A schedule element that runs itself, such as Continuous, or a system.
  template <typename SystemType>
  static auto run(const Step& step, SystemType& system,
                  lib::InOut<WorldType> world) -> void {
    if constexpr (RunsItself<SystemType, WorldType>) {
      system.run(step, world);
    } else {
      SystemRunner::run(step, lib::InOut(system), world);
    }
  }

  template <typename... Types>
  static auto names(TypeList<Types...>) -> std::string {
    std::string text;
    ((text += (text.empty() ? "" : ", ") + lib::to_type_string<Types>()), ...);
    return text.empty() ? "-" : text;
  }

  typename TupleOf<FlattenedSystemList>::type systems_;
};

}  // namespace simon::framework
