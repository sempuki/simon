// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
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
#include "core/argument.hpp"
#include "core/time.hpp"
#include "framework/entity.hpp"
#include "framework/name.hpp"
#include "framework/timeline.hpp"
#include "framework/type_list.hpp"
#include "framework/world.hpp"

namespace simon::framework {

// A schedule: systems in execution order. Schedules nest and flatten. A
// schedule is a type (for ordering checks) and a value (holding each system,
// so systems built from lambdas with captures can be scheduled).
template <typename... SystemTypes>
struct SystemList final {
  SystemList()
    requires(std::is_default_constructible_v<SystemTypes> && ...)
  = default;
  explicit SystemList(SystemTypes... given) : systems{std::move(given)...} {}

  std::tuple<SystemTypes...> systems;
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
  explicit ProjectedWorld(Depend<WorldType> world) : world_{world.get()} {}

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
                              TypeList<>, false>{Depend(*world_)};
  }
  auto destroy() {
    return DestroyQueryBuilder<WorldType, ReadAllowed, void>{Depend(*world_)};
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

//-- Running systems -----------------------------------------------------------

struct SystemRunner final {
  template <typename StepType, typename SystemType, typename WorldType>
  static auto run(const StepType& step, InOut<SystemType> system,
                  InOut<WorldType> world) -> void {
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

    ProjectedWorld<SystemType, WorldType> access{Depend(*world)};
    // A prepare stage that returns false skips the per-entity loop and the
    // resolve stage after it, for steps with nothing to do. Resolve is the
    // loop's post-processing; work the step needs regardless belongs in
    // prepare.
    bool proceed = stage(
        step,
        [](auto& target,
           auto&... arguments) -> decltype(target.prepare(arguments...)) {
          return target.prepare(arguments...);
        },
        system, InOut(access));
    if (!proceed) {
      return;
    }
    loop(step, typename SystemType::OtherComponentList{}, system, world,
         InOut(access));
    stage(
        step,
        [](auto& target,
           auto&... arguments) -> decltype(target.resolve(arguments...)) {
          return target.resolve(arguments...);
        },
        system, InOut(access));
  }

 private:
  // Calls an optional stage (prepare or resolve) as stage(world, step) or
  // stage(world), whichever the system declares. Returns what a stage that
  // returns bool returned, and true otherwise.
  template <typename StepType, typename SystemType, typename ProjectedWorldType,
            typename CallType>
  static auto stage(const StepType& step, CallType call,
                    InOut<SystemType> system, InOut<ProjectedWorldType> access)
      -> bool {
    StepType copy = step;
    auto outcome = [](auto&& invoke) {
      if constexpr (std::is_same_v<decltype(invoke()), bool>) {
        return invoke();
      } else {
        invoke();
        return true;
      }
    };
    if constexpr (std::is_invocable_v<CallType, SystemType&,
                                      ProjectedWorldType&, StepType&>) {
      return outcome([&] { return call(*system, *access, copy); });
    } else if constexpr (std::is_invocable_v<CallType, SystemType&,
                                             ProjectedWorldType&>) {
      return outcome([&] { return call(*system, *access); });
    } else {
      return true;
    }
  }

  template <typename StepType, typename SystemType, typename WorldType,
            typename ProjectedWorldType, typename... OtherComponentTypes>
  static auto loop(const StepType& step, TypeList<OtherComponentTypes...>,
                   InOut<SystemType> system, InOut<WorldType> world,
                   InOut<ProjectedWorldType> access) -> void {
    using DrivingComponentType = typename SystemType::DrivingComponent;
    constexpr bool TAKES_STEP =
        std::is_invocable_v<SystemType&, ProjectedWorldType&, Entity,
                            DrivingComponentType&, OtherComponentTypes*...,
                            StepType>;
    static_assert(
        TAKES_STEP ||
            std::is_invocable_v<SystemType&, ProjectedWorldType&, Entity,
                                DrivingComponentType&, OtherComponentTypes*...>,
        "A system's call operator must accept (ProjectedWorld&, Entity, "
        "DrivingComponentType&, OtherComponentTypes*...[, Step]), with const "
        "exactly where the System declares it.");

    // Unwrapped once, outside the loop, so the hot path has no pointer checks.
    SystemType& call = *system;
    ProjectedWorldType& shared = *access;
    world->template for_each_with<DrivingComponentType, OtherComponentTypes...>(
        SchedulerKey{}, exclude_component_list_of_t<SystemType>{},
        [&](Entity entity, DrivingComponentType& driving,
            OtherComponentTypes*... others) {
          if constexpr (TAKES_STEP) {
            call(shared, entity, driving, others..., step);
          } else {
            call(shared, entity, driving, others...);
          }
        });
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
template <typename Type, typename WorldType, typename StepType = Step>
concept RunsItself =
    requires(Type& element, const StepType& step, InOut<WorldType> world) {
      element.run(step, world);
    };

// A system that declares the period it runs at, `static constexpr Duration
// PERIOD = 100ms;`, unless its scheduler is told otherwise.
template <typename Type>
concept HasPeriod = requires {
  { Duration{Type::PERIOD} } -> std::same_as<Duration>;
};

// Runs a schedule's systems in order, applying each system's commands before
// the next one runs. Holds one instance of each system, so systems may keep
// state between stages and steps.
template <typename WorldType, typename ScheduleType>
class Scheduler final : private Timeline::Source {
 public:
  using FlattenedSystemList = flattened_list_t<ScheduleType>;

  static_assert(ScheduleCheck<FlattenedSystemList>::unique,
                "A schedule may list each system once.");
  static_assert(ScheduleCheck<FlattenedSystemList>::ordered,
                "A system is scheduled before a system it must run after.");

  Scheduler()
    requires std::is_default_constructible_v<
        typename TupleOf<FlattenedSystemList>::type>
      : rates_{declared_rates()} {}
  explicit Scheduler(ScheduleType schedule)
      : systems_{flatten_systems(std::move(schedule))},
        rates_{declared_rates()} {}

  // Runs every system for `step`, a BasicStep of the simulation's tick. A
  // system with a period runs only in a step that holds one of its
  // boundaries; see set_period.
  template <typename StepType = Step>
  auto step(const StepType& step, InOut<WorldType> world) -> void {
    std::size_t index = 0;
    std::apply(
        [&](auto&... system) {
          (run_at_rate(rates_[index++], step, system, world), ...);
        },
        systems_);
  }

  // Runs `SystemType` once per `period` instead of every step, at `phase`
  // plus whole periods from time zero, skipping it whole between: no
  // prepare, no loop, no resolve. A boundary that passed before it is next
  // stepped makes it run on that step. Each run's step lasts the time since
  // its last run, its period on the first. Periods count nanoseconds, so only
  // a simulation stepped in nanoseconds can use them.
  template <typename SystemType>
  auto set_period(Duration period, Duration phase = Duration::zero()) -> void {
    CHECK_PRECONDITION(period > Duration::zero());
    CHECK_PRECONDITION(phase >= Duration::zero() && phase < period);
    Rate& rate = rate_of<SystemType>();
    if (timeline_ && !rate.period) {
      timeline_->add_continuous(-1);
    }
    rate = Rate{.period = period, .next = TimePoint{} + phase};
  }

  // Runs `SystemType` every step again.
  template <typename SystemType>
  auto clear_period() -> void {
    Rate& rate = rate_of<SystemType>();
    if (timeline_ && rate.period) {
      timeline_->add_continuous(1);
    }
    rate = Rate{};
  }

  template <typename SystemType>
  auto period_of() const -> std::optional<Duration> {
    return rates_[index_of_v<FlattenedSystemList, SystemType>].period;
  }

  // Tells `timeline` when each system with a period is next due, and how many
  // systems run every step, for as long as the scheduler lives and stays in
  // place.
  auto attach(Depend<Timeline> timeline) -> void {
    CHECK_PRECONDITION(!timeline_);
    timeline_ = timeline.get();
    timeline_->add(Depend<const Timeline::Source>(*this));
    for (const Rate& rate : rates_) {
      if (!rate.period) {
        timeline_->add_continuous(1);
      }
    }
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
          format_identity(world, name_of<SystemType>()),
          lib::to_type_string<SystemType>(),
          names(write_list_of_t<SystemType>{}),
          names(read_list_of_t<SystemType>{}),
          names(allow_component_list_of_t<SystemType>{}),
          names(exclude_component_list_of_t<SystemType>{}));
      if constexpr (HasPeriod<SystemType>) {
        text += std::format("  period: {}\n", Duration{SystemType::PERIOD});
      }
    });
    return text;
  }

 private:
  // A schedule element that runs itself, such as Continuous, or a system.
  template <typename StepType, typename SystemType>
  static auto run(const StepType& step, SystemType& system,
                  InOut<WorldType> world) -> void {
    if constexpr (RunsItself<SystemType, WorldType, StepType>) {
      system.run(step, world);
    } else {
      SystemRunner::run(step, InOut(system), world);
    }
  }

  // When a system runs: every step without a period; with one, at each of
  // its boundaries.
  struct Rate final {
    std::optional<Duration> period;
    TimePoint next{};               // Its next boundary, with a period.
    std::optional<TimePoint> last;  // When it last ran.
  };

  using Rates = std::array<Rate, FlattenedSystemList::size>;

  // Each system's period as its type declares it, if it does.
  static auto declared_rates() -> Rates {
    Rates rates{};
    std::size_t index = 0;
    for_each_type(FlattenedSystemList{}, [&]<typename SystemType>() {
      if constexpr (HasPeriod<SystemType>) {
        rates[index].period = Duration{SystemType::PERIOD};
        rates[index].next = TimePoint{};
      }
      ++index;
    });
    return rates;
  }

  template <typename SystemType>
  auto rate_of() -> Rate& {
    static_assert(contains_v<FlattenedSystemList, SystemType>,
                  "This system is not scheduled.");
    return rates_[index_of_v<FlattenedSystemList, SystemType>];
  }

  // The earliest boundary of any system with a period.
  auto earliest() const -> std::optional<TimePoint> override {
    std::optional<TimePoint> first;
    for (const Rate& rate : rates_) {
      if (rate.period && (!first || rate.next < *first)) {
        first = rate.next;
      }
    }
    return first;
  }

  // The earliest boundary after `time` of any system with a period.
  auto earliest_after(TimePoint time) const
      -> std::optional<TimePoint> override {
    std::optional<TimePoint> first;
    for (const Rate& rate : rates_) {
      if (!rate.period) {
        continue;
      }
      TimePoint after =
          rate.next > time
              ? rate.next
              : rate.next +
                    ((time - rate.next) / *rate.period + 1) * *rate.period;
      if (!first || after < *first) {
        first = after;
      }
    }
    return first;
  }

  // Runs `system` for `step` if it runs every step, or if `step` holds one of
  // its boundaries or starts at or after one (an instant, a step of no
  // length, holds the one at its time), then moves its next boundary past the
  // step.
  template <typename StepType, typename SystemType>
  auto run_at_rate(Rate& rate, const StepType& step, SystemType& system,
                   InOut<WorldType> world) -> void {
    if (!rate.period) {
      run(step, system, world);
      world->sync();
      return;
    }
    if constexpr (std::same_as<StepType, Step>) {
      TimePoint end = step.time + step.dt;
      if (rate.next < end || rate.next <= step.time) {
        Duration dt = rate.last ? step.time - *rate.last : *rate.period;
        run(Step{.time = step.time, .dt = dt}, system, world);
        world->sync();
        rate.last = step.time;
        // Past every boundary the step holds, and the one at an instant.
        Duration passed = std::max(end - rate.next, Duration{1});
        rate.next += ((passed - Duration{1}) / *rate.period + 1) * *rate.period;
      }
    } else {
      // Periods count nanoseconds; a simulation with a coarser tick cannot
      // set one.
      CHECK_UNREACHABLE();
    }
  }

  template <typename... Types>
  static auto names(TypeList<Types...>) -> std::string {
    std::string text;
    ((text += (text.empty() ? "" : ", ") + lib::to_type_string<Types>()), ...);
    return text.empty() ? "-" : text;
  }

  typename TupleOf<FlattenedSystemList>::type systems_;
  Rates rates_;
  Timeline* timeline_ = nullptr;
};

}  // namespace simon::framework
