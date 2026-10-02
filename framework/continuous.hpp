// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "base/core.hpp"
#include "framework/step.hpp"
#include "framework/system.hpp"
#include "framework/type_list.hpp"
#include "framework/vocabulary.hpp"

// Continuous state, integrated by an explicit Runge-Kutta method. Opt in by
// scheduling a Continuous element; a simulation that does not pays nothing.
//
//   using Dynamics = Continuous<RungeKutta4, TypeList<Kinematics>,
//                               SystemList<Gravity, EquationsOfMotion>>;
//   using Systems = SystemList<FlightControl, Dynamics, CheckOutcome>;
//
// The derivative systems are ordinary systems that write rate components. For
// each stage of the method, the integrator sets every entity's state to the
// stage's trial state, in place, runs the derivative systems at the stage's
// time, and keeps the rates. It then advances every state from where it began
// the step by the method's weighted rates.
namespace simon::framework {

// A component that changes continuously along its rate component. The rate
// combines linearly; `advance` moves a state along a rate for a duration, and
// may renormalize a state that is not a vector space, such as an attitude.
template <typename StateType>
concept ContinuousState = requires(
    const StateType& state, const typename StateType::RateComponent& rate,
    double weight, Duration dt) {
  { advance(state, rate, dt) } -> std::same_as<StateType>;
  { rate + rate } -> std::convertible_to<typename StateType::RateComponent>;
  { weight * rate } -> std::convertible_to<typename StateType::RateComponent>;
};

template <typename StateType>
using rate_of_t = typename StateType::RateComponent;

//-- Methods: explicit Runge-Kutta, as Butcher tableaus ------------------------

// Forward Euler: one stage, first order.
struct Euler final {
  static constexpr std::size_t stages = 1;
  static constexpr std::array<std::array<double, 1>, 1> a{{{0.0}}};
  static constexpr std::array<double, 1> b{1.0};
  static constexpr std::array<double, 1> c{0.0};
};

// The explicit midpoint method: two stages, second order. Exact for a
// constant second derivative, as model::integrate_midpoint is.
struct Midpoint final {
  static constexpr std::size_t stages = 2;
  static constexpr std::array<std::array<double, 2>, 2> a{{
      {0.0, 0.0},
      {0.5, 0.0},
  }};
  static constexpr std::array<double, 2> b{0.0, 1.0};
  static constexpr std::array<double, 2> c{0.0, 0.5};
};

// The classic Runge-Kutta method: four stages, fourth order.
struct RungeKutta4 final {
  static constexpr std::size_t stages = 4;
  static constexpr std::array<std::array<double, 4>, 4> a{{
      {0.0, 0.0, 0.0, 0.0},
      {0.5, 0.0, 0.0, 0.0},
      {0.0, 0.5, 0.0, 0.0},
      {0.0, 0.0, 1.0, 0.0},
  }};
  static constexpr std::array<double, 4> b{1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0,
                                           1.0 / 6.0};
  static constexpr std::array<double, 4> c{0.0, 0.5, 0.5, 1.0};
};

//-- The integrator ------------------------------------------------------------

// Walks continuous state in place. The only code besides the scheduler that
// may write a store directly.
struct ContinuousRunner final {
  // Calls `visit(n, state, rate)` for each entity with `StateType`, where `n`
  // counts entities in walk order and `rate` is null if the entity has none.
  // The order is the same on every call until the next sync.
  template <typename StateType, typename WorldType, typename VisitorType>
  static auto walk(InOut<WorldType> world, VisitorType&& visit) -> void {
    using RateType = rate_of_t<StateType>;
    auto& states = world->template mutable_store_of<StateType>(SchedulerKey{});
    const auto& rates = std::as_const(*world).template store_of<RateType>();

    // Archetypes that require the state, segment by segment.
    std::size_t n = 0;
    [&]<std::size_t... ARCHETYPES>(std::index_sequence<ARCHETYPES...>) {
      (walk_archetype<StateType, WorldType, ARCHETYPES>(states, rates, n,
                                                        visit),
       ...);
    }(std::make_index_sequence<WorldType::ArchetypeList::size>{});

    // Entities whose archetype only allows the state look up their rate.
    std::size_t allowed = states.segments() - 1;
    for (std::size_t ordinal = 0; ordinal < states.chunks_in(allowed);
         ++ordinal) {
      auto chunk = states.chunk(allowed, ordinal);
      for (std::size_t i = 0; i < chunk.size; ++i) {
        visit(n++, chunk.components[i],
              rates.maybe_component_of(chunk.owners[i]));
      }
    }
  }

 private:
  template <typename StateType, typename WorldType, std::size_t ARCHETYPE,
            typename StateStoreType, typename RateStoreType,
            typename VisitorType>
  static auto walk_archetype(StateStoreType& states, const RateStoreType& rates,
                             std::size_t& n, VisitorType& visit) -> void {
    using RateType = rate_of_t<StateType>;

    // An archetype that cannot have the rate is not integrated here: it
    // advances its state some other way, such as a cheaper single pass.
    if constexpr (WorldType::template archetype_requires<StateType>(
                      ARCHETYPE) &&
                  WorldType::template archetype_permits<RateType>(ARCHETYPE)) {
      static_assert(
          WorldType::template archetype_requires<RateType>(ARCHETYPE),
          "An archetype that requires a continuous state and allows its rate "
          "must require the rate.");

      // The state and its rate sit at the same slot of matching chunks.
      constexpr std::size_t STATE_SEGMENT =
          WorldType::template segment_of<StateType>(ARCHETYPE);
      constexpr std::size_t RATE_SEGMENT =
          WorldType::template segment_of<RateType>(ARCHETYPE);
      CHECK_INVARIANT(states.segment_size(STATE_SEGMENT) ==
                      rates.segment_size(RATE_SEGMENT));

      for (std::size_t ordinal = 0; ordinal < states.chunks_in(STATE_SEGMENT);
           ++ordinal) {
        auto state_chunk = states.chunk(STATE_SEGMENT, ordinal);
        auto rate_chunk = rates.chunk(RATE_SEGMENT, ordinal);
        for (std::size_t i = 0; i < state_chunk.size; ++i) {
          visit(n++, state_chunk.components[i], &rate_chunk.components[i]);
        }
      }
    }
  }
};

namespace internal {

// What a Continuous element declares, from its states and derivative systems:
// it writes the states and whatever its systems write, and reads the rest of
// what they read.
template <typename StateListType, typename SystemListType>
struct ContinuousDeclared;

template <typename... StateTypes, typename... SystemTypes>
struct ContinuousDeclared<TypeList<StateTypes...>, TypeList<SystemTypes...>>
    final {
  using WriteList = unique_t<
      concatenate_t<TypeList<StateTypes...>, write_list_of_t<SystemTypes>...>>;
  using ReadList =
      without_t<unique_t<concatenate_t<read_list_of_t<SystemTypes>...>>,
                WriteList>;
  using DeclaredList =
      concatenate_t<WriteList, map_t<ReadList, std::add_const_t>>;

  using AllowList =
      unique_t<concatenate_t<allow_component_list_of_t<SystemTypes>...>>;

  static constexpr bool writes_no_state =
      (!intersects_v<write_list_of_t<SystemTypes>, TypeList<StateTypes...>> &&
       ...);
};

template <typename ListType>
struct Split;

template <typename FirstType, typename... RestTypes>
struct Split<TypeList<FirstType, RestTypes...>> final {
  using First = FirstType;
  using Rest = TypeList<RestTypes...>;
};

}  // namespace internal

// A schedule element that integrates the continuous components in
// `StateListType` with `MethodType`, using the rates the systems in
// `DerivativeScheduleType` write. To the rest of the schedule it is one entry
// that writes the states and everything its derivative systems write.
//
// Derivative systems run once per stage, so they must give the same result
// each time they run for the same state: they do not write their own
// members, and they plan no structural changes (a contract check fails if
// one does). A derivative system that queries space sees the stage's state,
// and rebuilds the spatial index once per stage to do so.
template <typename MethodType, typename StateListType,
          typename DerivativeScheduleType>
class Continuous;

template <typename MethodType, typename... StateTypes,
          typename DerivativeScheduleType>
class Continuous<MethodType, TypeList<StateTypes...>, DerivativeScheduleType>
    final {
  using DerivativeSystemList = flattened_list_t<DerivativeScheduleType>;
  using Declared = internal::ContinuousDeclared<TypeList<StateTypes...>,
                                                DerivativeSystemList>;

  static_assert(sizeof...(StateTypes) > 0,
                "A Continuous element integrates at least one state.");
  static_assert((ContinuousState<StateTypes> && ...),
                "Each state names its RateComponent and supports advance(), "
                "rate + rate and weight * rate.");
  static_assert(is_unique_v<TypeList<StateTypes...>>,
                "A Continuous element lists each state once.");
  static_assert(is_valid_schedule_v<DerivativeScheduleType>,
                "Derivative systems are listed once each, in an order that "
                "satisfies their SequenceAfterSystemLists.");
  static_assert(Declared::writes_no_state,
                "Only the integrator writes continuous state; a derivative "
                "system writes rates.");

 public:
  // What the element writes (the states, and what its derivative systems
  // write) and reads, in the form a system declares them, for the schedule's
  // checks and describe().
  using DrivingComponent =
      typename internal::Split<typename Declared::DeclaredList>::First;
  using OtherComponentList =
      typename internal::Split<typename Declared::DeclaredList>::Rest;
  using AllowComponentList = typename Declared::AllowList;
  using ExcludeComponentList = TypeList<>;
  using SequenceAfterSystemList = SystemList<>;

  Continuous()
    requires std::is_default_constructible_v<
                 typename TupleOf<DerivativeSystemList>::type>
  = default;
  explicit Continuous(DerivativeScheduleType derivatives)
      : systems_{flatten_systems(std::move(derivatives))} {}

  // A derivative system, by type.
  template <typename SystemType>
  auto system() -> SystemType& {
    return std::get<SystemType>(systems_);
  }

  // Advances every state by `step.dt`.
  template <typename WorldType>
  auto run(const Step& step, InOut<WorldType> world) -> void {
    static_assert(
        (contains_v<typename WorldType::ComponentList, StateTypes> && ...),
        "A continuous state is not in the world.");
    static_assert(
        (contains_v<typename WorldType::ComponentList, rate_of_t<StateTypes>> &&
         ...),
        "A continuous state's rate component is not in the world.");

    if constexpr (STAGES == 1) {
      // One stage evaluates at the step's start state, so nothing is copied.
      derive(stage_step<0>(step), world);
      (advance_each<StateTypes>(step.dt, world), ...);
    } else {
      (keep_start<StateTypes>(world), ...);

      [&]<std::size_t... STAGE>(std::index_sequence<STAGE...>) {
        (stage<STAGE>(step, world), ...);
      }(std::make_index_sequence<STAGES>{});
    }
  }

 private:
  static constexpr std::size_t STAGES = MethodType::stages;

  // Where each state began the step, and its rate at each stage, by walk
  // order. Unused, and never allocated, by one-stage methods.
  template <typename StateType>
  struct Scratch final {
    std::vector<StateType> start;
    std::array<std::vector<rate_of_t<StateType>>, STAGES> rates;
  };

  template <std::size_t STAGE>
  static auto stage_step(const Step& step) -> Step {
    constexpr double C = MethodType::c[STAGE];
    if constexpr (C == 0.0) {
      return step;
    } else {
      auto offset = std::chrono::duration_cast<Duration>(
          std::chrono::duration<double, Duration::period>(
              C * static_cast<double>(step.dt.count())));
      return Step{.time = step.time + offset, .dt = step.dt};
    }
  }

  // Runs the derivative systems once, with no sync point after any of them.
  template <typename WorldType>
  auto derive(const Step& step, InOut<WorldType> world) -> void {
    std::size_t pending = world->pending();

    std::apply(
        [&](auto&... system) {
          (SystemRunner::run(step, InOut(system), world), ...);
        },
        systems_);

    CHECK_INVARIANT(world->pending() == pending);  // No structural changes.
  }

  template <typename StateType, typename WorldType>
  static auto advance_each(Duration dt, InOut<WorldType> world) -> void {
    ContinuousRunner::walk<StateType>(
        world,
        [&](std::size_t, StateType& state, const rate_of_t<StateType>* rate) {
          if (rate) {
            state = advance(state, *rate, dt);
          }
        });
  }

  template <typename StateType, typename WorldType>
  auto keep_start(InOut<WorldType> world) -> void {
    Scratch<StateType>& scratch = std::get<Scratch<StateType>>(scratch_);

    // Sized once, to the store's capacity, on the first step.
    std::size_t capacity = world->template store_of<StateType>().capacity();
    if (scratch.start.size() < capacity) {
      scratch.start.resize(capacity);
      for (auto& rates : scratch.rates) {
        rates.resize(capacity);
      }
    }

    ContinuousRunner::walk<StateType>(
        world, [&](std::size_t n, StateType& state,
                   const rate_of_t<StateType>*) { scratch.start[n] = state; });
  }

  // The rates of the first `COUNT` stages, weighted by `weights`.
  template <typename StateType, std::size_t COUNT, std::size_t SIZE>
  static auto weighted(const Scratch<StateType>& scratch, std::size_t n,
                       const std::array<double, SIZE>& weights)
      -> rate_of_t<StateType> {
    rate_of_t<StateType> sum = weights[0] * scratch.rates[0][n];
    for (std::size_t j = 1; j < COUNT; ++j) {
      if (weights[j] != 0.0) {
        sum = sum + weights[j] * scratch.rates[j][n];
      }
    }
    return sum;
  }

  template <std::size_t STAGE, typename WorldType>
  auto stage(const Step& step, InOut<WorldType> world) -> void {
    derive(stage_step<STAGE>(step), world);
    (after_stage<StateTypes, STAGE>(step.dt, world), ...);
  }

  // Keeps each entity's rate for `STAGE`, then moves its state to the next
  // stage's trial state, or past the last stage to the end of the step. One
  // pass over the state per stage.
  template <typename StateType, std::size_t STAGE, typename WorldType>
  auto after_stage(Duration dt, InOut<WorldType> world) -> void {
    Scratch<StateType>& scratch = std::get<Scratch<StateType>>(scratch_);
    ContinuousRunner::walk<StateType>(
        world,
        [&](std::size_t n, StateType& state, const rate_of_t<StateType>* rate) {
          if (!rate) {
            return;
          }

          scratch.rates[STAGE][n] = *rate;

          if constexpr (STAGE + 1 < STAGES) {
            state = advance(scratch.start[n],
                            weighted<StateType, STAGE + 1>(
                                scratch, n, MethodType::a[STAGE + 1]),
                            dt);
          } else {
            state = advance(
                scratch.start[n],
                weighted<StateType, STAGES>(scratch, n, MethodType::b), dt);
          }
        });
  }

  typename TupleOf<DerivativeSystemList>::type systems_;
  std::tuple<Scratch<StateTypes>...> scratch_;
};

}  // namespace simon::framework
