// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <concepts>
#include <expected>

#include "base/status.hpp"
#include "core/time.hpp"

namespace simon::engine {

using lib::Status;

// Whether a simulation wants to keep going. Errors are Statuses, never Flow.
enum class Flow { CONTINUE, STOP };

using PhaseResult = std::expected<Flow, Status>;
using FinishResult = std::expected<void, Status>;

// Every simulation follows one lifecycle, whatever drives it:
//
//   configure -> initialize -> step -> step -> ... -> finalize
//
// `step` is required. `configure`, `initialize` and `finalize` are optional;
// a driver calls them when the simulation has them.
//
// A simulation counts time in nanoseconds unless it declares a coarser tick,
// as `using Tick = ...;`, and its steps are BasicSteps of that tick.
template <typename Type>
struct TickOf final {
  using type = Duration;
};
template <typename Type>
  requires requires { typename Type::Tick; }
struct TickOf<Type> final {
  using type = typename Type::Tick;
};
template <typename Type>
using tick_of_t = typename TickOf<Type>::type;

template <typename Type>
concept Simulation =
    requires(Type simulation, const BasicStep<tick_of_t<Type>>& step) {
      { simulation.step(step) } -> std::same_as<PhaseResult>;
    };

// A driver's phase in the lifecycle.
enum class Phase {
  NEW,       // Not yet configured and initialized.
  RUNNING,   // Stepping.
  STOPPED,   // A phase returned Flow::STOP or an error; no more steps.
  FINISHED,  // Finalized.
};

}  // namespace simon::engine
