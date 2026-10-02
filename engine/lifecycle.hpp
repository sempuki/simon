// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <concepts>
#include <expected>

#include "base/status.hpp"
#include "framework/step.hpp"

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
template <typename Type>
concept Simulation = requires(Type simulation, const framework::Step& step) {
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
