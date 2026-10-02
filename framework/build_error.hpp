// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstddef>

#include "base/status.hpp"

namespace simon::framework {

using lib::Status;

// The reasons a builder refuses an utterance. `build()` returns one of these
// as a Status and emits nothing. Compare with `status == lib::watch(error)`.
enum class BuildError {
  ENTITY_CAPACITY_EXHAUSTED,
  COMPONENT_CAPACITY_EXHAUSTED,
  ENTITY_NOT_ALIVE,
  COMPONENT_ALREADY_ATTACHED,
  COMPONENT_NOT_ATTACHED,
  COMPONENT_NOT_PERMITTED,  // Archetype neither requires nor allows it.
  COMPONENT_REQUIRED,       // The entity's archetype requires it.
  ALIAS_INVALID,
  ALIAS_ALREADY_GIVEN,
  ALIAS_NOT_GIVEN,
  CELL_SIZE_INVALID,   // A world's spatial index cell size is not positive.
  CAPACITY_TOO_LARGE,  // A world holds more than a store can index.
  COUNT,
};

inline constexpr std::size_t BUILD_ERROR_COUNT =
    static_cast<std::size_t>(BuildError::COUNT);

}  // namespace simon::framework

// Messages for each BuildError, defined in build_error.cpp.
template <>
const std::array<lib::StatusConditionEntry, simon::framework::BUILD_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::framework::BuildError,
        simon::framework::BUILD_ERROR_COUNT>::conditions_;
