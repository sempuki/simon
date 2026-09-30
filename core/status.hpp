// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstddef>

#include "base/status.hpp"

namespace simon::core {

using lib::Status;

// Why a builder refused an utterance. `build()` returns one of these as a
// Status and emits nothing. Compare with `status == lib::watch(condition)`.
enum class BuildCondition {
  ENTITY_CAPACITY_EXHAUSTED,
  COMPONENT_CAPACITY_EXHAUSTED,
  ENTITY_NOT_ALIVE,
  COMPONENT_ALREADY_ATTACHED,
  COMPONENT_NOT_ATTACHED,
  ALIAS_INVALID,
  ALIAS_ALREADY_GIVEN,
  ALIAS_NOT_GIVEN,
  COUNT,
};

inline constexpr std::size_t BUILD_CONDITION_COUNT =
    static_cast<std::size_t>(BuildCondition::COUNT);

}  // namespace simon::core

// Messages for each condition, defined in status.cpp.
template <>
const std::array<lib::StatusConditionEntry, simon::core::BUILD_CONDITION_COUNT>
    lib::EnumStatusKindConditionMixin<simon::core::BuildCondition,
                                      simon::core::BUILD_CONDITION_COUNT>::conditions_;
