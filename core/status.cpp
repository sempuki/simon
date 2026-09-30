// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "core/status.hpp"

template <>
const std::array<lib::StatusConditionEntry, simon::core::BUILD_CONDITION_COUNT>
    lib::EnumStatusKindConditionMixin<simon::core::BuildCondition,
                                      simon::core::BUILD_CONDITION_COUNT>::conditions_ = {
        lib::StatusConditionEntry{"entity capacity exhausted"},
        lib::StatusConditionEntry{"component capacity exhausted"},
        lib::StatusConditionEntry{"entity not alive"},
        lib::StatusConditionEntry{"component already attached"},
        lib::StatusConditionEntry{"component not attached"},
        lib::StatusConditionEntry{"alias invalid"},
        lib::StatusConditionEntry{"alias already given"},
        lib::StatusConditionEntry{"alias not given"},
};
