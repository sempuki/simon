// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/builder.hpp"

template <>
const std::array<lib::StatusConditionEntry, simon::framework::BUILD_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::framework::BuildError,
        simon::framework::BUILD_ERROR_COUNT>::conditions_ = {
        lib::StatusConditionEntry{"entity capacity exhausted"},
        lib::StatusConditionEntry{"component capacity exhausted"},
        lib::StatusConditionEntry{"entity not alive"},
        lib::StatusConditionEntry{"component already attached"},
        lib::StatusConditionEntry{"component not attached"},
        lib::StatusConditionEntry{"component not permitted"},
        lib::StatusConditionEntry{"component required"},
        lib::StatusConditionEntry{"alias invalid"},
        lib::StatusConditionEntry{"alias already given"},
        lib::StatusConditionEntry{"alias not given"},
        lib::StatusConditionEntry{"cell size invalid"},
        lib::StatusConditionEntry{"capacity too large"},
};
