// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "format/format_error.hpp"

template <>
const std::array<lib::StatusConditionEntry, simon::format::FORMAT_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::format::FormatError,
        simon::format::FORMAT_ERROR_COUNT>::conditions_ = {
        lib::StatusConditionEntry{"file unreadable"},
        lib::StatusConditionEntry{"file malformed"},
        lib::StatusConditionEntry{"file unsupported"},
};
