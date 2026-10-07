// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstddef>
#include <expected>

#include "base/status.hpp"

// What every reader in format/ says when it cannot read a file.
namespace simon::format {

// The reasons a file of any format could not be read.
enum class FormatError {
  UNREADABLE,   // The file could not be opened.
  MALFORMED,    // The message says on which line, and how.
  UNSUPPORTED,  // Something simon does not run; the message names it.
  COUNT,
};

inline constexpr std::size_t FORMAT_ERROR_COUNT =
    static_cast<std::size_t>(FormatError::COUNT);

// The error a reader returns.
using Failure = std::unexpected<lib::Status>;

}  // namespace simon::format

// Messages for each FormatError, defined in format_error.cpp.
template <>
const std::array<lib::StatusConditionEntry, simon::format::FORMAT_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::format::FormatError,
        simon::format::FORMAT_ERROR_COUNT>::conditions_;
