// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

// Numbers read from text, the same way everywhere: by file readers, by
// mains reading their arguments, and by tests reading reference tables.
namespace simon {

// `text` without the blanks either side.
auto trim(std::string_view text) -> std::string_view;

// The finite number `text` holds, blanks either side and a leading `+`
// allowed, or none if it holds anything else.
auto parse_number(std::string_view text) -> std::optional<double>;

// The whole number `text` holds, read as parse_number reads a number.
auto parse_integer(std::string_view text) -> std::optional<std::int64_t>;

}  // namespace simon
