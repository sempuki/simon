// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstddef>
#include <expected>
#include <optional>
#include <string>
#include <string_view>

#include "base/status.hpp"
#include "format/format_error.hpp"

// What every reader does with text: reads a file whole, trims, parses a
// number the same way, and says on which line something is wrong.
namespace simon::format {

// The whole of the file at `path`; UNREADABLE if it cannot be opened.
auto read_text_file(const std::string& path)
    -> std::expected<std::string, lib::Status>;

// `text` without the blanks either side.
auto trim(std::string_view text) -> std::string_view;

// The finite number `text` holds, blanks either side and a leading `+`
// allowed, or none if it holds anything else.
auto parse_number(std::string_view text) -> std::optional<double>;

// The line that `offset` into `text` falls on, counted from 1.
auto find_line(std::string_view text, std::ptrdiff_t offset) -> std::size_t;

// A MALFORMED failure: "line N: why".
auto fail_at(std::size_t line, std::string_view why) -> Failure;

// An UNSUPPORTED failure naming `what`.
auto refuse(std::string_view what) -> Failure;

}  // namespace simon::format
