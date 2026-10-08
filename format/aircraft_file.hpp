// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <expected>
#include <string>
#include <string_view>

#include "base/status.hpp"
#include "format/format_error.hpp"  // IWYU pragma: export
#include "model/aircraft/definition.hpp"

// Reads an aircraft from the file tools/jsbsim/convert.py writes from a
// JSBSim aircraft (see model/aircraft/definition.hpp). A failure is a
// FormatError.
namespace simon::format {

// Reads the aircraft that `text` describes.
auto parse_aircraft(std::string_view text)
    -> std::expected<aircraft::Definition, lib::Status>;

// Reads the aircraft in the file at `path`.
auto load_aircraft(const std::string& path)
    -> std::expected<aircraft::Definition, lib::Status>;

}  // namespace simon::format
