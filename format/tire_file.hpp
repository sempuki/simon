// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <expected>
#include <string>
#include <string_view>

#include "base/status.hpp"
#include "format/format_error.hpp"  // IWYU pragma: export
#include "model/vehicle/tire.hpp"

// Reads a Magic Formula 5.2 tire from a TNO tire property file (.tir), as
// MF-Tyre, ADAMS and Project Chrono write them for the PAC2002 format (see
// model/REFERENCES.md): sections in brackets, each line KEY = value, and $
// and ! starting comments. The steady-state coefficients are read; the
// vertical, transient, rolling and overturning ones, tables and shapes are
// left out. A file of another format is refused. A failure is a FormatError.
namespace simon::format {

// Reads the tire that `text` describes.
auto parse_tire_file(std::string_view text)
    -> std::expected<vehicle::MagicFormulaTire, lib::Status>;

// Reads the tire in the file at `path`.
auto load_tire_file(const std::string& path)
    -> std::expected<vehicle::MagicFormulaTire, lib::Status>;

}  // namespace simon::format
