// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstddef>
#include <expected>
#include <string>
#include <string_view>

#include "base/status.hpp"
#include "model/tire.hpp"

// Reads a Magic Formula 5.2 tire from a TNO tire property file (.tir), as
// MF-Tyre, ADAMS and Project Chrono write them for the PAC2002 format (see
// model/REFERENCES.md): sections in brackets, each line KEY = value, and $
// and ! starting comments. The steady-state coefficients are read; the
// vertical, transient, rolling and overturning ones, tables and shapes are
// left out. A file of another format is refused.
namespace simon::format {

// The reasons a tire could not be read.
enum class TireFileError {
  UNREADABLE,  // The file could not be opened.
  MALFORMED,   // The message says where and how.
  COUNT,
};

inline constexpr std::size_t TIRE_FILE_ERROR_COUNT =
    static_cast<std::size_t>(TireFileError::COUNT);

// Reads the tire that `text` describes.
auto parse_tire_file(std::string_view text)
    -> std::expected<model::MagicFormulaTire, lib::Status>;

// Reads the tire in the file at `path`.
auto load_tire_file(const std::string& path)
    -> std::expected<model::MagicFormulaTire, lib::Status>;

}  // namespace simon::format

// Messages for each TireFileError, defined in tire_file.cpp.
template <>
const std::array<lib::StatusConditionEntry,
                 simon::format::TIRE_FILE_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::format::TireFileError,
        simon::format::TIRE_FILE_ERROR_COUNT>::conditions_;
