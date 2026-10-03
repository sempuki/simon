// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstddef>
#include <expected>
#include <string>
#include <string_view>

#include "base/status.hpp"
#include "model/aircraft_data.hpp"

// Reads an aircraft from the file tools/jsbsim/convert.py writes from a
// JSBSim aircraft (see model/aircraft_data.hpp).
namespace simon::format {

// The reasons an aircraft could not be read.
enum class AircraftFileError {
  UNREADABLE,  // The file could not be opened.
  MALFORMED,   // The message says where and how.
  COUNT,
};

inline constexpr std::size_t AIRCRAFT_FILE_ERROR_COUNT =
    static_cast<std::size_t>(AircraftFileError::COUNT);

// Reads the aircraft that `text` describes.
auto parse_aircraft(std::string_view text)
    -> std::expected<model::AircraftData, lib::Status>;

// Reads the aircraft in the file at `path`.
auto load_aircraft(const std::string& path)
    -> std::expected<model::AircraftData, lib::Status>;

}  // namespace simon::format

// Messages for each AircraftFileError, defined in aircraft_file.cpp.
template <>
const std::array<lib::StatusConditionEntry,
                 simon::format::AIRCRAFT_FILE_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::format::AircraftFileError,
        simon::format::AIRCRAFT_FILE_ERROR_COUNT>::conditions_;
