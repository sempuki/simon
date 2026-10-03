// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstddef>
#include <expected>
#include <string>
#include <string_view>

#include "base/status.hpp"
#include "model/road.hpp"

// Reads roads from ASAM OpenDRIVE files (see model/REFERENCES.md): each
// road's reference line (lines, arcs, spirals and parametric cubics), its
// elevation and superelevation, its lane offset, and its lane sections with
// their lanes' widths, and how roads, lanes and junctions link. Those are what
// the roads' geometry and the lanes' connections need. A road's
// lateral shape, lane heights, road marks, objects and signals are left out.
// The deprecated poly3 geometry and lanes given by their borders rather than
// their widths are refused, as libOpenDRIVE refuses them.
namespace simon::model {

// The reasons a road network could not be read.
enum class OpenDriveError {
  UNREADABLE,  // The file could not be opened.
  MALFORMED,   // The message says where and how.
  COUNT,
};

inline constexpr std::size_t OPEN_DRIVE_ERROR_COUNT =
    static_cast<std::size_t>(OpenDriveError::COUNT);

// Reads the road network that `text` describes.
auto parse_opendrive(std::string_view text)
    -> std::expected<RoadNetwork, lib::Status>;

// Reads the road network in the file at `path`.
auto load_opendrive(const std::string& path)
    -> std::expected<RoadNetwork, lib::Status>;

}  // namespace simon::model

// Messages for each OpenDriveError, defined in opendrive.cpp.
template <>
const std::array<lib::StatusConditionEntry,
                 simon::model::OPEN_DRIVE_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::model::OpenDriveError,
        simon::model::OPEN_DRIVE_ERROR_COUNT>::conditions_;
