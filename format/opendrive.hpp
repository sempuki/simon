// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <expected>
#include <string>
#include <string_view>

#include "base/status.hpp"
#include "format/format_error.hpp"  // IWYU pragma: export
#include "model/road/road.hpp"

// Reads roads from ASAM OpenDRIVE files (see model/REFERENCES.md): each
// road's reference line (lines, arcs, spirals and parametric cubics), its
// elevation and superelevation, its lane offset, and its lane sections with
// their lanes' widths, and how roads, lanes and junctions link. Those are what
// the roads' geometry and the lanes' connections need. Each road's signals
// and objects, with their outlines and the lanes they hold for, junctions'
// priorities and controllers, and the controllers that group signals are read
// for traffic control. A road's lateral shape, lane heights, road marks,
// signal references and repeated objects are left out. The deprecated poly3
// geometry and lanes given by their borders rather than their widths are
// refused, as libOpenDRIVE refuses them. A failure is a FormatError.
namespace simon::format {

// Reads the road network that `text` describes.
auto parse_opendrive(std::string_view text)
    -> std::expected<model::RoadNetwork, lib::Status>;

// Reads the road network in the file at `path`.
auto load_opendrive(const std::string& path)
    -> std::expected<model::RoadNetwork, lib::Status>;

}  // namespace simon::format
