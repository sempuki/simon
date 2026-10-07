// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows MuJoCo 3.14.0, Copyright 2021 DeepMind Technologies Limited,
// Apache-2.0; translated to C++ and changed. See NOTICE.md.

#pragma once

#include <expected>
#include <string>
#include <string_view>

#include "base/status.hpp"
#include "format/format_error.hpp"  // IWYU pragma: export
#include "model/articulated/articulated.hpp"

// Reads articulated models from MJCF, MuJoCo's XML format (see
// model/REFERENCES.md), and compiles them as MuJoCo's compiler does: default
// classes and childclass, every orientation, fromto shapes, each geom's mass
// and inertia from its shape and density, a body's inertia combined from its
// geoms on principal axes, angles in degrees by default, and limits from
// ranges. Anything that would change how the model moves and that simon does
// not yet run is refused, tendons, equalities, meshes, contact pairs and
// actuators other than motors among them, so that a model never runs other
// than as written; what only shows a model, its visuals, assets, lights,
// cameras, sites and sensors, is left out. A failure is a FormatError: what
// is wrong in the text says its line, and what is wrong in the compiled
// model names its body or joint.
namespace simon::format {

// Reads and compiles the model that `text` describes.
auto parse_mjcf(std::string_view text)
    -> std::expected<articulated::Scene, lib::Status>;

// Reads and compiles the model in the file at `path`, its includes expanded
// in place, so that a failure's line counts in the expanded text.
auto load_mjcf(const std::string& path)
    -> std::expected<articulated::Scene, lib::Status>;

}  // namespace simon::format
