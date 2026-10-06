// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows MuJoCo 3.14.0, Copyright 2021 DeepMind Technologies Limited,
// Apache-2.0; translated to C++ and changed. See NOTICE.md.

#pragma once

#include <array>
#include <cstddef>
#include <expected>
#include <string>
#include <string_view>

#include "base/status.hpp"
#include "model/articulated.hpp"

// Reads articulated models from MJCF, MuJoCo's XML format (see
// model/REFERENCES.md), and compiles them as MuJoCo's compiler does: default
// classes and childclass, every orientation, fromto shapes, each geom's mass
// and inertia from its shape and density, a body's inertia combined from its
// geoms on principal axes, angles in degrees by default, and limits from
// ranges. Anything that would change how the model moves and that simon does
// not yet run is refused, tendons, equalities, meshes, contact pairs and
// actuators other than motors among them, so that a model never runs other
// than as written; what only shows a model, its visuals, assets, lights,
// cameras, sites and sensors, is left out.
namespace simon::format {

// The reasons a model could not be read.
enum class MjcfError {
  UNREADABLE,   // A file could not be opened.
  MALFORMED,    // The message says where and how.
  UNSUPPORTED,  // An element or attribute simon does not run; the message
                // names it.
  COUNT,
};

inline constexpr std::size_t MJCF_ERROR_COUNT =
    static_cast<std::size_t>(MjcfError::COUNT);

// Reads and compiles the model that `text` describes.
auto parse_mjcf(std::string_view text)
    -> std::expected<model::ArticulatedModel, lib::Status>;

// Reads and compiles the model in the file at `path`.
auto load_mjcf(const std::string& path)
    -> std::expected<model::ArticulatedModel, lib::Status>;

}  // namespace simon::format

// Messages for each MjcfError, defined in mjcf.cpp.
template <>
const std::array<lib::StatusConditionEntry, simon::format::MJCF_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::format::MjcfError, simon::format::MJCF_ERROR_COUNT>::conditions_;
