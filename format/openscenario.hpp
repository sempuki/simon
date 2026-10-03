// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstddef>
#include <expected>
#include <string>
#include <string_view>
#include <vector>

#include "base/status.hpp"
#include "scenario/openscenario.hpp"

// Reads scenarios from ASAM OpenSCENARIO 1.x files (see model/REFERENCES.md)
// into scenario/openscenario's description. Parameters are substituted where
// they are referred to as $name, and expressions ${...} are evaluated, as 1.1
// defines them: numbers, parameters, + - * / %, parentheses and unary minus.
// Anything else that would change what happens is refused, so that a
// scenario never runs other than as written; what only shows a scenario, its
// scene graph and 3D models, is left out.
namespace simon::format {

// The reasons a scenario could not be read.
enum class ScenarioError {
  UNREADABLE,   // A file could not be opened.
  MALFORMED,    // The message says where and how.
  UNSUPPORTED,  // An element simon does not run; the message names it.
  COUNT,
};

inline constexpr std::size_t SCENARIO_ERROR_COUNT =
    static_cast<std::size_t>(ScenarioError::COUNT);

// Reads the scenario that `text` describes, its relative paths, to the road
// network and catalogs, from `directory`.
auto parse_openscenario(std::string_view text, const std::string& directory)
    -> std::expected<scenario::Scenario, lib::Status>;

// Reads the scenario in the file at `path`.
auto load_openscenario(const std::string& path)
    -> std::expected<scenario::Scenario, lib::Status>;

// The value of the expression `text` (the body of ${...}) with
// `parameters`' values.
auto evaluate_expression(std::string_view text,
                         const std::vector<scenario::Parameter>& parameters)
    -> std::expected<double, std::string>;

}  // namespace simon::format

// Messages for each ScenarioError, defined in openscenario.cpp.
template <>
const std::array<lib::StatusConditionEntry, simon::format::SCENARIO_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::format::ScenarioError,
        simon::format::SCENARIO_ERROR_COUNT>::conditions_;
