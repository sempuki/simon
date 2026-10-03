// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "scenario/openscenario.hpp"

#include <algorithm>

namespace simon::scenario {

auto Scenario::find_entity(std::string_view name) const -> const Entity* {
  auto found = std::ranges::find(entities, name, &Entity::name);
  return found == entities.end() ? nullptr : &*found;
}

}  // namespace simon::scenario
