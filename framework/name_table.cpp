// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "framework/name_table.hpp"

#include <algorithm>
#include <limits>

#include "base/core.hpp"

namespace simon::framework {

auto NameTable::reset(std::size_t entities) -> void {
  next_entity_instance_ = 0;
  instance_of_index_.assign(entities, 0);
  entity_of_instance_.clear();
  next_archetype_instance_ = 0;
  archetypes_.clear();
  aliases_.clear();
  aliases_of_name_.clear();
}

auto NameTable::name_entity(Entity entity) -> std::uint32_t {
  CHECK_PRECONDITION(next_entity_instance_ <
                     std::numeric_limits<std::uint32_t>::max());
  std::uint32_t instance = next_entity_instance_++;
  instance_of_index_[entity.index] = instance;
  entity_of_instance_.emplace(instance, entity);
  return instance;
}

auto NameTable::forget_entity(std::uint32_t instance) -> void {
  entity_of_instance_.erase(instance);
}

auto NameTable::entity_of(std::uint32_t instance) const
    -> std::optional<Entity> {
  auto iter = entity_of_instance_.find(instance);
  return iter != entity_of_instance_.end() ? std::optional{iter->second}
                                           : std::nullopt;
}

auto NameTable::name_archetype(std::string_view name) -> Name {
  auto [iter, inserted] = archetypes_.try_emplace(std::string{name});
  if (inserted) {
    iter->second = Name{Kind::ARCHETYPE, next_archetype_instance_++};
    give_alias(iter->second, Alias{name});
  }
  return iter->second;
}

auto NameTable::give_alias(Name name, const Alias& alias) -> void {
  aliases_.emplace(alias, name);
  aliases_of_name_[name].push_back(alias);
}

auto NameTable::take_alias(Name name, const Alias& alias) -> void {
  auto [begin, end] = aliases_.equal_range(alias);
  for (auto iter = begin; iter != end; ++iter) {
    if (iter->second == name) {
      aliases_.erase(iter);
      break;
    }
  }
  std::vector<Alias>& given = aliases_of_name_[name];
  std::erase(given, alias);
  if (given.empty()) {
    aliases_of_name_.erase(name);
  }
}

auto NameTable::has_alias(Name name, const Alias& alias) const -> bool {
  auto iter = aliases_of_name_.find(name);
  return iter != aliases_of_name_.end() &&
         std::ranges::contains(iter->second, alias);
}

auto NameTable::find_names(const Alias& alias) const -> std::vector<Name> {
  std::vector<Name> names;
  auto [begin, end] = aliases_.equal_range(alias);
  for (auto iter = begin; iter != end; ++iter) {
    names.push_back(iter->second);
  }
  return names;
}

auto NameTable::aliases_of(Name name) const -> std::vector<Alias> {
  auto iter = aliases_of_name_.find(name);
  return iter != aliases_of_name_.end() ? iter->second : std::vector<Alias>{};
}

}  // namespace simon::framework
