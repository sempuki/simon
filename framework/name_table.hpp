// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "framework/entity.hpp"
#include "framework/name.hpp"

// A world's names: each entity's instance, numbered as entities are created
// and never reused, the archetypes' names, and aliases, many to many. None of
// it depends on a world's components or archetypes, so it is compiled once
// rather than for every world type.
namespace simon::framework {

class NameTable final {
 public:
  // Forgets every name, with room for `entities` entity indices.
  auto reset(std::size_t entities) -> void;

  // Names a newly created `entity`, and returns its instance.
  auto name_entity(Entity entity) -> std::uint32_t;

  // Forgets the entity with `instance`.
  auto forget_entity(std::uint32_t instance) -> void;

  // The instance of the live entity at `entity`'s index.
  auto instance_of(Entity entity) const -> std::uint32_t {
    return instance_of_index_[entity.index];
  }

  // The live entity with `instance`.
  auto entity_of(std::uint32_t instance) const -> std::optional<Entity>;

  // The name of the archetype called `name`, named, and given that alias,
  // the first time it is asked for.
  auto name_archetype(std::string_view name) -> Name;

  // How many archetypes are named.
  auto archetype_count() const -> std::uint32_t {
    return next_archetype_instance_;
  }

  auto give_alias(Name name, const Alias& alias) -> void;
  auto take_alias(Name name, const Alias& alias) -> void;
  auto has_alias(Name name, const Alias& alias) const -> bool;

  // Every name with `alias`, in the order the aliases were given.
  auto find_names(const Alias& alias) const -> std::vector<Name>;

  // Every alias `name` has, in the order they were given.
  auto aliases_of(Name name) const -> std::vector<Alias>;

 private:
  std::uint32_t next_entity_instance_ = 0;
  std::vector<std::uint32_t> instance_of_index_;
  std::unordered_map<std::uint32_t, Entity> entity_of_instance_;
  std::uint32_t next_archetype_instance_ = 0;
  std::map<std::string, Name, std::less<>> archetypes_;
  // A multimap keeps equal aliases in the order given.
  std::multimap<Alias, Name> aliases_;
  std::unordered_map<Name, std::vector<Alias>> aliases_of_name_;
};

}  // namespace simon::framework
