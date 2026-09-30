// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <cstddef>
#include <string_view>

#include "core/entity.hpp"
#include "core/name.hpp"
#include "core/type_list.hpp"

namespace simon::core {

// A string usable as a template argument: Archetype<"ball", ...>.
template <std::size_t Size>
struct FixedString {
  char value[Size] = {};

  constexpr FixedString(const char (&text)[Size]) { std::copy_n(text, Size, value); }
  constexpr std::string_view view() const { return {value, Size - 1}; }
};

template <typename... Components>
struct Requires {};

template <typename... Components>
struct Allows {};

// An archetype: what sort of object an entity is, and which components it must
// and may have. Every entity is created from one, and the archetype is recorded
// on the entity, so an IO layer can map it to a foreign object type.
//
//   struct Ball : Archetype<"ball", Requires<Kinematics, Collider>,
//                           Allows<Thrust, Drag>> {};
//   world.create<Ball>("my-fav-ball").with(Kinematics{}).with(Collider{}).build();
template <FixedString ArchetypeName, typename RequiresType = Requires<>,
          typename AllowsType = Allows<>>
struct Archetype;

template <FixedString ArchetypeName, typename... Required, typename... Allowed>
struct Archetype<ArchetypeName, Requires<Required...>, Allows<Allowed...>> {
  static constexpr std::string_view name = ArchetypeName.view();
  using RequiredComponents = TypeList<Required...>;
  using PermittedComponents = TypeList<Required..., Allowed...>;
};

template <typename Type>
concept ArchetypeType = requires {
  { Type::name } -> std::convertible_to<std::string_view>;
  typename Type::RequiredComponents;
  typename Type::PermittedComponents;
};

// Built-in components every world has.

// The archetype an entity was created from. Every entity has one.
struct EntityArchetype {
  Name archetype;
};

// The entity an entity was created under (`create<...>().under(parent)`). The
// relation goes stale, and lookups through it fail, when the parent is destroyed.
struct Parent {
  Entity entity;
};

}  // namespace simon::core
