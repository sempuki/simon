// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <string_view>

#include "framework/entity.hpp"
#include "framework/name.hpp"
#include "framework/type_list.hpp"

namespace simon::framework {

// A string usable as a template argument: Archetype<"ball", ...>.
template <std::size_t Size>
struct FixedString final {
  std::array<char, Size> value{};

  // A C-array reference, so a string literal binds and deduces Size.
  constexpr FixedString(const char (&text)[Size]) {
    std::copy_n(text, Size, value.begin());
  }
  constexpr auto view() const -> std::string_view {
    return {value.data(), Size - 1};
  }
};

template <typename... ComponentTypes>
struct Requires final {};

template <typename... ComponentTypes>
struct Allows final {};

// An archetype: what sort of object an entity is, and which components it must
// and may have. Every entity is created from one, and the archetype is recorded
// on the entity, so an IO layer can map it to a foreign object type.
//
//   struct Ball : Archetype<"ball", Requires<Kinematics, Collider>,
//                           Allows<Thrust, Drag>> {};
//   world.create<Ball>("my-fav-ball").with(Kinematics{}).with(Collider{}).build();
template <FixedString ArchetypeName,           //
          typename RequiresType = Requires<>,  //
          typename AllowsType = Allows<>>
struct Archetype;

template <FixedString ArchetypeName,          //
          typename... RequiredTypes,          //
          typename... AllowedTypes>           //
struct Archetype<ArchetypeName,               //
                 Requires<RequiredTypes...>,  //
                 Allows<AllowedTypes...>> {
  static constexpr std::string_view name = ArchetypeName.view();
  using RequiredComponentList = TypeList<RequiredTypes...>;
  using PermittedComponentList = TypeList<RequiredTypes..., AllowedTypes...>;
};

template <typename Type>
concept Archetypal = requires {
  { Type::name } -> std::convertible_to<std::string_view>;
  typename Type::RequiredComponentList;
  typename Type::PermittedComponentList;
};

// Built-in components every world has.

// The archetype an entity was created from. Every entity has one.
struct EntityArchetype final {
  Name archetype;
};

// The entity an entity was created under (`create<...>().under(parent)`). The
// relation goes stale, and lookups through it fail, when the parent is
// destroyed.
struct Parent final {
  Entity entity;
};

}  // namespace simon::framework
