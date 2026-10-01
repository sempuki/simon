// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <compare>
#include <concepts>
#include <cstdint>
#include <format>
#include <functional>
#include <optional>
#include <string>
#include <string_view>

// Everything in a world is known three ways:
//
//   Name      {kind, instance}: two integers. Compared in hot loops, written
//             to logs, sent across processes.
//   Identity  "/world/1/entity/2/component/3": the canonical string, for
//             debugging from a console. Computed from the Name.
//   Alias     "ego", "Luke Skywalker": meaning only people bring.
//             Many-to-many.
namespace simon::framework {

// What a name names. Each component type is its own kind for its
// entity-components: kind ENTITY_COMPONENT + c names entity-components of the
// world's component number c.
enum class Kind : std::uint32_t {
  NONE = 0,
  WORLD,
  ARCHETYPE,
  COMPONENT,
  SYSTEM,
  ENTITY,
  ENTITY_COMPONENT,
};

// A string that is one kind of thing and cannot be mistaken for another kind:
// an Identity is never an Alias. Converts implicitly from anything that
// converts to std::string_view, including string literals.
template <typename TagType>
class TaggedString final {
 public:
  TaggedString() = default;
  template <typename SourceType>
    requires std::convertible_to<const SourceType&, std::string_view>
  TaggedString(const SourceType& text)  // NOLINT(google-explicit-constructor)
      : value_{std::string_view{text}} {}

  std::string_view view() const { return value_; }
  const std::string& string() const { return value_; }
  bool empty() const { return value_.empty(); }

  friend auto operator<=>(const TaggedString&, const TaggedString&) = default;
  friend bool operator==(const TaggedString&, const TaggedString&) = default;

 private:
  std::string value_;
};

struct IdentityTag;
struct AliasTag;

// The canonical REST-like string for a Name, e.g. "/world/1/entity/2".
using Identity = TaggedString<IdentityTag>;

// A string with meaning only people bring, e.g. "ego" or "Luke Skywalker".
using Alias = TaggedString<AliasTag>;

struct Name final {
  std::uint32_t kind = static_cast<std::uint32_t>(Kind::NONE);
  std::uint32_t instance = 0;

  constexpr Name() = default;
  constexpr Name(Kind kind_of, std::uint32_t instance_of)
      : kind{static_cast<std::uint32_t>(kind_of)}, instance{instance_of} {}

  friend constexpr auto operator<=>(const Name&, const Name&) = default;
};

constexpr Name entity_component_name(std::uint32_t component,
                                     std::uint32_t entity) {
  Name name{Kind::ENTITY_COMPONENT, entity};
  name.kind += component;
  return name;
}

constexpr bool is_entity_component(Name name) {
  return name.kind >= static_cast<std::uint32_t>(Kind::ENTITY_COMPONENT);
}

// The world's component number of an entity-component name.
constexpr std::uint32_t component_of(Name name) {
  return name.kind - static_cast<std::uint32_t>(Kind::ENTITY_COMPONENT);
}

// The canonical identity of `name` in world number `world`.
Identity identity_of(std::uint32_t world, Name name);

struct ParsedIdentity final {
  std::uint32_t world = 0;
  Name name;
};

// Parses an identity made by identity_of. Returns nothing for anything else.
std::optional<ParsedIdentity> parse_identity(const Identity& given);

}  // namespace simon::framework

template <>
struct std::hash<simon::framework::Name> final {
  std::size_t operator()(simon::framework::Name name) const noexcept {
    return (static_cast<std::size_t>(name.kind) << 32) ^ name.instance;
  }
};

template <typename TagType>
struct std::formatter<simon::framework::TaggedString<TagType>>
    : std::formatter<std::string_view> {
  auto format(const simon::framework::TaggedString<TagType>& text,
              std::format_context& context) const {
    return std::formatter<std::string_view>::format(text.view(), context);
  }
};
