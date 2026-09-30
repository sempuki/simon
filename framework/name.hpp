// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <charconv>
#include <compare>
#include <concepts>
#include <cstdint>
#include <format>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

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
inline Identity identity_of(std::uint32_t world, Name name) {
  if (is_entity_component(name)) {
    return std::format("/world/{}/entity/{}/component/{}", world, name.instance,
                       component_of(name));
  }
  switch (static_cast<Kind>(name.kind)) {
    case Kind::WORLD:
      return std::format("/world/{}", name.instance);
    case Kind::ARCHETYPE:
      return std::format("/world/{}/archetype/{}", world, name.instance);
    case Kind::COMPONENT:
      return std::format("/world/{}/component/{}", world, name.instance);
    case Kind::SYSTEM:
      return std::format("/world/{}/system/{}", world, name.instance);
    case Kind::ENTITY:
      return std::format("/world/{}/entity/{}", world, name.instance);
    case Kind::NONE:
    case Kind::ENTITY_COMPONENT:
      break;
  }
  return {};
}

struct ParsedIdentity final {
  std::uint32_t world = 0;
  Name name;
};

// Parses an identity made by identity_of. Returns nothing for anything else.
inline std::optional<ParsedIdentity> parse_identity(const Identity& given) {
  std::string_view identity = given.view();
  if (!identity.starts_with('/')) {
    return std::nullopt;
  }
  std::vector<std::string_view> segments;
  for (std::string_view rest = identity.substr(1); !rest.empty();) {
    std::size_t slash = rest.find('/');
    segments.push_back(rest.substr(0, slash));
    rest = slash == std::string_view::npos ? std::string_view{}
                                           : rest.substr(slash + 1);
  }

  auto number = [](std::string_view text) -> std::optional<std::uint32_t> {
    std::uint32_t value = 0;
    auto [end, error] =
        std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size() ||
        text.empty()) {
      return std::nullopt;
    }
    return value;
  };
  auto pair = [&](std::size_t at,
                  std::string_view word) -> std::optional<std::uint32_t> {
    if (segments.size() < at + 2 || segments[at] != word) {
      return std::nullopt;
    }
    return number(segments[at + 1]);
  };

  std::optional<std::uint32_t> world = pair(0, "world");
  if (!world) {
    return std::nullopt;
  }
  if (segments.size() == 2) {
    return ParsedIdentity{*world, Name{Kind::WORLD, *world}};
  }
  if (segments.size() == 4) {
    for (auto [word, kind] : {std::pair{"archetype", Kind::ARCHETYPE},
                              std::pair{"component", Kind::COMPONENT},
                              std::pair{"system", Kind::SYSTEM},
                              std::pair{"entity", Kind::ENTITY}}) {
      if (std::optional<std::uint32_t> instance = pair(2, word)) {
        return ParsedIdentity{*world, Name{kind, *instance}};
      }
    }
    return std::nullopt;
  }
  if (segments.size() == 6) {
    std::optional<std::uint32_t> entity = pair(2, "entity");
    std::optional<std::uint32_t> component = pair(4, "component");
    if (entity && component) {
      return ParsedIdentity{*world, entity_component_name(*component, *entity)};
    }
  }
  return std::nullopt;
}

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
