// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/name.hpp"

#include <charconv>
#include <format>
#include <optional>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace simon::framework {

Identity identity_of(std::uint32_t world, Name name) {
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

std::optional<ParsedIdentity> parse_identity(const Identity& given) {
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
