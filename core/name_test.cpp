// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "core/name.hpp"

#include "base/testing.hpp"

namespace simon::core {

TEST_CASE("Name") {
  SECTION("ShouldBeEqualGivenSameKindAndInstance") {
    CHECK(Name{Kind::ENTITY, 2} == Name{Kind::ENTITY, 2});
    CHECK(Name{Kind::ENTITY, 2} != Name{Kind::SYSTEM, 2});
    CHECK(Name{Kind::ENTITY, 2} != Name{Kind::ENTITY, 3});
  }

  SECTION("ShouldBeNoneGivenDefault") {
    CHECK(Name{}.kind == static_cast<std::uint32_t>(Kind::NONE));
  }

  SECTION("ShouldRoundTripComponentGivenEntityComponentName") {
    Name name = entity_component_name(3, 2);
    CHECK(is_entity_component(name));
    CHECK(component_of(name) == 3u);
    CHECK(name.instance == 2u);
    CHECK_FALSE(is_entity_component(Name{Kind::ENTITY, 2}));
  }
}

TEST_CASE("Identity") {
  SECTION("ShouldFormatRestPathGivenEachKind") {
    CHECK(identity_of(1, Name{Kind::WORLD, 1}) == "/world/1");
    CHECK(identity_of(1, Name{Kind::ARCHETYPE, 0}) == "/world/1/archetype/0");
    CHECK(identity_of(1, Name{Kind::COMPONENT, 3}) == "/world/1/component/3");
    CHECK(identity_of(1, Name{Kind::SYSTEM, 4}) == "/world/1/system/4");
    CHECK(identity_of(1, Name{Kind::ENTITY, 2}) == "/world/1/entity/2");
    CHECK(identity_of(1, entity_component_name(3, 2)) ==
          "/world/1/entity/2/component/3");
  }

  SECTION("ShouldRoundTripGivenFormattedIdentity") {
    for (Name name : {Name{Kind::WORLD, 7}, Name{Kind::ARCHETYPE, 1},
                      Name{Kind::COMPONENT, 3}, Name{Kind::SYSTEM, 4},
                      Name{Kind::ENTITY, 2}, entity_component_name(5, 9)}) {
      auto parsed = parse_identity(identity_of(7, name));
      REQUIRE(parsed);
      CHECK(parsed->world == 7u);
      CHECK(parsed->name == name);
    }
  }

  SECTION("ShouldRejectGivenMalformedIdentity") {
    for (std::string_view text :
         {"", "world/1", "/world", "/world/x", "/world/1/entity",
          "/world/1/entity/2/component", "/world/1/robot/2",
          "/world/1/entity/-2", "/world/1/entity/2/extra/3",
          "/world/1/entity/2x"}) {
      CHECK_FALSE(parse_identity(text));
    }
  }
}

}  // namespace simon::core
