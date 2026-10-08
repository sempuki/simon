// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "framework/name_table.hpp"

#include <optional>
#include <vector>

#include "base/testing.hpp"

namespace simon::framework {

TEST_CASE("NameTable") {
  NameTable names;
  names.reset(4);

  SECTION("ShouldNumberEntitiesWithoutReuseGivenCreateAndForget") {
    // Preconditions.
    Entity first{.index = 0, .generation = 0};
    Entity again{.index = 0, .generation = 1};

    // Under Test.
    std::uint32_t one = names.name_entity(first);
    names.forget_entity(one);
    std::uint32_t two = names.name_entity(again);

    // Postconditions.
    CHECK(one == 0u);
    CHECK(two == 1u);
    CHECK(names.entity_of(one) == std::nullopt);
    CHECK(names.entity_of(two) == again);
    CHECK(names.instance_of(again) == two);
  }

  SECTION("ShouldNameArchetypeOnceGivenAskedTwice") {
    // Under Test.
    Name ball = names.name_archetype("ball");
    Name same = names.name_archetype("ball");

    // Postconditions.
    CHECK(ball == same);
    CHECK(names.archetype_count() == 1u);
    CHECK(names.find_names(Alias{"ball"}) == std::vector<Name>{ball});
  }

  SECTION("ShouldKeepAliasesInOrderGivenGiveAndTake") {
    // Preconditions.
    Name a{Kind::ENTITY, 0};
    Name b{Kind::ENTITY, 1};

    // Under Test.
    names.give_alias(a, Alias{"red"});
    names.give_alias(b, Alias{"red"});
    names.give_alias(a, Alias{"lead"});
    names.take_alias(a, Alias{"red"});

    // Postconditions.
    CHECK(names.find_names(Alias{"red"}) == std::vector<Name>{b});
    CHECK(names.aliases_of(a) == std::vector<Alias>{Alias{"lead"}});
    CHECK(names.has_alias(a, Alias{"lead"}));
    CHECK_FALSE(names.has_alias(a, Alias{"red"}));
  }
}

}  // namespace simon::framework
