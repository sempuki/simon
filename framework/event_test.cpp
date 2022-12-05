// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/event.hpp"

#include <type_traits>

#include "base/testing.hpp"

namespace simon::framework {

TEST_CASE("Event") {
  SECTION("ShouldHaveStronglyTypedName") {
    REQUIRE(!std::is_same_v<EventName, Name>);
  }

  struct M final {
    int value = 0;
  } m{5};

  TimePoint t;
  Event<M> e{t, m};

  SECTION("ShouldHaveSameNameForObjectAsType") {
    CHECK(e.event_name() == Event<M>::name());
  }

  SECTION("ShouldHaveSameNameForBaseObjectAsType") {
    CHECK(static_cast<EventBase*>(&e)->event_name() == Event<M>::name());
  }

  SECTION("ShouldHaveSameTimePoint") {
    CHECK(e.time() == t);
  }

  SECTION("ShouldHaveSameMessage") {
    CHECK(e.data().value == m.value);
  }
}

}  // namespace simon::framework
