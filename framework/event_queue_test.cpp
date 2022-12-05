// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/event_queue.hpp"

#include "base/testing.hpp"

namespace simon::framework {

TEST_CASE("EventQueue") {
  EventQueue events;
  TimePoint start{Duration{0.0}};
  TimePoint later{Duration{1.0}};

  struct M {
    int value = 0;
  } mesg{5};

  SECTION("ShouldSubscribeToPublishedMessages") {
    bool called = false;
    events.subscribe<M>([&called, later](auto time, M mesg) {
      CHECK(time == later);
      CHECK(mesg.value == 5);
      called = true;
    });

    events.process_until(start);
    CHECK(!called);

    events.publish<M>(later, mesg);
    CHECK(!called);

    events.process_until(later);
    CHECK(called);
  }

  SECTION("ShouldCallImmidateTimers") {
    bool called = false;
    events.start_timer(start, [&called](auto time) { called = true; });
    events.process_until(start);
    CHECK(called);
  }

  SECTION("ShouldCallDelayedTimers") {
    bool called = false;
    events.start_timer(later, [&called](auto time) { called = true; });
    events.process_until(start);
    CHECK(!called);
    events.process_until(later);
    CHECK(called);
  }
}
}  // namespace simon::framework
