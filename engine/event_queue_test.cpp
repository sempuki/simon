// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "engine/event_queue.hpp"

#include <functional>
#include <vector>

#include "base/testing.hpp"

namespace simon::engine {

TEST_CASE("EventQueue") {
  EventQueue events;
  TimePoint start{std::chrono::seconds{0}};
  TimePoint later{std::chrono::seconds{1}};

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
    events.start_timer(start, [&called](auto /*time*/) { called = true; });
    events.process_until(start);
    CHECK(called);
  }

  SECTION("ShouldCallDelayedTimers") {
    bool called = false;
    events.start_timer(later, [&called](auto /*time*/) { called = true; });
    events.process_until(start);
    CHECK(!called);
    events.process_until(later);
    CHECK(called);
  }

  SECTION("ShouldDeliverEarliestFirstGivenEventsPublishedOutOfOrder") {
    std::vector<int> seen;
    events.subscribe<M>(
        [&seen](auto /*time*/, M mesg) { seen.push_back(mesg.value); });

    events.publish<M>(TimePoint{std::chrono::seconds{5}}, M{5});
    events.publish<M>(TimePoint{std::chrono::seconds{1}}, M{1});
    events.publish<M>(TimePoint{std::chrono::seconds{3}}, M{3});

    events.process_until(TimePoint{std::chrono::seconds{2}});
    CHECK(seen == std::vector<int>{1});

    events.process_until(TimePoint{std::chrono::seconds{10}});
    CHECK(seen == std::vector<int>{1, 3, 5});
  }

  SECTION("ShouldDeliverInPublishOrderGivenEqualTimes") {
    std::vector<int> seen;
    events.subscribe<M>(
        [&seen](auto /*time*/, M mesg) { seen.push_back(mesg.value); });

    for (int value = 0; value < 20; ++value) {
      events.publish<M>(later, M{value});
    }
    events.process_until(later);

    std::vector<int> expected;
    for (int value = 0; value < 20; ++value) {
      expected.push_back(value);
    }
    CHECK(seen == expected);
  }

  SECTION("ShouldPassEventTimeGivenLaterProcessTime") {
    TimePoint received;
    events.subscribe<M>(
        [&received](auto time, M /*mesg*/) { received = time; });

    events.publish<M>(later, mesg);
    events.process_until(TimePoint{std::chrono::seconds{3}});

    CHECK(received == later);
  }

  SECTION("ShouldCallEverySubscriberInOrderGivenSeveralSubscribers") {
    std::vector<int> seen;
    events.subscribe<M>(
        [&seen](auto /*time*/, M /*mesg*/) { seen.push_back(1); });
    events.subscribe<M>(
        [&seen](auto /*time*/, M /*mesg*/) { seen.push_back(2); });

    events.publish<M>(start, mesg);
    events.process_until(start);

    CHECK(seen == std::vector<int>{1, 2});
  }

  SECTION("ShouldNotCrossDispatchGivenDifferentMessageTypes") {
    struct N {};
    int m_calls = 0;
    int n_calls = 0;
    events.subscribe<M>([&m_calls](auto /*time*/, M /*mesg*/) { m_calls++; });
    events.subscribe<N>([&n_calls](auto /*time*/, N /*mesg*/) { n_calls++; });

    events.publish<N>(start);
    events.process_until(start);

    CHECK(m_calls == 0);
    CHECK(n_calls == 1);
  }

  SECTION("ShouldRepeatTimerGivenTimerThatRestartsItself") {
    std::vector<TimePoint> fired;
    std::function<void(TimePoint)> tick = [&](TimePoint time) {
      fired.push_back(time);
      if (fired.size() < 3) {
        events.start_timer(time + std::chrono::seconds{1}, tick);
      }
    };
    events.start_timer(start, tick);

    events.process_until(TimePoint{std::chrono::seconds{10}});

    CHECK(fired == std::vector<TimePoint>{start, later,
                                          TimePoint{std::chrono::seconds{2}}});
  }

  SECTION(
      "ShouldDeliverToNewSubscriberFromNextEventGivenSubscribeInsideHandler") {
    int outer_calls = 0;
    int inner_calls = 0;
    events.subscribe<M>([&](auto /*time*/, M /*mesg*/) {
      outer_calls++;
      // Enough subscriptions to force the handler container to grow.
      for (int i = 0; i < 64; ++i) {
        events.subscribe<M>(
            [&inner_calls](auto /*time*/, M /*mesg*/) { inner_calls++; });
      }
    });

    events.publish<M>(start, mesg);
    events.process_until(start);
    CHECK(outer_calls == 1);
    CHECK(inner_calls == 0);

    events.publish<M>(later, mesg);
    events.process_until(later);
    CHECK(outer_calls == 2);
    CHECK(inner_calls == 64);
  }
}
}  // namespace simon::engine
