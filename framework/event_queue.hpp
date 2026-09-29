// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <type_traits>
#include <vector>

#include "base/time.hpp"
#include "framework/event.hpp"

namespace simon {
using lib::Duration;
using lib::SimClock;
using lib::TimePoint;
}  // namespace simon

namespace simon::framework {

class EventQueue final {
 public:
  EventQueue() {
    // Install the bespoke timer handler.
    handlers_[Event<Timer>::name()].emplace_back([](TimePoint time, const EventBase* base) {
      auto* event = dynamic_cast<const Event<Timer>*>(base);
      event->data().action(time);
    });
  }

  template <typename HandlerType>
  void start_timer(TimePoint time, HandlerType&& handler) {
    publish<Timer>(time, Timer{std::forward<HandlerType>(handler)});
  }

  template <typename MessageType, typename HandlerType>
  void subscribe(HandlerType&& handler) {
    static_assert(std::is_same_v<MessageType, std::remove_cvref_t<MessageType>>,
                  "Unsupported: cv-ref qualified messages");

    handlers_[Event<MessageType>::name()].emplace_back(
      [msg_handler = std::forward<HandlerType>(handler)](TimePoint time, const EventBase* base) {
        auto* event = dynamic_cast<const Event<MessageType>*>(base);
        msg_handler(time, event->data());
      });
  }

  template <typename MessageType, typename... DeducedMessageArgs>
  void publish(TimePoint time, DeducedMessageArgs&&... args) {
    static_assert(std::is_same_v<MessageType, std::remove_cvref_t<MessageType>>,
                  "Unsupported: cv-ref qualified messages");

    events_.push_back(Entry{
      .sequence = next_sequence_++,
      .event =
        std::make_unique<Event<MessageType>>(time, std::forward<DeducedMessageArgs>(args)...),
    });
    std::push_heap(events_.begin(), events_.end(), Later{});
  }

  // Delivers every event at or before `time`, earliest first, and events with
  // equal times in the order they were published. Each handler receives the
  // event's own time. Handlers may publish or subscribe while being called.
  void process_until(TimePoint time) {
    while (!events_.empty() && events_.front().event->time() <= time) {
      // Take ownership before calling handlers, since they may publish.
      std::pop_heap(events_.begin(), events_.end(), Later{});
      std::unique_ptr<EventBase> event = std::move(events_.back().event);
      events_.pop_back();

      // A deque keeps existing handlers in place when a handler subscribes.
      // Handlers subscribed during delivery first see the next event.
      auto& handlers = handlers_[event->event_name()];
      for (std::size_t i = 0, count = handlers.size(); i < count; ++i) {
        handlers[i](event->time(), event.get());
      }
    }
  }

 private:
  struct Timer final {
    std::function<void(TimePoint)> action;
  };

  struct Entry final {
    std::uint64_t sequence = 0;
    std::unique_ptr<EventBase> event;
  };

  // Heap order that puts the earliest event, then the first published, on top.
  struct Later final {
    bool operator()(const Entry& a, const Entry& b) const {
      if (a.event->time() != b.event->time()) {
        return a.event->time() > b.event->time();
      }
      return a.sequence > b.sequence;
    }
  };

  std::vector<Entry> events_;
  std::uint64_t next_sequence_ = 0;
  std::map<EventName, std::deque<std::function<void(TimePoint, const EventBase*)>>> handlers_;
};

}  // namespace simon::framework
