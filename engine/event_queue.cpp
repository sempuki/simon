// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "engine/event_queue.hpp"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <utility>

namespace simon::engine {

EventQueue::EventQueue() {
  // Install the bespoke timer handler.
  handlers_[event_type_of<Timer>()].emplace_back(
      [](TimePoint time, const EventBase* base) {
        auto* event = static_cast<const Event<Timer>*>(base);
        event->data().action(time);
      });
}

auto EventQueue::process_until(TimePoint time) -> void {
  while (!events_.empty() && events_.front().event->time() <= time) {
    // Take ownership before calling handlers, since they may publish.
    std::pop_heap(events_.begin(), events_.end(), Later{});
    std::unique_ptr<EventBase> event = std::move(events_.back().event);
    events_.pop_back();

    // A deque keeps existing handlers in place when a handler subscribes.
    // Handlers subscribed during delivery first see the next event.
    auto& handlers = handlers_[event->event_type()];
    for (std::size_t i = 0, count = handlers.size(); i < count; ++i) {
      handlers[i](event->time(), event.get());
    }
  }
}

}  // namespace simon::engine
