// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "core/step.hpp"

namespace simon::drive {

using core::Duration;
using core::TimePoint;

// Identifies a message type. Handlers are found by it, and it is checked before
// an event is cast back to its message type, so no dynamic_cast is needed.
using EventType = const void*;

template <typename MessageType>
inline constexpr char event_type_tag = 0;

template <typename MessageType>
constexpr EventType event_type_of() {
  return &event_type_tag<MessageType>;
}

class EventBase {
 public:
  virtual ~EventBase() = default;
  virtual EventType event_type() const = 0;
  virtual TimePoint time() const = 0;
};

template <typename MessageType>
class Event final : public EventBase {
 public:
  template <typename... Arguments>
  explicit Event(TimePoint time, Arguments&&... arguments)
      : time_{time}, message_{std::forward<Arguments>(arguments)...} {}

  EventType event_type() const override { return event_type_of<MessageType>(); }
  TimePoint time() const override { return time_; }
  const MessageType& data() const { return message_; }

 private:
  TimePoint time_{};
  MessageType message_;
};

// Rare, discrete happenings, delivered in time order. Per-step traffic belongs
// in components, not here.
class EventQueue final {
 public:
  EventQueue() {
    // Install the bespoke timer handler.
    handlers_[event_type_of<Timer>()].emplace_back(
        [](TimePoint time, const EventBase* base) {
          auto* event = static_cast<const Event<Timer>*>(base);
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

    handlers_[event_type_of<MessageType>()].emplace_back(
        [msg_handler = std::forward<HandlerType>(handler)](
            TimePoint time, const EventBase* base) {
          auto* event = static_cast<const Event<MessageType>*>(base);
          msg_handler(time, event->data());
        });
  }

  template <typename MessageType, typename... DeducedMessageArgs>
  void publish(TimePoint time, DeducedMessageArgs&&... args) {
    static_assert(std::is_same_v<MessageType, std::remove_cvref_t<MessageType>>,
                  "Unsupported: cv-ref qualified messages");

    events_.push_back(Entry{
        .sequence = next_sequence_++,
        .event = std::make_unique<Event<MessageType>>(
            time, std::forward<DeducedMessageArgs>(args)...),
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
      auto& handlers = handlers_[event->event_type()];
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
  std::map<EventType,
           std::deque<std::function<void(TimePoint, const EventBase*)>>>
      handlers_;
};

}  // namespace simon::drive
