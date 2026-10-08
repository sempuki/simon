// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include "base/core.hpp"
#include "core/argument.hpp"
#include "core/time.hpp"
#include "framework/timeline.hpp"

namespace simon::engine {

// Identifies a message type. Handlers are found by it, and it is checked before
// an event is cast back to its message type, so no dynamic_cast is needed.
using EventType = const void*;

template <typename MessageType>
inline constexpr char event_type_tag = 0;

template <typename MessageType>
constexpr auto event_type_of() -> EventType {
  return &event_type_tag<MessageType>;
}

class EventBase {
 public:
  virtual ~EventBase() = default;
  virtual auto event_type() const -> EventType = 0;
  virtual auto time() const -> TimePoint = 0;
};

template <typename MessageType>
class Event final : public EventBase {
 public:
  template <typename... ArgumentTypes>
  explicit Event(TimePoint time, ArgumentTypes&&... arguments)
      : time_{time}, message_{std::forward<ArgumentTypes>(arguments)...} {}

  auto event_type() const -> EventType override {
    return event_type_of<MessageType>();
  }
  auto time() const -> TimePoint override { return time_; }
  auto data() const -> const MessageType& { return message_; }

 private:
  TimePoint time_{};
  MessageType message_;
};

// Rare, discrete happenings, delivered in time order. Per-step traffic belongs
// in components, not here.
class EventQueue final : private framework::Timeline::Source {
 public:
  EventQueue();

  template <typename HandlerType>
  auto start_timer(TimePoint time, HandlerType&& handler) -> void {
    publish<Timer>(time, Timer{std::forward<HandlerType>(handler)});
  }

  template <typename MessageType, typename HandlerType>
  auto subscribe(HandlerType&& handler) -> void {
    static_assert(std::is_same_v<MessageType, std::remove_cvref_t<MessageType>>,
                  "Unsupported: cv-ref qualified messages");

    handlers_[event_type_of<MessageType>()].emplace_back(
        [msg_handler = std::forward<HandlerType>(handler)](
            TimePoint time, const EventBase* base) {
          auto* event = static_cast<const Event<MessageType>*>(base);
          msg_handler(time, event->data());
        });
  }

  template <typename MessageType, typename... DeducedMessageArgumentTypes>
  auto publish(TimePoint time, DeducedMessageArgumentTypes&&... args) -> void {
    static_assert(std::is_same_v<MessageType, std::remove_cvref_t<MessageType>>,
                  "Unsupported: cv-ref qualified messages");

    events_.push_back(Entry{
        .sequence = next_sequence_++,
        .event = std::make_unique<Event<MessageType>>(
            time, std::forward<DeducedMessageArgumentTypes>(args)...),
    });
    std::push_heap(events_.begin(), events_.end(), Later{});
  }

  // Tells `timeline` when events are due, for as long as the queue lives and
  // stays in place, so a driver ends a step at each event's own time.
  auto attach(Depend<framework::Timeline> timeline) -> void {
    timeline->add(Depend<const framework::Timeline::Source>(*this));
  }

  // Delivers every event at or before `time`, earliest first, and events with
  // equal times in the order they were published. Each handler receives the
  // event's own time. Handlers may publish or subscribe while being called.
  auto process_until(TimePoint time) -> void;

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
    auto operator()(const Entry& a, const Entry& b) const -> bool {
      if (a.event->time() != b.event->time()) {
        return a.event->time() > b.event->time();
      }
      return a.sequence > b.sequence;
    }
  };

  // The earliest event's time.
  auto earliest() const -> std::optional<TimePoint> override {
    if (events_.empty()) {
      return std::nullopt;
    }
    return events_.front().event->time();
  }

  // The earliest event after `time`: the earliest event unless that is due at
  // `time`, and then a scan, as events are rare.
  auto earliest_after(TimePoint time) const
      -> std::optional<TimePoint> override;

  std::vector<Entry> events_;
  std::uint64_t next_sequence_ = 0;
  std::map<EventType,
           std::deque<std::function<void(TimePoint, const EventBase*)>>>
      handlers_;
};

}  // namespace simon::engine
