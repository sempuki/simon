// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <type_traits>

#include "base/time.hpp"
#include "framework/event.hpp"

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

    events_.emplace(
      std::make_unique<Event<MessageType>>(time, std::forward<DeducedMessageArgs>(args)...));
  }

  void process_until(TimePoint time) {
    while (events_.size() && events_.top()->time() <= time) {
      for (auto& handler : handlers_[events_.top()->event_name()]) {
        handler(time, events_.top().get());
      }
      events_.pop();
    }
  }

 private:
  struct Timer final {
    std::function<void(TimePoint)> action;
  };

  struct Compare final {
    bool operator()(const std::unique_ptr<EventBase>& a, const std::unique_ptr<EventBase>& b) {
      return a->time() < b->time();
    }
  };

  std::priority_queue<std::unique_ptr<EventBase>, std::vector<std::unique_ptr<EventBase>>, Compare>
    events_;
  std::map<EventName, std::vector<std::function<void(TimePoint, const EventBase*)>>> handlers_;
};

}  // namespace simon::framework
