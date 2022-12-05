// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <type_traits>

#include "base/time.hpp"
#include "framework/identity.hpp"

namespace simon::framework {

struct EventName final : public Name {
  EventName(Name name) : Name{name} {}
};

class EventBase {
 public:
  virtual EventName event_name() const = 0;
  virtual TimePoint time() const = 0;
};

template <typename MessageType>
class Event final : public EventBase, public PerTypeIdentity<MessageType> {
 public:
  template <typename... Args>
  explicit Event(TimePoint time, Args&&... args)
    : time_{time}, message_{std::forward<Args>(args)...} {}

  TimePoint time() const override { return time_; }
  const MessageType& data() const { return message_; }

  EventName event_name() const override { return Event::name(); }
  static EventName name() { return Event::id().name(); }

 private:
  TimePoint time_{};
  MessageType message_;

  friend bool operator<(const Event& a, const Event& b) { return a.time_ < b.time_; }
};

}  // namespace simon::framework
