// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <compare>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "base/core.hpp"

namespace simon::framework {

// An entity's local alias: an index into the world's entity table, plus the
// generation that was current when the entity was created. Destroying an
// entity bumps its generation, so every copy of the old value goes stale.
struct Entity final {
  static constexpr std::uint32_t INVALID_INDEX =
      std::numeric_limits<std::uint32_t>::max();

  std::uint32_t index = INVALID_INDEX;
  std::uint32_t generation = 0;

  friend constexpr auto operator<=>(const Entity&, const Entity&) = default;
};

// Fixed-capacity table of entity generations. Never reallocates. Destroyed
// indices are reused first-in first-out, so a destroyed index waits as long as
// possible before a new entity takes it.
class EntityTable final {
 public:
  explicit EntityTable(std::size_t capacity)
      : generations_(capacity, 1u), alive_(capacity, false), free_(capacity) {
    CHECK_PRECONDITION(capacity < Entity::INVALID_INDEX);
    for (std::size_t i = 0; i < capacity; ++i) {
      free_[i] = static_cast<std::uint32_t>(i);
    }
    free_count_ = capacity;
  }

  std::size_t capacity() const { return generations_.size(); }
  std::size_t size() const { return capacity() - free_count_; }

  bool alive(Entity entity) const {
    return entity.index < capacity() && alive_[entity.index] &&
           generations_[entity.index] == entity.generation;
  }

  Entity create() {
    CHECK_PRECONDITION(free_count_ > 0);
    std::uint32_t index = free_[free_head_];
    free_head_ = (free_head_ + 1) % capacity();
    --free_count_;
    alive_[index] = true;
    return Entity{.index = index, .generation = generations_[index]};
  }

  void destroy(Entity entity) {
    CHECK_PRECONDITION(alive(entity));
    alive_[entity.index] = false;
    ++generations_[entity.index];
    free_[(free_head_ + free_count_) % capacity()] = entity.index;
    ++free_count_;
  }

 private:
  std::vector<std::uint32_t> generations_;
  std::vector<bool> alive_;
  std::vector<std::uint32_t> free_;  // Ring buffer of free indices.
  std::size_t free_head_ = 0;
  std::size_t free_count_ = 0;
};

}  // namespace simon::framework
