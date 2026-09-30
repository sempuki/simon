// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <utility>
#include <vector>

#include "base/core.hpp"
#include "core/entity.hpp"

namespace simon::core {

// The array of one component's entity-components.
//
// `data` and `owner` stay dense: erasing an entity-component moves the last
// one into the gap. `index` maps an entity index to a dense position and holds
// the generation of the entity that owns it, so a stale entity never matches.
// Every array is allocated once, at construction, and never reallocates.
template <typename Component>
class Store final {
 public:
  DECLARE_COPY_DELETE(Store);
  DECLARE_MOVE_DEFAULT(Store);

  Store(std::size_t capacity, std::size_t entity_capacity)
      : index_(entity_capacity) {
    owner_.reserve(capacity);
    data_.reserve(capacity);
  }
  ~Store() = default;

  std::size_t size() const { return data_.size(); }
  std::size_t capacity() const { return data_.capacity(); }

  bool contains(Entity entity) const { return position_of(entity) != ABSENT; }

  Component* try_get(Entity entity) {
    std::uint32_t position = position_of(entity);
    return position != ABSENT ? &data_[position] : nullptr;
  }
  const Component* try_get(Entity entity) const {
    std::uint32_t position = position_of(entity);
    return position != ABSENT ? &data_[position] : nullptr;
  }

  Component& get(Entity entity) {
    Component* component = try_get(entity);
    CHECK_PRECONDITION(component);
    return *component;
  }
  const Component& get(Entity entity) const {
    const Component* component = try_get(entity);
    CHECK_PRECONDITION(component);
    return *component;
  }

  // Dense access, in iteration order.
  Entity owner(std::size_t position) const { return owner_[position]; }
  Component& data(std::size_t position) { return data_[position]; }
  const Component& data(std::size_t position) const { return data_[position]; }
  std::span<const Entity> owners() const { return owner_; }
  std::span<Component> values() { return data_; }
  std::span<const Component> values() const { return data_; }

  void append(Entity entity, Component component) {
    CHECK_PRECONDITION(entity.index < index_.size());
    CHECK_PRECONDITION(!contains(entity));
    CHECK_PRECONDITION(size() < capacity());  // Never reallocate.
    index_[entity.index] = Slot{
        .position = static_cast<std::uint32_t>(data_.size()),
        .generation = entity.generation,
    };
    owner_.push_back(entity);
    data_.push_back(std::move(component));
  }

  void erase(Entity entity) {
    std::uint32_t position = position_of(entity);
    CHECK_PRECONDITION(position != ABSENT);
    std::uint32_t last = static_cast<std::uint32_t>(data_.size() - 1);
    if (position != last) {
      data_[position] = std::move(data_[last]);
      owner_[position] = owner_[last];
      index_[owner_[position].index].position = position;
    }
    data_.pop_back();
    owner_.pop_back();
    index_[entity.index] = Slot{};
  }

 private:
  static constexpr std::uint32_t ABSENT =
      std::numeric_limits<std::uint32_t>::max();

  struct Slot {
    std::uint32_t position = ABSENT;
    std::uint32_t generation = 0;
  };

  std::uint32_t position_of(Entity entity) const {
    if (entity.index >= index_.size()) {
      return ABSENT;
    }
    const Slot& slot = index_[entity.index];
    return slot.generation == entity.generation ? slot.position : ABSENT;
  }

  std::vector<Slot> index_;
  std::vector<Entity> owner_;
  std::vector<Component> data_;
};

}  // namespace simon::core
