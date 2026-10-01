// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "base/core.hpp"
#include "framework/entity.hpp"

namespace simon::framework {

// Memory for `size` objects of `Type`, allocated once. The owner constructs
// objects in it and must destroy what it constructed.
template <typename Type>
class Uninitialized final {
 public:
  DECLARE_COPY_DELETE(Uninitialized);

  explicit Uninitialized(std::size_t size)
      : data_{std::allocator<Type>{}.allocate(size)}, size_{size} {}
  Uninitialized(Uninitialized&& that) noexcept
      : data_{std::exchange(that.data_, nullptr)},
        size_{std::exchange(that.size_, 0)} {}
  Uninitialized& operator=(Uninitialized&& that) noexcept {
    if (this != &that) {
      if (data_) {
        std::allocator<Type>{}.deallocate(data_, size_);
      }
      data_ = std::exchange(that.data_, nullptr);
      size_ = std::exchange(that.size_, 0);
    }
    return *this;
  }
  ~Uninitialized() {
    if (data_) {
      std::allocator<Type>{}.deallocate(data_, size_);
    }
  }

  Type* data() const { return data_; }

 private:
  Type* data_ = nullptr;
  std::size_t size_;
};

// The array of one component's entity-components, divided into segments.
//
// A world gives each archetype that requires this component its own segment,
// in archetype order, and a last segment for entities whose archetype only
// allows it. Within an archetype, every store orders its segment the same way.
// The world appends and erases in all of them together, so an entity's
// required components sit at the same local index in each.
//
// A segment is a list of chunks of `chunk_size` entries, drawn from a pool the
// store allocates once. A segment grows by taking a chunk and shrinks by
// returning one, so it never moves another segment's data, and the store never
// reallocates. Erasing moves the segment's last entity-component into the gap,
// so every segment stays dense.
//
// `index` maps an entity index to a slot, a position in the pool, and holds the
// generation of the entity that owns it, so a stale entity never matches.
template <typename ComponentType>
class ComponentStore final {
 public:
  using Slot = std::uint32_t;

  // One chunk of a segment: `size` owners and entity-components, in order.
  template <typename DataType>
  struct Chunk final {
    const Entity* owners = nullptr;
    DataType* components = nullptr;
    std::size_t size = 0;
  };

  // Chunks of at most 1024 entries, and smaller in small stores.
  static std::size_t default_chunk_size(std::size_t capacity) {
    return std::min<std::size_t>(
        1024, std::bit_ceil(std::max<std::size_t>(capacity, 1)));
  }

  DECLARE_COPY_DELETE(ComponentStore);

  ComponentStore(ComponentStore&&) noexcept = default;
  ComponentStore& operator=(ComponentStore&& that) noexcept {
    if (this != &that) {
      destroy_all();
      capacity_ = that.capacity_;
      chunk_size_ = that.chunk_size_;
      chunks_ = that.chunks_;
      data_ = std::move(that.data_);
      owner_ = std::move(that.owner_);
      place_ = std::move(that.place_);
      index_ = std::move(that.index_);
      segments_ = std::move(that.segments_);
      free_ = std::move(that.free_);
      size_ = std::exchange(that.size_, 0);
      that.segments_.clear();
    }
    return *this;
  }

  ComponentStore() : ComponentStore{0, 0} {}

  // Room for `capacity` entity-components among `segments` segments, for
  // entities whose indices are below `entity_capacity`. `chunk_size` must be a
  // power of two, and the same in every store of a world; zero picks
  // default_chunk_size.
  ComponentStore(std::size_t capacity,         //
                 std::size_t entity_capacity,  //
                 std::size_t segments = 1,     //
                 std::size_t chunk_size = 0)
      : capacity_{capacity},
        chunk_size_{chunk_size ? chunk_size : default_chunk_size(capacity)},
        // Enough chunks for every entry, plus a partly filled one per segment.
        chunks_{(capacity + chunk_size_ - 1) / chunk_size_ + segments},
        data_{chunks_ * chunk_size_},
        owner_(chunks_ * chunk_size_),
        place_(chunks_),
        index_(entity_capacity),
        segments_(segments) {
    CHECK_PRECONDITION(std::has_single_bit(chunk_size_));
    CHECK_PRECONDITION(segments >= 1);
    CHECK_PRECONDITION(chunks_ * chunk_size_ <
                       std::numeric_limits<Slot>::max());

    for (Segment& segment : segments_) {
      segment.chunks.reserve(chunks_);
    }
    free_.reserve(chunks_);

    for (std::size_t chunk = chunks_; chunk > 0; --chunk) {
      free_.push_back(static_cast<std::uint32_t>(chunk - 1));
    }
  }

  ~ComponentStore() { destroy_all(); }

  std::size_t size() const { return size_; }
  std::size_t capacity() const { return capacity_; }
  std::size_t chunk_size() const { return chunk_size_; }

  bool contains(Entity entity) const { return slot_of(entity) != ABSENT; }

  ComponentType* maybe_component_of(Entity entity) {
    Slot slot = slot_of(entity);
    return slot != ABSENT ? &data_.data()[slot] : nullptr;
  }
  const ComponentType* maybe_component_of(Entity entity) const {
    Slot slot = slot_of(entity);
    return slot != ABSENT ? &data_.data()[slot] : nullptr;
  }

  ComponentType& component_of(Entity entity) {
    ComponentType* component = maybe_component_of(entity);
    CHECK_PRECONDITION(component);
    return *component;
  }
  const ComponentType& component_of(Entity entity) const {
    const ComponentType* component = maybe_component_of(entity);
    CHECK_PRECONDITION(component);
    return *component;
  }

  // Visits every entity-component as `visit(Entity, ComponentType&)`, segment
  // by segment, each in order.
  template <typename VisitorType>
  void for_each(VisitorType&& visit) {
    walk(*this, std::forward<VisitorType>(visit));
  }
  template <typename VisitorType>
  void for_each(VisitorType&& visit) const {
    walk(*this, std::forward<VisitorType>(visit));
  }

  //-- For the framework ------------------------------------------------------

  // Segments and their chunks, for the scheduler's loops.
  std::size_t segments() const { return segments_.size(); }
  std::size_t segment_size(std::size_t segment) const {
    return segments_[segment].size;
  }
  std::size_t chunks_in(std::size_t segment) const {
    return segments_[segment].chunks.size();
  }
  Chunk<ComponentType> chunk(std::size_t segment, std::size_t ordinal) {
    return chunk_of<ComponentType>(*this, segment, ordinal);
  }
  Chunk<const ComponentType> chunk(std::size_t segment,
                                   std::size_t ordinal) const {
    return chunk_of<const ComponentType>(*this, segment, ordinal);
  }

  // Slots, for indexes such as the spatial index. A slot names the same
  // entity-component until the next append or erase.
  template <typename VisitorType>
  void for_each_slot(VisitorType&& visit) const {
    for (std::size_t segment = 0; segment < segments(); ++segment) {
      for (std::size_t ordinal = 0; ordinal < chunks_in(segment); ++ordinal) {
        Slot first =
            static_cast<Slot>(segments_[segment].chunks[ordinal] * chunk_size_);
        std::size_t size = chunk(segment, ordinal).size;
        for (std::size_t i = 0; i < size; ++i) {
          Slot slot = first + static_cast<Slot>(i);
          visit(slot, owner_[slot], data_.data()[slot]);
        }
      }
    }
  }
  Entity owner_at(Slot slot) const { return owner_[slot]; }
  const ComponentType& component_at(Slot slot) const {
    return data_.data()[slot];
  }

  // Appends to the end of `segment`.
  void append(Entity entity, ComponentType component, std::size_t segment = 0) {
    CHECK_PRECONDITION(entity.index < index_.size());
    CHECK_PRECONDITION(!contains(entity));
    CHECK_PRECONDITION(size_ < capacity_);  // Never reallocate.
    CHECK_PRECONDITION(segment < segments_.size());

    Segment& into = segments_[segment];
    if (into.size % chunk_size_ == 0) {
      std::uint32_t taken = free_.back();
      free_.pop_back();
      place_[taken] =
          Place{.segment = static_cast<std::uint32_t>(segment),
                .ordinal = static_cast<std::uint32_t>(into.chunks.size())};
      into.chunks.push_back(taken);
    }

    Slot slot = slot_at(into, into.size);
    std::construct_at(&data_.data()[slot], std::move(component));
    owner_[slot] = entity;
    index_[entity.index] =
        IndexEntry{.slot = slot, .generation = entity.generation};
    ++into.size;
    ++size_;
  }

  // Erases, moving the segment's last entity-component into the gap.
  void erase(Entity entity) {
    Slot slot = slot_of(entity);
    CHECK_PRECONDITION(slot != ABSENT);

    Segment& from = segments_[place_[slot / chunk_size_].segment];
    Slot last = slot_at(from, from.size - 1);
    ComponentType* data = data_.data();
    if (slot != last) {
      data[slot] = std::move(data[last]);
      owner_[slot] = owner_[last];
      index_[owner_[slot].index].slot = slot;
    }

    std::destroy_at(&data[last]);
    owner_[last] = Entity{};
    index_[entity.index] = IndexEntry{};
    --from.size;
    --size_;
    if (from.size % chunk_size_ == 0) {
      free_.push_back(from.chunks.back());
      from.chunks.pop_back();
    }
  }

 private:
  static constexpr Slot ABSENT = std::numeric_limits<Slot>::max();

  void destroy_all() {
    for_each(
        [](Entity, ComponentType& component) { std::destroy_at(&component); });
  }

  struct IndexEntry final {
    Slot slot = ABSENT;
    std::uint32_t generation = 0;
  };

  struct Segment final {
    std::vector<std::uint32_t> chunks;  // Pool chunks, in local order.
    std::size_t size = 0;
  };

  // Where a pool chunk is: its segment, and its position in that segment.
  struct Place final {
    std::uint32_t segment = 0;
    std::uint32_t ordinal = 0;
  };

  Slot slot_of(Entity entity) const {
    if (entity.index >= index_.size()) {
      return ABSENT;
    }
    const IndexEntry& entry = index_[entity.index];
    return entry.generation == entity.generation ? entry.slot : ABSENT;
  }

  Slot slot_at(const Segment& segment, std::size_t local) const {
    return static_cast<Slot>(segment.chunks[local / chunk_size_] * chunk_size_ +
                             local % chunk_size_);
  }

  template <typename DataType, typename StoreType>
  static Chunk<DataType> chunk_of(StoreType& store, std::size_t segment,
                                  std::size_t ordinal) {
    const Segment& in = store.segments_[segment];
    std::size_t first = in.chunks[ordinal] * store.chunk_size_;
    return Chunk<DataType>{
        .owners = store.owner_.data() + first,
        .components = store.data_.data() + first,
        .size =
            std::min(store.chunk_size_, in.size - ordinal * store.chunk_size_)};
  }

  template <typename StoreType, typename VisitorType>
  static void walk(StoreType& store, VisitorType&& visit) {
    for (std::size_t segment = 0; segment < store.segments(); ++segment) {
      for (std::size_t ordinal = 0; ordinal < store.chunks_in(segment);
           ++ordinal) {
        auto chunk = store.chunk(segment, ordinal);
        for (std::size_t i = 0; i < chunk.size; ++i) {
          visit(chunk.owners[i], chunk.components[i]);
        }
      }
    }
  }

  std::size_t capacity_;
  std::size_t chunk_size_;
  std::size_t chunks_;
  Uninitialized<ComponentType> data_;
  std::vector<Entity> owner_;
  std::vector<Place> place_;  // By pool chunk.
  std::vector<IndexEntry> index_;
  std::vector<Segment> segments_;
  std::vector<std::uint32_t> free_;  // Pool chunks no segment holds.
  std::size_t size_ = 0;
};

}  // namespace simon::framework
