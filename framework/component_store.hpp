// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
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
  auto operator=(Uninitialized&& that) noexcept -> Uninitialized& {
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

  auto data() const -> Type* { return data_; }

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
  static auto default_chunk_size(std::size_t capacity) -> std::size_t {
    return std::min<std::size_t>(
        1024, std::bit_ceil(std::max<std::size_t>(capacity, 1)));
  }

  DECLARE_COPY_DELETE(ComponentStore);

  ComponentStore(ComponentStore&&) noexcept = default;
  auto operator=(ComponentStore&& that) noexcept -> ComponentStore& {
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

  auto size() const -> std::size_t { return size_; }
  auto capacity() const -> std::size_t { return capacity_; }
  auto chunk_size() const -> std::size_t { return chunk_size_; }

  auto contains(Entity entity) const -> bool {
    return slot_of(entity) != ABSENT;
  }

  auto maybe_component_of(Entity entity) -> ComponentType* {
    Slot slot = slot_of(entity);
    return slot != ABSENT ? &data_.data()[slot] : nullptr;
  }
  auto maybe_component_of(Entity entity) const -> const ComponentType* {
    Slot slot = slot_of(entity);
    return slot != ABSENT ? &data_.data()[slot] : nullptr;
  }

  auto component_of(Entity entity) -> ComponentType& {
    ComponentType* component = maybe_component_of(entity);
    CHECK_PRECONDITION(component);
    return *component;
  }
  auto component_of(Entity entity) const -> const ComponentType& {
    const ComponentType* component = maybe_component_of(entity);
    CHECK_PRECONDITION(component);
    return *component;
  }

  // Visits every entity-component as `visit(Entity, ComponentType&)`, segment
  // by segment, each in order.
  template <typename VisitorType>
  auto for_each(VisitorType&& visit) -> void {
    walk(*this, std::forward<VisitorType>(visit));
  }
  template <typename VisitorType>
  auto for_each(VisitorType&& visit) const -> void {
    walk(*this, std::forward<VisitorType>(visit));
  }

  //-- For the framework ------------------------------------------------------

  // Segments and their chunks, for the scheduler's loops.
  auto segments() const -> std::size_t { return segments_.size(); }
  auto segment_size(std::size_t segment) const -> std::size_t {
    return segments_[segment].size;
  }
  auto chunks_in(std::size_t segment) const -> std::size_t {
    return segments_[segment].chunks.size();
  }
  auto chunk(std::size_t segment, std::size_t ordinal) -> Chunk<ComponentType> {
    return chunk_of<ComponentType>(*this, segment, ordinal);
  }
  auto chunk(std::size_t segment, std::size_t ordinal) const
      -> Chunk<const ComponentType> {
    return chunk_of<const ComponentType>(*this, segment, ordinal);
  }

  // Slots, for indexes such as the spatial index. A slot names the same
  // entity-component until the next append or erase.
  template <typename VisitorType>
  auto for_each_slot(VisitorType&& visit) const -> void {
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

  auto owner_at(Slot slot) const -> Entity { return owner_[slot]; }
  auto component_at(Slot slot) const -> const ComponentType& {
    return data_.data()[slot];
  }

  // Appends to the end of `segment`.
  auto append(Entity entity, ComponentType component, std::size_t segment = 0)
      -> void {
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

  // Reorders `segment`: the entity-component at local index `order[i]`
  // moves to local index i. `order` holds each of the segment's local
  // indices once.
  auto permute(std::size_t segment, std::span<const std::uint32_t> order)
      -> void {
    const Segment& in = segments_[segment];
    CHECK_PRECONDITION(order.size() == in.size);
    ComponentType* data = data_.data();
    std::vector<ComponentType> components;
    std::vector<Entity> owners;
    components.reserve(order.size());
    owners.reserve(order.size());
    for (std::uint32_t local : order) {
      Slot from = slot_at(in, local);
      components.push_back(std::move(data[from]));
      owners.push_back(owner_[from]);
    }
    for (std::size_t local = 0; local < order.size(); ++local) {
      Slot to = slot_at(in, local);
      data[to] = std::move(components[local]);
      owner_[to] = owners[local];
      index_[owners[local].index].slot = to;
    }
  }

  // Erases, moving the segment's last entity-component into the gap.
  auto erase(Entity entity) -> void {
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

  auto destroy_all() -> void {
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

  // A pool chunk's place: its segment, and its position in that segment.
  struct Place final {
    std::uint32_t segment = 0;
    std::uint32_t ordinal = 0;
  };

  auto slot_of(Entity entity) const -> Slot {
    if (entity.index >= index_.size()) {
      return ABSENT;
    }
    const IndexEntry& entry = index_[entity.index];
    return entry.generation == entity.generation ? entry.slot : ABSENT;
  }

  auto slot_at(const Segment& segment, std::size_t local) const -> Slot {
    return static_cast<Slot>(segment.chunks[local / chunk_size_] * chunk_size_ +
                             local % chunk_size_);
  }

  template <typename DataType, typename StoreType>
  static auto chunk_of(StoreType& store, std::size_t segment,
                       std::size_t ordinal) -> Chunk<DataType> {
    const Segment& in = store.segments_[segment];
    std::size_t first = in.chunks[ordinal] * store.chunk_size_;
    return Chunk<DataType>{
        .owners = store.owner_.data() + first,
        .components = store.data_.data() + first,
        .size =
            std::min(store.chunk_size_, in.size - ordinal * store.chunk_size_)};
  }

  template <typename StoreType, typename VisitorType>
  static auto walk(StoreType& store, VisitorType&& visit) -> void {
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
