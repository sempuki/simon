// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <limits>
#include <utility>

#include "base/core.hpp"
#include "base/status.hpp"
#include "framework/archetype.hpp"
#include "framework/build_error.hpp"
#include "framework/spatial.hpp"
#include "framework/type_list.hpp"

namespace simon::framework {

// Builds a world. How many of each archetype it holds at once sizes it: the
// entity capacity is the total, and each component's store holds as many as
// every archetype that requires or allows the component. Nothing is reserved
// per archetype; capacity is pooled. Start with `World::set_up()`, and build
// into the caller's world, since a world never moves:
//
//   World world;
//   std::expected<void, Status> built =
//       World::set_up()
//           .numbered(1)
//           .holding<archetype::RedDrone>(drones)
//           .holding<archetype::Track>(drones)
//           .cells_of(250.0 * model::meter)
//           .build(lib::Out(world));
template <typename WorldType>
class [[nodiscard]] SetUpBuilder final {
 public:
  using SpatialType = typename WorldType::SpatialComponent;
  using ArchetypeList = typename WorldType::ArchetypeList;
  using ComponentList = typename WorldType::ComponentList;

  // The world's instance in its Name and Identity. Zero unless given.
  SetUpBuilder numbered(std::uint32_t number) && {
    number_ = number;
    return std::move(*this);
  }

  // Holds `count` more entities of `ArchetypeType` alive at once. Holdings add
  // up, so a scenario can count each thing that creates the archetype.
  template <Archetypal ArchetypeType>
  SetUpBuilder holding(std::size_t count) && {
    static_assert(contains_v<ArchetypeList, ArchetypeType>,
                  "This archetype is not in the world's archetype list.");
    std::size_t& holding = holdings_[index_of_v<ArchetypeList, ArchetypeType>];
    holding = add_saturating(holding, count);
    return std::move(*this);
  }

  // The edge of a spatial index cell. About the radius of a typical query
  // works well. One coordinate unit unless given.
  SetUpBuilder cells_of(distance_of_t<SpatialType> size) && {
    static_assert(std::default_initializable<SpatialType>,
                  "Sizing cells needs a default spatial component to convert "
                  "the distance with.");
    cell_size_ = coordinate_length(SpatialType{}, size);
    return std::move(*this);
  }

  // Fills `world` as planned, discarding everything it held. A refused plan
  // leaves `world` as it was.
  std::expected<void, Status> build(lib::Out<WorldType> world) && {
    if (!(cell_size_ > 0.0) || !std::isfinite(cell_size_)) {
      return std::unexpected(
          lib::raise(BuildError::CELL_SIZE_INVALID,
                     "A world's spatial index cells must have a positive, "
                     "finite size."));
    }
    typename WorldType::Configuration configuration{.number = number_,
                                                    .cell_size = cell_size_};
    for (std::size_t holding : holdings_) {
      configuration.entities = add_saturating(configuration.entities, holding);
    }
    // Every store indexes its pool with 32 bits, with a partly filled chunk
    // per segment to spare.
    constexpr std::size_t MOST =
        std::size_t{std::numeric_limits<std::uint32_t>::max()} / 2;
    if (configuration.entities > MOST) {
      return std::unexpected(
          lib::raise(BuildError::CAPACITY_TOO_LARGE,
                     std::format("A world holds at most {} entities, not {}.",
                                 MOST, configuration.entities)));
    }
    for_each_type(ComponentList{}, [&]<typename ComponentType>() {
      std::size_t& capacity =
          configuration.capacities[index_of_v<ComponentList, ComponentType>];
      for (std::size_t archetype = 0; archetype < holdings_.size();
           ++archetype) {
        if (WorldType::template archetype_permits<ComponentType>(archetype)) {
          capacity = add_saturating(capacity, holdings_[archetype]);
        }
      }
    });
    world->initialize(configuration);
    return {};
  }

 private:
  // Sums that would wrap stay at the largest size, so build() refuses them.
  static std::size_t add_saturating(std::size_t a, std::size_t b) {
    return b > std::numeric_limits<std::size_t>::max() - a
               ? std::numeric_limits<std::size_t>::max()
               : a + b;
  }

  std::uint32_t number_ = 0;
  // How many of each archetype, by its position in ArchetypeList.
  std::array<std::size_t, ArchetypeList::size> holdings_{};
  double cell_size_ = 1.0;
};

}  // namespace simon::framework
