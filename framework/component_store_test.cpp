// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/component_store.hpp"

#include <stdexcept>
#include <vector>

#include "base/testing.hpp"

namespace simon::framework {

namespace {
struct Mass final {
  double kilograms = 0.0;
};

// Counts how many exist, to check the store destroys what it constructs.
struct Counted final {
  static inline int alive = 0;
  Counted() { ++alive; }
  Counted(const Counted&) { ++alive; }
  Counted(Counted&&) noexcept { ++alive; }
  Counted& operator=(const Counted&) = default;
  Counted& operator=(Counted&&) noexcept = default;
  ~Counted() { --alive; }
};

template <typename ComponentType>
std::vector<Entity> owners_of(const ComponentStore<ComponentType>& store) {
  std::vector<Entity> owners;
  store.for_each(
      [&](Entity owner, const ComponentType&) { owners.push_back(owner); });
  return owners;
}
}  // namespace

TEST_CASE("ComponentStore") {
  EntityTable entities{8};
  ComponentStore<Mass> store{4, 8};
  Entity a = entities.create();
  Entity b = entities.create();
  Entity c = entities.create();

  SECTION("ShouldFindValueGivenAppended") {
    store.append(a, Mass{1.0});
    REQUIRE(store.try_component_of(a));
    CHECK(store.component_of(a).kilograms == 1.0);
    CHECK(store.size() == 1u);
  }

  SECTION("ShouldReturnNullGivenAbsent") {
    CHECK(store.try_component_of(a) == nullptr);
    CHECK_THROWS_AS(store.component_of(a), std::logic_error);
  }

  SECTION("ShouldStayDenseGivenMiddleErased") {
    store.append(a, Mass{1.0});
    store.append(b, Mass{2.0});
    store.append(c, Mass{3.0});

    store.erase(a);

    CHECK(store.size() == 2u);
    std::vector<Entity> owners;
    store.for_each([&](Entity owner, const Mass&) { owners.push_back(owner); });
    CHECK(owners == std::vector<Entity>{c, b});  // The last moved into the gap.
    CHECK(store.component_of(c).kilograms == 3.0);
    CHECK(store.component_of(b).kilograms == 2.0);
    CHECK_FALSE(store.contains(a));
  }

  SECTION("ShouldReturnNullGivenStaleEntityWhoseIndexWasReused") {
    store.append(a, Mass{1.0});
    store.erase(a);
    entities.destroy(a);
    Entity reused = Entity{};
    for (int i = 0; i < 6 && reused.index != a.index; ++i) {
      reused = entities.create();
    }
    REQUIRE(reused.index == a.index);
    store.append(reused, Mass{9.0});

    CHECK(store.try_component_of(a) == nullptr);
    CHECK(store.component_of(reused).kilograms == 9.0);
  }

  SECTION("ShouldKeepAddressesGivenAddsWithinCapacity") {
    store.append(a, Mass{1.0});
    const Mass* before = store.try_component_of(a);
    store.append(b, Mass{2.0});
    store.append(c, Mass{3.0});
    CHECK(store.try_component_of(a) == before);
  }

  SECTION("ShouldThrowGivenAppendBeyondCapacity") {
    Entity d = entities.create();
    Entity e = entities.create();
    store.append(a, Mass{});
    store.append(b, Mass{});
    store.append(c, Mass{});
    store.append(d, Mass{});
    CHECK_THROWS_AS(store.append(e, Mass{}), std::logic_error);
  }

  SECTION("ShouldVisitSegmentsInOrderGivenAppendsToEach") {
    ComponentStore<Mass> segmented{4, 8, 2, 2};
    segmented.append(a, Mass{1.0}, 1);
    segmented.append(b, Mass{2.0}, 0);
    segmented.append(c, Mass{3.0}, 1);

    CHECK(owners_of(segmented) == std::vector<Entity>{b, a, c});
    CHECK(segmented.segment_size(0) == 1u);
    CHECK(segmented.segment_size(1) == 2u);
    CHECK(segmented.component_of(c).kilograms == 3.0);
  }

  SECTION("ShouldTakeAndReturnChunksGivenAppendsAndErasesAcrossChunks") {
    ComponentStore<Mass> chunked{8, 8, 1, 2};
    std::vector<Entity> more{a, b, c, entities.create(), entities.create()};
    for (std::size_t i = 0; i < more.size(); ++i) {
      chunked.append(more[i], Mass{static_cast<double>(i)});
    }
    REQUIRE(chunked.chunks_in(0) == 3u);

    chunked.erase(more[0]);
    chunked.erase(more[1]);
    chunked.erase(more[2]);

    CHECK(chunked.chunks_in(0) == 1u);
    CHECK(chunked.component_of(more[3]).kilograms == 3.0);
    CHECK(chunked.component_of(more[4]).kilograms == 4.0);
  }

  SECTION("ShouldNameSameComponentGivenSlotBeforeNextChange") {
    ComponentStore<Mass> segmented{4, 8, 2, 2};
    segmented.append(a, Mass{1.0}, 1);
    segmented.append(b, Mass{2.0}, 0);

    int visited = 0;
    segmented.for_each_slot(
        [&](ComponentStore<Mass>::Slot slot, Entity owner, const Mass& mass) {
          CHECK(segmented.owner_at(slot) == owner);
          CHECK(&segmented.component_at(slot) == &mass);
          CHECK(&segmented.component_of(owner) == &mass);
          ++visited;
        });
    CHECK(visited == 2);
  }

  SECTION("ShouldDestroyEveryComponentGivenEraseOrDestruction") {
    Counted::alive = 0;
    {
      ComponentStore<Counted> counted{4, 8};
      counted.append(a, Counted{});
      counted.append(b, Counted{});
      counted.append(c, Counted{});
      counted.erase(a);
      CHECK(Counted::alive == 2);
    }
    CHECK(Counted::alive == 0);
  }

  SECTION("ShouldThrowGivenDuplicateAppend") {
    store.append(a, Mass{});
    CHECK_THROWS_AS(store.append(a, Mass{}), std::logic_error);
  }
}

}  // namespace simon::framework
