// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/component_system.hpp"

#include <type_traits>
#include <vector>

#include "base/testing.hpp"
#include "base/time.hpp"

namespace simon::framework {

TEST_CASE("ComponentSystem") {
  struct T final : public Component<T> {};
  struct ComputeT {
    void prepare(T* component) {}
    void resolve(T* component) {}
    void operator()(T* component, TimePoint time, Duration step, std::nullptr_t) {
      times->push_back(time);
    }
    std::vector<TimePoint>* times = nullptr;
  };
  std::vector<TimePoint> was_called_at;
  ComponentSystem<T, ComputeT> sys{ComputeT{&was_called_at}};

  SECTION("ShouldHaveSameComponentNameAsComponent") {
    CHECK(sys.component_name() == T::name());
  }

  SECTION("ShouldAttachTypedComponentsToEntities") {
    GIVEN("an entity") {
      Entity a;

      WHEN("it's attached to a system") {
        sys.attach(&a);

        THEN("it should have a component") {
          CHECK(a.component<T>() != nullptr);
        }
      }
    }
  }

  SECTION("ShouldInvokeComputation") {
    // Setup
    Entity a;
    TimePoint time;
    Duration step;
    sys.attach(&a);

    // Act
    sys(time, step, nullptr);

    // Verify
    REQUIRE(was_called_at.size());
    CHECK(was_called_at.back() == time);
  }
}

}  // namespace simon::framework
