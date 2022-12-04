// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/component_system.hpp"

#include <type_traits>
#include <vector>

#include "base/testing.hpp"

namespace simon::framework {

TEST_CASE("ComponentSystem") {
  struct T final : public Component<T> {};
  struct ComputeT {
    void prepare() {}
    void resolve() {}
    void operator()(T* component, double time, double step, std::nullptr_t) {
      times->push_back(time);
    }
    std::vector<double>* times = nullptr;
  };
  std::vector<double> was_called_at;
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
    double time = 0.0;
    double step = 0.0;
    sys.attach(&a);

    // Act
    sys(time, step, nullptr);

    // Verify
    REQUIRE(was_called_at.size());
    CHECK(was_called_at.back() == time);
  }
}

}  // namespace simon::framework
