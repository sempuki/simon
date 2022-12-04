// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/type_macros.hpp"

#include <type_traits>

#include "base/testing.hpp"

namespace simon::framework {

TEST_CASE("type_macros") {
  SECTION("ShouldMakeDefaultableTypeDefaultable") {
    struct T {
      DECLARE_DEFAULTABLE(T);
    };
    REQUIRE(std::is_default_constructible_v<T>);
  }

  SECTION("ShouldMakeNonDefaultableTypeNotDefaultable") {
    struct T {
      DECLARE_NON_DEFAULTABLE(T);
    };
    REQUIRE(!std::is_default_constructible_v<T>);
  }

  SECTION("ShouldMakeCopyableTypeCopyable") {
    struct T {
      DECLARE_COPYABLE(T);
    };
    REQUIRE(std::is_copy_constructible_v<T>);
    REQUIRE(std::is_copy_assignable_v<T>);
  }

  SECTION("ShouldMakeNonCopyableTypeNotCopyable") {
    struct T {
      DECLARE_NON_COPYABLE(T);
    };
    REQUIRE(!std::is_copy_constructible_v<T>);
    REQUIRE(!std::is_copy_assignable_v<T>);
  }

  SECTION("ShouldMakeMovableTypeMovable") {
    struct T {
      DECLARE_MOVABLE(T);
    };
    REQUIRE(std::is_move_constructible_v<T>);
    REQUIRE(std::is_move_assignable_v<T>);
  }

  SECTION("ShouldMakeNonMovableTypeNotMovable") {
    struct T {
      DECLARE_NON_MOVABLE(T);
    };
    REQUIRE(!std::is_move_constructible_v<T>);
    REQUIRE(!std::is_move_assignable_v<T>);
  }

  SECTION("ShouldMakeCopyOnlyTypeCopyableButNotMovable") {
    struct T {
      DECLARE_COPY_ONLY(T);
    };
    REQUIRE(std::is_copy_constructible_v<T>);
    REQUIRE(std::is_copy_assignable_v<T>);
    REQUIRE(!std::is_move_constructible_v<T>);
    REQUIRE(!std::is_move_assignable_v<T>);
  }

  SECTION("ShouldMakeMoveOnlyTypeMovableButNotCopyable") {
    struct T {
      DECLARE_MOVE_ONLY(T);
    };
    REQUIRE(!std::is_copy_constructible_v<T>);
    REQUIRE(!std::is_copy_assignable_v<T>);
    REQUIRE(std::is_move_constructible_v<T>);
    REQUIRE(std::is_move_assignable_v<T>);
  }

  SECTION("ShouldMakeMoveCopyAbleTypeMovableAndCopyable") {
    struct T {
      DECLARE_MOVE_COPY_ABLE(T);
    };
    REQUIRE(std::is_copy_constructible_v<T>);
    REQUIRE(std::is_copy_assignable_v<T>);
    REQUIRE(std::is_move_constructible_v<T>);
    REQUIRE(std::is_move_assignable_v<T>);
  }

  SECTION("ShouldMakeUnmovableTypeNotCopyableAndNotMovable") {
    struct T {
      DECLARE_UNMOVABLE(T);
    };
    REQUIRE(!std::is_copy_constructible_v<T>);
    REQUIRE(!std::is_copy_assignable_v<T>);
    REQUIRE(!std::is_move_constructible_v<T>);
    REQUIRE(!std::is_move_assignable_v<T>);
  }

  SECTION("ShouldMakeSemiRegularTypeDefaultableAndMovableAndCopyable") {
    struct T {
      DECLARE_SEMIREGULAR(T);
    };
    REQUIRE(std::is_default_constructible_v<T>);
    REQUIRE(std::is_copy_constructible_v<T>);
    REQUIRE(std::is_copy_assignable_v<T>);
    REQUIRE(std::is_move_constructible_v<T>);
    REQUIRE(std::is_move_assignable_v<T>);
  }
}

}  // namespace simon::framework
