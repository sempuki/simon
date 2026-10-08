// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/arguments.hpp"

#include <array>
#include <optional>
#include <string>

#include "base/testing.hpp"

namespace simon::application {
namespace {

// Arguments as a main receives them, after the program's name.
template <std::size_t COUNT>
auto make_arguments(std::array<const char*, COUNT> given) -> Arguments {
  std::array<char*, COUNT + 1> argv{};
  argv[0] = const_cast<char*>("program");
  for (std::size_t i = 0; i < COUNT; ++i) {
    argv[i + 1] = const_cast<char*>(given[i]);
  }
  return Arguments{static_cast<int>(argv.size()), argv.data()};
}

}  // namespace

TEST_CASE("Arguments") {
  SECTION("ShouldReadPositionalAndOptionsGivenBoth") {
    // Preconditions.
    Arguments arguments = make_arguments(std::array{
        "ring.xodr", "--seconds=60", "40", "--disk", "2.5", "--step=0.25"});

    // Under Test.
    std::string_view roads = arguments.text(0, "none");
    std::int64_t vehicles = arguments.integer(1, 0);
    double rate = arguments.number(2, 0.0);
    std::int64_t seconds = arguments.option_integer("seconds", 300);
    double step = arguments.option_number("step", 1.0);
    bool disk = arguments.flag("disk");

    // Postconditions.
    CHECK(roads == "ring.xodr");
    CHECK(vehicles == 40);
    CHECK(rate == 2.5);
    CHECK(seconds == 60);
    CHECK(step == 0.25);
    CHECK(disk);
    CHECK(arguments.error() == std::nullopt);
  }

  SECTION("ShouldGiveFallbacksGivenNothing") {
    // Preconditions.
    Arguments arguments = make_arguments(std::array<const char*, 0>{});

    // Under Test.
    std::int64_t vehicles = arguments.integer(1, 40);
    std::string_view track = arguments.option_text("track", "");
    bool disk = arguments.flag("disk");

    // Postconditions.
    CHECK(vehicles == 40);
    CHECK(track.empty());
    CHECK_FALSE(disk);
    CHECK(arguments.error() == std::nullopt);
  }

  SECTION("ShouldRefuseGivenMalformedNumber") {
    // Preconditions.
    Arguments arguments = make_arguments(std::array{"4o"});

    // Under Test.
    std::int64_t vehicles = arguments.integer(0, 40);

    // Postconditions.
    CHECK(vehicles == 40);
    REQUIRE(arguments.error());
    CHECK(arguments.error()->contains("4o"));
  }

  SECTION("ShouldRefuseGivenOptionNeverRead") {
    // Preconditions.
    Arguments arguments = make_arguments(std::array{"--sceond=60"});

    // Under Test.
    std::int64_t seconds = arguments.option_integer("seconds", 300);

    // Postconditions.
    CHECK(seconds == 300);
    REQUIRE(arguments.error());
    CHECK(arguments.error()->contains("--sceond"));
  }

  SECTION("ShouldRefuseGivenMorePositionalThanRead") {
    // Preconditions.
    Arguments arguments = make_arguments(std::array{"1", "2"});

    // Under Test.
    std::int64_t seed = arguments.integer(0, 1);

    // Postconditions.
    CHECK(seed == 1);
    REQUIRE(arguments.error());
    CHECK(arguments.error()->contains("\"2\""));
  }
}

}  // namespace simon::application
