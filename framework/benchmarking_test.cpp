// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "framework/benchmarking.hpp"

#include <array>
#include <optional>
#include <string_view>
#include <vector>

#include "base/testing.hpp"

namespace simon::framework::benchmark {

TEST_CASE("Contention") {
  SECTION("ShouldUseEverySpareCoreGivenBareFlag") {
    // Postconditions.
    CHECK(Contention::threads_from("--contend") == Contention::spare_cores());
  }

  SECTION("ShouldUseTheCountGivenFlagWithNumber") {
    // Postconditions.
    CHECK(Contention::threads_from("--contend=3") == 3u);
    CHECK(Contention::threads_from("--contend=0") == 0u);
  }

  SECTION("ShouldAskForNothingGivenOtherOrMalformedArguments") {
    // Postconditions.
    CHECK(Contention::threads_from("--competing") == std::nullopt);
    CHECK(Contention::threads_from("--contend=") == std::nullopt);
    CHECK(Contention::threads_from("--contend=two") == std::nullopt);
    CHECK(Contention::threads_from("--contend=2x") == std::nullopt);
    CHECK(Contention::threads_from("--contender") == std::nullopt);
  }

  SECTION("ShouldDescribeTheLoadGivenThreadCount") {
    // Postconditions.
    CHECK(Contention::describe(0) == "uncontended");
    CHECK(Contention::describe(1) == "contended by 1 thread");
    CHECK(Contention::describe(4) == "contended by 4 threads");
  }
}

TEST_CASE("parse_count") {
  SECTION("ShouldReadTheNumberGivenWholePositiveNumber") {
    // Postconditions.
    CHECK(parse_count("1") == 1);
    CHECK(parse_count("100000") == 100'000);
  }

  SECTION("ShouldReadNothingGivenZeroNegativeOrMalformedText") {
    // Postconditions.
    CHECK(parse_count("0") == std::nullopt);
    CHECK(parse_count("-3") == std::nullopt);
    CHECK(parse_count("") == std::nullopt);
    CHECK(parse_count("12x") == std::nullopt);
    CHECK(parse_count("--steps") == std::nullopt);
  }
}

TEST_CASE("parse_arguments") {
  SECTION("ShouldTakeStepsAndContentionAndKeepTheRestGivenMixedArguments") {
    // Preconditions.
    std::array<const char*, 6> argv{"benchmark", "1000",        "--steps",
                                    "20",        "--contend=3", "--grid"};

    // Under Test.
    auto arguments = parse_arguments(static_cast<int>(argv.size()),
                                     const_cast<char**>(argv.data()));

    // Postconditions.
    REQUIRE(arguments.has_value());
    CHECK(arguments->steps == 20);
    CHECK(arguments->threads == 3u);
    CHECK(arguments->rest == std::vector<std::string_view>{"1000", "--grid"});
  }

  SECTION("ShouldTakeNoStepsGivenNoStepsArgument") {
    // Preconditions.
    std::array<const char*, 2> argv{"benchmark", "1000"};

    // Under Test.
    auto arguments = parse_arguments(static_cast<int>(argv.size()),
                                     const_cast<char**>(argv.data()));

    // Postconditions.
    REQUIRE(arguments.has_value());
    CHECK(arguments->steps == std::nullopt);
    CHECK(arguments->threads == 0u);
  }

  SECTION("ShouldFailGivenStepsWithoutAWholePositiveNumber") {
    // Preconditions.
    std::array<const char*, 3> argv{"benchmark", "--steps", "0"};
    std::array<const char*, 2> last{"benchmark", "--steps"};

    // Postconditions.
    CHECK_FALSE(parse_arguments(static_cast<int>(argv.size()),
                                const_cast<char**>(argv.data())));
    CHECK_FALSE(parse_arguments(static_cast<int>(last.size()),
                                const_cast<char**>(last.data())));
  }
}

}  // namespace simon::framework::benchmark
