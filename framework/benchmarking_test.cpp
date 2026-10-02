// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "framework/benchmarking.hpp"

#include <optional>

#include "base/testing.hpp"

namespace simon::framework::benchmark {

TEST_CASE("Contention") {
  SECTION("ShouldUseEverySpareCoreGivenBareFlag") {
    CHECK(Contention::threads_from("--contend") == Contention::spare_cores());
  }

  SECTION("ShouldUseTheCountGivenFlagWithNumber") {
    CHECK(Contention::threads_from("--contend=3") == 3u);
    CHECK(Contention::threads_from("--contend=0") == 0u);
  }

  SECTION("ShouldAskForNothingGivenOtherOrMalformedArguments") {
    CHECK(Contention::threads_from("--competing") == std::nullopt);
    CHECK(Contention::threads_from("--contend=") == std::nullopt);
    CHECK(Contention::threads_from("--contend=two") == std::nullopt);
    CHECK(Contention::threads_from("--contend=2x") == std::nullopt);
    CHECK(Contention::threads_from("--contender") == std::nullopt);
  }

  SECTION("ShouldDescribeTheLoadGivenThreadCount") {
    CHECK(Contention::describe(0) == "uncontended");
    CHECK(Contention::describe(1) == "contended by 1 thread");
    CHECK(Contention::describe(4) == "contended by 4 threads");
  }
}

TEST_CASE("parse_count") {
  SECTION("ShouldReadTheNumberGivenWholePositiveNumber") {
    CHECK(parse_count("1") == 1);
    CHECK(parse_count("100000") == 100'000);
  }

  SECTION("ShouldReadNothingGivenZeroNegativeOrMalformedText") {
    CHECK(parse_count("0") == std::nullopt);
    CHECK(parse_count("-3") == std::nullopt);
    CHECK(parse_count("") == std::nullopt);
    CHECK(parse_count("12x") == std::nullopt);
    CHECK(parse_count("--steps") == std::nullopt);
  }
}

}  // namespace simon::framework::benchmark
