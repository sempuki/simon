// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/testing.hpp"

namespace simon::framework {

std::string SummaryReporter::getDescription() {
  return "Summary reporter";
}

void SummaryReporter::testRunStarting(Catch::TestRunInfo const& info) {
  std::cout << "== " << info.name << "\n";
}

void SummaryReporter::testRunEnded(Catch::TestRunStats const& info) {
  //
}

void SummaryReporter::sectionStarting(Catch::SectionInfo const& info) {
  depth_++;
  if (depth_ <= 2) {
    for (std::size_t i = 0; i < depth_ * 2; ++i) {
      std::cout << ' ';
    }
  }
  std::cout << info.name << "\n";
}

void SummaryReporter::sectionEnded(Catch::SectionStats const& stats) {
  depth_--;
  if (depth_ == 2) {
    for (std::size_t i = 0; i < depth_ * 2; ++i) {
      std::cout << ' ';
    }
    if (stats.assertions.allPassed()) {
      std::cout << "Passed.\n";
    } else {
      std::cout << "Failed.\n";
    }
  }
}

CATCH_REGISTER_REPORTER("summary", SummaryReporter)

}  // namespace simon::framework
