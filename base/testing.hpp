// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <sstream>
#include <vector>

#include "catch2/catch.hpp"

namespace simon {

// See: external/catch2/examples/210-Evt-EventListeners.cpp
class SummaryReporter : public Catch::StreamingReporterBase<SummaryReporter> {
 public:
  using StreamingReporterBase<SummaryReporter>::StreamingReporterBase;
  static std::string getDescription();

  void testRunStarting(Catch::TestRunInfo const& info) override;
  void testRunEnded(Catch::TestRunStats const& stats) override;

  void sectionStarting(Catch::SectionInfo const& info) override;
  void sectionEnded(Catch::SectionStats const& stats) override;

  void assertionStarting(Catch::AssertionInfo const& info) override;
  bool assertionEnded(Catch::AssertionStats const& stats) override;

 private:
  std::size_t depth_ = 0;
  std::vector<std::stringstream> report_;
};

}  // namespace simon
