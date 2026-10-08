// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "scenario/parameter_distribution.hpp"

#include <vector>

#include "base/testing.hpp"
#include "format/openscenario.hpp"

// Reading esmini's parameter distributions (see 3rd_party/esmini/LICENSE)
// and numbering their permutations.
namespace simon::scenario {

TEST_CASE("ParameterDistribution") {
  SECTION("ShouldStepRangesToTheirUpperLimit") {
    auto distribution = format::parse_parameter_distribution(
        R"(<OpenSCENARIO><ParameterValueDistribution>
             <ScenarioFile filepath="s.xosc"/>
             <Deterministic>
               <DeterministicSingleParameterDistribution parameterName="a">
                 <DistributionRange stepWidth="0.1">
                   <Range lowerLimit="0.1" upperLimit="0.7"/>
                 </DistributionRange>
               </DeterministicSingleParameterDistribution>
             </Deterministic>
           </ParameterValueDistribution></OpenSCENARIO>)",
        "here");
    REQUIRE(distribution);
    CHECK(distribution->scenario == "here/s.xosc");
    REQUIRE(count_permutations(*distribution) == 7);
    CHECK(find_permutation(*distribution, 2)[0].value == "0.3");
    CHECK(find_permutation(*distribution, 6)[0].value == "0.7");
  }

  SECTION("ShouldRefuseStochasticDistributions") {
    auto distribution = format::parse_parameter_distribution(
        R"(<OpenSCENARIO><ParameterValueDistribution>
             <ScenarioFile filepath="s.xosc"/>
             <Stochastic numberOfTestRuns="5"/>
           </ParameterValueDistribution></OpenSCENARIO>)",
        "here");
    CHECK_FALSE(distribution);
  }

  SECTION("ShouldRefuseAssigningUndeclaredParameters") {
    std::vector<ParameterAssignment> assignments = {
        {.name = "Nonesuch", .value = "1"}};
    auto read = format::load_openscenario("3rd_party/esmini/xosc/cut-in.xosc",
                                          assignments);
    CHECK_FALSE(read);
  }
}
}  // namespace simon::scenario
