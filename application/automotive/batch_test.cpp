// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <numbers>
#include <string>
#include <vector>

#include "application/automotive/scenario_batch.hpp"
#include "application/automotive/testing.hpp"
#include "base/testing.hpp"
#include "format/openscenario.hpp"

// esmini's parameter distribution over cut-in, its 12 permutations played by
// simon in a batch and by esmini, from the tables
// reference/esmini_scenarios.py recorded; and each run measured by simon and
// by nuPlan's code on esmini's runs, from the tables
// reference/nuplan_metrics.py recorded.
namespace simon::automotive {

namespace {

using namespace std::chrono_literals;
using namespace testing;

constexpr std::string_view DISTRIBUTION =
    "3rd_party/esmini/xosc/cut-in_parameter_set.xosc";
constexpr std::size_t PERMUTATIONS = 12;
constexpr std::size_t LIMIT = 72'000;

auto load_distribution() -> scenario::ParameterDistribution {
  auto distribution =
      format::load_parameter_distribution(std::string{DISTRIBUTION});
  REQUIRE(distribution);
  return *distribution;
}

// Each permutation's rows of a table, in order.
auto load_by_permutation(std::string_view table)
    -> std::vector<std::vector<Row>> {
  std::vector<std::vector<Row>> rows(PERMUTATIONS);
  for (const Row& row : load_rows(table)) {
    rows[static_cast<std::size_t>(number(row, "permutation"))].push_back(row);
  }
  return rows;
}

// Each permutation's run in simon.
auto record_runs(const scenario::ParameterDistribution& distribution)
    -> std::vector<RunRecord> {
  std::vector<RunRecord> runs;
  for (std::size_t i = 0; i < PERMUTATIONS; ++i) {
    auto run =
        record_run(distribution.scenario,
                   scenario::find_permutation(distribution, i), 50ms, LIMIT);
    REQUIRE(run);
    runs.push_back(std::move(*run));
  }
  return runs;
}

}  // namespace

TEST_CASE("BatchAgainstEsminiAndNuPlan") {
  scenario::ParameterDistribution distribution = load_distribution();
  REQUIRE(scenario::count_permutations(distribution) == PERMUTATIONS);

  SECTION("ShouldNumberPermutationsAsEsmini") {
    // Each permutation's values, in order; numbers equal as numbers, since
    // esmini writes a range's values to six decimals.
    std::vector<std::vector<Row>> esmini =
        load_by_permutation("esmini_parameters.csv");
    for (std::size_t i = 0; i < PERMUTATIONS; ++i) {
      std::vector<scenario::ParameterAssignment> ours =
          scenario::find_permutation(distribution, i);
      REQUIRE(ours.size() == esmini[i].size());
      for (std::size_t k = 0; k < ours.size(); ++k) {
        CAPTURE(i, ours[k].name, ours[k].value);
        CHECK(ours[k].name == esmini[i][k].at("name"));
        const std::string& theirs = esmini[i][k].at("value");
        if (ours[k].name == "TargetSpeedFactor" || ours[k].name == "EgoSpeed") {
          CHECK(std::abs(std::stod(ours[k].value) - std::stod(theirs)) < 1e-12);
        } else {
          CHECK(ours[k].value == theirs);
        }
      }
    }
  }

  std::vector<RunRecord> runs = record_runs(distribution);

  SECTION("ShouldMatchEsminiRuns") {
    // Every entity at every step, its box from the vehicle the permutation
    // chose: positions as cut-in's own are, 1.1 mm on e6mini's curves at
    // most; speeds to esmini's log's six decimals.
    std::vector<std::vector<Row>> esmini =
        load_by_permutation("esmini_permutations.csv");
    double position = 0.0;
    double speed = 0.0;
    double box = 0.0;
    for (std::size_t i = 0; i < PERMUTATIONS; ++i) {
      std::map<std::string, std::vector<const Row*>> by_time;
      for (const Row& row : esmini[i]) {
        by_time[row.at("time")].push_back(&row);
      }
      // esmini logs the start, which simon does not sample.
      CHECK(runs[i].samples.size() + 1 == by_time.size());
      for (std::size_t k = 0; k < runs[i].samples.size(); ++k) {
        auto found = by_time.find(
            std::format("{:.2f}", 0.05 * static_cast<double>(k + 1)));
        REQUIRE(found != by_time.end());
        for (const Row* row : found->second) {
          std::size_t entity = row->at("entity") == "Ego" ? 0 : 1;
          const RunSample& ours = runs[i].samples[k][entity];
          position = std::max(position, std::hypot(ours.x - number(*row, "x"),
                                                   ours.y - number(*row, "y")));
          speed = std::max(speed, std::abs(ours.speed - number(*row, "speed")));
          const RunBox& b = runs[i].boxes[entity];
          box = std::max({box, std::abs(b.length - number(*row, "length")),
                          std::abs(b.width - number(*row, "width")),
                          std::abs(b.center - number(*row, "center"))});
        }
      }
    }
    CAPTURE(position, speed, box);
    CHECK(position < 1.5e-3);
    CHECK(speed < 1e-6);
    CHECK(box < 1e-9);
  }

  SECTION("ShouldMeasureAsNuPlan") {
    // At every sample, simon's time to collision and gap on its own runs,
    // and nuPlan's on esmini's: every time to collision the same, and the
    // gaps as close as the runs are.
    std::vector<std::vector<Row>> nuplan =
        load_by_permutation("nuplan_permutations.csv");
    std::vector<std::vector<Row>> verdicts =
        load_by_permutation("nuplan_verdicts.csv");
    int compared = 0;
    int agree = 0;
    double gap = 0.0;
    for (std::size_t i = 0; i < PERMUTATIONS; ++i) {
      RunMeasures measures = measure_run(runs[i]);
      REQUIRE(measures.times_to_collision.size() == nuplan[i].size());
      for (std::size_t k = 0; k < nuplan[i].size(); ++k) {
        const std::string& theirs = nuplan[i][k].at("ttc");
        const std::optional<double>& ours = measures.times_to_collision[k];
        ++compared;
        if ((!ours && theirs.empty()) ||
            (ours && !theirs.empty() &&
             std::abs(*ours - number(nuplan[i][k], "ttc")) < 1e-9)) {
          ++agree;
        } else {
          UNSCOPED_INFO("permutation " << i << " at " << nuplan[i][k].at("time")
                                       << " s: simon " << ours.value_or(-1.0)
                                       << ", nuPlan " << theirs);
        }
        gap = std::max(
            gap, std::abs(measures.gaps[k] - number(nuplan[i][k], "gap")));
      }
      const Row& verdict = verdicts[i].front();
      CAPTURE(i);
      CHECK(measures.comfortable == (verdict.at("comfortable") == "true"));
      CHECK(measures.time_to_collision_within_bound ==
            (verdict.at("ttc_within_bound") == "true"));
      CHECK(measures.min_time_to_collision.has_value() ==
            !verdict.at("min_ttc").empty());
      CHECK(std::abs(measures.min_gap - number(verdict, "min_gap")) < 1.5e-3);
    }
    CAPTURE(compared, agree, gap);
    CHECK(agree == compared);
    CHECK(gap < 1.5e-3);
  }

  SECTION("ShouldRunTheSameOnAnyThreads") {
    // Each run on its own world: the same measures, exactly, on one thread
    // or many.
    std::vector<BatchRun> alone = run_batch(distribution, 1, 50ms, LIMIT);
    std::vector<BatchRun> together = run_batch(distribution, 4, 50ms, LIMIT);
    REQUIRE(alone.size() == PERMUTATIONS);
    REQUIRE(together.size() == PERMUTATIONS);
    for (std::size_t i = 0; i < PERMUTATIONS; ++i) {
      REQUIRE(alone[i].measures);
      REQUIRE(together[i].measures);
      CHECK(alone[i].measures->gaps == together[i].measures->gaps);
      CHECK(alone[i].measures->times_to_collision ==
            together[i].measures->times_to_collision);
      CHECK(alone[i].measures->gaps == measure_run(runs[i]).gaps);
    }
  }
}

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
    REQUIRE(scenario::count_permutations(*distribution) == 7);
    CHECK(scenario::find_permutation(*distribution, 2)[0].value == "0.3");
    CHECK(scenario::find_permutation(*distribution, 6)[0].value == "0.7");
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
    std::vector<scenario::ParameterAssignment> assignments = {
        {.name = "Nonesuch", .value = "1"}};
    auto scenario = format::load_openscenario(
        "3rd_party/esmini/xosc/cut-in.xosc", assignments);
    CHECK_FALSE(scenario);
  }
}

}  // namespace simon::automotive
