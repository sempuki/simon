// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstddef>
#include <expected>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "application/automotive/scenario_simulation.hpp"
#include "base/status.hpp"
#include "core/time.hpp"
#include "scenario/openscenario.hpp"
#include "scenario/parameter_distribution.hpp"

// Scenarios run in batches: every permutation of a parameter distribution
// played on several threads, each run measured by nuPlan's metrics (see
// model/driving_metrics.hpp) of its ego, the scenario's first entity.
namespace simon::automotive {

// A vehicle at a step: its reference point and heading in the world, and
// its speed.
struct RunSample final {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double speed = 0.0;
};

// A vehicle's box about its reference point.
struct RunBox final {
  double length = 0.0;
  double width = 0.0;
  double center = 0.0;  // m, ahead of the reference point.
};

// Each entity's box, in the scenario's order.
auto convert_entities_to_boxes(const scenario::Scenario& scenario)
    -> std::vector<RunBox>;

// Each entity of a configured simulation where it is now, in the scenario's
// order.
auto sample_entities(const ScenarioSimulation& simulation)
    -> std::vector<RunSample>;

// The ego's measures at one sample: its time to collision with the vehicles
// ahead, within 30 degrees of its heading as nuPlan's is_agent_ahead counts
// them, none if none comes within reach; and the least gap between its box
// and another's.
struct SampleMeasures final {
  std::optional<double> time_to_collision;
  double gap = std::numeric_limits<double>::infinity();
};

auto measure_sample(std::span<const RunSample> samples,
                    std::span<const RunBox> boxes) -> SampleMeasures;

// A run of a scenario: each entity's box, in the scenario's order, and
// every entity after each step, in the same order, but for the step on which
// the storyboard stops.
struct RunRecord final {
  double step = 0.0;  // s.
  std::vector<RunBox> boxes;
  std::vector<std::vector<RunSample>> samples;
};

// Plays the scenario at `path` with `assignments`, `step` at a time, until
// its storyboard stops or `limit` steps have run.
auto record_run(const std::string& path,
                std::vector<scenario::ParameterAssignment> assignments,
                Duration step, std::size_t limit)
    -> std::expected<RunRecord, lib::Status>;

// nuPlan's measures of a run's ego, and how near it came to the others.
struct RunMeasures final {
  double duration = 0.0;  // s.
  // Whether every comfort signal stayed within nuPlan's bounds throughout.
  bool comfortable = false;
  // The least time to collision with a vehicle ahead; none if none came
  // within reach.
  std::optional<double> min_time_to_collision;
  // Whether the time to collision stayed above nuPlan's 0.95 s throughout.
  bool time_to_collision_within_bound = false;
  // m, the least gap between the ego's box and another's; 0 if they touched.
  double min_gap = std::numeric_limits<double>::infinity();
  // At each sample, the time to collision, none if no vehicle ahead came
  // within reach, and the least gap.
  std::vector<std::optional<double>> times_to_collision;
  std::vector<double> gaps;
};

// The ego's samples as nuPlan reads them: its acceleration along its
// heading the change in speed over the step, and across it its speed times
// its rate of turn over the step, both 0 at the first sample; and each
// sample measured as measure_sample does.
auto measure_run(const RunRecord& run) -> RunMeasures;

// One permutation's run.
struct BatchRun final {
  std::size_t permutation = 0;
  std::vector<scenario::ParameterAssignment> assignments;
  std::expected<RunMeasures, lib::Status> measures;
};

// Plays every permutation of `distribution` on `threads` threads, each as
// record_run does, and measures it. In permutation order.
auto run_batch(const scenario::ParameterDistribution& distribution,
               std::size_t threads, Duration step, std::size_t limit)
    -> std::vector<BatchRun>;

}  // namespace simon::automotive
