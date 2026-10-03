// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/automotive/scenario_batch.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <thread>
#include <utility>

#include "application/automotive/scenario_simulation.hpp"
#include "model/collision.hpp"
#include "model/driving_metrics.hpp"

namespace simon::automotive {

namespace {

// nuPlan's time_to_collision_within_bound: the least time to collision
// allowed, and the tolerance of is_agent_ahead.
constexpr double LEAST_TIME_TO_COLLISION = 0.95;  // s.
constexpr double AHEAD = 30.0 * std::numbers::pi / 180.0;

auto convert_sample_to_box(const RunSample& sample, const RunBox& box)
    -> model::OrientedBox {
  return {.x = sample.x + box.center * std::cos(sample.heading),
          .y = sample.y + box.center * std::sin(sample.heading),
          .heading = sample.heading,
          .length = box.length,
          .width = box.width};
}

}  // namespace

auto record_run(const std::string& path,
                std::vector<scenario::ParameterAssignment> assignments,
                framework::Duration step, std::size_t limit)
    -> std::expected<RunRecord, lib::Status> {
  ScenarioSimulation simulation{path, std::move(assignments)};
  RETURN_IF_UNEXPECTED(simulation.configure());
  RunRecord run{.step = std::chrono::duration<double>(step).count()};
  for (const scenario::Entity& entity : simulation.scenario().entities) {
    const scenario::Vehicle& vehicle = entity.vehicle;
    run.boxes.push_back({.length = vehicle.dimensions[0],
                         .width = vehicle.dimensions[1],
                         .center = vehicle.center[0]});
  }
  const ScenarioWorld& world = simulation.world();
  for (std::size_t k = 0; k < limit; ++k) {
    framework::Step now{.time = framework::TimePoint{} + k * step, .dt = step};
    RETURN_OR_ASSIGN(engine::Flow flow, simulation.step(now));
    // The step on which the storyboard stops is not sampled, as esmini
    // does not log it.
    if (flow == engine::Flow::STOP || !simulation.player().running()) {
      break;
    }
    std::vector<RunSample> samples(run.boxes.size());
    world.store_of<ScenarioActor>().for_each([&](Entity owner,
                                                 const ScenarioActor& actor) {
      const VehiclePose& pose =
          world.store_of<VehiclePose>().component_of(owner);
      Vector3 at = model::eigen(pose.position);
      samples[actor.entity] = {
          .x = at.x(),
          .y = at.y(),
          .heading = model::radians(pose.heading),
          .speed = world.store_of<ScenarioSpeed>().component_of(owner).speed};
    });
    run.samples.push_back(std::move(samples));
  }
  return run;
}

auto measure_run(const RunRecord& run) -> RunMeasures {
  RunMeasures measures{.duration =
                           run.step * static_cast<double>(run.samples.size())};
  if (run.samples.empty()) {
    measures.comfortable = true;
    measures.time_to_collision_within_bound = true;
    return measures;
  }
  std::vector<model::DrivingSample> ego;
  for (std::size_t k = 0; k < run.samples.size(); ++k) {
    const RunSample& now = run.samples[k][0];
    model::DrivingSample sample{.time = run.step * static_cast<double>(k),
                                .x = now.x,
                                .y = now.y,
                                .heading = now.heading,
                                .speed = now.speed};
    if (k > 0) {
      const RunSample& before = run.samples[k - 1][0];
      double turn =
          std::remainder(now.heading - before.heading, 2.0 * std::numbers::pi);
      sample.acceleration_x = (now.speed - before.speed) / run.step;
      sample.acceleration_y = now.speed * turn / run.step;
    }
    ego.push_back(sample);
  }
  measures.comfortable =
      model::check_comfort(model::compute_comfort_signals(ego));

  measures.time_to_collision_within_bound = true;
  for (const std::vector<RunSample>& samples : run.samples) {
    const RunSample& own = samples[0];
    model::OrientedBox own_box = convert_sample_to_box(own, run.boxes[0]);
    std::vector<model::MovingBox> tracks;
    double gap = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < samples.size(); ++i) {
      model::OrientedBox box = convert_sample_to_box(samples[i], run.boxes[i]);
      gap = std::min(gap, model::compute_gap(own_box, box));
      if (model::check_ahead(own.x, own.y, own.heading,
                             {.x = box.x, .y = box.y}, AHEAD)) {
        tracks.push_back({.box = box, .speed = samples[i].speed});
      }
    }
    std::optional<double> ttc = model::compute_time_to_collision(
        {.box = own_box, .speed = own.speed}, tracks);
    measures.gaps.push_back(gap);
    measures.min_gap = std::min(measures.min_gap, gap);
    measures.times_to_collision.push_back(ttc);
    if (ttc) {
      measures.min_time_to_collision =
          std::min(*ttc, measures.min_time_to_collision.value_or(*ttc));
      measures.time_to_collision_within_bound &= *ttc > LEAST_TIME_TO_COLLISION;
    }
  }
  return measures;
}

auto run_batch(const scenario::ParameterDistribution& distribution,
               std::size_t threads, framework::Duration step, std::size_t limit)
    -> std::vector<BatchRun> {
  std::size_t count = scenario::count_permutations(distribution);
  std::vector<BatchRun> runs;
  if (count == 0) {
    return runs;
  }
  for (std::size_t i = 0; i < count; ++i) {
    runs.push_back({.permutation = i,
                    .assignments = scenario::find_permutation(distribution, i),
                    .measures = RunMeasures{}});
  }
  std::atomic<std::size_t> next = 0;
  auto work = [&] {
    for (std::size_t i = next++; i < count; i = next++) {
      BatchRun& run = runs[i];
      run.measures =
          record_run(distribution.scenario, run.assignments, step, limit)
              .transform(measure_run);
    }
  };
  {
    std::vector<std::jthread> workers;
    for (std::size_t t = 1; t < std::clamp<std::size_t>(threads, 1, count);
         ++t) {
      workers.emplace_back(work);
    }
    work();
  }
  return runs;
}

}  // namespace simon::automotive
