// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Times a simulation with nothing to do on most steps, idling and not:
//
//   bazel run -c opt //engine:idle_benchmark
//
// A thousand bodies, each holding a timer that fires every 1 to 60 s, and two
// systems over them at 1 s and 5 s, run for an hour at a 10 ms maximum step.
// Nothing runs every step, so an idling driver steps only to due times. Both
// runs must end in the same state.

#include <chrono>
#include <cstdint>
#include <expected>
#include <print>
#include <vector>

#include "base/core.hpp"
#include "core/argument.hpp"
#include "core/time.hpp"
#include "engine/driver.hpp"
#include "engine/event_queue.hpp"
#include "engine/lifecycle.hpp"
#include "framework/benchmarking.hpp"
#include "framework/system.hpp"
#include "framework/test_world.hpp"
#include "framework/timeline.hpp"

namespace simon::engine {
namespace {

using namespace std::chrono_literals;
using framework::Entity;
using framework::System;
using framework::SystemList;
using framework::testing::Body;
using framework::testing::Health;
using framework::testing::TestWorld;

constexpr std::size_t BODIES = 1000;

// Each body's health grows by a point a second.
struct Grow final : System<Health> {
  using SystemWorld = framework::ProjectedWorld<Grow, TestWorld>;
  static constexpr Duration PERIOD = 1s;
  auto operator()(SystemWorld&, Entity, Health& health, Step step) const
      -> void {
    health.points += std::chrono::duration<double>(step.dt).count();
  }
};

// And loses a hundredth of it every 5 s.
struct Decay final : System<Health> {
  using SystemWorld = framework::ProjectedWorld<Decay, TestWorld>;
  using SequenceAfterSystemList = SystemList<Grow>;
  static constexpr Duration PERIOD = 5s;
  auto operator()(SystemWorld&, Entity, Health& health) const -> void {
    health.points *= 0.99;
  }
};

class IdleSimulation final {
 public:
  auto configure() -> PhaseResult {
    std::expected<void, framework::Status> built =
        TestWorld::set_up().numbered(1).holding<Body>(BODIES).build(
            Out(world_));
    RETURN_IF_UNEXPECTED(built);
    for (std::size_t i = 0; i < BODIES; ++i) {
      RETURN_IF_UNEXPECTED(world_.create<Body>().with(Health{}).build());
    }
    world_.sync();
    scheduler_.attach(Depend(timeline_));
    events_.attach(Depend(timeline_));
    fired_.assign(BODIES, 0);
    for (std::size_t i = 0; i < BODIES; ++i) {
      schedule(i, TimePoint{} + period_of(i));
    }
    return Flow::CONTINUE;
  }

  auto step(const Step& step) -> PhaseResult {
    ++steps_;
    events_.process_until(step.time);
    scheduler_.step(step, InOut(world_));
    return Flow::CONTINUE;
  }

  auto timeline() const -> const framework::Timeline& { return timeline_; }

  auto steps() const -> std::uint64_t { return steps_; }

  // Every body's health and every timer's firings, summed.
  auto checksum() const -> double {
    double sum = 0.0;
    world_.store_of<Health>().for_each(
        [&](Entity, const Health& health) { sum += health.points; });
    for (std::uint64_t fired : fired_) {
      sum += static_cast<double>(fired);
    }
    return sum;
  }

 private:
  static auto period_of(std::size_t i) -> Duration {
    return std::chrono::seconds{1 + static_cast<std::int64_t>(i % 60)};
  }

  auto schedule(std::size_t i, TimePoint time) -> void {
    events_.start_timer(time, [this, i](TimePoint now) {
      ++fired_[i];
      schedule(i, now + period_of(i));
    });
  }

  TestWorld world_;
  framework::Timeline timeline_;
  framework::Scheduler<TestWorld, SystemList<Grow, Decay>> scheduler_;
  EventQueue events_;
  std::vector<std::uint64_t> fired_;
  std::uint64_t steps_ = 0;
};

struct Result final {
  std::uint64_t steps = 0;
  double seconds = 0.0;
  double checksum = 0.0;
};

auto measure(bool idle) -> Result {
  IdleSimulation simulation;
  BatchDriver driver{Timing{.max_step = 10ms, .idle = idle},
                     Depend(simulation)};
  framework::benchmark::Stopwatch stopwatch;
  std::expected<TimePoint, Status> end = driver.run(TimePoint{1h});
  CHECK_POSTCONDITION(end.has_value());
  return Result{.steps = simulation.steps(),
                .seconds = stopwatch.seconds(),
                .checksum = simulation.checksum()};
}

}  // namespace
}  // namespace simon::engine

auto main() -> int {
  using simon::engine::measure;
  auto busy = measure(false);
  auto idle = measure(true);
  std::println("1000 bodies on 1 to 60 s timers, systems at 1 s and 5 s, 1 h:");
  std::println("  every 10 ms  {:>9} steps {:>9.1f} ms", busy.steps,
               1e3 * busy.seconds);
  std::println("  idling       {:>9} steps {:>9.1f} ms", idle.steps,
               1e3 * idle.seconds);
  std::println("  same final state: {}",
               busy.checksum == idle.checksum ? "yes" : "NO");
  return busy.checksum == idle.checksum ? 0 : 1;
}
