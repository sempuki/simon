// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <expected>
#include <optional>
#include <print>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "base/core.hpp"
#include "core/argument.hpp"
#include "core/time.hpp"
#include "framework/system.hpp"
#include "framework/type_list.hpp"

// The harness simon's benchmarks share: their common arguments, contending
// threads, and timing a schedule system by system.
namespace simon::framework::benchmark {

// While alive, runs `threads` threads that stream over buffers much larger
// than any cache, to compete for shared cache and memory bandwidth as a busy
// cloud host would.
class Contention final {
 public:
  DECLARE_COPY_DELETE(Contention);
  DECLARE_MOVE_DELETE(Contention);

  explicit Contention(unsigned threads);
  ~Contention();

  // One thread per core but the benchmark's own.
  static auto spare_cores() -> unsigned;

  // The number of contending threads an argument asks for: "--contend" for
  // one per spare core, "--contend=N" for N. Nothing for any other argument,
  // and nothing for a count that is not a number.
  static auto threads_from(std::string_view argument)
      -> std::optional<unsigned>;

  // "uncontended", or how many threads contend.
  static auto describe(unsigned threads) -> std::string;

 private:
  std::atomic<bool> stop_ = false;
  std::vector<std::thread> threads_;
};

// A whole positive number, or nothing.
auto parse_count(std::string_view text) -> std::optional<int>;

// The arguments every benchmark takes, "--steps N" and "--contend[=N]", and
// the others in order, for the benchmark to read.
struct Arguments final {
  std::optional<int> steps;  // Nothing unless given.
  unsigned threads = 0;      // Contending threads.
  std::vector<std::string_view> rest;
};

// Reads `argv`, or says why it cannot.
auto parse_arguments(int argc, char** argv)
    -> std::expected<Arguments, std::string>;

// The seconds since it was made.
class Stopwatch final {
 public:
  auto seconds() const -> double {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         start_)
        .count();
  }

 private:
  std::chrono::steady_clock::time_point start_ =
      std::chrono::steady_clock::now();
};

// Each system's name, unqualified: a Continuous element by the state it
// integrates, the rest by name.
template <typename... SystemTypes>
auto collect_system_names(TypeList<SystemTypes...>)
    -> std::array<std::string, sizeof...(SystemTypes)> {
  auto unqualified = [](const std::string& name) {
    std::size_t colons = name.rfind("::");
    return colons == std::string::npos ? name : name.substr(colons + 2);
  };
  auto short_name = [&](const std::string& name) {
    std::size_t open = name.find('<');
    std::string head =
        unqualified(open == std::string::npos ? name : name.substr(0, open));
    std::size_t states = name.find("TypeList<");
    if (head == "Continuous" && states != std::string::npos) {
      std::size_t from = states + std::string_view{"TypeList<"}.size();
      std::size_t to = name.find_first_of(",>", from);
      return head + "(" + unqualified(name.substr(from, to - from)) + ")";
    }
    return head;
  };
  return {short_name(lib::to_type_string<SystemTypes>())...};
}

// Times each of `SystemTypes` on its own: a single-system scheduler for
// each, stepped in turn against one world. That is the work of their whole
// schedule, which also syncs after every system. Other work timed with
// `time` gets a row of its own.
template <typename WorldType, typename... SystemTypes>
class SystemTimer final {
 public:
  SystemTimer()
    requires(std::default_initializable<SystemTypes> && ...)
  = default;
  explicit SystemTimer(SystemTypes... systems)
      : schedulers_{SystemList<SystemTypes>{std::move(systems)}...} {}

  // Steps each system once, and returns the seconds the step took. Counts
  // them unless `measured` is false, as while a world settles.
  auto step(const Step& step, InOut<WorldType> world, bool measured = true)
      -> double {
    double elapsed = 0.0;
    std::size_t index = 0;
    std::apply(
        [&](auto&... scheduler) {
          auto time_one = [&](auto& one) {
            Stopwatch stopwatch;
            one.step(step, world);
            double seconds = stopwatch.seconds();
            if (measured) {
              seconds_[index] += seconds;
            }
            elapsed += seconds;
            ++index;
          };
          (time_one(scheduler), ...);
        },
        schedulers_);
    if (measured) {
      slowest_ = std::max(slowest_, elapsed);
      ++steps_;
    }
    return elapsed;
  }

  // Runs `work`, and counts its seconds on the row `name` unless `measured`
  // is false.
  template <typename WorkType>
  auto time(std::string_view name, WorkType&& work, bool measured = true)
      -> void {
    Stopwatch stopwatch;
    std::forward<WorkType>(work)();
    double seconds = stopwatch.seconds();
    if (!measured) {
      return;
    }
    auto row = std::ranges::find(others_, name, &Other::name);
    if (row == others_.end()) {
      others_.push_back(Other{.name = std::string{name}});
      row = std::prev(others_.end());
    }
    row->seconds += seconds;
  }

  // Prints the measured time per step and per entity-step, over
  // `entity_steps` in all, the slowest step, and each row's share, a
  // system's with the bytes its loop can read per entity
  // (bytes_per_entity_v).
  auto print(double entity_steps) const -> void {
    static const auto NAMES = collect_system_names(TypeList<SystemTypes...>{});
    constexpr std::array<std::size_t, sizeof...(SystemTypes)> BYTES{
        bytes_per_entity_v<SystemTypes>...};
    double total = 0.0;
    std::size_t width = 0;
    for (std::size_t i = 0; i < NAMES.size(); ++i) {
      total += seconds_[i];
      width = std::max(width, NAMES[i].size());
    }
    for (const Other& other : others_) {
      total += other.seconds;
      width = std::max(width, other.name.size());
    }
    // Padded by hand: Clang cannot check a dynamic width in libstdc++'s
    // format strings at compile time.
    auto pad = [&](std::string_view name) {
      return std::string{name} + std::string(width - name.size(), ' ');
    };
    double steps = std::max(steps_, 1);
    std::println(
        "  {} {:10.3f} ms/step {:10.1f} ns/entity-step, slowest step {:.3f} ms",
        pad("total"), 1e3 * total / steps,
        1e9 * total / std::max(entity_steps, 1.0), 1e3 * slowest_);
    auto share = [&](double seconds) {
      return total > 0.0 ? 100.0 * seconds / total : 0.0;
    };
    for (std::size_t i = 0; i < NAMES.size(); ++i) {
      std::println("  {} {:10.3f} ms/step {:6.1f}% {:6} B/entity",
                   pad(NAMES[i]), 1e3 * seconds_[i] / steps, share(seconds_[i]),
                   BYTES[i]);
    }
    for (const Other& other : others_) {
      std::println("  {} {:10.3f} ms/step {:6.1f}%", pad(other.name),
                   1e3 * other.seconds / steps, share(other.seconds));
    }
  }

 private:
  struct Other final {
    std::string name;
    double seconds = 0.0;
  };

  std::tuple<Scheduler<WorldType, SystemList<SystemTypes>>...> schedulers_;
  std::array<double, sizeof...(SystemTypes)> seconds_{};
  std::vector<Other> others_;
  double slowest_ = 0.0;
  int steps_ = 0;
};

template <typename WorldType, typename ListType>
struct SystemTimerForList;

template <typename WorldType, typename... SystemTypes>
struct SystemTimerForList<WorldType, TypeList<SystemTypes...>> final {
  using type = SystemTimer<WorldType, SystemTypes...>;
};

// The timer for every system of a list, such as a scheduler's
// FlattenedSystemList.
template <typename WorldType, typename ListType>
using SystemTimerFor = typename SystemTimerForList<WorldType, ListType>::type;

}  // namespace simon::framework::benchmark
