// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <vector>

#include "base/core.hpp"
#include "framework/system.hpp"
#include "framework/type_list.hpp"

// Shared by simon's benchmarks.
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

// A scheduler of its own for each system, to time them one by one.
template <typename WorldType, typename... SystemTypes>
auto create_schedulers(TypeList<SystemTypes...>) {
  return std::tuple<Scheduler<WorldType, SystemList<SystemTypes>>...>{};
}

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

// The bytes each system's per-entity loop can read for each entity; see
// bytes_per_entity_v.
template <typename... SystemTypes>
auto collect_bytes_per_entity(TypeList<SystemTypes...>)
    -> std::array<std::size_t, sizeof...(SystemTypes)> {
  return {bytes_per_entity_v<SystemTypes>...};
}

}  // namespace simon::framework::benchmark
