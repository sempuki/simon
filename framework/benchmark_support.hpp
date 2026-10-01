// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <atomic>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "base/core.hpp"

// Shared by the framework's benchmarks.
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
  static unsigned spare_cores();

  // The number of contending threads an argument asks for: "--contend" for
  // one per spare core, "--contend=N" for N. Nothing for any other argument,
  // and nothing for a count that is not a number.
  static std::optional<unsigned> threads_from(std::string_view argument);

  // "uncontended", or how many threads contend.
  static std::string describe(unsigned threads);

 private:
  std::atomic<bool> stop_ = false;
  std::vector<std::thread> threads_;
};

}  // namespace simon::framework::benchmark
