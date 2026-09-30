// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
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

  explicit Contention(unsigned threads) {
    for (unsigned i = 0; i < threads; ++i) {
      threads_.emplace_back([this] {
        std::vector<std::uint64_t> buffer(32'000'000, 1);  // 256 MB.
        std::uint64_t sum = 0;
        while (!stop_.load(std::memory_order_relaxed)) {
          for (std::uint64_t& value : buffer) {
            sum += value;
            value = sum;
          }
        }
        DECLARE_UNUSED(sum);
      });
    }
  }
  ~Contention() {
    stop_ = true;
    for (std::thread& thread : threads_) {
      thread.join();
    }
  }

  // One thread per core but the benchmark's own.
  static unsigned spare_cores() {
    return std::max(1u, std::thread::hardware_concurrency()) - 1;
  }

 private:
  std::atomic<bool> stop_ = false;
  std::vector<std::thread> threads_;
};

}  // namespace simon::framework::benchmark
