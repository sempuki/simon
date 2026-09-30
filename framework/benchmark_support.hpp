// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <atomic>
#include <charconv>
#include <cstdint>
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
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

  // The number of contending threads an argument asks for: "--contend" for
  // one per spare core, "--contend=N" for N. Nothing for any other argument,
  // and nothing for a count that is not a number.
  static std::optional<unsigned> threads_from(std::string_view argument) {
    constexpr std::string_view FLAG = "--contend";
    if (argument == FLAG) {
      return spare_cores();
    }
    if (!argument.starts_with(FLAG) || argument.size() <= FLAG.size() + 1 ||
        argument[FLAG.size()] != '=') {
      return std::nullopt;
    }
    std::string_view count = argument.substr(FLAG.size() + 1);
    unsigned threads = 0;
    auto [end, error] =
        std::from_chars(count.data(), count.data() + count.size(), threads);
    if (error != std::errc{} || end != count.data() + count.size()) {
      return std::nullopt;
    }
    return threads;
  }

  // "uncontended", or how many threads contend.
  static std::string describe(unsigned threads) {
    return threads == 0 ? std::string{"uncontended"}
                        : std::format("contended by {} thread{}", threads,
                                      threads == 1 ? "" : "s");
  }

 private:
  std::atomic<bool> stop_ = false;
  std::vector<std::thread> threads_;
};

}  // namespace simon::framework::benchmark
