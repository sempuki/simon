// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "framework/benchmarking.hpp"

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <format>
#include <system_error>

namespace simon::framework::benchmark {

Contention::Contention(unsigned threads) {
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

Contention::~Contention() {
  stop_ = true;
  for (std::thread& thread : threads_) {
    thread.join();
  }
}

auto Contention::spare_cores() -> unsigned {
  return std::max(1u, std::thread::hardware_concurrency()) - 1;
}

auto Contention::threads_from(std::string_view argument)
    -> std::optional<unsigned> {
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

auto Contention::describe(unsigned threads) -> std::string {
  return threads == 0 ? std::string{"uncontended"}
                      : std::format("contended by {} thread{}", threads,
                                    threads == 1 ? "" : "s");
}

auto parse_count(std::string_view text) -> std::optional<int> {
  int count = 0;
  auto [end, error] =
      std::from_chars(text.data(), text.data() + text.size(), count);
  if (error != std::errc{} || end != text.data() + text.size() || count <= 0) {
    return std::nullopt;
  }
  return count;
}

auto parse_arguments(int argc, char** argv)
    -> std::expected<Arguments, std::string> {
  Arguments arguments;
  for (int i = 1; i < argc; ++i) {
    std::string_view argument{argv[i]};
    if (argument == "--steps") {
      std::optional<int> count =
          i + 1 < argc ? parse_count(argv[i + 1]) : std::nullopt;
      if (!count) {
        return std::unexpected(
            std::string{"--steps needs a whole positive number"});
      }
      arguments.steps = *count;
      ++i;
    } else if (std::optional<unsigned> threads =
                   Contention::threads_from(argument)) {
      arguments.threads = *threads;
    } else {
      arguments.rest.push_back(argument);
    }
  }
  return arguments;
}

}  // namespace simon::framework::benchmark
