// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <random>

namespace simon::model {

// Random numbers that are the same on every platform for the same seed.
// std::mt19937_64's output is fixed by the standard; the standard library's
// distributions are not, so this converts raw bits itself.
class Random final {
 public:
  explicit Random(std::uint64_t seed) : engine_{seed} {}

  // Uniform in [0, 1), with 53 random bits.
  double unit() { return static_cast<double>(engine_() >> 11) * 0x1.0p-53; }

  // Uniform in [low, high).
  double uniform(double low, double high) {
    return low + (high - low) * unit();
  }

 private:
  std::mt19937_64 engine_;
};

}  // namespace simon::model
