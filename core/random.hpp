// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>
#include <random>

namespace simon {

// Random numbers that are the same on every platform for the same seed.
// std::mt19937_64's output is fixed by the standard; the standard library's
// distributions are not, so this converts raw bits itself.
class Random final {
 public:
  explicit Random(std::uint64_t seed) : engine_{seed} {}

  // Uniform in [0, 1), with 53 random bits.
  auto unit() -> double {
    return static_cast<double>(engine_() >> 11) * 0x1.0p-53;
  }

  // Uniform in [low, high).
  auto uniform(double low, double high) -> double {
    return low + (high - low) * unit();
  }

  // Normal with `mean` and `deviation`, by Box and Muller's transform of two
  // uniform numbers (see model/REFERENCES.md).
  auto normal(double mean, double deviation) -> double {
    double u = 1.0 - unit();  // In (0, 1], so its logarithm is finite.
    double v = unit();
    return mean + deviation * std::sqrt(-2.0 * std::log(u)) *
                      std::cos(2.0 * std::numbers::pi * v);
  }

 private:
  std::mt19937_64 engine_;
};

}  // namespace simon
