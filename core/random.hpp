// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <random>

namespace simon {

// The `n`th output of the SplitMix64 stream `seed` (see model/REFERENCES.md):
// the same everywhere for the same seed, and needing no state but the count.
inline auto compute_split_mix(std::uint64_t seed, std::uint64_t n)
    -> std::uint64_t {
  std::uint64_t z = seed + (n + 1) * 0x9e3779b97f4a7c15ULL;
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  return z ^ (z >> 31);
}

// Two independent standard normal numbers from `u` uniform in (0, 1] and `v`
// uniform over a unit interval, by Box and Muller's transform (see
// model/REFERENCES.md).
inline auto transform_box_muller(double u, double v) -> std::array<double, 2> {
  double radius = std::sqrt(-2.0 * std::log(u));
  double angle = 2.0 * std::numbers::pi * v;
  return {radius * std::cos(angle), radius * std::sin(angle)};
}

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

  // Normal with `mean` and `deviation`, from two uniform numbers.
  auto normal(double mean, double deviation) -> double {
    double u = 1.0 - unit();  // In (0, 1], so its logarithm is finite.
    double v = unit();
    return mean + deviation * transform_box_muller(u, v)[0];
  }

 private:
  std::mt19937_64 engine_;
};

}  // namespace simon
