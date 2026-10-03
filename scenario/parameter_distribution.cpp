// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "scenario/parameter_distribution.hpp"

namespace simon::scenario {

auto count_permutations(const ParameterDistribution& distribution)
    -> std::size_t {
  if (distribution.distributions.empty()) {
    return 0;
  }
  std::size_t count = 1;
  for (const std::vector<ParameterChoice>& choices :
       distribution.distributions) {
    count *= choices.size();
  }
  return count;
}

// The index in mixed radix, each distribution a digit, the last the least
// significant.
auto find_permutation(const ParameterDistribution& distribution,
                      std::size_t index) -> std::vector<ParameterAssignment> {
  std::vector<std::size_t> digits(distribution.distributions.size());
  for (std::size_t i = digits.size(); i > 0; --i) {
    std::size_t base = distribution.distributions[i - 1].size();
    digits[i - 1] = index % base;
    index /= base;
  }
  std::vector<ParameterAssignment> values;
  for (std::size_t i = 0; i < digits.size(); ++i) {
    const ParameterChoice& choice = distribution.distributions[i][digits[i]];
    values.insert(values.end(), choice.begin(), choice.end());
  }
  return values;
}

}  // namespace simon::scenario
