// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/identity.hpp"

#include <algorithm>
#include <random>

namespace simon::framework {
namespace {
// https://www.pcg-random.org/posts/cpp-seeding-surprises.html
struct Lcg32SeedSequence {
  using result_type = std::uint32_t;
  Lcg32SeedSequence() : seeder{device()} {}

  template <typename I>
  void generate(I begin, I end) {
    std::generate(begin, end, [this] {
      // Assume the random device has finite but continual entropy.
      if ((count++ % 13) == 0) {
        seeder.seed(device());
      }
      return uniform(seeder);
    });
  }

  std::random_device device;
  // https://onlinelibrary.wiley.com/doi/10.1002/spe.3030
  std::linear_congruential_engine<result_type, 0x915f77f5, 0x3c6ef35f, 0U> seeder;
  std::uniform_int_distribution<result_type> uniform;
  size_t count = 0;
};

std::uint64_t random_uniform_64_slow() {
  // Must seed 19937 bits of internal state.
  Lcg32SeedSequence seeder;
  std::mt19937_64 generate_64{seeder};
  std::uniform_int_distribution<std::uint64_t> uniform;
  return uniform(generate_64);
}
}  // namespace

Identity::Identity() : id_{random_uniform_64_slow()} {}

}  // namespace simon::framework

