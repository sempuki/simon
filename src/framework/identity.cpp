// Copyright 2022 -- CONTRIBUTORS.

#include "framework/identity.hpp"

#include <algorithm>

namespace simon::framework {
namespace {
template <std::size_t BitSize>
struct LcgSeedSequence {
  using result_type = std::uint32_t;
  static_assert((BitSize % (sizeof(result_type) * 8)) == 0);

  LcgSeedSequence()
    : seeder{device()} {}  // https://www.pcg-random.org/posts/cpp-seeding-surprises.html

  template <typename I>
  void generate(I begin, I end) {
    std::generate(begin, end, [this] { return uniform(seeder); });
  }

  // https://onlinelibrary.wiley.com/doi/10.1002/spe.3030
  std::random_device device;
  std::linear_congruential_engine<result_type, 0x915f77f5, 0x3c6ef35f, 0U> seeder;
  std::uniform_int_distribution<result_type> uniform;
};

std::uint64_t random_uniform_64_slow() {
  // Must seed 19937 bits of internal state.
  return std::uniform_int_distribution<std::uint64_t>{}(std::mt19937_64{LcgSeedSequence<19937>{}});
}
}  // namespace

Identity::Identity() : id_{random_uniform_64_slow()} {}

}  // namespace simon::framework

