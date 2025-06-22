#include "base/status.hpp"

#include <type_traits>

#include "base/testing.hpp"

namespace simon {

TEST_CASE("BitCode") {
  SECTION("ShouldPass") {
    impl::BitCode code;
    REQUIRE(true);
  }
}

}  // namespace simon
