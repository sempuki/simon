// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "model/aircraft_data.hpp"

#include <string>

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"
#include "catch2/matchers/catch_matchers_string.hpp"

namespace simon::model {

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

namespace {

constexpr char GLIDER[] = R"(simon-aircraft 1
# A comment.
name glider
metrics 10 15 0.7
aero_reference 1 0 0.5
empty_mass 300
empty_inertia 100 200 300 0 -5 0
empty_center_of_mass 1.1 0 0.4
tank 1 0 0 50 40
engine turbine small
  location 2 0 0
  feeds 0
  military_thrust 1000
  idle_thrust
    table mach density_altitude
      columns 0 1000
      0 0.1 0.2
      1 0.3 0.4
    end
  military_thrust_factor
    table mach density_altitude
      columns 0 1000
      0 1 0.8
      1 1.1 0.9
    end
end
term lift CLalpha
  constant 10
  factor dynamic_pressure
  table alpha
    0 0.2
    0.2 1.2
  end
end
)";

}  // namespace

TEST_CASE("ParseAircraft") {
  SECTION("ShouldReadEveryEntryGivenWellFormedText") {
    auto data = parse_aircraft(GLIDER);
    REQUIRE(data);
    CHECK(data->name == "glider");
    CHECK(data->wing_area == 10.0 * square_meter);
    CHECK(data->empty_inertia[4] == -5.0);
    REQUIRE(data->tanks.size() == 1);
    CHECK(data->tanks[0].contents == 40.0 * kilogram);
    REQUIRE(data->engines.size() == 1);
    CHECK(data->engines[0].feeds == std::vector<std::size_t>{0});
    REQUIRE(data->engines[0].idle_thrust);

    AeroInputs inputs;
    inputs[AeroVariable::MACH] = 0.5;
    inputs[AeroVariable::DENSITY_ALTITUDE] = 500.0;
    CHECK_THAT((*data->engines[0].idle_thrust)(inputs), WithinAbs(0.25, 1e-12));

    const auto& lift =
        data->aero.axes[static_cast<std::size_t>(AeroAxis::LIFT)];
    REQUIRE(lift.size() == 1);
    inputs[AeroVariable::DYNAMIC_PRESSURE] = 2.0;
    inputs[AeroVariable::ALPHA] = 0.1;
    CHECK_THAT(lift[0](inputs), WithinAbs(10.0 * 2.0 * 0.7, 1e-12));
  }

  SECTION("ShouldSayWhereGivenUnknownVariable") {
    std::string text = GLIDER;
    text.replace(text.find("factor dynamic_pressure"), 23, "factor wind");
    auto data = parse_aircraft(text);
    REQUIRE_FALSE(data);
    CHECK_THAT(std::string{data.error().message()},
               ContainsSubstring("line 29") && ContainsSubstring("wind"));
  }

  SECTION("ShouldRefuseGivenBreakpointsOutOfOrder") {
    std::string text = GLIDER;
    text.replace(text.find("    0.2 1.2"), 11, "    -1 1.2");
    auto data = parse_aircraft(text);
    REQUIRE_FALSE(data);
    CHECK_THAT(std::string{data.error().message()},
               ContainsSubstring("increasing"));
  }

  SECTION("ShouldRefuseGivenEngineWithoutThrustTable") {
    std::string text = GLIDER;
    std::size_t from = text.find("  military_thrust_factor");
    std::size_t to = text.find("    end\n", from) + 8;
    text.erase(from, to - from);
    auto data = parse_aircraft(text);
    REQUIRE_FALSE(data);
    CHECK_THAT(std::string{data.error().message()},
               ContainsSubstring("both thrust tables"));
  }

  SECTION("ShouldRefuseGivenNoHeader") {
    CHECK_FALSE(parse_aircraft("name glider\n"));
  }
}

}  // namespace simon::model
