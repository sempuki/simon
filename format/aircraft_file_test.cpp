// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "format/aircraft_file.hpp"

#include <optional>
#include <string>

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"
#include "catch2/matchers/catch_matchers_string.hpp"

namespace simon::format {

using namespace model;
using namespace aircraft;

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
    // Preconditions.
    AeroInputs inputs;
    inputs[AeroVariable::MACH] = 0.5;
    inputs[AeroVariable::DENSITY_ALTITUDE] = 500.0;
    inputs[AeroVariable::DYNAMIC_PRESSURE] = 2.0;
    inputs[AeroVariable::ALPHA] = 0.1;

    // Under Test.
    auto data = parse_aircraft(GLIDER);

    // Postconditions.
    REQUIRE(data);
    CHECK(data->name == "glider");
    CHECK(data->wing_area == 10.0 * square_meter);
    CHECK(data->empty_inertia[4] == -5.0);
    REQUIRE(data->tanks.size() == 1);
    CHECK(data->tanks[0].contents == 40.0 * kilogram);
    REQUIRE(data->engines.size() == 1);
    CHECK(data->engines[0].feeds == std::vector<std::size_t>{0});
    REQUIRE(data->engines[0].idle_thrust);
    CHECK_THAT((*data->engines[0].idle_thrust)(inputs), WithinAbs(0.25, 1e-12));
    const auto& lift =
        data->aero.axes[static_cast<std::size_t>(AeroAxis::LIFT)];
    REQUIRE(lift.size() == 1);
    CHECK_THAT(lift[0](inputs), WithinAbs(10.0 * 2.0 * 0.7, 1e-12));
  }

  SECTION("ShouldReadSignalsGivenFlightControlsBeforeTerms") {
    // Preconditions.
    std::string text = GLIDER;
    text.replace(text.find("term lift"), 0, R"(flight_controls
  block pure_gain flap
    input flaps_command
    gain 0.5
  end
end
term drag CDflap
  constant 3
  factor flap
  factor |flap|
end
)");

    // Under Test.
    auto data = parse_aircraft(text);

    // Postconditions.
    REQUIRE(data);
    std::optional<std::size_t> flap =
        find_signal(data->flight_controls, "flap");
    REQUIRE(flap);
    FlightSignals signals;
    signals.values[*flap] = -0.5;
    const auto& drag =
        data->aero.axes[static_cast<std::size_t>(AeroAxis::DRAG)];
    REQUIRE(drag.size() == 1);
    AeroInputs inputs;
    inputs.read(data->aero.signals, signals.values);
    CHECK_THAT(drag[0](inputs), WithinAbs(3.0 * -0.5 * 0.5, 1e-12));
  }

  SECTION("ShouldReadSwitchesPidsAndFunctionsGivenTheirEntries") {
    // Preconditions.
    std::string text = GLIDER;
    text.replace(text.find("term lift"), 0, R"(flight_controls
  signal override
  block switch pick
    default 0.5
    test or -alpha
      condition mach gt 0.9
      condition override eq 1
    end
  end
  block pid hold
    input pick 2
    trigger override
    kp 3
    ki 0.5
    integrator trap
  end
  block function product
    push alpha
    cos
    constant 2
    product 2
  end
end
)");

    // Under Test.
    auto data = parse_aircraft(text);

    // Postconditions.
    REQUIRE(data);
    const FlightControlData& controls = data->flight_controls;
    REQUIRE(controls.blocks.size() == 3);
    const FlightBlock& pick = controls.blocks[0];
    REQUIRE(pick.tests.size() == 1);
    CHECK(pick.tests[0].any);
    CHECK(pick.tests[0].value.input->negated);
    CHECK(pick.tests[0].conditions.size() == 2);
    CHECK(pick.fallback.value == 0.5);
    const FlightBlock& hold = controls.blocks[1];
    CHECK(hold.inputs[0].scale == 2.0);
    CHECK(hold.integrator == FlightBlock::Integrator::TRAPEZOIDAL);
    CHECK(controls.signals[hold.state] == "hold/integral");
    CHECK(controls.blocks[2].operations.size() == 4);
  }

  SECTION("ShouldRefuseGivenFunctionThatLeavesTwoValues") {
    // Preconditions.
    std::string text = GLIDER;
    text.replace(text.find("term lift"), 0, R"(flight_controls
  block function both
    push alpha
    push beta
  end
end
)");

    // Under Test.
    auto data = parse_aircraft(text);

    // Postconditions.
    REQUIRE_FALSE(data);
    CHECK_THAT(std::string{data.error().message()},
               ContainsSubstring("leave one value"));
  }

  SECTION("ShouldSayWhereGivenUnknownVariable") {
    // Preconditions.
    std::string text = GLIDER;
    text.replace(text.find("factor dynamic_pressure"), 23, "factor wind");

    // Under Test.
    auto data = parse_aircraft(text);

    // Postconditions.
    REQUIRE_FALSE(data);
    CHECK_THAT(std::string{data.error().message()},
               ContainsSubstring("line 29") && ContainsSubstring("wind"));
  }

  SECTION("ShouldRefuseGivenBreakpointsOutOfOrder") {
    // Preconditions.
    std::string text = GLIDER;
    text.replace(text.find("    0.2 1.2"), 11, "    -1 1.2");

    // Under Test.
    auto data = parse_aircraft(text);

    // Postconditions.
    REQUIRE_FALSE(data);
    CHECK_THAT(std::string{data.error().message()},
               ContainsSubstring("increasing"));
  }

  SECTION("ShouldRefuseGivenNonFiniteNumber") {
    // Preconditions.
    std::string text = GLIDER;
    text.replace(text.find("    0.2 1.2"), 11, "    nan 1.2");

    // Under Test.
    auto data = parse_aircraft(text);

    // Postconditions.
    REQUIRE_FALSE(data);
    CHECK_THAT(std::string{data.error().message()},
               ContainsSubstring("finite"));
  }

  SECTION("ShouldRefuseGivenEngineWithoutThrustTable") {
    // Preconditions.
    std::string text = GLIDER;
    std::size_t from = text.find("  military_thrust_factor");
    std::size_t to = text.find("    end\n", from) + 8;
    text.erase(from, to - from);

    // Under Test.
    auto data = parse_aircraft(text);

    // Postconditions.
    REQUIRE_FALSE(data);
    CHECK_THAT(std::string{data.error().message()},
               ContainsSubstring("both thrust tables"));
  }

  SECTION("ShouldRefuseGivenNoHeader") {
    // Postconditions.
    CHECK_FALSE(parse_aircraft("name glider\n"));
  }
}

}  // namespace simon::format
