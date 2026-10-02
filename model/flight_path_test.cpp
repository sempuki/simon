// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "model/flight_path.hpp"

#include <chrono>
#include <cmath>
#include <numbers>

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"

namespace simon::model {

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using namespace std::chrono_literals;

namespace {
constexpr double G = 9.80665;
constexpr double PI = std::numbers::pi;

const Airframe JET{.mass = 20000.0 * kilogram,
                   .wing_area = 50.0 * square_meter,
                   .zero_lift_drag = 0.02,
                   .induced_drag = 0.045,
                   .thrust = 100000.0 * newton};

auto level(double speed, double heading) -> AirState {
  return AirState{.position = meters(0.0, 0.0, 5000.0),
                  .speed = speed * meter_per_second,
                  .heading = heading * radian};
}
}  // namespace

TEST_CASE("PointMassRate") {
  Air air = standard_air(5000.0 * meter);

  SECTION("ShouldHoldPathGivenLevelUnbankedFlight") {
    AirStateRate rate =
        point_mass_rate(level(200.0, 0.0), FlightControls{}, JET, air);

    CHECK_THAT(rate.climb.numerical_value_in(radian_per_second),
               WithinAbs(0.0, 1e-12));
    CHECK_THAT(rate.turn.numerical_value_in(radian_per_second),
               WithinAbs(0.0, 1e-12));
    CHECK(rate.velocity.numerical_value_in(meter_per_second)
              .is_approximately(QuantityVector{0.0, 200.0, 0.0}));
    // With no thrust, drag decelerates.
    CHECK(rate.acceleration < 0.0 * meter_per_second_squared);
  }

  SECTION("ShouldTurnAtCoordinatedRateGivenBankAndMatchingLoadFactor") {
    double bank = PI / 3.0;
    FlightControls controls{.load_factor = 1.0 / std::cos(bank),
                            .bank = bank * radian};
    AirStateRate rate = point_mass_rate(level(200.0, 0.0), controls, JET, air);

    CHECK_THAT(rate.climb.numerical_value_in(radian_per_second),
               WithinAbs(0.0, 1e-12));
    CHECK_THAT(rate.turn.numerical_value_in(radian_per_second),
               WithinRel(G * std::tan(bank) / 200.0, 1e-12));
  }

  SECTION("ShouldBalanceThrustAndDragGivenTrimThrottle") {
    // Drag at 200 m/s and 5 km, in level flight, from the drag polar.
    double rho = air.density.numerical_value_in(kilogram_per_cubic_meter);
    double qs = 0.5 * rho * 200.0 * 200.0 * 50.0;
    double cl = 20000.0 * G / qs;
    double drag = qs * (0.02 + 0.045 * cl * cl);
    double throttle = drag / (100000.0 * rho / 1.225);
    FlightControls controls{.throttle = throttle};

    AirStateRate rate = point_mass_rate(level(200.0, 0.0), controls, JET, air);

    CHECK_THAT(rate.acceleration.numerical_value_in(meter_per_second_squared),
               WithinAbs(0.0, 1e-9));
  }
}

TEST_CASE("Fly") {
  SECTION("ShouldMoveAlongHeadingGivenSteadyFlight") {
    AirState state = level(100.0, PI / 2.0);  // East.
    AirStateRate rate{.velocity = compute_velocity(state)};

    AirState next = fly(state, rate, 2s);

    CHECK(next.position.numerical_value_in(meter).is_approximately(
        QuantityVector{200.0, 0.0, 5000.0}, 1e-9));
  }

  SECTION("ShouldMoveAlongNewVelocityGivenClimbingTurn") {
    // The position follows the velocity at the end of the step, to first
    // order in the step.
    AirState state = level(200.0, 0.4);
    state.flight_path_angle = 0.1 * radian;
    FlightControls controls{
        .load_factor = 1.5, .bank = 0.6 * radian, .throttle = 0.5};
    AirStateRate rate =
        point_mass_rate(state, controls, JET, standard_air(5000.0 * meter));

    AirState next = fly(state, rate, 20ms);
    QuantityVector expected =
        (state.position + compute_velocity(next) * (0.02 * second))
            .numerical_value_in(meter);

    CHECK(next.position.numerical_value_in(meter).is_approximately(expected,
                                                                   1e-3));
  }

  SECTION("ShouldWrapHeadingGivenTurnPastSouth") {
    AirState state = level(100.0, 3.0);
    AirStateRate rate{.turn = 0.5 * radian_per_second};

    AirState next = fly(state, rate, 1s);

    CHECK_THAT(radians(next.heading), WithinAbs(3.5 - 2.0 * PI, 1e-12));
  }

  SECTION("ShouldMatchAdvanceGivenNoChangeInVelocity") {
    AirState state = level(100.0, 0.3);
    AirStateRate rate{.velocity = compute_velocity(state)};

    AirState flown = fly(state, rate, 500ms);
    AirState advanced = advance(state, rate, 500ms);

    CHECK(flown.position.numerical_value_in(meter).is_approximately(
        advanced.position.numerical_value_in(meter), 1e-9));
  }
}

TEST_CASE("AutopilotLaws") {
  SECTION("ShouldClimbAtMostSteepestGivenFarBelowAltitude") {
    Angle gamma = climb_command(level(200.0, 0.0), 9000.0 * meter,
                                0.2 * per_second, 0.25 * radian);
    CHECK(radians(gamma) == 0.25);
  }

  SECTION("ShouldClimbGentlyGivenNearAltitude") {
    Angle gamma = climb_command(level(200.0, 0.0), 5100.0 * meter,
                                0.2 * per_second, 0.25 * radian);
    CHECK_THAT(radians(gamma), WithinRel(std::asin(20.0 / 200.0), 1e-12));
  }

  SECTION("ShouldHoldOneGGivenLevelPathAndNoBank") {
    double n = load_factor_command(level(200.0, 0.0), 0.0 * radian,
                                   0.0 * radian, 1.0 * per_second);
    CHECK_THAT(n, WithinAbs(1.0, 1e-12));
  }

  SECTION("ShouldTurnTheShortWayGivenTargetAcrossSouth") {
    // From 170 degrees to -170 degrees is 20 degrees to the right.
    Angle bank = bank_command(level(200.0, 170.0 * PI / 180.0),
                              -170.0 * PI / 180.0 * radian, 0.5 * per_second,
                              1.0 * radian);
    CHECK(radians(bank) > 0.0);
  }

  SECTION("ShouldMeasureBearingFromNorthTowardEast") {
    CHECK_THAT(radians(bearing(meters(0, 0, 0), meters(10, 0, 0))),
               WithinAbs(PI / 2.0, 1e-12));
    CHECK_THAT(radians(bearing(meters(0, 0, 0), meters(0, 10, 0))),
               WithinAbs(0.0, 1e-12));
    CHECK(ground_distance(meters(0, 0, 0), meters(3, 4, 100)) == 5.0 * meter);
  }
}

}  // namespace simon::model
