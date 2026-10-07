// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/aircraft/aerodynamics.hpp"

#include <array>
#include <numbers>

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"

namespace simon::model {

using Catch::Matchers::WithinAbs;

TEST_CASE("AeroTerm") {
  AeroInputs inputs;
  inputs[AeroVariable::DYNAMIC_PRESSURE] = 1000.0;
  inputs[AeroVariable::ALPHA] = 0.1;
  inputs[AeroVariable::MACH] = 0.5;

  SECTION("ShouldMultiplyConstantByFactors") {
    AeroTerm term{.constant = 2.0,
                  .factors = {aero_input(AeroVariable::DYNAMIC_PRESSURE),
                              aero_input(AeroVariable::ALPHA)}};
    CHECK_THAT(term(inputs), WithinAbs(200.0, 1e-12));
  }

  SECTION("ShouldReadSignalsGivenSignalAndMagnitudeFactors") {
    std::array<double, 3> signals{7.0, 0.5, -0.25};
    std::array<AeroSignal, 2> read{AeroSignal{.signal = 1},
                                   AeroSignal{.signal = 2, .magnitude = true}};
    inputs.read(read, signals);
    AeroTerm term{.constant = 2.0,
                  .factors = {AeroInput{.index = AERO_VARIABLE_COUNT},
                              AeroInput{.index = AERO_VARIABLE_COUNT + 1}}};
    CHECK_THAT(term(inputs), WithinAbs(2.0 * 0.5 * 0.25, 1e-12));
  }

  SECTION("ShouldMultiplyByTablesGivenTablesOfOneAndTwoVariables") {
    AeroTerm term{
        .constant = 1.0,
        .tables = {
            AeroTable{.row = aero_input(AeroVariable::ALPHA),
                      .table = Table1<>{{0.0, 0.2}, {0.0, 2.0}}},
            AeroTable{.row = aero_input(AeroVariable::MACH),
                      .column = aero_input(AeroVariable::ALPHA),
                      .table = Table2<>{{0.0, 1.0}, {0.0, 0.2}, {1, 1, 3, 3}}},
        }};
    // 1.0 from the first table, times 2.0 from the second.
    CHECK_THAT(term(inputs), WithinAbs(2.0, 1e-12));
  }
}

TEST_CASE("AeroModel") {
  SECTION("ShouldSumTermsByAxis") {
    AeroModel model;
    model.axes[static_cast<std::size_t>(AeroAxis::LIFT)] = {
        AeroTerm{.constant = 3.0}, AeroTerm{.constant = 4.0}};
    model.axes[static_cast<std::size_t>(AeroAxis::YAW)] = {
        AeroTerm{.constant = -1.0}};

    AeroSums sums = model(AeroInputs{});
    CHECK(sums[static_cast<std::size_t>(AeroAxis::LIFT)] == 7.0);
    CHECK(sums[static_cast<std::size_t>(AeroAxis::YAW)] == -1.0);
    CHECK(sums[static_cast<std::size_t>(AeroAxis::DRAG)] == 0.0);
  }
}

TEST_CASE("AeroLoads") {
  AeroSums sums{};
  sums[static_cast<std::size_t>(AeroAxis::DRAG)] = 1.0;
  sums[static_cast<std::size_t>(AeroAxis::LIFT)] = 10.0;

  SECTION("ShouldPointDragBackAndLiftUpGivenNoAngles") {
    AeroLoads loads = compute_aero_loads(sums, 0.0 * radian, 0.0 * radian,
                                         meters(0.0, 0.0, 0.0));
    CHECK(loads.force.numerical_value_in(newton).is_approximately(
        QuantityVector{-1.0, 0.0, -10.0}));
  }

  SECTION("ShouldTiltLiftForwardGivenAngleOfAttack") {
    double alpha = std::numbers::pi / 2.0;
    AeroLoads loads = compute_aero_loads(sums, alpha * radian, 0.0 * radian,
                                         meters(0.0, 0.0, 0.0));
    // The air comes from below: drag pushes up, lift pushes forward.
    CHECK(loads.force.numerical_value_in(newton).is_approximately(
        QuantityVector{10.0, 0.0, -1.0}));
  }

  SECTION("ShouldPitchUpGivenLiftAheadOfCenterOfMass") {
    AeroLoads loads = compute_aero_loads(sums, 0.0 * radian, 0.0 * radian,
                                         meters(2.0, 0.0, 0.0));
    CHECK_THAT(loads.moment.numerical_value_in(newton_meter).eigen().y(),
               WithinAbs(20.0, 1e-12));
  }
}

}  // namespace simon::model
