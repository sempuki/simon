// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "format/mjcf.hpp"

#include <cmath>
#include <numbers>
#include <string>

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_string.hpp"
#include "model/articulated/articulated.hpp"

// Reading MJCF from text: what it compiles, what it refuses, and where a
// failure points. robotic's model_test checks the compiled values against
// MuJoCo's on whole models.
namespace simon::format {

using Catch::Matchers::ContainsSubstring;

TEST_CASE("Mjcf") {
  SECTION("ShouldCompileABodyFromItsGeometryGivenAPendulum") {
    // Preconditions.
    constexpr std::string_view PENDULUM = R"(<mujoco model="pendulum">
      <option timestep="0.01"/>
      <worldbody>
        <body name="bob" pos="0 0 1">
          <joint name="swing" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1"/>
        </body>
      </worldbody>
    </mujoco>)";

    // Under Test.
    auto scene = parse_mjcf(PENDULUM);

    // Postconditions.
    REQUIRE(scene.has_value());
    CHECK(scene->name == "pendulum");
    CHECK(scene->physics.timestep == 0.01);
    REQUIRE(scene->bodies.size() == 2u);  // The world, then the bob.
    const articulated::Body& bob = scene->bodies[1];
    CHECK(bob.name == "bob");
    CHECK(bob.pos.z() == 1.0);
    // A sphere of 0.1 m at 1000 kg/m^3, and its moment, 2/5 m r^2.
    double mass = 1000.0 * 4.0 / 3.0 * std::numbers::pi * 0.001;
    CHECK(std::abs(bob.mass - mass) < 1e-12);
    CHECK(std::abs(bob.inertia.x() - 0.4 * mass * 0.01) < 1e-12);
    REQUIRE(scene->joints.size() == 1u);
    CHECK(scene->joints[0].type == articulated::JointType::HINGE);
    CHECK(scene->joints[0].axis.y() == 1.0);
  }

  SECTION("ShouldApplyDefaultsGivenDefaultClass") {
    // Preconditions.
    constexpr std::string_view DAMPED = R"(<mujoco>
      <default><joint damping="2"/></default>
      <worldbody>
        <body><joint type="hinge"/><geom type="sphere" size="0.1"/></body>
      </worldbody>
    </mujoco>)";

    // Under Test.
    auto scene = parse_mjcf(DAMPED);

    // Postconditions.
    REQUIRE(scene.has_value());
    REQUIRE(scene->dofs.size() == 1u);
    CHECK(scene->dofs[0].damping == 2.0);
  }

  SECTION("ShouldRefuseWhatItDoesNotRunGivenAnEquality") {
    // Preconditions.
    constexpr std::string_view WELDED = R"(<mujoco>
      <worldbody>
        <body name="a"><freejoint/><geom type="sphere" size="0.1"/></body>
      </worldbody>
      <equality><weld body1="a"/></equality>
    </mujoco>)";

    // Under Test.
    auto scene = parse_mjcf(WELDED);

    // Postconditions.
    REQUIRE_FALSE(scene.has_value());
    CHECK_THAT(std::string{scene.error().message()},
               ContainsSubstring("equality"));
  }

  SECTION("ShouldNameTheLineGivenAnUnknownIntegrator") {
    // Preconditions.
    constexpr std::string_view UNKNOWN = R"(<mujoco>
      <option integrator="Bogus"/>
    </mujoco>)";

    // Under Test.
    auto scene = parse_mjcf(UNKNOWN);

    // Postconditions.
    REQUIRE_FALSE(scene.has_value());
    std::string message{scene.error().message()};
    CHECK_THAT(message, ContainsSubstring("Bogus"));
    CHECK_THAT(message, ContainsSubstring("line 2"));
  }
}

}  // namespace simon::format
