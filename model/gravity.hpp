// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <numbers>
#include <span>
#include <vector>

#include "framework/entity.hpp"
#include "framework/system.hpp"
#include "framework/vocabulary.hpp"
#include "model/kinematics.hpp"
#include "model/units.hpp"

// Newtonian gravity between point masses, softened so that close passes stay
// finite, and the astronomical scales it works at (see model/REFERENCES.md).
namespace simon::model {

//-- Constants -----------------------------------------------------------------

// G, CODATA 2018.
inline constexpr double GRAVITATIONAL_CONSTANT = 6.67430e-11;  // m^3/(kg s^2).

// The astronomical unit, exact by IAU 2012 Resolution B2, and the parsec,
// exactly 648000 / pi of them by IAU 2015 Resolution B2.
inline constexpr Length ASTRONOMICAL_UNIT = 149597870700.0 * meter;
inline constexpr Length PARSEC =
    ASTRONOMICAL_UNIT * (648000.0 / std::numbers::pi);
inline constexpr Length KILOPARSEC = 1000.0 * PARSEC;

// The nominal solar mass parameter GM, exact by IAU 2015 Resolution B3, over
// G: the Sun's mass as far as G is known.
inline constexpr double SOLAR_MASS_PARAMETER = 1.3271244e20;  // m^3/s^2.
inline constexpr Mass SOLAR_MASS =
    SOLAR_MASS_PARAMETER / GRAVITATIONAL_CONSTANT * kilogram;

// The Julian year, 365.25 days of 86,400 s.
inline constexpr Time JULIAN_YEAR = 365.25 * 86400.0 * second;
inline constexpr Time MEGAYEAR = 1e6 * JULIAN_YEAR;

//-- Components ----------------------------------------------------------------

// A body's mass, which pulls on every other body. A body without one feels
// gravity but exerts none: a test particle.
struct PointMass final {
  Mass mass = 0.0 * kilogram;
};

// The acceleration gravity gives a body. A gravity system writes it; the
// leapfrog's kicks read it.
struct Gravity final {
  Acceleration acceleration = meters_per_second_squared(0.0, 0.0, 0.0);
};

//-- Direct summation ----------------------------------------------------------

// A body that pulls on others, as a gravity system gathers it.
struct GravitySource final {
  Vector3 position = Vector3::Zero();  // Meters.
  double mass = 0.0;                   // Kilograms.
  framework::Entity entity;
};

// Computes the acceleration at `position` from every source but `self`, each
// softened by Plummer's kernel: G m d / (|d|^2 + softening^2)^(3/2) (Dehnen),
// in REBOUND's order of operations. Meters, kilograms and seconds.
auto sum_gravity(std::span<const GravitySource> sources, framework::Entity self,
                 const Vector3& position, double softening) -> Vector3;

// Computes the potential energy of `sources`, each pair once, softened as
// sum_gravity softens them: -G m_i m_j / (|d|^2 + softening^2)^(1/2). Joules.
auto compute_potential_energy(std::span<const GravitySource> sources,
                              double softening) -> double;

// Sums every source's pull on each body directly. It costs N^2 and is exact
// to rounding: the reference for faster methods. Each body writes only its
// own Gravity, from sources gathered once a step, so bodies run in any order.
struct SumGravity final           //
    : framework::System<Gravity,  //
                        const Kinematics> {
  using AllowComponentList = framework::TypeList<Kinematics, PointMass>;

  auto prepare(auto& world) -> void {
    sources.clear();
    store_of<PointMass>(world).for_each(
        [&](framework::Entity entity, const PointMass& point) {
          const Kinematics* kinematics =
              maybe_component_of<Kinematics>(world, entity);
          if (!kinematics) return;
          sources.push_back(GravitySource{
              .position =
                  kinematics->position.numerical_value_ref_in(meter).eigen(),
              .mass = point.mass.numerical_value_in(kilogram),
              .entity = entity});
        });
  }

  auto operator()(auto&, framework::Entity self,  //
                  Gravity& gravity,               //
                  const Kinematics* kinematics) const -> void {
    if (!kinematics) return;
    gravity.acceleration =
        QuantityVector{sum_gravity(
            sources, self,
            kinematics->position.numerical_value_ref_in(meter).eigen(),
            softening.numerical_value_in(meter))} *
        meter_per_second_squared;
  }

  Length softening = 0.0 * meter;
  std::vector<GravitySource> sources;
};

}  // namespace simon::model
