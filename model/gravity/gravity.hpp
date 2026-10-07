// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <vector>

#include "core/units.hpp"
#include "core/vocabulary.hpp"

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

// The acceleration gravity gives a body.
struct Gravity final {
  Acceleration acceleration = meters_per_second_squared(0.0, 0.0, 0.0);
};

//-- Direct summation ----------------------------------------------------------

// A body that pulls on others. `id` is the caller's name for the body, which
// leaves it out of its own pull.
struct GravitySource final {
  Vector3 position = Vector3::Zero();  // Meters.
  double mass = 0.0;                   // Kilograms.
  std::uint32_t id = 0;
};

// Computes the acceleration at `position` from every source but the one
// whose id is `self`, each
// softened by Plummer's kernel: G m d / (|d|^2 + softening^2)^(3/2) (Dehnen),
// in REBOUND's order of operations. Meters, kilograms and seconds.
auto sum_gravity(std::span<const GravitySource> sources, std::uint32_t self,
                 const Vector3& position, double softening) -> Vector3;

// Computes the potential energy of `sources`, each pair once, softened as
// sum_gravity softens them: -G m_i m_j / (|d|^2 + softening^2)^(1/2). Joules.
auto compute_potential_energy(std::span<const GravitySource> sources,
                              double softening) -> double;

//-- Barnes and Hut's tree -----------------------------------------------------

// An octree over gravity sources, each cell holding its mass and center of
// mass, after Barnes and Hut. A body far from a cell takes the cell's pull as
// that of one point of its mass at its center of mass, so a body's
// acceleration costs about log N cells instead of N sources.
class GravityTree final {
 public:
  // Rebuilds the tree over `sources`: a cube around them, split into octants
  // until each cell holds one source, or several that cannot be told apart.
  auto build(std::span<const GravitySource> sources) -> void;

  // Computes the acceleration at `position` on `self`, opening each cell
  // whose width is more than `opening_angle` times its distance from
  // `position` to its center of mass, as REBOUND's tree does. An unopened cell
  // pulls as a point; a source pulls as sum_gravity has it. Meters, kilograms
  // and seconds.
  auto compute_acceleration(std::uint32_t self, const Vector3& position,
                            double opening_angle, double softening) const
      -> Vector3;

  auto cells() const -> std::size_t { return cells_.size(); }

 private:
  struct Cell final {
    Vector3 center_of_mass = Vector3::Zero();
    double mass = 0.0;
    double width = 0.0;
    std::uint32_t first = 0;  // First child cell, or first source of a leaf.
    std::uint32_t count = 0;  // Child cells, or sources of a leaf.
    bool leaf = false;
  };

  auto build_cell(std::uint32_t cell, std::uint32_t begin, std::uint32_t end,
                  const Vector3& corner, double width, int depth) -> void;

  std::vector<GravitySource> sources_;  // In tree order.
  std::vector<GravitySource> scratch_;
  std::vector<Cell> cells_;
};

}  // namespace simon::model
