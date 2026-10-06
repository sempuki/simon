// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstddef>
#include <vector>

#include "base/core.hpp"
#include "model/random.hpp"
#include "model/units.hpp"

// Galaxies and star clusters as bodies: the mass distributions galaxy codes
// start from, sampled body by body (see model/REFERENCES.md).
namespace simon::model {

// A body as a run starts it.
struct BodyStart final {
  Position position = meters(0.0, 0.0, 0.0);
  Velocity velocity = meters_per_second(0.0, 0.0, 0.0);
  Mass mass = 0.0 * kilogram;
};

//-- Plummer's sphere ----------------------------------------------------------

// Plummer's sphere of `mass` and scale radius `scale`: density proportional
// to (1 + r^2 / a^2)^(-5/2), and isotropic velocities that keep it in
// equilibrium.
struct Plummer final {
  Mass mass = 0.0 * kilogram;
  Length scale = 0.0 * meter;
};

// Computes the radius inside which `fraction` of a Plummer sphere's mass
// lies: a (fraction^(-2/3) - 1)^(-1/2).
auto compute_plummer_radius(const Plummer& plummer, double fraction) -> Length;

// Computes a Plummer sphere's total energy, -3 pi G M^2 / 64 a, unsoftened.
auto compute_plummer_energy(const Plummer& plummer)
    -> units::quantity<units::si::joule, double>;

// Computes a stellar system's crossing time, G M^(5/2) / (-2 E)^(3/2), from
// its mass and energy: 2 sqrt(2) in Heggie and Mathieu's standard units.
auto compute_crossing_time(Mass mass,
                           units::quantity<units::si::joule, double> energy)
    -> Time;

// Appends to `bodies` `count` bodies of equal mass sampled from `plummer` by
// Aarseth, Henon and Wielen's method, then moves them all so that their
// center of mass is at the origin and at rest.
auto append_plummer(const Plummer& plummer, std::size_t count,
                    InOut<Random> random, InOut<std::vector<BodyStart>> bodies)
    -> void;

}  // namespace simon::model
