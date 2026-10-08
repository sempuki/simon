// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstddef>
#include <span>
#include <vector>

#include "base/core.hpp"
#include "core/argument.hpp"
#include "core/math.hpp"
#include "core/random.hpp"
#include "core/units.hpp"

// Galaxies and star clusters as bodies: the mass distributions galaxy codes
// start from, sampled body by body (see model/REFERENCES.md).
namespace simon::gravity {

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

//-- Disk galaxies -------------------------------------------------------------

// A disk galaxy after Hernquist (1993): an exponential disk of stars,
// Sigma(R) = M_d / (2 pi h^2) exp(-R / h), sech^2(z / z_0) thick, inside a
// halo of dark matter with Hernquist's (1990) profile, M(r) = M_h r^2 / (r +
// a)^2, cut off at `halo_cutoff`. It has no bulge. The disk turns
// counterclockwise about z.
struct DiskGalaxy final {
  Mass disk_mass = 0.0 * kilogram;
  Length disk_scale = 0.0 * meter;      // h.
  Length disk_thickness = 0.0 * meter;  // z_0.
  Mass halo_mass = 0.0 * kilogram;      // Without the cutoff.
  Length halo_scale = 0.0 * meter;      // a.
  Length halo_cutoff = 0.0 * meter;
  // Toomre's Q at `stability_radius`, which sets the disk's radial
  // dispersion everywhere.
  double stability = 1.5;
  Length stability_radius = 0.0 * meter;
  std::size_t disk_bodies = 0;
  std::size_t halo_bodies = 0;
};

// Computes the speed of a circular orbit of radius `radius` in the galaxy's
// plane: the halo's G M(R) / R and Freeman's thin exponential disk,
// 4 pi G Sigma_0 h y^2 [I0 K0 - I1 K1](y) with y = R / 2h.
auto compute_circular_speed(const DiskGalaxy& galaxy, Length radius) -> Speed;

// Appends to `bodies` the galaxy's disk, then its halo, sampled with
// Hernquist's (1993) velocities: the halo's from the Jeans equation in the
// halo's and disk's mass, the disk's from its surface density, Toomre's Q,
// the epicyclic approximation and asymmetric drift. The disk's center of mass
// and the halo's are each at the origin and at rest.
auto append_disk_galaxy(const DiskGalaxy& galaxy, InOut<Random> random,
                        InOut<std::vector<BodyStart>> bodies) -> void;

// Turns `bodies` from `first` on by `rotation`, then moves them by `position`
// and `velocity`: a galaxy placed in an encounter.
auto place_bodies(std::size_t first, const Matrix3& rotation,
                  const Position& position, const Velocity& velocity,
                  InOut<std::vector<BodyStart>> bodies) -> void;

//-- Encounters ----------------------------------------------------------------

// Two masses on a parabolic orbit about each other that passes within
// `pericenter`, as Toomre and Toomre's encounters do.
struct ParabolicOrbit final {
  Mass first = 0.0 * kilogram;
  Mass second = 0.0 * kilogram;
  Length pericenter = 0.0 * meter;
};

// The second mass's position and velocity relative to the first's.
struct Separation final {
  Position position = meters(0.0, 0.0, 0.0);
  Velocity velocity = meters_per_second(0.0, 0.0, 0.0);
};

// Computes where the second mass of `orbit` is, relative to the first,
// `time` after pericenter (before it if negative), from Barker's equation
// solved in closed form (Vallado). The orbit lies in the x-y plane, turning
// counterclockwise about z, with pericenter on the x axis.
auto compute_parabolic_separation(const ParabolicOrbit& orbit, Time time)
    -> Separation;

// Rings of test particles about a mass: `counts[i]` particles evenly spaced
// on a ring of radius `radii[i]`, the first at angle 0, each on a circular
// orbit counterclockwise about z under the mass's gravity softened by
// `softening`.
struct RingDisk final {
  std::vector<Length> radii;
  std::vector<int> counts;
  Length softening = 0.0 * meter;
};

// Toomre and Toomre's disk: 120 particles in rings of 12, 18, 24, 30 and 36
// at 0.2, 0.3, 0.4, 0.5 and 0.6 of `pericenter`.
auto make_toomre_disk(Length pericenter, Length softening) -> RingDisk;

// Appends to `bodies` the particles of `disk` about `center`, a body of
// `mass` whose position and velocity they start from.
auto append_ring_disk(const RingDisk& disk, const BodyStart& center,
                      InOut<std::vector<BodyStart>> bodies) -> void;

}  // namespace simon::gravity
