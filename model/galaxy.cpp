// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/galaxy.hpp"

#include <cmath>
#include <numbers>

#include "model/gravity.hpp"

namespace simon::model {

namespace {

// A vector of length `length` in a direction uniform on the sphere, from two
// uniform numbers (Aarseth, Henon and Wielen, A3 and A6).
auto sample_direction(double length, InOut<Random> random) -> Vector3 {
  double z = (1.0 - 2.0 * random->unit()) * length;
  double across = std::sqrt(length * length - z * z);
  double angle = 2.0 * std::numbers::pi * random->unit();
  return Vector3{across * std::cos(angle), across * std::sin(angle), z};
}

}  // namespace

auto compute_plummer_radius(const Plummer& plummer, double fraction) -> Length {
  return plummer.scale / std::sqrt(std::pow(fraction, -2.0 / 3.0) - 1.0);
}

auto compute_plummer_energy(const Plummer& plummer)
    -> units::quantity<units::si::joule, double> {
  double m = plummer.mass.numerical_value_in(kilogram);
  double a = plummer.scale.numerical_value_in(meter);
  return -3.0 * std::numbers::pi * GRAVITATIONAL_CONSTANT * m * m / (64.0 * a) *
         units::si::joule;
}

auto compute_crossing_time(Mass mass,
                           units::quantity<units::si::joule, double> energy)
    -> Time {
  double m = mass.numerical_value_in(kilogram);
  double e = energy.numerical_value_in(units::si::joule);
  return GRAVITATIONAL_CONSTANT * std::pow(m, 2.5) / std::pow(-2.0 * e, 1.5) *
         second;
}

// In units where G = M = a = 1, a body's radius solves M(r) = r^3 (1 +
// r^2)^(-3/2) = X for a uniform X (A1, A2). Its speed is q times the escape
// speed there, sqrt(2) (1 + r^2)^(-1/4) (A4), with q drawn by von Neumann's
// rejection from g(q) = q^2 (1 - q^2)^(7/2), which never reaches 0.1 (A5).
// Lengths then scale by a and speeds by sqrt(G M / a).
auto append_plummer(const Plummer& plummer, std::size_t count,
                    InOut<Random> random, InOut<std::vector<BodyStart>> bodies)
    -> void {
  double m = plummer.mass.numerical_value_in(kilogram);
  double a = plummer.scale.numerical_value_in(meter);
  double speed_unit = std::sqrt(GRAVITATIONAL_CONSTANT * m / a);
  double each = m / static_cast<double>(count);

  std::vector<Vector3> positions;
  std::vector<Vector3> velocities;
  positions.reserve(count);
  velocities.reserve(count);
  Vector3 center = Vector3::Zero();
  Vector3 drift = Vector3::Zero();
  for (std::size_t i = 0; i < count; ++i) {
    double x = random->unit();  // In [0, 1), so the radius is finite.
    double r = 1.0 / std::sqrt(std::pow(x, -2.0 / 3.0) - 1.0);
    positions.push_back(sample_direction(r * a, random));

    double q = 0.0;
    for (;;) {
      double candidate = random->unit();
      double height = 0.1 * random->unit();
      if (height <
          candidate * candidate * std::pow(1.0 - candidate * candidate, 3.5)) {
        q = candidate;
        break;
      }
    }
    double speed = q * std::sqrt(2.0) * std::pow(1.0 + r * r, -0.25);

    velocities.push_back(sample_direction(speed * speed_unit, random));
    center += positions.back();
    drift += velocities.back();
  }
  center /= static_cast<double>(count);
  drift /= static_cast<double>(count);

  for (std::size_t i = 0; i < count; ++i) {
    bodies->push_back(BodyStart{
        .position = QuantityVector{positions[i] - center} * meter,
        .velocity = QuantityVector{velocities[i] - drift} * meter_per_second,
        .mass = each * kilogram});
  }
}

// Barker's equation, D + D^3 / 3 = sqrt(mu / 2 q^3) t with D = tan(f / 2),
// has the one real root D = B - 1 / B, B = (A + sqrt(A^2 + 1))^(1/3), A =
// 3/2 sqrt(mu / 2 q^3) t. Then the separation is q (1 - D^2, 2 D), and D
// changes at sqrt(mu / 2 q^3) / (1 + D^2) (derived here from Barker's).
auto compute_parabolic_separation(const ParabolicOrbit& orbit, Time time)
    -> Separation {
  double mu = GRAVITATIONAL_CONSTANT *
              (orbit.first + orbit.second).numerical_value_in(kilogram);
  double q = orbit.pericenter.numerical_value_in(meter);
  double rate = std::sqrt(mu / (2.0 * q * q * q));
  double a = 1.5 * rate * time.numerical_value_in(second);
  double b = std::cbrt(a + std::sqrt(a * a + 1.0));
  double d = b - 1.0 / b;
  double d_rate = rate / (1.0 + d * d);
  return Separation{.position = meters(q * (1.0 - d * d), 2.0 * q * d, 0.0),
                    .velocity = meters_per_second(-2.0 * q * d * d_rate,
                                                  2.0 * q * d_rate, 0.0)};
}

auto make_toomre_disk(Length pericenter, Length softening) -> RingDisk {
  RingDisk disk{.softening = softening};
  for (int ring = 0; ring < 5; ++ring) {
    disk.radii.push_back((0.2 + 0.1 * ring) * pericenter);
    disk.counts.push_back(12 + 6 * ring);
  }
  return disk;
}

// A circular orbit of radius r about a mass M softened by e has speed
// sqrt(G M r^2 / (r^2 + e^2)^(3/2)), where the softened pull balances the
// centripetal acceleration (derived here).
auto append_ring_disk(const RingDisk& disk, const BodyStart& center,
                      InOut<std::vector<BodyStart>> bodies) -> void {
  double gm = GRAVITATIONAL_CONSTANT * center.mass.numerical_value_in(kilogram);
  double e = disk.softening.numerical_value_in(meter);
  for (std::size_t ring = 0; ring < disk.radii.size(); ++ring) {
    double r = disk.radii[ring].numerical_value_in(meter);
    double speed = std::sqrt(gm * r * r / std::pow(r * r + e * e, 1.5));
    for (int i = 0; i < disk.counts[ring]; ++i) {
      double angle = 2.0 * std::numbers::pi * i / disk.counts[ring];
      double c = std::cos(angle);
      double s = std::sin(angle);
      bodies->push_back(BodyStart{
          .position = center.position + meters(r * c, r * s, 0.0),
          .velocity =
              center.velocity + meters_per_second(-speed * s, speed * c, 0.0),
          .mass = 0.0 * kilogram});
    }
  }
}

}  // namespace simon::model
