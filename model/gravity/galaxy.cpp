// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/gravity/galaxy.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

#include "model/gravity/gravity.hpp"

namespace simon::gravity {

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
  return -3.0 * std::numbers::pi * CONSTANT * m * m / (64.0 * a) *
         units::si::joule;
}

auto compute_crossing_time(Mass mass,
                           units::quantity<units::si::joule, double> energy)
    -> Time {
  double m = mass.numerical_value_in(kilogram);
  double e = energy.numerical_value_in(units::si::joule);
  return CONSTANT * std::pow(m, 2.5) / std::pow(-2.0 * e, 1.5) * second;
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
  double speed_unit = std::sqrt(CONSTANT * m / a);
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

namespace {

// A disk galaxy's numbers in SI units, and the halo's profile and the disk's
// spherically averaged mass that its dispersions and escape speeds come from.
struct GalaxyNumbers final {
  explicit GalaxyNumbers(const DiskGalaxy& galaxy)
      : disk_mass{galaxy.disk_mass.numerical_value_in(kilogram)},
        h{galaxy.disk_scale.numerical_value_in(meter)},
        z0{galaxy.disk_thickness.numerical_value_in(meter)},
        halo_mass{galaxy.halo_mass.numerical_value_in(kilogram)},
        a{galaxy.halo_scale.numerical_value_in(meter)},
        cutoff{galaxy.halo_cutoff.numerical_value_in(meter)} {}

  // Hernquist's halo inside `r`, cut off.
  auto halo_inside(double r) const -> double {
    double x = std::min(r, cutoff);
    return halo_mass * x * x / ((x + a) * (x + a));
  }
  auto halo_density(double r) const -> double {
    if (r > cutoff) return 0.0;
    return halo_mass * a / (2.0 * std::numbers::pi * r * std::pow(r + a, 3.0));
  }
  // The disk inside the cylinder of radius `r`, taken as inside the sphere.
  auto disk_inside(double r) const -> double {
    return disk_mass * (1.0 - (1.0 + r / h) * std::exp(-r / h));
  }
  auto surface_density(double r) const -> double {
    return disk_mass / (2.0 * std::numbers::pi * h * h) * std::exp(-r / h);
  }

  double disk_mass;
  double h;
  double z0;
  double halo_mass;
  double a;
  double cutoff;
};

// The halo's radial dispersion squared and its escape speed squared, on a
// grid of radii spaced evenly in log r, integrated inward from the cutoff:
// sigma_r^2 = 1 / rho int_r rho G M / r'^2 dr' (Hernquist 1993, 2.14) and
// v_esc^2 = 2 G (M_total / cutoff + int_r M / r'^2 dr').
struct HaloTable final {
  static constexpr int POINTS = 400;

  explicit HaloTable(const GalaxyNumbers& numbers) {
    low = std::log(1e-4 * numbers.a);
    step = (std::log(numbers.cutoff) - low) / (POINTS - 1);
    double total = numbers.halo_inside(numbers.cutoff) +
                   numbers.disk_inside(numbers.cutoff);
    double pressure = 0.0;  // int rho G M / r^2.
    double depth = CONSTANT * total / numbers.cutoff;
    for (int i = POINTS - 1; i >= 0; --i) {
      double r = radius_at(i);
      if (i < POINTS - 1) {
        double outer = radius_at(i + 1);
        auto pull = [&](double x) {
          return CONSTANT * (numbers.halo_inside(x) + numbers.disk_inside(x)) /
                 (x * x);
        };
        pressure += 0.5 *
                    (numbers.halo_density(r) * pull(r) +
                     numbers.halo_density(outer) * pull(outer)) *
                    (outer - r);
        depth += 0.5 * (pull(r) + pull(outer)) * (outer - r);
      }
      double density = numbers.halo_density(r);
      dispersion2[i] = density > 0.0 ? pressure / density : 0.0;
      escape2[i] = 2.0 * depth;
    }
  }

  auto radius_at(int i) const -> double { return std::exp(low + step * i); }

  // Interpolates `values` at radius `r`, linearly in log r.
  auto interpolate(const std::array<double, POINTS>& values, double r) const
      -> double {
    double at = std::clamp((std::log(r) - low) / step, 0.0, POINTS - 1.0);
    int i = std::min(static_cast<int>(at), POINTS - 2);
    double t = at - i;
    return values[i] * (1.0 - t) + values[i + 1] * t;
  }

  double low = 0.0;
  double step = 0.0;
  std::array<double, POINTS> dispersion2{};
  std::array<double, POINTS> escape2{};
};

// Inverts the exponential disk's cumulative mass, 1 - (1 + x) e^-x = X, for
// x = R / h by Newton's method.
auto invert_disk_mass(double fraction) -> double {
  double x = 1.0;
  for (int i = 0; i < 100; ++i) {
    double f = 1.0 - (1.0 + x) * std::exp(-x) - fraction;
    double slope = x * std::exp(-x);
    double next = std::max(x - f / std::max(slope, 1e-300), 0.5 * x);
    if (std::abs(next - x) < 1e-14 * x) return next;
    x = next;
  }
  return x;
}

// Moves `bodies` from `begin` to `end` so that their center of mass is at the
// origin and at rest. The disk and the halo are centered each on its own, so
// that the halo's sampling noise, which is kiloparsecs in its center of mass,
// does not move the disk off its center.
auto center_bodies(std::size_t begin, std::size_t end,
                   InOut<std::vector<BodyStart>> bodies) -> void {
  Vector3 center = Vector3::Zero();
  Vector3 drift = Vector3::Zero();
  double mass = 0.0;
  for (std::size_t i = begin; i < end; ++i) {
    double m = (*bodies)[i].mass.numerical_value_in(kilogram);
    center += m * (*bodies)[i].position.numerical_value_in(meter).eigen();
    drift +=
        m * (*bodies)[i].velocity.numerical_value_in(meter_per_second).eigen();
    mass += m;
  }
  for (std::size_t i = begin; i < end; ++i) {
    (*bodies)[i].position -= QuantityVector{Vector3{center / mass}} * meter;
    (*bodies)[i].velocity -=
        QuantityVector{Vector3{drift / mass}} * meter_per_second;
  }
}

}  // namespace

auto compute_circular_speed(const DiskGalaxy& galaxy, Length radius) -> Speed {
  GalaxyNumbers numbers{galaxy};
  double r = radius.numerical_value_in(meter);
  if (!(r > 0.0)) return 0.0 * meter_per_second;
  double halo = CONSTANT * numbers.halo_inside(r) / r;
  double y = r / (2.0 * numbers.h);
  double disk = 4.0 * std::numbers::pi * CONSTANT *
                numbers.surface_density(0.0) * numbers.h * y * y *
                (std::cyl_bessel_i(0.0, y) * std::cyl_bessel_k(0.0, y) -
                 std::cyl_bessel_i(1.0, y) * std::cyl_bessel_k(1.0, y));
  return std::sqrt(halo + disk) * meter_per_second;
}

// Disk (Hernquist 1993, section 2.2.3): sigma_z^2 = pi G Sigma z_0 (2.22);
// sigma_R^2 proportional to exp(-R / h) (2.21), with Q sigma_crit = 3.36 G
// Sigma / kappa at the stability radius (2.23, Toomre); sigma_phi^2 =
// sigma_R^2 kappa^2 / 4 Omega^2 (2.26); and the mean rotation from v_phi^2 =
// v_c^2 + sigma_R^2 (1 - kappa^2 / 4 Omega^2 - 2 R / h) (2.28), none where
// that is negative. kappa^2 = (1 / R) dv_c^2 / dR + 2 v_c^2 / R^2 (2.24),
// by central differences. Halo (2.2.1): speeds of a Maxwellian of the
// radial dispersion, below 0.95 of the escape speed, in random directions.
auto append_disk_galaxy(const DiskGalaxy& galaxy, InOut<Random> random,
                        InOut<std::vector<BodyStart>> bodies) -> void {
  GalaxyNumbers numbers{galaxy};
  HaloTable table{numbers};
  std::size_t first = bodies->size();

  auto speed2 = [&](double r) {
    return std::pow(compute_circular_speed(galaxy, r * meter)
                        .numerical_value_in(meter_per_second),
                    2.0);
  };
  auto epicyclic2 = [&](double r) {
    double dr = 1e-4 * r;
    return (speed2(r + dr) - speed2(r - dr)) / (2.0 * dr) / r +
           2.0 * speed2(r) / (r * r);
  };
  double reference = galaxy.stability_radius.numerical_value_in(meter);
  double radial_reference = galaxy.stability * 3.36 * CONSTANT *
                            numbers.surface_density(reference) /
                            std::sqrt(epicyclic2(reference));

  double disk_each =
      numbers.disk_mass / static_cast<double>(galaxy.disk_bodies);
  for (std::size_t i = 0; i < galaxy.disk_bodies; ++i) {
    double r = numbers.h * invert_disk_mass(random->unit());
    double angle = 2.0 * std::numbers::pi * random->unit();
    double z = numbers.z0 * std::atanh(2.0 * random->unit() - 1.0);

    double v2 = speed2(r);
    double omega2 = v2 / (r * r);
    double kappa2 = epicyclic2(r);
    double radial =
        radial_reference * std::exp(-0.5 * (r - reference) / numbers.h);
    double vertical = std::sqrt(std::numbers::pi * CONSTANT *
                                numbers.surface_density(r) * numbers.z0);
    double azimuthal = radial * std::sqrt(kappa2 / (4.0 * omega2));
    double mean2 =
        v2 +
        radial * radial * (1.0 - kappa2 / (4.0 * omega2) - 2.0 * r / numbers.h);
    double mean = std::sqrt(std::max(mean2, 0.0));

    double v_r = random->normal(0.0, radial);
    double v_phi = mean + random->normal(0.0, azimuthal);
    double v_z = random->normal(0.0, vertical);
    double c = std::cos(angle);
    double s = std::sin(angle);
    bodies->push_back(BodyStart{
        .position = meters(r * c, r * s, z),
        .velocity =
            meters_per_second(v_r * c - v_phi * s, v_r * s + v_phi * c, v_z),
        .mass = disk_each * kilogram});
  }

  double cut = numbers.cutoff / (numbers.cutoff + numbers.a);
  double halo_each = numbers.halo_inside(numbers.cutoff) /
                     static_cast<double>(galaxy.halo_bodies);
  for (std::size_t i = 0; i < galaxy.halo_bodies; ++i) {
    double root = cut * std::sqrt(random->unit());
    double r = numbers.a * root / (1.0 - root);
    double sigma = std::sqrt(table.interpolate(table.dispersion2, r));
    double limit2 = 0.95 * 0.95 * table.interpolate(table.escape2, r);
    Vector3 velocity;
    do {
      velocity = Vector3{random->normal(0.0, sigma), random->normal(0.0, sigma),
                         random->normal(0.0, sigma)};
    } while (velocity.squaredNorm() > limit2);
    bodies->push_back(BodyStart{
        .position = QuantityVector{sample_direction(r, random)} * meter,
        .velocity = QuantityVector{velocity} * meter_per_second,
        .mass = halo_each * kilogram});
  }

  center_bodies(first, first + galaxy.disk_bodies, bodies);
  center_bodies(first + galaxy.disk_bodies, bodies->size(), bodies);
}

auto place_bodies(std::size_t first, const Matrix3& rotation,
                  const Position& position, const Velocity& velocity,
                  InOut<std::vector<BodyStart>> bodies) -> void {
  for (std::size_t i = first; i < bodies->size(); ++i) {
    BodyStart& body = (*bodies)[i];
    body.position =
        QuantityVector{Vector3{
            rotation * body.position.numerical_value_in(meter).eigen()}} *
            meter +
        position;
    body.velocity =
        QuantityVector{Vector3{
            rotation *
            body.velocity.numerical_value_in(meter_per_second).eigen()}} *
            meter_per_second +
        velocity;
  }
}

// Barker's equation, D + D^3 / 3 = sqrt(mu / 2 q^3) t with D = tan(f / 2),
// has the one real root D = B - 1 / B, B = (A + sqrt(A^2 + 1))^(1/3), A =
// 3/2 sqrt(mu / 2 q^3) t. Then the separation is q (1 - D^2, 2 D), and D
// changes at sqrt(mu / 2 q^3) / (1 + D^2) (derived here from Barker's).
auto compute_parabolic_separation(const ParabolicOrbit& orbit, Time time)
    -> Separation {
  double mu =
      CONSTANT * (orbit.first + orbit.second).numerical_value_in(kilogram);
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
  double gm = CONSTANT * center.mass.numerical_value_in(kilogram);
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

}  // namespace simon::gravity
