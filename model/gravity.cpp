// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/gravity.hpp"

#include <cmath>

namespace simon::model {

auto sum_gravity(std::span<const GravitySource> sources, framework::Entity self,
                 const Vector3& position, double softening) -> Vector3 {
  double softening2 = softening * softening;
  Vector3 acceleration = Vector3::Zero();
  for (const GravitySource& source : sources) {
    if (source.entity == self) continue;
    double dx = position.x() - source.position.x();
    double dy = position.y() - source.position.y();
    double dz = position.z() - source.position.z();
    double r = std::sqrt(dx * dx + dy * dy + dz * dz + softening2);
    double pull = -GRAVITATIONAL_CONSTANT / (r * r * r) * source.mass;
    acceleration.x() += pull * dx;
    acceleration.y() += pull * dy;
    acceleration.z() += pull * dz;
  }
  return acceleration;
}

auto compute_potential_energy(std::span<const GravitySource> sources,
                              double softening) -> double {
  double softening2 = softening * softening;
  double energy = 0.0;
  for (std::size_t i = 0; i < sources.size(); ++i) {
    for (std::size_t j = 0; j < i; ++j) {
      Vector3 d = sources[i].position - sources[j].position;
      energy -= GRAVITATIONAL_CONSTANT * sources[i].mass * sources[j].mass /
                std::sqrt(d.squaredNorm() + softening2);
    }
  }
  return energy;
}

}  // namespace simon::model
