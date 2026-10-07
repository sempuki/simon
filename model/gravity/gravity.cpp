// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows REBOUND 5.2.2, Copyright (c) 2011 Hanno Rein, Shangfei Liu,
// GPL-3.0; translated to C++ and changed. See NOTICE.md.

#include "model/gravity/gravity.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace simon::gravity {

auto sum_gravity(std::span<const GravitySource> sources, std::uint32_t self,
                 const Vector3& position, double softening) -> Vector3 {
  double softening2 = softening * softening;
  Vector3 acceleration = Vector3::Zero();
  for (const GravitySource& source : sources) {
    if (source.id == self) continue;
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

namespace {

// The deepest a tree splits: sources closer than a 2^-64th of the root's
// width share a leaf.
constexpr int MOST_DEPTH = 64;

// The octant of `position` about `middle`, one bit an axis.
auto find_octant(const Vector3& position, const Vector3& middle) -> int {
  return (position.x() >= middle.x() ? 1 : 0) |
         (position.y() >= middle.y() ? 2 : 0) |
         (position.z() >= middle.z() ? 4 : 0);
}

}  // namespace

auto GravityTree::build(std::span<const GravitySource> sources) -> void {
  sources_.assign(sources.begin(), sources.end());
  scratch_.resize(sources_.size());
  cells_.clear();
  if (sources_.empty()) return;

  Vector3 low = sources_.front().position;
  Vector3 high = low;
  for (const GravitySource& source : sources_) {
    low = low.cwiseMin(source.position);
    high = high.cwiseMax(source.position);
  }
  double width = (high - low).maxCoeff();
  if (!(width > 0.0)) width = 1.0;

  cells_.push_back(Cell{});
  build_cell(0, 0, static_cast<std::uint32_t>(sources_.size()), low, width, 0);
}

// Sorts the cell's sources by octant, stably, so the tree depends only on
// the sources' order, and builds a child for each octant that has any. A
// cell's mass and center of mass are its children's, or its sources'.
auto GravityTree::build_cell(std::uint32_t cell, std::uint32_t begin,
                             std::uint32_t end, const Vector3& corner,
                             double width, int depth) -> void {
  cells_[cell].width = width;
  if (end - begin == 1 || depth == MOST_DEPTH) {
    double mass = 0.0;
    Vector3 moment = Vector3::Zero();
    for (std::uint32_t i = begin; i < end; ++i) {
      mass += sources_[i].mass;
      moment += sources_[i].mass * sources_[i].position;
    }
    Cell& leaf = cells_[cell];
    leaf.leaf = true;
    leaf.first = begin;
    leaf.count = end - begin;
    leaf.mass = mass;
    leaf.center_of_mass =
        mass > 0.0 ? Vector3{moment / mass} : sources_[begin].position;
    return;
  }

  double half = 0.5 * width;
  Vector3 middle = corner + Vector3::Constant(half);
  std::array<std::uint32_t, 9> offsets{};
  for (std::uint32_t i = begin; i < end; ++i) {
    ++offsets[find_octant(sources_[i].position, middle) + 1];
  }
  std::array<std::uint32_t, 8> counts{};
  for (int octant = 0; octant < 8; ++octant) {
    counts[octant] = offsets[octant + 1];
    offsets[octant + 1] += offsets[octant];
  }
  std::array<std::uint32_t, 9> starts = offsets;
  for (std::uint32_t i = begin; i < end; ++i) {
    int octant = find_octant(sources_[i].position, middle);
    scratch_[begin + offsets[octant]++] = sources_[i];
  }
  std::copy(scratch_.begin() + begin, scratch_.begin() + end,
            sources_.begin() + begin);

  auto first = static_cast<std::uint32_t>(cells_.size());
  std::uint32_t children = 0;
  for (std::uint32_t count : counts) children += count > 0 ? 1 : 0;
  cells_[cell].first = first;
  cells_[cell].count = children;
  cells_.resize(cells_.size() + children);

  std::uint32_t child = first;
  for (int octant = 0; octant < 8; ++octant) {
    if (counts[octant] == 0) continue;
    Vector3 child_corner = corner + half * Vector3{(octant & 1) ? 1.0 : 0.0,
                                                   (octant & 2) ? 1.0 : 0.0,
                                                   (octant & 4) ? 1.0 : 0.0};
    build_cell(child, begin + starts[octant], begin + starts[octant + 1],
               child_corner, half, depth + 1);
    ++child;
  }

  double mass = 0.0;
  Vector3 moment = Vector3::Zero();
  for (std::uint32_t i = first; i < first + children; ++i) {
    mass += cells_[i].mass;
    moment += cells_[i].mass * cells_[i].center_of_mass;
  }
  cells_[cell].mass = mass;
  cells_[cell].center_of_mass = mass > 0.0 ? Vector3{moment / mass} : middle;
}

auto GravityTree::compute_acceleration(std::uint32_t self,
                                       const Vector3& position,
                                       double opening_angle,
                                       double softening) const -> Vector3 {
  Vector3 acceleration = Vector3::Zero();
  if (cells_.empty()) return acceleration;
  double softening2 = softening * softening;
  double opening2 = opening_angle * opening_angle;

  std::array<std::uint32_t, 8 * (MOST_DEPTH + 1)> stack{};
  std::size_t size = 0;
  stack[size++] = 0;
  while (size > 0) {
    const Cell& cell = cells_[stack[--size]];
    if (cell.leaf) {
      acceleration +=
          sum_gravity(std::span{sources_}.subspan(cell.first, cell.count), self,
                      position, softening);
      continue;
    }
    double dx = position.x() - cell.center_of_mass.x();
    double dy = position.y() - cell.center_of_mass.y();
    double dz = position.z() - cell.center_of_mass.z();
    double r2 = dx * dx + dy * dy + dz * dz;
    if (cell.width * cell.width > opening2 * r2) {
      for (std::uint32_t child = cell.first + cell.count; child > cell.first;
           --child) {
        stack[size++] = child - 1;
      }
      continue;
    }
    double r = std::sqrt(r2 + softening2);
    double pull = -GRAVITATIONAL_CONSTANT / (r * r * r) * cell.mass;
    acceleration.x() += pull * dx;
    acceleration.y() += pull * dy;
    acceleration.z() += pull * dz;
  }
  return acceleration;
}

}  // namespace simon::gravity
