// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/mass_balance.hpp"

#include <cstddef>

#include "base/core.hpp"

namespace simon::model {

namespace {

// The point mass inertia of `mass` at `offset` from the center of mass.
auto point_inertia(double mass, const Vector3& offset) -> Matrix3 {
  return mass * (offset.squaredNorm() * Matrix3::Identity() -
                 offset * offset.transpose());
}

}  // namespace

auto fill_fuel_tanks(const AircraftData& aircraft) -> FuelTanks {
  FuelTanks tanks;
  for (std::size_t i = 0; i < aircraft.tanks.size(); ++i) {
    tanks.contents[i] = aircraft.tanks[i].contents;
  }
  return tanks;
}

auto body_offset(const Displacement& structural,
                 const Displacement& center_of_mass) -> Displacement {
  Vector3 apart = eigen(structural) - eigen(center_of_mass);
  // Structural x aft and z up; body x forward and z down.
  return meters(-apart.x(), apart.y(), -apart.z());
}

auto compute_mass_balance(const AircraftData& aircraft,
                          std::span<const Mass> contents) -> MassBalance {
  CHECK_PRECONDITION(contents.size() == aircraft.tanks.size());
  double empty = aircraft.empty_mass.numerical_value_in(kilogram);
  double total = empty;
  Vector3 moment = empty * eigen(aircraft.empty_center_of_mass);
  for (const PointMass& point : aircraft.point_masses) {
    double mass = point.mass.numerical_value_in(kilogram);
    total += mass;
    moment += mass * eigen(point.location);
  }
  for (std::size_t i = 0; i < contents.size(); ++i) {
    double fuel = contents[i].numerical_value_in(kilogram);
    total += fuel;
    moment += fuel * eigen(aircraft.tanks[i].location);
  }
  Displacement center = QuantityVector{moment / total} * meter;

  const std::array<double, 6>& j = aircraft.empty_inertia;
  Matrix3 inertia;
  inertia << j[0], j[3], j[4],  //
      j[3], j[1], j[5],         //
      j[4], j[5], j[2];
  inertia += point_inertia(
      empty, eigen(body_offset(aircraft.empty_center_of_mass, center)));
  for (const PointMass& point : aircraft.point_masses) {
    inertia += point_inertia(point.mass.numerical_value_in(kilogram),
                             eigen(body_offset(point.location, center)));
  }
  for (std::size_t i = 0; i < contents.size(); ++i) {
    inertia +=
        point_inertia(contents[i].numerical_value_in(kilogram),
                      eigen(body_offset(aircraft.tanks[i].location, center)));
  }
  return MassBalance{
      .properties = compute_mass_properties(total * kilogram, inertia),
      .center_of_mass = center,
  };
}

auto compute_mass_balance(const AircraftData& aircraft) -> MassBalance {
  return compute_mass_balance(aircraft, fill_fuel_tanks(aircraft));
}

auto compute_mass_balance(const AircraftData& aircraft, const FuelTanks& tanks)
    -> MassBalance {
  return compute_mass_balance(
      aircraft, std::span{tanks.contents.data(), aircraft.tanks.size()});
}

}  // namespace simon::model
