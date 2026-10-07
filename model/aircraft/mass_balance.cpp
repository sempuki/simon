// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows JSBSim 1.3.1, Copyright (C) 2000 Jon S. Berndt and the JSBSim
// authors, LGPL-2.0-or-later; translated to C++ and changed. See NOTICE.md.

#include "model/aircraft/mass_balance.hpp"

#include <cstddef>

#include "base/core.hpp"

namespace simon::aircraft {

auto fill_fuel_tanks(const AircraftData& aircraft) -> FuelTanks {
  FuelTanks tanks;
  for (std::size_t i = 0; i < aircraft.tanks.size(); ++i) {
    tanks.contents[i] = aircraft.tanks[i].contents;
  }
  return tanks;
}

auto compute_body_offset(const Displacement& structural,
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
  Vector3 center = moment / total;

  // Each mass's inertia about the center of mass, in body axes: structural x
  // aft and z up, body x forward and z down. The tensor is symmetric, so its
  // six terms are summed alone: xx, yy, zz, xy, xz, yz.
  const std::array<double, 6>& j = aircraft.empty_inertia;
  std::array<double, 6> terms = j;
  auto add = [&](double mass, const Displacement& location) {
    Vector3 apart = eigen(location) - center;
    double x = -apart.x();
    double y = apart.y();
    double z = -apart.z();
    terms[0] += mass * (y * y + z * z);
    terms[1] += mass * (x * x + z * z);
    terms[2] += mass * (x * x + y * y);
    terms[3] -= mass * x * y;
    terms[4] -= mass * x * z;
    terms[5] -= mass * y * z;
  };
  add(empty, aircraft.empty_center_of_mass);
  for (const PointMass& point : aircraft.point_masses) {
    add(point.mass.numerical_value_in(kilogram), point.location);
  }
  for (std::size_t i = 0; i < contents.size(); ++i) {
    add(contents[i].numerical_value_in(kilogram), aircraft.tanks[i].location);
  }
  Matrix3 inertia;
  inertia << terms[0], terms[3], terms[4],  //
      terms[3], terms[1], terms[5],         //
      terms[4], terms[5], terms[2];
  return MassBalance{
      .properties = model::compute_mass_properties(total * kilogram, inertia),
      .center_of_mass = QuantityVector{center} * meter,
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

}  // namespace simon::aircraft
