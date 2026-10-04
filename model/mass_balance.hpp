// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <span>

#include "model/aircraft_data.hpp"
#include "model/rigid_body.hpp"
#include "model/units.hpp"

// An aircraft's fuel and mass balance: its mass, center of mass and inertia,
// from the empty aircraft, its point masses and the fuel in its tanks.
namespace simon::model {

// The fuel in each tank, in the aircraft data's order.
struct FuelTanks final {
  std::array<Mass, MAX_TANKS> contents{};
};

// The tanks as the aircraft data fills them.
auto fill_fuel_tanks(const AircraftData& aircraft) -> FuelTanks;

// An aircraft's mass properties as loaded: its mass, its inertia about its
// center of mass in body axes, and where that center is, in the structural
// frame (x aft, y right, z up).
struct MassBalance final {
  MassProperties properties;
  Displacement center_of_mass = meters(0.0, 0.0, 0.0);
};

// The mass balance of `aircraft` with `contents` in its tanks, one for each,
// by the inertia tensor's definition, the parallel axis theorem in tensor
// form for the empty aircraft's own inertia (Goldstein; see
// model/REFERENCES.md), as
// JSBSim's FGMassBalance finds it: the empty aircraft, its point masses and
// each tank's fuel, as a point mass, about the combined center of mass.
auto compute_mass_balance(const AircraftData& aircraft,
                          std::span<const Mass> contents) -> MassBalance;

// The same, with each tank as the aircraft data fills it, or as `tanks`
// holds.
auto compute_mass_balance(const AircraftData& aircraft) -> MassBalance;
auto compute_mass_balance(const AircraftData& aircraft, const FuelTanks& tanks)
    -> MassBalance;

// A point in the structural frame, from the center of mass, in body axes.
auto compute_body_offset(const Displacement& structural,
                         const Displacement& center_of_mass) -> Displacement;

}  // namespace simon::model
