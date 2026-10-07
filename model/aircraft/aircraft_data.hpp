// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "core/units.hpp"
#include "model/aircraft/aerodynamics.hpp"
#include "model/aircraft/flight_control.hpp"

// An aircraft as data: what tools/jsbsim/convert.py writes from a JSBSim
// aircraft, read back by format/aircraft_file. Locations are in JSBSim's
// structural frame (x aft, y right, z up), in meters.
namespace simon::model {

// The most engines and tanks an aircraft may have.
inline constexpr std::size_t MAX_ENGINES = 4;
inline constexpr std::size_t MAX_TANKS = 8;

// A mass fixed at a point, such as the pilot.
struct PointMass final {
  Mass mass = 0.0 * kilogram;
  Displacement location;
};

struct FuelTank final {
  Displacement location;
  Mass capacity = 0.0 * kilogram;
  Mass contents = 0.0 * kilogram;
};

// A turbine as JSBSim models one, with its thrust tables by Mach number and
// density altitude, as fractions of military thrust, and of maximum thrust
// for one with reheat.
struct TurbineData final {
  auto has_reheat() const -> bool { return max_thrust_factor.has_value(); }

  std::string name;
  Displacement location;
  std::vector<std::size_t> feeds;  // The tanks it draws from.
  Force military_thrust = 0.0 * newton;
  double bypass_ratio = 0.0;
  double thrust_specific_fuel_consumption = 0.0;  // kg/s per N.
  double bleed = 0.0;
  // With reheat, which a throttle past 1 lights and which is full at 2.
  Force max_thrust = 0.0 * newton;
  double reheat_thrust_specific_fuel_consumption = 0.0;  // kg/s per N.
  double idle_n1 = 0.0;                                  // Percent.
  double idle_n2 = 0.0;
  double max_n1 = 0.0;
  double max_n2 = 0.0;
  double idle_fuel_flow = 0.0;  // kg/s.
  // Each spool's rates of speeding up and slowing down at full speed at sea
  // level, in percent per second.
  double n1_spool_up = 0.0;
  double n1_spool_down = 0.0;
  double n2_spool_up = 0.0;
  double n2_spool_down = 0.0;
  std::optional<AeroTable> idle_thrust;
  std::optional<AeroTable> military_thrust_factor;
  std::optional<AeroTable> max_thrust_factor;  // Only with reheat.
};

struct AircraftData final {
  std::string name;
  Area wing_area = 0.0 * square_meter;
  Length wing_span = 0.0 * meter;
  Length chord = 0.0 * meter;
  Displacement aero_reference;
  Displacement eye_point;  // Where the pilot's accelerations are felt.

  Mass empty_mass = 0.0 * kilogram;
  // The inertia tensor's elements, as JSBSim builds it: xx, yy, zz, xy, xz,
  // yz, in kg m^2, in body axes.
  std::array<double, 6> empty_inertia{};
  Displacement empty_center_of_mass;
  std::vector<PointMass> point_masses;

  std::vector<FuelTank> tanks;
  std::vector<TurbineData> engines;
  FlightControlData flight_controls;
  AeroModel aero;
};

}  // namespace simon::model
