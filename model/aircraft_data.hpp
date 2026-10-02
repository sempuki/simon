// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstddef>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "base/status.hpp"
#include "model/aerodynamics.hpp"
#include "model/units.hpp"

// An aircraft as data: what tools/jsbsim/convert.py writes from a JSBSim
// aircraft, read back. Locations are in JSBSim's structural frame (x aft,
// y right, z up), in meters.
namespace simon::model {

struct FuelTank final {
  Displacement location;
  Mass capacity = 0.0 * kilogram;
  Mass contents = 0.0 * kilogram;
};

// A turbine as JSBSim models one, with its thrust tables by Mach number and
// density altitude, as fractions of military thrust.
struct TurbineData final {
  std::string name;
  Displacement location;
  std::vector<std::size_t> feeds;  // The tanks it draws from.
  Force military_thrust = 0.0 * newton;
  double bypass_ratio = 0.0;
  double thrust_specific_fuel_consumption = 0.0;  // kg/s per N.
  double bleed = 0.0;
  double idle_n1 = 0.0;  // Percent.
  double idle_n2 = 0.0;
  double max_n1 = 0.0;
  double max_n2 = 0.0;
  std::optional<AeroTable> idle_thrust;
  std::optional<AeroTable> military_thrust_factor;
};

struct AircraftData final {
  std::string name;
  Area wing_area = 0.0 * square_meter;
  Length wing_span = 0.0 * meter;
  Length chord = 0.0 * meter;
  Displacement aero_reference;

  Mass empty_mass = 0.0 * kilogram;
  // The inertia tensor's elements, as JSBSim builds it: xx, yy, zz, xy, xz,
  // yz, in kg m^2, in body axes.
  std::array<double, 6> empty_inertia{};
  Displacement empty_center_of_mass;

  std::vector<FuelTank> tanks;
  std::vector<TurbineData> engines;
  AeroModel aero;
};

// Why an aircraft could not be read.
enum class AircraftDataError {
  UNREADABLE,  // The file could not be opened.
  MALFORMED,   // The message says where and how.
  COUNT,
};

inline constexpr std::size_t AIRCRAFT_DATA_ERROR_COUNT =
    static_cast<std::size_t>(AircraftDataError::COUNT);

// Reads the aircraft that `text` describes.
auto parse_aircraft(std::string_view text)
    -> std::expected<AircraftData, lib::Status>;

// Reads the aircraft in the file at `path`.
auto load_aircraft(const std::string& path)
    -> std::expected<AircraftData, lib::Status>;

}  // namespace simon::model

// Messages for each AircraftDataError, defined in aircraft_data.cpp.
template <>
const std::array<lib::StatusConditionEntry,
                 simon::model::AIRCRAFT_DATA_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::model::AircraftDataError,
        simon::model::AIRCRAFT_DATA_ERROR_COUNT>::conditions_;
