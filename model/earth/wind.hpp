// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstdint>

#include "core/argument.hpp"
#include "core/math.hpp"
#include "core/units.hpp"

// Wind and turbulence: the air's motion relative to the Earth, which aircraft
// fly through. A steady wind is the same everywhere. Turbulence has the Dryden
// spectra of MIL-F-8785C (see model/REFERENCES.md), frozen in the air: an
// aircraft meets it along its path at its airspeed. Each aircraft carries the
// state of its own filters, so the turbulence it meets depends on its own
// flight and its seed.
//
// The filters are sampled exactly. Each step draws the next state from the
// distribution the continuous filter would reach over the step, so the
// turbulence's variance and correlation are the specification's at any step.
// JSBSim's MIL-F-8785C and Tustin turbulence discretize the filters, so theirs
// change with the frame.
namespace simon::model {

// The air's motion at an aircraft relative to the Earth, in the local
// north-east-down frame: its velocity, and the rotation turbulence gives it.
// Aircraft hold it over a step.
struct Wind final {
  Velocity north_east_down = meters_per_second(0.0, 0.0, 0.0);
  AngularVelocity rotation = QuantityVector{} * radian_per_second;
};

// Whether `wind` leaves the air still relative to the Earth.
inline auto is_still(const Wind& wind) -> bool {
  const Vector3& velocity =
      wind.north_east_down.numerical_value_ref_in(meter_per_second).eigen();
  const Vector3& rotation =
      wind.rotation.numerical_value_ref_in(radian_per_second).eigen();
  return velocity.x() == 0.0 && velocity.y() == 0.0 && velocity.z() == 0.0 &&
         rotation.x() == 0.0 && rotation.y() == 0.0 && rotation.z() == 0.0;
}

// How rough turbulence is. Above 2,000 ft its intensity is MIL-F-8785C's that
// is exceeded with a probability of 10^-2, 10^-3 and 10^-5; below 1,000 ft it
// follows from a wind at 20 ft of 15, 30 and 45 knots.
enum class Turbulence : std::uint8_t { NONE, LIGHT, MODERATE, SEVERE };

// The wind a world's air carries: a steady wind, the same everywhere, and
// turbulence.
struct WindField final {
  Velocity north_east_down = meters_per_second(0.0, 0.0, 0.0);
  Turbulence turbulence = Turbulence::NONE;
};

// Whether `field` has neither wind nor turbulence.
auto is_still(const WindField& field) -> bool;

// Turbulence's intensities and scale lengths at an altitude: along the flight
// path (u), across it (v) and down (w).
struct TurbulenceScales final {
  Speed sigma_u = 0.0 * meter_per_second;
  Speed sigma_v = 0.0 * meter_per_second;
  Speed sigma_w = 0.0 * meter_per_second;
  Length length_u = 0.0 * meter;
  Length length_v = 0.0 * meter;
  Length length_w = 0.0 * meter;
};

// The scales of `turbulence` at `altitude` above the ground, from
// MIL-F-8785C: its low-altitude model up to 1,000 ft, its table of
// intensities above 2,000 ft, and in between a straight line from one to the
// other. Altitudes below 10 ft are taken as 10 ft.
auto find_turbulence_scales(Turbulence turbulence, Length altitude)
    -> TurbulenceScales;

// The gusts one aircraft meets: its filters' states, and the gust they make
// now, along the aircraft's path through the air (x along its heading, y to
// the right, z down).
struct Gusts final {
  std::uint64_t seed = 0;
  std::uint64_t draws = 0;  // Steps drawn: none before the first.
  // Each filter's state, scaled to unit variance: u's, v's and w's (two
  // each, for their second-order spectra) and the roll rate's.
  double u = 0.0;
  std::array<double, 2> v{};
  std::array<double, 2> w{};
  double p = 0.0;
  // The v and w gusts through a lag, from which the yaw and pitch rates
  // follow.
  Speed lagged_v = 0.0 * meter_per_second;
  Speed lagged_w = 0.0 * meter_per_second;
  Velocity velocity = meters_per_second(0.0, 0.0, 0.0);
  AngularVelocity rotation = QuantityVector{} * radian_per_second;
};

// Advances `gusts` by `dt` for an aircraft of wing span `span`, flying
// through `turbulence` at `altitude` and `airspeed`. The first draw starts
// the filters from their steady distribution, so turbulence needs no time to
// build up.
auto advance_gusts(Turbulence turbulence, Length altitude, Speed airspeed,
                   Length span, Time dt, InOut<Gusts> gusts) -> void;

// The wind an aircraft heading `heading` through the air meets: `field`'s
// steady wind, and `gusts` turned from the aircraft's path into the local
// north-east-down frame.
auto compute_wind(const WindField& field, const Gusts& gusts, Angle heading)
    -> Wind;

// The steady wind alone.
auto compute_wind(const WindField& field) -> Wind;

}  // namespace simon::model
