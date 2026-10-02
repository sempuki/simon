// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>

#include "Eigen/Dense"
#include "framework/vocabulary.hpp"
#include "mp-units/framework.h"
#include "mp-units/systems/si.h"

// Physical quantities carry their units in their types. Time uses std::chrono
// (framework::Duration); everything else uses mp-units, in plain SI units.
//
// Plain units check that meters are not added to meters per second. mp-units'
// ISQ quantity kinds would also tell a position from a displacement or an
// altitude from a range, but Clang 22 cannot compile them (a Clang regression,
// llvm/llvm-project#175831). Every quantity type is an alias here, so moving to
// quantity kinds later changes this file, not the code that uses it.
namespace simon::model {

namespace units = mp_units;

// A 3-vector that mp-units accepts as a representation. Plain Eigen vectors do
// not work: Eigen's operators return expression types, which mp-units rejects,
// so every operation here evaluates back to a QuantityVector.
class QuantityVector final {
 public:
  using value_type = double;

  QuantityVector() : value_{Vector3::Zero()} {}
  QuantityVector(double x, double y, double z) : value_{x, y, z} {}
  template <typename DerivedType>
  explicit QuantityVector(const Eigen::MatrixBase<DerivedType>& value)
      : value_{value} {}

  auto eigen() const -> const Vector3& { return value_; }

  auto x() const -> double { return value_.x(); }
  auto y() const -> double { return value_.y(); }
  auto z() const -> double { return value_.z(); }

  auto is_approximately(const QuantityVector& that,
                        double precision = 1e-12) const -> bool {
    return value_.isApprox(that.value_, precision) ||
           (value_ - that.value_).norm() <= precision;
  }

  friend auto operator-(const QuantityVector& a) -> QuantityVector {
    return QuantityVector{-a.value_};
  }
  friend auto operator+(const QuantityVector& a, const QuantityVector& b)
      -> QuantityVector {
    return QuantityVector{a.value_ + b.value_};
  }
  friend auto operator-(const QuantityVector& a, const QuantityVector& b)
      -> QuantityVector {
    return QuantityVector{a.value_ - b.value_};
  }
  friend auto operator*(const QuantityVector& a, double scale)
      -> QuantityVector {
    return QuantityVector{a.value_ * scale};
  }
  friend auto operator*(double scale, const QuantityVector& a)
      -> QuantityVector {
    return QuantityVector{scale * a.value_};
  }
  friend auto operator/(const QuantityVector& a, double scale)
      -> QuantityVector {
    return QuantityVector{a.value_ / scale};
  }

  auto operator+=(const QuantityVector& that) -> QuantityVector& {
    value_ += that.value_;
    return *this;
  }
  auto operator-=(const QuantityVector& that) -> QuantityVector& {
    value_ -= that.value_;
    return *this;
  }
  auto operator*=(double scale) -> QuantityVector& {
    value_ *= scale;
    return *this;
  }
  auto operator/=(double scale) -> QuantityVector& {
    value_ /= scale;
    return *this;
  }

  friend auto operator==(const QuantityVector& a, const QuantityVector& b)
      -> bool {
    return a.value_ == b.value_;
  }

  // Found by mp_units::magnitude through argument-dependent lookup.
  friend auto magnitude(const QuantityVector& a) -> double {
    return a.value_.norm();
  }
  friend auto dot(const QuantityVector& a, const QuantityVector& b) -> double {
    return a.value_.dot(b.value_);
  }
  friend auto cross(const QuantityVector& a, const QuantityVector& b)
      -> QuantityVector {
    return QuantityVector{a.value_.cross(b.value_)};
  }

 private:
  Vector3 value_;
};

inline constexpr auto meter = units::si::metre;
inline constexpr auto second = units::si::second;
inline constexpr auto meter_per_second = meter / second;
inline constexpr auto meter_per_second_squared = meter / units::square(second);
inline constexpr auto per_second = units::one / second;
inline constexpr auto radian = units::si::radian;
inline constexpr auto radian_per_second = radian / second;
inline constexpr auto radian_per_second_squared =
    radian / units::square(second);
inline constexpr auto kilogram = units::si::kilogram;
inline constexpr auto newton = units::si::newton;
inline constexpr auto newton_meter = newton * meter;
inline constexpr auto square_meter = units::square(meter);
inline constexpr auto kilogram_per_cubic_meter = kilogram / units::cubic(meter);
inline constexpr auto kelvin = units::si::kelvin;
inline constexpr auto pascal = units::si::pascal;
inline constexpr auto joule_per_kilogram_kelvin =
    units::si::joule / (kilogram * kelvin);

// Scalars.
using Length = units::quantity<meter, double>;
using Time = units::quantity<second, double>;
using Speed = units::quantity<meter_per_second, double>;
using AccelerationMagnitude = units::quantity<meter_per_second_squared, double>;
using Rate = units::quantity<per_second, double>;
using Angle = units::quantity<radian, double>;
using AngularRate = units::quantity<radian_per_second, double>;
using Mass = units::quantity<kilogram, double>;
using Force = units::quantity<newton, double>;
using Area = units::quantity<square_meter, double>;
using Density = units::quantity<kilogram_per_cubic_meter, double>;
using Temperature = units::quantity<kelvin, double>;
using Pressure = units::quantity<pascal, double>;

// Vectors.
using Displacement = units::quantity<meter, QuantityVector>;
using Velocity = units::quantity<meter_per_second, QuantityVector>;
using Acceleration = units::quantity<meter_per_second_squared, QuantityVector>;
using ForceVector = units::quantity<newton, QuantityVector>;
using Moment = units::quantity<newton_meter, QuantityVector>;
using AngularVelocity = units::quantity<radian_per_second, QuantityVector>;
using AngularAcceleration =
    units::quantity<radian_per_second_squared, QuantityVector>;

// Positions are displacements from the world origin, the origin of the local
// Cartesian frame.
using Position = Displacement;

inline auto meters(double x, double y, double z) -> Displacement {
  return QuantityVector{x, y, z} * meter;
}
inline auto meters_per_second(double x, double y, double z) -> Velocity {
  return QuantityVector{x, y, z} * meter_per_second;
}
inline auto meters_per_second_squared(double x, double y, double z)
    -> Acceleration {
  return QuantityVector{x, y, z} * meter_per_second_squared;
}

// An angle in radians, as a plain number for the standard math functions.
inline auto radians(Angle angle) -> double {
  return angle.numerical_value_in(radian);
}

// A dimensionless quantity, such as a ratio of two forces, as a plain number:
// the one place a quantity may leave its units behind, because it has none.
template <auto UNIT, typename RepresentationType>
auto number_of(const units::quantity<UNIT, RepresentationType>& quantity)
    -> RepresentationType {
  return quantity.numerical_value_in(units::one);
}

// The larger, the smaller, or the clamp of quantities of one type. Use these
// instead of std::max, std::min and std::clamp on hot paths: those compare
// through mp-units' <=>, which GCC 16 compiles to branches and spills instead
// of maxsd and minsd. std::max on a speed cost the flight model's Fly 8%.
template <auto UNIT>
auto max(const units::quantity<UNIT, double>& a,
         const units::quantity<UNIT, double>& b)
    -> units::quantity<UNIT, double> {
  return std::max(a.numerical_value_in(UNIT), b.numerical_value_in(UNIT)) *
         UNIT;
}
template <auto UNIT>
auto min(const units::quantity<UNIT, double>& a,
         const units::quantity<UNIT, double>& b)
    -> units::quantity<UNIT, double> {
  return std::min(a.numerical_value_in(UNIT), b.numerical_value_in(UNIT)) *
         UNIT;
}
template <auto UNIT>
auto clamp(const units::quantity<UNIT, double>& value,
           const units::quantity<UNIT, double>& low,
           const units::quantity<UNIT, double>& high)
    -> units::quantity<UNIT, double> {
  return min(max(value, low), high);
}

// Trigonometry on angles. mp-units' own functions need ISQ quantity kinds,
// which Clang 22 cannot compile (see the top of this file).
inline auto sin(Angle angle) -> double { return std::sin(radians(angle)); }
inline auto cos(Angle angle) -> double { return std::cos(radians(angle)); }
inline auto arcsin(double ratio) -> Angle { return std::asin(ratio) * radian; }
inline auto arctan(double ratio) -> Angle { return std::atan(ratio) * radian; }

// A std::chrono duration as seconds.
template <typename RepresentationType, typename PeriodType>
auto seconds(std::chrono::duration<RepresentationType, PeriodType> duration)
    -> Time {
  return std::chrono::duration<double>(duration).count() * second;
}

// Vector algebra on quantities. mp-units 2.5 has none for custom
// representations, so these unwrap, operate, and rewrap with the product unit.
template <auto A, auto B>
auto dot(const units::quantity<A, QuantityVector>& a,
         const units::quantity<B, QuantityVector>& b) {
  return dot(a.numerical_value_ref_in(a.unit),
             b.numerical_value_ref_in(b.unit)) *
         (A * B);
}

template <auto A, auto B>
auto cross(const units::quantity<A, QuantityVector>& a,
           const units::quantity<B, QuantityVector>& b) {
  return cross(a.numerical_value_ref_in(a.unit),
               b.numerical_value_ref_in(b.unit)) *
         (A * B);
}

template <auto A>
auto norm(const units::quantity<A, QuantityVector>& a) {
  return magnitude(a.numerical_value_ref_in(a.unit)) * A;
}

}  // namespace simon::model
