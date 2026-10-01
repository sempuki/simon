// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>

#include "Eigen/Dense"
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
// so every operation here evaluates back to a Vector3d.
class Vector3d final {
 public:
  using value_type = double;

  Vector3d() : value_{Eigen::Vector3d::Zero()} {}
  Vector3d(double x, double y, double z) : value_{x, y, z} {}
  template <typename DerivedType>
  explicit Vector3d(const Eigen::MatrixBase<DerivedType>& value)
      : value_{value} {}

  auto eigen() const -> const Eigen::Vector3d& { return value_; }

  auto x() const -> double { return value_.x(); }
  auto y() const -> double { return value_.y(); }
  auto z() const -> double { return value_.z(); }

  auto is_approximately(const Vector3d& that, double precision = 1e-12) const
      -> bool {
    return value_.isApprox(that.value_, precision) ||
           (value_ - that.value_).norm() <= precision;
  }

  friend auto operator-(const Vector3d& a) -> Vector3d {
    return Vector3d{-a.value_};
  }
  friend auto operator+(const Vector3d& a, const Vector3d& b) -> Vector3d {
    return Vector3d{a.value_ + b.value_};
  }
  friend auto operator-(const Vector3d& a, const Vector3d& b) -> Vector3d {
    return Vector3d{a.value_ - b.value_};
  }
  friend auto operator*(const Vector3d& a, double scale) -> Vector3d {
    return Vector3d{a.value_ * scale};
  }
  friend auto operator*(double scale, const Vector3d& a) -> Vector3d {
    return Vector3d{scale * a.value_};
  }
  friend auto operator/(const Vector3d& a, double scale) -> Vector3d {
    return Vector3d{a.value_ / scale};
  }

  auto operator+=(const Vector3d& that) -> Vector3d& {
    value_ += that.value_;
    return *this;
  }
  auto operator-=(const Vector3d& that) -> Vector3d& {
    value_ -= that.value_;
    return *this;
  }
  auto operator*=(double scale) -> Vector3d& {
    value_ *= scale;
    return *this;
  }
  auto operator/=(double scale) -> Vector3d& {
    value_ /= scale;
    return *this;
  }

  friend auto operator==(const Vector3d& a, const Vector3d& b) -> bool {
    return a.value_ == b.value_;
  }

  // Found by mp_units::magnitude through argument-dependent lookup.
  friend auto magnitude(const Vector3d& a) -> double { return a.value_.norm(); }
  friend auto dot(const Vector3d& a, const Vector3d& b) -> double {
    return a.value_.dot(b.value_);
  }
  friend auto cross(const Vector3d& a, const Vector3d& b) -> Vector3d {
    return Vector3d{a.value_.cross(b.value_)};
  }

 private:
  Eigen::Vector3d value_;
};

inline constexpr auto meter = units::si::metre;
inline constexpr auto second = units::si::second;
inline constexpr auto meter_per_second = meter / second;
inline constexpr auto meter_per_second_squared = meter / units::square(second);
inline constexpr auto per_second = units::one / second;
inline constexpr auto radian = units::si::radian;
inline constexpr auto radian_per_second = radian / second;
inline constexpr auto kilogram = units::si::kilogram;
inline constexpr auto newton = units::si::newton;
inline constexpr auto square_meter = units::square(meter);
inline constexpr auto kilogram_per_cubic_meter =
    kilogram / units::cubic(meter);

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

// Vectors.
using Displacement = units::quantity<meter, Vector3d>;
using Velocity = units::quantity<meter_per_second, Vector3d>;
using Acceleration = units::quantity<meter_per_second_squared, Vector3d>;

// Positions are displacements from the world origin, the origin of the local
// Cartesian frame.
using Position = Displacement;

inline auto meters(double x, double y, double z) -> Displacement {
  return Vector3d{x, y, z} * meter;
}
inline auto meters_per_second(double x, double y, double z) -> Velocity {
  return Vector3d{x, y, z} * meter_per_second;
}
inline auto meters_per_second_squared(double x, double y, double z)
    -> Acceleration {
  return Vector3d{x, y, z} * meter_per_second_squared;
}

// An angle in radians, as a plain number for the standard math functions.
inline auto radians(Angle angle) -> double {
  return angle.numerical_value_in(radian);
}

// A std::chrono duration as seconds.
template <typename RepresentationType, typename PeriodType>
auto seconds(std::chrono::duration<RepresentationType, PeriodType> duration)
    -> Time {
  return std::chrono::duration<double>(duration).count() * second;
}

// Vector algebra on quantities. mp-units 2.5 has none for custom
// representations, so these unwrap, operate, and rewrap with the product unit.
template <auto A, auto B>
auto dot(const units::quantity<A, Vector3d>& a,
         const units::quantity<B, Vector3d>& b) {
  return dot(a.numerical_value_ref_in(a.unit),
             b.numerical_value_ref_in(b.unit)) *
         (A * B);
}

template <auto A, auto B>
auto cross(const units::quantity<A, Vector3d>& a,
           const units::quantity<B, Vector3d>& b) {
  return cross(a.numerical_value_ref_in(a.unit),
               b.numerical_value_ref_in(b.unit)) *
         (A * B);
}

template <auto A>
auto norm(const units::quantity<A, Vector3d>& a) {
  return magnitude(a.numerical_value_ref_in(a.unit)) * A;
}

}  // namespace simon::model
