// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>

#include "Eigen/Dense"
#include "mp-units/framework.h"
#include "mp-units/systems/si.h"

// Physical quantities carry their units in their types. Time uses std::chrono
// (framework::Duration); everything else uses mp-units, in plain SI units.
//
// Plain units check that metres are not added to metres per second. mp-units'
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
  template <typename Derived>
  explicit Vector3d(const Eigen::MatrixBase<Derived>& value) : value_{value} {}

  const Eigen::Vector3d& eigen() const { return value_; }

  double x() const { return value_.x(); }
  double y() const { return value_.y(); }
  double z() const { return value_.z(); }

  bool is_approximately(const Vector3d& that, double precision = 1e-12) const {
    return value_.isApprox(that.value_, precision) ||
           (value_ - that.value_).norm() <= precision;
  }

  friend Vector3d operator-(const Vector3d& a) { return Vector3d{-a.value_}; }
  friend Vector3d operator+(const Vector3d& a, const Vector3d& b) {
    return Vector3d{a.value_ + b.value_};
  }
  friend Vector3d operator-(const Vector3d& a, const Vector3d& b) {
    return Vector3d{a.value_ - b.value_};
  }
  friend Vector3d operator*(const Vector3d& a, double scale) {
    return Vector3d{a.value_ * scale};
  }
  friend Vector3d operator*(double scale, const Vector3d& a) {
    return Vector3d{scale * a.value_};
  }
  friend Vector3d operator/(const Vector3d& a, double scale) {
    return Vector3d{a.value_ / scale};
  }

  Vector3d& operator+=(const Vector3d& that) {
    value_ += that.value_;
    return *this;
  }
  Vector3d& operator-=(const Vector3d& that) {
    value_ -= that.value_;
    return *this;
  }
  Vector3d& operator*=(double scale) {
    value_ *= scale;
    return *this;
  }
  Vector3d& operator/=(double scale) {
    value_ /= scale;
    return *this;
  }

  friend bool operator==(const Vector3d& a, const Vector3d& b) {
    return a.value_ == b.value_;
  }

  // Found by mp_units::magnitude through argument-dependent lookup.
  friend double magnitude(const Vector3d& a) { return a.value_.norm(); }
  friend double dot(const Vector3d& a, const Vector3d& b) {
    return a.value_.dot(b.value_);
  }
  friend Vector3d cross(const Vector3d& a, const Vector3d& b) {
    return Vector3d{a.value_.cross(b.value_)};
  }

 private:
  Eigen::Vector3d value_;
};

inline constexpr auto metre = units::si::metre;
inline constexpr auto second = units::si::second;
inline constexpr auto metre_per_second = metre / second;
inline constexpr auto metre_per_second_squared = metre / units::square(second);
inline constexpr auto per_second = units::one / second;

// Scalars.
using Length = units::quantity<metre, double>;
using Time = units::quantity<second, double>;
using Speed = units::quantity<metre_per_second, double>;
using Rate = units::quantity<per_second, double>;

// Vectors.
using Displacement = units::quantity<metre, Vector3d>;
using Velocity = units::quantity<metre_per_second, Vector3d>;
using Acceleration = units::quantity<metre_per_second_squared, Vector3d>;

// Positions are displacements from the world origin, the origin of the local
// Cartesian frame.
using Position = Displacement;

inline Displacement metres(double x, double y, double z) {
  return Vector3d{x, y, z} * metre;
}
inline Velocity metres_per_second(double x, double y, double z) {
  return Vector3d{x, y, z} * metre_per_second;
}
inline Acceleration metres_per_second_squared(double x, double y, double z) {
  return Vector3d{x, y, z} * metre_per_second_squared;
}

// A std::chrono duration as seconds.
template <typename Representation, typename Period>
Time seconds(std::chrono::duration<Representation, Period> duration) {
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
