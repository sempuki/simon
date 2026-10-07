// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows MuJoCo 3.14.0, Copyright 2021 DeepMind Technologies Limited,
// Apache-2.0; translated to C++ and changed. See NOTICE.md.

#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "core/argument.hpp"
#include "core/math.hpp"
#include "model/articulated/articulated.hpp"
#include "model/articulated/dynamics.hpp"

// Contacts between the geoms of articulated bodies, as MuJoCo finds them
// (see model/REFERENCES.md): which bodies may touch, which geoms by their
// contact type and affinity, a bounding sphere test, then the primitive
// colliders of engine_collision_primitive.c and engine_collision_box.c, or
// for the pairs those leave out its general convex collider
// (model/articulated/convex), and each contact's parameters mixed from its
// two geoms.
namespace simon::articulated {

// A geom's pose in the world.
struct GeomFrame final {
  Vector3 pos = Vector3::Zero();
  Matrix3 mat = Matrix3::Identity();
};

// What a collider returns: penetration (negative) or separation, the point
// midway between the surfaces, the normal from the first geom toward the
// second, and a tangent to align the frame with, or zero.
struct PreContact final {
  double dist = 0.0;
  Vector3 pos = Vector3::Zero();
  Vector3 normal = Vector3::Zero();
  Vector3 tangent = Vector3::Zero();
};

// A contact, as MuJoCo's mjContact holds it: the frame's rows are the normal
// and two tangents; friction is sliding twice, torsional, rolling twice.
struct Contact final {
  double dist = 0.0;
  Vector3 pos = Vector3::Zero();
  Matrix3 frame = Matrix3::Identity();
  std::array<std::uint32_t, 2> geom{};
  std::uint32_t dim = 3;
  double include_margin = 0.0;
  std::array<double, 5> friction{};
  SoftConstraint soft;
  bool exclude = false;  // In the gap, past the margin.
};

// The most contacts one pair of primitives makes: a box face clipped by
// another's.
inline constexpr std::size_t MAX_PAIR_CONTACTS = 12;

// Whether a primitive collider handles geoms of these types, the first no
// later than the second in GeomType.
auto has_collider(GeomType first, GeomType second) -> bool;

// A geom's bounding sphere's radius, zero for a plane (mjCGeom::GetRBound).
auto compute_bounding_radius(const Geom& geom) -> double;

// A body's frame and inertial frame in the world.
struct BodyFrame final {
  Vector3 xpos = Vector3::Zero();
  Quaternion xquat = Quaternion::Identity();
  Matrix3 xmat = Matrix3::Identity();
  Vector3 xipos = Vector3::Zero();
  Matrix3 ximat = Matrix3::Identity();
};

// A geom's pose from its body's, or the frame it is snapped to
// (mj_local2Global).
auto compute_geom_frame(const Geom& geom, const BodyFrame& body) -> GeomFrame;

// The contacts of two geoms within `margin`, the first's type no later than
// the second's; writes at most MAX_PAIR_CONTACTS and returns how many.
auto collide(const Geom& first, const GeomFrame& first_frame,
             const Geom& second, const GeomFrame& second_frame, double margin,
             std::span<PreContact, MAX_PAIR_CONTACTS> out) -> std::uint32_t;

// Which pairs of bodies MuJoCo lets touch: never two on one rigid assembly
// (a body without joints is welded to its parent), never two that cannot
// move, never an assembly and its parent's (mj_broadphase, filterBodyPair).
class BodyFilter final {
 public:
  explicit BodyFilter(const ArticulatedModel& model);

  auto discards(std::uint32_t first, std::uint32_t second) const -> bool;

 private:
  std::vector<std::uint32_t> weld_;
  std::vector<std::uint32_t> weld_parent_;
  std::vector<std::uint32_t> weld_dofs_;
};

// Appends the contacts of geoms `first` and `second`, on bodies that may
// touch, `first` on the lower numbered body, to `out` (mj_collideGeoms with
// filterCollisionPair, mj_narrowphase, mj_contactParam and mj_setContact):
// their types and affinities must match and their bounding spheres overlap
// within the margin and gap, then the collider runs with the lower type first.
auto append_contacts(const ArticulatedModel& model, std::uint32_t first,
                     std::uint32_t second, std::span<const GeomFrame> frames,
                     std::span<const double> radii,
                     InOut<std::vector<Contact>> out) -> void;

}  // namespace simon::articulated
