// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

// Articulated rigid bodies: kinematic trees of bodies on joints, the shapes
// they collide with and the actuators that drive them, as MuJoCo compiles a
// model (see model/REFERENCES.md). Joints are generalized coordinates: a
// hinge or slide adds one position and one degree of freedom, a ball four and
// three, a free joint seven and six. Body 0 is the world. Every pose is
// relative to the parent body's frame, quaternions w, x, y, z. Plain SI
// numbers, as MuJoCo's are, read once and shared, never changed.
namespace simon::model {

using Array3 = std::array<double, 3>;
using Quaternion4 = std::array<double, 4>;  // w, x, y, z.

// Joint and shape types, numbered as MuJoCo numbers them.
enum class JointType : std::uint8_t { FREE, BALL, SLIDE, HINGE };
enum class GeomType : std::uint8_t {
  PLANE,
  HEIGHT_FIELD,
  SPHERE,
  CAPSULE,
  ELLIPSOID,
  CYLINDER,
  BOX,
  MESH,
};

// How a contact or limit softens: MuJoCo's solref (time constant and damping
// ratio) and solimp (impedance from width, its ends, midpoint and power).
struct SoftConstraint final {
  std::array<double, 2> reference{0.02, 1.0};
  std::array<double, 5> impedance{0.9, 0.95, 0.001, 0.5, 2.0};
};

// A rigid body: its frame on its parent's, its center of mass and principal
// axes in its own frame, its mass and principal moments of inertia, and its
// joints, degrees of freedom and geoms, as ranges.
struct ArticulatedBody final {
  std::string name;
  std::uint32_t parent = 0;
  std::uint32_t root = 0;  // The child of the world whose tree it is in.
  Array3 pos{};
  Quaternion4 quat{1.0, 0.0, 0.0, 0.0};
  Array3 inertial_pos{};
  Quaternion4 inertial_quat{1.0, 0.0, 0.0, 0.0};
  double mass = 0.0;  // kg.
  Array3 inertia{};  // kg m^2, about the principal axes.
  std::uint32_t first_joint = 0;
  std::uint32_t joints = 0;
  std::uint32_t first_dof = 0;
  std::uint32_t dofs = 0;
  std::uint32_t first_geom = 0;
  std::uint32_t geoms = 0;
};

// A joint: where on its body and about or along which axis, its limits, and
// its spring.
struct Joint final {
  std::string name;
  JointType type = JointType::HINGE;
  std::uint32_t body = 0;
  Array3 pos{};
  Array3 axis{0.0, 0.0, 1.0};
  std::array<double, 2> range{};  // rad or m.
  double stiffness = 0.0;
  double margin = 0.0;
  SoftConstraint limit;
  std::uint32_t qpos = 0;  // Its first position's address.
  std::uint32_t dof = 0;   // Its first degree of freedom's.
  bool limited = false;
};

// A degree of freedom: its body and joint, the degree of freedom it moves
// on, and what resists it.
struct Dof final {
  static constexpr std::uint32_t NONE = ~std::uint32_t{0};

  std::uint32_t body = 0;
  std::uint32_t joint = 0;
  std::uint32_t parent = NONE;  // The one it moves on, toward the world.
  double armature = 0.0;
  double damping = 0.0;
  double friction_loss = 0.0;
};

// A shape a body collides with, in its body's frame, and how its contacts
// behave: friction sliding, torsional and rolling, the dimensions of its
// contact, which shapes it may touch, and how softly.
struct Geom final {
  std::string name;
  GeomType type = GeomType::SPHERE;
  std::uint32_t body = 0;
  Array3 size{};
  Array3 pos{};
  Quaternion4 quat{1.0, 0.0, 0.0, 0.0};
  Array3 friction{1.0, 0.005, 0.0001};
  SoftConstraint contact;
  double margin = 0.0;
  double gap = 0.0;
  std::uint32_t condim = 3;
  std::uint32_t contype = 1;
  std::uint32_t conaffinity = 1;
  std::int32_t priority = 0;
};

// An actuator on a joint: its force a fixed gain times its control plus,
// for an affine bias, a constant and terms in the joint's length and
// velocity, both times the gear, clamped to its range, then times its gear
// on the joint; and a damping its joint feels, taken implicitly. A motor
// has gain 1 and no bias, a position servo gain kp and bias 0, -kp, -kv, a
// velocity servo gain kv and bias 0, 0, -kv.
struct Actuator final {
  enum class Bias : std::uint8_t { NONE, AFFINE };

  std::string name;
  std::uint32_t joint = 0;
  std::array<double, 6> gear{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 3> gain{1.0, 0.0, 0.0};
  std::array<double, 3> bias{};
  std::array<double, 2> control_range{};
  std::array<double, 2> force_range{};
  double damping = 0.0;
  Bias bias_type = Bias::NONE;
  bool control_limited = false;
  bool force_limited = false;
};

// How the model steps, as MuJoCo's option element sets it.
struct Physics final {
  enum class Integrator : std::uint8_t { EULER, RK4, IMPLICIT, IMPLICIT_FAST };
  enum class Cone : std::uint8_t { PYRAMIDAL, ELLIPTIC };
  enum class Solver : std::uint8_t { PGS, CG, NEWTON };

  double timestep = 0.002;  // s.
  Array3 gravity{0.0, 0.0, -9.81};
  Integrator integrator = Integrator::EULER;
  Cone cone = Cone::PYRAMIDAL;
  Solver solver = Solver::NEWTON;
  std::uint32_t iterations = 100;
  double tolerance = 1e-8;
};

// One kinematic tree of a model: a child of the world and every body under
// it, which lie together in depth-first order, with their joints, degrees of
// freedom, positions, geoms and actuators, as ranges.
struct Tree final {
  std::uint32_t first_body = 0;
  std::uint32_t bodies = 0;
  std::uint32_t first_joint = 0;
  std::uint32_t joints = 0;
  std::uint32_t first_dof = 0;
  std::uint32_t dofs = 0;
  std::uint32_t first_qpos = 0;
  std::uint32_t qpos = 0;
  std::uint32_t first_geom = 0;
  std::uint32_t geoms = 0;
  std::vector<std::uint32_t> actuators;  // On its joints.
};

struct ArticulatedModel final {
  std::string name;
  Physics physics;
  std::vector<ArticulatedBody> bodies;  // The world first.
  std::vector<Joint> joints;
  std::vector<Dof> dofs;
  std::vector<Geom> geoms;
  std::vector<Actuator> actuators;
  std::vector<double> qpos0;        // Each position at rest.
  std::vector<double> qpos_spring;  // Where each spring is unstretched.
};

// Each kinematic tree of `model`, in order.
auto find_trees(const ArticulatedModel& model) -> std::vector<Tree>;

}  // namespace simon::model
