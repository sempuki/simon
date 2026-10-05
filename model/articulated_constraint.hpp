// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "framework/vocabulary.hpp"
#include "model/articulated.hpp"

// The constraints of one island of trees and the accelerations they leave,
// as MuJoCo solves them (see model/REFERENCES.md): each constraint a soft
// row, its stiffness, damping and impedance from solref and solimp, its
// force the minimizer of a convex cost in the accelerations (Todorov 2014);
// solved in the accelerations by Newton's method with an exact line search,
// or in the forces by projected Gauss–Seidel. Dense: an island's rows and
// degrees of freedom are few.
namespace simon::model {

// A row's kind, in the order an island lists them: a degree of freedom's
// dry friction, a joint's limit, a contact without friction, an edge of a
// contact's pyramidal friction cone, and a direction of a contact's
// elliptic friction cone: its normal, then its tangents, torsion and
// rolling.
enum class ConstraintKind : std::uint8_t {
  FRICTION,
  LIMIT,
  FRICTIONLESS,
  PYRAMIDAL,
  ELLIPTIC,
};

// An island's dynamics without constraints, and its rows. A group is the
// rows one constraint makes: one, but a pyramidal contact's 2 (dim - 1) and
// an elliptic contact's dim.
struct ConstraintProblem final {
  std::uint32_t dofs = 0;
  std::vector<double> mass;  // dofs by dofs.
  std::vector<double> qacc_smooth;
  std::vector<double> qfrc_smooth;
  std::vector<double> warmstart;  // The last step's accelerations.

  // By row.
  std::vector<double> jacobian;  // Rows by dofs.
  std::vector<ConstraintKind> kind;
  std::vector<std::uint32_t> group;  // Its group's first row.
  std::vector<double> pos;           // Distance, negative when violated.
  std::vector<double> margin;
  std::vector<double> friction_loss;
  std::vector<double> diagonal;  // The inverse inertia it sees, roughly.
  std::vector<double> velocity;  // J qvel.
  // A contact's friction: sliding twice, torsional, rolling twice.
  std::vector<std::array<double, 5>> friction;
  std::vector<SoftConstraint> soft;

  auto rows() const -> std::uint32_t {
    return static_cast<std::uint32_t>(kind.size());
  }
};

// How to solve: the step, and the solver's choice, iterations and
// tolerance; PGS scales its improvement by the model's mean inertia.
struct ConstraintSettings final {
  double timestep = 0.002;
  Physics::Solver solver = Physics::Solver::NEWTON;
  std::uint32_t iterations = 100;
  double tolerance = 1e-8;
  double mean_inertia = 1.0;
  std::uint32_t model_dofs = 1;
};

struct ConstraintSolution final {
  std::vector<double> qacc;
  std::vector<double> qfrc_constraint;
  std::vector<double> force;  // By row.
  std::uint32_t iterations = 0;
};

// The accelerations and constraint forces of `problem`.
auto solve_constraints(const ConstraintProblem& problem,
                       const ConstraintSettings& settings,
                       Out<ConstraintSolution> solution) -> void;

}  // namespace simon::model
