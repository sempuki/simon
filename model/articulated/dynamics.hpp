// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows MuJoCo 3.14.0, Copyright 2021 DeepMind Technologies Limited,
// Apache-2.0; translated to C++ and changed. See NOTICE.md.

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "Eigen/LU"
#include "core/argument.hpp"
#include "core/lie.hpp"
#include "core/math.hpp"
#include "model/articulated/articulated.hpp"

// The smooth dynamics of one kinematic tree, as MuJoCo computes a model's
// (see model/REFERENCES.md): forward kinematics; each body's inertia and each
// degree of freedom's motion in a frame at the tree's center of mass; the
// mass matrix by the composite rigid body algorithm and its LDLᵀ along the
// tree; the bias forces by recursive Newton–Euler; springs, dampers and
// motors; and the accelerations they give. Then semi-implicit Euler, with the
// dampers taken implicitly. Everything is sized at compile time by the
// tree's capacity, so a tree's state and work lie inline. Motions and forces
// are twists and wrenches, rotation first (core/lie.hpp); the mass matrix and
// its factor lie row by row.
namespace simon::articulated {

// A body's inertia about a point, in the world's axes: its rotational inertia
// about that point, its first moment (its mass times its center of mass's
// offset from the point), and its mass (Featherstone, 2.13). Inertias of
// bodies about one point add.
struct SpatialInertia final {
  // The momentum of a body that moves with `motion`.
  auto operator*(const Vector6& motion) const -> Vector6 {
    Vector3 w = motion.head<3>();
    Vector3 v = motion.tail<3>();
    Vector6 momentum;
    momentum << rotational * w + moment.cross(v), mass * v - moment.cross(w);
    return momentum;
  }

  auto operator+=(const SpatialInertia& other) -> SpatialInertia& {
    rotational += other.rotational;
    moment += other.moment;
    mass += other.mass;
    return *this;
  }

  Matrix3 rotational = Matrix3::Zero();
  Vector3 moment = Vector3::Zero();
  double mass = 0.0;
};

// How large a tree may be: its bodies and degrees of freedom, and so its
// positions, at most one quaternion's extra per body.
template <std::size_t BODIES, std::size_t DOFS>
struct TreeCapacity final {
  static constexpr std::size_t bodies = BODIES;
  static constexpr std::size_t dofs = DOFS;
  static constexpr std::size_t qpos = DOFS + BODIES;
};

// A tree's state: its positions and velocities, and the accelerations of
// its last step, where the constraint solver starts.
template <typename Capacity>
struct TreeState final {
  std::array<double, Capacity::qpos> qpos{};
  std::array<double, Capacity::dofs> qvel{};
  std::array<double, Capacity::dofs> warmstart{};
};

// A tree's actuators' controls, by the tree's actuators.
template <typename Capacity>
struct TreeControl final {
  std::array<double, Capacity::dofs> control{};
};

// What a step computes on the way to a tree's accelerations, kept for
// collisions and constraints: poses in the world, motion and inertia at the
// tree's center of mass, the factored mass matrix, and the forces. Each array
// holds the tree's bodies, joints or dofs first; what lies past them is not
// written.
template <typename Capacity>
struct TreeDynamics final {
  static constexpr std::size_t B = Capacity::bodies;
  static constexpr std::size_t V = Capacity::dofs;

  std::array<Vector3, B> xpos;
  std::array<Quaternion, B> xquat;
  std::array<Matrix3, B> xmat;
  std::array<Vector3, B> xipos;
  std::array<Matrix3, B> ximat;
  std::array<Vector3, V> xanchor;  // By joint.
  std::array<Vector3, V> xaxis;
  Vector3 com = Vector3::Zero();  // The tree's center of mass.
  std::array<SpatialInertia, B> cinert;
  std::array<Vector6, V> cdof;
  std::array<Vector6, V> cdof_dot;
  std::array<Vector6, B> cvel;
  std::array<double, V * V> mass{};    // M(i, j), j an ancestor of i or i.
  std::array<double, V * V> factor{};  // L of M, its diagonal D.
  std::array<double, V> inverse_diagonal{};
  std::array<double, V> bias{};
  std::array<double, V> passive{};
  std::array<double, V> actuator{};
  std::array<double, V> actuator_force{};  // By the tree's actuators.
  std::array<double, V> smooth{};          // passive - bias + actuator.
  std::array<double, V> acceleration{};    // M⁻¹ smooth.
};

constexpr std::uint32_t NONE = ~std::uint32_t{0};

// A body's inertia about a point `offset` from its center of mass, its
// principal moments `principal` on axes `axes`, by the parallel axis theorem
// (mju_inertCom; Featherstone, 2.63).
inline auto shift_inertia(const Vector3& principal, const Matrix3& axes,
                          const Vector3& offset, double mass)
    -> SpatialInertia {
  Vector3 moment = mass * offset;
  Matrix3 rotational;
  rotational.noalias() = axes * principal.asDiagonal() * axes.transpose();
  rotational.noalias() -= moment * offset.transpose();
  rotational.diagonal().array() += moment.dot(offset);
  return {.rotational = rotational, .moment = moment, .mass = mass};
}

// A rotation about `axis` at `offset` from the center of mass, as a motion
// there, and a translation along `axis`.
inline auto turn_about(const Vector3& axis, const Vector3& offset) -> Vector6 {
  Vector6 motion;
  motion << axis, axis.cross(offset);
  return motion;
}

inline auto slide_along(const Vector3& axis) -> Vector6 {
  Vector6 motion;
  motion << Vector3::Zero(), axis;
  return motion;
}

// The sum of `dofs` weighted by `w` (mju_mulDofVec).
inline auto combine(std::span<const Vector6> dofs, std::span<const double> w)
    -> Vector6 {
  Vector6 sum = Vector6::Zero();
  for (std::size_t j = 0; j < w.size(); ++j) {
    sum += dofs[j] * w[j];
  }
  return sum;
}

// `q` turned by angular velocity `rate`, in its own frame, for `dt`
// (mju_quatIntegrate).
inline auto integrate_quaternion(const Quaternion& q, const Vector3& rate,
                                 double dt) -> Quaternion {
  return q.normalized() * so3::exp(rate * dt);
}

// The Jacobian of `point` on body `body` of `tree`, its translation and
// rotation each 3 rows by the tree's dofs, from each dof's motion at the
// tree's center of mass `com` (mj_jac).
inline auto compute_point_jacobian(const Scene& model, const Tree& tree,
                                   std::span<const Vector6> cdof,
                                   const Vector3& com, std::uint32_t body,
                                   const Vector3& point,
                                   std::span<double> translation,
                                   std::span<double> rotation) -> void {
  std::uint32_t n = tree.dofs;
  std::fill(translation.begin(), translation.end(), 0.0);
  std::fill(rotation.begin(), rotation.end(), 0.0);
  while (body != 0 && model.bodies[body].dofs == 0) {
    body = model.bodies[body].parent;
  }
  if (body == 0) {
    return;
  }
  const Body& b = model.bodies[body];
  Vector3 offset = point - com;
  for (std::uint32_t d = b.first_dof + b.dofs - 1; d != Dof::NONE;
       d = model.dofs[d].parent) {
    std::uint32_t c = d - tree.first_dof;
    const Vector6& s = cdof[c];
    Vector3 moved = s.tail<3>() + s.head<3>().cross(offset);
    for (std::uint32_t k = 0; k < 3; ++k) {
      translation[k * n + c] = moved[k];
      rotation[k * n + c] = s[k];
    }
  }
}

// One tree's dynamics, over a model's arrays.
template <typename Capacity>
class TreeKernel final {
 public:
  using State = TreeState<Capacity>;
  using Control = TreeControl<Capacity>;
  using Dynamics = TreeDynamics<Capacity>;
  static constexpr std::size_t V = Capacity::dofs;

  // `implicit` if any dof of the model is damped or actuated, which makes
  // MuJoCo take every dof's dampers implicitly.
  TreeKernel(const Scene& model, const Tree& tree, bool implicit)
      : model_{&model}, tree_{&tree}, implicit_{implicit} {
    using articulated::NONE;
    for (std::uint32_t b = 0; b < tree.bodies; ++b) {
      std::uint32_t parent = model.bodies[tree.first_body + b].parent;
      parent_[b] = parent == 0 ? NONE : parent - tree.first_body;
    }
    for (std::uint32_t d = 0; d < tree.dofs; ++d) {
      const Dof& dof = model.dofs[tree.first_dof + d];
      dof_parent_[d] =
          dof.parent == Dof::NONE ? NONE : dof.parent - tree.first_dof;
      dof_body_[d] = dof.body - tree.first_body;
    }
    // Each dof's ancestors, from the root down.
    for (std::uint32_t d = 0; d < tree.dofs; ++d) {
      std::array<std::uint32_t, V> chain{};
      std::uint32_t count = 0;
      for (std::uint32_t j = dof_parent_[d]; j != NONE; j = dof_parent_[j]) {
        chain[count++] = j;
      }
      depth_[d] = count;
      for (std::uint32_t i = 0; i < count; ++i) {
        ancestors_[d * V + i] = chain[count - 1 - i];
      }
    }
    // Subtree masses, children added in reverse order.
    for (std::uint32_t b = 0; b < tree.bodies; ++b) {
      subtree_mass_[b] = model.bodies[tree.first_body + b].mass;
    }
    for (std::uint32_t b = tree.bodies; b-- > 0;) {
      if (parent_[b] != NONE) {
        subtree_mass_[parent_[b]] += subtree_mass_[b];
      }
    }
    // Each dof's damping, with its actuators' times the gear squared
    // (mj_actuatorDamping).
    for (std::uint32_t d = 0; d < tree.dofs; ++d) {
      damping_[d] = model.dofs[tree.first_dof + d].damping;
    }
    for (std::uint32_t a : tree.actuators) {
      const Actuator& actuator = model.actuators[a];
      double gear2 = actuator.gear[0] * actuator.gear[0];
      std::uint32_t d = model.joints[actuator.joint].dof - tree.first_dof;
      damping_[d] = damping_[d] + actuator.damping * gear2;
    }
  }

  // Everything to the accelerations without constraints.
  auto forward(const State& state, const Control& control,
               Out<Dynamics> out) const -> void {
    compute_kinematics(state, out);
    compute_com(out);
    compute_mass(out);
    factor(out->mass, Out(out->factor), Out(out->inverse_diagonal));
    compute_velocities(state, out);
    compute_bias(state, out);
    compute_passive(state, out);
    compute_actuation(state, control, out);
    for (std::uint32_t i = 0; i < tree_->dofs; ++i) {
      out->smooth[i] = out->passive[i] - out->bias[i];
      out->smooth[i] += out->actuator[i];
      out->acceleration[i] = out->smooth[i];
    }
    solve(out->factor, out->inverse_diagonal, out->acceleration);
  }

  // A step of semi-implicit Euler from `state` at `acceleration`, with the
  // dampers implicit when any dof of the model is damped or actuated, the
  // accelerations then from `force` (mj_Euler, mj_advance).
  auto advance(const Dynamics& dynamics, std::span<const double> force,
               std::span<const double> acceleration, InOut<State> state) const
      -> void {
    const Scene& m = *model_;
    double h = m.physics.timestep;
    std::array<double, V> qacc{};
    if (m.physics.integrator == Physics::Integrator::IMPLICIT_FAST) {
      solve_implicit_fast(dynamics, force, state->qvel, qacc);
    } else if (!implicit_) {
      for (std::uint32_t i = 0; i < tree_->dofs; ++i) {
        qacc[i] = acceleration[i];
      }
    } else {
      std::array<double, V * V> h_mass = dynamics.mass;
      for (std::uint32_t i = 0; i < tree_->dofs; ++i) {
        h_mass[i * V + i] += h * damping_[i];
      }
      std::array<double, V * V> h_factor{};
      std::array<double, V> h_inverse{};
      factor(h_mass, Out(h_factor), Out(h_inverse));
      for (std::uint32_t i = 0; i < tree_->dofs; ++i) {
        qacc[i] = force[i];
      }
      solve(h_factor, h_inverse, qacc);
    }
    for (std::uint32_t i = 0; i < tree_->dofs; ++i) {
      state->qvel[i] += qacc[i] * h;
    }
    integrate_positions(state, h);
  }

  // The accelerations of implicitfast: (M - h D)⁻¹ force, D the forces'
  // derivative in the velocities but for the velocity products, by dampers
  // and actuators; for a lone free body, its own 6 by 6 solve with the
  // velocity products' derivative too (mj_implicitSkip, mjd_smooth_vel,
  // mjd_freeMhat).
  auto solve_implicit_fast(const Dynamics& dynamics,
                           std::span<const double> force,
                           std::span<const double> qvel,
                           std::span<double> qacc) const -> void {
    const Scene& m = *model_;
    double h = m.physics.timestep;
    std::uint32_t n = tree_->dofs;
    std::array<double, V> deriv{};  // Its diagonal; joint actuators and
                                    // dampers touch nothing else.
    for (std::size_t a = 0; a < tree_->actuators.size(); ++a) {
      const Actuator& actuator = m.actuators[tree_->actuators[a]];
      double f = dynamics.actuator_force[a];
      if (actuator.force_limited &&
          (f <= actuator.force_range[0] || f >= actuator.force_range[1])) {
        continue;
      }
      if (actuator.bias_type == Actuator::Bias::AFFINE &&
          actuator.bias[2] != 0) {
        double gear = actuator.gear[0];
        std::uint32_t d = m.joints[actuator.joint].dof - tree_->first_dof;
        deriv[d] += gear * actuator.bias[2] * gear;
      }
    }
    for (std::uint32_t i = 0; i < n; ++i) {
      deriv[i] -= damping_[i];
    }
    bool free = free_body();
    std::array<double, V * V> h_mass = dynamics.mass;
    for (std::uint32_t i = free ? 6 : 0; i < n; ++i) {
      h_mass[i * V + i] = dynamics.mass[i * V + i] + -h * deriv[i];
    }
    std::array<double, V * V> h_factor{};
    std::array<double, V> h_inverse{};
    factor(h_mass, Out(h_factor), Out(h_inverse));
    for (std::uint32_t i = 0; i < n; ++i) {
      qacc[i] = force[i];
    }
    solve(h_factor, h_inverse, qacc);
    if (free) {
      solve_free_body(dynamics, force, qvel, deriv, qacc);
    }
  }

  // Whether the tree is one free body, its children fixed to it.
  auto free_body() const -> bool {
    const Body& root = model_->bodies[tree_->first_body];
    return root.joints == 1 &&
           model_->joints[root.first_joint].type == JointType::FREE &&
           tree_->dofs == 6;
  }

  // A free body's accelerations from A = M - h D + h B, B the velocity
  // products' derivative in its angular velocity, by LU with partial
  // pivoting (mjd_freeMhat, mjd_freeBias_vel).
  auto solve_free_body(const Dynamics& dynamics, std::span<const double> force,
                       std::span<const double> qvel,
                       const std::array<double, V>& deriv,
                       std::span<double> qacc) const -> void {
    double h = model_->physics.timestep;
    Matrix6 a;
    for (std::uint32_t r = 0; r < 6; ++r) {
      for (std::uint32_t c = 0; c <= r; ++c) {
        a(r, c) = a(c, r) = dynamics.mass[r * V + c];
      }
    }
    for (std::uint32_t r = 0; r < 6; ++r) {
      a(r, r) -= h * deriv[r];
    }
    // The tree's composite inertia about its center of mass.
    SpatialInertia composite;
    for (std::uint32_t b = 0; b < tree_->bodies; ++b) {
      composite += dynamics.cinert[b];
    }
    double mass = composite.mass;
    const Matrix3& inertia = composite.rotational;
    const Matrix3& r = dynamics.xmat[0];
    Vector3 s = dynamics.com - dynamics.xpos[0];
    Vector3 w = r * Vector3{qvel[3], qvel[4], qvel[5]};
    // The velocity products' derivatives in the body's angular velocity: the
    // linear momentum's through `k`, the angular's through `c`.
    Matrix3 k = s * w.transpose() - w.dot(s) * Matrix3::Identity() +
                so3::hat(w.cross(s));
    Matrix3 c =
        -mass * so3::hat(s) * k + so3::hat(w) * inertia - so3::hat(inertia * w);
    a.block<3, 3>(0, 3) += h * (-mass * k * r);
    a.block<3, 3>(3, 3) += h * (r.transpose() * c * r);
    Eigen::PartialPivLU<Matrix6> lu{a};
    if (lu.matrixLU().diagonal().cwiseAbs().minCoeff() < articulated::MINVAL) {
      return;
    }
    Vector6 f;
    for (int i = 0; i < 6; ++i) {
      f[i] = force[i];
    }
    Vector6 x = lu.solve(f);
    for (int i = 0; i < 6; ++i) {
      qacc[i] = x[i];
    }
  }

  // Positions moved by the velocities for `dt` (mj_integratePos).
  auto integrate_positions(InOut<State> state, double dt) const -> void {
    const Scene& m = *model_;
    for (std::uint32_t j = 0; j < tree_->joints; ++j) {
      const Joint& joint = m.joints[tree_->first_joint + j];
      std::uint32_t p = joint.qpos - tree_->first_qpos;
      std::uint32_t v = joint.dof - tree_->first_dof;
      switch (joint.type) {
        case JointType::FREE:
          for (std::uint32_t k = 0; k < 3; ++k) {
            state->qpos[p + k] += dt * state->qvel[v + k];
          }
          p += 3;
          v += 3;
          [[fallthrough]];
        case JointType::BALL: {
          Quaternion q = articulated::integrate_quaternion(
              read_quaternion(state->qpos, p),
              {state->qvel[v], state->qvel[v + 1], state->qvel[v + 2]}, dt);
          write_quaternion(q, p, state->qpos);
          break;
        }
        default:
          state->qpos[p] += dt * state->qvel[v];
          break;
      }
    }
  }

  // x = M⁻¹ x, from a factor (mj_solveLD).
  auto solve(const std::array<double, V * V>& f,
             const std::array<double, V>& inverse, std::span<double> x) const
      -> void {
    using articulated::NONE;
    std::uint32_t n = tree_->dofs;
    for (std::uint32_t k = n; k-- > 0;) {
      if (dof_parent_[k] == NONE || x[k] == 0) {
        continue;
      }
      double xk = x[k];
      for_ancestors(k, [&](std::uint32_t c) { x[c] -= f[k * V + c] * xk; });
    }
    for (std::uint32_t k = 0; k < n; ++k) {
      x[k] *= inverse[k];
    }
    for (std::uint32_t k = 0; k < n; ++k) {
      if (dof_parent_[k] == NONE) {
        continue;
      }
      double sum = 0;
      for_ancestors(k, [&](std::uint32_t c) { sum += f[k * V + c] * x[c]; });
      x[k] -= sum;
    }
  }

 private:
  // The quaternion at `p` among positions `qpos`, w, x, y, z.
  static auto read_quaternion(std::span<const double> qpos, std::uint32_t p)
      -> Quaternion {
    return {qpos[p], qpos[p + 1], qpos[p + 2], qpos[p + 3]};
  }

  static auto write_quaternion(const Quaternion& q, std::uint32_t p,
                               std::span<double> qpos) -> void {
    qpos[p] = q.w();
    qpos[p + 1] = q.x();
    qpos[p + 2] = q.y();
    qpos[p + 3] = q.z();
  }

  // Calls `f` on each ancestor of dof `k`, from the root down.
  template <typename F>
  auto for_ancestors(std::uint32_t k, F f) const -> void {
    for (std::uint32_t i = 0; i < depth_[k]; ++i) {
      f(ancestors_[k * V + i]);
    }
  }

  // mj_kinematics1 and the inertial frames of mj_kinematics2.
  auto compute_kinematics(const State& state, Out<Dynamics> out) const -> void {
    using articulated::NONE;
    const Scene& m = *model_;
    for (std::uint32_t b = 0; b < tree_->bodies; ++b) {
      const Body& body = m.bodies[tree_->first_body + b];
      Vector3 xpos = Vector3::Zero();
      Quaternion xquat = Quaternion::Identity();
      if (body.joints == 1 &&
          m.joints[body.first_joint].type == JointType::FREE) {
        const Joint& joint = m.joints[body.first_joint];
        std::uint32_t q = joint.qpos - tree_->first_qpos;
        std::uint32_t j = body.first_joint - tree_->first_joint;
        xpos = {state.qpos[q], state.qpos[q + 1], state.qpos[q + 2]};
        xquat = read_quaternion(state.qpos, q + 3);
        out->xanchor[j] = xpos;
        out->xaxis[j] = joint.axis;
      } else {
        std::uint32_t p = parent_[b];
        if (p != NONE) {
          xpos = out->xmat[p] * body.pos + out->xpos[p];
          xquat = out->xquat[p] * body.quat;
        } else {
          xpos = body.pos;
          xquat = body.quat;
        }
        for (std::uint32_t jj = 0; jj < body.joints; ++jj) {
          const Joint& joint = m.joints[body.first_joint + jj];
          std::uint32_t j = body.first_joint + jj - tree_->first_joint;
          std::uint32_t q = joint.qpos - tree_->first_qpos;
          Vector3 xaxis = xquat * joint.axis;
          Vector3 xanchor = xquat * joint.pos + xpos;
          if (joint.type == JointType::SLIDE) {
            xpos += xaxis * (state.qpos[q] - m.qpos0[joint.qpos]);
          } else {
            Quaternion local =
                joint.type == JointType::BALL
                    ? read_quaternion(state.qpos, q).normalized()
                    : Quaternion{AngleAxis{state.qpos[q] - m.qpos0[joint.qpos],
                                           joint.axis}};
            xquat = xquat * local;
            xpos = xanchor - xquat * joint.pos;
          }
          out->xanchor[j] = xanchor;
          out->xaxis[j] = xaxis;
        }
      }
      xquat.normalize();
      out->xquat[b] = xquat;
      out->xpos[b] = xpos;
      out->xmat[b] = xquat.toRotationMatrix();
      out->xipos[b] = body.inertial_frame == SameFrame::BODY
                          ? xpos
                          : Vector3{out->xmat[b] * body.inertial_pos + xpos};
      out->ximat[b] =
          body.inertial_frame == SameFrame::NONE
              ? Matrix3{(xquat * body.inertial_quat).toRotationMatrix()}
              : out->xmat[b];
    }
  }

  // mj_comPos.
  auto compute_com(Out<Dynamics> out) const -> void {
    const Scene& m = *model_;
    // Each subtree's first moment about the world's origin.
    std::array<Vector3, Capacity::bodies> subtree;
    for (std::uint32_t b = 0; b < tree_->bodies; ++b) {
      subtree[b] = out->xipos[b] * m.bodies[tree_->first_body + b].mass;
    }
    for (std::uint32_t b = tree_->bodies; b-- > 0;) {
      if (parent_[b] != NONE) {
        subtree[parent_[b]] += subtree[b];
      }
    }
    out->com = subtree_mass_[0] < MINVAL
                   ? out->xipos[0]
                   : Vector3{subtree[0] / subtree_mass_[0]};
    for (std::uint32_t b = 0; b < tree_->bodies; ++b) {
      const Body& body = m.bodies[tree_->first_body + b];
      out->cinert[b] = shift_inertia(body.inertia, out->ximat[b],
                                     out->xipos[b] - out->com, body.mass);
    }
    for (std::uint32_t b = 0; b < tree_->bodies; ++b) {
      const Body& body = m.bodies[tree_->first_body + b];
      for (std::uint32_t jj = 0; jj < body.joints; ++jj) {
        const Joint& joint = m.joints[body.first_joint + jj];
        std::uint32_t j = body.first_joint + jj - tree_->first_joint;
        std::uint32_t da = joint.dof - tree_->first_dof;
        Vector3 offset = out->com - out->xanchor[j];
        std::uint32_t skip = 0;
        switch (joint.type) {
          case JointType::FREE:
            for (std::uint32_t k = 0; k < 3; ++k) {
              out->cdof[da + k] = slide_along(Vector3::Unit(k));
            }
            skip = 3;
            [[fallthrough]];
          case JointType::BALL:
            for (std::uint32_t k = 0; k < 3; ++k) {
              out->cdof[da + skip + k] =
                  turn_about(out->xmat[b].col(k), offset);
            }
            break;
          case JointType::SLIDE:
            out->cdof[da] = slide_along(out->xaxis[j]);
            break;
          case JointType::HINGE:
            out->cdof[da] = turn_about(out->xaxis[j], offset);
            break;
        }
      }
    }
  }

  // mj_crb, without the fixed inertia MuJoCo keeps for simple dofs.
  auto compute_mass(Out<Dynamics> out) const -> void {
    using articulated::NONE;
    const Scene& m = *model_;
    std::array<SpatialInertia, Capacity::bodies> crb = out->cinert;
    for (std::uint32_t b = tree_->bodies; b-- > 0;) {
      if (parent_[b] != NONE) {
        crb[parent_[b]] += crb[b];
      }
    }
    out->mass.fill(0.0);
    for (std::uint32_t i = 0; i < tree_->dofs; ++i) {
      out->mass[i * V + i] = m.dofs[tree_->first_dof + i].armature;
      Vector6 momentum = crb[dof_body_[i]] * out->cdof[i];
      for (std::uint32_t j = i; j != NONE; j = dof_parent_[j]) {
        out->mass[i * V + j] += out->cdof[j].dot(momentum);
      }
    }
  }

  // L'DL of `mass` along the tree, clamping pivots (mj_factorI).
  auto factor(const std::array<double, V * V>& mass,
              Out<std::array<double, V * V>> factor_out,
              Out<std::array<double, V>> inverse_out) const -> void {
    std::array<double, V * V>& f = *factor_out;
    std::array<double, V>& inverse = *inverse_out;
    f = mass;
    for (std::uint32_t k = tree_->dofs; k-- > 0;) {
      double& pivot = f[k * V + k];
      if (pivot < MINVAL) {
        pivot = MINVAL;
      }
      double inv = 1 / pivot;
      inverse[k] = inv;
      // Each ancestor, nearest first: its row less row k's, scaled.
      for (std::uint32_t i = dof_parent_[k]; i != NONE; i = dof_parent_[i]) {
        double scale = -f[k * V + i] * inv;
        f[i * V + i] += f[k * V + i] * scale;
        for_ancestors(
            i, [&](std::uint32_t c) { f[i * V + c] += f[k * V + c] * scale; });
      }
      for_ancestors(k, [&](std::uint32_t c) { f[k * V + c] *= inv; });
    }
  }

  // mj_comVel.
  auto compute_velocities(const State& state, Out<Dynamics> out) const -> void {
    const Scene& m = *model_;
    for (std::uint32_t b = 0; b < tree_->bodies; ++b) {
      const Body& body = m.bodies[tree_->first_body + b];
      Vector6 cvel =
          parent_[b] != NONE ? out->cvel[parent_[b]] : Vector6::Zero();
      std::uint32_t bda = body.first_dof - tree_->first_dof;
      auto add = [&](std::uint32_t first, std::size_t count) {
        cvel += combine(std::span{out->cdof}.subspan(first, count),
                        std::span{state.qvel}.subspan(first, count));
      };
      for (std::uint32_t j = 0; j < body.dofs; ++j) {
        const Dof& dof = m.dofs[body.first_dof + j];
        JointType type = m.joints[dof.joint].type;
        if (type == JointType::FREE) {
          for (std::uint32_t k = 0; k < 3; ++k) {
            out->cdof_dot[bda + j + k] = Vector6::Zero();
          }
          add(bda + j, 3);
          j += 3;
          type = JointType::BALL;
        }
        if (type == JointType::BALL) {
          for (std::uint32_t k = 0; k < 3; ++k) {
            out->cdof_dot[bda + j + k] =
                se3::cross_motion(cvel, out->cdof[bda + j + k]);
          }
          add(bda + j, 3);
          j += 2;
        } else {
          out->cdof_dot[bda + j] = se3::cross_motion(cvel, out->cdof[bda + j]);
          add(bda + j, 1);
        }
      }
      out->cvel[b] = cvel;
    }
  }

  // mj_rne without accelerations: gravity and velocity products.
  auto compute_bias(const State& state, Out<Dynamics> out) const -> void {
    const Scene& m = *model_;
    // Gravity, as the world accelerating the other way.
    Vector6 world;
    world << Vector3::Zero(), -m.physics.gravity;
    std::array<Vector6, Capacity::bodies> cacc;
    std::array<Vector6, Capacity::bodies> force;
    for (std::uint32_t b = 0; b < tree_->bodies; ++b) {
      const Body& body = m.bodies[tree_->first_body + b];
      std::uint32_t bda = body.first_dof - tree_->first_dof;
      const Vector6& before = parent_[b] != NONE ? cacc[parent_[b]] : world;
      cacc[b] =
          before + combine(std::span{out->cdof_dot}.subspan(bda, body.dofs),
                           std::span{state.qvel}.subspan(bda, body.dofs));
      force[b] = out->cinert[b] * cacc[b] +
                 se3::cross_force(out->cvel[b], out->cinert[b] * out->cvel[b]);
    }
    for (std::uint32_t b = tree_->bodies; b-- > 0;) {
      if (parent_[b] != NONE) {
        force[parent_[b]] += force[b];
      }
    }
    for (std::uint32_t i = 0; i < tree_->dofs; ++i) {
      out->bias[i] = out->cdof[i].dot(force[dof_body_[i]]);
    }
  }

  // Joint springs and dof dampers (mj_springdamper).
  auto compute_passive(const State& state, Out<Dynamics> out) const -> void {
    const Scene& m = *model_;
    out->passive.fill(0.0);
    for (std::uint32_t j = 0; j < tree_->joints; ++j) {
      const Joint& joint = m.joints[tree_->first_joint + j];
      if (joint.stiffness == 0) {
        continue;
      }
      std::uint32_t q = joint.qpos - tree_->first_qpos;
      std::uint32_t v = joint.dof - tree_->first_dof;
      std::uint32_t qs = joint.qpos;
      switch (joint.type) {
        case JointType::FREE:
        case JointType::BALL: {
          std::uint32_t skip = joint.type == JointType::FREE ? 3 : 0;
          for (std::uint32_t k = 0; k < skip; ++k) {
            out->passive[v + k] +=
                (state.qpos[q + k] - m.qpos_spring[qs + k]) * -joint.stiffness;
          }
          // The turn from the spring's rest to the joint's quaternion.
          Quaternion turned =
              read_quaternion(m.qpos_spring, qs + skip).conjugate() *
              read_quaternion(state.qpos, q + skip).normalized();
          Vector3 angle = so3::log(turned);
          for (std::uint32_t k = 0; k < 3; ++k) {
            out->passive[v + skip + k] += angle[k] * -joint.stiffness;
          }
          break;
        }
        default: {
          double x = state.qpos[q] - m.qpos_spring[joint.qpos];
          out->passive[v] = -x * joint.stiffness;
          break;
        }
      }
    }
    for (std::uint32_t i = 0; i < tree_->dofs; ++i) {
      double damping = damping_[i];
      if (damping != 0) {
        double damper = -state.qvel[i] * damping;
        out->passive[i] = out->passive[i] + damper;
      }
    }
  }

  // Actuators: each control clamped to its range, times its gain, plus its
  // bias in the joint's length and velocity, clamped to its force range,
  // times its gear on the joint (mj_transmission, mj_fwdActuation).
  auto compute_actuation(const State& state, const Control& control,
                         Out<Dynamics> out) const -> void {
    const Scene& m = *model_;
    out->actuator.fill(0.0);
    for (std::size_t a = 0; a < tree_->actuators.size(); ++a) {
      const Actuator& actuator = m.actuators[tree_->actuators[a]];
      const Joint& joint = m.joints[actuator.joint];
      std::uint32_t q = joint.qpos - tree_->first_qpos;
      std::uint32_t v = joint.dof - tree_->first_dof;
      double gear = actuator.gear[0];
      double input = control.control[a];
      if (actuator.control_limited) {
        input = std::clamp(input, actuator.control_range[0],
                           actuator.control_range[1]);
      }
      double force = actuator.gain[0] * input;
      if (actuator.bias_type == Actuator::Bias::AFFINE) {
        double length = state.qpos[q] * gear;
        double velocity = gear * state.qvel[v];
        force += actuator.bias[0] + actuator.bias[1] * length +
                 actuator.bias[2] * velocity;
      }
      if (actuator.force_limited) {
        force =
            std::clamp(force, actuator.force_range[0], actuator.force_range[1]);
      }
      out->actuator_force[a] = force;
      out->actuator[v] += gear * force;
    }
  }

  const Scene* model_ = nullptr;
  const Tree* tree_ = nullptr;
  bool implicit_ = false;
  std::array<std::uint32_t, Capacity::bodies> parent_{};
  std::array<std::uint32_t, V> dof_parent_{};
  std::array<std::uint32_t, V> dof_body_{};
  std::array<std::uint32_t, V> depth_{};  // By dof, its ancestors' count.
  std::array<std::uint32_t, V * V> ancestors_{};  // By dof, root first.
  std::array<double, Capacity::bodies> subtree_mass_{};
  std::array<double, V> damping_{};  // By dof, with its actuators'.
};

}  // namespace simon::articulated
