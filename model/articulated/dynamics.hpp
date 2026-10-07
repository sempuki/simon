// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows MuJoCo 3.14.0, Copyright 2021 DeepMind Technologies Limited,
// Apache-2.0; translated to C++ and changed. See NOTICE.md.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>

#include "core/vocabulary.hpp"
#include "model/articulated/arithmetic.hpp"
#include "model/articulated/articulated.hpp"

// The smooth dynamics of one kinematic tree, as MuJoCo computes a model's
// (see model/REFERENCES.md): forward kinematics; each body's inertia and each
// degree of freedom's motion in a frame at the tree's center of mass; the
// mass matrix by the composite rigid body algorithm and its LDLᵀ along the
// tree; the bias forces by recursive Newton–Euler; springs, dampers and
// motors; and the accelerations they give. Then semi-implicit Euler, with the
// dampers taken implicitly. Everything is sized at compile time by the
// tree's capacity, so a tree's state and work lie inline. Spatial vectors are
// rotation then translation; matrices row by row.
namespace simon::model {

using Spatial = std::array<double, 6>;
using Inertia10 = std::array<double, 10>;  // xx yy zz xy xz yz, m c, m.

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
// tree's center of mass, the factored mass matrix, and the forces.
template <typename Capacity>
struct TreeDynamics final {
  static constexpr std::size_t B = Capacity::bodies;
  static constexpr std::size_t V = Capacity::dofs;

  std::array<Vector3, B> xpos{};
  std::array<Quaternion4, B> xquat{};
  std::array<Matrix3, B> xmat{};
  std::array<Vector3, B> xipos{};
  std::array<Matrix3, B> ximat{};
  std::array<Vector3, V> xanchor{};  // By joint.
  std::array<Vector3, V> xaxis{};
  Vector3 com = Vector3::Zero();  // The tree's center of mass.
  std::array<Inertia10, B> cinert{};
  std::array<Spatial, V> cdof{};
  std::array<Spatial, V> cdof_dot{};
  std::array<Spatial, B> cvel{};
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

namespace articulated {

constexpr std::uint32_t NONE = ~std::uint32_t{0};

inline auto dot6(const Spatial& a, const Spatial& b) -> double {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3] + a[4] * b[4] +
         a[5] * b[5];
}

// The cross product of a motion with a motion, and with a force.
inline auto cross_motion(const Spatial& vel, const Spatial& v) -> Spatial {
  Spatial r{-vel[2] * v[1] + vel[1] * v[2], vel[2] * v[0] - vel[0] * v[2],
            -vel[1] * v[0] + vel[0] * v[1], -vel[2] * v[4] + vel[1] * v[5],
            vel[2] * v[3] - vel[0] * v[5],  -vel[1] * v[3] + vel[0] * v[4]};
  r[3] += -vel[5] * v[1] + vel[4] * v[2];
  r[4] += vel[5] * v[0] - vel[3] * v[2];
  r[5] += -vel[4] * v[0] + vel[3] * v[1];
  return r;
}

inline auto cross_force(const Spatial& vel, const Spatial& f) -> Spatial {
  Spatial r{-vel[2] * f[1] + vel[1] * f[2], vel[2] * f[0] - vel[0] * f[2],
            -vel[1] * f[0] + vel[0] * f[1], -vel[2] * f[4] + vel[1] * f[5],
            vel[2] * f[3] - vel[0] * f[5],  -vel[1] * f[3] + vel[0] * f[4]};
  r[0] += -vel[5] * f[4] + vel[4] * f[5];
  r[1] += vel[5] * f[3] - vel[3] * f[5];
  r[2] += -vel[4] * f[3] + vel[3] * f[4];
  return r;
}

// A body's inertia about a point `dif` from its center of mass, its axes
// turned by `mat` (mju_inertCom).
inline auto shift_inertia(const Vector3& inert, const Matrix3& mat,
                          const Vector3& dif, double mass) -> Inertia10 {
  std::array<double, 9> t{
      mat[0] * inert[0], mat[3] * inert[0], mat[6] * inert[0],
      mat[1] * inert[1], mat[4] * inert[1], mat[7] * inert[1],
      mat[2] * inert[2], mat[5] * inert[2], mat[8] * inert[2]};
  Inertia10 r{mat[0] * t[0] + mat[1] * t[3] + mat[2] * t[6],
              mat[3] * t[1] + mat[4] * t[4] + mat[5] * t[7],
              mat[6] * t[2] + mat[7] * t[5] + mat[8] * t[8],
              mat[0] * t[1] + mat[1] * t[4] + mat[2] * t[7],
              mat[0] * t[2] + mat[1] * t[5] + mat[2] * t[8],
              mat[3] * t[2] + mat[4] * t[5] + mat[5] * t[8]};
  r[0] += mass * (dif[1] * dif[1] + dif[2] * dif[2]);
  r[1] += mass * (dif[0] * dif[0] + dif[2] * dif[2]);
  r[2] += mass * (dif[0] * dif[0] + dif[1] * dif[1]);
  r[3] -= mass * dif[0] * dif[1];
  r[4] -= mass * dif[0] * dif[2];
  r[5] -= mass * dif[1] * dif[2];
  r[6] = mass * dif[0];
  r[7] = mass * dif[1];
  r[8] = mass * dif[2];
  r[9] = mass;
  return r;
}

inline auto multiply(const Inertia10& i, const Spatial& v) -> Spatial {
  return {i[0] * v[0] + i[3] * v[1] + i[4] * v[2] - i[8] * v[4] + i[7] * v[5],
          i[3] * v[0] + i[1] * v[1] + i[5] * v[2] + i[8] * v[3] - i[6] * v[5],
          i[4] * v[0] + i[5] * v[1] + i[2] * v[2] - i[7] * v[3] + i[6] * v[4],
          i[8] * v[1] - i[7] * v[2] + i[9] * v[3],
          i[6] * v[2] - i[8] * v[0] + i[9] * v[4],
          i[7] * v[0] - i[6] * v[1] + i[9] * v[5]};
}

// A rotation's motion about the center of mass, and a translation's.
inline auto turn_about(const Vector3& axis, const Vector3& offset) -> Spatial {
  Vector3 c = cross(axis, offset);
  return {axis[0], axis[1], axis[2], c[0], c[1], c[2]};
}

inline auto slide_along(const Vector3& axis) -> Spatial {
  return {0.0, 0.0, 0.0, axis[0], axis[1], axis[2]};
}

// The sum of `n` motions weighted by `w` (mju_mulDofVec).
inline auto combine(std::span<const Spatial> dofs, std::span<const double> w)
    -> Spatial {
  Spatial r{};
  if (w.size() == 1) {
    for (std::size_t k = 0; k < 6; ++k) {
      r[k] = dofs[0][k] * w[0];
    }
    return r;
  }
  for (std::size_t j = 0; j < w.size(); ++j) {
    if (w[j] != 0) {
      for (std::size_t k = 0; k < 6; ++k) {
        r[k] += dofs[j][k] * w[j];
      }
    }
  }
  return r;
}

// The quaternion turned by angular velocity `vel` for `scale`
// (mju_quatIntegrate).
inline auto integrate_quaternion(Quaternion4 q, const Vector3& vel,
                                 double scale) -> Quaternion4 {
  Vector3 axis = vel;
  double angle = scale * normalize3(InOut(axis));
  Quaternion4 turn = convert_axis_angle(axis, angle);
  normalize4(InOut(q));
  return multiply(q, turn);
}

}  // namespace articulated

// The Jacobian of `point` on body `body` of `tree`, its translation and
// rotation each 3 rows by the tree's dofs, from each dof's motion at the
// tree's center of mass `com` (mj_jac).
inline auto compute_point_jacobian(const ArticulatedModel& model,
                                   const Tree& tree,
                                   std::span<const Spatial> cdof,
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
  const ArticulatedBody& b = model.bodies[body];
  Vector3 offset{point[0] - com[0], point[1] - com[1], point[2] - com[2]};
  for (std::uint32_t d = b.first_dof + b.dofs - 1; d != Dof::NONE;
       d = model.dofs[d].parent) {
    std::uint32_t c = d - tree.first_dof;
    const Spatial& s = cdof[c];
    Vector3 turn = articulated::cross({s[0], s[1], s[2]}, offset);
    for (std::uint32_t k = 0; k < 3; ++k) {
      translation[k * n + c] = s[3 + k] + turn[k];
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
  TreeKernel(const ArticulatedModel& model, const Tree& tree, bool implicit)
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
    const ArticulatedModel& m = *model_;
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
    const ArticulatedModel& m = *model_;
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
    const ArticulatedBody& root = model_->bodies[tree_->first_body];
    return root.joints == 1 &&
           model_->joints[root.first_joint].type == JointType::FREE &&
           tree_->dofs == 6;
  }

  // A free body's accelerations from A = M - h D + h B, B the velocity
  // products' derivative in its angular velocity, by LU with pivoting
  // (mjd_freeMhat, mjd_freeBias_vel, mju_factorLU6).
  auto solve_free_body(const Dynamics& dynamics, std::span<const double> force,
                       std::span<const double> qvel,
                       const std::array<double, V>& deriv,
                       std::span<double> qacc) const -> void {
    using namespace articulated;
    double h = model_->physics.timestep;
    std::array<double, 36> a{};
    for (std::uint32_t r = 0; r < 6; ++r) {
      for (std::uint32_t c = 0; c <= r; ++c) {
        a[6 * r + c] = a[6 * c + r] = dynamics.mass[r * V + c];
      }
    }
    for (std::uint32_t r = 0; r < 6; ++r) {
      a[6 * r + r] -= h * deriv[r];
    }
    // The tree's composite inertia about its center of mass.
    Inertia10 crb{};
    for (std::uint32_t b = 0; b < tree_->bodies; ++b) {
      for (int k = 0; k < 10; ++k) {
        crb[k] += dynamics.cinert[b][k];
      }
    }
    double mass = crb[9];
    std::array<double, 9> iw{crb[0], crb[3], crb[4], crb[3], crb[1],
                             crb[5], crb[4], crb[5], crb[2]};
    const Matrix3& r = dynamics.xmat[0];
    Vector3 s{dynamics.com[0] - dynamics.xpos[0][0],
              dynamics.com[1] - dynamics.xpos[0][1],
              dynamics.com[2] - dynamics.xpos[0][2]};
    Vector3 w = multiply(r, Vector3{qvel[3], qvel[4], qvel[5]});
    Vector3 ws = cross(w, s);
    Vector3 iww = multiply(iw, w);
    double w_dot_s = w[0] * s[0] + w[1] * s[1] + w[2] * s[2];
    std::array<double, 9> k{
        s[0] * w[0] - w_dot_s, s[0] * w[1] - ws[2],   s[0] * w[2] + ws[1],
        s[1] * w[0] + ws[2],   s[1] * w[1] - w_dot_s, s[1] * w[2] - ws[0],
        s[2] * w[0] - ws[1],   s[2] * w[1] + ws[0],   s[2] * w[2] - w_dot_s};
    auto product = [](const std::array<double, 9>& x,
                      const std::array<double, 9>& y) {
      std::array<double, 9> z{};
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          z[3 * i + j] = x[3 * i] * y[j] + x[3 * i + 1] * y[3 + j] +
                         x[3 * i + 2] * y[6 + j];
        }
      }
      return z;
    };
    std::array<double, 9> rm{r[0], r[1], r[2], r[3], r[4],
                             r[5], r[6], r[7], r[8]};
    std::array<double, 9> lin = product(k, rm);
    std::array<double, 9> c{};
    for (int col = 0; col < 3; ++col) {
      double sk0 = s[1] * k[6 + col] - s[2] * k[3 + col];
      double sk1 = s[2] * k[col] - s[0] * k[6 + col];
      double sk2 = s[0] * k[3 + col] - s[1] * k[col];
      double wi0 = w[1] * iw[6 + col] - w[2] * iw[3 + col];
      double wi1 = w[2] * iw[col] - w[0] * iw[6 + col];
      double wi2 = w[0] * iw[3 + col] - w[1] * iw[col];
      c[col] =
          -mass * sk0 + wi0 + (col == 1 ? iww[2] : (col == 2 ? -iww[1] : 0));
      c[3 + col] =
          -mass * sk1 + wi1 + (col == 0 ? -iww[2] : (col == 2 ? iww[0] : 0));
      c[6 + col] =
          -mass * sk2 + wi2 + (col == 0 ? iww[1] : (col == 1 ? -iww[0] : 0));
    }
    std::array<double, 9> rt{r[0], r[3], r[6], r[1], r[4],
                             r[7], r[2], r[5], r[8]};
    std::array<double, 9> rot = product(product(rt, c), rm);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        a[6 * i + 3 + j] += h * (-mass * lin[3 * i + j]);
        a[6 * (i + 3) + 3 + j] += h * rot[3 * i + j];
      }
    }
    // LU with partial pivoting, then the solve.
    std::array<int, 6> pivot{};
    for (int col = 0; col < 6; ++col) {
      pivot[col] = col;
      double largest = std::abs(a[col * 6 + col]);
      int row = col;
      for (int i = col + 1; i < 6; ++i) {
        if (std::abs(a[i * 6 + col]) > largest) {
          largest = std::abs(a[i * 6 + col]);
          row = i;
        }
      }
      if (largest < MINVAL) {
        return;
      }
      if (row != col) {
        pivot[col] = row;
        for (int j = 0; j < 6; ++j) {
          std::swap(a[col * 6 + j], a[row * 6 + j]);
        }
      }
      double inverse = 1.0 / a[col * 6 + col];
      for (int i = col + 1; i < 6; ++i) {
        a[i * 6 + col] *= inverse;
        for (int j = col + 1; j < 6; ++j) {
          a[i * 6 + j] -= a[i * 6 + col] * a[col * 6 + j];
        }
      }
    }
    std::array<double, 6> x{};
    for (int i = 0; i < 6; ++i) {
      x[i] = force[i];
    }
    for (int i = 0; i < 6; ++i) {
      if (pivot[i] != i) {
        std::swap(x[i], x[pivot[i]]);
      }
      for (int j = 0; j < i; ++j) {
        x[i] -= a[i * 6 + j] * x[j];
      }
    }
    for (int i = 5; i >= 0; --i) {
      for (int j = i + 1; j < 6; ++j) {
        x[i] -= a[i * 6 + j] * x[j];
      }
      x[i] /= a[i * 6 + i];
    }
    for (int i = 0; i < 6; ++i) {
      qacc[i] = x[i];
    }
  }

  // Positions moved by the velocities for `dt` (mj_integratePos).
  auto integrate_positions(InOut<State> state, double dt) const -> void {
    const ArticulatedModel& m = *model_;
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
          Quaternion4 q{state->qpos[p], state->qpos[p + 1], state->qpos[p + 2],
                        state->qpos[p + 3]};
          q = articulated::integrate_quaternion(
              q, {state->qvel[v], state->qvel[v + 1], state->qvel[v + 2]}, dt);
          for (std::uint32_t k = 0; k < 4; ++k) {
            state->qpos[p + k] = q[k];
          }
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
  // Calls `f` on each ancestor of dof `k`, from the root down.
  template <typename F>
  auto for_ancestors(std::uint32_t k, F f) const -> void {
    for (std::uint32_t i = 0; i < depth_[k]; ++i) {
      f(ancestors_[k * V + i]);
    }
  }

  // mj_kinematics1 and the inertial frames of mj_kinematics2.
  auto compute_kinematics(const State& state, Out<Dynamics> out) const -> void {
    using namespace articulated;
    const ArticulatedModel& m = *model_;
    for (std::uint32_t b = 0; b < tree_->bodies; ++b) {
      const ArticulatedBody& body = m.bodies[tree_->first_body + b];
      Vector3 xpos = Vector3::Zero();
      Quaternion4 xquat{};
      if (body.joints == 1 &&
          m.joints[body.first_joint].type == JointType::FREE) {
        const Joint& joint = m.joints[body.first_joint];
        std::uint32_t q = joint.qpos - tree_->first_qpos;
        std::uint32_t j = body.first_joint - tree_->first_joint;
        xpos = {state.qpos[q], state.qpos[q + 1], state.qpos[q + 2]};
        xquat = {state.qpos[q + 3], state.qpos[q + 4], state.qpos[q + 5],
                 state.qpos[q + 6]};
        normalize4(InOut(xquat));
        out->xanchor[j] = xpos;
        out->xaxis[j] = joint.axis;
      } else {
        std::uint32_t p = parent_[b];
        if (p != NONE) {
          xpos = multiply(out->xmat[p], body.pos);
          for (int k = 0; k < 3; ++k) {
            xpos[k] += out->xpos[p][k];
          }
          xquat = multiply(out->xquat[p], body.quat);
        } else {
          xpos = body.pos;
          xquat = body.quat;
        }
        for (std::uint32_t jj = 0; jj < body.joints; ++jj) {
          const Joint& joint = m.joints[body.first_joint + jj];
          std::uint32_t j = body.first_joint + jj - tree_->first_joint;
          std::uint32_t q = joint.qpos - tree_->first_qpos;
          Vector3 xaxis = rotate(joint.axis, xquat);
          Vector3 xanchor = rotate(joint.pos, xquat);
          for (int k = 0; k < 3; ++k) {
            xanchor[k] += xpos[k];
          }
          if (joint.type == JointType::SLIDE) {
            double s = state.qpos[q] - m.qpos0[joint.qpos];
            for (int k = 0; k < 3; ++k) {
              xpos[k] += xaxis[k] * s;
            }
          } else {
            Quaternion4 local{};
            if (joint.type == JointType::BALL) {
              local = {state.qpos[q], state.qpos[q + 1], state.qpos[q + 2],
                       state.qpos[q + 3]};
              normalize4(InOut(local));
            } else {
              local = convert_axis_angle(joint.axis,
                                         state.qpos[q] - m.qpos0[joint.qpos]);
            }
            xquat = multiply(xquat, local);
            Vector3 v = rotate(joint.pos, xquat);
            for (int k = 0; k < 3; ++k) {
              xpos[k] = xanchor[k] - v[k];
            }
          }
          out->xanchor[j] = xanchor;
          out->xaxis[j] = xaxis;
        }
      }
      normalize4(InOut(xquat));
      out->xquat[b] = xquat;
      out->xpos[b] = xpos;
      out->xmat[b] = convert_to_matrix(xquat);
      if (body.inertial_frame == SameFrame::BODY) {
        out->xipos[b] = xpos;
      } else {
        Vector3 xipos = multiply(out->xmat[b], body.inertial_pos);
        for (int k = 0; k < 3; ++k) {
          xipos[k] += xpos[k];
        }
        out->xipos[b] = xipos;
      }
      out->ximat[b] =
          body.inertial_frame == SameFrame::NONE
              ? convert_to_matrix(multiply(xquat, body.inertial_quat))
              : out->xmat[b];
    }
  }

  // mj_comPos.
  auto compute_com(Out<Dynamics> out) const -> void {
    using namespace articulated;
    const ArticulatedModel& m = *model_;
    std::array<Vector3, Capacity::bodies> subtree;
    subtree.fill(Vector3::Zero());
    for (std::uint32_t b = 0; b < tree_->bodies; ++b) {
      double mass = m.bodies[tree_->first_body + b].mass;
      for (int k = 0; k < 3; ++k) {
        subtree[b][k] = out->xipos[b][k] * mass;
      }
    }
    for (std::uint32_t b = tree_->bodies; b-- > 0;) {
      if (parent_[b] != NONE) {
        for (int k = 0; k < 3; ++k) {
          subtree[parent_[b]][k] += subtree[b][k];
        }
      }
    }
    if (subtree_mass_[0] < MINVAL) {
      out->com = out->xipos[0];
    } else {
      double inverse = 1.0 / subtree_mass_[0];
      for (int k = 0; k < 3; ++k) {
        out->com[k] = subtree[0][k] * inverse;
      }
    }
    for (std::uint32_t b = 0; b < tree_->bodies; ++b) {
      const ArticulatedBody& body = m.bodies[tree_->first_body + b];
      Vector3 offset{out->xipos[b][0] - out->com[0],
                     out->xipos[b][1] - out->com[1],
                     out->xipos[b][2] - out->com[2]};
      out->cinert[b] =
          shift_inertia(body.inertia, out->ximat[b], offset, body.mass);
    }
    for (std::uint32_t b = 0; b < tree_->bodies; ++b) {
      const ArticulatedBody& body = m.bodies[tree_->first_body + b];
      for (std::uint32_t jj = 0; jj < body.joints; ++jj) {
        const Joint& joint = m.joints[body.first_joint + jj];
        std::uint32_t j = body.first_joint + jj - tree_->first_joint;
        std::uint32_t da = joint.dof - tree_->first_dof;
        Vector3 offset{out->com[0] - out->xanchor[j][0],
                       out->com[1] - out->xanchor[j][1],
                       out->com[2] - out->xanchor[j][2]};
        std::uint32_t skip = 0;
        switch (joint.type) {
          case JointType::FREE:
            for (std::uint32_t k = 0; k < 3; ++k) {
              out->cdof[da + k] = {};
              out->cdof[da + k][3 + k] = 1;
            }
            skip = 3;
            [[fallthrough]];
          case JointType::BALL:
            for (std::uint32_t k = 0; k < 3; ++k) {
              const Matrix3& x = out->xmat[b];
              out->cdof[da + skip + k] =
                  turn_about({x[k], x[k + 3], x[k + 6]}, offset);
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
    using namespace articulated;
    const ArticulatedModel& m = *model_;
    std::array<Inertia10, Capacity::bodies> crb = out->cinert;
    for (std::uint32_t b = tree_->bodies; b-- > 0;) {
      if (parent_[b] != NONE) {
        for (int k = 0; k < 10; ++k) {
          crb[parent_[b]][k] += crb[b][k];
        }
      }
    }
    out->mass.fill(0.0);
    for (std::uint32_t i = 0; i < tree_->dofs; ++i) {
      out->mass[i * V + i] = m.dofs[tree_->first_dof + i].armature;
      Spatial buffer = multiply(crb[dof_body_[i]], out->cdof[i]);
      for (std::uint32_t j = i; j != NONE; j = dof_parent_[j]) {
        out->mass[i * V + j] += dot6(out->cdof[j], buffer);
      }
    }
  }

  // L'DL of `mass` along the tree, clamping pivots (mj_factorI).
  auto factor(const std::array<double, V * V>& mass,
              Out<std::array<double, V * V>> factor_out,
              Out<std::array<double, V>> inverse_out) const -> void {
    using namespace articulated;
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
    using namespace articulated;
    const ArticulatedModel& m = *model_;
    for (std::uint32_t b = 0; b < tree_->bodies; ++b) {
      const ArticulatedBody& body = m.bodies[tree_->first_body + b];
      Spatial cvel = parent_[b] != NONE ? out->cvel[parent_[b]] : Spatial{};
      std::uint32_t bda = body.first_dof - tree_->first_dof;
      auto add = [&](std::uint32_t first, std::size_t count) {
        Spatial t = combine(std::span{out->cdof}.subspan(first, count),
                            std::span{state.qvel}.subspan(first, count));
        for (int k = 0; k < 6; ++k) {
          cvel[k] += t[k];
        }
      };
      for (std::uint32_t j = 0; j < body.dofs; ++j) {
        const Dof& dof = m.dofs[body.first_dof + j];
        JointType type = m.joints[dof.joint].type;
        if (type == JointType::FREE) {
          for (std::uint32_t k = 0; k < 3; ++k) {
            out->cdof_dot[bda + j + k] = {};
          }
          add(bda + j, 3);
          j += 3;
          type = JointType::BALL;
        }
        if (type == JointType::BALL) {
          for (std::uint32_t k = 0; k < 3; ++k) {
            out->cdof_dot[bda + j + k] =
                cross_motion(cvel, out->cdof[bda + j + k]);
          }
          add(bda + j, 3);
          j += 2;
        } else {
          out->cdof_dot[bda + j] = cross_motion(cvel, out->cdof[bda + j]);
          add(bda + j, 1);
        }
      }
      out->cvel[b] = cvel;
    }
  }

  // mj_rne without accelerations: gravity and velocity products.
  auto compute_bias(const State& state, Out<Dynamics> out) const -> void {
    using namespace articulated;
    const ArticulatedModel& m = *model_;
    Spatial world{0.0,
                  0.0,
                  0.0,
                  -m.physics.gravity[0],
                  -m.physics.gravity[1],
                  -m.physics.gravity[2]};
    std::array<Spatial, Capacity::bodies> cacc{};
    std::array<Spatial, Capacity::bodies> force{};
    for (std::uint32_t b = 0; b < tree_->bodies; ++b) {
      const ArticulatedBody& body = m.bodies[tree_->first_body + b];
      std::uint32_t bda = body.first_dof - tree_->first_dof;
      Spatial t =
          body.dofs == 0
              ? Spatial{}
              : combine(std::span{out->cdof_dot}.subspan(bda, body.dofs),
                        std::span{state.qvel}.subspan(bda, body.dofs));
      const Spatial& before = parent_[b] != NONE ? cacc[parent_[b]] : world;
      for (int k = 0; k < 6; ++k) {
        cacc[b][k] = before[k] + t[k];
      }
      force[b] = multiply(out->cinert[b], cacc[b]);
      Spatial momentum = multiply(out->cinert[b], out->cvel[b]);
      Spatial c = cross_force(out->cvel[b], momentum);
      for (int k = 0; k < 6; ++k) {
        force[b][k] += c[k];
      }
    }
    for (std::uint32_t b = tree_->bodies; b-- > 0;) {
      if (parent_[b] != NONE) {
        for (int k = 0; k < 6; ++k) {
          force[parent_[b]][k] += force[b][k];
        }
      }
    }
    for (std::uint32_t i = 0; i < tree_->dofs; ++i) {
      out->bias[i] = dot6(out->cdof[i], force[dof_body_[i]]);
    }
  }

  // Joint springs and dof dampers (mj_springdamper).
  auto compute_passive(const State& state, Out<Dynamics> out) const -> void {
    using namespace articulated;
    const ArticulatedModel& m = *model_;
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
          if (skip > 0) {
            Vector3 dif{state.qpos[q] - m.qpos_spring[qs],
                        state.qpos[q + 1] - m.qpos_spring[qs + 1],
                        state.qpos[q + 2] - m.qpos_spring[qs + 2]};
            for (int k = 0; k < 3; ++k) {
              out->passive[v + k] += dif[k] * -joint.stiffness;
            }
          }
          Quaternion4 quat{state.qpos[q + skip], state.qpos[q + skip + 1],
                           state.qpos[q + skip + 2], state.qpos[q + skip + 3]};
          normalize4(InOut(quat));
          Quaternion4 spring{
              m.qpos_spring[qs + skip], -m.qpos_spring[qs + skip + 1],
              -m.qpos_spring[qs + skip + 2], -m.qpos_spring[qs + skip + 3]};
          Quaternion4 dq = multiply(spring, quat);
          Vector3 axis{dq[1], dq[2], dq[3]};
          double sin_half = normalize3(InOut(axis));
          double speed = 2 * std::atan2(sin_half, dq[0]);
          if (speed > std::numbers::pi) {
            speed -= 2 * std::numbers::pi;
          }
          for (int k = 0; k < 3; ++k) {
            out->passive[v + skip + k] += axis[k] * speed * -joint.stiffness;
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
    const ArticulatedModel& m = *model_;
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

  const ArticulatedModel* model_ = nullptr;
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

}  // namespace simon::model
