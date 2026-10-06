// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows MuJoCo 3.14.0, Copyright 2021 DeepMind Technologies Limited,
// Apache-2.0; translated to C++ and changed. See NOTICE.md.

#include "model/articulated_constraint.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <span>

namespace simon::model {

namespace {

constexpr double MINVAL = 1e-15;       // mjMINVAL.
constexpr double MINIMP = 0.0001;      // mjMINIMP.
constexpr double MAXIMP = 0.9999;      // mjMAXIMP.
constexpr double LS_TOLERANCE = 0.01;  // mjOption's ls_tolerance.
constexpr std::uint32_t LS_ITERATIONS = 50;

enum class RowState : std::uint8_t {
  SATISFIED,
  QUADRATIC,
  LINEAR_NEGATIVE,
  LINEAR_POSITIVE,
  CONE,
};

auto dot(std::span<const double> a, std::span<const double> b) -> double {
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    sum += a[i] * b[i];
  }
  return sum;
}

auto norm(std::span<const double> a) -> double { return std::sqrt(dot(a, a)); }

// y = A x, A rows by cols.
auto multiply(std::span<const double> a, std::uint32_t rows, std::uint32_t cols,
              std::span<const double> x, std::span<double> y) -> void {
  for (std::uint32_t r = 0; r < rows; ++r) {
    y[r] = dot(a.subspan(std::size_t{r} * cols, cols), x);
  }
}

// y = Aᵀ x, A rows by cols.
auto multiply_transposed(std::span<const double> a, std::uint32_t rows,
                         std::uint32_t cols, std::span<const double> x,
                         std::span<double> y) -> void {
  std::fill(y.begin(), y.end(), 0.0);
  for (std::uint32_t r = 0; r < rows; ++r) {
    if (x[r] == 0) {
      continue;
    }
    for (std::uint32_t c = 0; c < cols; ++c) {
      y[c] += a[std::size_t{r} * cols + c] * x[r];
    }
  }
}

// Cholesky in place, the lower triangle, pivots below `mindiag` raised to it
// (mju_cholFactor).
auto factor_cholesky(InOut<std::vector<double>> mat, std::uint32_t n,
                     double mindiag) -> void {
  std::vector<double>& m = *mat;
  for (std::uint32_t j = 0; j < n; ++j) {
    double pivot = m[j * n + j];
    for (std::uint32_t k = 0; k < j; ++k) {
      pivot -= m[j * n + k] * m[j * n + k];
    }
    pivot = std::max(pivot, mindiag);
    m[j * n + j] = std::sqrt(pivot);
    double inverse = 1 / m[j * n + j];
    for (std::uint32_t i = j + 1; i < n; ++i) {
      double sum = m[i * n + j];
      for (std::uint32_t k = 0; k < j; ++k) {
        sum -= m[i * n + k] * m[j * n + k];
      }
      m[i * n + j] = sum * inverse;
    }
  }
}

// x = (L Lᵀ)⁻¹ x (mju_cholSolve).
auto solve_cholesky(std::span<const double> l, std::uint32_t n,
                    std::span<double> x) -> void {
  for (std::uint32_t i = 0; i < n; ++i) {
    double sum = x[i];
    for (std::uint32_t k = 0; k < i; ++k) {
      sum -= l[i * n + k] * x[k];
    }
    x[i] = sum / l[i * n + i];
  }
  for (std::uint32_t i = n; i-- > 0;) {
    double sum = x[i];
    for (std::uint32_t k = i + 1; k < n; ++k) {
      sum -= l[k * n + i] * x[k];
    }
    x[i] = sum / l[i * n + i];
  }
}

auto power(double a, double b) -> double {
  if (b == 1) {
    return a;
  }
  if (b == 2) {
    return a * a;
  }
  return std::pow(a, b);
}

// solref and solimp as MuJoCo uses them: solref no shorter than two steps,
// solimp within its bounds (getsolparam).
auto clamp_soft(SoftConstraint soft, double timestep) -> SoftConstraint {
  std::array<double, 2>& ref = soft.reference;
  if ((ref[0] > 0) != (ref[1] > 0)) {
    ref = SoftConstraint{}.reference;
  }
  if (ref[0] > 0) {
    ref[0] = std::max(ref[0], 2 * timestep);
  }
  std::array<double, 5>& imp = soft.impedance;
  imp[0] = std::min(MAXIMP, std::max(MINIMP, imp[0]));
  imp[1] = std::min(MAXIMP, std::max(MINIMP, imp[1]));
  imp[2] = std::max(0.0, imp[2]);
  imp[3] = std::min(MAXIMP, std::max(MINIMP, imp[3]));
  imp[4] = std::max(1.0, imp[4]);
  return soft;
}

// The impedance at `pos` past `margin`, from d0 to dwidth over the width,
// by two power curves meeting at the midpoint (getimpedance).
auto compute_impedance(const std::array<double, 5>& solimp, double pos,
                       double margin) -> double {
  if (solimp[0] == solimp[1] || solimp[2] <= MINVAL) {
    return 0.5 * (solimp[0] + solimp[1]);
  }
  double x = std::abs((pos - margin) / solimp[2]);
  if (x >= 1 || x <= 0) {
    return x >= 1 ? solimp[1] : solimp[0];
  }
  double y = 0.0;
  if (solimp[4] == 1) {
    y = x;
  } else if (x <= solimp[3]) {
    double a = 1 / power(solimp[3], solimp[4] - 1);
    y = a * power(x, solimp[4]);
  } else {
    double b = 1 / power(1 - solimp[3], solimp[4] - 1);
    y = 1 - b * power(1 - x, solimp[4]);
  }
  return solimp[0] + y * (solimp[1] - solimp[0]);
}

// Each row's regularization R and its inverse D, and its reference
// acceleration -B v - K I (pos - margin) (mj_makeImpedance,
// mj_referenceConstraint).
struct Regularized final {
  std::vector<double> r;
  std::vector<double> d;
  std::vector<double> aref;
  std::vector<double> mu;  // An elliptic cone's, at its first row.
};

// The row past the last of the group starting at `first`.
auto group_end(const ConstraintProblem& p, std::uint32_t first)
    -> std::uint32_t {
  std::uint32_t end = first + 1;
  while (end < p.rows() && p.group[end] == first) {
    ++end;
  }
  return end;
}

auto regularize(const ConstraintProblem& p, double timestep, double impratio)
    -> Regularized {
  std::uint32_t rows = p.rows();
  Regularized out{std::vector<double>(rows), std::vector<double>(rows),
                  std::vector<double>(rows), std::vector<double>(rows)};
  for (std::uint32_t i = 0; i < rows; ++i) {
    std::uint32_t first = p.group[i];
    SoftConstraint soft = clamp_soft(p.soft[first], timestep);
    const std::array<double, 2>& ref = soft.reference;
    const std::array<double, 5>& solimp = soft.impedance;
    double imp = compute_impedance(solimp, p.pos[first], p.margin[first]);
    out.r[i] = std::max(MINVAL, (1 - imp) * p.diagonal[i] / imp);
    double k = 0.0;
    bool sliding = p.kind[i] == ConstraintKind::FRICTION ||
                   (p.kind[i] == ConstraintKind::ELLIPTIC && i != first);
    if (sliding) {
      k = 0.0;
    } else if (ref[0] > 0) {
      k = 1 / std::max(MINVAL, solimp[1] * solimp[1] * ref[0] * ref[0] *
                                   ref[1] * ref[1]);
    } else {
      k = -ref[0] / std::max(MINVAL, solimp[1] * solimp[1]);
    }
    double b = ref[1] > 0 ? 2 / std::max(MINVAL, solimp[1] * ref[0])
                          : -ref[1] / std::max(MINVAL, solimp[1]);
    out.aref[i] = -b * p.velocity[i] - k * imp * (p.pos[i] - p.margin[i]);
  }
  // A contact's friction directions: the first R over impratio, its cone's
  // friction from the ratio; an elliptic cone's others so that each R mu²
  // matches, a pyramid's rows one R matching the elliptic cone's friction.
  for (std::uint32_t i = 0; i < rows; ++i) {
    bool pyramid = p.kind[i] == ConstraintKind::PYRAMIDAL;
    if ((!pyramid && p.kind[i] != ConstraintKind::ELLIPTIC) ||
        p.group[i] != i) {
      continue;
    }
    const std::array<double, 5>& f = p.friction[i];
    std::uint32_t end = group_end(p, i);
    double r1 = out.r[i] / std::max(MINVAL, impratio);
    double mu = f[0] * std::sqrt(r1 / out.r[i]);
    out.mu[i] = mu;
    if (pyramid) {
      double shared = 2 * mu * mu * out.r[i];
      for (std::uint32_t j = i; j < end; ++j) {
        out.r[j] = shared;
      }
    } else {
      out.r[i + 1] = r1;
      for (std::uint32_t j = 1; j + 1 < end - i; ++j) {
        out.r[i + j + 1] = out.r[i + 1] * f[0] * f[0] / (f[j] * f[j]);
      }
    }
  }
  for (std::uint32_t i = 0; i < rows; ++i) {
    out.d[i] = 1 / out.r[i];
  }
  return out;
}

// Each row's force and state at jar = J qacc - aref, and their cost
// (mj_constraintUpdate_impl).
auto update_rows(const ConstraintProblem& p, const Regularized& reg,
                 std::span<const double> jar, std::span<double> force,
                 std::span<RowState> state, std::span<double> cone = {})
    -> double {
  double cost = 0.0;
  for (std::uint32_t i = 0; i < p.rows(); ++i) {
    force[i] = -reg.d[i] * jar[i];
  }
  for (std::uint32_t i = 0; i < p.rows(); ++i) {
    double d = reg.d[i];
    if (p.kind[i] == ConstraintKind::ELLIPTIC) {
      // In the cone's own space, its friction made circular: above the
      // cone no force, below it a quadratic, between them a quadratic in
      // the distance to its surface.
      std::uint32_t end = group_end(p, i);
      std::uint32_t dim = end - i;
      double mu = reg.mu[i];
      const std::array<double, 5>& friction = p.friction[i];
      std::array<double, 6> u{};
      u[0] = jar[i] * mu;
      double t2 = 0.0;
      for (std::uint32_t j = 1; j < dim; ++j) {
        u[j] = jar[i + j] * friction[j - 1];
        t2 += u[j] * u[j];
      }
      double n = u[0];
      double t = std::sqrt(t2);
      RowState zone = RowState::QUADRATIC;
      if (n >= mu * t || (t <= 0 && n >= 0)) {
        for (std::uint32_t j = i; j < end; ++j) {
          force[j] = 0;
        }
        zone = RowState::SATISFIED;
      } else if (mu * n + t <= 0 || (t <= 0 && n < 0)) {
        for (std::uint32_t j = i; j < end; ++j) {
          cost += 0.5 * reg.d[j] * jar[j] * jar[j];
        }
      } else {
        double dm = d / (mu * mu * (1 + mu * mu));
        double nmt = n - mu * t;
        cost += 0.5 * dm * nmt * nmt;
        force[i] = -dm * nmt * mu;
        for (std::uint32_t j = 1; j < dim; ++j) {
          force[i + j] = -force[i] / t * u[j] * friction[j - 1];
        }
        zone = RowState::CONE;
        if (!cone.empty()) {
          // The cone's Hessian in the rows' space (mj_constraintUpdate).
          double* h = &cone[std::size_t{i} * 36];
          std::fill(h, h + dim * dim, 0.0);
          h[0] = 1;
          for (std::uint32_t j = 1; j < dim; ++j) {
            h[j] = -mu / t * u[j];
          }
          double outer = mu * n / (t * t * t);
          for (std::uint32_t k = 1; k < dim; ++k) {
            for (std::uint32_t j = k; j < dim; ++j) {
              h[k * dim + j] = outer * u[j] * u[k];
            }
          }
          double diagonal = mu * mu - mu * n / t;
          for (std::uint32_t j = 1; j < dim; ++j) {
            h[j * (dim + 1)] += diagonal;
          }
          for (std::uint32_t k = 0; k < dim; ++k) {
            double sk = dm * (k == 0 ? mu : friction[k - 1]);
            for (std::uint32_t j = k; j < dim; ++j) {
              h[k * dim + j] *= sk * (j == 0 ? mu : friction[j - 1]);
            }
          }
          for (std::uint32_t k = 0; k < dim; ++k) {
            for (std::uint32_t j = k + 1; j < dim; ++j) {
              h[j * dim + k] = h[k * dim + j];
            }
          }
        }
      }
      for (std::uint32_t j = i; j < end; ++j) {
        state[j] = zone;
      }
      i = end - 1;
      continue;
    }
    if (p.kind[i] == ConstraintKind::FRICTION) {
      double floss = p.friction_loss[i];
      double r = reg.r[i];
      if (jar[i] <= -r * floss) {
        cost += -0.5 * r * floss * floss - floss * jar[i];
        force[i] = floss;
        state[i] = RowState::LINEAR_NEGATIVE;
      } else if (jar[i] >= r * floss) {
        cost += -0.5 * r * floss * floss + floss * jar[i];
        force[i] = -floss;
        state[i] = RowState::LINEAR_POSITIVE;
      } else {
        cost += 0.5 * d * jar[i] * jar[i];
        state[i] = RowState::QUADRATIC;
      }
    } else if (jar[i] >= 0) {
      force[i] = 0;
      state[i] = RowState::SATISFIED;
    } else {
      cost += 0.5 * d * jar[i] * jar[i];
      state[i] = RowState::QUADRATIC;
    }
  }
  return cost;
}

//-- Newton (engine_solver.c, mj_solPrimal) -----------------------------------

// The vectors a Newton solve works in, kept from island to island.
struct NewtonBuffers final {
  std::vector<double> qacc;
  std::vector<double> ma;
  std::vector<double> jaref;
  std::vector<double> force;
  std::vector<RowState> state;
  std::vector<double> qfrc;
  std::vector<double> grad;
  std::vector<double> mgrad;
  std::vector<double> search;
  std::vector<double> mv;
  std::vector<double> jv;
  std::vector<double> quad;
  std::vector<double> hessian;
  std::vector<double> mass_factor;
  std::vector<double> jar;
  std::vector<double> da;
  std::vector<double> mda;
  std::vector<std::uint32_t> columns;  // Each row's nonzero columns.
  std::vector<std::uint32_t> starts;   // By row, into columns; one more.
  std::vector<double> cone;            // By row, an elliptic cone's Hessian.
  std::vector<double> cone_quad;       // By row, its line search's terms.
  std::vector<double> cone_rows;       // Its Hessian times its rows.
};

template <typename T>
auto sized(std::vector<T>& v, std::size_t n) -> std::vector<T>& {
  v.resize(n);
  return v;
}

class Newton final {
 public:
  Newton(const ConstraintProblem& p, const Regularized& reg,
         const ConstraintSettings& settings, InOut<NewtonBuffers> buffers)
      : p_{&p},
        reg_{&reg},
        settings_{&settings},
        n_{p.dofs},
        m_{p.rows()},
        qacc_{sized(buffers->qacc, n_)},
        ma_{sized(buffers->ma, n_)},
        jaref_{sized(buffers->jaref, m_)},
        force_{sized(buffers->force, m_)},
        state_{sized(buffers->state, m_)},
        qfrc_{sized(buffers->qfrc, n_)},
        grad_{sized(buffers->grad, n_)},
        mgrad_{sized(buffers->mgrad, n_)},
        search_{sized(buffers->search, n_)},
        mv_{sized(buffers->mv, n_)},
        jv_{sized(buffers->jv, m_)},
        quad_{sized(buffers->quad, 3 * std::size_t{m_})},
        hessian_{buffers->hessian},
        mass_factor_{buffers->mass_factor},
        jar_{sized(buffers->jar, m_)},
        da_{sized(buffers->da, n_)},
        mda_{sized(buffers->mda, n_)},
        columns_{buffers->columns},
        starts_{sized(buffers->starts, std::size_t{m_} + 1)},
        cone_{buffers->cone},
        cone_quad_{sized(buffers->cone_quad, 6 * std::size_t{m_})},
        cone_rows_{buffers->cone_rows} {
    elliptic_ = std::ranges::contains(p.kind, ConstraintKind::ELLIPTIC);
    cone_.resize(elliptic_ ? 36 * std::size_t{m_} : 0);
    columns_.clear();
    for (std::uint32_t r = 0; r < m_; ++r) {
      starts_[r] = static_cast<std::uint32_t>(columns_.size());
      for (std::uint32_t c = 0; c < n_; ++c) {
        if (p.jacobian[std::size_t{r} * n_ + c] != 0) {
          columns_.push_back(c);
        }
      }
    }
    starts_[m_] = static_cast<std::uint32_t>(columns_.size());
  }

  auto solve(Out<ConstraintSolution> solution) -> void {
    const ConstraintProblem& p = *p_;
    double tolerance = settings_->tolerance;
    choose_start();
    multiply(p.mass, n_, n_, qacc_, ma_);
    multiply_jacobian(qacc_, jaref_);
    for (std::uint32_t i = 0; i < m_; ++i) {
      jaref_[i] -= reg_->aref[i];
    }
    update();
    update_grad();
    double inertia = 0.0;
    for (std::uint32_t i = 0; i < n_; ++i) {
      inertia += p.mass[i * n_ + i];
    }
    scale_ = 1 / inertia;

    // Done already if the gradient is small and M⁻¹ grad, which bounds the
    // cost's suboptimality, is too; M is factored only to find out.
    bool gradient = scale_ * norm(grad_) < tolerance;
    bool done = false;
    if (gradient) {
      mass_factor_ = p.mass;
      factor_cholesky(InOut(mass_factor_), n_, MINVAL);
      mgrad_ = grad_;
      solve_cholesky(mass_factor_, n_, mgrad_);
      done = std::max(0.0, 0.5 * scale_ * dot(grad_, mgrad_)) < tolerance;
    }
    if (!done) {
      factor_hessian();
      mgrad_ = grad_;
      solve_cholesky(hessian_, n_, mgrad_);
      done = gradient &&
             std::max(0.0, 0.5 * scale_ * dot(grad_, mgrad_)) < tolerance;
    }
    if (!done) {
      for (std::uint32_t i = 0; i < n_; ++i) {
        search_[i] = -mgrad_[i];
      }
    }
    std::uint32_t iterations = 0;
    while (!done && iterations < settings_->iterations) {
      double improvement = 0.0;
      double alpha = search(tolerance * LS_TOLERANCE, InOut(improvement));
      if (alpha == 0) {
        break;
      }
      for (std::uint32_t i = 0; i < n_; ++i) {
        qacc_[i] += search_[i] * alpha;
        ma_[i] += mv_[i] * alpha;
      }
      for (std::uint32_t i = 0; i < m_; ++i) {
        jaref_[i] += jv_[i] * alpha;
      }
      update();
      factor_hessian();
      update_grad();
      mgrad_ = grad_;
      solve_cholesky(hessian_, n_, mgrad_);
      double scaled = scale_ * improvement;
      double slope = scale_ * norm(grad_);
      double decrement = std::max(0.0, 0.5 * scale_ * dot(grad_, mgrad_));
      ++iterations;
      if ((scaled > 0 && scaled < tolerance) || slope < tolerance ||
          decrement < tolerance) {
        break;
      }
      for (std::uint32_t i = 0; i < n_; ++i) {
        search_[i] = -mgrad_[i];
      }
    }
    solution->qacc = qacc_;
    solution->qfrc_constraint = qfrc_;
    solution->force = force_;
    solution->iterations = iterations;
  }

 private:
  struct Point final {
    double alpha = 0.0;
    double cost = 0.0;
    std::array<double, 2> deriv{};
  };

  // The last step's accelerations, unless the smooth ones cost less
  // (warmstart).
  auto choose_start() -> void {
    const ConstraintProblem& p = *p_;
    auto cost_at = [&](std::span<const double> qacc) {
      multiply_jacobian(qacc, jar_);
      for (std::uint32_t i = 0; i < m_; ++i) {
        jar_[i] -= reg_->aref[i];
      }
      return update_rows(p, *reg_, jar_, force_, state_);
    };
    double warm = cost_at(p.warmstart);
    for (std::uint32_t i = 0; i < n_; ++i) {
      da_[i] = p.warmstart[i] - p.qacc_smooth[i];
    }
    multiply(p.mass, n_, n_, da_, mda_);
    warm += 0.5 * dot(da_, mda_);
    double smooth = cost_at(p.qacc_smooth);
    qacc_ = warm > smooth ? p.qacc_smooth : p.warmstart;
  }

  // y = J x, over each row's nonzero columns.
  auto multiply_jacobian(std::span<const double> x, std::span<double> y) const
      -> void {
    const std::vector<double>& j = p_->jacobian;
    for (std::uint32_t r = 0; r < m_; ++r) {
      double sum = 0.0;
      for (std::uint32_t k = starts_[r]; k < starts_[r + 1]; ++k) {
        sum += j[std::size_t{r} * n_ + columns_[k]] * x[columns_[k]];
      }
      y[r] = sum;
    }
  }

  // y = Jᵀ f.
  auto multiply_jacobian_transposed(std::span<const double> f,
                                    std::span<double> y) const -> void {
    const std::vector<double>& j = p_->jacobian;
    std::fill(y.begin(), y.end(), 0.0);
    for (std::uint32_t r = 0; r < m_; ++r) {
      if (f[r] == 0) {
        continue;
      }
      for (std::uint32_t k = starts_[r]; k < starts_[r + 1]; ++k) {
        y[columns_[k]] += j[std::size_t{r} * n_ + columns_[k]] * f[r];
      }
    }
  }

  // Forces, states and cost at the current accelerations, Gauss's term
  // included (PrimalUpdateConstraint).
  auto update() -> void {
    const ConstraintProblem& p = *p_;
    cost_ = update_rows(p, *reg_, jaref_, force_, state_, cone_);
    multiply_jacobian_transposed(force_, qfrc_);
    double gauss = 0.0;
    for (std::uint32_t i = 0; i < n_; ++i) {
      gauss +=
          0.5 * (ma_[i] - p.qfrc_smooth[i]) * (qacc_[i] - p.qacc_smooth[i]);
    }
    quad_gauss_[0] = gauss;
    cost_ += gauss;
  }

  auto update_grad() -> void {
    for (std::uint32_t i = 0; i < n_; ++i) {
      grad_[i] = ma_[i] - p_->qfrc_smooth[i] - qfrc_[i];
    }
  }

  // H = M + Jᵀ D J over the quadratic rows, and Jᵀ H J over each elliptic
  // cone between its zones, factored (MakeHessian, HessianCone,
  // FactorizeHessian).
  auto factor_hessian() -> void {
    const ConstraintProblem& p = *p_;
    hessian_ = p.mass;
    for (std::uint32_t i = 0; elliptic_ && i < m_; ++i) {
      if (p.kind[i] != ConstraintKind::ELLIPTIC || p.group[i] != i) {
        continue;
      }
      std::uint32_t dim = group_end(p, i) - i;
      if (state_[i] == RowState::CONE) {
        const double* h = &cone_[std::size_t{i} * 36];
        const double* j = &p.jacobian[std::size_t{i} * n_];
        cone_rows_.assign(std::size_t{dim} * n_, 0.0);
        for (std::uint32_t a = 0; a < dim; ++a) {
          for (std::uint32_t b = 0; b < dim; ++b) {
            for (std::uint32_t c = 0; c < n_; ++c) {
              cone_rows_[a * n_ + c] += h[a * dim + b] * j[b * n_ + c];
            }
          }
        }
        for (std::uint32_t x = 0; x < n_; ++x) {
          for (std::uint32_t y = 0; y <= x; ++y) {
            double sum = 0.0;
            for (std::uint32_t a = 0; a < dim; ++a) {
              sum += j[a * n_ + x] * cone_rows_[a * n_ + y];
            }
            hessian_[x * n_ + y] += sum;
          }
        }
      }
      i += dim - 1;
    }
    for (std::uint32_t r = 0; r < m_; ++r) {
      if (state_[r] != RowState::QUADRATIC) {
        continue;
      }
      double d = reg_->d[r];
      const double* j = &p.jacobian[std::size_t{r} * n_];
      for (std::uint32_t ka = starts_[r]; ka < starts_[r + 1]; ++ka) {
        std::uint32_t a = columns_[ka];
        for (std::uint32_t kb = starts_[r]; kb <= ka; ++kb) {
          std::uint32_t b = columns_[kb];
          hessian_[a * n_ + b] += j[a] * d * j[b];
        }
      }
    }
    factor_cholesky(InOut(hessian_), n_, MINVAL);
  }

  // The cost's quadratic pieces along the search direction (PrimalPrepare).
  auto prepare() -> void {
    const ConstraintProblem& p = *p_;
    quad_gauss_[1] = dot(search_, ma_) - dot(p.qfrc_smooth, search_);
    quad_gauss_[2] = 0.5 * dot(search_, mv_);
    for (std::uint32_t i = 0; i < m_; ++i) {
      double d = reg_->d[i];
      double dj0 = d * jaref_[i];
      double q0 = jaref_[i] * dj0;
      double q1 = jv_[i] * dj0;
      double q2 = jv_[i] * d * jv_[i];
      if (p.kind[i] == ConstraintKind::ELLIPTIC) {
        // The whole cone's quadratic, for below it, and its terms in the
        // cone's circular space, for between its zones.
        std::uint32_t dim = group_end(p, i) - i;
        double mu = reg_->mu[i];
        const std::array<double, 5>& friction = p.friction[i];
        double uu = 0.0;
        double uv = 0.0;
        double vv = 0.0;
        for (std::uint32_t j = 1; j < dim; ++j) {
          double dj = reg_->d[i + j] * jaref_[i + j];
          q0 += jaref_[i + j] * dj;
          q1 += jv_[i + j] * dj;
          q2 += jv_[i + j] * reg_->d[i + j] * jv_[i + j];
          double u = jaref_[i + j] * friction[j - 1];
          double v = jv_[i + j] * friction[j - 1];
          uu += u * u;
          uv += u * v;
          vv += v * v;
        }
        double* c = &cone_quad_[6 * std::size_t{i}];
        c[0] = jaref_[i] * mu;
        c[1] = jv_[i] * mu;
        c[2] = uu;
        c[3] = uv;
        c[4] = vv;
        c[5] = d / ((mu * mu) * (1 + (mu * mu)));
      }
      quad_[3 * i] = 0.5 * q0;
      quad_[3 * i + 1] = q1;
      quad_[3 * i + 2] = 0.5 * q2;
      if (p.kind[i] == ConstraintKind::ELLIPTIC) {
        i = group_end(p, i) - 1;
      }
    }
  }

  static auto friction_cost(double x, double f, double rf, double d) -> double {
    if (-rf < x && x < rf) {
      return 0.5 * d * x * x;
    }
    if (x <= -rf) {
      return f * (-0.5 * rf - x);
    }
    return f * (-0.5 * rf + x);
  }

  static auto friction_cost_change(double start, double x, double f, double rf,
                                   double d) -> double {
    auto zone = [&](double v) {
      return (-rf < v && v < rf) ? 0 : (v <= -rf ? -1 : 1);
    };
    int a = zone(start);
    int b = zone(x);
    if (a == 0 && b == 0) {
      return 0.5 * d * (x - start) * (x + start);
    }
    if (a == -1 && b == -1) {
      return f * (start - x);
    }
    if (a == 1 && b == 1) {
      return f * (x - start);
    }
    return friction_cost(x, f, rf, d) - friction_cost(start, f, rf, d);
  }

  // An elliptic cone's cost at `alpha` less its cost at 0, by the zones it
  // is in at each (ellipticCostDif).
  auto cone_cost_change(std::uint32_t i, double alpha) const -> double {
    const double* q = &quad_[3 * std::size_t{i}];
    const double* c = &cone_quad_[6 * std::size_t{i}];
    double mu = reg_->mu[i];
    double u0 = c[0];
    double v0 = c[1];
    double uu = c[2];
    double uv = c[3];
    double vv = c[4];
    double dm = c[5];
    auto zone_of = [&](double n, double tsqr, double& t) {
      if (tsqr <= 0) {
        t = 0;
        return n < 0 ? 2 : 1;
      }
      t = std::sqrt(tsqr);
      return n >= mu * t ? 1 : (mu * n + t <= 0 ? 2 : 3);
    };
    double t0 = 0.0;
    int zone0 = zone_of(u0, uu, t0);
    double n = u0 + alpha * v0;
    double t = 0.0;
    int zone = zone_of(n, uu + alpha * (2 * uv + alpha * vv), t);
    if (zone0 == 1 && zone == 1) {
      return 0;
    }
    if (zone0 == 2 && zone == 2) {
      return alpha * alpha * q[2] + alpha * q[1];
    }
    if (zone0 == 3 && zone == 3) {
      double tsqr_delta = alpha * (2 * uv + alpha * vv);
      double t_delta = tsqr_delta / (t + t0);
      double r_delta = alpha * v0 - mu * t_delta;
      double r0 = u0 - mu * t0;
      return 0.5 * dm * r_delta * (2 * r0 + r_delta);
    }
    if (zone0 == 3 && zone == 2) {
      double boundary0 = mu * u0 + t0;
      return alpha * (alpha * q[2] + q[1]) + 0.5 * dm * boundary0 * boundary0;
    }
    if (zone0 == 2 && zone == 3) {
      double boundary = mu * n + t;
      return alpha * (alpha * q[2] + q[1]) - 0.5 * dm * boundary * boundary;
    }
    if (zone0 == 1 && zone == 2) {
      return alpha * alpha * q[2] + alpha * q[1] + q[0];
    }
    if (zone0 == 1 && zone == 3) {
      double r = n - mu * t;
      return 0.5 * dm * r * r;
    }
    if (zone0 == 3 && zone == 1) {
      double r0 = u0 - mu * t0;
      return -0.5 * dm * r0 * r0;
    }
    if (zone0 == 2 && zone == 1) {
      return -q[0];
    }
    return 0;
  }

  // The cost at `alpha` less the cost at 0, and its first two derivatives
  // (PrimalEval).
  auto evaluate(InOut<Point> point) -> void {
    const ConstraintProblem& p = *p_;
    double alpha = point->alpha;
    double cost = 0.0;
    std::array<double, 2> deriv{};
    std::array<double, 3> total{0.0, quad_gauss_[1], quad_gauss_[2]};
    for (std::uint32_t i = 0; i < m_; ++i) {
      if (p.kind[i] == ConstraintKind::ELLIPTIC) {
        const double* q = &quad_[3 * std::size_t{i}];
        const double* c = &cone_quad_[6 * std::size_t{i}];
        double mu = reg_->mu[i];
        cost += cone_cost_change(i, alpha);
        double n = c[0] + alpha * c[1];
        double tsqr = c[2] + alpha * (2 * c[3] + alpha * c[4]);
        if (tsqr <= 0) {
          if (n < 0) {
            deriv[0] += 2 * alpha * q[2] + q[1];
            deriv[1] += 2 * q[2];
          }
        } else {
          double t = std::sqrt(tsqr);
          if (n >= mu * t) {
          } else if (mu * n + t <= 0) {
            deriv[0] += 2 * alpha * q[2] + q[1];
            deriv[1] += 2 * q[2];
          } else {
            double n1 = c[1];
            double t1 = (c[3] + alpha * c[4]) / t;
            double t2 = c[4] / t - (c[3] + alpha * c[4]) * t1 / (t * t);
            deriv[0] += c[5] * (n - mu * t) * (n1 - mu * t1);
            deriv[1] += c[5] * ((n1 - mu * t1) * (n1 - mu * t1) +
                                (n - mu * t) * (-mu * t2));
          }
        }
        i = group_end(p, i) - 1;
        continue;
      }
      double start = jaref_[i];
      double dir = jv_[i];
      double x = start + alpha * dir;
      if (p.kind[i] == ConstraintKind::FRICTION) {
        double f = p.friction_loss[i];
        double d = reg_->d[i];
        double rf = reg_->r[i] * f;
        cost += friction_cost_change(start, x, f, rf, d);
        if (-rf < x && x < rf) {
          deriv[0] += d * x * dir;
          deriv[1] += d * dir * dir;
        } else if (x <= -rf) {
          deriv[0] += -f * dir;
        } else {
          deriv[0] += f * dir;
        }
        continue;
      }
      double cost0 = start < 0 ? quad_[3 * i] : 0;
      if (x < 0) {
        total[0] += quad_[3 * i] - cost0;
        total[1] += quad_[3 * i + 1];
        total[2] += quad_[3 * i + 2];
      } else {
        cost -= cost0;
      }
    }
    cost += alpha * alpha * total[2] + alpha * total[1] + total[0];
    deriv[0] += 2 * alpha * total[2] + total[1];
    deriv[1] += 2 * total[2];
    if (deriv[1] <= 0) {
      deriv[1] = MINVAL;
    }
    point->cost = cost;
    point->deriv = deriv;
    ++line_iterations_;
  }

  // Moves `p` to whichever candidate brackets the minimum more tightly on
  // its side, then its Newton point (updateBracket).
  auto update_bracket(InOut<Point> p, const std::array<Point, 3>& candidates,
                      InOut<Point> next) -> bool {
    bool moved = false;
    for (const Point& c : candidates) {
      if ((p->deriv[0] < 0 && c.deriv[0] < 0 && p->deriv[0] < c.deriv[0]) ||
          (p->deriv[0] > 0 && c.deriv[0] > 0 && p->deriv[0] > c.deriv[0])) {
        *p = c;
        moved = true;
      }
    }
    if (moved) {
      next->alpha = p->alpha - p->deriv[0] / p->deriv[1];
      evaluate(next);
    }
    return moved;
  }

  // The step along the search direction minimizing the cost, by Newton
  // steps on the piecewise quadratic, bracketing if they overshoot
  // (PrimalSearch).
  auto search(double tolerance, InOut<double> improvement) -> double {
    const ConstraintProblem& p = *p_;
    line_iterations_ = 0;
    *improvement = 0;
    double snorm = norm(search_);
    if (snorm < MINVAL) {
      return 0;
    }
    double gtol = tolerance * snorm / scale_;
    multiply(p.mass, n_, n_, search_, mv_);
    multiply_jacobian(search_, jv_);
    prepare();

    Point p0;
    evaluate(InOut(p0));
    Point p1{.alpha = p0.alpha - p0.deriv[0] / p0.deriv[1]};
    evaluate(InOut(p1));
    if (std::abs(p1.deriv[0]) < gtol && (p1.alpha == 0 || p1.cost < 0)) {
      *improvement = -p1.cost;
      return p1.alpha;
    }
    double dir = p1.deriv[0] < 0 ? 1 : -1;
    Point p2 = p0;
    while (p1.deriv[0] * dir <= -gtol && line_iterations_ < LS_ITERATIONS) {
      p2 = p1;
      p1.alpha -= p1.deriv[0] / p1.deriv[1];
      evaluate(InOut(p1));
      if (std::abs(p1.deriv[0]) < gtol && p1.cost < 0) {
        *improvement = -p1.cost;
        return p1.alpha;
      }
    }
    if (line_iterations_ >= LS_ITERATIONS) {
      *improvement = -p1.cost;
      return p1.alpha;
    }
    Point p2next = p1;
    Point p1next{.alpha = p1.alpha - p1.deriv[0] / p1.deriv[1]};
    evaluate(InOut(p1next));
    while (line_iterations_ < LS_ITERATIONS) {
      Point mid{.alpha = 0.5 * (p1.alpha + p2.alpha)};
      evaluate(InOut(mid));
      std::array<Point, 3> candidates{p1next, p2next, mid};
      int best = -1;
      double best_cost = 0.0;
      for (int i = 0; i < 3; ++i) {
        if (std::abs(candidates[i].deriv[0]) < gtol &&
            (best == -1 || candidates[i].cost < best_cost)) {
          best_cost = candidates[i].cost;
          best = i;
        }
      }
      if (best >= 0) {
        *improvement = -candidates[best].cost;
        return candidates[best].alpha;
      }
      bool b1 = update_bracket(InOut(p1), candidates, InOut(p1next));
      bool b2 = update_bracket(InOut(p2), candidates, InOut(p2next));
      if (!b1 && !b2) {
        *improvement = -mid.cost;
        return mid.alpha;
      }
    }
    if (p1.cost <= p2.cost && p1.cost < 0) {
      *improvement = -p1.cost;
      return p1.alpha;
    }
    if (p2.cost <= p1.cost && p2.cost < 0) {
      *improvement = -p2.cost;
      return p2.alpha;
    }
    return 0;
  }

  const ConstraintProblem* p_ = nullptr;
  const Regularized* reg_ = nullptr;
  const ConstraintSettings* settings_ = nullptr;
  std::uint32_t n_ = 0;
  std::uint32_t m_ = 0;
  std::vector<double>& qacc_;
  std::vector<double>& ma_;
  std::vector<double>& jaref_;
  std::vector<double>& force_;
  std::vector<RowState>& state_;
  std::vector<double>& qfrc_;
  std::vector<double>& grad_;
  std::vector<double>& mgrad_;
  std::vector<double>& search_;
  std::vector<double>& mv_;
  std::vector<double>& jv_;
  std::vector<double>& quad_;
  std::vector<double>& hessian_;
  std::vector<double>& mass_factor_;
  std::vector<double>& jar_;
  std::vector<double>& da_;
  std::vector<double>& mda_;
  std::vector<std::uint32_t>& columns_;
  std::vector<std::uint32_t>& starts_;
  std::vector<double>& cone_;
  std::vector<double>& cone_quad_;
  std::vector<double>& cone_rows_;
  bool elliptic_ = false;
  std::array<double, 3> quad_gauss_{};
  double cost_ = 0.0;
  double scale_ = 1.0;
  std::uint32_t line_iterations_ = 0;
};

//-- Projected Gauss–Seidel (engine_solver.c, solPGS) -------------------------

// MuJoCo's permuted congruential generator for the sweep order.
struct Pcg32 final {
  std::uint64_t state = 0;
  std::uint64_t increment = 1;

  auto next() -> std::uint32_t {
    std::uint64_t old = state;
    state = old * 6364136223846793005ULL + (increment | 1);
    auto shifted = static_cast<std::uint32_t>(((old >> 18U) ^ old) >> 27U);
    auto rotation = static_cast<std::uint32_t>(old >> 59U);
    return (shifted >> rotation) | (shifted << ((-rotation) & 31U));
  }
};

// Scales `friction` onto the ellipsoid sum (f_j / mu_j)² = normal², or
// only if outside it when `feasible` (projectEllipsoid).
auto project_ellipsoid(std::span<double> friction, double normal,
                       std::span<const double> mu, bool feasible) -> void {
  double s = 0.0;
  for (std::size_t j = 0; j < friction.size(); ++j) {
    s += friction[j] * friction[j] / (mu[j] * mu[j]);
  }
  double normal2 = normal * normal;
  if (!feasible || s > normal2) {
    double scl = std::sqrt(normal2 / std::max(MINVAL, s));
    for (double& f : friction) {
      f *= scl;
    }
  }
}

// min ½ xᵀ A x + xᵀ b subject to sum (x_j / d_j)² <= r², by Newton's method
// on the multiplier; whether the constraint holds it (mju_QCQP).
auto solve_qcqp(std::span<double> x, std::span<const double> a_in,
                std::span<const double> b_in, std::span<const double> d,
                double r) -> bool {
  auto n = static_cast<std::uint32_t>(x.size());
  std::array<double, 25> a{};
  std::array<double, 5> b{};
  for (std::uint32_t i = 0; i < n; ++i) {
    b[i] = b_in[i] * d[i];
    for (std::uint32_t j = 0; j < n; ++j) {
      a[j + i * n] = a_in[j + i * n] * d[i] * d[j];
    }
  }
  double la = 0.0;
  for (int iteration = 0; iteration < 20; ++iteration) {
    std::vector<double> ala(a.begin(), a.begin() + n * n);
    for (std::uint32_t i = 0; i < n; ++i) {
      ala[i * (n + 1)] += la;
    }
    bool full = true;
    for (std::uint32_t j = 0; j < n && full; ++j) {
      double pivot = ala[j * n + j];
      for (std::uint32_t k = 0; k < j; ++k) {
        pivot -= ala[j * n + k] * ala[j * n + k];
      }
      full = pivot >= 1e-10;
      if (!full) {
        break;
      }
      ala[j * n + j] = std::sqrt(pivot);
      for (std::uint32_t i = j + 1; i < n; ++i) {
        double sum = ala[i * n + j];
        for (std::uint32_t k = 0; k < j; ++k) {
          sum -= ala[i * n + k] * ala[j * n + k];
        }
        ala[i * n + j] = sum / ala[j * n + j];
      }
    }
    if (!full) {
      std::fill(x.begin(), x.end(), 0.0);
      return false;
    }
    std::copy_n(b.begin(), n, x.begin());
    solve_cholesky(ala, n, x);
    for (double& v : x) {
      v = -v;
    }
    double val = dot(x, x) - r * r;
    if (val < 1e-10) {
      break;
    }
    std::array<double, 5> tmp{};
    std::copy(x.begin(), x.end(), tmp.begin());
    solve_cholesky(ala, n, {tmp.data(), n});
    double deriv = -2.0 * dot(x, {tmp.data(), n});
    double delta = -val / deriv;
    if (delta < 1e-10) {
      break;
    }
    la += delta;
  }
  for (std::uint32_t i = 0; i < n; ++i) {
    x[i] *= d[i];
  }
  return la != 0;
}

auto solve_pgs(const ConstraintProblem& p, const Regularized& reg,
               const ConstraintSettings& settings,
               Out<ConstraintSolution> solution) -> void {
  std::uint32_t n = p.dofs;
  std::uint32_t m = p.rows();

  // AR = J M⁻¹ Jᵀ + R, and b = J qacc_smooth - aref.
  std::vector<double> mass_factor = p.mass;
  factor_cholesky(InOut(mass_factor), n, MINVAL);
  std::vector<double> minv_jt(std::size_t{m} * n);  // Row r: M⁻¹ J_rᵀ.
  for (std::uint32_t r = 0; r < m; ++r) {
    std::span<double> row{&minv_jt[std::size_t{r} * n], n};
    std::copy_n(&p.jacobian[std::size_t{r} * n], n, row.begin());
    solve_cholesky(mass_factor, n, row);
  }
  std::vector<double> ar(std::size_t{m} * m);
  for (std::uint32_t a = 0; a < m; ++a) {
    for (std::uint32_t b = 0; b < m; ++b) {
      ar[a * m + b] = dot({&p.jacobian[std::size_t{a} * n], n},
                          {&minv_jt[std::size_t{b} * n], n});
    }
    ar[a * m + a] += reg.r[a];
  }
  std::vector<double> b(m);
  multiply(p.jacobian, m, n, p.qacc_smooth, b);
  for (std::uint32_t i = 0; i < m; ++i) {
    b[i] -= reg.aref[i];
  }

  // Warmstart: the last step's accelerations' forces, unless their dual
  // cost is positive.
  std::vector<double> force(m);
  std::vector<RowState> state(m);
  {
    std::vector<double> jar(m);
    multiply(p.jacobian, m, n, p.warmstart, jar);
    for (std::uint32_t i = 0; i < m; ++i) {
      jar[i] -= reg.aref[i];
    }
    update_rows(p, reg, jar, force, state);
    std::vector<double> arf(m);
    multiply(ar, m, m, force, arf);
    if (dot(force, b) + 0.5 * dot(force, arf) > 0) {
      std::fill(force.begin(), force.end(), 0.0);
    }
  }

  // Each row a block, but an elliptic cone's rows one.
  std::vector<std::uint32_t> blocks;
  for (std::uint32_t i = 0; i < m;) {
    blocks.push_back(i);
    i = p.kind[i] == ConstraintKind::ELLIPTIC ? group_end(p, i) : i + 1;
  }
  auto block_end = [&](std::uint32_t i) {
    return p.kind[i] == ConstraintKind::ELLIPTIC ? group_end(p, i) : i + 1;
  };
  // A block's forces back into their set (projectCone).
  auto project = [&](std::uint32_t i) {
    if (p.kind[i] == ConstraintKind::FRICTION) {
      force[i] = std::clamp(force[i], -p.friction_loss[i], p.friction_loss[i]);
    } else if (p.kind[i] == ConstraintKind::ELLIPTIC) {
      std::uint32_t dim = block_end(i) - i;
      if (force[i] < 0) {
        std::fill_n(&force[i], dim, 0.0);
      } else {
        project_ellipsoid({&force[i + 1], dim - 1}, force[i],
                          {p.friction[i].data(), dim - 1}, true);
      }
    } else if (force[i] < 0) {
      force[i] = 0;
    }
  };

  double scale = 1 / (settings.mean_inertia *
                      std::max<std::uint32_t>(1, settings.model_dofs));
  std::vector<std::uint32_t>& order = blocks;
  Pcg32 rng;
  rng.next();
  std::vector<double> previous = force;
  std::vector<double> momentum(m);
  std::uint32_t nesterov = 0;
  std::uint32_t iterations = 0;
  while (iterations < settings.iterations) {
    // Nesterov's momentum, restarted when it works against the sweep.
    double beta = iterations > 0
                      ? static_cast<double>(static_cast<int>(nesterov) - 1) /
                            static_cast<double>(nesterov + 2)
                      : 0.0;
    if (beta > 0) {
      for (std::uint32_t i = 0; i < m; ++i) {
        double saved = force[i];
        force[i] += beta * (force[i] - previous[i]);
        previous[i] = saved;
      }
      for (std::uint32_t i : blocks) {
        project(i);
      }
    } else {
      previous = force;
    }
    momentum = force;

    double improvement = 0.0;
    for (auto i = static_cast<std::uint32_t>(order.size()); i-- > 1;) {
      std::uint32_t j = rng.next() % (i + 1);
      std::swap(order[i], order[j]);
    }
    for (std::uint32_t i : order) {
      std::uint32_t dim = block_end(i) - i;
      std::array<double, 6> res{};
      std::array<double, 6> old{};
      std::array<double, 36> block{};
      for (std::uint32_t j = 0; j < dim; ++j) {
        res[j] = b[i + j] + dot({&ar[std::size_t{i + j} * m], m}, force);
        old[j] = force[i + j];
        for (std::uint32_t k = 0; k < dim; ++k) {
          block[j * dim + k] = ar[std::size_t{i + j} * m + i + k];
        }
      }
      if (p.kind[i] != ConstraintKind::ELLIPTIC) {
        force[i] -= res[0] / ar[i * m + i];
        project(i);
      } else {
        // The normal by itself if it is not pushing; else along the
        // block's own forces, then its friction by the QCQP in its
        // ellipsoid (solPGS).
        const std::array<double, 5>& mu = p.friction[i];
        if (force[i] < MINVAL) {
          force[i] -= res[0] / ar[i * m + i];
          force[i] = std::max(force[i], 0.0);
          std::fill_n(&force[i + 1], dim - 1, 0.0);
        } else {
          std::array<double, 6> v{};
          std::copy_n(&force[i], dim, v.begin());
          double denom = 0.0;
          for (std::uint32_t j = 0; j < dim; ++j) {
            double row = 0.0;
            for (std::uint32_t k = 0; k < dim; ++k) {
              row += block[j * dim + k] * v[k];
            }
            denom += v[j] * row;
          }
          if (denom >= MINVAL) {
            double x = -dot({v.data(), dim}, {res.data(), dim}) / denom;
            if (force[i] + x * v[0] < 0) {
              x = -v[0] / force[i];
            }
            for (std::uint32_t j = 0; j < dim; ++j) {
              force[i + j] += x * v[j];
            }
          }
        }
        std::array<double, 5> bc{};
        std::array<double, 25> ac{};
        for (std::uint32_t j = 0; j + 1 < dim; ++j) {
          bc[j] = res[j + 1];
          for (std::uint32_t k = 0; k + 1 < dim; ++k) {
            ac[j * (dim - 1) + k] = block[(j + 1) * dim + k + 1];
            bc[j] -= ac[j * (dim - 1) + k] * old[k + 1];
          }
          bc[j] += block[(j + 1) * dim] * (force[i] - old[0]);
        }
        if (force[i] < MINVAL) {
          std::fill_n(&force[i + 1], dim - 1, 0.0);
        } else {
          std::array<double, 5> friction{};
          std::span<double> x{friction.data(), dim - 1};
          if (solve_qcqp(x, {ac.data(), (dim - 1) * (dim - 1)},
                         {bc.data(), dim - 1}, {mu.data(), dim - 1},
                         force[i])) {
            project_ellipsoid(x, force[i], {mu.data(), dim - 1}, false);
          }
          std::copy_n(friction.begin(), dim - 1, &force[i + 1]);
        }
      }
      // Undone if it raised the cost (costChange).
      std::array<double, 6> delta{};
      for (std::uint32_t j = 0; j < dim; ++j) {
        delta[j] = force[i + j] - old[j];
      }
      double change = 0.0;
      for (std::uint32_t j = 0; j < dim; ++j) {
        double row = 0.0;
        for (std::uint32_t k = 0; k < dim; ++k) {
          row += block[j * dim + k] * delta[k];
        }
        change += 0.5 * delta[j] * row + delta[j] * res[j];
      }
      if (change > 1e-10) {
        std::copy_n(old.begin(), dim, &force[i]);
        change = 0;
      }
      improvement -= change;
    }
    improvement *= scale;

    if (iterations > 0) {
      double alignment = 0.0;
      for (std::uint32_t i = 0; i < m; ++i) {
        alignment += (force[i] - momentum[i]) * (momentum[i] - previous[i]);
      }
      nesterov = alignment < 0 ? 0 : nesterov + 1;
    } else {
      ++nesterov;
    }
    ++iterations;
    if (improvement < settings.tolerance) {
      break;
    }
  }

  // qacc = qacc_smooth + M⁻¹ Jᵀ f (mj_dualFinish).
  solution->qfrc_constraint.assign(n, 0.0);
  multiply_transposed(p.jacobian, m, n, force, solution->qfrc_constraint);
  solution->qacc = solution->qfrc_constraint;
  solve_cholesky(mass_factor, n, solution->qacc);
  for (std::uint32_t i = 0; i < n; ++i) {
    solution->qacc[i] += p.qacc_smooth[i];
  }
  solution->force = force;
  solution->iterations = iterations;
}

}  // namespace

auto solve_constraints(const ConstraintProblem& problem,
                       const ConstraintSettings& settings,
                       Out<ConstraintSolution> solution) -> void {
  Regularized reg = regularize(problem, settings.timestep, settings.impratio);
  if (settings.solver == Physics::Solver::PGS) {
    solve_pgs(problem, reg, settings, solution);
  } else {
    // Each thread keeps its own buffers from island to island.
    thread_local NewtonBuffers buffers;
    Newton{problem, reg, settings, InOut(buffers)}.solve(solution);
  }
}

}  // namespace simon::model
