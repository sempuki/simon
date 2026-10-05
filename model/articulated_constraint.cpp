// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

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
constexpr double IMPRATIO = 1.0;       // mjOption's impratio.
constexpr double LS_TOLERANCE = 0.01;  // mjOption's ls_tolerance.
constexpr std::uint32_t LS_ITERATIONS = 50;

enum class RowState : std::uint8_t {
  SATISFIED,
  QUADRATIC,
  LINEAR_NEGATIVE,
  LINEAR_POSITIVE,
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
};

auto regularize(const ConstraintProblem& p, double timestep) -> Regularized {
  std::uint32_t rows = p.rows();
  Regularized out{std::vector<double>(rows), std::vector<double>(rows),
                  std::vector<double>(rows)};
  for (std::uint32_t i = 0; i < rows; ++i) {
    std::uint32_t first = p.group[i];
    SoftConstraint soft = clamp_soft(p.soft[first], timestep);
    const std::array<double, 2>& ref = soft.reference;
    const std::array<double, 5>& solimp = soft.impedance;
    double imp = compute_impedance(solimp, p.pos[first], p.margin[first]);
    out.r[i] = std::max(MINVAL, (1 - imp) * p.diagonal[i] / imp);
    double k = 0.0;
    if (p.kind[i] == ConstraintKind::FRICTION) {
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
  // A pyramid's rows share one R, matching the elliptic cone's friction.
  for (std::uint32_t i = 0; i < rows; ++i) {
    if (p.kind[i] != ConstraintKind::PYRAMIDAL || p.group[i] != i) {
      continue;
    }
    double r1 = out.r[i] / std::max(MINVAL, IMPRATIO);
    double mu = p.friction[i] * std::sqrt(r1 / out.r[i]);
    double shared = 2 * mu * mu * out.r[i];
    for (std::uint32_t j = i; j < rows && p.group[j] == i; ++j) {
      out.r[j] = shared;
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
                 std::span<RowState> state) -> double {
  double cost = 0.0;
  for (std::uint32_t i = 0; i < p.rows(); ++i) {
    double d = reg.d[i];
    force[i] = -d * jar[i];
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
        starts_{sized(buffers->starts, std::size_t{m_} + 1)} {
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
    cost_ = update_rows(p, *reg_, jaref_, force_, state_);
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

  // H = M + Jᵀ D J over the quadratic rows, factored (MakeHessian,
  // FactorizeHessian).
  auto factor_hessian() -> void {
    const ConstraintProblem& p = *p_;
    hessian_ = p.mass;
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
      quad_[3 * i] = 0.5 * (jaref_[i] * dj0);
      quad_[3 * i + 1] = jv_[i] * dj0;
      quad_[3 * i + 2] = 0.5 * (jv_[i] * d * jv_[i]);
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

  // The cost at `alpha` less the cost at 0, and its first two derivatives
  // (PrimalEval).
  auto evaluate(InOut<Point> point) -> void {
    const ConstraintProblem& p = *p_;
    double alpha = point->alpha;
    double cost = 0.0;
    std::array<double, 2> deriv{};
    std::array<double, 3> total{0.0, quad_gauss_[1], quad_gauss_[2]};
    for (std::uint32_t i = 0; i < m_; ++i) {
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

  auto clamp_row = [&](std::uint32_t i) {
    if (p.kind[i] == ConstraintKind::FRICTION) {
      force[i] = std::clamp(force[i], -p.friction_loss[i], p.friction_loss[i]);
    } else if (force[i] < 0) {
      force[i] = 0;
    }
  };

  double scale = 1 / (settings.mean_inertia *
                      std::max<std::uint32_t>(1, settings.model_dofs));
  std::vector<std::uint32_t> order(m);
  for (std::uint32_t i = 0; i < m; ++i) {
    order[i] = i;
  }
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
      for (std::uint32_t i = 0; i < m; ++i) {
        clamp_row(i);
      }
    } else {
      previous = force;
    }
    momentum = force;

    double improvement = 0.0;
    for (std::uint32_t i = m; i-- > 1;) {
      std::uint32_t j = rng.next() % (i + 1);
      std::swap(order[i], order[j]);
    }
    for (std::uint32_t i : order) {
      double res = b[i] + dot({&ar[std::size_t{i} * m], m}, force);
      double old = force[i];
      force[i] -= res / ar[i * m + i];
      clamp_row(i);
      double delta = force[i] - old;
      double change = 0.5 * delta * delta * ar[i * m + i] + delta * res;
      if (change > 1e-10) {
        force[i] = old;
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
  Regularized reg = regularize(problem, settings.timestep);
  if (settings.solver == Physics::Solver::PGS) {
    solve_pgs(problem, reg, settings, solution);
  } else {
    // Each thread keeps its own buffers from island to island.
    thread_local NewtonBuffers buffers;
    Newton{problem, reg, settings, InOut(buffers)}.solve(solution);
  }
}

}  // namespace simon::model
