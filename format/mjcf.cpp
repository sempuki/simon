// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "format/mjcf.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <map>
#include <numbers>
#include <optional>
#include <span>
#include <sstream>
#include <utility>
#include <vector>

#include "base/core.hpp"
#include "pugixml.hpp"

template <>
const std::array<lib::StatusConditionEntry, simon::format::MJCF_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::format::MjcfError,
        simon::format::MJCF_ERROR_COUNT>::conditions_ = {
        lib::StatusConditionEntry{"model unreadable"},
        lib::StatusConditionEntry{"model malformed"},
        lib::StatusConditionEntry{"model unsupported"},
};

namespace simon::format {

namespace {

using model::Actuator;
using model::ArticulatedBody;
using model::ArticulatedModel;
using model::Dof;
using model::Geom;
using model::GeomType;
using model::Joint;
using model::JointType;
using model::Physics;
using model::Quaternion4;
using model::SoftConstraint;
using model::Array3;

using Failure = std::unexpected<lib::Status>;
using lib::Out;

constexpr double PI = std::numbers::pi;
constexpr double EPS = 1e-14;  // MuJoCo's mjEPS.

//-- MuJoCo's compiler arithmetic ----------------------------------------------
//
// Each in the order of operations of MuJoCo 3.14.0's user_util.cc and
// user_objects.cc (Apache-2.0), so that a compiled model matches MuJoCo's to
// rounding.

// Normalizes `n` numbers, unless their norm is within EPS of 1; the norm
// before, 0 if below EPS (mjuu_normvec).
auto normalize(std::span<double> v) -> double {
  double norm = 0.0;
  for (double x : v) {
    norm += x * x;
  }
  if (norm < EPS) {
    return 0.0;
  }
  norm = std::sqrt(norm);
  if (std::abs(norm - 1.0) > EPS) {
    for (double& x : v) {
      x /= norm;
    }
  }
  return norm;
}

// The rotation matrix of `q`, row by row (mjuu_quat2mat).
auto convert_to_matrix(const Quaternion4& q) -> std::array<double, 9> {
  if (q[0] == 1.0 && q[1] == 0.0 && q[2] == 0.0 && q[3] == 0.0) {
    return {1, 0, 0, 0, 1, 0, 0, 0, 1};
  }
  double q00 = q[0] * q[0];
  double q01 = q[0] * q[1];
  double q02 = q[0] * q[2];
  double q03 = q[0] * q[3];
  double q11 = q[1] * q[1];
  double q12 = q[1] * q[2];
  double q13 = q[1] * q[3];
  double q22 = q[2] * q[2];
  double q23 = q[2] * q[3];
  double q33 = q[3] * q[3];
  return {q00 + q11 - q22 - q33, 2 * (q12 - q03),       2 * (q13 + q02),
          2 * (q12 + q03),       q00 - q11 + q22 - q33, 2 * (q23 - q01),
          2 * (q13 - q02),       2 * (q23 + q01),       q00 - q11 - q22 + q33};
}

// a b, normalized (mjuu_mulquat).
auto multiply(const Quaternion4& a, const Quaternion4& b) -> Quaternion4 {
  Quaternion4 r{a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
  normalize(r);
  return r;
}

auto multiply_matrices(const std::array<double, 9>& a,
                       const std::array<double, 9>& b)
    -> std::array<double, 9> {
  std::array<double, 9> r{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      r[3 * i + j] =
          a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
    }
  }
  return r;
}

auto transpose(const std::array<double, 9>& m) -> std::array<double, 9> {
  return {m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]};
}

auto cross(const Array3& b, const Array3& c) -> Array3 {
  return {b[1] * c[2] - b[2] * c[1], b[2] * c[0] - b[0] * c[2],
          b[0] * c[1] - b[1] * c[0]};
}

// The minimal rotation from z to `v` (mjuu_z2quat).
auto rotate_z_to(const Array3& v) -> Quaternion4 {
  Array3 axis = cross({0.0, 0.0, 1.0}, v);
  double s = normalize(axis);
  if (s < 1e-10) {
    axis = {1.0, 0.0, 0.0};
  }
  double angle = std::atan2(s, v[2]);
  return {std::cos(angle / 2), axis[0] * std::sin(angle / 2),
          axis[1] * std::sin(angle / 2), axis[2] * std::sin(angle / 2)};
}

// The rotation whose columns are `x`, `y` and `z` (mjuu_frame2quat).
auto convert_frame(const Array3& x, const Array3& y, const Array3& z)
    -> Quaternion4 {
  const Array3* m[3] = {&x, &y, &z};  // m[column][row].
  auto at = [&](int c, int r) { return (*m[c])[r]; };
  Quaternion4 q{};
  if (at(0, 0) + at(1, 1) + at(2, 2) > 0) {
    q[0] = 0.5 * std::sqrt(1 + at(0, 0) + at(1, 1) + at(2, 2));
    q[1] = 0.25 * (at(1, 2) - at(2, 1)) / q[0];
    q[2] = 0.25 * (at(2, 0) - at(0, 2)) / q[0];
    q[3] = 0.25 * (at(0, 1) - at(1, 0)) / q[0];
  } else if (at(0, 0) > at(1, 1) && at(0, 0) > at(2, 2)) {
    q[1] = 0.5 * std::sqrt(1 + at(0, 0) - at(1, 1) - at(2, 2));
    q[0] = 0.25 * (at(1, 2) - at(2, 1)) / q[1];
    q[2] = 0.25 * (at(1, 0) + at(0, 1)) / q[1];
    q[3] = 0.25 * (at(2, 0) + at(0, 2)) / q[1];
  } else if (at(1, 1) > at(2, 2)) {
    q[2] = 0.5 * std::sqrt(1 - at(0, 0) + at(1, 1) - at(2, 2));
    q[0] = 0.25 * (at(2, 0) - at(0, 2)) / q[2];
    q[1] = 0.25 * (at(1, 0) + at(0, 1)) / q[2];
    q[3] = 0.25 * (at(2, 1) + at(1, 2)) / q[2];
  } else {
    q[3] = 0.5 * std::sqrt(1 - at(0, 0) - at(1, 1) + at(2, 2));
    q[0] = 0.25 * (at(0, 1) - at(1, 0)) / q[3];
    q[1] = 0.25 * (at(2, 0) + at(0, 2)) / q[3];
    q[2] = 0.25 * (at(2, 1) + at(1, 2)) / q[3];
  }
  normalize(q);
  return q;
}

// A diagonal inertia on axes turned by `q`, as the six entries xx, yy, zz,
// xy, xz, yz of the turned tensor (mjuu_globalinertia).
auto turn_inertia(const Array3& local, const Quaternion4& q)
    -> std::array<double, 6> {
  std::array<double, 9> m = convert_to_matrix(q);
  std::array<double, 9> t{m[0] * local[0], m[3] * local[0], m[6] * local[0],
                          m[1] * local[1], m[4] * local[1], m[7] * local[1],
                          m[2] * local[2], m[5] * local[2], m[8] * local[2]};
  return {m[0] * t[0] + m[1] * t[3] + m[2] * t[6],
          m[3] * t[1] + m[4] * t[4] + m[5] * t[7],
          m[6] * t[2] + m[7] * t[5] + m[8] * t[8],
          m[0] * t[1] + m[1] * t[4] + m[2] * t[7],
          m[0] * t[2] + m[1] * t[5] + m[2] * t[8],
          m[3] * t[2] + m[4] * t[5] + m[5] * t[8]};
}

// The parallel axis term of `mass` at `d` (mjuu_offcenter).
auto shift_inertia(double mass, const Array3& d) -> std::array<double, 6> {
  return {mass * (d[1] * d[1] + d[2] * d[2]),
          mass * (d[0] * d[0] + d[2] * d[2]),
          mass * (d[0] * d[0] + d[1] * d[1]),
          -mass * d[0] * d[1],
          -mass * d[0] * d[2],
          -mass * d[1] * d[2]};
}

// The eigenvalues of a symmetric 3x3 matrix, largest first, and the
// rotation to its eigenvectors, by Jacobi rotations kept as a quaternion
// (mjuu_eig3; G. H. Golub and C. F. Van Loan, Matrix Computations, 4th
// edition, 2013, section 8.5, the symmetric Schur decomposition).
auto decompose_symmetric(const std::array<double, 9>& mat)
    -> std::pair<Array3, Quaternion4> {
  constexpr double REL_TOL = 4e-15;
  constexpr double EIG_EPS = 1e-12;
  double scale = 0.0;
  for (double x : mat) {
    scale = std::max(scale, std::abs(x));
  }
  double tol = scale * REL_TOL;
  Quaternion4 quat{1.0, 0.0, 0.0, 0.0};
  Array3 eigval{};
  for (int iteration = 0; iteration < 500; ++iteration) {
    std::array<double, 9> v = convert_to_matrix(quat);
    std::array<double, 9> d =
        multiply_matrices(multiply_matrices(transpose(v), mat), v);
    eigval = {d[0], d[4], d[8]};
    int rk = 0;
    int ck = 0;
    int rotk = 0;
    if (std::abs(d[1]) > std::abs(d[2]) && std::abs(d[1]) > std::abs(d[5])) {
      rk = 0;
      ck = 1;
      rotk = 2;
    } else if (std::abs(d[2]) > std::abs(d[5])) {
      rk = 0;
      ck = 2;
      rotk = 1;
    } else {
      rk = 1;
      ck = 2;
      rotk = 0;
    }
    if (std::abs(d[3 * rk + ck]) <= tol) {
      break;
    }
    double tau = (d[4 * ck] - d[4 * rk]) / (2 * d[3 * rk + ck]);
    double t = tau >= 0 ? 1.0 / (tau + std::sqrt(1 + tau * tau))
                        : -1.0 / (-tau + std::sqrt(1 + tau * tau));
    double h = t / (1 + std::sqrt(1 + t * t));
    Quaternion4 turn{1 / std::sqrt(1 + h * h), 0.0, 0.0, 0.0};
    turn[rotk + 1] = (rotk == 1 ? h : -h) * turn[0];
    quat = multiply(quat, turn);
    normalize(quat);
  }
  // Largest first, by bubble sort over 0, 1, 0.
  double eps = scale * EIG_EPS;
  for (int j = 0; j < 3; ++j) {
    int lead = j % 2;
    if (eigval[lead] + eps < eigval[lead + 1]) {
      std::swap(eigval[lead], eigval[lead + 1]);
      Quaternion4 turn{0.707106781186548, 0.0, 0.0, 0.0};
      turn[(lead + 2) % 3 + 1] = turn[0];
      quat = multiply(quat, turn);
      normalize(quat);
    }
  }
  return {eigval, quat};
}

// A tensor given as xx, yy, zz, xy, xz, yz on its principal axes: the
// moments and the rotation to them (mjuu_fullInertia).
auto find_principal_axes(const std::array<double, 6>& full)
    -> std::pair<Array3, Quaternion4> {
  return decompose_symmetric({full[0], full[3], full[4], full[3], full[1],
                              full[5], full[4], full[5], full[2]});
}

//-- Specifications, as read ---------------------------------------------------

// A frame's orientation, as one of the ways MJCF gives it.
struct Orientation final {
  enum class Kind : std::uint8_t { QUAT, AXIS_ANGLE, XY_AXES, Z_AXIS, EULER };

  Kind kind = Kind::QUAT;
  Quaternion4 quat{1.0, 0.0, 0.0, 0.0};
  std::array<double, 4> axis_angle{};
  std::array<double, 6> xy_axes{};
  Array3 z_axis{};
  Array3 euler{};
};

enum class Limited : std::uint8_t { AUTO, YES, NO };

struct GeomSpec final {
  Geom geom;
  Orientation orientation;
  std::optional<std::array<double, 6>> fromto;
  std::optional<double> mass;
  double density = 1000.0;
  int group = 0;
};

struct JointSpec final {
  Joint joint;
  Limited limited = Limited::AUTO;
  double springref = 0.0;
  double ref = 0.0;
  double damping = 0.0;
  double armature = 0.0;
  double friction_loss = 0.0;
};

struct MotorSpec final {
  Actuator actuator;
  std::string joint;
  Limited limited = Limited::AUTO;
};

// What a default class gives each element.
struct Defaults final {
  GeomSpec geom;
  JointSpec joint;
  MotorSpec motor;
};

struct InertialSpec final {
  Array3 pos{};
  Orientation orientation;
  double mass = 0.0;
  Array3 diagonal{};
  std::optional<std::array<double, 6>> full;
};

struct BodySpec final {
  std::string name;
  Array3 pos{};
  Orientation orientation;
  std::optional<InertialSpec> inertial;
  std::vector<JointSpec> joints;
  std::vector<GeomSpec> geoms;
  std::vector<BodySpec> children;
};

struct CompilerSpec final {
  bool degree = true;
  std::string euler_sequence = "xyz";
  enum class FromGeom : std::uint8_t {
    NO,
    YES,
    AUTO
  } from_geom = FromGeom::AUTO;
  bool auto_limits = true;
  double bound_mass = 0.0;
  double bound_inertia = 0.0;
  bool balance_inertia = false;
  std::array<int, 2> inertia_groups{0, 5};
};

//-- Reading -------------------------------------------------------------------

class Reader final {
 public:
  auto read(std::string_view text)
      -> std::expected<ArticulatedModel, lib::Status> {
    pugi::xml_document document;
    if (pugi::xml_parse_result result =
            document.load_buffer(text.data(), text.size());
        !result) {
      return fail(result.description());
    }
    pugi::xml_node root = document.child("mujoco");
    if (!root) {
      return fail("no mujoco element");
    }
    ArticulatedModel model{.name = root.attribute("model").as_string()};
    // The compiler's settings and the defaults come first, wherever they
    // stand.
    for (pugi::xml_node compiler : root.children("compiler")) {
      RETURN_IF_UNEXPECTED(read_compiler(compiler));
    }
    defaults_["main"] = Defaults{};
    for (pugi::xml_node node : root.children("default")) {
      RETURN_IF_UNEXPECTED(read_defaults(node, defaults_["main"]));
    }
    BodySpec world{.name = "world"};
    for (pugi::xml_node node : root.children()) {
      std::string_view kind = node.name();
      if (kind == "compiler" || kind == "default") {
        continue;
      }
      if (kind == "option") {
        RETURN_IF_UNEXPECTED(read_option(node, Out(model.physics)));
      } else if (kind == "worldbody") {
        RETURN_IF_UNEXPECTED(read_body_contents(node, "main", Out(world)));
      } else if (kind == "actuator") {
        for (pugi::xml_node actuator : node.children()) {
          RETURN_IF_UNEXPECTED(read_actuator(actuator));
        }
      } else if (kind == "visual" || kind == "asset" || kind == "statistic" ||
                 kind == "sensor" || kind == "keyframe" || kind == "custom" ||
                 kind == "size") {
        continue;  // Shows or records the model; does not move it.
      } else {
        return refuse(node);
      }
    }
    RETURN_IF_UNEXPECTED(compile(world, Out(model)));
    return model;
  }

 private:
  auto fail(std::string_view why) const -> Failure {
    return Failure{lib::raise(MjcfError::MALFORMED, std::string{why})};
  }
  auto refuse(pugi::xml_node node) const -> Failure {
    return Failure{lib::raise(MjcfError::UNSUPPORTED,
                              "<" + std::string{node.name()} + ">")};
  }
  auto refuse(pugi::xml_node node, std::string_view attribute) const
      -> Failure {
    return Failure{lib::raise(
        MjcfError::UNSUPPORTED,
        "<" + std::string{node.name()} + "> " + std::string{attribute})};
  }

  // Up to `into.size()` numbers from the attribute, if it is there, the
  // rest left as they were, as MuJoCo reads a partial vector.
  auto read_numbers(pugi::xml_node node, const char* name,
                    std::span<double> into) const
      -> std::expected<bool, lib::Status> {
    pugi::xml_attribute attribute = node.attribute(name);
    if (!attribute) {
      return false;
    }
    std::string_view text = attribute.as_string();
    std::size_t count = 0;
    while (!text.empty()) {
      std::size_t start = text.find_first_not_of(" \t\n\r");
      if (start == std::string_view::npos) {
        break;
      }
      text.remove_prefix(start);
      std::size_t end = text.find_first_of(" \t\n\r");
      std::string_view word = text.substr(0, end);
      if (count == into.size()) {
        return fail("<" + std::string{node.name()} + "> " + name +
                    " has too many numbers");
      }
      double value = 0.0;
      // A leading '+' or '.' from_chars does not take on its own.
      std::string number{word};
      if (number.starts_with('+')) {
        number.erase(0, 1);
      }
      auto [ptr, error] =
          std::from_chars(number.data(), number.data() + number.size(), value);
      if (error != std::errc{} || ptr != number.data() + number.size()) {
        return fail("<" + std::string{node.name()} + "> " + name +
                    " is not a number: " + std::string{word});
      }
      into[count++] = value;
      text.remove_prefix(end == std::string_view::npos ? text.size() : end);
    }
    if (count == 0) {
      return fail("<" + std::string{node.name()} + "> " + name + " is empty");
    }
    return true;
  }

  template <std::size_t N>
  auto read_array(pugi::xml_node node, const char* name,
                  std::array<double, N>& into) const
      -> std::expected<bool, lib::Status> {
    return read_numbers(node, name, into);
  }

  auto read_number(pugi::xml_node node, const char* name, double& into) const
      -> std::expected<bool, lib::Status> {
    return read_numbers(node, name, std::span<double>{&into, 1});
  }

  template <typename Integer>
  auto read_integer(pugi::xml_node node, const char* name, Integer& into) const
      -> std::expected<void, lib::Status> {
    double value = 0.0;
    RETURN_OR_ASSIGN(bool read, read_number(node, name, value));
    if (read) {
      into = static_cast<Integer>(value);
    }
    return {};
  }

  auto read_flag(pugi::xml_node node, const char* name, bool& into) const
      -> std::expected<void, lib::Status> {
    pugi::xml_attribute attribute = node.attribute(name);
    if (!attribute) {
      return {};
    }
    std::string_view text = attribute.as_string();
    if (text != "true" && text != "false") {
      return fail("<" + std::string{node.name()} + "> " + name +
                  " must be true or false");
    }
    into = text == "true";
    return {};
  }

  auto read_limited(pugi::xml_node node, const char* name, Limited& into) const
      -> std::expected<void, lib::Status> {
    pugi::xml_attribute attribute = node.attribute(name);
    if (!attribute) {
      return {};
    }
    std::string_view text = attribute.as_string();
    into = text == "true"    ? Limited::YES
           : text == "false" ? Limited::NO
                             : Limited::AUTO;
    return {};
  }

  // The one orientation the element gives, if any.
  auto read_orientation(pugi::xml_node node, Orientation& into) const
      -> std::expected<void, lib::Status> {
    int given = 0;
    std::array<double, 4> quat = into.quat;
    RETURN_OR_ASSIGN(bool has_quat, read_array(node, "quat", quat));
    if (has_quat) {
      into.kind = Orientation::Kind::QUAT;
      into.quat = quat;
      ++given;
    }
    RETURN_OR_ASSIGN(bool has_axis,
                     read_array(node, "axisangle", into.axis_angle));
    if (has_axis) {
      into.kind = Orientation::Kind::AXIS_ANGLE;
      ++given;
    }
    RETURN_OR_ASSIGN(bool has_xy, read_array(node, "xyaxes", into.xy_axes));
    if (has_xy) {
      into.kind = Orientation::Kind::XY_AXES;
      ++given;
    }
    RETURN_OR_ASSIGN(bool has_z, read_array(node, "zaxis", into.z_axis));
    if (has_z) {
      into.kind = Orientation::Kind::Z_AXIS;
      ++given;
    }
    RETURN_OR_ASSIGN(bool has_euler, read_array(node, "euler", into.euler));
    if (has_euler) {
      into.kind = Orientation::Kind::EULER;
      ++given;
    }
    if (given > 1) {
      return fail("<" + std::string{node.name()} +
                  "> gives more than one orientation");
    }
    return {};
  }

  // The quaternion an orientation gives (ResolveOrientation).
  auto resolve(const Orientation& orientation) const
      -> std::expected<Quaternion4, lib::Status> {
    switch (orientation.kind) {
      case Orientation::Kind::QUAT: {
        Quaternion4 q = orientation.quat;
        normalize(q);
        return q;
      }
      case Orientation::Kind::AXIS_ANGLE: {
        std::array<double, 4> a = orientation.axis_angle;
        if (compiler_.degree) {
          a[3] = a[3] / 180.0 * PI;
        }
        if (normalize(std::span{a}.first(3)) < EPS) {
          return fail("axisangle too small");
        }
        double half = a[3] / 2;
        return Quaternion4{std::cos(half), std::sin(half) * a[0],
                           std::sin(half) * a[1], std::sin(half) * a[2]};
      }
      case Orientation::Kind::XY_AXES: {
        std::array<double, 6> xy = orientation.xy_axes;
        if (normalize(std::span{xy}.first(3)) < EPS) {
          return fail("xaxis too small");
        }
        double d = xy[0] * xy[3] + xy[1] * xy[4] + xy[2] * xy[5];
        xy[3] -= xy[0] * d;
        xy[4] -= xy[1] * d;
        xy[5] -= xy[2] * d;
        if (normalize(std::span{xy}.subspan(3, 3)) < EPS) {
          return fail("yaxis too small");
        }
        Array3 x{xy[0], xy[1], xy[2]};
        Array3 y{xy[3], xy[4], xy[5]};
        Array3 z = cross(x, y);
        if (normalize(z) < EPS) {
          return fail("cross(xaxis, yaxis) too small");
        }
        return convert_frame(x, y, z);
      }
      case Orientation::Kind::Z_AXIS: {
        Array3 z = orientation.z_axis;
        if (normalize(z) < EPS) {
          return fail("zaxis too small");
        }
        return rotate_z_to(z);
      }
      case Orientation::Kind::EULER: {
        Array3 euler = orientation.euler;
        if (compiler_.degree) {
          for (double& angle : euler) {
            angle = angle / 180.0 * PI;
          }
        }
        Quaternion4 q{1.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < 3; ++i) {
          char axis = compiler_.euler_sequence[static_cast<std::size_t>(i)];
          Quaternion4 turn{std::cos(euler[i] / 2), 0.0, 0.0, 0.0};
          double s = std::sin(euler[i] / 2);
          char lower = static_cast<char>(std::tolower(axis));
          turn[lower == 'x' ? 1 : lower == 'y' ? 2 : 3] = s;
          // Moving axes post-multiply, fixed axes pre-multiply.
          q = std::islower(axis) != 0 ? multiply(q, turn) : multiply(turn, q);
        }
        normalize(q);
        return q;
      }
    }
    return Quaternion4{1.0, 0.0, 0.0, 0.0};
  }

  auto read_compiler(pugi::xml_node node) -> std::expected<void, lib::Status> {
    for (pugi::xml_attribute attribute : node.attributes()) {
      std::string_view name = attribute.name();
      std::string_view value = attribute.as_string();
      if (name == "angle") {
        compiler_.degree = value != "radian";
      } else if (name == "eulerseq") {
        if (value.size() != 3 ||
            value.find_first_not_of("xyzXYZ") != std::string_view::npos) {
          return fail("eulerseq must be three of x, y, z, X, Y, Z");
        }
        compiler_.euler_sequence = std::string{value};
      } else if (name == "inertiafromgeom") {
        compiler_.from_geom = value == "true"    ? CompilerSpec::FromGeom::YES
                              : value == "false" ? CompilerSpec::FromGeom::NO
                                                 : CompilerSpec::FromGeom::AUTO;
      } else if (name == "autolimits") {
        RETURN_IF_UNEXPECTED(
            read_flag(node, "autolimits", compiler_.auto_limits));
      } else if (name == "boundmass") {
        RETURN_IF_UNEXPECTED(
            read_number(node, "boundmass", compiler_.bound_mass));
      } else if (name == "boundinertia") {
        RETURN_IF_UNEXPECTED(
            read_number(node, "boundinertia", compiler_.bound_inertia));
      } else if (name == "balanceinertia") {
        RETURN_IF_UNEXPECTED(
            read_flag(node, "balanceinertia", compiler_.balance_inertia));
      } else if (name == "inertiagrouprange") {
        std::array<double, 2> range{0, 5};
        RETURN_IF_UNEXPECTED(read_array(node, "inertiagrouprange", range));
        compiler_.inertia_groups = {static_cast<int>(range[0]),
                                    static_cast<int>(range[1])};
      } else if (name == "meshdir" || name == "texturedir" ||
                 name == "assetdir") {
        continue;
      } else if ((name == "alignfree" || name == "fusestatic" ||
                  name == "discardvisual") &&
                 value == "false") {
        continue;
      } else {
        return refuse(node, name);
      }
    }
    return {};
  }

  auto read_option(pugi::xml_node node, Out<Physics> physics)
      -> std::expected<void, lib::Status> {
    for (pugi::xml_attribute attribute : node.attributes()) {
      std::string_view name = attribute.name();
      std::string_view value = attribute.as_string();
      if (name == "timestep") {
        RETURN_IF_UNEXPECTED(read_number(node, "timestep", physics->timestep));
      } else if (name == "gravity") {
        RETURN_IF_UNEXPECTED(read_array(node, "gravity", physics->gravity));
      } else if (name == "integrator") {
        static constexpr std::array<
            std::pair<std::string_view, Physics::Integrator>, 4>
            NAMES{{{"Euler", Physics::Integrator::EULER},
                   {"RK4", Physics::Integrator::RK4},
                   {"implicit", Physics::Integrator::IMPLICIT},
                   {"implicitfast", Physics::Integrator::IMPLICIT_FAST}}};
        auto found = std::ranges::find(NAMES, value,
                                       &decltype(NAMES)::value_type::first);
        if (found == NAMES.end()) {
          return fail("unknown integrator " + std::string{value});
        }
        physics->integrator = found->second;
      } else if (name == "cone") {
        physics->cone = value == "elliptic" ? Physics::Cone::ELLIPTIC
                                            : Physics::Cone::PYRAMIDAL;
      } else if (name == "solver") {
        physics->solver = value == "PGS"  ? Physics::Solver::PGS
                          : value == "CG" ? Physics::Solver::CG
                                          : Physics::Solver::NEWTON;
      } else if (name == "iterations") {
        RETURN_IF_UNEXPECTED(
            read_integer(node, "iterations", physics->iterations));
      } else if (name == "tolerance") {
        RETURN_IF_UNEXPECTED(
            read_number(node, "tolerance", physics->tolerance));
      } else {
        return refuse(node, name);
      }
    }
    if (node.first_child()) {
      return refuse(node.first_child());
    }
    return {};
  }

  //-- Defaults ----------------------------------------------------------------

  // A default class and those under it, each starting from its parent's.
  auto read_defaults(pugi::xml_node node, const Defaults& parent)
      -> std::expected<void, lib::Status> {
    std::string name = node.attribute("class").as_string("main");
    Defaults defaults = parent;
    for (pugi::xml_node child : node.children()) {
      std::string_view kind = child.name();
      if (kind == "geom") {
        RETURN_IF_UNEXPECTED(apply_geom(child, defaults.geom));
      } else if (kind == "joint") {
        RETURN_IF_UNEXPECTED(apply_joint(child, defaults.joint));
      } else if (kind == "motor") {
        RETURN_IF_UNEXPECTED(apply_motor(child, defaults.motor));
      } else if (kind == "default") {
        continue;
      } else if (kind == "site" || kind == "camera" || kind == "light" ||
                 kind == "material") {
        continue;
      } else {
        return refuse(child);
      }
    }
    defaults_[name] = defaults;
    for (pugi::xml_node child : node.children("default")) {
      if (!child.attribute("class")) {
        return fail("a nested <default> needs a class");
      }
      RETURN_IF_UNEXPECTED(read_defaults(child, defaults_[name]));
    }
    return {};
  }

  // The defaults of the class an element names, or else of the class it
  // inherits.
  auto find_defaults(pugi::xml_node node, const std::string& inherited) const
      -> std::expected<const Defaults*, lib::Status> {
    std::string name = node.attribute("class").as_string(inherited.c_str());
    auto found = defaults_.find(name);
    if (found == defaults_.end()) {
      return fail("no default class " + name);
    }
    return &found->second;
  }

  //-- Elements ----------------------------------------------------------------

  auto apply_geom(pugi::xml_node node, GeomSpec& spec) const
      -> std::expected<void, lib::Status> {
    Geom& geom = spec.geom;
    for (pugi::xml_attribute attribute : node.attributes()) {
      std::string_view name = attribute.name();
      static constexpr std::array<std::string_view, 21> KNOWN{
          "name",    "class",       "type",     "size",     "pos",
          "quat",    "axisangle",   "xyaxes",   "zaxis",    "euler",
          "fromto",  "mass",        "density",  "friction", "condim",
          "contype", "conaffinity", "priority", "solref",   "solimp",
          "margin"};
      static constexpr std::array<std::string_view, 4> SHOWN{"rgba", "material",
                                                             "group", "user"};
      if (name == "gap" || std::ranges::contains(KNOWN, name) ||
          std::ranges::contains(SHOWN, name)) {
        continue;
      }
      if (name == "solmix" && attribute.as_double() == 1.0) {
        continue;
      }
      return refuse(node, name);
    }
    if (node.attribute("name")) {
      geom.name = node.attribute("name").as_string();
    }
    if (pugi::xml_attribute type = node.attribute("type")) {
      static constexpr std::array<std::pair<std::string_view, GeomType>, 6>
          TYPES{{{"plane", GeomType::PLANE},
                 {"sphere", GeomType::SPHERE},
                 {"capsule", GeomType::CAPSULE},
                 {"ellipsoid", GeomType::ELLIPSOID},
                 {"cylinder", GeomType::CYLINDER},
                 {"box", GeomType::BOX}}};
      auto found = std::ranges::find(TYPES, std::string_view{type.as_string()},
                                     &decltype(TYPES)::value_type::first);
      if (found == TYPES.end()) {
        return refuse(node, std::string{"type "} + type.as_string());
      }
      geom.type = found->second;
    }
    RETURN_IF_UNEXPECTED(read_array(node, "size", geom.size));
    RETURN_IF_UNEXPECTED(read_array(node, "pos", geom.pos));
    RETURN_IF_UNEXPECTED(read_orientation(node, spec.orientation));
    std::array<double, 6> fromto{};
    RETURN_OR_ASSIGN(bool has_fromto, read_array(node, "fromto", fromto));
    if (has_fromto) {
      spec.fromto = fromto;
    }
    double mass = 0.0;
    RETURN_OR_ASSIGN(bool has_mass, read_number(node, "mass", mass));
    if (has_mass) {
      spec.mass = mass;
    }
    RETURN_IF_UNEXPECTED(read_number(node, "density", spec.density));
    RETURN_IF_UNEXPECTED(read_array(node, "friction", geom.friction));
    RETURN_IF_UNEXPECTED(read_integer(node, "condim", geom.condim));
    RETURN_IF_UNEXPECTED(read_integer(node, "contype", geom.contype));
    RETURN_IF_UNEXPECTED(read_integer(node, "conaffinity", geom.conaffinity));
    RETURN_IF_UNEXPECTED(read_integer(node, "priority", geom.priority));
    RETURN_IF_UNEXPECTED(read_array(node, "solref", geom.contact.reference));
    RETURN_IF_UNEXPECTED(read_array(node, "solimp", geom.contact.impedance));
    RETURN_IF_UNEXPECTED(read_number(node, "margin", geom.margin));
    RETURN_IF_UNEXPECTED(read_number(node, "gap", geom.gap));
    RETURN_IF_UNEXPECTED(read_integer(node, "group", spec.group));
    return {};
  }

  auto apply_joint(pugi::xml_node node, JointSpec& spec) const
      -> std::expected<void, lib::Status> {
    Joint& joint = spec.joint;
    for (pugi::xml_attribute attribute : node.attributes()) {
      std::string_view name = attribute.name();
      static constexpr std::array<std::string_view, 20> KNOWN{
          "name",
          "class",
          "type",
          "pos",
          "axis",
          "range",
          "limited",
          "stiffness",
          "springref",
          "damping",
          "armature",
          "frictionloss",
          "ref",
          "margin",
          "solreflimit",
          "solimplimit",
          "group",
          "user",
          "actuatorfrclimited",
          "actuatorgravcomp"};
      if (!std::ranges::contains(KNOWN, name)) {
        return refuse(node, name);
      }
      if ((name == "actuatorfrclimited" &&
           attribute.as_string() != std::string{"false"} &&
           attribute.as_string() != std::string{"auto"}) ||
          (name == "actuatorgravcomp" &&
           attribute.as_string() != std::string{"false"})) {
        return refuse(node, name);
      }
    }
    if (node.attribute("name")) {
      joint.name = node.attribute("name").as_string();
    }
    if (pugi::xml_attribute type = node.attribute("type")) {
      std::string_view text = type.as_string();
      joint.type = text == "free"    ? JointType::FREE
                   : text == "ball"  ? JointType::BALL
                   : text == "slide" ? JointType::SLIDE
                                     : JointType::HINGE;
      if (text != "free" && text != "ball" && text != "slide" &&
          text != "hinge") {
        return fail("unknown joint type " + std::string{text});
      }
    }
    RETURN_IF_UNEXPECTED(read_array(node, "pos", joint.pos));
    RETURN_IF_UNEXPECTED(read_array(node, "axis", joint.axis));
    RETURN_IF_UNEXPECTED(read_array(node, "range", joint.range));
    RETURN_IF_UNEXPECTED(read_limited(node, "limited", spec.limited));
    RETURN_IF_UNEXPECTED(read_number(node, "stiffness", joint.stiffness));
    RETURN_IF_UNEXPECTED(read_number(node, "springref", spec.springref));
    RETURN_IF_UNEXPECTED(read_number(node, "damping", spec.damping));
    RETURN_IF_UNEXPECTED(read_number(node, "armature", spec.armature));
    RETURN_IF_UNEXPECTED(read_number(node, "frictionloss", spec.friction_loss));
    RETURN_IF_UNEXPECTED(read_number(node, "ref", spec.ref));
    RETURN_IF_UNEXPECTED(read_number(node, "margin", joint.margin));
    RETURN_IF_UNEXPECTED(
        read_array(node, "solreflimit", joint.limit.reference));
    RETURN_IF_UNEXPECTED(
        read_array(node, "solimplimit", joint.limit.impedance));
    return {};
  }

  auto apply_motor(pugi::xml_node node, MotorSpec& spec) const
      -> std::expected<void, lib::Status> {
    for (pugi::xml_attribute attribute : node.attributes()) {
      std::string_view name = attribute.name();
      static constexpr std::array<std::string_view, 7> KNOWN{
          "name",      "class",       "joint", "gear",
          "ctrlrange", "ctrllimited", "group"};
      if (!std::ranges::contains(KNOWN, name)) {
        return refuse(node, name);
      }
    }
    if (node.attribute("name")) {
      spec.actuator.name = node.attribute("name").as_string();
    }
    if (node.attribute("joint")) {
      spec.joint = node.attribute("joint").as_string();
    }
    RETURN_IF_UNEXPECTED(read_array(node, "gear", spec.actuator.gear));
    RETURN_IF_UNEXPECTED(
        read_array(node, "ctrlrange", spec.actuator.control_range));
    RETURN_IF_UNEXPECTED(read_limited(node, "ctrllimited", spec.limited));
    return {};
  }

  auto read_inertial(pugi::xml_node node)
      -> std::expected<InertialSpec, lib::Status> {
    InertialSpec spec;
    for (pugi::xml_attribute attribute : node.attributes()) {
      std::string_view name = attribute.name();
      static constexpr std::array<std::string_view, 10> KNOWN{
          "pos",   "quat", "axisangle",   "xyaxes",      "zaxis",
          "euler", "mass", "diaginertia", "fullinertia", "class"};
      if (!std::ranges::contains(KNOWN, name)) {
        return refuse(node, name);
      }
    }
    RETURN_IF_UNEXPECTED(read_array(node, "pos", spec.pos));
    RETURN_IF_UNEXPECTED(read_orientation(node, spec.orientation));
    RETURN_OR_ASSIGN(bool has_mass, read_number(node, "mass", spec.mass));
    if (!has_mass) {
      return fail("<inertial> needs a mass");
    }
    RETURN_OR_ASSIGN(bool diagonal,
                     read_array(node, "diaginertia", spec.diagonal));
    std::array<double, 6> full{};
    RETURN_OR_ASSIGN(bool has_full, read_array(node, "fullinertia", full));
    if (has_full) {
      spec.full = full;
    }
    if (diagonal == has_full) {
      return fail("<inertial> needs one of diaginertia and fullinertia");
    }
    return spec;
  }

  // A body's elements, those that take a class inheriting `inherited`.
  auto read_body_contents(pugi::xml_node node, const std::string& inherited,
                          Out<BodySpec> body)
      -> std::expected<void, lib::Status> {
    std::string classes =
        node.attribute("childclass").as_string(inherited.c_str());
    for (pugi::xml_node child : node.children()) {
      std::string_view kind = child.name();
      if (kind == "geom") {
        RETURN_OR_ASSIGN(const Defaults* defaults,
                         find_defaults(child, classes));
        GeomSpec geom = defaults->geom;
        geom.geom.name.clear();
        RETURN_IF_UNEXPECTED(apply_geom(child, geom));
        body->geoms.push_back(std::move(geom));
      } else if (kind == "joint" || kind == "freejoint") {
        JointSpec joint;
        if (kind == "joint") {
          RETURN_OR_ASSIGN(const Defaults* defaults,
                           find_defaults(child, classes));
          joint = defaults->joint;
          joint.joint.name.clear();
          RETURN_IF_UNEXPECTED(apply_joint(child, joint));
        } else {
          for (pugi::xml_attribute attribute : child.attributes()) {
            std::string_view name = attribute.name();
            if (name != "name" && name != "group" &&
                !(name == "align" &&
                  attribute.as_string() == std::string{"false"})) {
              return refuse(child, name);
            }
          }
          joint.joint.name = child.attribute("name").as_string();
          joint.joint.type = JointType::FREE;
        }
        body->joints.push_back(std::move(joint));
      } else if (kind == "inertial") {
        RETURN_OR_ASSIGN(body->inertial, read_inertial(child));
      } else if (kind == "body") {
        BodySpec spec{.name = child.attribute("name").as_string()};
        for (pugi::xml_attribute attribute : child.attributes()) {
          std::string_view name = attribute.name();
          static constexpr std::array<std::string_view, 9> KNOWN{
              "name",  "pos",   "quat",       "axisangle", "xyaxes",
              "zaxis", "euler", "childclass", "user"};
          if (!std::ranges::contains(KNOWN, name) &&
              !(name == "mocap" &&
                attribute.as_string() == std::string{"false"}) &&
              !(name == "gravcomp" && attribute.as_double() == 0.0)) {
            return refuse(child, name);
          }
        }
        RETURN_IF_UNEXPECTED(read_array(child, "pos", spec.pos));
        RETURN_IF_UNEXPECTED(read_orientation(child, spec.orientation));
        RETURN_IF_UNEXPECTED(read_body_contents(child, classes, Out(spec)));
        body->children.push_back(std::move(spec));
      } else if (kind == "site" || kind == "camera" || kind == "light") {
        continue;  // Shows the model; does not move it.
      } else {
        return refuse(child);
      }
    }
    return {};
  }

  auto read_actuator(pugi::xml_node node) -> std::expected<void, lib::Status> {
    if (std::string_view{node.name()} != "motor") {
      return refuse(node);
    }
    RETURN_OR_ASSIGN(const Defaults* defaults, find_defaults(node, "main"));
    MotorSpec motor = defaults->motor;
    motor.actuator.name.clear();
    motor.joint.clear();
    RETURN_IF_UNEXPECTED(apply_motor(node, motor));
    if (motor.joint.empty()) {
      return fail("<motor> needs a joint");
    }
    motors_.push_back(std::move(motor));
    return {};
  }

  //-- Compiling ---------------------------------------------------------------

  // A geom's mass and principal moments from its shape and density
  // (mjCGeom::GetVolume and SetInertia, solid shapes).
  static auto compute_volume(const Geom& geom) -> double {
    const Array3& s = geom.size;
    switch (geom.type) {
      case GeomType::SPHERE:
        return 4 * PI * s[0] * s[0] * s[0] / 3;
      case GeomType::CAPSULE: {
        double height = 2 * s[1];
        return PI * (s[0] * s[0] * height + 4 * s[0] * s[0] * s[0] / 3);
      }
      case GeomType::CYLINDER:
        return PI * s[0] * s[0] * 2 * s[1];
      case GeomType::ELLIPSOID:
        return 4 * PI * s[0] * s[1] * s[2] / 3;
      case GeomType::BOX:
        return s[0] * s[1] * s[2] * 8;
      default:
        return 0.0;
    }
  }

  static auto compute_inertia(const Geom& geom, double mass) -> Array3 {
    const Array3& s = geom.size;
    switch (geom.type) {
      case GeomType::SPHERE: {
        double i = 2 * mass * s[0] * s[0] / 5;
        return {i, i, i};
      }
      case GeomType::CAPSULE: {
        double height = 2 * s[1];
        double radius = s[0];
        double sphere_mass = mass * 4 * radius / (4 * radius + 3 * height);
        double cylinder_mass = mass - sphere_mass;
        double side =
            cylinder_mass * (3 * radius * radius + height * height) / 12;
        double axial = cylinder_mass * radius * radius / 2;
        double sphere = 2 * sphere_mass * radius * radius / 5;
        side += sphere + sphere_mass * height * (3 * radius + 2 * height) / 8;
        axial += sphere;
        return {side, side, axial};
      }
      case GeomType::CYLINDER: {
        double height = 2 * s[1];
        double side = mass * (3 * s[0] * s[0] + height * height) / 12;
        return {side, side, mass * s[0] * s[0] / 2};
      }
      case GeomType::ELLIPSOID: {
        double s00 = s[0] * s[0];
        double s11 = s[1] * s[1];
        double s22 = s[2] * s[2];
        return {mass * (s11 + s22) / 5, mass * (s00 + s22) / 5,
                mass * (s00 + s11) / 5};
      }
      case GeomType::BOX:
        return {mass * (s[1] * s[1] + s[2] * s[2]) / 3,
                mass * (s[0] * s[0] + s[2] * s[2]) / 3,
                mass * (s[0] * s[0] + s[1] * s[1]) / 3};
      default:
        return {};
    }
  }

  // A geom's frame and size from fromto or its orientation, and its mass
  // and inertia if `infer` (mjCGeom::Compile).
  auto compile_geom(GeomSpec spec, bool infer)
      -> std::expected<std::pair<Geom, double>, lib::Status> {
    Geom geom = spec.geom;
    double mass = 0.0;
    if (spec.fromto) {
      if (geom.type != GeomType::CAPSULE && geom.type != GeomType::CYLINDER &&
          geom.type != GeomType::ELLIPSOID && geom.type != GeomType::BOX) {
        return fail("fromto requires capsule, cylinder, box or ellipsoid");
      }
      if (geom.pos[0] != 0.0 || geom.pos[1] != 0.0 || geom.pos[2] != 0.0) {
        return fail("both pos and fromto defined in geom");
      }
      const std::array<double, 6>& f = *spec.fromto;
      Array3 v{f[0] - f[3], f[1] - f[4], f[2] - f[5]};
      geom.size[1] = normalize(v) / 2;
      if (geom.size[1] < EPS) {
        return fail("fromto points too close in geom");
      }
      if (geom.type == GeomType::ELLIPSOID || geom.type == GeomType::BOX) {
        geom.size[2] = geom.size[1];
        geom.size[1] = geom.size[0];
      }
      geom.pos = {(f[0] + f[3]) / 2, (f[1] + f[4]) / 2, (f[2] + f[5]) / 2};
      geom.quat = rotate_z_to(v);
    } else {
      RETURN_OR_ASSIGN(geom.quat, resolve(spec.orientation));
    }
    if (infer) {
      double volume = compute_volume(geom);
      if (spec.mass) {
        if (*spec.mass != 0.0 && volume > EPS) {
          mass = *spec.mass;
        }
      } else if (spec.density != 0.0) {
        mass = spec.density * volume;
      }
    }
    return std::pair{geom, mass};
  }

  // Every body, depth first, the world first.
  auto compile(const BodySpec& world, Out<ArticulatedModel> model)
      -> std::expected<void, lib::Status> {
    model->bodies.push_back(ArticulatedBody{.name = "world"});
    std::vector<std::uint32_t> last_dof{Dof::NONE};  // By body.
    RETURN_IF_UNEXPECTED(compile_geoms_of(world, 0, model));
    for (const BodySpec& child : world.children) {
      RETURN_IF_UNEXPECTED(compile_body(child, 0, Out(last_dof), model));
    }
    for (MotorSpec& motor : motors_) {
      auto joint = std::ranges::find(model->joints, motor.joint, &Joint::name);
      if (joint == model->joints.end()) {
        return fail("no joint " + motor.joint + " for motor");
      }
      Actuator actuator = motor.actuator;
      actuator.joint =
          static_cast<std::uint32_t>(joint - model->joints.begin());
      bool has_range = !(actuator.control_range[0] == 0.0 &&
                         actuator.control_range[1] == 0.0);
      actuator.control_limited =
          motor.limited == Limited::YES || (motor.limited == Limited::AUTO &&
                                            compiler_.auto_limits && has_range);
      model->actuators.push_back(actuator);
    }
    return {};
  }

  // The world's geoms, which carry no mass.
  auto compile_geoms_of(const BodySpec& body, std::uint32_t index,
                        Out<ArticulatedModel> model)
      -> std::expected<void, lib::Status> {
    model->bodies[index].first_geom =
        static_cast<std::uint32_t>(model->geoms.size());
    for (const GeomSpec& spec : body.geoms) {
      RETURN_OR_ASSIGN(auto compiled, compile_geom(spec, false));
      compiled.first.body = index;
      model->geoms.push_back(compiled.first);
    }
    model->bodies[index].geoms = static_cast<std::uint32_t>(body.geoms.size());
    return {};
  }

  // A body, its inertia, joints and geoms, then its children
  // (mjCBody::Compile, InertiaFromGeom and mjCModel's joint pass).
  auto compile_body(const BodySpec& spec, std::uint32_t parent,
                    Out<std::vector<std::uint32_t>> last_dof,
                    Out<ArticulatedModel> model)
      -> std::expected<void, lib::Status> {
    auto index = static_cast<std::uint32_t>(model->bodies.size());
    ArticulatedBody body{.name = spec.name, .parent = parent, .pos = spec.pos};
    body.root = parent == 0 ? index : model->bodies[parent].root;
    RETURN_OR_ASSIGN(body.quat, resolve(spec.orientation));

    // An explicit inertial, a full tensor turned to the body's axes and
    // found on its principal axes.
    bool explicit_inertial = spec.inertial.has_value();
    bool inertial_defined = false;
    if (explicit_inertial) {
      const InertialSpec& inertial = *spec.inertial;
      body.inertial_pos = inertial.pos;
      body.mass = inertial.mass;
      body.inertia = inertial.diagonal;
      Quaternion4 quat = inertial.orientation.quat;
      normalize(quat);
      if (inertial.full) {
        if (inertial.orientation.kind != Orientation::Kind::QUAT) {
          return fail(
              "fullinertia and inertial orientation cannot both be specified");
        }
        std::array<double, 9> m = convert_to_matrix(quat);
        const std::array<double, 6>& f = *inertial.full;
        std::array<double, 9> full{f[0], f[3], f[4], f[3], f[1],
                                   f[5], f[4], f[5], f[2]};
        std::array<double, 9> turned =
            multiply_matrices(multiply_matrices(m, full), transpose(m));
        auto [moments, axes] = find_principal_axes(
            {turned[0], turned[4], turned[8], turned[1], turned[2], turned[5]});
        if (moments[2] < EPS) {
          return fail("inertia must have positive eigenvalues");
        }
        body.inertia = moments;
        quat = axes;
      } else if (inertial.orientation.kind != Orientation::Kind::QUAT) {
        RETURN_OR_ASSIGN(quat, resolve(inertial.orientation));
      }
      body.inertial_quat = quat;
      inertial_defined = true;
    }

    // Geoms, and their masses where the body's inertia comes from them.
    bool infer = !explicit_inertial ||
                 compiler_.from_geom == CompilerSpec::FromGeom::YES;
    std::vector<std::pair<Geom, double>> geoms;
    for (const GeomSpec& geom_spec : spec.geoms) {
      bool in_group = geom_spec.group >= compiler_.inertia_groups[0] &&
                      geom_spec.group <= compiler_.inertia_groups[1];
      RETURN_OR_ASSIGN(auto compiled,
                       compile_geom(geom_spec, infer && in_group));
      geoms.push_back(compiled);
    }
    if (compiler_.from_geom == CompilerSpec::FromGeom::YES ||
        (!inertial_defined &&
         compiler_.from_geom == CompilerSpec::FromGeom::AUTO)) {
      std::vector<std::size_t> massive;
      for (std::size_t i = 0; i < geoms.size(); ++i) {
        if (geoms[i].second > EPS) {
          massive.push_back(i);
        }
      }
      if (massive.size() == 1) {
        const auto& [geom, mass] = geoms[massive[0]];
        body.inertial_pos = geom.pos;
        body.inertial_quat = geom.quat;
        body.mass = mass;
        body.inertia = compute_inertia(geom, mass);
        inertial_defined = true;
      } else if (massive.size() > 1) {
        double total = 0.0;
        Array3 center{};
        for (std::size_t i : massive) {
          const auto& [geom, mass] = geoms[i];
          total += mass;
          center[0] += mass * geom.pos[0];
          center[1] += mass * geom.pos[1];
          center[2] += mass * geom.pos[2];
        }
        if (total < EPS) {
          return fail("body mass is too small, cannot compute center of mass");
        }
        body.inertial_pos = {center[0] / total, center[1] / total,
                             center[2] / total};
        std::array<double, 6> tensor{};
        for (std::size_t i : massive) {
          const auto& [geom, mass] = geoms[i];
          Array3 d{geom.pos[0] - body.inertial_pos[0],
                    geom.pos[1] - body.inertial_pos[1],
                    geom.pos[2] - body.inertial_pos[2]};
          std::array<double, 6> own =
              turn_inertia(compute_inertia(geom, mass), geom.quat);
          std::array<double, 6> shift = shift_inertia(mass, d);
          for (std::size_t j = 0; j < 6; ++j) {
            tensor[j] = tensor[j] + own[j] + shift[j];
          }
        }
        auto [moments, axes] = find_principal_axes(tensor);
        if (moments[2] < EPS) {
          return fail("inertia must have positive eigenvalues");
        }
        body.mass = total;
        body.inertia = moments;
        body.inertial_quat = axes;
        inertial_defined = true;
      }
    }
    if (!inertial_defined) {
      body.inertial_pos = body.pos;
      body.inertial_quat = body.quat;
    }
    body.mass = std::max(body.mass, compiler_.bound_mass);
    for (double& moment : body.inertia) {
      moment = std::max(moment, compiler_.bound_inertia);
    }
    const Array3& i = body.inertia;
    if (body.mass < 0 || i[0] < 0 || i[1] < 0 || i[2] < 0) {
      return fail("mass and inertia cannot be negative in body " + spec.name);
    }
    if (i[0] + i[1] < i[2] || i[0] + i[2] < i[1] || i[1] + i[2] < i[0]) {
      if (!compiler_.balance_inertia) {
        return fail("inertia must satisfy A + B >= C in body " + spec.name);
      }
      double mean = (i[0] + i[1] + i[2]) / 3.0;
      body.inertia = {mean, mean, mean};
    }

    // Joints and their degrees of freedom, each on the one before, the
    // first on the parent's last.
    body.first_joint = static_cast<std::uint32_t>(model->joints.size());
    body.first_dof = static_cast<std::uint32_t>(model->dofs.size());
    last_dof->push_back((*last_dof)[parent]);
    bool ball = false;
    for (const JointSpec& joint_spec : spec.joints) {
      Joint joint = joint_spec.joint;
      joint.body = index;
      if ((joint.type == JointType::BALL || joint.type == JointType::HINGE) &&
          ball) {
        return fail("ball followed by rotation in body " + spec.name);
      }
      ball = ball || joint.type == JointType::BALL;
      if (joint.type == JointType::FREE) {
        joint.limited = false;
      } else {
        bool has_range = !(joint.range[0] == 0.0 && joint.range[1] == 0.0);
        if (joint_spec.limited == Limited::AUTO) {
          if (has_range && !compiler_.auto_limits) {
            return fail(
                "joint has range but limited is auto and autolimits is off");
          }
          joint.limited = has_range;
        } else {
          joint.limited = joint_spec.limited == Limited::YES;
        }
      }
      if (joint.limited) {
        if (joint.range[0] >= joint.range[1] && joint.type != JointType::BALL) {
          return fail("range[0] should be smaller than range[1] in joint " +
                      joint.name);
        }
        if (joint.range[0] != 0.0 && joint.type == JointType::BALL) {
          return fail("range[0] should be 0 in ball joint " + joint.name);
        }
        if (compiler_.degree &&
            (joint.type == JointType::HINGE || joint.type == JointType::BALL)) {
          for (double& end : joint.range) {
            if (end != 0.0) {
              end *= PI / 180.0;
            }
          }
        }
      }
      if (joint.type == JointType::FREE || joint.type == JointType::BALL) {
        joint.axis = {0.0, 0.0, 1.0};
      }
      if (normalize(joint.axis) < EPS) {
        return fail("axis too small in joint " + joint.name);
      }
      if (joint.type == JointType::FREE) {
        joint.pos = {};
      }
      double ref = joint_spec.ref;
      double springref = joint_spec.springref;
      if (joint.type == JointType::HINGE && compiler_.degree) {
        ref *= PI / 180.0;
        springref *= PI / 180.0;
      }
      joint.qpos = static_cast<std::uint32_t>(model->qpos0.size());
      joint.dof = static_cast<std::uint32_t>(model->dofs.size());
      switch (joint.type) {
        case JointType::FREE:
          for (double x : body.pos) {
            model->qpos0.push_back(x);
          }
          for (double x : body.quat) {
            model->qpos0.push_back(x);
          }
          for (std::size_t k = model->qpos0.size() - 7; k < model->qpos0.size();
               ++k) {
            model->qpos_spring.push_back(model->qpos0[k]);
          }
          break;
        case JointType::BALL:
          for (double x : {1.0, 0.0, 0.0, 0.0}) {
            model->qpos0.push_back(x);
            model->qpos_spring.push_back(x);
          }
          break;
        default:
          model->qpos0.push_back(ref);
          model->qpos_spring.push_back(springref);
          break;
      }
      int count = joint.type == JointType::FREE   ? 6
                  : joint.type == JointType::BALL ? 3
                                                  : 1;
      for (int k = 0; k < count; ++k) {
        model->dofs.push_back(
            Dof{.body = index,
                .joint = static_cast<std::uint32_t>(model->joints.size()),
                .parent = last_dof->back(),
                .armature = joint_spec.armature,
                .damping = joint_spec.damping,
                .friction_loss = joint_spec.friction_loss});
        last_dof->back() = static_cast<std::uint32_t>(model->dofs.size() - 1);
      }
      model->joints.push_back(joint);
    }
    body.joints = static_cast<std::uint32_t>(spec.joints.size());
    body.dofs = static_cast<std::uint32_t>(model->dofs.size()) - body.first_dof;
    if (body.dofs > 6) {
      return fail("more than 6 dofs in body " + spec.name);
    }

    body.first_geom = static_cast<std::uint32_t>(model->geoms.size());
    body.geoms = static_cast<std::uint32_t>(geoms.size());
    for (auto& [geom, mass] : geoms) {
      if (geom.type == GeomType::PLANE) {
        return fail("plane only allowed in static bodies");
      }
      geom.body = index;
      model->geoms.push_back(geom);
    }
    model->bodies.push_back(body);
    for (const BodySpec& child : spec.children) {
      RETURN_IF_UNEXPECTED(compile_body(child, index, last_dof, model));
    }
    return {};
  }

  CompilerSpec compiler_;
  std::map<std::string, Defaults> defaults_;
  std::vector<MotorSpec> motors_;
};

}  // namespace

auto parse_mjcf(std::string_view text)
    -> std::expected<model::ArticulatedModel, lib::Status> {
  return Reader{}.read(text);
}

auto load_mjcf(const std::string& path)
    -> std::expected<model::ArticulatedModel, lib::Status> {
  std::ifstream file{path};
  if (!file) {
    return std::unexpected(
        lib::raise(MjcfError::UNREADABLE, "cannot open " + path));
  }
  std::stringstream text;
  text << file.rdbuf();
  return parse_mjcf(text.str());
}

}  // namespace simon::format
