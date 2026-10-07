// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows MuJoCo 3.14.0, Copyright 2021 DeepMind Technologies Limited,
// Apache-2.0; translated to C++ and changed. See NOTICE.md.

#include "format/mjcf.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <numbers>
#include <optional>
#include <span>
#include <sstream>
#include <utility>
#include <vector>

#include "Eigen/Eigenvalues"
#include "base/core.hpp"
#include "format/text.hpp"
#include "format/xml.hpp"
#include "pugixml.hpp"

namespace simon::format {

namespace {

using articulated::Actuator;
using articulated::ArticulatedBody;
using articulated::ArticulatedModel;
using articulated::Dof;
using articulated::Geom;
using articulated::GeomType;
using articulated::Joint;
using articulated::JointType;
using articulated::Physics;
using articulated::SameFrame;
using articulated::SoftConstraint;
using articulated::Tendon;

using lib::InOut;
using lib::Out;

constexpr double PI = std::numbers::pi;
constexpr double EPS = 1e-14;  // MuJoCo's mjEPS.

//-- The compiler's geometry ---------------------------------------------------
//
// After MuJoCo 3.14.0's user_util.cc and user_objects.cc (Apache-2.0).

// Normalizes `v` and returns its length; one shorter than EPS has no
// direction, and is left as it was with length 0 (mjuu_normvec).
auto normalize(InOut<Vector3> v) -> double {
  double length = v->norm();
  if (length < EPS) {
    return 0.0;
  }
  *v /= length;
  return length;
}

// The quaternion of the numbers `q`, w, x, y, z, normalized.
auto to_quaternion(const std::array<double, 4>& q) -> Quaternion {
  return Quaternion{q[0], q[1], q[2], q[3]}.normalized();
}

// The least rotation that takes z to `v`, a unit vector; about x when `v`
// is -z (mjuu_z2quat).
auto rotate_z_to(const Vector3& v) -> Quaternion {
  Vector3 axis = Vector3::UnitZ().cross(v);
  double s = axis.norm();
  axis = s < 1e-10 ? Vector3::UnitX() : Vector3{axis / s};
  return Quaternion{AngleAxis{std::atan2(s, v.z()), axis}};
}

// The rotation whose columns are `x`, `y` and `z` (mjuu_frame2quat).
auto convert_frame(const Vector3& x, const Vector3& y, const Vector3& z)
    -> Quaternion {
  Matrix3 m;
  m << x, y, z;
  return Quaternion{m}.normalized();
}

// A body's inertia about its center of mass from a geom's: its principal
// moments `local` on axes turned by `q`, and its `mass` at `offset` from the
// center (mjuu_globalinertia, mjuu_offcenter).
auto place_inertia(const Vector3& local, const Quaternion& q, double mass,
                   const Vector3& offset) -> Matrix3 {
  Matrix3 r = q.toRotationMatrix();
  return r * local.asDiagonal() * r.transpose() +
         mass * (offset.squaredNorm() * Matrix3::Identity() -
                 offset * offset.transpose());
}

// The principal moments of a symmetric inertia, largest first, and the
// rotation to their axes (mjuu_fullInertia, mjuu_eig3). Each axis is
// defined only up to its sign, so the rotation is one of several.
auto find_principal_axes(const Matrix3& inertia)
    -> std::pair<Vector3, Quaternion> {
  Eigen::SelfAdjointEigenSolver<Matrix3> solver{inertia};
  Vector3 moments = solver.eigenvalues().reverse();
  Matrix3 axes = solver.eigenvectors().rowwise().reverse();
  if (axes.determinant() < 0) {
    axes.col(2) = -axes.col(2);
  }
  return {moments, Quaternion{axes}.normalized()};
}

//-- Specifications, as read ---------------------------------------------------

// A frame's orientation, as one of the ways MJCF gives it.
struct Orientation final {
  enum class Kind : std::uint8_t { QUAT, AXIS_ANGLE, XY_AXES, Z_AXIS, EULER };

  Kind kind = Kind::QUAT;
  std::array<double, 4> quat{1.0, 0.0, 0.0, 0.0};  // w, x, y, z.
  std::array<double, 4> axis_angle{};
  std::array<double, 6> xy_axes{};
  Vector3 z_axis = Vector3::Zero();
  Vector3 euler = Vector3::Zero();
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

struct TendonSpec final {
  Tendon tendon;
  std::vector<std::string> joints;
  Limited limited = Limited::AUTO;
};

struct ActuatorSpec final {
  Actuator actuator;
  std::string joint;
  Limited control_limited = Limited::AUTO;
  Limited force_limited = Limited::AUTO;
};

// What a default class gives each element.
struct Defaults final {
  GeomSpec geom;
  JointSpec joint;
  ActuatorSpec actuator;
};

struct InertialSpec final {
  Vector3 pos = Vector3::Zero();
  Orientation orientation;
  double mass = 0.0;
  Vector3 diagonal = Vector3::Zero();
  std::optional<std::array<double, 6>> full;
};

struct BodySpec final {
  std::string name;
  Vector3 pos = Vector3::Zero();
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

// Reads one document, saying on which line anything in the text is wrong,
// and compiles it, naming the body or joint anything in the model is wrong
// in.
class Reader final {
 public:
  auto read(std::string_view text)
      -> std::expected<ArticulatedModel, lib::Status> {
    RETURN_IF_UNEXPECTED(document_.load(std::string{text}));
    RETURN_OR_ASSIGN(pugi::xml_node root, document_.find_root("mujoco"));
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
      } else if (kind == "tendon") {
        for (pugi::xml_node tendon : node.children()) {
          RETURN_IF_UNEXPECTED(read_tendon(tendon));
        }
      } else if (kind == "contact") {
        for (pugi::xml_node contact : node.children()) {
          if (std::string_view{contact.name()} != "exclude") {
            return refuse(contact);
          }
          for (pugi::xml_attribute attribute : contact.attributes()) {
            std::string_view name = attribute.name();
            if (name != "name" && name != "body1" && name != "body2") {
              return refuse(contact, name);
            }
          }
          excludes_.push_back({contact.attribute("body1").as_string(),
                               contact.attribute("body2").as_string()});
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
  // Something wrong in the text, at `node`.
  auto fail(pugi::xml_node node, std::string_view why) const -> Failure {
    return document_.fail(node, why);
  }
  auto refuse(pugi::xml_node node, std::string_view what = {}) const
      -> Failure {
    return document_.refuse(node, what);
  }

  // Something wrong in the compiled model, which the message names.
  static auto fail(std::string_view why) -> Failure {
    return Failure{lib::raise(FormatError::MALFORMED, std::string{why})};
  }
  static auto refuse(std::string_view what) -> Failure {
    return format::refuse(what);
  }

  // Up to `into.size()` numbers from the attribute, if it is there, the
  // rest left as they were, as MuJoCo reads a partial vector.
  auto read_numbers(pugi::xml_node node, std::string_view name,
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
        return fail(node, std::string{name} + " has too many numbers");
      }
      std::optional<double> value = parse_number(word);
      if (!value) {
        return fail(node, std::string{name} + " `" + std::string{word} +
                              "` is not a finite number");
      }
      into[count++] = *value;
      text.remove_prefix(end == std::string_view::npos ? text.size() : end);
    }
    if (count == 0) {
      return fail(node, std::string{name} + " is empty");
    }
    return true;
  }

  template <std::size_t N>
  auto read_array(pugi::xml_node node, std::string_view name,
                  InOut<std::array<double, N>> into) const
      -> std::expected<bool, lib::Status> {
    return read_numbers(node, name, *into);
  }

  auto read_array(pugi::xml_node node, std::string_view name,
                  InOut<Vector3> into) const
      -> std::expected<bool, lib::Status> {
    return read_numbers(node, name, std::span<double>{into->data(), 3});
  }

  auto read_number(pugi::xml_node node, std::string_view name,
                   InOut<double> into) const
      -> std::expected<bool, lib::Status> {
    return read_numbers(node, name, std::span<double>{&*into, 1});
  }

  template <typename Integer>
  auto read_integer(pugi::xml_node node, std::string_view name,
                    InOut<Integer> into) const
      -> std::expected<void, lib::Status> {
    double value = 0.0;
    RETURN_OR_ASSIGN(bool read, read_number(node, name, InOut(value)));
    if (read) {
      *into = static_cast<Integer>(value);
    }
    return {};
  }

  auto read_flag(pugi::xml_node node, std::string_view name,
                 InOut<bool> into) const -> std::expected<void, lib::Status> {
    pugi::xml_attribute attribute = node.attribute(name);
    if (!attribute) {
      return {};
    }
    std::string_view text = attribute.as_string();
    if (text != "true" && text != "false") {
      return fail(node, std::string{name} + " must be true or false");
    }
    *into = text == "true";
    return {};
  }

  auto read_limited(pugi::xml_node node, std::string_view name,
                    InOut<Limited> into) const
      -> std::expected<void, lib::Status> {
    pugi::xml_attribute attribute = node.attribute(name);
    if (!attribute) {
      return {};
    }
    std::string_view text = attribute.as_string();
    *into = text == "true"    ? Limited::YES
            : text == "false" ? Limited::NO
                              : Limited::AUTO;
    return {};
  }

  // The one orientation the element gives, if any.
  auto read_orientation(pugi::xml_node node, InOut<Orientation> into) const
      -> std::expected<void, lib::Status> {
    int given = 0;
    std::array<double, 4> quat = into->quat;
    RETURN_OR_ASSIGN(bool has_quat, read_array(node, "quat", InOut(quat)));
    if (has_quat) {
      into->kind = Orientation::Kind::QUAT;
      into->quat = quat;
      ++given;
    }
    RETURN_OR_ASSIGN(bool has_axis,
                     read_array(node, "axisangle", InOut(into->axis_angle)));
    if (has_axis) {
      into->kind = Orientation::Kind::AXIS_ANGLE;
      ++given;
    }
    RETURN_OR_ASSIGN(bool has_xy,
                     read_array(node, "xyaxes", InOut(into->xy_axes)));
    if (has_xy) {
      into->kind = Orientation::Kind::XY_AXES;
      ++given;
    }
    RETURN_OR_ASSIGN(bool has_z,
                     read_array(node, "zaxis", InOut(into->z_axis)));
    if (has_z) {
      into->kind = Orientation::Kind::Z_AXIS;
      ++given;
    }
    RETURN_OR_ASSIGN(bool has_euler,
                     read_array(node, "euler", InOut(into->euler)));
    if (has_euler) {
      into->kind = Orientation::Kind::EULER;
      ++given;
    }
    if (given > 1) {
      return fail(node, "gives more than one orientation");
    }
    return {};
  }

  // The quaternion an orientation gives (ResolveOrientation).
  auto resolve(const Orientation& orientation) const
      -> std::expected<Quaternion, lib::Status> {
    double degree = compiler_.degree ? PI / 180.0 : 1.0;
    switch (orientation.kind) {
      case Orientation::Kind::QUAT:
        return to_quaternion(orientation.quat);
      case Orientation::Kind::AXIS_ANGLE: {
        const std::array<double, 4>& a = orientation.axis_angle;
        Vector3 axis{a[0], a[1], a[2]};
        if (normalize(InOut(axis)) < EPS) {
          return fail("axisangle too small");
        }
        return Quaternion{AngleAxis{a[3] * degree, axis}};
      }
      case Orientation::Kind::XY_AXES: {
        const std::array<double, 6>& xy = orientation.xy_axes;
        Vector3 x{xy[0], xy[1], xy[2]};
        if (normalize(InOut(x)) < EPS) {
          return fail("xaxis too small");
        }
        Vector3 y{xy[3], xy[4], xy[5]};
        y -= x * x.dot(y);
        if (normalize(InOut(y)) < EPS) {
          return fail("yaxis too small");
        }
        Vector3 z = x.cross(y);
        if (normalize(InOut(z)) < EPS) {
          return fail("cross(xaxis, yaxis) too small");
        }
        return convert_frame(x, y, z);
      }
      case Orientation::Kind::Z_AXIS: {
        Vector3 z = orientation.z_axis;
        if (normalize(InOut(z)) < EPS) {
          return fail("zaxis too small");
        }
        return rotate_z_to(z);
      }
      case Orientation::Kind::EULER: {
        Quaternion q = Quaternion::Identity();
        for (int i = 0; i < 3; ++i) {
          char axis = compiler_.euler_sequence[static_cast<std::size_t>(i)];
          char lower = static_cast<char>(std::tolower(axis));
          Quaternion turn{AngleAxis{orientation.euler[i] * degree,
                                    Vector3::Unit(lower == 'x'   ? 0
                                                  : lower == 'y' ? 1
                                                                 : 2)}};
          // Moving axes post-multiply, fixed axes pre-multiply.
          q = std::islower(axis) != 0 ? q * turn : turn * q;
        }
        return q.normalized();
      }
    }
    return Quaternion::Identity();
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
          return fail(node, "eulerseq must be three of x, y, z, X, Y, Z");
        }
        compiler_.euler_sequence = std::string{value};
      } else if (name == "inertiafromgeom") {
        compiler_.from_geom = value == "true"    ? CompilerSpec::FromGeom::YES
                              : value == "false" ? CompilerSpec::FromGeom::NO
                                                 : CompilerSpec::FromGeom::AUTO;
      } else if (name == "autolimits") {
        RETURN_IF_UNEXPECTED(
            read_flag(node, "autolimits", InOut(compiler_.auto_limits)));
      } else if (name == "boundmass") {
        RETURN_IF_UNEXPECTED(
            read_number(node, "boundmass", InOut(compiler_.bound_mass)));
      } else if (name == "boundinertia") {
        RETURN_IF_UNEXPECTED(
            read_number(node, "boundinertia", InOut(compiler_.bound_inertia)));
      } else if (name == "balanceinertia") {
        RETURN_IF_UNEXPECTED(read_flag(node, "balanceinertia",
                                       InOut(compiler_.balance_inertia)));
      } else if (name == "inertiagrouprange") {
        std::array<double, 2> range{0, 5};
        RETURN_IF_UNEXPECTED(
            read_array(node, "inertiagrouprange", InOut(range)));
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
        RETURN_IF_UNEXPECTED(
            read_number(node, "timestep", InOut(physics->timestep)));
      } else if (name == "gravity") {
        RETURN_IF_UNEXPECTED(
            read_array(node, "gravity", InOut(physics->gravity)));
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
          return fail(node, "unknown integrator " + std::string{value});
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
            read_integer(node, "iterations", InOut(physics->iterations)));
      } else if (name == "tolerance") {
        RETURN_IF_UNEXPECTED(
            read_number(node, "tolerance", InOut(physics->tolerance)));
      } else if (name == "impratio") {
        RETURN_IF_UNEXPECTED(
            read_number(node, "impratio", InOut(physics->impratio)));
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
        RETURN_IF_UNEXPECTED(apply_geom(child, InOut(defaults.geom)));
      } else if (kind == "joint") {
        RETURN_IF_UNEXPECTED(apply_joint(child, InOut(defaults.joint)));
      } else if (kind == "motor" || kind == "position" || kind == "velocity" ||
                 kind == "general") {
        RETURN_IF_UNEXPECTED(apply_actuator(child, InOut(defaults.actuator)));
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
        return fail(child, "needs a class, being nested");
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
      return fail(node, "no default class " + name);
    }
    return &found->second;
  }

  //-- Elements ----------------------------------------------------------------

  auto apply_geom(pugi::xml_node node, InOut<GeomSpec> spec) const
      -> std::expected<void, lib::Status> {
    Geom& geom = spec->geom;
    for (pugi::xml_attribute attribute : node.attributes()) {
      std::string_view name = attribute.name();
      static constexpr std::array<std::string_view, 21> KNOWN{
          "name",    "class",       "type",     "size",     "pos",
          "quat",    "axisangle",   "xyaxes",   "zaxis",    "euler",
          "fromto",  "mass",        "density",  "friction", "condim",
          "contype", "conaffinity", "priority", "solref",   "solimp",
          "margin"};
      static constexpr std::array<std::string_view, 5> SHOWN{
          "rgba", "material", "group", "user", "mesh"};
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
      static constexpr std::array<std::pair<std::string_view, GeomType>, 7>
          TYPES{{{"plane", GeomType::PLANE},
                 {"sphere", GeomType::SPHERE},
                 {"capsule", GeomType::CAPSULE},
                 {"ellipsoid", GeomType::ELLIPSOID},
                 {"cylinder", GeomType::CYLINDER},
                 {"box", GeomType::BOX},
                 {"mesh", GeomType::MESH}}};
      auto found = std::ranges::find(TYPES, std::string_view{type.as_string()},
                                     &decltype(TYPES)::value_type::first);
      if (found == TYPES.end()) {
        return refuse(node, std::string{"type "} + type.as_string());
      }
      geom.type = found->second;
    }
    RETURN_IF_UNEXPECTED(read_array(node, "size", InOut(geom.size)));
    RETURN_IF_UNEXPECTED(read_array(node, "pos", InOut(geom.pos)));
    RETURN_IF_UNEXPECTED(read_orientation(node, InOut(spec->orientation)));
    std::array<double, 6> fromto{};
    RETURN_OR_ASSIGN(bool has_fromto,
                     read_array(node, "fromto", InOut(fromto)));
    if (has_fromto) {
      spec->fromto = fromto;
    }
    double mass = 0.0;
    RETURN_OR_ASSIGN(bool has_mass, read_number(node, "mass", InOut(mass)));
    if (has_mass) {
      spec->mass = mass;
    }
    RETURN_IF_UNEXPECTED(read_number(node, "density", InOut(spec->density)));
    RETURN_IF_UNEXPECTED(read_array(node, "friction", InOut(geom.friction)));
    RETURN_IF_UNEXPECTED(read_integer(node, "condim", InOut(geom.condim)));
    RETURN_IF_UNEXPECTED(read_integer(node, "contype", InOut(geom.contype)));
    RETURN_IF_UNEXPECTED(
        read_integer(node, "conaffinity", InOut(geom.conaffinity)));
    RETURN_IF_UNEXPECTED(read_integer(node, "priority", InOut(geom.priority)));
    RETURN_IF_UNEXPECTED(
        read_array(node, "solref", InOut(geom.contact.reference)));
    RETURN_IF_UNEXPECTED(
        read_array(node, "solimp", InOut(geom.contact.impedance)));
    RETURN_IF_UNEXPECTED(read_number(node, "margin", InOut(geom.margin)));
    RETURN_IF_UNEXPECTED(read_number(node, "gap", InOut(geom.gap)));
    RETURN_IF_UNEXPECTED(read_integer(node, "group", InOut(spec->group)));
    return {};
  }

  auto apply_joint(pugi::xml_node node, InOut<JointSpec> spec) const
      -> std::expected<void, lib::Status> {
    Joint& joint = spec->joint;
    for (pugi::xml_attribute attribute : node.attributes()) {
      std::string_view name = attribute.name();
      static constexpr std::array<std::string_view, 22> KNOWN{
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
          "solreffriction",
          "solimpfriction",
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
        return fail(node, "unknown joint type " + std::string{text});
      }
    }
    RETURN_IF_UNEXPECTED(read_array(node, "pos", InOut(joint.pos)));
    RETURN_IF_UNEXPECTED(read_array(node, "axis", InOut(joint.axis)));
    RETURN_IF_UNEXPECTED(read_array(node, "range", InOut(joint.range)));
    RETURN_IF_UNEXPECTED(read_limited(node, "limited", InOut(spec->limited)));
    RETURN_IF_UNEXPECTED(
        read_number(node, "stiffness", InOut(joint.stiffness)));
    RETURN_IF_UNEXPECTED(
        read_number(node, "springref", InOut(spec->springref)));
    RETURN_IF_UNEXPECTED(read_number(node, "damping", InOut(spec->damping)));
    RETURN_IF_UNEXPECTED(read_number(node, "armature", InOut(spec->armature)));
    RETURN_IF_UNEXPECTED(
        read_number(node, "frictionloss", InOut(spec->friction_loss)));
    RETURN_IF_UNEXPECTED(read_number(node, "ref", InOut(spec->ref)));
    RETURN_IF_UNEXPECTED(read_number(node, "margin", InOut(joint.margin)));
    RETURN_IF_UNEXPECTED(
        read_array(node, "solreflimit", InOut(joint.limit.reference)));
    RETURN_IF_UNEXPECTED(
        read_array(node, "solimplimit", InOut(joint.limit.impedance)));
    RETURN_IF_UNEXPECTED(
        read_array(node, "solreffriction", InOut(joint.friction.reference)));
    RETURN_IF_UNEXPECTED(
        read_array(node, "solimpfriction", InOut(joint.friction.impedance)));
    return {};
  }

  // An actuator's attributes, and its kind's gain and bias, as MuJoCo's
  // mjs_setToMotor, mjs_setToPosition and mjs_setToVelocity set them.
  auto apply_actuator(pugi::xml_node node, InOut<ActuatorSpec> spec) const
      -> std::expected<void, lib::Status> {
    std::string_view kind = node.name();
    Actuator& actuator = spec->actuator;
    for (pugi::xml_attribute attribute : node.attributes()) {
      std::string_view name = attribute.name();
      std::string_view value = attribute.as_string();
      static constexpr std::array<std::string_view, 10> COMMON{
          "name",        "class",      "joint",        "gear",    "ctrlrange",
          "ctrllimited", "forcerange", "forcelimited", "damping", "group"};
      bool known =
          std::ranges::contains(COMMON, name) ||
          (kind == "position" && (name == "kp" || name == "kv")) ||
          (kind == "velocity" && name == "kv") ||
          (kind == "general" &&
           (name == "gainprm" || name == "biasprm" ||
            (name == "gaintype" && value == "fixed") ||
            (name == "biastype" && (value == "none" || value == "affine")) ||
            (name == "dyntype" && value == "none"))) ||
          ((name == "timeconst" || name == "inheritrange") &&
           attribute.as_double() == 0.0);
      if (!known) {
        return refuse(node, name);
      }
    }
    if (node.attribute("name")) {
      actuator.name = node.attribute("name").as_string();
    }
    if (node.attribute("joint")) {
      spec->joint = node.attribute("joint").as_string();
    }
    RETURN_IF_UNEXPECTED(read_array(node, "gear", InOut(actuator.gear)));
    RETURN_IF_UNEXPECTED(
        read_array(node, "ctrlrange", InOut(actuator.control_range)));
    RETURN_IF_UNEXPECTED(
        read_limited(node, "ctrllimited", InOut(spec->control_limited)));
    RETURN_IF_UNEXPECTED(
        read_array(node, "forcerange", InOut(actuator.force_range)));
    RETURN_IF_UNEXPECTED(
        read_limited(node, "forcelimited", InOut(spec->force_limited)));
    RETURN_IF_UNEXPECTED(read_number(node, "damping", InOut(actuator.damping)));
    if (kind == "motor") {
      actuator.gain[0] = 1;
      actuator.bias_type = Actuator::Bias::NONE;
    } else if (kind == "position") {
      double kp = actuator.gain[0];
      RETURN_IF_UNEXPECTED(read_number(node, "kp", InOut(kp)));
      actuator.gain[0] = kp;
      actuator.bias[1] = -kp;
      double kv = 0.0;
      RETURN_OR_ASSIGN(bool has_kv, read_number(node, "kv", InOut(kv)));
      if (has_kv) {
        if (kv < 0) {
          return fail(node, "kv cannot be negative");
        }
        actuator.bias[2] = -kv;
      }
      actuator.bias_type = Actuator::Bias::AFFINE;
    } else if (kind == "velocity") {
      double kv = actuator.gain[0];
      RETURN_IF_UNEXPECTED(read_number(node, "kv", InOut(kv)));
      actuator.bias = {};
      actuator.gain[0] = kv;
      actuator.bias[2] = -kv;
      actuator.bias_type = Actuator::Bias::AFFINE;
    } else {
      std::array<double, 10> gain{};
      std::array<double, 10> bias{};
      std::ranges::copy(actuator.gain, gain.begin());
      std::ranges::copy(actuator.bias, bias.begin());
      RETURN_IF_UNEXPECTED(read_array(node, "gainprm", InOut(gain)));
      RETURN_IF_UNEXPECTED(read_array(node, "biasprm", InOut(bias)));
      if (std::ranges::any_of(gain.begin() + 3, gain.end(),
                              [](double x) { return x != 0.0; }) ||
          std::ranges::any_of(bias.begin() + 3, bias.end(),
                              [](double x) { return x != 0.0; })) {
        return refuse(node, "parameters past the third");
      }
      std::copy_n(gain.begin(), 3, actuator.gain.begin());
      std::copy_n(bias.begin(), 3, actuator.bias.begin());
      if (pugi::xml_attribute type = node.attribute("biastype")) {
        actuator.bias_type = std::string_view{type.as_string()} == "affine"
                                 ? Actuator::Bias::AFFINE
                                 : Actuator::Bias::NONE;
      }
    }
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
    RETURN_IF_UNEXPECTED(read_array(node, "pos", InOut(spec.pos)));
    RETURN_IF_UNEXPECTED(read_orientation(node, InOut(spec.orientation)));
    RETURN_OR_ASSIGN(bool has_mass,
                     read_number(node, "mass", InOut(spec.mass)));
    if (!has_mass) {
      return fail(node, "needs a mass");
    }
    RETURN_OR_ASSIGN(bool diagonal,
                     read_array(node, "diaginertia", InOut(spec.diagonal)));
    std::array<double, 6> full{};
    RETURN_OR_ASSIGN(bool has_full,
                     read_array(node, "fullinertia", InOut(full)));
    if (has_full) {
      spec.full = full;
    }
    if (diagonal == has_full) {
      return fail(node, "needs one of diaginertia and fullinertia");
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
        RETURN_IF_UNEXPECTED(apply_geom(child, InOut(geom)));
        body->geoms.push_back(std::move(geom));
      } else if (kind == "joint" || kind == "freejoint") {
        JointSpec joint;
        if (kind == "joint") {
          RETURN_OR_ASSIGN(const Defaults* defaults,
                           find_defaults(child, classes));
          joint = defaults->joint;
          joint.joint.name.clear();
          RETURN_IF_UNEXPECTED(apply_joint(child, InOut(joint)));
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
        RETURN_IF_UNEXPECTED(read_array(child, "pos", InOut(spec.pos)));
        RETURN_IF_UNEXPECTED(read_orientation(child, InOut(spec.orientation)));
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

  // A fixed tendon and its joints' coefficients; spatial tendons, which
  // wrap around sites and geoms, and tendon springs are refused.
  auto read_tendon(pugi::xml_node node) -> std::expected<void, lib::Status> {
    if (std::string_view{node.name()} != "fixed") {
      return refuse(node);
    }
    for (pugi::xml_attribute attribute : node.attributes()) {
      std::string_view name = attribute.name();
      static constexpr std::array<std::string_view, 15> KNOWN{
          "name",           "limited",     "range",       "margin",
          "frictionloss",   "solreflimit", "solimplimit", "solreffriction",
          "solimpfriction", "group",       "rgba",        "width",
          "material",       "user",        "class"};
      bool spring = (name == "stiffness" || name == "damping") &&
                    attribute.as_double() == 0.0;
      if (!std::ranges::contains(KNOWN, name) && !spring) {
        return refuse(node, name);
      }
    }
    if (node.attribute("class")) {
      return refuse(node, "class");
    }
    TendonSpec spec;
    Tendon& tendon = spec.tendon;
    tendon.name = node.attribute("name").as_string();
    RETURN_IF_UNEXPECTED(read_limited(node, "limited", InOut(spec.limited)));
    RETURN_IF_UNEXPECTED(read_array(node, "range", InOut(tendon.range)));
    RETURN_IF_UNEXPECTED(read_number(node, "margin", InOut(tendon.margin)));
    RETURN_IF_UNEXPECTED(
        read_number(node, "frictionloss", InOut(tendon.friction_loss)));
    RETURN_IF_UNEXPECTED(
        read_array(node, "solreflimit", InOut(tendon.limit.reference)));
    RETURN_IF_UNEXPECTED(
        read_array(node, "solimplimit", InOut(tendon.limit.impedance)));
    RETURN_IF_UNEXPECTED(
        read_array(node, "solreffriction", InOut(tendon.friction.reference)));
    RETURN_IF_UNEXPECTED(
        read_array(node, "solimpfriction", InOut(tendon.friction.impedance)));
    for (pugi::xml_node joint : node.children()) {
      if (std::string_view{joint.name()} != "joint") {
        return refuse(joint);
      }
      spec.joints.emplace_back(joint.attribute("joint").as_string());
      double coefficient = 1.0;
      RETURN_IF_UNEXPECTED(read_number(joint, "coef", InOut(coefficient)));
      tendon.coefficients.push_back(coefficient);
    }
    tendons_.push_back(std::move(spec));
    return {};
  }

  auto read_actuator(pugi::xml_node node) -> std::expected<void, lib::Status> {
    std::string_view kind = node.name();
    if (kind != "motor" && kind != "position" && kind != "velocity" &&
        kind != "general") {
      return refuse(node);
    }
    RETURN_OR_ASSIGN(const Defaults* defaults, find_defaults(node, "main"));
    ActuatorSpec actuator = defaults->actuator;
    actuator.actuator.name.clear();
    actuator.joint.clear();
    RETURN_IF_UNEXPECTED(apply_actuator(node, InOut(actuator)));
    if (actuator.joint.empty()) {
      return fail(node, "needs a joint");
    }
    actuators_.push_back(std::move(actuator));
    return {};
  }

  //-- Compiling ---------------------------------------------------------------

  // A geom's mass and principal moments from its shape and density
  // (mjCGeom::GetVolume and SetInertia, solid shapes).
  static auto compute_volume(const Geom& geom) -> double {
    const Vector3& s = geom.size;
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

  static auto compute_inertia(const Geom& geom, double mass) -> Vector3 {
    const Vector3& s = geom.size;
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
      Vector3 v{f[0] - f[3], f[1] - f[4], f[2] - f[5]};
      geom.size[1] = normalize(InOut(v)) / 2;
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
    // A mesh only shows the model: it may neither collide nor weigh, as
    // its shape is not read.
    if (geom.type == GeomType::MESH) {
      if (geom.contype != 0 || geom.conaffinity != 0) {
        return refuse("a mesh geom that collides");
      }
      if (infer && (spec.mass ? *spec.mass != 0.0 : spec.density != 0.0)) {
        return refuse("a mesh geom that gives its body mass");
      }
      return std::pair{geom, 0.0};
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
    RETURN_IF_UNEXPECTED(compile_world_geoms(world, 0, model));
    for (const BodySpec& child : world.children) {
      RETURN_IF_UNEXPECTED(compile_body(child, 0, Out(last_dof), model));
    }
    auto resolve_limited = [&](Limited limited,
                               const std::array<double, 2>& range) {
      bool has_range = !(range[0] == 0.0 && range[1] == 0.0);
      return limited == Limited::YES ||
             (limited == Limited::AUTO && compiler_.auto_limits && has_range);
    };
    for (ActuatorSpec& spec : actuators_) {
      auto joint = std::ranges::find(model->joints, spec.joint, &Joint::name);
      if (joint == model->joints.end()) {
        return fail("no joint " + spec.joint + " for actuator");
      }
      if (joint->type == JointType::FREE || joint->type == JointType::BALL) {
        return refuse("actuator on a ball or free joint");
      }
      Actuator actuator = spec.actuator;
      actuator.joint =
          static_cast<std::uint32_t>(joint - model->joints.begin());
      actuator.control_limited =
          resolve_limited(spec.control_limited, actuator.control_range);
      actuator.force_limited =
          resolve_limited(spec.force_limited, actuator.force_range);
      model->actuators.push_back(actuator);
    }
    for (TendonSpec& spec : tendons_) {
      Tendon tendon = spec.tendon;
      for (const std::string& name : spec.joints) {
        auto joint = std::ranges::find(model->joints, name, &Joint::name);
        if (joint == model->joints.end()) {
          return fail("no joint " + name + " for tendon " + tendon.name);
        }
        if (joint->type == JointType::FREE || joint->type == JointType::BALL) {
          return fail("tendon " + tendon.name + " on a ball or free joint");
        }
        tendon.joints.push_back(
            static_cast<std::uint32_t>(joint - model->joints.begin()));
      }
      tendon.limited = resolve_limited(spec.limited, tendon.range);
      model->tendons.push_back(std::move(tendon));
    }
    for (const auto& [first, second] : excludes_) {
      auto a = std::ranges::find(model->bodies, first, &ArticulatedBody::name);
      auto b = std::ranges::find(model->bodies, second, &ArticulatedBody::name);
      if (a == model->bodies.end() || b == model->bodies.end()) {
        return fail("no body " + (a == model->bodies.end() ? first : second) +
                    " for a contact exclusion");
      }
      auto i = static_cast<std::uint32_t>(a - model->bodies.begin());
      auto j = static_cast<std::uint32_t>(b - model->bodies.begin());
      model->excludes.push_back({std::min(i, j), std::max(i, j)});
    }
    std::ranges::sort(model->excludes);
    classify_frames(model);
    return {};
  }

  // Which frames MuJoCo treats as one: within 1e-6 of each other, a
  // quaternion either sign (IsSamePose, IsNullPose).
  static auto classify_frames(Out<ArticulatedModel> model) -> void {
    constexpr double EPS = 1e-6;
    auto same_pos = [&](const Vector3& a, const Vector3& b) {
      return std::abs(a[0] - b[0]) < EPS && std::abs(a[1] - b[1]) < EPS &&
             std::abs(a[2] - b[2]) < EPS;
    };
    // Either sign of a quaternion is the same turn.
    auto same_quat = [&](const Quaternion& a, const Quaternion& b) {
      return (a.coeffs() - b.coeffs()).cwiseAbs().maxCoeff() < EPS ||
             (a.coeffs() + b.coeffs()).cwiseAbs().maxCoeff() < EPS;
    };
    const Vector3 zero = Vector3::Zero();
    const Quaternion unit = Quaternion::Identity();
    for (ArticulatedBody& body : model->bodies) {
      body.inertial_frame = same_pos(body.inertial_pos, zero) &&
                                    same_quat(body.inertial_quat, unit)
                                ? SameFrame::BODY
                            : same_quat(body.inertial_quat, unit)
                                ? SameFrame::BODY_ROTATION
                                : SameFrame::NONE;
    }
    for (Geom& geom : model->geoms) {
      const ArticulatedBody& body = model->bodies[geom.body];
      bool rotation = same_quat(geom.quat, unit);
      bool inertial_rotation = same_quat(geom.quat, body.inertial_quat);
      geom.frame = same_pos(geom.pos, zero) && rotation ? SameFrame::BODY
                   : rotation ? SameFrame::BODY_ROTATION
                   : same_pos(geom.pos, body.inertial_pos) && inertial_rotation
                       ? SameFrame::INERTIA
                   : inertial_rotation ? SameFrame::INERTIA_ROTATION
                                       : SameFrame::NONE;
    }
  }

  // The world's geoms, which carry no mass.
  auto compile_world_geoms(const BodySpec& body, std::uint32_t index,
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
      Quaternion quat = to_quaternion(inertial.orientation.quat);
      if (inertial.full) {
        if (inertial.orientation.kind != Orientation::Kind::QUAT) {
          return fail(
              "fullinertia and inertial orientation cannot both be specified");
        }
        Matrix3 m = quat.toRotationMatrix();
        const std::array<double, 6>& f = *inertial.full;
        Matrix3 full;
        full << f[0], f[3], f[4],  //
            f[3], f[1], f[5],      //
            f[4], f[5], f[2];
        auto [moments, axes] = find_principal_axes(m * full * m.transpose());
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
        Vector3 center = Vector3::Zero();
        for (std::size_t i : massive) {
          const auto& [geom, mass] = geoms[i];
          total += mass;
          center += mass * geom.pos;
        }
        if (total < EPS) {
          return fail("body mass is too small, cannot compute center of mass");
        }
        body.inertial_pos = center / total;
        Matrix3 tensor = Matrix3::Zero();
        for (std::size_t i : massive) {
          const auto& [geom, mass] = geoms[i];
          tensor += place_inertia(compute_inertia(geom, mass), geom.quat, mass,
                                  geom.pos - body.inertial_pos);
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
    const Vector3& i = body.inertia;
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
      if (normalize(InOut(joint.axis)) < EPS) {
        return fail("axis too small in joint " + joint.name);
      }
      if (joint.type == JointType::FREE) {
        joint.pos = Vector3::Zero();
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
          for (double x :
               {body.quat.w(), body.quat.x(), body.quat.y(), body.quat.z()}) {
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
    // A plane only on a body welded to the world: no joints on it or on
    // any body above it.
    bool fixed = body.joints == 0;
    for (std::uint32_t b = parent; fixed && b != 0;
         b = model->bodies[b].parent) {
      fixed = model->bodies[b].joints == 0;
    }
    for (auto& [geom, mass] : geoms) {
      if (geom.type == GeomType::PLANE && !fixed) {
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

  XmlDocument document_;
  CompilerSpec compiler_;
  std::map<std::string, Defaults> defaults_;
  std::vector<ActuatorSpec> actuators_;
  std::vector<TendonSpec> tendons_;
  std::vector<std::pair<std::string, std::string>> excludes_;
};

}  // namespace

auto parse_mjcf(std::string_view text)
    -> std::expected<articulated::ArticulatedModel, lib::Status> {
  return Reader{}.read(text);
}

namespace {

// Replaces each include element under `node` by the children of the root of
// the file it names, found beside the model, as MuJoCo's include does;
// included files may include others.
auto expand_includes(pugi::xml_node node, const std::filesystem::path& folder,
                     int depth) -> std::expected<void, lib::Status> {
  if (depth > 16) {
    return Failure{
        lib::raise(FormatError::MALFORMED, "includes nested too deeply")};
  }
  for (pugi::xml_node child = node.first_child(); child;) {
    pugi::xml_node next = child.next_sibling();
    if (std::string_view{child.name()} == "include") {
      std::filesystem::path path = folder / child.attribute("file").as_string();
      pugi::xml_document included;
      if (!included.load_file(path.c_str())) {
        return Failure{lib::raise(FormatError::UNREADABLE,
                                  "cannot read " + path.string())};
      }
      pugi::xml_node root = included.child("mujoco");
      if (!root) {
        return Failure{lib::raise(FormatError::MALFORMED,
                                  "no mujoco element in " + path.string())};
      }
      for (pugi::xml_node part : root.children()) {
        node.insert_copy_before(part, child);
      }
      node.remove_child(child);
      next = node.first_child();  // Expand what came in, too.
    } else {
      RETURN_IF_UNEXPECTED(expand_includes(child, folder, depth + 1));
    }
    child = next;
  }
  return {};
}

}  // namespace

auto load_mjcf(const std::string& path)
    -> std::expected<articulated::ArticulatedModel, lib::Status> {
  pugi::xml_document document;
  if (!document.load_file(path.c_str())) {
    return Failure{lib::raise(FormatError::UNREADABLE, "cannot open " + path)};
  }
  RETURN_IF_UNEXPECTED(
      expand_includes(document, std::filesystem::path{path}.parent_path(), 0));
  std::stringstream text;
  document.save(text);
  return parse_mjcf(text.str());
}

}  // namespace simon::format
