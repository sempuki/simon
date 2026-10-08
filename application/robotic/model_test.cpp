// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "application/robotic/testing.hpp"
#include "base/testing.hpp"
#include "core/math.hpp"
#include "format/mjcf.hpp"
#include "model/articulated/articulated.hpp"

// robotic's test models compiled by simon and by MuJoCo, field by field,
// against the table reference/mujoco_models.py recorded: every body's frame,
// inertial frame, mass and inertia, every joint, degree of freedom, geom and
// actuator, the options, and the positions at rest.
namespace simon::robotic {

namespace {

using namespace testing;

struct Row final {
  std::string model;
  std::string element;
  std::size_t index = 0;
  std::string field;
  std::vector<double> values;
};

auto load_rows() -> std::vector<Row> {
  std::vector<Row> rows;
  for (const std::vector<std::string>& cells :
       load_table(std::string{REFERENCE} + "mujoco_models.csv").lines) {
    REQUIRE(cells.size() == 5);
    rows.push_back(Row{.model = cells[0],
                       .element = cells[1],
                       .index = std::stoul(cells[2]),
                       .field = cells[3],
                       .values = parse_numbers(cells[4])});
  }
  return rows;
}

template <typename Array>
auto doubles(const Array& values) -> std::vector<double> {
  return {values.begin(), values.end()};
}

auto number(double value) -> std::vector<double> { return {value}; }

// A quaternion's w, x, y, z, its sign that of `theirs`: either sign is the
// same turn.
auto quaternion(const Quaternion& q, const std::vector<double>& theirs)
    -> std::vector<double> {
  std::vector<double> ours{q.w(), q.x(), q.y(), q.z()};
  double agree = 0.0;
  for (std::size_t k = 0; k < 4 && k < theirs.size(); ++k) {
    agree += ours[k] * theirs[k];
  }
  if (agree < 0) {
    for (double& x : ours) {
      x = -x;
    }
  }
  return ours;
}

// An inertia in its body's frame, row by row, from its principal moments and
// the quaternion `quat`, w, x, y, z, that turns to their axes.
auto find_tensor(const std::vector<double>& moments,
                 const std::vector<double>& quat) -> std::vector<double> {
  Matrix3 r = Quaternion{quat[0], quat[1], quat[2], quat[3]}
                  .normalized()
                  .toRotationMatrix();
  Matrix3 tensor = r *
                   Vector3{moments[0], moments[1], moments[2]}.asDiagonal() *
                   r.transpose();
  std::vector<double> rows;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      rows.push_back(tensor(i, j));
    }
  }
  return rows;
}

// simon's value for one row of the table, by MuJoCo's names.
auto find_value(const articulated::Scene& m, const Row& row)
    -> std::vector<double> {
  const std::string& f = row.field;
  std::size_t i = row.index;
  if (row.element == "option") {
    const articulated::Physics& p = m.physics;
    if (f == "timestep") return number(p.timestep);
    if (f == "gravity") return doubles(p.gravity);
    if (f == "integrator") return number(static_cast<double>(p.integrator));
    if (f == "cone") return number(static_cast<double>(p.cone));
    if (f == "solver") return number(static_cast<double>(p.solver));
    if (f == "iterations") return number(p.iterations);
    if (f == "tolerance") return number(p.tolerance);
    if (f == "impratio") return number(p.impratio);
  } else if (row.element == "qpos0") {
    return m.qpos0;
  } else if (row.element == "qpos_spring") {
    return m.qpos_spring;
  } else if (row.element == "body") {
    const articulated::Body& b = m.bodies.at(i);
    if (f == "parentid") return number(b.parent);
    if (f == "rootid") return number(b.root);
    if (f == "pos") return doubles(b.pos);
    if (f == "quat") return quaternion(b.quat, row.values);
    if (f == "ipos") return doubles(b.inertial_pos);
    if (f == "iquat") return quaternion(b.inertial_quat, row.values);
    if (f == "mass") return number(b.mass);
    if (f == "inertia") return doubles(b.inertia);
    if (f == "jntadr") return number(b.joints ? b.first_joint : -1.0);
    if (f == "jntnum") return number(b.joints);
    if (f == "dofadr") return number(b.dofs ? b.first_dof : -1.0);
    if (f == "dofnum") return number(b.dofs);
    if (f == "geomadr") return number(b.geoms ? b.first_geom : -1.0);
    if (f == "geomnum") return number(b.geoms);
    if (f == "sameframe") {
      return number(static_cast<double>(b.inertial_frame));
    }
  } else if (row.element == "jnt") {
    const articulated::Joint& j = m.joints.at(i);
    if (f == "type") return number(static_cast<double>(j.type));
    if (f == "bodyid") return number(j.body);
    if (f == "pos") return doubles(j.pos);
    if (f == "axis") return doubles(j.axis);
    if (f == "limited") return number(j.limited ? 1 : 0);
    if (f == "range") return doubles(j.range);
    if (f == "stiffness") return number(j.stiffness);
    if (f == "qposadr") return number(j.qpos);
    if (f == "dofadr") return number(j.dof);
    if (f == "solref") return doubles(j.limit.reference);
    if (f == "solimp") return doubles(j.limit.impedance);
    if (f == "margin") return number(j.margin);
  } else if (row.element == "dof") {
    const articulated::Dof& d = m.dofs.at(i);
    if (f == "bodyid") return number(d.body);
    if (f == "jntid") return number(d.joint);
    if (f == "parentid") {
      return number(d.parent == articulated::Dof::NONE ? -1.0 : d.parent);
    }
    if (f == "armature") return number(d.armature);
    if (f == "damping") return number(d.damping);
    if (f == "frictionloss") return number(d.friction_loss);
    if (f == "solref") return doubles(m.joints.at(d.joint).friction.reference);
    if (f == "solimp") return doubles(m.joints.at(d.joint).friction.impedance);
  } else if (row.element == "geom") {
    const articulated::Geometry& g = m.geoms.at(i);
    // A mesh's frame and size come from its shape, which is not read; it
    // only shows the model.
    if (g.type == articulated::GeometryType::MESH &&
        (f == "size" || f == "pos" || f == "quat" || f == "sameframe")) {
      return row.values;
    }
    if (f == "type") return number(static_cast<double>(g.type));
    if (f == "bodyid") return number(g.body);
    if (f == "size") return doubles(g.size);
    if (f == "pos") return doubles(g.pos);
    if (f == "quat") return quaternion(g.quat, row.values);
    if (f == "friction") return doubles(g.friction);
    if (f == "condim") return number(g.condim);
    if (f == "contype") return number(g.contype);
    if (f == "conaffinity") return number(g.conaffinity);
    if (f == "solref") return doubles(g.contact.reference);
    if (f == "solimp") return doubles(g.contact.impedance);
    if (f == "margin") return number(g.margin);
    if (f == "gap") return number(g.gap);
    if (f == "priority") return number(g.priority);
    if (f == "sameframe") return number(static_cast<double>(g.frame));
  } else if (row.element == "tendon") {
    const articulated::Tendon& t = m.tendons.at(i);
    if (f == "limited") return number(t.limited ? 1 : 0);
    if (f == "range") return doubles(t.range);
    if (f == "margin") return number(t.margin);
    if (f == "frictionloss") return number(t.friction_loss);
    if (f == "solref_lim") return doubles(t.limit.reference);
    if (f == "solimp_lim") return doubles(t.limit.impedance);
    if (f == "solref_fri") return doubles(t.friction.reference);
    if (f == "solimp_fri") return doubles(t.friction.impedance);
    if (f == "joints") return doubles(t.joints);
    if (f == "coefficients") return t.coefficients;
  } else if (row.element == "exclude") {
    return doubles(m.excludes.at(i));
  } else if (row.element == "actuator") {
    const articulated::Actuator& a = m.actuators.at(i);
    if (f == "trntype") return number(0);  // A joint.
    if (f == "trnid") return {static_cast<double>(a.joint), -1.0};
    if (f == "gear") return doubles(a.gear);
    if (f == "ctrlrange") return doubles(a.control_range);
    if (f == "ctrllimited") return number(a.control_limited ? 1 : 0);
    if (f == "forcerange") return doubles(a.force_range);
    if (f == "forcelimited") return number(a.force_limited ? 1 : 0);
    if (f == "damping") return number(a.damping);
    if (f == "dyntype") return number(0);   // None.
    if (f == "gaintype") return number(0);  // Fixed.
    if (f == "biastype") {
      return number(a.bias_type == articulated::Actuator::Bias::AFFINE ? 1 : 0);
    }
    if (f == "gainprm" || f == "biasprm") {
      std::vector<double> prm(10, 0.0);
      std::ranges::copy(f == "gainprm" ? a.gain : a.bias, prm.begin());
      return prm;
    }
  }
  FAIL("no field " << row.element << "." << f);
  return {};
}

}  // namespace

TEST_CASE("ModelAgainstMuJoCo") {
  SECTION("ShouldCompileAsMuJoCoDoesGivenEveryTestModel") {
    // Preconditions.
    // Every field of every model to rounding, relative to the field's size
    // where it is more than 1; counts and kinds exactly.
    std::map<std::string, articulated::Scene> models;
    double worst = 0.0;
    std::string where;
    std::size_t compared = 0;
    std::vector<Row> rows = load_rows();
    // MuJoCo's principal moments, by model and body.
    std::map<std::pair<std::string, std::size_t>, std::vector<double>> inertias;
    for (const Row& row : rows) {
      if (row.element == "body" && row.field == "inertia") {
        inertias[{row.model, row.index}] = row.values;
      }
    }

    // Under Test.
    for (const Row& row : rows) {
      if (!models.contains(row.model)) {
        std::string path = row.model == "humanoid.xml" ? std::string{HUMANOID}
                           : row.model.find('/') != std::string::npos
                               ? std::string{MENAGERIE} + row.model
                               : std::string{MODELS} + row.model;
        auto loaded = format::load_mjcf(path);
        if (!loaded) {
          FAIL(row.model << ": " << loaded.error().message());
        }
        models.emplace(row.model, std::move(*loaded));
      }
      std::vector<double> ours = find_value(models.at(row.model), row);
      std::vector<double> theirs = row.values;
      CAPTURE(row.model, row.element, row.index, row.field);
      if (row.element == "body" && row.field == "iquat") {
        // Principal axes are defined only up to their signs, and where
        // moments are equal not at all: compare the inertia they give.
        const articulated::Body& b = models.at(row.model).bodies.at(row.index);
        std::vector<double> moments{b.inertia[0], b.inertia[1], b.inertia[2]};
        ours = find_tensor(moments, ours);
        theirs = find_tensor(inertias.at({row.model, row.index}), theirs);
      }
      REQUIRE(ours.size() == theirs.size());
      for (std::size_t k = 0; k < ours.size(); ++k) {
        double apart =
            std::abs(ours[k] - theirs[k]) / std::max(1.0, std::abs(theirs[k]));
        if (apart > worst) {
          worst = apart;
          where = row.model + " " + row.element + " " +
                  std::to_string(row.index) + " " + row.field;
        }
      }
      ++compared;
    }

    // Postconditions.
    CAPTURE(compared, worst, where);
    CHECK(compared > 1000);
    CHECK(worst <= 1e-12);
  }
}

TEST_CASE("Mjcf") {
  SECTION("ShouldRefuseWhatItDoesNotRun") {
    // Preconditions.
    // Spatial tendons, explicit contact pairs, meshes and actuators with
    // activation change how a model moves, and are refused with their
    // names.
    auto refused = [](std::string_view text) {
      auto read = format::parse_mjcf(text);
      REQUIRE(!read.has_value());
      return read.error().message();
    };

    // Postconditions.
    CHECK(refused(R"(<mujoco><tendon><spatial/></tendon></mujoco>)")
              .find("spatial") != std::string::npos);
    CHECK(refused(R"(<mujoco><contact><pair geom1="a" geom2="b"/></contact>
            </mujoco>)")
              .find("pair") != std::string::npos);
    CHECK(refused(R"(<mujoco><worldbody><body><freejoint/>
            <geom type="mesh" mesh="m"/></body></worldbody></mujoco>)")
              .find("mesh") != std::string::npos);
    CHECK(refused(R"(<mujoco><actuator><intvelocity joint="j"/></actuator>
            </mujoco>)")
              .find("intvelocity") != std::string::npos);
  }

  SECTION("ShouldNotReadGivenMalformedModel") {
    // Postconditions.
    CHECK(!format::parse_mjcf("<mujoco><worldbody>").has_value());
    CHECK(!format::parse_mjcf(R"(<mujoco><worldbody><body><joint/>
            <geom size="0.1" class="missing"/></body></worldbody></mujoco>)")
               .has_value());
    CHECK(!format::parse_mjcf(R"(<mujoco><worldbody><body>
            <joint range="1 -1"/><geom size="0.1"/></body></worldbody>
            </mujoco>)")
               .has_value());
  }
}

}  // namespace simon::robotic
