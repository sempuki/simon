// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "base/testing.hpp"
#include "format/mjcf.hpp"
#include "model/articulated.hpp"

// robotic's test models compiled by simon and by MuJoCo, field by field,
// against the table reference/mujoco_models.py recorded: every body's frame,
// inertial frame, mass and inertia, every joint, degree of freedom, geom and
// actuator, the options, and the positions at rest.
namespace simon::robotic {

namespace {

using model::ArticulatedModel;

constexpr std::string_view MODELS = "application/robotic/models/";

struct Row final {
  std::string model;
  std::string element;
  std::size_t index = 0;
  std::string field;
  std::vector<double> values;
};

auto load_rows() -> std::vector<Row> {
  std::ifstream file{"application/robotic/reference/mujoco_models.csv"};
  REQUIRE(file);
  std::vector<Row> rows;
  std::string line;
  std::getline(file, line);
  while (std::getline(file, line)) {
    std::vector<std::string> cells;
    std::stringstream stream{line};
    std::string cell;
    while (std::getline(stream, cell, ',')) {
      cells.push_back(cell);
    }
    REQUIRE(cells.size() == 5);
    Row row{.model = cells[0],
            .element = cells[1],
            .index = std::stoul(cells[2]),
            .field = cells[3]};
    std::stringstream numbers{cells[4]};
    double value = 0.0;
    while (numbers >> value) {
      row.values.push_back(value);
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

template <typename Array>
auto doubles(const Array& values) -> std::vector<double> {
  return {values.begin(), values.end()};
}

auto number(double value) -> std::vector<double> { return {value}; }

// simon's value for one row of the table, by MuJoCo's names.
auto find_value(const ArticulatedModel& m, const Row& row)
    -> std::vector<double> {
  const std::string& f = row.field;
  std::size_t i = row.index;
  if (row.element == "option") {
    const model::Physics& p = m.physics;
    if (f == "timestep") return number(p.timestep);
    if (f == "gravity") return doubles(p.gravity);
    if (f == "integrator") return number(static_cast<double>(p.integrator));
    if (f == "cone") return number(static_cast<double>(p.cone));
    if (f == "solver") return number(static_cast<double>(p.solver));
    if (f == "iterations") return number(p.iterations);
    if (f == "tolerance") return number(p.tolerance);
  } else if (row.element == "qpos0") {
    return m.qpos0;
  } else if (row.element == "qpos_spring") {
    return m.qpos_spring;
  } else if (row.element == "body") {
    const model::ArticulatedBody& b = m.bodies.at(i);
    if (f == "parentid") return number(b.parent);
    if (f == "rootid") return number(b.root);
    if (f == "pos") return doubles(b.pos);
    if (f == "quat") return doubles(b.quat);
    if (f == "ipos") return doubles(b.inertial_pos);
    if (f == "iquat") return doubles(b.inertial_quat);
    if (f == "mass") return number(b.mass);
    if (f == "inertia") return doubles(b.inertia);
    if (f == "jntadr") return number(b.joints ? b.first_joint : -1.0);
    if (f == "jntnum") return number(b.joints);
    if (f == "dofadr") return number(b.dofs ? b.first_dof : -1.0);
    if (f == "dofnum") return number(b.dofs);
    if (f == "geomadr") return number(b.geoms ? b.first_geom : -1.0);
    if (f == "geomnum") return number(b.geoms);
  } else if (row.element == "jnt") {
    const model::Joint& j = m.joints.at(i);
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
    const model::Dof& d = m.dofs.at(i);
    if (f == "bodyid") return number(d.body);
    if (f == "jntid") return number(d.joint);
    if (f == "parentid") {
      return number(d.parent == model::Dof::NONE ? -1.0 : d.parent);
    }
    if (f == "armature") return number(d.armature);
    if (f == "damping") return number(d.damping);
    if (f == "frictionloss") return number(d.friction_loss);
  } else if (row.element == "geom") {
    const model::Geom& g = m.geoms.at(i);
    if (f == "type") return number(static_cast<double>(g.type));
    if (f == "bodyid") return number(g.body);
    if (f == "size") return doubles(g.size);
    if (f == "pos") return doubles(g.pos);
    if (f == "quat") return doubles(g.quat);
    if (f == "friction") return doubles(g.friction);
    if (f == "condim") return number(g.condim);
    if (f == "contype") return number(g.contype);
    if (f == "conaffinity") return number(g.conaffinity);
    if (f == "solref") return doubles(g.contact.reference);
    if (f == "solimp") return doubles(g.contact.impedance);
    if (f == "margin") return number(g.margin);
    if (f == "gap") return number(g.gap);
    if (f == "priority") return number(g.priority);
  } else if (row.element == "actuator") {
    const model::Actuator& a = m.actuators.at(i);
    if (f == "trntype") return number(0);  // A joint.
    if (f == "trnid") return {static_cast<double>(a.joint), -1.0};
    if (f == "gear") return doubles(a.gear);
    if (f == "ctrlrange") return doubles(a.control_range);
    if (f == "ctrllimited") return number(a.control_limited ? 1 : 0);
    if (f == "dyntype") return number(0);   // None.
    if (f == "gaintype") return number(0);  // Fixed.
    if (f == "biastype") return number(0);  // None.
    if (f == "gainprm") {
      std::vector<double> gain(10, 0.0);
      gain[0] = 1.0;
      return gain;
    }
    if (f == "biasprm") return std::vector<double>(10, 0.0);
  }
  FAIL("no field " << row.element << "." << f);
  return {};
}

}  // namespace

TEST_CASE("ModelAgainstMuJoCo") {
  SECTION("ShouldCompileAsMuJoCoDoesGivenEveryTestModel") {
    // Every field of every model to rounding; counts and kinds exactly.
    std::map<std::string, ArticulatedModel> models;
    double worst = 0.0;
    std::string where;
    std::size_t compared = 0;
    for (const Row& row : load_rows()) {
      if (!models.contains(row.model)) {
        auto loaded = format::load_mjcf(std::string{MODELS} + row.model);
        if (!loaded) {
          FAIL(row.model << ": " << loaded.error().message());
        }
        models.emplace(row.model, std::move(*loaded));
      }
      std::vector<double> ours = find_value(models.at(row.model), row);
      CAPTURE(row.model, row.element, row.index, row.field);
      REQUIRE(ours.size() == row.values.size());
      for (std::size_t k = 0; k < ours.size(); ++k) {
        double apart = std::abs(ours[k] - row.values[k]) /
                       std::max(1.0, std::abs(row.values[k]));
        if (apart > worst) {
          worst = apart;
          where = row.model + " " + row.element + " " +
                  std::to_string(row.index) + " " + row.field;
        }
      }
      ++compared;
    }
    CAPTURE(compared, worst, where);
    CHECK(compared > 1000);
    CHECK(worst < 1e-14);
  }
}

TEST_CASE("Mjcf") {
  SECTION("ShouldRefuseWhatItDoesNotRun") {
    // Tendons, meshes and actuators other than motors change how a model
    // moves, and are refused with their names; MuJoCo's humanoid, for its
    // contact exclusions, the first of those and its tendons.
    auto refused = [](std::string_view text) {
      auto read = format::parse_mjcf(text);
      REQUIRE(!read.has_value());
      return read.error().message();
    };
    CHECK(refused(R"(<mujoco><tendon/></mujoco>)").find("tendon") !=
          std::string::npos);
    CHECK(refused(R"(<mujoco><worldbody><body><freejoint/>
            <geom type="mesh" mesh="m"/></body></worldbody></mujoco>)")
              .find("mesh") != std::string::npos);
    CHECK(refused(R"(<mujoco><actuator><position joint="j"/></actuator>
            </mujoco>)")
              .find("position") != std::string::npos);
    auto humanoid = format::load_mjcf("3rd_party/mujoco/humanoid.xml");
    REQUIRE(!humanoid.has_value());
    CHECK(humanoid.error().message().find("contact") != std::string::npos);
  }

  SECTION("ShouldNotReadGivenMalformedModel") {
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
