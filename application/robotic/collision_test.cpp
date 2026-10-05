// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "application/robotic/simulation.hpp"
#include "base/testing.hpp"

// Contacts by simon and by MuJoCo, against the table
// reference/mujoco_contacts.py recorded: every primitive pair, at 400 poses
// drawn at random.
namespace simon::robotic {

namespace {

constexpr std::string_view MODELS = "application/robotic/models/";

auto parse_numbers(const std::string& text) -> std::vector<double> {
  std::vector<double> values;
  std::stringstream stream{text};
  double value = 0.0;
  while (stream >> value) {
    values.push_back(value);
  }
  return values;
}

// A contact as the table holds it: geoms, distance, position, frame,
// dimension, friction, solref, solimp, included margin, excluded.
using Row = std::vector<double>;

auto flatten(const model::Contact& c) -> Row {
  Row row{static_cast<double>(c.geom[0]), static_cast<double>(c.geom[1]),
          c.dist};
  row.insert(row.end(), c.pos.begin(), c.pos.end());
  row.insert(row.end(), c.frame.begin(), c.frame.end());
  row.push_back(static_cast<double>(c.dim));
  row.insert(row.end(), c.friction.begin(), c.friction.end());
  row.insert(row.end(), c.soft.reference.begin(), c.soft.reference.end());
  row.insert(row.end(), c.soft.impedance.begin(), c.soft.impedance.end());
  row.push_back(c.include_margin);
  row.push_back(c.exclude ? 1.0 : 0.0);
  return row;
}

// By geoms, then position.
auto sort_rows(InOut<std::vector<Row>> rows) -> void {
  std::ranges::sort(*rows, [](const Row& a, const Row& b) {
    return std::tie(a[0], a[1], a[3], a[4], a[5]) <
           std::tie(b[0], b[1], b[3], b[4], b[5]);
  });
}

struct Pose final {
  std::vector<double> qpos;
  std::vector<Row> contacts;
};

auto load_poses() -> std::vector<Pose> {
  std::ifstream file{"application/robotic/reference/mujoco_contacts.csv"};
  REQUIRE(file);
  std::vector<Pose> poses;
  std::string line;
  std::getline(file, line);
  while (std::getline(file, line)) {
    std::stringstream stream{line};
    std::string index;
    std::string kind;
    std::string values;
    std::getline(stream, index, ',');
    std::getline(stream, kind, ',');
    std::getline(stream, values, ',');
    if (kind == "qpos") {
      poses.push_back(Pose{.qpos = parse_numbers(values)});
    } else {
      poses.back().contacts.push_back(parse_numbers(values));
    }
  }
  return poses;
}

auto step_once(InOut<Simulation> simulation) -> void {
  auto dt = std::chrono::nanoseconds{2'000'000};
  REQUIRE(simulation->step(
      framework::Step{.time = framework::TimePoint{}, .dt = dt}));
}

}  // namespace

TEST_CASE("CollisionAgainstMuJoCo") {
  SECTION("ShouldFindMuJoCosContactsGivenEveryPrimitivePair") {
    // Each pose's contacts, as many as MuJoCo's between the same geoms, and
    // every value of each equal to MuJoCo's.
    std::vector<Pose> poses = load_poses();
    REQUIRE(poses.size() == 400);
    std::size_t total = 0;
    std::size_t miscounted = 0;
    double largest = 0.0;
    for (std::size_t p = 0; p < poses.size(); ++p) {
      Scenario scenario{.model = std::string{MODELS} + "collisions.xml"};
      for (std::uint32_t q = 0; q < poses[p].qpos.size(); ++q) {
        scenario.qpos.emplace_back(q, poses[p].qpos[q]);
      }
      Simulation simulation{scenario};
      auto configured = simulation.configure();
      if (!configured) {
        FAIL(configured.error().message());
      }
      step_once(InOut(simulation));
      std::vector<Row> found;
      for (const model::Contact& contact : simulation.contacts()) {
        found.push_back(flatten(contact));
      }
      std::vector<Row> expected = poses[p].contacts;
      sort_rows(InOut(found));
      sort_rows(InOut(expected));
      total += expected.size();
      if (found.size() != expected.size()) {
        CAPTURE(p, found.size(), expected.size());
        ++miscounted;
        continue;
      }
      for (std::size_t k = 0; k < found.size(); ++k) {
        REQUIRE(found[k].size() == expected[k].size());
        CHECK(found[k][0] == expected[k][0]);
        CHECK(found[k][1] == expected[k][1]);
        for (std::size_t i = 2; i < found[k].size(); ++i) {
          largest = std::max(largest, std::abs(found[k][i] - expected[k][i]));
        }
      }
    }
    CAPTURE(total, miscounted, largest);
    CHECK(total > 5000);
    CHECK(miscounted == 0);
    CHECK(largest == 0.0);
  }

  SECTION("ShouldCountEachTreesContacts") {
    // The arm's tree is in every contact of its four geoms.
    std::vector<Pose> poses = load_poses();
    Scenario scenario{.model = std::string{MODELS} + "collisions.xml"};
    for (std::uint32_t q = 0; q < poses[0].qpos.size(); ++q) {
      scenario.qpos.emplace_back(q, poses[0].qpos[q]);
    }
    Simulation simulation{scenario};
    REQUIRE(simulation.configure());
    step_once(InOut(simulation));
    const model::ArticulatedModel& m = simulation.mechanics().model();
    std::uint32_t arm =
        static_cast<std::uint32_t>(simulation.mechanics().trees().size() - 1);
    std::uint32_t first = simulation.mechanics().trees()[arm].first_geom;
    std::uint32_t expected = 0;
    for (const model::Contact& contact : simulation.contacts()) {
      expected += contact.geom[0] >= first || contact.geom[1] >= first;
    }
    std::uint32_t counted = 0;
    simulation.world().store_of<Touching>().for_each(
        [&](framework::Entity owner, const Touching& touching) {
          if (simulation.world()
                  .store_of<Mechanism>()
                  .component_of(owner)
                  .tree == arm) {
            counted = touching.contacts;
          }
        });
    CAPTURE(expected, m.geoms.size());
    CHECK(expected > 0);
    CHECK(counted == expected);
  }

  SECTION("ShouldRefuseGivenPairsOnlyAConvexColliderHandles") {
    // A capsule that may touch a cylinder needs MuJoCo's general convex
    // collider, which robotic does not have.
    std::filesystem::path path =
        std::filesystem::temp_directory_path() / "convex.xml";
    std::ofstream{path} << R"(<mujoco><worldbody>
      <body><freejoint/><geom type="capsule" size="0.1 0.2"/></body>
      <body><freejoint/><geom type="cylinder" size="0.1 0.2"/></body>
      </worldbody></mujoco>)";
    Simulation simulation{Scenario{.model = path.string()}};
    auto configured = simulation.configure();
    REQUIRE(!configured);
    CHECK(configured.error().message().find("capsule with cylinder") !=
          std::string::npos);
  }
}

}  // namespace simon::robotic
