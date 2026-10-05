// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

#include "application/robotic/simulation.hpp"
#include "application/robotic/testing.hpp"
#include "base/testing.hpp"

// Contacts by simon and by MuJoCo, against the table
// reference/mujoco_contacts.py recorded: every primitive pair, at 400 poses
// drawn at random.
namespace simon::robotic {

namespace {

using namespace testing;

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

auto load_poses(std::string_view table = "mujoco_contacts.csv")
    -> std::vector<Pose> {
  std::vector<Pose> poses;
  for (const std::vector<std::string>& cells :
       load_table(std::string{REFERENCE} + std::string{table}).lines) {
    REQUIRE(cells.size() == 3);
    if (cells[1] == "qpos") {
      poses.push_back(Pose{.qpos = parse_numbers(cells[2])});
    } else {
      poses.back().contacts.push_back(parse_numbers(cells[2]));
    }
  }
  return poses;
}

struct Comparison final {
  std::size_t total = 0;
  std::size_t miscounted = 0;
  std::size_t mismatched = 0;  // Contacts between other geoms.
  double largest = 0.0;
};

// Each pose of `table` posed in `model`, its contacts against MuJoCo's.
auto compare_poses(std::string_view model, std::string_view table)
    -> Comparison {
  std::vector<Pose> poses = load_poses(table);
  REQUIRE(poses.size() == 400);
  Comparison result;
  for (std::size_t p = 0; p < poses.size(); ++p) {
    Scenario scenario{.model = std::string{MODELS} + std::string{model}};
    for (std::uint32_t q = 0; q < poses[p].qpos.size(); ++q) {
      scenario.qpos.emplace_back(q, poses[p].qpos[q]);
    }
    Simulation simulation{scenario};
    auto configured = simulation.configure();
    if (!configured) {
      FAIL(configured.error().message());
    }
    step_once(InOut(simulation), 0);
    std::vector<Row> found;
    for (const model::Contact& contact : simulation.contacts()) {
      found.push_back(flatten(contact));
    }
    std::vector<Row> expected = poses[p].contacts;
    sort_rows(InOut(found));
    sort_rows(InOut(expected));
    result.total += expected.size();
    if (found.size() != expected.size()) {
      ++result.miscounted;
      continue;
    }
    for (std::size_t k = 0; k < found.size(); ++k) {
      REQUIRE(found[k].size() == expected[k].size());
      if (found[k][0] != expected[k][0] || found[k][1] != expected[k][1]) {
        ++result.mismatched;
        continue;
      }
      for (std::size_t i = 2; i < found[k].size(); ++i) {
        result.largest =
            std::max(result.largest, std::abs(found[k][i] - expected[k][i]));
      }
    }
  }
  return result;
}

}  // namespace

TEST_CASE("CollisionAgainstMuJoCo") {
  SECTION("ShouldFindMuJoCosContactsGivenEveryPrimitivePair") {
    // Each pose's contacts, as many as MuJoCo's between the same geoms, and
    // every value of each equal to MuJoCo's.
    Comparison c = compare_poses("collisions.xml", "mujoco_contacts.csv");
    CAPTURE(c.total, c.miscounted, c.mismatched, c.largest);
    CHECK(c.total > 5000);
    CHECK(c.miscounted == 0);
    CHECK(c.mismatched == 0);
    CHECK(c.largest == 0.0);
  }

  SECTION("ShouldFindMuJoCosContactsGivenPairsOnlyItsConvexColliderTakes") {
    // Ellipsoids and cylinders by GJK and EPA, their faces clipped or the
    // geoms turned for more: every contact equal to MuJoCo's.
    Comparison c = compare_poses("convex.xml", "mujoco_convex.csv");
    CAPTURE(c.total, c.miscounted, c.mismatched, c.largest);
    CHECK(c.total > 1500);
    CHECK(c.miscounted == 0);
    CHECK(c.mismatched == 0);
    CHECK(c.largest == 0.0);
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
    step_once(InOut(simulation), 0);
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
}

}  // namespace simon::robotic
