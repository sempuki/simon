// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <array>
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

// How far a contact's distance, position or frame may differ from MuJoCo's
// at least: a pose is one computation, with nothing to accumulate.
constexpr double ROUNDING = 1e-12;

using namespace testing;

// A contact as the table holds it: geoms, distance, position, frame,
// dimension, friction, solref, solimp, included margin, excluded.
using Row = std::vector<double>;

auto flatten(const articulated::Contact& c) -> Row {
  Row row{static_cast<double>(c.geom[0]), static_cast<double>(c.geom[1]),
          c.dist};
  row.insert(row.end(), c.pos.begin(), c.pos.end());
  // The frame row by row, as MuJoCo's: the normal, then the tangents.
  for (int r = 0; r < 3; ++r) {
    for (int k = 0; k < 3; ++k) {
      row.push_back(c.frame(r, k));
    }
  }
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
  // Poses whose contacts differ by more than rounding moves MuJoCo's there,
  // and where the first of them lies: the pose, the contact's geoms, and the
  // value, by its place in a Row.
  std::size_t beyond = 0;
  std::size_t beyond_pose = 0;
  std::array<double, 2> beyond_geoms{};
  std::size_t beyond_value = 0;
  double beyond_apart = 0.0;
  double beyond_allowed = 0.0;
};

// How far rounding moves MuJoCo's contacts at a pose, and whether it
// changes which there are (reference/mujoco_checks.py).
struct ContactSpread final {
  double spread = 0.0;
  bool recount = false;
};

auto load_contact_spreads(std::string_view table)
    -> std::vector<ContactSpread> {
  std::vector<ContactSpread> spreads;
  for (const std::vector<std::string>& cells :
       load_table(std::string{REFERENCE} + "mujoco_contact_spread.csv").lines) {
    REQUIRE(cells.size() == 4);
    if (cells[0] == table) {
      spreads.push_back(
          {.spread = std::stod(cells[2]), .recount = cells[3] == "1"});
    }
  }
  return spreads;
}

// Each pose of `table` posed in `model`, its contacts against MuJoCo's,
// each value within ROUNDING, or within what rounding moves MuJoCo's
// contacts at that pose.
auto compare_poses(std::string_view model, std::string_view table)
    -> Comparison {
  std::vector<Pose> poses = load_poses(table);
  REQUIRE(poses.size() == 400);
  std::vector<ContactSpread> spreads = load_contact_spreads(table);
  REQUIRE(spreads.size() == poses.size());
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
    for (const articulated::Contact& contact : simulation.contacts()) {
      found.push_back(flatten(contact));
    }
    std::vector<Row> expected = poses[p].contacts;
    sort_rows(InOut(found));
    sort_rows(InOut(expected));
    result.total += expected.size();
    // Where rounding alone changes MuJoCo's contacts, which there are is
    // rounding's to choose.
    if (spreads[p].recount) {
      continue;
    }
    if (found.size() != expected.size()) {
      ++result.miscounted;
      continue;
    }
    double allowed = std::max(SPREADS * spreads[p].spread, ROUNDING);
    bool beyond = false;
    for (std::size_t k = 0; k < found.size(); ++k) {
      REQUIRE(found[k].size() == expected[k].size());
      if (found[k][0] != expected[k][0] || found[k][1] != expected[k][1]) {
        ++result.mismatched;
        continue;
      }
      for (std::size_t i = 2; i < found[k].size(); ++i) {
        double apart = std::abs(found[k][i] - expected[k][i]);
        result.largest = std::max(result.largest, apart);
        if (apart > allowed && !beyond) {
          beyond = true;
          if (result.beyond == 0) {
            result.beyond_pose = p;
            result.beyond_geoms = {found[k][0], found[k][1]};
            result.beyond_value = i;
            result.beyond_apart = apart;
            result.beyond_allowed = allowed;
          }
        }
      }
    }
    result.beyond += beyond ? 1 : 0;
  }
  return result;
}

}  // namespace

TEST_CASE("CollisionAgainstMuJoCo") {
  SECTION("ShouldFindMuJoCosContactsGivenEveryPrimitivePair") {
    // Each pose's contacts, as many as MuJoCo's between the same geoms, and
    // every value of each as MuJoCo's to rounding: within 1e-12, or within
    // what rounding moves MuJoCo's own contacts at that pose.
    Comparison c = compare_poses("collisions.xml", "mujoco_contacts.csv");
    CAPTURE(c.total, c.miscounted, c.mismatched, c.largest, c.beyond,
            c.beyond_pose, c.beyond_geoms[0], c.beyond_geoms[1], c.beyond_value,
            c.beyond_apart, c.beyond_allowed);
    CHECK(c.total > 5000);
    CHECK(c.miscounted == 0);
    CHECK(c.mismatched == 0);
    CHECK(c.beyond == 0);
  }

  SECTION("ShouldFindMuJoCosContactsGivenPairsOnlyItsConvexColliderTakes") {
    // Ellipsoids and cylinders by GJK and EPA, their faces clipped or the
    // geoms turned for more: every contact as MuJoCo's to rounding. At some
    // poses rounding alone moves MuJoCo's contact far, where a box's edge
    // meets a cylinder's, and where EPA stops within its tolerance moves it
    // too; the spread MuJoCo measures of both says how far.
    Comparison c = compare_poses("convex.xml", "mujoco_convex.csv");
    CAPTURE(c.total, c.miscounted, c.mismatched, c.largest, c.beyond,
            c.beyond_pose, c.beyond_geoms[0], c.beyond_geoms[1], c.beyond_value,
            c.beyond_apart, c.beyond_allowed);
    CHECK(c.total > 1500);
    CHECK(c.miscounted == 0);
    CHECK(c.mismatched == 0);
    CHECK(c.beyond == 0);
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
    const articulated::Scene& m = simulation.mechanics().model();
    std::uint32_t arm =
        static_cast<std::uint32_t>(simulation.mechanics().trees().size() - 1);
    std::uint32_t first = simulation.mechanics().trees()[arm].first_geom;
    std::uint32_t expected = 0;
    for (const articulated::Contact& contact : simulation.contacts()) {
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
