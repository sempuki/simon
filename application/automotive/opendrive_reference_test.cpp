// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "application/automotive/testing.hpp"
#include "base/testing.hpp"
#include "format/opendrive.hpp"
#include "framework/vocabulary.hpp"
#include "model/lane_graph.hpp"
#include "model/road.hpp"

// simon's roads against libOpenDRIVE's, on the roads in
// application/automotive/roads and CARLA's Town01: positions on and off each
// road's surface, each lane's outer border, the lane at each lane's middle,
// and the lane graph, from the tables reference/libopendrive_reference.cpp
// recorded.
namespace simon::automotive {

namespace {

using model::meter;

using namespace testing;

// Each file's road network, read once.
auto network_of(std::string_view file) -> const model::RoadNetwork& {
  static std::map<std::string, model::RoadNetwork, std::less<>> networks;
  auto found = networks.find(file);
  if (found == networks.end()) {
    auto network = format::load_opendrive(find_road_path(file));
    REQUIRE(network);
    found = networks.emplace(std::string{file}, std::move(*network)).first;
  }
  return found->second;
}

auto road_of(const Row& row) -> const model::Road& {
  const model::Road* road =
      network_of(row.at("file")).find_road(row.at("road"));
  REQUIRE(road);
  return *road;
}

// The largest difference in each file.
using Largest = std::map<std::string, double, std::less<>>;

// The largest distance of simon's positions from a table's, by file.
auto position_errors(std::string_view table) -> Largest {
  Largest largest;
  for (const Row& row : load_rows(table)) {
    model::Position position = model::compute_road_position(
        road_of(row), number(row, "s") * meter, number(row, "t") * meter,
        number(row, "h") * meter);
    Vector3 apart =
        position.numerical_value_in(meter).eigen() -
        Vector3{number(row, "x"), number(row, "y"), number(row, "z")};
    double& most = largest[row.at("file")];
    most = std::max(most, apart.norm());
  }
  return largest;
}

}  // namespace

TEST_CASE("OpenDriveAgainstLibOpenDrive") {
  SECTION("ShouldMatchPositionsGivenLinesArcsAndSpirals") {
    // Through elevation and superelevation, on and off the surface, to
    // rounding.
    Largest largest = position_errors("libopendrive_positions.csv");
    CAPTURE(largest["curves.xodr"], largest["Town01.xodr"]);
    CHECK(largest["curves.xodr"] < 1e-12);
    CHECK(largest["Town01.xodr"] < 1e-12);
  }

  SECTION("ShouldFollowArcLengthGivenParamPoly3") {
    // s is arc length. simon finds it to rounding, as an independent
    // quadrature does; libOpenDRIVE's table of chords, made to 1 cm, is 5.4 mm
    // off at most.
    double ours = position_errors("exact_positions.csv")["paramPoly3.xodr"];
    double theirs = 0.0;
    std::vector<Row> exact = load_rows("exact_positions.csv");
    std::map<std::string, const Row*, std::less<>> by_key;
    auto key = [](const Row& row) {
      return row.at("road") + "," + row.at("s") + "," + row.at("t") + "," +
             row.at("h");
    };
    for (const Row& row : exact) {
      by_key[key(row)] = &row;
    }
    for (const Row& row : load_rows("libopendrive_positions.csv")) {
      if (row.at("file") != "paramPoly3.xodr") {
        continue;
      }
      const Row& truth = *by_key.at(key(row));
      theirs =
          std::max(theirs, std::hypot(number(row, "x") - number(truth, "x"),
                                      number(row, "y") - number(truth, "y")));
    }
    CAPTURE(ours, theirs);
    CHECK(ours < 1e-9);
    CHECK(theirs < 0.05);
  }

  SECTION("ShouldMatchLaneBorders") {
    Largest largest;
    for (const Row& row : load_rows("libopendrive_borders.csv")) {
      double border =
          model::compute_lane_border(road_of(row), number(row, "s") * meter,
                                     static_cast<int>(number(row, "lane")))
              .numerical_value_in(meter);
      double& most = largest[row.at("file")];
      most = std::max(most, std::abs(border - number(row, "t")));
    }
    for (const auto& [file, most] : largest) {
      CAPTURE(file, most);
      CHECK(most < 1e-12);
    }
  }

  SECTION("ShouldMatchLaneGraph") {
    // Every edge, by road id, lane section start and lane id, in each file.
    using Edge = std::tuple<std::string, std::string, double, int, std::string,
                            double, int>;
    std::set<Edge> theirs;
    std::set<std::string> files;
    for (const Row& row : load_rows("libopendrive_successors.csv")) {
      files.insert(row.at("file"));
      theirs.emplace(
          row.at("file"), row.at("from_road"), number(row, "from_section"),
          static_cast<int>(number(row, "from_lane")), row.at("to_road"),
          number(row, "to_section"), static_cast<int>(number(row, "to_lane")));
    }
    std::set<Edge> ours;
    for (const std::string& file : files) {
      const model::RoadNetwork& network = network_of(file);
      for (const model::LaneGraph::Edge& edge :
           model::build_lane_graph(network).edges()) {
        const model::Road& from = network.roads[edge.from.road];
        const model::Road& to = network.roads[edge.to.road];
        ours.emplace(file, from.id, from.lane_sections[edge.from.section].s0,
                     edge.from.lane, to.id,
                     to.lane_sections[edge.to.section].s0, edge.to.lane);
      }
    }
    CHECK(theirs.size() == 285);
    CHECK(ours == theirs);
  }

  SECTION("ShouldFindLaneGivenLaneMiddle") {
    // libOpenDRIVE keys lanes by outer border, so where a lane has no width
    // and shares its neighbor's border, it can give that lane for a point in
    // the neighbor's middle. That is the only way the two may differ.
    int differ = 0;
    for (const Row& row : load_rows("libopendrive_lanes.csv")) {
      const model::Road& road = road_of(row);
      auto s = number(row, "s") * meter;
      std::optional<int> lane =
          model::find_lane(road, s, number(row, "t") * meter);
      auto theirs = static_cast<int>(number(row, "lane"));
      if (lane != theirs) {
        ++differ;
        int inner = theirs > 0 ? theirs - 1 : theirs + 1;
        CAPTURE(row.at("file"), row.at("road"), row.at("s"), theirs, lane);
        CHECK(model::compute_lane_border(road, s, theirs) ==
              model::compute_lane_border(road, s, inner));
      }
    }
    CHECK(differ == 1);
  }
}

}  // namespace simon::automotive
