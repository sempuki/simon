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
// application/automotive/roads, CARLA's Town01 and esmini's signed roads:
// positions on and off each road's surface, each lane's outer border, the lane
// at each lane's middle, the lane graph, signals, objects' outlines, and
// junctions' priorities and controllers, from the tables
// reference/libopendrive_reference.cpp recorded.
namespace simon::automotive {

namespace {

using model::meter;

using namespace testing;

// The edges of every file's lane graph together.
constexpr std::size_t EDGES = 387;

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

auto signal_of(const Row& row) -> const model::Signal& {
  const model::Road& road = road_of(row);
  auto found =
      std::ranges::find(road.signals, row.at("id"), &model::Signal::id);
  REQUIRE(found != road.signals.end());
  return *found;
}

auto object_of(const Row& row) -> const model::RoadObject& {
  const model::Road& road = road_of(row);
  auto found =
      std::ranges::find(road.objects, row.at("id"), &model::RoadObject::id);
  REQUIRE(found != road.objects.end());
  return *found;
}

auto orientation_word(model::RoadDirection orientation) -> std::string {
  switch (orientation) {
    case model::RoadDirection::POSITIVE:
      return "+";
    case model::RoadDirection::NEGATIVE:
      return "-";
    case model::RoadDirection::BOTH:
      return "none";
  }
  return "";
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
    CHECK(theirs.size() == EDGES);
    CHECK(ours == theirs);
  }

  SECTION("ShouldMatchSignals") {
    // Every field as read, every validity, and where each stands, to
    // rounding.
    std::map<std::string, std::set<std::pair<int, int>>, std::less<>> theirs;
    double farthest = 0.0;
    for (const Row& row : load_rows("libopendrive_signals.csv")) {
      const model::Signal& signal = signal_of(row);
      CAPTURE(row.at("file"), row.at("road"), row.at("id"));
      CHECK(signal.name == row.at("name"));
      CHECK(signal.s == number(row, "s"));
      CHECK(signal.t == number(row, "t"));
      CHECK(signal.z_offset == number(row, "z_offset"));
      CHECK(signal.dynamic == (row.at("dynamic") == "1"));
      CHECK(orientation_word(signal.orientation) == row.at("orientation"));
      CHECK(signal.country == row.at("country"));
      CHECK(signal.type == row.at("type"));
      CHECK(signal.subtype == row.at("subtype"));
      CHECK(signal.value.value_or(0.0) == number(row, "value"));
      CHECK(signal.unit == row.at("unit"));
      std::string key =
          row.at("file") + "," + row.at("road") + "," + row.at("id");
      auto& validities = theirs[key];
      if (!row.at("from_lane").empty()) {
        validities.emplace(static_cast<int>(number(row, "from_lane")),
                           static_cast<int>(number(row, "to_lane")));
      }
      model::Position position = model::compute_road_position(
          road_of(row), signal.s * meter, signal.t * meter,
          signal.z_offset * meter);
      Vector3 apart =
          position.numerical_value_in(meter).eigen() -
          Vector3{number(row, "x"), number(row, "y"), number(row, "z")};
      farthest = std::max(farthest, apart.norm());
    }
    for (const auto& [key, validities] : theirs) {
      std::vector<std::string> cells = split_cells(key);
      Row row{{"file", cells[0]}, {"road", cells[1]}, {"id", cells[2]}};
      std::set<std::pair<int, int>> ours;
      for (const model::LaneValidity& validity : signal_of(row).validities) {
        ours.emplace(validity.from, validity.to);
      }
      CAPTURE(key);
      CHECK(ours == validities);
    }
    CAPTURE(farthest);
    CHECK(theirs.size() == 24);
    CHECK(farthest < 1e-12);
  }

  SECTION("ShouldMatchObjectOutlines") {
    // Each corner of each crosswalk, in road coordinates or turned, pitched
    // and rolled in its own frame, on a road that climbs and leans, to
    // rounding.
    std::set<std::string> objects;
    double farthest = 0.0;
    for (const Row& row : load_rows("libopendrive_objects.csv")) {
      const model::RoadObject& object = object_of(row);
      CAPTURE(row.at("file"), row.at("road"), row.at("id"));
      objects.insert(row.at("file") + "," + row.at("id"));
      CHECK(object.type == row.at("type"));
      CHECK(object.s == number(row, "s"));
      CHECK(object.t == number(row, "t"));
      CHECK(object.z_offset == number(row, "z_offset"));
      CHECK(object.heading == number(row, "heading"));
      CHECK(object.pitch == number(row, "pitch"));
      CHECK(object.roll == number(row, "roll"));
      std::string validities;
      for (const model::LaneValidity& validity : object.validities) {
        validities += (validities.empty() ? "" : ";") +
                      std::to_string(validity.from) + ":" +
                      std::to_string(validity.to);
      }
      CHECK(validities == row.at("validities"));
      const model::RoadObject::Outline& outline =
          object.outlines.at(static_cast<std::size_t>(number(row, "outline")));
      std::vector<model::Position> corners =
          model::compute_outline(road_of(row), object, outline);
      Vector3 apart =
          corners.at(static_cast<std::size_t>(number(row, "corner")))
              .numerical_value_in(meter)
              .eigen() -
          Vector3{number(row, "x"), number(row, "y"), number(row, "z")};
      farthest = std::max(farthest, apart.norm());
    }
    CAPTURE(farthest);
    CHECK(objects.size() == 7);
    CHECK(farthest < 1e-12);
  }

  SECTION("ShouldMatchJunctionPrioritiesAndControllers") {
    std::set<std::string> theirs;
    std::set<std::string> files;
    for (const Row& row : load_rows("libopendrive_successors.csv")) {
      files.insert(row.at("file"));
    }
    for (const Row& row : load_rows("libopendrive_junctions.csv")) {
      theirs.insert(row.at("file") + "," + row.at("junction") + "," +
                    row.at("kind") + "," + row.at("first") + "," +
                    row.at("second") + "," + row.at("third"));
    }
    std::set<std::string> ours;
    for (const std::string& file : files) {
      for (const model::Junction& junction : network_of(file).junctions) {
        for (const model::JunctionPriority& priority : junction.priorities) {
          ours.insert(file + "," + junction.id + ",priority," + priority.high +
                      "," + priority.low + ",");
        }
        for (const model::JunctionController& controller :
             junction.controllers) {
          ours.insert(file + "," + junction.id + ",controller," +
                      controller.id + "," + controller.type + "," +
                      std::to_string(controller.sequence));
        }
      }
    }
    CHECK(theirs.size() == 7);
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
