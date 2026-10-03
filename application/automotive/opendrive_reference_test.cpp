// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "base/testing.hpp"
#include "framework/vocabulary.hpp"
#include "model/opendrive.hpp"
#include "model/road.hpp"

// simon's roads against libOpenDRIVE's, on the roads in
// application/automotive/roads: positions on and off each road's surface,
// each lane's outer border, and the lane at each lane's middle, from the
// tables reference/libopendrive_reference.cpp recorded.
namespace simon::automotive {

namespace {

using model::meter;

constexpr std::string_view ROADS = "application/automotive/roads/";
constexpr std::string_view REFERENCE = "application/automotive/reference/";

// One row of a table, as text by column name.
using Row = std::map<std::string, std::string, std::less<>>;

auto load_rows(std::string_view name) -> std::vector<Row> {
  std::ifstream file{std::string{REFERENCE} + std::string{name}};
  REQUIRE(file);
  auto split = [](const std::string& line) {
    std::vector<std::string> cells;
    for (std::size_t at = 0; at <= line.size();) {
      std::size_t comma = std::min(line.find(',', at), line.size());
      cells.emplace_back(line.substr(at, comma - at));
      at = comma + 1;
    }
    return cells;
  };
  std::string line;
  std::getline(file, line);
  std::vector<std::string> names = split(line);
  std::vector<Row> rows;
  while (std::getline(file, line)) {
    std::vector<std::string> cells = split(line);
    REQUIRE(cells.size() == names.size());
    Row row;
    for (std::size_t i = 0; i < names.size(); ++i) {
      row[names[i]] = cells[i];
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

auto number(const Row& row, std::string_view column) -> double {
  const std::string& text = row.find(column)->second;
  double value = 0.0;
  auto [end, error] =
      std::from_chars(text.data(), text.data() + text.size(), value);
  REQUIRE(error == std::errc{});
  return value;
}

// Each file's road network, read once.
auto network_of(std::string_view file) -> const model::RoadNetwork& {
  static std::map<std::string, model::RoadNetwork, std::less<>> networks;
  auto found = networks.find(file);
  if (found == networks.end()) {
    auto network =
        model::load_opendrive(std::string{ROADS} + std::string{file});
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
