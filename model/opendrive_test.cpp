// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/opendrive.hpp"

#include <string>

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_string.hpp"

namespace simon::model {

namespace {

using Catch::Matchers::ContainsSubstring;

// One road: a line and an arc, with an elevation, a lane offset, and two lane
// sections, the second adding a right lane.
constexpr std::string_view ROAD = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="6"/>
  <road id="7" length="150" junction="-1">
    <link>
      <predecessor elementType="road" elementId="3" contactPoint="end"/>
      <successor elementType="junction" elementId="9"/>
    </link>
    <planView>
      <geometry s="100" x="100" y="0" hdg="0" length="50">
        <arc curvature="0.01"/>
      </geometry>
      <geometry s="0" x="0" y="0" hdg="0" length="100">
        <line/>
      </geometry>
    </planView>
    <elevationProfile>
      <elevation s="0" a="1" b="0.02" c="0" d="0"/>
    </elevationProfile>
    <lanes>
      <laneOffset s="0" a="0.25" b="0" c="0" d="0"/>
      <laneSection s="0">
        <left>
          <lane id="1" type="driving"><width sOffset="0" a="3.5"/></lane>
        </left>
        <center><lane id="0" type="none"/></center>
        <right>
          <lane id="-1" type="driving"><width sOffset="0" a="3.5"/></lane>
        </right>
      </laneSection>
      <laneSection s="80">
        <center><lane id="0" type="none"/></center>
        <right>
          <lane id="-2" type="shoulder"><link><predecessor id="-1"/></link><width sOffset="0" a="1"/></lane>
          <lane id="-1" type="driving"><width sOffset="0" a="3.5"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <junction id="9">
    <connection id="0" incomingRoad="7" connectingRoad="8" contactPoint="end">
      <laneLink from="-1" to="1"/>
    </connection>
  </junction>
</OpenDRIVE>
)";

// ROAD with `from` replaced by `to`.
auto edited(std::string_view from, std::string_view to) -> std::string {
  std::string text{ROAD};
  text.replace(text.find(from), from.size(), to);
  return text;
}

}  // namespace

TEST_CASE("OpenDrive") {
  SECTION("ShouldReadRoadGivenDocument") {
    auto network = parse_opendrive(ROAD);
    REQUIRE(network);
    const Road* road = network->find_road("7");
    REQUIRE(road);
    CHECK(road->length == 150.0);
    REQUIRE(road->plan.size() == 2);
    CHECK(road->plan[0].s0 == 0.0);  // Ordered by s.
    CHECK(std::holds_alternative<ArcGeometry>(road->plan[1].shape));
    CHECK(road->elevation.evaluate(50.0) == 2.0);
    REQUIRE(road->lane_sections.size() == 2);
    const LaneSection& second = road->lane_sections[1];
    REQUIRE(second.right.size() == 2);
    CHECK(second.right[0].id == -1);  // Ordered from the center out.
    CHECK(second.right[1].type == "shoulder");
    CHECK(second.left.empty());
    CHECK(second.right[1].predecessor == -1);
    CHECK_FALSE(second.right[1].successor);
    CHECK(road->predecessor.kind == RoadLink::Kind::ROAD);
    CHECK(road->predecessor.contact == RoadLink::Contact::END);
    CHECK(road->successor.kind == RoadLink::Kind::JUNCTION);
    CHECK(road->successor.id == "9");
    REQUIRE(network->junctions.size() == 1);
    const JunctionConnection& connection =
        network->junctions[0].connections.at(0);
    CHECK(connection.connecting_road == "8");
    CHECK(connection.contact == RoadLink::Contact::END);
    CHECK(connection.lane_links.at(0).from == -1);
    CHECK(connection.lane_links.at(0).to == 1);
  }

  SECTION("ShouldRefuseGivenUnknownContactPoint") {
    auto network = parse_opendrive(
        edited(R"(contactPoint="end"/>)", R"(contactPoint="middle"/>)"));
    REQUIRE_FALSE(network);
    CHECK_THAT(std::string{network.error().message()},
               ContainsSubstring("middle"));
  }

  SECTION("ShouldSayWhereGivenMissingAttribute") {
    auto network = parse_opendrive(edited(R"(curvature="0.01")", ""));
    REQUIRE_FALSE(network);
    CHECK_THAT(std::string{network.error().message()},
               ContainsSubstring("line 11") && ContainsSubstring("curvature"));
  }

  SECTION("ShouldRefuseGivenBadNumber") {
    auto network = parse_opendrive(edited(R"(a="3.5")", R"(a="wide")"));
    REQUIRE_FALSE(network);
    CHECK_THAT(std::string{network.error().message()},
               ContainsSubstring("not a finite number"));
  }

  SECTION("ShouldRefuseGivenPoly3") {
    auto network = parse_opendrive(edited("<line/>", R"(<poly3 a="0"/>)"));
    REQUIRE_FALSE(network);
    CHECK_THAT(std::string{network.error().message()},
               ContainsSubstring("deprecated"));
  }

  SECTION("ShouldRefuseGivenLaneBorders") {
    auto network = parse_opendrive(edited(R"(<width sOffset="0" a="1"/>)",
                                          R"(<border sOffset="0" a="1"/>)"));
    REQUIRE_FALSE(network);
    CHECK_THAT(std::string{network.error().message()},
               ContainsSubstring("widths"));
  }

  SECTION("ShouldRefuseGivenGapInLaneIds") {
    auto network =
        parse_opendrive(edited(R"(<lane id="-2")", R"(<lane id="-3")"));
    REQUIRE_FALSE(network);
    CHECK_THAT(std::string{network.error().message()},
               ContainsSubstring("without a gap"));
  }

  SECTION("ShouldRefuseGivenMalformedXml") {
    auto network = parse_opendrive("<OpenDRIVE><road>");
    REQUIRE_FALSE(network);
    CHECK_FALSE(load_opendrive("no/such/file.xodr"));
  }
}

}  // namespace simon::model
