// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "format/opendrive.hpp"

#include <string>

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_string.hpp"

namespace simon::format {

using namespace road;

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
    <objects>
      <object id="walk" type="crosswalk" s="95" t="0.5" hdg="0.1">
        <outlines>
          <outline id="0">
            <cornerLocal u="-2" v="-3" z="0"/>
            <cornerLocal u="2" v="-3" z="0"/>
            <cornerLocal u="2" v="3" z="0" height="0.1"/>
          </outline>
        </outlines>
        <validity fromLane="-1" toLane="1"/>
      </object>
    </objects>
    <signals>
      <signal id="light" name="east" s="90" t="-5" zOffset="2" dynamic="yes" orientation="+" country="DE" type="1000001" subtype="-1">
        <validity fromLane="-1" toLane="-1"/>
      </signal>
      <signal id="limit" s="10" t="5" dynamic="no" orientation="-" country="DE" type="274" subtype="55" value="50" unit="km/h"/>
    </signals>
  </road>
  <controller id="3" name="lights" sequence="1">
    <control signalId="light" type="0"/>
  </controller>
  <junction id="9">
    <connection id="0" incomingRoad="7" connectingRoad="8" contactPoint="end">
      <laneLink from="-1" to="1"/>
    </connection>
    <priority high="8" low="10"/>
    <controller id="3" type="0" sequence="2"/>
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
    CHECK(road->predecessor.kind == road::Link::Kind::ROAD);
    CHECK(road->predecessor.contact == road::Link::Contact::END);
    CHECK(road->successor.kind == road::Link::Kind::JUNCTION);
    CHECK(road->successor.id == "9");
    REQUIRE(network->junctions.size() == 1);
    const JunctionConnection& connection =
        network->junctions[0].connections.at(0);
    CHECK(connection.connecting_road == "8");
    CHECK(connection.contact == road::Link::Contact::END);
    CHECK(connection.lane_links.at(0).from == -1);
    CHECK(connection.lane_links.at(0).to == 1);
  }

  SECTION("ShouldReadSignalsGivenRoad") {
    auto network = parse_opendrive(ROAD);
    REQUIRE(network);
    const Road& road = *network->find_road("7");
    REQUIRE(road.signals.size() == 2);
    const Signal& light = road.signals[0];
    CHECK(light.id == "light");
    CHECK(light.name == "east");
    CHECK(light.s == 90.0);
    CHECK(light.t == -5.0);
    CHECK(light.z_offset == 2.0);
    CHECK(light.dynamic);
    CHECK(light.orientation == road::Direction::POSITIVE);
    CHECK(light.country == "DE");
    CHECK(light.type == "1000001");
    CHECK(light.subtype == "-1");
    CHECK_FALSE(light.value);
    REQUIRE(light.validities.size() == 1);
    CHECK(light.validities[0].from == -1);
    CHECK(light.validities[0].to == -1);
    const Signal& limit = road.signals[1];
    CHECK_FALSE(limit.dynamic);
    CHECK(limit.orientation == road::Direction::NEGATIVE);
    CHECK(limit.value == 50.0);
    CHECK(limit.unit == "km/h");
    CHECK(limit.validities.empty());  // Every lane in its orientation.
  }

  SECTION("ShouldReadObjectsGivenRoad") {
    auto network = parse_opendrive(ROAD);
    REQUIRE(network);
    const Road& road = *network->find_road("7");
    REQUIRE(road.objects.size() == 1);
    const road::Object& walk = road.objects[0];
    CHECK(walk.type == "crosswalk");
    CHECK(walk.s == 95.0);
    CHECK(walk.t == 0.5);
    CHECK(walk.heading == 0.1);
    CHECK(walk.orientation == road::Direction::BOTH);
    REQUIRE(walk.outlines.size() == 1);
    const road::Object::Outline& outline = walk.outlines[0];
    CHECK(outline.frame == road::Object::Outline::Frame::LOCAL);
    CHECK(outline.closed);
    REQUIRE(outline.corners.size() == 3);
    CHECK(outline.corners[1].first == 2.0);
    CHECK(outline.corners[1].second == -3.0);
    CHECK(outline.corners[2].height == 0.1);
    REQUIRE(walk.validities.size() == 1);
    CHECK(walk.validities[0].from == -1);
    CHECK(walk.validities[0].to == 1);
  }

  SECTION("ShouldReadControllersAndPrioritiesGivenNetwork") {
    auto network = parse_opendrive(ROAD);
    REQUIRE(network);
    REQUIRE(network->controllers.size() == 1);
    const SignalController& controller = network->controllers[0];
    CHECK(controller.id == "3");
    CHECK(controller.name == "lights");
    CHECK(controller.sequence == 1);
    REQUIRE(controller.controls.size() == 1);
    CHECK(controller.controls[0].signal == "light");
    const Junction& junction = network->junctions[0];
    REQUIRE(junction.priorities.size() == 1);
    CHECK(junction.priorities[0].high == "8");
    CHECK(junction.priorities[0].low == "10");
    REQUIRE(junction.controllers.size() == 1);
    CHECK(junction.controllers[0].id == "3");
    CHECK(junction.controllers[0].sequence == 2);
  }

  SECTION("ShouldRefuseGivenUnknownOrientation") {
    auto network = parse_opendrive(
        edited(R"(orientation="+")", R"(orientation="sideways")"));
    REQUIRE_FALSE(network);
    CHECK_THAT(std::string{network.error().message()},
               ContainsSubstring("sideways"));
  }

  SECTION("ShouldRefuseGivenUnknownDynamic") {
    auto network =
        parse_opendrive(edited(R"(dynamic="yes")", R"(dynamic="maybe")"));
    REQUIRE_FALSE(network);
    CHECK_THAT(std::string{network.error().message()},
               ContainsSubstring("maybe"));
  }

  SECTION("ShouldRefuseGivenValidityBackward") {
    auto network = parse_opendrive(
        edited(R"(fromLane="-1" toLane="1")", R"(fromLane="1" toLane="-1")"));
    REQUIRE_FALSE(network);
    CHECK_THAT(std::string{network.error().message()},
               ContainsSubstring("fromLane after toLane"));
  }

  SECTION("ShouldRefuseGivenMixedCorners") {
    auto network =
        parse_opendrive(edited(R"(<cornerLocal u="2" v="-3" z="0"/>)",
                               R"(<cornerRoad s="96" t="-3"/>)"));
    REQUIRE_FALSE(network);
    CHECK_THAT(std::string{network.error().message()},
               ContainsSubstring("mixes"));
  }

  SECTION("ShouldRefuseGivenPriorityWithoutLow") {
    auto network = parse_opendrive(edited(R"( low="10")", ""));
    REQUIRE_FALSE(network);
    CHECK_THAT(std::string{network.error().message()},
               ContainsSubstring("needs high and low"));
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

}  // namespace simon::format
