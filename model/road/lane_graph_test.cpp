// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/road/lane_graph.hpp"

#include <vector>

#include "base/testing.hpp"

namespace simon::road {

namespace {

// A road with one lane each way, linked to its neighbors as given.
auto road(std::string id, Link predecessor, Link successor) -> Road {
  return Road{
      .id = std::move(id),
      .length = 100.0,
      .lane_sections = {LaneSection{
          .left = {Lane{.id = 1, .predecessor = 1, .successor = 1}},
          .right = {Lane{.id = -1, .predecessor = -1, .successor = -1}}}},
      .predecessor = predecessor,
      .successor = successor,
  };
}

auto to_road(std::string id, Link::Contact contact) -> Link {
  return Link{
      .kind = Link::Kind::ROAD, .id = std::move(id), .contact = contact};
}

auto lanes(std::span<const LaneKey> keys) -> std::vector<LaneKey> {
  return {keys.begin(), keys.end()};
}

}  // namespace

TEST_CASE("LaneGraph") {
  SECTION("ShouldFollowTravelGivenRingOfTwoRoads") {
    // a's end meets b's start and b's end meets a's start.
    Map network{.roads = {road("a", to_road("b", Link::Contact::END),
                               to_road("b", Link::Contact::START)),
                          road("b", to_road("a", Link::Contact::END),
                               to_road("a", Link::Contact::START))}};
    LaneGraph graph = build_lane_graph(network);
    // Right lanes run with s, from a into b; left lanes against it, into the
    // road before.
    CHECK(lanes(graph.successors_of({.road = 0, .lane = -1})) ==
          std::vector<LaneKey>{{.road = 1, .lane = -1}});
    CHECK(lanes(graph.successors_of({.road = 0, .lane = 1})) ==
          std::vector<LaneKey>{{.road = 1, .lane = 1}});
    CHECK(graph.edges().size() == 4);
    // And back the other way.
    CHECK(lanes(graph.predecessors_of({.road = 1, .lane = -1})) ==
          std::vector<LaneKey>{{.road = 0, .lane = -1}});
    CHECK(lanes(graph.predecessors_of({.road = 1, .lane = 1})) ==
          std::vector<LaneKey>{{.road = 0, .lane = 1}});
  }

  SECTION("ShouldEnterConnectingRoadGivenJunction") {
    Map network{
        .roads = {road("in", {}, {.kind = Link::Kind::JUNCTION, .id = "j"}),
                  road("through", {}, {})},
        .junctions = {Junction{.id = "j",
                               .connections = {JunctionConnection{
                                   .incoming_road = "in",
                                   .connecting_road = "through",
                                   .contact = Link::Contact::START,
                                   .lane_links = {{.from = -1, .to = -1},
                                                  {.from = -9, .to = -1}}}}}}};
    LaneGraph graph = build_lane_graph(network);
    CHECK(lanes(graph.successors_of({.road = 0, .lane = -1})) ==
          std::vector<LaneKey>{{.road = 1, .lane = -1}});
    // A link to a lane the road lacks adds nothing.
    CHECK(graph.edges().size() == 1);
    CHECK(graph.successors_of({.road = 1, .lane = -1}).empty());
    CHECK(lanes(graph.predecessors_of({.road = 1, .lane = -1})) ==
          std::vector<LaneKey>{{.road = 0, .lane = -1}});
    CHECK(graph.predecessors_of({.road = 0, .lane = -1}).empty());
  }
}

}  // namespace simon::road
