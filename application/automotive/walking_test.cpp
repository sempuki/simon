// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "application/automotive/simulation.hpp"
#include "application/automotive/simulation_systems.hpp"
#include "application/automotive/testing.hpp"
#include "base/testing.hpp"
#include "core/argument.hpp"

// Pedestrians: the walking graph of each test network, routes against every
// shortest distance, walkers that keep their distance, and walking speeds
// drawn as Weidmann measured them.
namespace simon::automotive {

namespace {

using namespace std::chrono_literals;
using namespace testing;

// How many edges of each kind, and how many components.
struct Census final {
  int sidewalks = 0;
  int crossings = 0;
  int corners = 0;
  std::size_t components = 0;
};

auto take_census(const Network& network) -> Census {
  Census census;
  for (const road::WalkEdge& edge : network.walking.edges()) {
    census.sidewalks += edge.kind == road::WalkEdge::Kind::SIDEWALK ? 1 : 0;
    census.crossings += edge.kind == road::WalkEdge::Kind::CROSSING ? 1 : 0;
    census.corners += edge.kind == road::WalkEdge::Kind::CORNER ? 1 : 0;
  }
  census.components =
      std::set<std::uint32_t>(network.walking_components.begin(),
                              network.walking_components.end())
          .size();
  return census;
}

auto route_length(const Network& network, const std::vector<road::Leg>& legs)
    -> double {
  double length = 0.0;
  for (const road::Leg& leg : legs) {
    length += network.walking.edges()[leg.edge].length();
  }
  return length;
}

}  // namespace

TEST_CASE("Walking") {
  SECTION("ShouldJoinSidewalksByCrossingsAndCorners") {
    // Four arms, each sidewalk split by its crosswalk; four corners, 7.8 m
    // across; every sidewalk reachable.
    Census signalized = take_census(create_network("signalized.xodr"));
    CHECK(signalized.sidewalks == 16);
    CHECK(signalized.crossings == 4);
    CHECK(signalized.corners == 4);
    CHECK(signalized.components == 1);
    // A curve with two crosswalks and no junction.
    Census crosswalks = take_census(create_network("crosswalks.xodr"));
    CHECK(crosswalks.sidewalks == 6);
    CHECK(crosswalks.crossings == 2);
    CHECK(crosswalks.corners == 0);
    CHECK(crosswalks.components == 1);
    // A T with no crosswalks: its far side, and each corner of the minor
    // road, cut off from each other by the roads.
    Census priority = take_census(create_network("priority.xodr"));
    CHECK(priority.corners == 3);
    CHECK(priority.components == 3);
    // Town01 has no crosswalks: each block is its own.
    Census town = take_census(create_network("Town01.xodr"));
    CHECK(town.sidewalks == 52);
    CHECK(town.crossings == 0);
    CHECK(town.components == 8);
  }

  SECTION("ShouldRouteByTheShortestWayGivenAnyTwoPlaces") {
    // Every route's length against Floyd and Warshall's all-pairs shortest
    // distances.
    for (const char* file : {"signalized.xodr", "Town01.xodr"}) {
      Network network = create_network(file);
      std::size_t n = network.walking.nodes().size();
      constexpr double FAR = std::numeric_limits<double>::infinity();
      std::vector<std::vector<double>> best(n, std::vector<double>(n, FAR));
      for (std::size_t i = 0; i < n; ++i) {
        best[i][i] = 0.0;
      }
      for (const road::WalkEdge& edge : network.walking.edges()) {
        best[edge.from][edge.to] =
            std::min(best[edge.from][edge.to], edge.length());
        best[edge.to][edge.from] = best[edge.from][edge.to];
      }
      for (std::size_t k = 0; k < n; ++k) {
        for (std::size_t i = 0; i < n; ++i) {
          for (std::size_t j = 0; j < n; ++j) {
            best[i][j] = std::min(best[i][j], best[i][k] + best[k][j]);
          }
        }
      }
      double worst = 0.0;
      int routes = 0;
      for (std::uint32_t i = 0; i < n; ++i) {
        for (std::uint32_t j = 0; j < n; ++j) {
          std::vector<road::Leg> legs = network.walking.find_route(i, j);
          if (best[i][j] == FAR || i == j) {
            CHECK(legs.empty());
            continue;
          }
          ++routes;
          worst = std::max(worst,
                           std::abs(route_length(network, legs) - best[i][j]));
          // Leg after leg, each starting where the last ended.
          std::uint32_t at = i;
          for (const road::Leg& leg : legs) {
            const road::WalkEdge& edge = network.walking.edges()[leg.edge];
            CHECK((leg.forward ? edge.from : edge.to) == at);
            at = leg.forward ? edge.to : edge.from;
          }
          CHECK(at == j);
        }
      }
      CAPTURE(file, routes, worst);
      CHECK(routes > 0);
      CHECK(worst < 1e-9);
    }
  }

  SECTION("ShouldKeepHalfAMeterGivenSlowerWalkerAhead") {
    // A walker at 1.6 m/s behind one at 0.8 m/s, 10 m apart on one
    // sidewalk: it closes in and walks at 0.8 m/s behind, its half meter and
    // a second's walk, 1.3 m, behind.
    Network network = create_network("crosswalks.xodr");
    World world;
    REQUIRE(World::set_up().numbered(1).holding<archetype::Pedestrian>(2).build(
        Out(world)));
    std::uint32_t edge = 0;
    while (network.walking.edges()[edge].kind !=
               road::WalkEdge::Kind::SIDEWALK ||
           network.walking.edges()[edge].length() < 35.0) {
      ++edge;
    }
    std::vector<Entity> walkers;
    for (auto [along, speed] : {std::pair{10.0, 0.8}, std::pair{0.0, 1.6}}) {
      WalkRoute route{.legs = {road::Leg{.edge = edge, .forward = true}}};
      WalkState state{.along = along * meter};
      walkers.push_back(
          *world.create<archetype::Pedestrian>()
               .with(PlaceWalker::locate_walker(network, route, state))
               .with(state)
               .with(Walker{.desired_speed = speed * meter_per_second})
               .with(std::move(route))
               .with(WalkCommand{})
               .build());
    }
    world.sync();
    Scheduler scheduler{make_schedule(network)};
    double nearest = std::numeric_limits<double>::infinity();
    for (long k = 0; k < 200; ++k) {
      scheduler.step(Step{.time = TimePoint{} + k * 100ms, .dt = 100ms},
                     InOut(world));
      const auto& states = world.store_of<WalkState>();
      nearest = std::min(nearest, (states.component_of(walkers[0]).along -
                                   states.component_of(walkers[1]).along)
                                      .numerical_value_in(meter));
    }
    const WalkState& behind =
        world.store_of<WalkState>().component_of(walkers[1]);
    CAPTURE(nearest, behind.speed.numerical_value_in(meter_per_second));
    CHECK(nearest >= Pace::SPACE + 0.8 * Pace::HEADWAY - 0.01);
    CHECK(nearest < Pace::SPACE + 0.8 * Pace::HEADWAY + 0.01);
    CHECK(std::abs(behind.speed.numerical_value_in(meter_per_second) - 0.8) <
          1e-3);
  }

  SECTION("ShouldDrawWalkingSpeedsAsWeidmann") {
    // 2,000 pedestrians on Town01's sidewalks: their speeds' mean and
    // spread, 1.34 and 0.26 m/s, within four standard errors.
    Simulation simulation{Scenario{.roads = find_road_path("Town01.xodr"),
                                   .vehicles = 0,
                                   .pedestrians = 2000}};
    REQUIRE(simulation.configure());
    double sum = 0.0;
    double squares = 0.0;
    int count = 0;
    simulation.world().store_of<Walker>().for_each(
        [&](Entity, const Walker& walker) {
          double v = walker.desired_speed.numerical_value_in(meter_per_second);
          sum += v;
          squares += v * v;
          ++count;
        });
    double mean = sum / count;
    double spread = std::sqrt(squares / count - mean * mean);
    CAPTURE(mean, spread);
    CHECK(count == 2000);
    CHECK(std::abs(mean - 1.34) < 4.0 * 0.26 / std::sqrt(count));
    CHECK(std::abs(spread - 0.26) < 0.02);
  }

  SECTION("ShouldWalkFromPlaceToPlaceGivenJunction") {
    // 60 pedestrians for 10 min at the signalized junction: every one walks
    // somewhere and on, and one half a meter or more behind another on the
    // same way never comes nearer. Two can meet nearer where they step onto
    // the same edge at once; the one behind then waits.
    Simulation simulation{Scenario{.seed = 4,
                                   .roads = find_road_path("signalized.xodr"),
                                   .vehicles = 0,
                                   .pedestrians = 60}};
    REQUIRE(simulation.configure());
    double nearest = std::numeric_limits<double>::infinity();
    std::map<std::pair<Entity, Entity>, double> spaced;  // Behind, ahead.
    for (long k = 0; k < 6000; ++k) {
      REQUIRE(
          simulation.step(Step{.time = TimePoint{} + k * 100ms, .dt = 100ms}));
      using Place = std::pair<double, Entity>;
      std::map<std::pair<std::uint32_t, bool>, std::vector<Place>> ways;
      const World& world = simulation.world();
      world.store_of<WalkState>().for_each(
          [&](Entity owner, const WalkState& state) {
            const road::Leg& leg =
                world.store_of<WalkRoute>().component_of(owner).legs[state.leg];
            ways[{leg.edge, leg.forward}].emplace_back(
                state.along.numerical_value_in(meter), owner);
          });
      std::map<std::pair<Entity, Entity>, double> now;
      for (auto& [way, places] : ways) {
        std::ranges::sort(places);
        for (std::size_t i = 1; i < places.size(); ++i) {
          now[{places[i - 1].second, places[i].second}] =
              places[i].first - places[i - 1].first;
        }
      }
      for (const auto& [pair, gap] : now) {
        auto before = spaced.find(pair);
        if (before != spaced.end() && before->second >= Pace::SPACE) {
          nearest = std::min(nearest, gap);
        }
      }
      spaced = std::move(now);
    }
    int trips = 0;
    int idle = 0;
    simulation.world().store_of<WalkRoute>().for_each(
        [&](Entity, const WalkRoute& route) {
          trips += static_cast<int>(route.trips);
          idle += route.trips == 0 ? 1 : 0;
        });
    CAPTURE(nearest, trips, idle);
    CHECK(idle == 0);
    CHECK(trips > 60);
    CHECK(nearest >= Pace::SPACE - 1e-9);
  }
}

}  // namespace simon::automotive
