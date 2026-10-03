// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Records libOpenDRIVE's geometry of each road in application/automotive/roads,
// for opendrive_reference_test. libOpenDRIVE
// (https://github.com/pageldev/libOpenDRIVE, Apache-2.0) is not part of
// simon's build; build this beside it with CMake:
//
//   git clone https://github.com/pageldev/libOpenDRIVE
//   cmake -S libOpenDRIVE -B libOpenDRIVE/build -DCMAKE_BUILD_TYPE=Release
//   cmake --build libOpenDRIVE/build
//   g++ -std=c++17 -O2 -IlibOpenDRIVE/include \
//       -IlibOpenDRIVE/build/_deps/pugixml-src/src \
//       application/automotive/reference/libopendrive_reference.cpp \
//       libOpenDRIVE/build/libOpenDrive.a \
//       libOpenDRIVE/build/_deps/pugixml-build/libpugixml.a \
//       -o libopendrive_reference
//   ./libopendrive_reference application/automotive/roads application/automotive/reference
//
// For each road it samples s every `step` meters and at the road's end, and
// writes three tables:
//
//   libopendrive_positions.csv  x, y and z of (s, t, h) at several t and h
//   libopendrive_borders.csv    each lane's outer border at s
//   libopendrive_lanes.csv      the lane libOpenDRIVE finds at each lane's
//                               middle at s
//   libopendrive_successors.csv every edge of its routing graph: each lane
//                               and a lane traffic moves into from it
//
// It keeps the map's coordinates (center_map false), and reads lateral
// profiles but not lane heights, which simon leaves out.

#include <cstdio>
#include <string>
#include <vector>

#include "OpenDriveMap.h"
#include "RoutingGraph.h"

namespace {

struct File {
  std::string name;
  double step;  // m between samples.
};

std::vector<double> samples(double length, double step) {
  std::vector<double> s;
  for (double at = 0.0; at < length; at += step) {
    s.push_back(at);
  }
  s.push_back(length);
  return s;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s ROADS_DIR OUT_DIR\n", argv[0]);
    return 1;
  }
  std::string roads = argv[1];
  std::string out = argv[2];
  std::vector<File> files = {
      {"curves.xodr", 2.5}, {"paramPoly3.xodr", 2.5}, {"ring.xodr", 5.0},
      {"Town01.xodr", 5.0}};
  const double ts[] = {-6.0, -2.5, 0.0, 1.75, 5.0};

  FILE* positions = std::fopen((out + "/libopendrive_positions.csv").c_str(), "w");
  FILE* borders = std::fopen((out + "/libopendrive_borders.csv").c_str(), "w");
  FILE* lanes = std::fopen((out + "/libopendrive_lanes.csv").c_str(), "w");
  std::fprintf(positions, "file,road,s,t,h,x,y,z\n");
  std::fprintf(borders, "file,road,s,lane,t\n");
  std::fprintf(lanes, "file,road,s,t,lane\n");
  FILE* successors =
      std::fopen((out + "/libopendrive_successors.csv").c_str(), "w");
  std::fprintf(successors,
               "file,from_road,from_section,from_lane,to_road,to_section,to_lane\n");

  for (const File& file : files) {
    odr::OpenDriveMap map(roads + "/" + file.name, false, false, true, false);
    for (const odr::RoutingGraphEdge& edge : map.get_routing_graph().edges) {
      std::fprintf(successors, "%s,%s,%.17g,%d,%s,%.17g,%d\n", file.name.c_str(),
                   edge.from.road_id.c_str(), edge.from.lanesection_s0,
                   edge.from.lane_id, edge.to.road_id.c_str(),
                   edge.to.lanesection_s0, edge.to.lane_id);
    }
    for (const odr::Road& road : map.get_roads()) {
      for (double s : samples(road.length, file.step)) {
        for (double t : ts) {
          odr::Vec3D p = road.get_xyz(s, t, 0.0);
          std::fprintf(positions, "%s,%s,%.17g,%.17g,0,%.17g,%.17g,%.17g\n",
                       file.name.c_str(), road.id.c_str(), s, t, p[0], p[1], p[2]);
        }
        odr::Vec3D up = road.get_xyz(s, 2.0, 1.5);
        std::fprintf(positions, "%s,%s,%.17g,2,1.5,%.17g,%.17g,%.17g\n",
                     file.name.c_str(), road.id.c_str(), s, up[0], up[1], up[2]);

        odr::LaneSection section = road.get_lanesection(s);
        for (const odr::Lane& lane : section.get_lanes()) {
          double outer = lane.outer_border.evaluate(s);
          std::fprintf(borders, "%s,%s,%.17g,%d,%.17g\n", file.name.c_str(),
                       road.id.c_str(), s, lane.id, outer);
          if (lane.id == 0) {
            continue;
          }
          int inner_id = lane.id > 0 ? lane.id - 1 : lane.id + 1;
          double inner = section.get_lane(inner_id).outer_border.evaluate(s);
          if (outer == inner) {
            continue;  // No width here, so no middle.
          }
          double middle = 0.5 * (inner + outer);
          std::fprintf(lanes, "%s,%s,%.17g,%.17g,%d\n", file.name.c_str(),
                       road.id.c_str(), s, middle,
                       section.get_lane(s, middle).id);
        }
      }
    }
  }
  std::fclose(positions);
  std::fclose(borders);
  std::fclose(lanes);
  std::fclose(successors);
  return 0;
}
