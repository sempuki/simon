// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Records libOpenDRIVE's reading of each road in application/automotive/roads,
// of CARLA's Town01 in 3rd_party/carla and of esmini's signed roads in
// 3rd_party/esmini/xodr, for opendrive_reference_test.
// libOpenDRIVE (https://github.com/pageldev/libOpenDRIVE, Apache-2.0) is not
// part of simon's build; build this beside it with CMake:
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
//   ./libopendrive_reference application/automotive/roads 3rd_party/carla \
//       3rd_party/esmini/xodr application/automotive/reference
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
//   libopendrive_signals.csv    each signal as read, a row per validity, and
//                               its position
//   libopendrive_objects.csv    each corner of each flat outline of each
//                               object, as read and where it is
//   libopendrive_junctions.csv  each junction's priorities and controllers
//
// It keeps the map's coordinates (center_map false), and reads lateral
// profiles but not lane heights, which simon leaves out.

#include <cstdio>
#include <string>
#include <vector>

#include "OpenDriveMap.h"
#include "RoutingGraph.h"

namespace {

enum class Directory { ROADS, CARLA, ESMINI };

struct File {
  std::string name;
  double step;  // m between samples.
  Directory directory = Directory::ROADS;
};

const char* orientation_of(const std::string& word) {
  return word == "+" ? "+" : word == "-" ? "-" : "none";
}

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
  if (argc != 5) {
    std::fprintf(stderr, "usage: %s ROADS_DIR CARLA_DIR ESMINI_DIR OUT_DIR\n",
                 argv[0]);
    return 1;
  }
  std::string roads = argv[1];
  std::string carla = argv[2];
  std::string esmini = argv[3];
  std::string out = argv[4];
  std::vector<File> files = {
      {"curves.xodr", 2.5},
      {"paramPoly3.xodr", 2.5},
      {"ring.xodr", 5.0},
      {"signalized.xodr", 5.0},
      {"priority.xodr", 5.0},
      {"crosswalks.xodr", 5.0},
      {"Town01.xodr", 5.0, Directory::CARLA},
      {"some_signs.xodr", 25.0, Directory::ESMINI},
      {"lane_offset_intersection.xodr", 10.0, Directory::ESMINI}};
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
  FILE* signals = std::fopen((out + "/libopendrive_signals.csv").c_str(), "w");
  std::fprintf(signals,
               "file,road,id,name,s,t,z_offset,dynamic,orientation,country,"
               "type,subtype,value,unit,from_lane,to_lane,x,y,z\n");
  FILE* objects = std::fopen((out + "/libopendrive_objects.csv").c_str(), "w");
  std::fprintf(objects,
               "file,road,id,type,s,t,z_offset,heading,pitch,roll,validities,"
               "outline,corner,x,y,z\n");
  FILE* junctions =
      std::fopen((out + "/libopendrive_junctions.csv").c_str(), "w");
  std::fprintf(junctions, "file,junction,kind,first,second,third\n");

  for (const File& file : files) {
    std::string directory = file.directory == Directory::CARLA    ? carla
                            : file.directory == Directory::ESMINI ? esmini
                                                                  : roads;
    odr::OpenDriveMap map(directory + "/" + file.name, false, true, true, false);
    const char* name = file.name.c_str();
    for (const odr::Junction& junction : map.get_junctions()) {
      for (const odr::JunctionPriority& priority : junction.priorities) {
        std::fprintf(junctions, "%s,%s,priority,%s,%s,\n", name,
                     junction.id.c_str(), priority.high.c_str(),
                     priority.low.c_str());
      }
      for (const auto& [id, controller] : junction.id_to_controller) {
        std::fprintf(junctions, "%s,%s,controller,%s,%s,%u\n", name,
                     junction.id.c_str(), id.c_str(), controller.type.c_str(),
                     controller.sequence);
      }
    }
    for (const odr::RoutingGraphEdge& edge : map.get_routing_graph().edges) {
      std::fprintf(successors, "%s,%s,%.17g,%d,%s,%.17g,%d\n", name,
                   edge.from.road_id.c_str(), edge.from.lanesection_s0,
                   edge.from.lane_id, edge.to.road_id.c_str(),
                   edge.to.lanesection_s0, edge.to.lane_id);
    }
    for (const odr::Road& road : map.get_roads()) {
      for (const odr::RoadSignal& signal : road.get_road_signals()) {
        odr::Vec3D p = road.get_xyz(signal.s0, signal.t0, signal.zOffset);
        std::vector<odr::LaneValidityRecord> validities = signal.lane_validities;
        if (validities.empty()) {
          validities.push_back({0, 0});  // Written as none.
        }
        for (const odr::LaneValidityRecord& validity : validities) {
          bool none = signal.lane_validities.empty();
          std::fprintf(
              signals,
              "%s,%s,%s,%s,%.17g,%.17g,%.17g,%d,%s,%s,%s,%s,%.17g,%s,%s,%s,"
              "%.17g,%.17g,%.17g\n",
              name, road.id.c_str(), signal.id.c_str(), signal.name.c_str(),
              signal.s0, signal.t0, signal.zOffset, signal.is_dynamic ? 1 : 0,
              orientation_of(signal.orientation), signal.country.c_str(),
              signal.type.c_str(), signal.subtype.c_str(), signal.value,
              signal.unit.c_str(),
              none ? "" : std::to_string(validity.from_lane).c_str(),
              none ? "" : std::to_string(validity.to_lane).c_str(), p[0], p[1],
              p[2]);
        }
      }
      for (const odr::RoadObject& object : road.get_road_objects()) {
        std::string validities;
        for (const odr::LaneValidityRecord& validity : object.lane_validities) {
          validities += (validities.empty() ? "" : ";") +
                        std::to_string(validity.from_lane) + ":" +
                        std::to_string(validity.to_lane);
        }
        for (std::size_t o = 0; o < object.outlines.size(); ++o) {
          const odr::RoadObjectOutline& outline = object.outlines[o];
          bool flat = true;
          for (const odr::RoadObjectCorner& corner : outline.outline) {
            flat = flat && corner.height == 0.0;
          }
          if (!flat || outline.outline.size() < 2) {
            continue;
          }
          odr::RoadObject alone = object;
          alone.outlines = {outline};
          alone.repeats.clear();
          odr::Mesh3D mesh = road.get_road_object_mesh(alone, 0.1);
          for (std::size_t c = 0; c < mesh.vertices.size(); ++c) {
            const odr::Vec3D& p = mesh.vertices[c];
            std::fprintf(objects,
                         "%s,%s,%s,%s,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%s,%zu,"
                         "%zu,%.17g,%.17g,%.17g\n",
                         name, road.id.c_str(), object.id.c_str(),
                         object.type.c_str(), object.s0, object.t0, object.z0,
                         object.hdg, object.pitch, object.roll,
                         validities.c_str(), o, c, p[0], p[1], p[2]);
          }
        }
      }
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
  std::fclose(signals);
  std::fclose(objects);
  std::fclose(junctions);
  return 0;
}
