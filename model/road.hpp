// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "model/units.hpp"

// Roads as ASAM OpenDRIVE describes them (see model/REFERENCES.md): a
// reference line in the plane, built of lines, arcs, spirals and parametric
// cubics, with an elevation and a superelevation along it, and lanes beside
// it whose widths are cubics in the distance along the road.
//
// A point on a road is (s, t, h): s along the reference line, measured in the
// plane; t across it, positive to the left; h up from the road's surface. The
// world's frame is the map's: x east, y north, z up. The data is in plain SI
// numbers, meters and radians, since a cubic's coefficients carry different
// units; the functions take and give quantities.
namespace simon::model {

// a + b ds + c ds^2 + d ds^3, in the distance ds from where it starts.
struct Cubic final {
  auto evaluate(double ds) const -> double {
    return a + ds * (b + ds * (c + ds * d));
  }
  auto slope(double ds) const -> double {
    return b + ds * (2.0 * c + ds * 3.0 * d);
  }

  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
  double d = 0.0;
};

// A cubic for each stretch of s, each holding from its start to the next
// one's. Before the first start the first piece holds; with no pieces the
// profile is zero.
struct CubicProfile final {
  struct Piece final {
    double start = 0.0;  // s, in m.
    Cubic cubic;
  };

  auto evaluate(double s) const -> double;
  auto slope(double s) const -> double;

  std::vector<Piece> pieces;  // In increasing start.
};

// The shapes of the reference line's pieces.
struct LineGeometry final {};

struct ArcGeometry final {
  double curvature = 0.0;  // 1/m, positive turning left.
};

// Curvature changing linearly along the piece, from `curvature_start` to
// `curvature_end`: a clothoid.
struct SpiralGeometry final {
  double curvature_start = 0.0;
  double curvature_end = 0.0;
};

// u(p) along the piece's start heading and v(p) to its left, cubics in the
// parameter p. p runs over [0, length] in meters, or over [0, 1] if
// `normalized`. s is the arc length along the curve, as it is everywhere on
// the reference line, so p follows from s through the curve's arc length,
// scaled to the piece's length where a file's length and the arc length
// differ, so that s = length is the curve's end.
struct ParamPoly3Geometry final {
  Cubic u;
  Cubic v;
  bool normalized = true;
};

// One piece of the reference line, from `s0` for `length`, starting at
// (`x0`, `y0`) and heading `heading`, counterclockwise from east.
struct PlanGeometry final {
  double s0 = 0.0;
  double x0 = 0.0;
  double y0 = 0.0;
  double heading = 0.0;
  double length = 0.0;
  std::variant<LineGeometry, ArcGeometry, SpiralGeometry, ParamPoly3Geometry>
      shape;
};

// A point on the reference line, in the plane: where it is, the heading of
// its tangent, and its curvature.
struct PlanPoint final {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double curvature = 0.0;
};

// The point `ds` meters along `geometry` from its start.
auto compute_plan_point(const PlanGeometry& geometry, double ds) -> PlanPoint;

struct Lane final {
  // Each width holds from its `start`, measured from the lane section's
  // start, to the next width's.
  struct Width final {
    double start = 0.0;
    Cubic cubic;
  };

  int id = 0;  // Positive to the left of the reference line, negative right.
  std::string type;  // As OpenDRIVE names it: driving, shoulder, sidewalk...
  std::vector<Width> widths;
  // The lanes this one continues from and into, by s: in the lane section
  // before and after, or across the road's link at either end.
  std::optional<int> predecessor;
  std::optional<int> successor;
};

// The lanes from `s0` to the next section: left ones in increasing id from
// 1, right ones in decreasing id from -1. The center lane, 0, has no width.
struct LaneSection final {
  double s0 = 0.0;
  std::vector<Lane> left;
  std::vector<Lane> right;
};

// What a road's end joins: nothing, another road's start or end, or a
// junction.
struct RoadLink final {
  enum class Kind : std::uint8_t { NONE, ROAD, JUNCTION };
  enum class Contact : std::uint8_t { START, END };

  Kind kind = Kind::NONE;
  std::string id;
  Contact contact = Contact::START;  // The joined road's end, for a road.
};

// The direction of travel along s that a signal or object holds for: "+",
// "-" or "none" in OpenDRIVE, for traffic toward increasing s, decreasing s,
// or both.
enum class RoadDirection : std::uint8_t { POSITIVE, NEGATIVE, BOTH };

// The lanes a signal or object holds for, by id, from `from` to `to`.
struct LaneValidity final {
  int from = 0;
  int to = 0;
};

// A sign or a signal at (s, t), its face `z_offset` above the road. A dynamic
// one, such as a traffic light, changes what it shows; a controller groups
// them. `country`, `type` and `subtype` name it in a country's catalog, such
// as Germany's StVO, where type 1000001 is a traffic light and 206 a stop
// sign. With no validity it holds for every lane in its orientation.
struct Signal final {
  std::string id;
  std::string name;
  std::string country;
  std::string type;
  std::string subtype;
  std::string unit;
  std::optional<double> value;
  std::vector<LaneValidity> validities;
  double s = 0.0;
  double t = 0.0;
  double z_offset = 0.0;
  RoadDirection orientation = RoadDirection::BOTH;
  bool dynamic = false;
};

// An object on or beside a road, such as a crosswalk, at (s, t) and
// `z_offset` up, turned by `heading`, `pitch` and `roll` from the road's axes
// there. Its outlines give its shape, each corner either in road coordinates
// (s, t and dz up from the road) or in the object's own (u, v and z).
struct RoadObject final {
  struct Corner final {
    double first = 0.0;   // s or u.
    double second = 0.0;  // t or v.
    double up = 0.0;      // dz or z.
    double height = 0.0;
  };

  struct Outline final {
    enum class Frame : std::uint8_t { ROAD, LOCAL };

    std::vector<Corner> corners;
    Frame frame = Frame::ROAD;
    bool closed = true;
  };

  std::string id;
  std::string name;
  std::string type;  // As OpenDRIVE names it: crosswalk, pole, building...
  std::string subtype;
  std::vector<Outline> outlines;
  std::vector<LaneValidity> validities;
  double s = 0.0;
  double t = 0.0;
  double z_offset = 0.0;
  double heading = 0.0;
  double pitch = 0.0;
  double roll = 0.0;
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;
  RoadDirection orientation = RoadDirection::BOTH;
};

struct Road final {
  std::string id;
  std::string junction;  // "-1" if the road is in none.
  double length = 0.0;
  std::vector<PlanGeometry> plan;  // In increasing s0.
  CubicProfile elevation;
  CubicProfile superelevation;             // rad, about the reference line.
  CubicProfile lane_offset;                // The center lane's t.
  std::vector<LaneSection> lane_sections;  // In increasing s0.
  std::vector<Signal> signals;
  std::vector<RoadObject> objects;
  RoadLink predecessor;  // At s = 0.
  RoadLink successor;    // At s = length.
};

// Where traffic from an incoming road enters a junction: a connecting road,
// entered at its start or end, and which of the incoming road's lanes lead
// into which of the connecting road's.
struct JunctionConnection final {
  struct LaneLink final {
    int from = 0;
    int to = 0;
  };

  std::string incoming_road;
  std::string connecting_road;
  RoadLink::Contact contact = RoadLink::Contact::START;
  std::vector<LaneLink> lane_links;
};

// Within a junction, traffic on connecting road `high` goes before traffic
// on connecting road `low` where they meet.
struct JunctionPriority final {
  std::string high;
  std::string low;
};

// A controller whose signals govern a junction, and its place in the order
// the junction's controllers run in.
struct JunctionController final {
  std::string id;
  std::string type;
  std::uint32_t sequence = 0;
};

struct Junction final {
  std::string id;
  std::vector<JunctionConnection> connections;
  std::vector<JunctionPriority> priorities;
  std::vector<JunctionController> controllers;
};

// Signals that change together, such as a junction's lights for one approach,
// by their ids. OpenDRIVE says which signals a controller groups, not when
// they change.
struct SignalController final {
  struct Control final {
    std::string signal;
    std::string type;
  };

  std::string id;
  std::string name;
  std::vector<Control> controls;
  std::uint32_t sequence = 0;
};

struct RoadNetwork final {
  // The road with `id`, if there is one.
  auto find_road(std::string_view id) const -> const Road*;

  // The junction with `id`, if there is one.
  auto find_junction(std::string_view id) const -> const Junction*;

  std::vector<Road> roads;
  std::vector<Junction> junctions;
  std::vector<SignalController> controllers;
};

// The reference line's point at `s`.
auto compute_plan_point(const Road& road, Length s) -> PlanPoint;

// The world position of (`s`, `t`, `h`) on `road`. t runs along the road's
// surface across the reference line, tilted by the superelevation, and h
// along the surface's normal.
auto compute_road_position(const Road& road, Length s, Length t,
                           Length h = 0.0 * meter) -> Position;

// The same, given the reference line's point at `s`.
auto compute_road_position(const Road& road, const PlanPoint& point, Length s,
                           Length t, Length h = 0.0 * meter) -> Position;

// The world positions of `outline`'s corners on `object` on `road`, on the
// road's surface: road corners at (s, t, dz) on the road, and local ones
// turned by the object's heading, pitch and roll from the road's axes at the
// object's origin, and carried there.
auto compute_outline(const Road& road, const RoadObject& object,
                     const RoadObject::Outline& outline)
    -> std::vector<Position>;

// The lane section in force at `s`: the last that starts at or before it.
auto find_lane_section(const Road& road, Length s) -> const LaneSection&;

// The t of lane `id`'s outer border at `s`, or of the center lane for id 0:
// in the lane section in force at s, or in `section`, even at its end.
auto compute_lane_border(const Road& road, Length s, int id) -> Length;
auto compute_lane_border(const Road& road, const LaneSection& section, Length s,
                         int id) -> Length;

// The lane at (`s`, `t`): the one whose borders hold t, its inner border
// included. None beyond the outermost border.
auto find_lane(const Road& road, Length s, Length t) -> std::optional<int>;

// Road coordinates in the plane: s along the reference line and t along its
// normal.
struct RoadCoordinates final {
  Length s = 0.0 * meter;
  Length t = 0.0 * meter;
};

// The road coordinates of (`x`, `y`): the s on the reference line nearest it,
// where the line's normal passes through it, and the t along that normal.
auto find_road_coordinates(const Road& road, Length x, Length y)
    -> RoadCoordinates;

}  // namespace simon::model
