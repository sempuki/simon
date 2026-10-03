// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/road.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "base/core.hpp"

namespace simon::model {

namespace {

// Gauss-Legendre quadrature on [-1, 1] with 8 points, exact for polynomials
// up to degree 15 (Abramowitz and Stegun, table 25.4; see
// model/REFERENCES.md).
constexpr std::array<double, 8> GAUSS_NODES{
    -0.9602898564975363, -0.7966664774136267, -0.5255324099163290,
    -0.1834346424956498, 0.1834346424956498,  0.5255324099163290,
    0.7966664774136267,  0.9602898564975363};
constexpr std::array<double, 8> GAUSS_WEIGHTS{
    0.1012285362903763, 0.2223810344533745, 0.3137066458778873,
    0.3626837833783620, 0.3626837833783620, 0.3137066458778873,
    0.2223810344533745, 0.1012285362903763};

// The integral of `f` over [from, to], in `pieces` equal parts.
template <typename FunctionType>
auto integrate(FunctionType f, double from, double to, int pieces)
    -> decltype(f(0.0)) {
  using Value = decltype(f(0.0));
  Value total{};
  double width = (to - from) / pieces;
  for (int piece = 0; piece < pieces; ++piece) {
    double middle = from + (piece + 0.5) * width;
    for (std::size_t i = 0; i < GAUSS_NODES.size(); ++i) {
      total = total + (0.5 * width * GAUSS_WEIGHTS[i]) *
                          f(middle + 0.5 * width * GAUSS_NODES[i]);
    }
  }
  return total;
}

// The quadrature's parts for a curve that turns `turn` radians: each part
// turns at most half a radian, where the 8-point rule's error is far below
// rounding.
auto pieces_for(double turn) -> int {
  return std::max(1, static_cast<int>(std::ceil(std::abs(turn) / 0.5)));
}

struct Planar final {
  double x = 0.0;
  double y = 0.0;
};

auto operator+(Planar a, Planar b) -> Planar { return {a.x + b.x, a.y + b.y}; }
auto operator*(double k, Planar a) -> Planar { return {k * a.x, k * a.y}; }

// sin(z) / z, without the division where z is near zero.
auto sinc(double z) -> double {
  return std::abs(z) < 1e-4 ? 1.0 - z * z / 6.0 : std::sin(z) / z;
}

auto plan_point(const PlanGeometry& g, double ds, const LineGeometry&)
    -> PlanPoint {
  return PlanPoint{.x = g.x0 + ds * std::cos(g.heading),
                   .y = g.y0 + ds * std::sin(g.heading),
                   .heading = g.heading};
}

// The chord of an arc turning k ds, written so it holds as k goes to zero:
// sin(h + k ds) - sin(h) = 2 cos(h + k ds / 2) sin(k ds / 2).
auto plan_point(const PlanGeometry& g, double ds, const ArcGeometry& arc)
    -> PlanPoint {
  double half = 0.5 * arc.curvature * ds;
  double chord = ds * sinc(half);
  return PlanPoint{.x = g.x0 + chord * std::cos(g.heading + half),
                   .y = g.y0 + chord * std::sin(g.heading + half),
                   .heading = g.heading + arc.curvature * ds,
                   .curvature = arc.curvature};
}

// The clothoid's position is the integral of its unit tangent, whose heading
// is quadratic in the distance along it; that is the Fresnel integral, here
// by quadrature rather than its series.
auto plan_point(const PlanGeometry& g, double ds, const SpiralGeometry& spiral)
    -> PlanPoint {
  double rate = g.length > 0.0
                    ? (spiral.curvature_end - spiral.curvature_start) / g.length
                    : 0.0;
  auto heading = [&](double u) {
    return g.heading + u * (spiral.curvature_start + 0.5 * rate * u);
  };
  double end_curvature = spiral.curvature_start + rate * ds;
  double largest =
      std::max(std::abs(spiral.curvature_start), std::abs(end_curvature));
  Planar moved = integrate(
      [&](double u) {
        double h = heading(u);
        return Planar{std::cos(h), std::sin(h)};
      },
      0.0, ds, pieces_for(largest * ds));
  return PlanPoint{.x = g.x0 + moved.x,
                   .y = g.y0 + moved.y,
                   .heading = heading(ds),
                   .curvature = end_curvature};
}

// The parameter of a parametric cubic `ds` along it, by arc length, by
// Newton's method on the arc length, the integral of the speed. A file's
// length and the curve's arc length often differ a little, so the arc length
// is scaled to the file's: s runs over the curve in proportion to its arc
// length, from its start at 0 to its end at `length`.
auto parameter_at(const ParamPoly3Geometry& curve, double range, double ds,
                  double length) -> double {
  auto speed = [&](double p) {
    return std::hypot(curve.u.slope(p), curve.v.slope(p));
  };
  constexpr int PIECES = 16;
  auto arc_length = [&](double p) {
    return integrate(speed, 0.0, p,
                     std::max(1, static_cast<int>(PIECES * p / range) + 1));
  };
  if (length <= 0.0) {
    return 0.0;
  }
  double target = ds * arc_length(range) / length;
  double p = ds / length * range;
  for (int i = 0; i < 50; ++i) {
    double step = (arc_length(p) - target) / std::max(speed(p), 1e-12);
    p -= step;
    if (std::abs(step) <= 1e-15 * range) {
      break;
    }
  }
  return p;
}

auto plan_point(const PlanGeometry& g, double ds,
                const ParamPoly3Geometry& curve) -> PlanPoint {
  double range = curve.normalized ? 1.0 : g.length;
  double p = parameter_at(curve, range, ds, g.length);
  double u = curve.u.evaluate(p);
  double v = curve.v.evaluate(p);
  double du = curve.u.slope(p);
  double dv = curve.v.slope(p);
  double ddu = 2.0 * curve.u.c + 6.0 * curve.u.d * p;
  double ddv = 2.0 * curve.v.c + 6.0 * curve.v.d * p;
  double speed = std::hypot(du, dv);
  double c = std::cos(g.heading);
  double s = std::sin(g.heading);
  return PlanPoint{
      .x = g.x0 + u * c - v * s,
      .y = g.y0 + u * s + v * c,
      .heading = g.heading + std::atan2(dv, du),
      .curvature =
          speed > 0.0 ? (du * ddv - dv * ddu) / (speed * speed * speed) : 0.0,
  };
}

// The piece of `plan` in force at `s`: the last that starts at or before it.
auto find_geometry(const std::vector<PlanGeometry>& plan, double s)
    -> const PlanGeometry& {
  CHECK_PRECONDITION(!plan.empty());
  auto after = std::ranges::upper_bound(plan, s, {}, &PlanGeometry::s0);
  return after == plan.begin() ? plan.front() : *std::prev(after);
}

// The width of `lane` at `ds` into its lane section.
auto lane_width(const Lane& lane, double ds) -> double {
  if (lane.widths.empty()) {
    return 0.0;
  }
  auto after =
      std::ranges::upper_bound(lane.widths, ds, {}, &Lane::Width::start);
  const Lane::Width& width =
      after == lane.widths.begin() ? lane.widths.front() : *std::prev(after);
  return width.cubic.evaluate(ds - width.start);
}

}  // namespace

auto CubicProfile::evaluate(double s) const -> double {
  if (pieces.empty()) {
    return 0.0;
  }
  auto after = std::ranges::upper_bound(pieces, s, {}, &Piece::start);
  const Piece& piece =
      after == pieces.begin() ? pieces.front() : *std::prev(after);
  return piece.cubic.evaluate(s - piece.start);
}

auto CubicProfile::slope(double s) const -> double {
  if (pieces.empty()) {
    return 0.0;
  }
  auto after = std::ranges::upper_bound(pieces, s, {}, &Piece::start);
  const Piece& piece =
      after == pieces.begin() ? pieces.front() : *std::prev(after);
  return piece.cubic.slope(s - piece.start);
}

auto compute_plan_point(const PlanGeometry& geometry, double ds) -> PlanPoint {
  return std::visit(
      [&](const auto& shape) { return plan_point(geometry, ds, shape); },
      geometry.shape);
}

auto RoadNetwork::find_road(std::string_view id) const -> const Road* {
  auto found = std::ranges::find(roads, id, &Road::id);
  return found == roads.end() ? nullptr : &*found;
}

auto compute_plan_point(const Road& road, Length s) -> PlanPoint {
  double at = s.numerical_value_in(meter);
  const PlanGeometry& geometry = find_geometry(road.plan, at);
  return compute_plan_point(geometry, at - geometry.s0);
}

// The surface's axes at s, as OpenDRIVE defines them: e_s along the reference
// line, rising with the elevation; e_t across it, the horizontal normal
// turned about e_s by the superelevation; and e_h normal to both. Turning the
// horizontal normal n by an angle about the unit vector e_s, to which it is
// perpendicular, gives cos(angle) n + sin(angle) e_s x n (Rodrigues).
auto compute_road_position(const Road& road, Length s, Length t, Length h)
    -> Position {
  double at = s.numerical_value_in(meter);
  PlanPoint point = compute_plan_point(road, s);
  Vector3 along{std::cos(point.heading), std::sin(point.heading),
                road.elevation.slope(at)};
  Vector3 e_s = along.normalized();
  Vector3 normal{-e_s.y(), e_s.x(), 0.0};
  double roll = road.superelevation.evaluate(at);
  Vector3 e_t = (std::cos(roll) * normal + std::sin(roll) * e_s.cross(normal))
                    .normalized();
  Vector3 e_h = along.cross(e_t).normalized();
  Vector3 position = Vector3{point.x, point.y, road.elevation.evaluate(at)} +
                     t.numerical_value_in(meter) * e_t +
                     h.numerical_value_in(meter) * e_h;
  return QuantityVector{position} * meter;
}

auto find_lane_section(const Road& road, Length s) -> const LaneSection& {
  CHECK_PRECONDITION(!road.lane_sections.empty());
  auto after = std::ranges::upper_bound(
      road.lane_sections, s.numerical_value_in(meter), {}, &LaneSection::s0);
  return after == road.lane_sections.begin() ? road.lane_sections.front()
                                             : *std::prev(after);
}

auto compute_lane_border(const Road& road, Length s, int id) -> Length {
  double at = s.numerical_value_in(meter);
  const LaneSection& section = find_lane_section(road, s);
  double ds = at - section.s0;
  double border = road.lane_offset.evaluate(at);
  const std::vector<Lane>& side = id > 0 ? section.left : section.right;
  auto count = static_cast<std::size_t>(std::abs(id));
  CHECK_PRECONDITION(count <= side.size());
  double sign = id > 0 ? 1.0 : -1.0;
  for (std::size_t i = 0; i < count; ++i) {
    border += sign * lane_width(side[i], ds);
  }
  return border * meter;
}

auto find_lane(const Road& road, Length s, Length t) -> std::optional<int> {
  double at = s.numerical_value_in(meter);
  double across = t.numerical_value_in(meter);
  const LaneSection& section = find_lane_section(road, s);
  double ds = at - section.s0;
  double center = road.lane_offset.evaluate(at);
  bool leftward =
      across > center || (across == center && !section.left.empty());
  const std::vector<Lane>& side = leftward ? section.left : section.right;
  double sign = leftward ? 1.0 : -1.0;
  double inner = center;
  for (const Lane& lane : side) {
    double outer = inner + sign * lane_width(lane, ds);
    if (sign * (across - inner) >= 0.0 && sign * (outer - across) > 0.0) {
      return lane.id;
    }
    inner = outer;
  }
  return std::nullopt;
}

// Each piece of the reference line is sampled for its nearest point, at most
// 2 m and a tenth of a radian apart, and the nearest refined by Newton's
// method on (P - r(s)) . T(s) = 0, whose derivative is
// -1 + k(s) (P - r(s)) . N(s), with T the tangent, N the normal and k the
// curvature.
auto find_road_coordinates(const Road& road, Length x, Length y)
    -> RoadCoordinates {
  double px = x.numerical_value_in(meter);
  double py = y.numerical_value_in(meter);
  double best_distance = std::numeric_limits<double>::infinity();
  RoadCoordinates best;
  for (const PlanGeometry& geometry : road.plan) {
    auto apart = [&](const PlanPoint& point) {
      return Planar{px - point.x, py - point.y};
    };
    PlanPoint start = compute_plan_point(geometry, 0.0);
    PlanPoint end = compute_plan_point(geometry, geometry.length);
    double turn = std::max(std::abs(start.curvature), std::abs(end.curvature)) *
                  geometry.length;
    int samples = std::max({8, static_cast<int>(geometry.length / 2.0) + 1,
                            static_cast<int>(turn / 0.1) + 1});
    double ds = 0.0;
    double nearest = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= samples; ++i) {
      double at = geometry.length * i / samples;
      Planar d = apart(compute_plan_point(geometry, at));
      double squared = d.x * d.x + d.y * d.y;
      if (squared < nearest) {
        nearest = squared;
        ds = at;
      }
    }
    for (int i = 0; i < 30; ++i) {
      PlanPoint point = compute_plan_point(geometry, ds);
      Planar d = apart(point);
      double c = std::cos(point.heading);
      double s = std::sin(point.heading);
      double along = d.x * c + d.y * s;
      double across = -d.x * s + d.y * c;
      double slope = -1.0 + point.curvature * across;
      double next = std::clamp(slope != 0.0 ? ds - along / slope : ds, 0.0,
                               geometry.length);
      bool settled = std::abs(next - ds) <= 1e-12 * (1.0 + geometry.length);
      ds = next;
      if (settled) {
        break;
      }
    }
    PlanPoint point = compute_plan_point(geometry, ds);
    Planar d = apart(point);
    double distance = std::hypot(d.x, d.y);
    if (distance < best_distance) {
      best_distance = distance;
      best = RoadCoordinates{
          .s = (geometry.s0 + ds) * meter,
          .t =
              (-d.x * std::sin(point.heading) + d.y * std::cos(point.heading)) *
              meter,
      };
    }
  }
  return best;
}

}  // namespace simon::model
