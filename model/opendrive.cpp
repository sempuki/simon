// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/opendrive.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <sstream>
#include <utility>

#include "pugixml.hpp"

template <>
const std::array<lib::StatusConditionEntry,
                 simon::model::OPEN_DRIVE_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::model::OpenDriveError,
        simon::model::OPEN_DRIVE_ERROR_COUNT>::conditions_ = {
        lib::StatusConditionEntry{"road network unreadable"},
        lib::StatusConditionEntry{"road network malformed"},
};

namespace simon::model {

namespace {

using Failure = std::unexpected<lib::Status>;

// Reads one document, saying where in the text anything is wrong.
class Parser final {
 public:
  explicit Parser(std::string_view text) : text_{text} {}

  auto parse() -> std::expected<RoadNetwork, lib::Status> {
    pugi::xml_parse_result result =
        document_.load_buffer(text_.data(), text_.size());
    if (!result) {
      return fail(result.offset, result.description());
    }
    pugi::xml_node root = document_.child("OpenDRIVE");
    if (!root) {
      return fail(0, "no OpenDRIVE element");
    }
    RoadNetwork network;
    for (pugi::xml_node node : root.children("road")) {
      RETURN_OR_ASSIGN(Road road, read_road(node));
      network.roads.push_back(std::move(road));
    }
    for (pugi::xml_node node : root.children("junction")) {
      RETURN_OR_ASSIGN(Junction junction, read_junction(node));
      network.junctions.push_back(std::move(junction));
    }
    return network;
  }

 private:
  // The line of `offset` into the text, from 1.
  auto line_at(std::ptrdiff_t offset) const -> std::size_t {
    auto end = static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(
        offset, 0, static_cast<std::ptrdiff_t>(text_.size())));
    return 1 + static_cast<std::size_t>(
                   std::count(text_.begin(), text_.begin() + end, '\n'));
  }

  auto fail(std::ptrdiff_t offset, std::string_view why) const -> Failure {
    return Failure{lib::raise(
        OpenDriveError::MALFORMED,
        "line " + std::to_string(line_at(offset)) + ": " + std::string{why})};
  }

  auto fail(pugi::xml_node node, std::string_view why) const -> Failure {
    return fail(node.offset_debug(),
                "<" + std::string{node.name()} + "> " + std::string{why});
  }

  // The number in `node`'s attribute `name`, or `fallback` if it has none.
  auto read_number(pugi::xml_node node, const char* name,
                   std::optional<double> fallback = std::nullopt) const
      -> std::expected<double, lib::Status> {
    pugi::xml_attribute attribute = node.attribute(name);
    if (!attribute) {
      if (fallback) {
        return *fallback;
      }
      return fail(node, std::string{"needs "} + name);
    }
    std::string_view word = attribute.value();
    double value = 0.0;
    auto [end, error] =
        std::from_chars(word.data(), word.data() + word.size(), value);
    if (error != std::errc{} || end != word.data() + word.size() ||
        !std::isfinite(value)) {
      return fail(node, std::string{name} + " `" + std::string{word} +
                            "` is not a finite number");
    }
    return value;
  }

  auto read_contact(pugi::xml_node node, std::string_view value) const
      -> std::expected<RoadLink::Contact, lib::Status> {
    if (value == "start") {
      return RoadLink::Contact::START;
    }
    if (value == "end") {
      return RoadLink::Contact::END;
    }
    return fail(node, "contactPoint `" + std::string{value} +
                          "` is neither start nor end");
  }

  // The road link `node` describes, or none if there is no node.
  auto read_road_link(pugi::xml_node node) const
      -> std::expected<RoadLink, lib::Status> {
    RoadLink link;
    if (!node) {
      return link;
    }
    std::string_view kind = node.attribute("elementType").as_string();
    link.id = node.attribute("elementId").as_string();
    if (link.id.empty()) {
      return fail(node, "needs elementId");
    }
    if (kind == "road") {
      link.kind = RoadLink::Kind::ROAD;
      RETURN_OR_ASSIGN(
          link.contact,
          read_contact(node, node.attribute("contactPoint").as_string()));
    } else if (kind == "junction") {
      link.kind = RoadLink::Kind::JUNCTION;
    } else {
      return fail(node, "elementType `" + std::string{kind} +
                            "` is neither road nor junction");
    }
    return link;
  }

  // The lane id in `node`'s attribute `name`, if it has one.
  auto read_lane_id(pugi::xml_node node, const char* name) const
      -> std::expected<std::optional<int>, lib::Status> {
    if (!node || !node.attribute(name)) {
      return std::optional<int>{};
    }
    RETURN_OR_ASSIGN(double id, read_number(node, name));
    if (static_cast<int>(id) != id) {
      return fail(node, std::string{name} + " is not a whole number");
    }
    return std::optional<int>{static_cast<int>(id)};
  }

  auto read_junction(pugi::xml_node node) const
      -> std::expected<Junction, lib::Status> {
    Junction junction;
    junction.id = node.attribute("id").as_string();
    if (junction.id.empty()) {
      return fail(node, "needs id");
    }
    for (pugi::xml_node connection_node : node.children("connection")) {
      JunctionConnection connection;
      connection.incoming_road =
          connection_node.attribute("incomingRoad").as_string();
      connection.connecting_road =
          connection_node.attribute("connectingRoad").as_string();
      if (connection.incoming_road.empty() ||
          connection.connecting_road.empty()) {
        return fail(connection_node, "needs incomingRoad and connectingRoad");
      }
      RETURN_OR_ASSIGN(
          connection.contact,
          read_contact(connection_node,
                       connection_node.attribute("contactPoint").as_string()));
      for (pugi::xml_node link_node : connection_node.children("laneLink")) {
        RETURN_OR_ASSIGN(std::optional<int> from,
                         read_lane_id(link_node, "from"));
        RETURN_OR_ASSIGN(std::optional<int> to, read_lane_id(link_node, "to"));
        if (!from || !to) {
          return fail(link_node, "needs from and to");
        }
        connection.lane_links.push_back({.from = *from, .to = *to});
      }
      junction.connections.push_back(std::move(connection));
    }
    return junction;
  }

  auto read_cubic(pugi::xml_node node) const
      -> std::expected<Cubic, lib::Status> {
    Cubic cubic;
    RETURN_OR_ASSIGN(cubic.a, read_number(node, "a", 0.0));
    RETURN_OR_ASSIGN(cubic.b, read_number(node, "b", 0.0));
    RETURN_OR_ASSIGN(cubic.c, read_number(node, "c", 0.0));
    RETURN_OR_ASSIGN(cubic.d, read_number(node, "d", 0.0));
    return cubic;
  }

  // Appends to `profile` each of `parent`'s `name` children: a start `s` and
  // a cubic, ordered by start.
  auto read_profile(pugi::xml_node parent, const char* name,
                    InOut<CubicProfile> profile) const
      -> std::expected<void, lib::Status> {
    for (pugi::xml_node node : parent.children(name)) {
      CubicProfile::Piece piece;
      RETURN_OR_ASSIGN(piece.start, read_number(node, "s"));
      RETURN_OR_ASSIGN(piece.cubic, read_cubic(node));
      profile->pieces.push_back(piece);
    }
    std::ranges::stable_sort(profile->pieces, {}, &CubicProfile::Piece::start);
    return {};
  }

  auto read_geometry(pugi::xml_node node) const
      -> std::expected<PlanGeometry, lib::Status> {
    PlanGeometry geometry;
    RETURN_OR_ASSIGN(geometry.s0, read_number(node, "s"));
    RETURN_OR_ASSIGN(geometry.x0, read_number(node, "x"));
    RETURN_OR_ASSIGN(geometry.y0, read_number(node, "y"));
    RETURN_OR_ASSIGN(geometry.heading, read_number(node, "hdg"));
    RETURN_OR_ASSIGN(geometry.length, read_number(node, "length"));
    if (geometry.length < 0.0) {
      return fail(node, "has a negative length");
    }
    pugi::xml_node shape = node.first_child();
    while (shape && shape.type() != pugi::node_element) {
      shape = shape.next_sibling();
    }
    if (!shape) {
      return fail(node, "has no shape");
    }
    std::string_view kind = shape.name();
    if (kind == "line") {
      geometry.shape = LineGeometry{};
    } else if (kind == "arc") {
      ArcGeometry arc;
      RETURN_OR_ASSIGN(arc.curvature, read_number(shape, "curvature"));
      geometry.shape = arc;
    } else if (kind == "spiral") {
      SpiralGeometry spiral;
      RETURN_OR_ASSIGN(spiral.curvature_start, read_number(shape, "curvStart"));
      RETURN_OR_ASSIGN(spiral.curvature_end, read_number(shape, "curvEnd"));
      geometry.shape = spiral;
    } else if (kind == "paramPoly3") {
      ParamPoly3Geometry curve;
      RETURN_OR_ASSIGN(curve.u.a, read_number(shape, "aU", 0.0));
      RETURN_OR_ASSIGN(curve.u.b, read_number(shape, "bU", 0.0));
      RETURN_OR_ASSIGN(curve.u.c, read_number(shape, "cU", 0.0));
      RETURN_OR_ASSIGN(curve.u.d, read_number(shape, "dU", 0.0));
      RETURN_OR_ASSIGN(curve.v.a, read_number(shape, "aV", 0.0));
      RETURN_OR_ASSIGN(curve.v.b, read_number(shape, "bV", 0.0));
      RETURN_OR_ASSIGN(curve.v.c, read_number(shape, "cV", 0.0));
      RETURN_OR_ASSIGN(curve.v.d, read_number(shape, "dV", 0.0));
      std::string_view range =
          shape.attribute("pRange").as_string("normalized");
      if (range == "arcLength") {
        curve.normalized = false;
      } else if (range != "normalized") {
        return fail(shape, "pRange `" + std::string{range} +
                               "` is neither arcLength nor normalized");
      }
      geometry.shape = curve;
    } else if (kind == "poly3") {
      return fail(shape, "is deprecated and not supported; use paramPoly3");
    } else {
      return fail(shape, "is not a geometry");
    }
    return geometry;
  }

  auto read_lane(pugi::xml_node node) const
      -> std::expected<Lane, lib::Status> {
    Lane lane;
    RETURN_OR_ASSIGN(double id, read_number(node, "id"));
    lane.id = static_cast<int>(id);
    if (lane.id != id) {
      return fail(node, "id is not a whole number");
    }
    lane.type = node.attribute("type").as_string("none");
    if (node.child("border")) {
      return fail(node.child("border"),
                  "is not supported; give lanes by their widths");
    }
    RETURN_OR_ASSIGN(
        lane.predecessor,
        read_lane_id(node.child("link").child("predecessor"), "id"));
    RETURN_OR_ASSIGN(lane.successor,
                     read_lane_id(node.child("link").child("successor"), "id"));
    for (pugi::xml_node width_node : node.children("width")) {
      Lane::Width width;
      RETURN_OR_ASSIGN(width.start, read_number(width_node, "sOffset", 0.0));
      if (width.start < 0.0) {
        return fail(width_node, "has a negative sOffset");
      }
      RETURN_OR_ASSIGN(width.cubic, read_cubic(width_node));
      lane.widths.push_back(width);
    }
    std::ranges::stable_sort(lane.widths, {}, &Lane::Width::start);
    return lane;
  }

  // Reads one side's lanes into `lanes`, ordered from the center out, and
  // checks their ids run from 1 or -1 without a gap.
  auto read_side(pugi::xml_node side, int sign,
                 Out<std::vector<Lane>> lanes) const
      -> std::expected<void, lib::Status> {
    lanes->clear();
    for (pugi::xml_node node : side.children("lane")) {
      RETURN_OR_ASSIGN(Lane lane, read_lane(node));
      if (lane.id * sign <= 0) {
        return fail(node, "id " + std::to_string(lane.id) +
                              " is on the wrong side of the center");
      }
      lanes->push_back(std::move(lane));
    }
    std::ranges::sort(*lanes, {},
                      [&](const Lane& lane) { return lane.id * sign; });
    for (std::size_t i = 0; i < lanes->size(); ++i) {
      if ((*lanes)[i].id != sign * static_cast<int>(i + 1)) {
        return fail(side, "lane ids must run from " + std::to_string(sign) +
                              " without a gap");
      }
    }
    return {};
  }

  auto read_road(pugi::xml_node node) const
      -> std::expected<Road, lib::Status> {
    Road road;
    road.id = node.attribute("id").as_string();
    if (road.id.empty()) {
      return fail(node, "needs id");
    }
    road.junction = node.attribute("junction").as_string("-1");
    RETURN_OR_ASSIGN(road.length, read_number(node, "length"));
    RETURN_OR_ASSIGN(road.predecessor,
                     read_road_link(node.child("link").child("predecessor")));
    RETURN_OR_ASSIGN(road.successor,
                     read_road_link(node.child("link").child("successor")));

    for (pugi::xml_node geometry :
         node.child("planView").children("geometry")) {
      RETURN_OR_ASSIGN(PlanGeometry read, read_geometry(geometry));
      road.plan.push_back(std::move(read));
    }
    if (road.plan.empty()) {
      return fail(node, "has no planView geometry");
    }
    std::ranges::stable_sort(road.plan, {}, &PlanGeometry::s0);

    RETURN_IF_UNEXPECTED(read_profile(node.child("elevationProfile"),
                                      "elevation", InOut(road.elevation)));
    RETURN_IF_UNEXPECTED(read_profile(node.child("lateralProfile"),
                                      "superelevation",
                                      InOut(road.superelevation)));
    pugi::xml_node lanes = node.child("lanes");
    RETURN_IF_UNEXPECTED(
        read_profile(lanes, "laneOffset", InOut(road.lane_offset)));
    for (pugi::xml_node section_node : lanes.children("laneSection")) {
      LaneSection section;
      RETURN_OR_ASSIGN(section.s0, read_number(section_node, "s"));
      RETURN_IF_UNEXPECTED(
          read_side(section_node.child("left"), 1, Out(section.left)));
      RETURN_IF_UNEXPECTED(
          read_side(section_node.child("right"), -1, Out(section.right)));
      road.lane_sections.push_back(std::move(section));
    }
    if (road.lane_sections.empty()) {
      return fail(node, "has no laneSection");
    }
    std::ranges::stable_sort(road.lane_sections, {}, &LaneSection::s0);
    return road;
  }

  std::string_view text_;
  pugi::xml_document document_;
};

}  // namespace

auto parse_opendrive(std::string_view text)
    -> std::expected<RoadNetwork, lib::Status> {
  return Parser{text}.parse();
}

auto load_opendrive(const std::string& path)
    -> std::expected<RoadNetwork, lib::Status> {
  std::ifstream file{path};
  if (!file) {
    return std::unexpected(
        lib::raise(OpenDriveError::UNREADABLE, "cannot open " + path));
  }
  std::stringstream text;
  text << file.rdbuf();
  return parse_opendrive(text.str());
}

}  // namespace simon::model
