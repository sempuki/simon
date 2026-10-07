// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "format/opendrive.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include "base/core.hpp"
#include "format/text.hpp"
#include "format/xml.hpp"
#include "pugixml.hpp"

namespace simon::format {

using namespace road;

namespace {

using lib::InOut;
using lib::Out;

// Reads one document, saying on which line anything is wrong.
class Parser final {
 public:
  auto parse(std::string_view text) -> std::expected<road::Map, lib::Status> {
    RETURN_IF_UNEXPECTED(document_.load(std::string{text}));
    RETURN_OR_ASSIGN(pugi::xml_node root, document_.find_root("OpenDRIVE"));
    road::Map network;
    for (pugi::xml_node node : root.children("road")) {
      RETURN_OR_ASSIGN(Road road, read_road(node));
      network.roads.push_back(std::move(road));
    }
    for (pugi::xml_node node : root.children("junction")) {
      RETURN_OR_ASSIGN(Junction junction, read_junction(node));
      network.junctions.push_back(std::move(junction));
    }
    for (pugi::xml_node node : root.children("controller")) {
      RETURN_OR_ASSIGN(SignalController controller, read_controller(node));
      network.controllers.push_back(std::move(controller));
    }
    return network;
  }

 private:
  auto fail(pugi::xml_node node, std::string_view why) const -> Failure {
    return document_.fail(node, why);
  }

  // The number in `node`'s attribute `name`, or `fallback` if it has none.
  auto read_number(pugi::xml_node node, const char* name,
                   std::optional<double> fallback = std::nullopt) const
      -> std::expected<double, lib::Status> {
    if (fallback && !node.attribute(name)) {
      return *fallback;
    }
    return document_.read_number(node, name);
  }

  auto read_contact(pugi::xml_node node, std::string_view value) const
      -> std::expected<road::Link::Contact, lib::Status> {
    if (value == "start") {
      return road::Link::Contact::START;
    }
    if (value == "end") {
      return road::Link::Contact::END;
    }
    return fail(node, "contactPoint `" + std::string{value} +
                          "` is neither start nor end");
  }

  // The road link `node` describes, or none if there is no node.
  auto read_road_link(pugi::xml_node node) const
      -> std::expected<road::Link, lib::Status> {
    road::Link link;
    if (!node) {
      return link;
    }
    std::string_view kind = node.attribute("elementType").as_string();
    link.id = node.attribute("elementId").as_string();
    if (link.id.empty()) {
      return fail(node, "needs elementId");
    }
    if (kind == "road") {
      link.kind = road::Link::Kind::ROAD;
      RETURN_OR_ASSIGN(
          link.contact,
          read_contact(node, node.attribute("contactPoint").as_string()));
    } else if (kind == "junction") {
      link.kind = road::Link::Kind::JUNCTION;
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
    for (pugi::xml_node priority_node : node.children("priority")) {
      JunctionPriority priority{
          .high = priority_node.attribute("high").as_string(),
          .low = priority_node.attribute("low").as_string()};
      if (priority.high.empty() || priority.low.empty()) {
        return fail(priority_node, "needs high and low");
      }
      junction.priorities.push_back(std::move(priority));
    }
    for (pugi::xml_node controller_node : node.children("controller")) {
      JunctionController controller;
      controller.id = controller_node.attribute("id").as_string();
      if (controller.id.empty()) {
        return fail(controller_node, "needs id");
      }
      controller.type = controller_node.attribute("type").as_string();
      RETURN_OR_ASSIGN(controller.sequence,
                       read_count(controller_node, "sequence"));
      junction.controllers.push_back(std::move(controller));
    }
    return junction;
  }

  auto read_controller(pugi::xml_node node) const
      -> std::expected<SignalController, lib::Status> {
    SignalController controller;
    controller.id = node.attribute("id").as_string();
    if (controller.id.empty()) {
      return fail(node, "needs id");
    }
    controller.name = node.attribute("name").as_string();
    RETURN_OR_ASSIGN(controller.sequence, read_count(node, "sequence"));
    for (pugi::xml_node control_node : node.children("control")) {
      SignalController::Control control{
          .signal = control_node.attribute("signalId").as_string(),
          .type = control_node.attribute("type").as_string()};
      if (control.signal.empty()) {
        return fail(control_node, "needs signalId");
      }
      controller.controls.push_back(std::move(control));
    }
    return controller;
  }

  // The whole number in `node`'s attribute `name`, 0 if it has none.
  auto read_count(pugi::xml_node node, const char* name) const
      -> std::expected<std::uint32_t, lib::Status> {
    RETURN_OR_ASSIGN(double count, read_number(node, name, 0.0));
    if (count < 0.0 || count != std::floor(count) ||
        count > std::numeric_limits<std::uint32_t>::max()) {
      return fail(node, std::string{name} + " is not a count");
    }
    return static_cast<std::uint32_t>(count);
  }

  auto read_orientation(pugi::xml_node node) const
      -> std::expected<road::Orientation, lib::Status> {
    std::string_view word = node.attribute("orientation").as_string("none");
    if (word == "+") {
      return road::Orientation::POSITIVE;
    }
    if (word == "-") {
      return road::Orientation::NEGATIVE;
    }
    if (word == "none") {
      return road::Orientation::BOTH;
    }
    return fail(node,
                "orientation `" + std::string{word} + "` is not +, - or none");
  }

  // Appends `node`'s validity children to `validities`.
  auto read_validities(pugi::xml_node node,
                       InOut<std::vector<LaneValidity>> validities) const
      -> std::expected<void, lib::Status> {
    for (pugi::xml_node validity_node : node.children("validity")) {
      RETURN_OR_ASSIGN(std::optional<int> from,
                       read_lane_id(validity_node, "fromLane"));
      RETURN_OR_ASSIGN(std::optional<int> to,
                       read_lane_id(validity_node, "toLane"));
      if (!from || !to) {
        return fail(validity_node, "needs fromLane and toLane");
      }
      if (*from > *to) {
        return fail(validity_node, "has fromLane after toLane");
      }
      validities->push_back({.from = *from, .to = *to});
    }
    return {};
  }

  auto read_signal(pugi::xml_node node) const
      -> std::expected<Signal, lib::Status> {
    Signal signal;
    signal.id = node.attribute("id").as_string();
    if (signal.id.empty()) {
      return fail(node, "needs id");
    }
    signal.name = node.attribute("name").as_string();
    RETURN_OR_ASSIGN(signal.s, read_number(node, "s"));
    RETURN_OR_ASSIGN(signal.t, read_number(node, "t"));
    RETURN_OR_ASSIGN(signal.z_offset, read_number(node, "zOffset", 0.0));
    std::string_view dynamic = node.attribute("dynamic").as_string("no");
    if (dynamic != "yes" && dynamic != "no") {
      return fail(
          node, "dynamic `" + std::string{dynamic} + "` is neither yes nor no");
    }
    signal.dynamic = dynamic == "yes";
    RETURN_OR_ASSIGN(signal.orientation, read_orientation(node));
    signal.country = node.attribute("country").as_string();
    signal.type = node.attribute("type").as_string();
    signal.subtype = node.attribute("subtype").as_string();
    if (node.attribute("value")) {
      RETURN_OR_ASSIGN(double value, read_number(node, "value"));
      signal.value = value;
    }
    signal.unit = node.attribute("unit").as_string();
    RETURN_IF_UNEXPECTED(read_validities(node, InOut(signal.validities)));
    return signal;
  }

  auto read_outline(pugi::xml_node node) const
      -> std::expected<road::RoadObject::Outline, lib::Status> {
    road::RoadObject::Outline outline;
    outline.closed =
        node.attribute("closed").as_string("true") != std::string_view{"false"};
    bool road = false;
    bool local = false;
    for (pugi::xml_node corner_node : node.children()) {
      std::string_view kind = corner_node.name();
      road::RoadObject::Corner corner;
      if (kind == "cornerRoad") {
        road = true;
        RETURN_OR_ASSIGN(corner.first, read_number(corner_node, "s"));
        RETURN_OR_ASSIGN(corner.second, read_number(corner_node, "t"));
        RETURN_OR_ASSIGN(corner.up, read_number(corner_node, "dz", 0.0));
      } else if (kind == "cornerLocal") {
        local = true;
        RETURN_OR_ASSIGN(corner.first, read_number(corner_node, "u"));
        RETURN_OR_ASSIGN(corner.second, read_number(corner_node, "v"));
        RETURN_OR_ASSIGN(corner.up, read_number(corner_node, "z", 0.0));
      } else {
        continue;
      }
      RETURN_OR_ASSIGN(corner.height, read_number(corner_node, "height", 0.0));
      outline.corners.push_back(corner);
    }
    if (road && local) {
      return fail(node, "mixes cornerRoad and cornerLocal");
    }
    outline.frame = local ? road::RoadObject::Outline::Frame::LOCAL
                          : road::RoadObject::Outline::Frame::ROAD;
    return outline;
  }

  auto read_object(pugi::xml_node node) const
      -> std::expected<road::RoadObject, lib::Status> {
    road::RoadObject object;
    object.id = node.attribute("id").as_string();
    if (object.id.empty()) {
      return fail(node, "needs id");
    }
    object.name = node.attribute("name").as_string();
    object.type = node.attribute("type").as_string("none");
    object.subtype = node.attribute("subtype").as_string();
    RETURN_OR_ASSIGN(object.s, read_number(node, "s"));
    RETURN_OR_ASSIGN(object.t, read_number(node, "t"));
    RETURN_OR_ASSIGN(object.z_offset, read_number(node, "zOffset", 0.0));
    RETURN_OR_ASSIGN(object.heading, read_number(node, "hdg", 0.0));
    RETURN_OR_ASSIGN(object.pitch, read_number(node, "pitch", 0.0));
    RETURN_OR_ASSIGN(object.roll, read_number(node, "roll", 0.0));
    RETURN_OR_ASSIGN(object.length, read_number(node, "length", 0.0));
    RETURN_OR_ASSIGN(object.width, read_number(node, "width", 0.0));
    RETURN_OR_ASSIGN(object.height, read_number(node, "height", 0.0));
    RETURN_OR_ASSIGN(object.orientation, read_orientation(node));
    // OpenDRIVE 1.4 puts one outline in the object, 1.5 on several in
    // <outlines>.
    pugi::xml_node outlines =
        node.child("outlines") ? node.child("outlines") : node;
    for (pugi::xml_node outline_node : outlines.children("outline")) {
      RETURN_OR_ASSIGN(road::RoadObject::Outline outline,
                       read_outline(outline_node));
      object.outlines.push_back(std::move(outline));
    }
    RETURN_IF_UNEXPECTED(read_validities(node, InOut(object.validities)));
    return object;
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
    for (pugi::xml_node signal_node :
         node.child("signals").children("signal")) {
      RETURN_OR_ASSIGN(Signal signal, read_signal(signal_node));
      road.signals.push_back(std::move(signal));
    }
    for (pugi::xml_node object_node :
         node.child("objects").children("object")) {
      RETURN_OR_ASSIGN(road::RoadObject object, read_object(object_node));
      road.objects.push_back(std::move(object));
    }
    return road;
  }

  XmlDocument document_;
};

}  // namespace

auto parse_opendrive(std::string_view text)
    -> std::expected<road::Map, lib::Status> {
  return Parser{}.parse(text);
}

auto load_opendrive(const std::string& path)
    -> std::expected<road::Map, lib::Status> {
  RETURN_OR_ASSIGN(std::string text, read_text_file(path));
  return parse_opendrive(text);
}

}  // namespace simon::format
