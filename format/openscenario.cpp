// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows esmini 3.8.2, Copyright (c) partners of Simulation Scenarios,
// MPL-2.0; translated to C++ and changed. See NOTICE.md.

#include "format/openscenario.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <format>
#include <memory>
#include <optional>
#include <sstream>
#include <utility>

#include "base/core.hpp"
#include "core/argument.hpp"
#include "format/text.hpp"
#include "format/xml.hpp"
#include "pugixml.hpp"

namespace simon::format {

using namespace scenario;

namespace {

using lib::InOut;
using lib::Out;

// Evaluates OpenSCENARIO 1.1's arithmetic expressions by recursive descent:
//
//   expression = term {("+" | "-") term}
//   term       = unary {("*" | "/" | "%") unary}
//   unary      = "-" unary | primary
//   primary    = number | "$" name | "(" expression ")"
class Expression final {
 public:
  Expression(std::string_view text, const std::vector<Parameter>& parameters)
      : text_{text}, parameters_{&parameters} {}

  auto evaluate() -> std::expected<double, lib::Status> {
    auto value = parse_expression();
    skip_space();
    if (value && at_ < text_.size()) {
      return fail("unexpected '" + std::string{text_.substr(at_)} + "'");
    }
    return value;
  }

 private:
  static auto fail(const std::string& why) -> Failure {
    return Failure{lib::raise(FormatError::MALFORMED, why)};
  }

  auto skip_space() -> void {
    while (at_ < text_.size() &&
           std::isspace(static_cast<unsigned char>(text_[at_]))) {
      ++at_;
    }
  }

  auto take(char wanted) -> bool {
    skip_space();
    if (at_ < text_.size() && text_[at_] == wanted) {
      ++at_;
      return true;
    }
    return false;
  }

  auto parse_expression() -> std::expected<double, lib::Status> {
    auto value = parse_term();
    while (value) {
      if (take('+')) {
        auto rest = parse_term();
        if (!rest) {
          return rest;
        }
        *value += *rest;
      } else if (take('-')) {
        auto rest = parse_term();
        if (!rest) {
          return rest;
        }
        *value -= *rest;
      } else {
        break;
      }
    }
    return value;
  }

  auto parse_term() -> std::expected<double, lib::Status> {
    auto value = parse_unary();
    while (value) {
      char op = 0;
      if (take('*')) {
        op = '*';
      } else if (take('/')) {
        op = '/';
      } else if (take('%')) {
        op = '%';
      } else {
        break;
      }
      auto rest = parse_unary();
      if (!rest) {
        return rest;
      }
      *value = op == '*'   ? *value * *rest
               : op == '/' ? *value / *rest
                           : std::fmod(*value, *rest);
    }
    return value;
  }

  auto parse_unary() -> std::expected<double, lib::Status> {
    if (take('-')) {
      auto value = parse_unary();
      if (value) {
        *value = -*value;
      }
      return value;
    }
    return parse_primary();
  }

  auto parse_primary() -> std::expected<double, lib::Status> {
    skip_space();
    if (take('(')) {
      auto value = parse_expression();
      if (value && !take(')')) {
        return fail("missing ')'");
      }
      return value;
    }
    if (take('$')) {
      std::size_t start = at_;
      while (at_ < text_.size() &&
             (std::isalnum(static_cast<unsigned char>(text_[at_])) ||
              text_[at_] == '_')) {
        ++at_;
      }
      std::string_view name = text_.substr(start, at_ - start);
      auto found = std::ranges::find(*parameters_, name, &Parameter::name);
      if (found == parameters_->end()) {
        return fail("no parameter " + std::string{name});
      }
      std::optional<double> value = parse_number(found->value);
      if (!value) {
        return fail("parameter " + std::string{name} + " is not a number");
      }
      return *value;
    }
    std::size_t start = at_;
    while (at_ < text_.size() &&
           (std::isdigit(static_cast<unsigned char>(text_[at_])) ||
            text_[at_] == '.' || text_[at_] == 'e' || text_[at_] == 'E' ||
            ((text_[at_] == '-' || text_[at_] == '+') && at_ > start &&
             (text_[at_ - 1] == 'e' || text_[at_ - 1] == 'E')))) {
      ++at_;
    }
    std::optional<double> value =
        parse_number(text_.substr(start, at_ - start));
    if (!value) {
      return fail("expected a number at '" + std::string{text_.substr(start)} +
                  "'");
    }
    return *value;
  }

  std::string_view text_;
  const std::vector<Parameter>* parameters_ = nullptr;
  std::size_t at_ = 0;
};

// Reads one scenario, its parameters in scopes: the file's, then a story's
// or a catalog entry's, innermost last. Says on which line of the scenario
// or of a catalog anything is wrong.
class Parser final {
 public:
  Parser(std::string directory,
         std::span<const ParameterAssignment> assignments)
      : directory_{std::move(directory)}, assignments_{assignments} {}

  auto parse(std::string_view text) -> std::expected<Scenario, lib::Status> {
    RETURN_IF_UNEXPECTED(document_.load(std::string{text}));
    RETURN_OR_ASSIGN(pugi::xml_node root, document_.find_root("OpenSCENARIO"));
    RETURN_IF_UNEXPECTED(assign(root.child("ParameterDeclarations")));
    Scenario scenario;
    RETURN_IF_UNEXPECTED(
        declare(root.child("ParameterDeclarations"), Out(scenario.parameters)));
    scope_ = scenario.parameters;
    RETURN_IF_UNEXPECTED(read_catalogs(root.child("CatalogLocations")));

    pugi::xml_node logic = root.child("RoadNetwork").child("LogicFile");
    if (!logic) {
      return fail(root.child("RoadNetwork"), "needs a LogicFile");
    }
    RETURN_OR_ASSIGN(std::string road, read_text(logic, "filepath"));
    scenario.road_network = resolve(road);
    RETURN_OR_ASSIGN(scenario.signal_controllers,
                     read_signal_controllers(
                         root.child("RoadNetwork").child("TrafficSignals")));

    for (pugi::xml_node object : root.child("Entities").children()) {
      RETURN_OR_ASSIGN(Entity entity, read_entity(object));
      scenario.entities.push_back(std::move(entity));
    }
    RETURN_OR_ASSIGN(scenario.storyboard,
                     read_storyboard(root.child("Storyboard")));
    return scenario;
  }

 private:
  // The document `node` stands in: the scenario's, or a catalog's.
  auto document_of(pugi::xml_node node) const -> const XmlDocument& {
    for (const auto& catalog : catalogs_) {
      if (catalog->holds(node)) {
        return *catalog;
      }
    }
    return document_;
  }
  auto fail(pugi::xml_node node, std::string_view why) const -> Failure {
    return document_of(node).fail(node, why);
  }
  auto refuse(pugi::xml_node node, std::string_view what = {}) const
      -> Failure {
    return document_of(node).refuse(node, what);
  }

  auto resolve(const std::string& path) const -> std::string {
    std::filesystem::path relative{path};
    return relative.is_absolute()
               ? path
               : (std::filesystem::path{directory_} / relative)
                     .lexically_normal()
                     .string();
  }

  // Replaces the values of the declarations the assignments name.
  auto assign(pugi::xml_node declarations) -> std::expected<void, lib::Status> {
    for (const ParameterAssignment& assignment : assignments_) {
      pugi::xml_node declaration = declarations.find_child_by_attribute(
          "ParameterDeclaration", "name", assignment.name.c_str());
      if (!declaration) {
        return fail(declarations,
                    "no parameter " + assignment.name + " to assign");
      }
      declaration.attribute("value").set_value(assignment.value.c_str());
    }
    return {};
  }

  // Adds a scope's declarations to `parameters`, each value resolved in the
  // scope so far.
  auto declare(pugi::xml_node declarations, Out<std::vector<Parameter>> into)
      -> std::expected<void, lib::Status> {
    for (pugi::xml_node node : declarations.children("ParameterDeclaration")) {
      std::vector<Parameter> visible = scope_;
      visible.insert(visible.end(), into->begin(), into->end());
      std::swap(scope_, visible);
      auto value = read_text(node, "value");
      std::swap(scope_, visible);
      if (!value) {
        return std::unexpected(value.error());
      }
      into->push_back(
          Parameter{.name = node.attribute("name").as_string(),
                    .type = node.attribute("parameterType").as_string(),
                    .value = *value});
    }
    return {};
  }

  // An attribute's text, its parameter or expression resolved.
  auto read_text(pugi::xml_node node, std::string_view name) const
      -> std::expected<std::string, lib::Status> {
    pugi::xml_attribute attribute = node.attribute(name);
    if (!attribute) {
      return fail(node, "needs " + std::string{name});
    }
    std::string text = attribute.as_string();
    if (text.starts_with("${") && text.ends_with("}")) {
      auto value = evaluate_expression(
          std::string_view{text}.substr(2, text.size() - 3), scope_);
      if (!value) {
        return fail(node, std::string{name} + ": " +
                              std::string{value.error().message()});
      }
      std::ostringstream out;
      out.precision(17);
      out << *value;
      return out.str();
    }
    if (text.starts_with("$")) {
      // The innermost scope's parameter of that name.
      for (auto it = scope_.rbegin(); it != scope_.rend(); ++it) {
        if (it->name == std::string_view{text}.substr(1)) {
          return it->value;
        }
      }
      return fail(node, "no parameter " + text.substr(1));
    }
    return text;
  }

  auto read_number(pugi::xml_node node, std::string_view name) const
      -> std::expected<double, lib::Status> {
    RETURN_OR_ASSIGN(std::string text, read_text(node, name));
    std::optional<double> value = parse_number(text);
    if (!value) {
      return fail(node,
                  std::string{name} + " `" + text + "` is not a finite number");
    }
    return *value;
  }

  auto read_number_or(pugi::xml_node node, std::string_view name,
                      double otherwise) const
      -> std::expected<double, lib::Status> {
    if (!node.attribute(name)) {
      return otherwise;
    }
    return read_number(node, name);
  }

  auto read_flag(pugi::xml_node node, std::string_view name,
                 bool otherwise) const -> std::expected<bool, lib::Status> {
    if (!node.attribute(name)) {
      return otherwise;
    }
    RETURN_OR_ASSIGN(std::string text, read_text(node, name));
    return text == "true" || text == "1";
  }

  //-- Catalogs ---------------------------------------------------------------

  auto read_catalogs(pugi::xml_node locations)
      -> std::expected<void, lib::Status> {
    for (pugi::xml_node location : locations.children()) {
      RETURN_OR_ASSIGN(std::string path,
                       read_text(location.child("Directory"), "path"));
      std::filesystem::path directory{resolve(path)};
      if (!std::filesystem::is_directory(directory)) {
        continue;  // As esmini does, a missing catalog is only an error if
                   // used.
      }
      for (const auto& file : std::filesystem::directory_iterator{directory}) {
        if (file.path().extension() != ".xosc") {
          continue;
        }
        RETURN_OR_ASSIGN(std::string text,
                         read_text_file(file.path().string()));
        auto document = std::make_unique<XmlDocument>();
        if (auto loaded = document->load(std::move(text)); !loaded) {
          return fail(location, "catalog " + file.path().string() + ": " +
                                    std::string{loaded.error().message()});
        }
        catalogs_.push_back(std::move(document));
      }
    }
    return {};
  }

  // The catalog entry `entry` of the catalog `catalog`.
  auto find_catalog_entry(std::string_view catalog,
                          std::string_view entry) const -> pugi::xml_node {
    for (const auto& document : catalogs_) {
      pugi::xml_node root =
          document->node().child("OpenSCENARIO").child("Catalog");
      if (root.attribute("name").as_string() != catalog) {
        continue;
      }
      for (pugi::xml_node node : root.children()) {
        if (node.attribute("name").as_string() == entry) {
          return node;
        }
      }
    }
    return {};
  }

  //-- Traffic signals --------------------------------------------------------

  auto read_signal_controllers(pugi::xml_node signals)
      -> std::expected<std::vector<TrafficSignalController>, lib::Status> {
    std::vector<TrafficSignalController> controllers;
    for (pugi::xml_node node : signals.children()) {
      if (std::string_view{node.name()} != "TrafficSignalController") {
        return refuse(node);
      }
      TrafficSignalController controller;
      RETURN_OR_ASSIGN(controller.name, read_text(node, "name"));
      RETURN_OR_ASSIGN(controller.delay, read_number_or(node, "delay", 0.0));
      if (node.attribute("reference")) {
        RETURN_OR_ASSIGN(controller.reference, read_text(node, "reference"));
      }
      for (pugi::xml_node phase : node.children()) {
        if (std::string_view{phase.name()} != "Phase") {
          return refuse(phase);
        }
        TrafficSignalPhase read;
        RETURN_OR_ASSIGN(read.name, read_text(phase, "name"));
        RETURN_OR_ASSIGN(read.duration, read_number(phase, "duration"));
        for (pugi::xml_node state : phase.children()) {
          if (std::string_view{state.name()} != "TrafficSignalState") {
            return refuse(state);
          }
          TrafficSignalState signal;
          RETURN_OR_ASSIGN(signal.signal, read_text(state, "trafficSignalId"));
          RETURN_OR_ASSIGN(signal.state, read_text(state, "state"));
          read.states.push_back(std::move(signal));
        }
        controller.phases.push_back(std::move(read));
      }
      if (controller.phases.empty()) {
        return fail(node, controller.name + " has no phases");
      }
      controllers.push_back(std::move(controller));
    }
    for (const TrafficSignalController& controller : controllers) {
      if (!controller.reference.empty() &&
          std::ranges::find(controllers, controller.reference,
                            &TrafficSignalController::name) ==
              controllers.end()) {
        return fail(signals, "controller " + controller.name +
                                 " refers to no controller " +
                                 controller.reference);
      }
    }
    return controllers;
  }

  //-- Entities ---------------------------------------------------------------

  auto read_entity(pugi::xml_node object)
      -> std::expected<Entity, lib::Status> {
    if (std::string_view{object.name()} != "ScenarioObject") {
      return refuse(object);
    }
    Entity entity{.name = object.attribute("name").as_string()};
    for (pugi::xml_node child : object.children()) {
      std::string_view kind = child.name();
      if (kind == "Vehicle") {
        RETURN_OR_ASSIGN(entity.vehicle, read_vehicle(child));
      } else if (kind == "Pedestrian") {
        RETURN_OR_ASSIGN(entity.vehicle, read_pedestrian(child));
        entity.kind = Entity::Kind::PEDESTRIAN;
      } else if (kind == "CatalogReference") {
        RETURN_OR_ASSIGN(std::string catalog, read_text(child, "catalogName"));
        RETURN_OR_ASSIGN(std::string name, read_text(child, "entryName"));
        pugi::xml_node vehicle = find_catalog_entry(catalog, name);
        if (!vehicle) {
          return fail(child, "no catalog entry " + catalog + "/" + name);
        }
        std::string_view entry = vehicle.name();
        if (entry != "Vehicle" && entry != "Pedestrian") {
          return refuse(vehicle);
        }
        // The entry's own parameters, overridden by the reference's.
        std::vector<Parameter> outer = scope_;
        std::vector<Parameter> local;
        RETURN_IF_UNEXPECTED(
            declare(vehicle.child("ParameterDeclarations"), Out(local)));
        for (pugi::xml_node assignment :
             child.child("ParameterAssignments").children()) {
          RETURN_OR_ASSIGN(std::string value, read_text(assignment, "value"));
          std::string reference =
              assignment.attribute("parameterRef").as_string();
          auto found = std::ranges::find(local, reference, &Parameter::name);
          if (found != local.end()) {
            found->value = value;
          }
        }
        scope_.insert(scope_.end(), local.begin(), local.end());
        auto read = entry == "Vehicle" ? read_vehicle(vehicle)
                                       : read_pedestrian(vehicle);
        scope_ = std::move(outer);
        if (!read) {
          return std::unexpected(read.error());
        }
        entity.vehicle = std::move(*read);
        if (entry == "Pedestrian") {
          entity.kind = Entity::Kind::PEDESTRIAN;
        }
      } else if (kind == "ObjectController") {
        return refuse(child);
      } else {
        return refuse(child);
      }
    }
    return entity;
  }

  auto read_box(pugi::xml_node box, InOut<Vehicle> vehicle) const
      -> std::expected<void, lib::Status> {
    RETURN_OR_ASSIGN(vehicle->center[0], read_number(box.child("Center"), "x"));
    RETURN_OR_ASSIGN(vehicle->center[1], read_number(box.child("Center"), "y"));
    RETURN_OR_ASSIGN(vehicle->center[2], read_number(box.child("Center"), "z"));
    RETURN_OR_ASSIGN(vehicle->dimensions[0],
                     read_number(box.child("Dimensions"), "length"));
    RETURN_OR_ASSIGN(vehicle->dimensions[1],
                     read_number(box.child("Dimensions"), "width"));
    RETURN_OR_ASSIGN(vehicle->dimensions[2],
                     read_number(box.child("Dimensions"), "height"));
    return {};
  }

  // A pedestrian, as a vehicle with its box and esmini's limits for it:
  // 1e10 on speed and on each change of speed, none in effect.
  auto read_pedestrian(pugi::xml_node node)
      -> std::expected<Vehicle, lib::Status> {
    constexpr double UNLIMITED = 1e10;
    Vehicle pedestrian{.name = node.attribute("name").as_string(),
                       .max_speed = UNLIMITED,
                       .max_acceleration = UNLIMITED,
                       .max_deceleration = UNLIMITED};
    RETURN_IF_UNEXPECTED(
        read_box(node.child("BoundingBox"), InOut(pedestrian)));
    return pedestrian;
  }

  auto read_vehicle(pugi::xml_node node)
      -> std::expected<Vehicle, lib::Status> {
    Vehicle vehicle{.name = node.attribute("name").as_string()};
    RETURN_IF_UNEXPECTED(read_box(node.child("BoundingBox"), InOut(vehicle)));
    pugi::xml_node performance = node.child("Performance");
    RETURN_OR_ASSIGN(vehicle.max_speed, read_number(performance, "maxSpeed"));
    RETURN_OR_ASSIGN(vehicle.max_acceleration,
                     read_number(performance, "maxAcceleration"));
    RETURN_OR_ASSIGN(vehicle.max_deceleration,
                     read_number(performance, "maxDeceleration"));
    pugi::xml_node axles = node.child("Axles");
    RETURN_OR_ASSIGN(double front, read_number_or(axles.child("FrontAxle"),
                                                  "positionX", 0.0));
    RETURN_OR_ASSIGN(double rear,
                     read_number_or(axles.child("RearAxle"), "positionX", 0.0));
    vehicle.wheelbase = front - rear;
    // A trailer is an entity of its own, which simon does not tow.
    if (pugi::xml_node trailer = node.child("Trailer")) {
      RETURN_OR_ASSIGN(std::string reference,
                       read_text(trailer.child("TrailerRef"), "entityRef"));
      if (!reference.empty()) {
        return refuse(trailer);
      }
    }
    return vehicle;
  }

  //-- Positions --------------------------------------------------------------

  auto read_orientation(pugi::xml_node node)
      -> std::expected<std::optional<Orientation>, lib::Status> {
    pugi::xml_node orientation = node.child("Orientation");
    if (!orientation) {
      return std::nullopt;
    }
    Orientation result;
    if (orientation.attribute("type")) {
      RETURN_OR_ASSIGN(std::string type, read_text(orientation, "type"));
      result.relative = type != "absolute";
    }
    RETURN_OR_ASSIGN(result.h, read_number_or(orientation, "h", 0.0));
    return result;
  }

  auto read_position(pugi::xml_node position)
      -> std::expected<Position, lib::Status> {
    pugi::xml_node node = position.first_child();
    std::string_view kind = node.name();
    if (kind == "WorldPosition") {
      WorldPosition world;
      RETURN_OR_ASSIGN(world.x, read_number(node, "x"));
      RETURN_OR_ASSIGN(world.y, read_number(node, "y"));
      RETURN_OR_ASSIGN(world.z, read_number_or(node, "z", 0.0));
      RETURN_OR_ASSIGN(world.h, read_number_or(node, "h", 0.0));
      return world;
    }
    if (kind == "LanePosition") {
      LanePosition lane;
      RETURN_OR_ASSIGN(lane.road, read_text(node, "roadId"));
      RETURN_OR_ASSIGN(double id, read_number(node, "laneId"));
      lane.lane = static_cast<int>(id);
      RETURN_OR_ASSIGN(lane.s, read_number(node, "s"));
      RETURN_OR_ASSIGN(lane.offset, read_number_or(node, "offset", 0.0));
      RETURN_OR_ASSIGN(lane.orientation, read_orientation(node));
      return lane;
    }
    if (kind == "RoadPosition") {
      RoadPosition road;
      RETURN_OR_ASSIGN(road.road, read_text(node, "roadId"));
      RETURN_OR_ASSIGN(road.s, read_number(node, "s"));
      RETURN_OR_ASSIGN(road.t, read_number(node, "t"));
      RETURN_OR_ASSIGN(road.orientation, read_orientation(node));
      return road;
    }
    if (kind == "RelativeRoadPosition") {
      RelativeRoadPosition relative;
      RETURN_OR_ASSIGN(relative.entity, read_text(node, "entityRef"));
      RETURN_OR_ASSIGN(relative.ds, read_number(node, "ds"));
      RETURN_OR_ASSIGN(relative.dt, read_number(node, "dt"));
      RETURN_OR_ASSIGN(relative.orientation, read_orientation(node));
      return relative;
    }
    if (kind == "RelativeLanePosition") {
      RelativeLanePosition relative;
      RETURN_OR_ASSIGN(relative.entity, read_text(node, "entityRef"));
      RETURN_OR_ASSIGN(double lanes, read_number(node, "dLane"));
      relative.lanes = static_cast<int>(lanes);
      RETURN_OR_ASSIGN(relative.ds, read_number_or(node, "ds", 0.0));
      RETURN_OR_ASSIGN(relative.offset, read_number_or(node, "offset", 0.0));
      RETURN_OR_ASSIGN(relative.orientation, read_orientation(node));
      return relative;
    }
    return refuse(node);
  }

  //-- Actions ----------------------------------------------------------------

  auto read_dynamics(pugi::xml_node node)
      -> std::expected<TransitionDynamics, lib::Status> {
    TransitionDynamics dynamics;
    RETURN_OR_ASSIGN(std::string shape, read_text(node, "dynamicsShape"));
    RETURN_OR_ASSIGN(DynamicsShape parsed, read_shape(node, shape));
    dynamics.shape = parsed;
    RETURN_OR_ASSIGN(std::string dimension,
                     read_text(node, "dynamicsDimension"));
    if (dimension == "time") {
      dynamics.dimension = DynamicsDimension::TIME;
    } else if (dimension == "distance") {
      dynamics.dimension = DynamicsDimension::DISTANCE;
    } else if (dimension == "rate") {
      dynamics.dimension = DynamicsDimension::RATE;
    } else {
      return fail(node, "unknown dynamicsDimension " + dimension);
    }
    RETURN_OR_ASSIGN(dynamics.value, read_number(node, "value"));
    return dynamics;
  }

  auto read_shape(pugi::xml_node node, std::string_view shape) const
      -> std::expected<DynamicsShape, lib::Status> {
    if (shape == "step") {
      return DynamicsShape::STEP;
    }
    if (shape == "linear") {
      return DynamicsShape::LINEAR;
    }
    if (shape == "cubic") {
      return DynamicsShape::CUBIC;
    }
    if (shape == "sinusoidal") {
      return DynamicsShape::SINUSOIDAL;
    }
    return fail(node, "unknown shape " + std::string{shape});
  }

  auto read_private_action(pugi::xml_node action)
      -> std::expected<PrivateAction, lib::Status> {
    pugi::xml_node kind = action.first_child();
    std::string_view name = kind.name();
    if (name == "LongitudinalAction") {
      pugi::xml_node speed = kind.child("SpeedAction");
      if (!speed) {
        return refuse(kind.first_child());
      }
      SpeedAction result;
      RETURN_OR_ASSIGN(result.dynamics,
                       read_dynamics(speed.child("SpeedActionDynamics")));
      pugi::xml_node target = speed.child("SpeedActionTarget").first_child();
      if (std::string_view{target.name()} == "AbsoluteTargetSpeed") {
        RETURN_OR_ASSIGN(double value, read_number(target, "value"));
        result.target = AbsoluteTargetSpeed{.value = value};
      } else if (std::string_view{target.name()} == "RelativeTargetSpeed") {
        RelativeTargetSpeed relative;
        RETURN_OR_ASSIGN(relative.entity, read_text(target, "entityRef"));
        RETURN_OR_ASSIGN(relative.value, read_number(target, "value"));
        RETURN_OR_ASSIGN(std::string type,
                         read_text(target, "speedTargetValueType"));
        relative.kind = type == "factor" ? RelativeTargetSpeed::Kind::FACTOR
                                         : RelativeTargetSpeed::Kind::DELTA;
        RETURN_OR_ASSIGN(relative.continuous,
                         read_flag(target, "continuous", false));
        result.target = relative;
      } else {
        return refuse(target);
      }
      return result;
    }
    if (name == "LateralAction") {
      pugi::xml_node lateral = kind.first_child();
      std::string_view lateral_name = lateral.name();
      if (lateral_name == "LaneChangeAction") {
        LaneChangeAction result;
        RETURN_OR_ASSIGN(
            result.dynamics,
            read_dynamics(lateral.child("LaneChangeActionDynamics")));
        RETURN_OR_ASSIGN(result.target_offset,
                         read_number_or(lateral, "targetLaneOffset", 0.0));
        pugi::xml_node target = lateral.child("LaneChangeTarget").first_child();
        if (std::string_view{target.name()} == "AbsoluteTargetLane") {
          RETURN_OR_ASSIGN(double lane, read_number(target, "value"));
          result.target = AbsoluteTargetLane{.lane = static_cast<int>(lane)};
        } else if (std::string_view{target.name()} == "RelativeTargetLane") {
          RETURN_OR_ASSIGN(std::string entity, read_text(target, "entityRef"));
          RETURN_OR_ASSIGN(double lanes, read_number(target, "value"));
          result.target = RelativeTargetLane{.entity = entity,
                                             .lanes = static_cast<int>(lanes)};
        } else {
          return refuse(target);
        }
        return result;
      }
      if (lateral_name == "LaneOffsetAction") {
        LaneOffsetAction result;
        RETURN_OR_ASSIGN(result.continuous,
                         read_flag(lateral, "continuous", false));
        pugi::xml_node dynamics = lateral.child("LaneOffsetActionDynamics");
        RETURN_OR_ASSIGN(std::string shape,
                         read_text(dynamics, "dynamicsShape"));
        RETURN_OR_ASSIGN(result.shape, read_shape(dynamics, shape));
        RETURN_OR_ASSIGN(result.max_lateral_acceleration,
                         read_number(dynamics, "maxLateralAcc"));
        pugi::xml_node target = lateral.child("LaneOffsetTarget").first_child();
        if (std::string_view{target.name()} == "AbsoluteTargetLaneOffset") {
          RETURN_OR_ASSIGN(result.value, read_number(target, "value"));
        } else if (std::string_view{target.name()} ==
                   "RelativeTargetLaneOffset") {
          RETURN_OR_ASSIGN(std::string entity, read_text(target, "entityRef"));
          result.relative_to = entity;
          RETURN_OR_ASSIGN(result.value, read_number(target, "value"));
        } else {
          return refuse(target);
        }
        return result;
      }
      return refuse(lateral);
    }
    if (name == "TeleportAction") {
      RETURN_OR_ASSIGN(Position position,
                       read_position(kind.child("Position")));
      return TeleportAction{.position = position};
    }
    if (name == "RoutingAction") {
      pugi::xml_node routing = kind.first_child();
      std::string_view routing_name = routing.name();
      if (routing_name == "AssignRouteAction") {
        return read_assign_route(routing);
      }
      if (routing_name == "FollowTrajectoryAction") {
        return read_follow_trajectory(routing);
      }
      return routing ? refuse(routing) : refuse(kind);
    }
    return refuse(kind);
  }

  auto read_assign_route(pugi::xml_node node)
      -> std::expected<PrivateAction, lib::Status> {
    pugi::xml_node route = node.child("Route");
    if (!route) {
      return refuse(node.first_child());
    }
    AssignRouteAction action;
    action.route.name = route.attribute("name").as_string();
    RETURN_OR_ASSIGN(bool closed, read_flag(route, "closed", false));
    if (closed) {
      return refuse(route, "closed");
    }
    for (pugi::xml_node waypoint : route.children("Waypoint")) {
      // simon routes by the shortest way only; it refuses the others rather
      // than take them as shortest.
      RETURN_OR_ASSIGN(std::string strategy,
                       read_text(waypoint, "routeStrategy"));
      if (strategy != "shortest") {
        return refuse(waypoint, "routeStrategy " + strategy);
      }
      Waypoint read;
      RETURN_OR_ASSIGN(read.position,
                       read_position(waypoint.child("Position")));
      action.route.waypoints.push_back(std::move(read));
    }
    if (action.route.waypoints.size() < 2) {
      return fail(route, action.route.name + " needs two waypoints or more");
    }
    return action;
  }

  // A polyline followed without timing, held to the line: the only kind
  // simon follows.
  auto read_follow_trajectory(pugi::xml_node node)
      -> std::expected<PrivateAction, lib::Status> {
    pugi::xml_node trajectory = node.child("Trajectory");
    if (!trajectory) {
      return refuse(node.first_child());
    }
    FollowTrajectoryAction action{.name =
                                      trajectory.attribute("name").as_string()};
    RETURN_OR_ASSIGN(action.initial_distance_offset,
                     read_number_or(node, "initialDistanceOffset", 0.0));
    RETURN_OR_ASSIGN(bool closed, read_flag(trajectory, "closed", false));
    if (closed) {
      return refuse(trajectory, "closed");
    }
    pugi::xml_node shape = trajectory.child("Shape").first_child();
    if (std::string_view{shape.name()} != "Polyline") {
      return refuse(shape);
    }
    for (pugi::xml_node vertex : shape.children("Vertex")) {
      // esmini turns the entity to a vertex's own heading, which simon does
      // not.
      if (vertex.child("Position").first_child().child("Orientation")) {
        return refuse(vertex, "with an orientation");
      }
      Vertex read;
      RETURN_OR_ASSIGN(read.time, read_number_or(vertex, "time", 0.0));
      RETURN_OR_ASSIGN(read.position, read_position(vertex.child("Position")));
      action.vertices.push_back(std::move(read));
    }
    if (action.vertices.size() < 2) {
      return fail(trajectory, action.name + " needs two vertices");
    }
    pugi::xml_node timing = node.child("TimeReference").first_child();
    if (std::string_view{timing.name()} != "None") {
      return refuse(timing);
    }
    pugi::xml_node following = node.child("TrajectoryFollowingMode");
    RETURN_OR_ASSIGN(std::string mode, read_text(following, "followingMode"));
    if (mode != "position") {
      return refuse(following, mode);
    }
    return action;
  }

  auto read_global_action(pugi::xml_node global)
      -> std::expected<GlobalAction, lib::Status> {
    if (global.child("ParameterAction")) {
      return read_parameter_action(global.child("ParameterAction"));
    }
    pugi::xml_node signal =
        global.child("InfrastructureAction").child("TrafficSignalAction");
    if (!signal) {
      return refuse(global.first_child());
    }
    if (pugi::xml_node state = signal.child("TrafficSignalStateAction")) {
      TrafficSignalStateAction action;
      RETURN_OR_ASSIGN(action.signal, read_text(state, "name"));
      RETURN_OR_ASSIGN(action.state, read_text(state, "state"));
      return action;
    }
    if (pugi::xml_node phase = signal.child("TrafficSignalControllerAction")) {
      TrafficSignalControllerAction action;
      RETURN_OR_ASSIGN(action.controller,
                       read_text(phase, "trafficSignalControllerRef"));
      RETURN_OR_ASSIGN(action.phase, read_text(phase, "phase"));
      return action;
    }
    return refuse(signal.first_child());
  }

  auto read_parameter_action(pugi::xml_node node)
      -> std::expected<ParameterAction, lib::Status> {
    ParameterAction action;
    RETURN_OR_ASSIGN(action.parameter, read_text(node, "parameterRef"));
    pugi::xml_node change = node.first_child();
    std::string_view kind = change.name();
    if (kind == "SetAction") {
      action.kind = ParameterAction::Kind::SET;
      RETURN_OR_ASSIGN(action.value, read_text(change, "value"));
    } else if (kind == "ModifyAction") {
      pugi::xml_node rule = change.child("Rule").first_child();
      action.kind = std::string_view{rule.name()} == "AddValue"
                        ? ParameterAction::Kind::ADD
                        : ParameterAction::Kind::MULTIPLY;
      RETURN_OR_ASSIGN(action.value, read_text(rule, "value"));
    } else {
      return refuse(change);
    }
    return action;
  }

  //-- Conditions -------------------------------------------------------------

  auto read_rule(pugi::xml_node node) const
      -> std::expected<Rule, lib::Status> {
    RETURN_OR_ASSIGN(std::string rule, read_text(node, "rule"));
    if (rule == "greaterThan") {
      return Rule::GREATER_THAN;
    }
    if (rule == "lessThan") {
      return Rule::LESS_THAN;
    }
    if (rule == "equalTo") {
      return Rule::EQUAL_TO;
    }
    if (rule == "greaterOrEqual") {
      return Rule::GREATER_OR_EQUAL;
    }
    if (rule == "lessOrEqual") {
      return Rule::LESS_OR_EQUAL;
    }
    if (rule == "notEqualTo") {
      return Rule::NOT_EQUAL_TO;
    }
    return fail(node, "unknown rule " + rule);
  }

  // How a condition measures distance. In 1.0, alongRoute: longitudinal
  // along the road, or straight. After, a coordinate system, the entity's by
  // default, and a relative distance type, straight by default. With
  // freespace, only along the road, as bounding boxes are not yet measured
  // otherwise.
  auto read_relative_distance(pugi::xml_node node) const
      -> std::expected<RelativeDistance, lib::Status> {
    RelativeDistance distance;
    RETURN_OR_ASSIGN(distance.freespace, read_flag(node, "freespace", false));
    if (node.attribute("alongRoute")) {
      RETURN_OR_ASSIGN(distance.along_road,
                       read_flag(node, "alongRoute", false));
      if (distance.along_road) {
        distance.kind = RelativeDistance::Kind::LONGITUDINAL;
      }
    } else {
      if (node.attribute("coordinateSystem")) {
        RETURN_OR_ASSIGN(std::string system,
                         read_text(node, "coordinateSystem"));
        distance.along_road = system == "road" || system == "lane";
      }
      if (node.attribute("relativeDistanceType")) {
        RETURN_OR_ASSIGN(std::string type,
                         read_text(node, "relativeDistanceType"));
        distance.kind = type == "longitudinal"
                            ? RelativeDistance::Kind::LONGITUDINAL
                        : type == "lateral" ? RelativeDistance::Kind::LATERAL
                                            : RelativeDistance::Kind::EUCLIDEAN;
      }
    }
    if (distance.freespace &&
        !(distance.along_road &&
          distance.kind == RelativeDistance::Kind::LONGITUDINAL)) {
      return refuse(node, "freespace other than longitudinal along the road");
    }
    return distance;
  }

  auto read_entity_condition(pugi::xml_node node)
      -> std::expected<EntityConditionKind, lib::Status> {
    std::string_view kind = node.name();
    if (kind == "SpeedCondition") {
      SpeedCondition speed;
      RETURN_OR_ASSIGN(speed.value, read_number(node, "value"));
      RETURN_OR_ASSIGN(speed.rule, read_rule(node));
      return speed;
    }
    if (kind == "AccelerationCondition") {
      AccelerationCondition acceleration;
      RETURN_OR_ASSIGN(acceleration.value, read_number(node, "value"));
      RETURN_OR_ASSIGN(acceleration.rule, read_rule(node));
      return acceleration;
    }
    if (kind == "TimeHeadwayCondition") {
      TimeHeadwayCondition headway;
      RETURN_OR_ASSIGN(headway.entity, read_text(node, "entityRef"));
      RETURN_OR_ASSIGN(headway.value, read_number(node, "value"));
      RETURN_OR_ASSIGN(headway.distance, read_relative_distance(node));
      RETURN_OR_ASSIGN(headway.rule, read_rule(node));
      return headway;
    }
    if (kind == "RelativeDistanceCondition") {
      RelativeDistanceCondition distance;
      RETURN_OR_ASSIGN(distance.entity, read_text(node, "entityRef"));
      RETURN_OR_ASSIGN(distance.value, read_number(node, "value"));
      RETURN_OR_ASSIGN(distance.distance, read_relative_distance(node));
      RETURN_OR_ASSIGN(distance.rule, read_rule(node));
      return distance;
    }
    if (kind == "ReachPositionCondition") {
      ReachPositionCondition reach;
      RETURN_OR_ASSIGN(reach.tolerance, read_number(node, "tolerance"));
      RETURN_OR_ASSIGN(reach.position, read_position(node.child("Position")));
      return reach;
    }
    if (kind == "DistanceCondition") {
      DistanceCondition distance;
      RETURN_OR_ASSIGN(distance.value, read_number(node, "value"));
      RETURN_OR_ASSIGN(distance.distance, read_relative_distance(node));
      if (distance.distance.freespace) {
        return refuse(node, "freespace");
      }
      RETURN_OR_ASSIGN(distance.rule, read_rule(node));
      RETURN_OR_ASSIGN(distance.position,
                       read_position(node.child("Position")));
      return distance;
    }
    if (kind == "EndOfRoadCondition") {
      EndOfRoadCondition end;
      RETURN_OR_ASSIGN(end.duration, read_number(node, "duration"));
      return end;
    }
    if (kind == "OffroadCondition") {
      OffroadCondition offroad;
      RETURN_OR_ASSIGN(offroad.duration, read_number(node, "duration"));
      return offroad;
    }
    return refuse(node);
  }

  auto read_value_condition(pugi::xml_node node)
      -> std::expected<ValueCondition, lib::Status> {
    std::string_view kind = node.name();
    if (kind == "SimulationTimeCondition") {
      SimulationTimeCondition time;
      RETURN_OR_ASSIGN(time.value, read_number(node, "value"));
      RETURN_OR_ASSIGN(time.rule, read_rule(node));
      return time;
    }
    if (kind == "ParameterCondition") {
      ParameterCondition parameter;
      RETURN_OR_ASSIGN(parameter.parameter, read_text(node, "parameterRef"));
      RETURN_OR_ASSIGN(parameter.value, read_text(node, "value"));
      RETURN_OR_ASSIGN(parameter.rule, read_rule(node));
      return parameter;
    }
    if (kind == "StoryboardElementStateCondition") {
      StoryboardElementStateCondition state;
      RETURN_OR_ASSIGN(state.element, read_text(node, "storyboardElementRef"));
      RETURN_OR_ASSIGN(std::string type,
                       read_text(node, "storyboardElementType"));
      static constexpr std::array<
          std::pair<std::string_view, StoryboardElementType>, 6>
          TYPES{{{"story", StoryboardElementType::STORY},
                 {"act", StoryboardElementType::ACT},
                 {"maneuverGroup", StoryboardElementType::MANEUVER_GROUP},
                 {"maneuver", StoryboardElementType::MANEUVER},
                 {"event", StoryboardElementType::EVENT},
                 {"action", StoryboardElementType::ACTION}}};
      auto found_type =
          std::ranges::find(TYPES, type, &decltype(TYPES)::value_type::first);
      if (found_type == TYPES.end()) {
        return fail(node, "unknown storyboardElementType " + type);
      }
      state.type = found_type->second;
      RETURN_OR_ASSIGN(std::string name, read_text(node, "state"));
      static constexpr std::array<
          std::pair<std::string_view, StoryboardElementState>, 7>
          STATES{{{"standbyState", StoryboardElementState::STANDBY},
                  {"runningState", StoryboardElementState::RUNNING},
                  {"completeState", StoryboardElementState::COMPLETE},
                  {"startTransition", StoryboardElementState::START_TRANSITION},
                  {"endTransition", StoryboardElementState::END_TRANSITION},
                  {"stopTransition", StoryboardElementState::STOP_TRANSITION},
                  {"skipTransition", StoryboardElementState::SKIP_TRANSITION}}};
      auto found_state =
          std::ranges::find(STATES, name, &decltype(STATES)::value_type::first);
      if (found_state == STATES.end()) {
        return fail(node, "unknown state " + name);
      }
      state.state = found_state->second;
      return state;
    }
    if (kind == "TrafficSignalCondition") {
      TrafficSignalCondition signal;
      RETURN_OR_ASSIGN(signal.signal, read_text(node, "name"));
      RETURN_OR_ASSIGN(signal.state, read_text(node, "state"));
      return signal;
    }
    if (kind == "TrafficSignalControllerCondition") {
      TrafficSignalControllerCondition phase;
      RETURN_OR_ASSIGN(phase.controller,
                       read_text(node, "trafficSignalControllerRef"));
      RETURN_OR_ASSIGN(phase.phase, read_text(node, "phase"));
      return phase;
    }
    return refuse(node);
  }

  auto read_condition(pugi::xml_node node)
      -> std::expected<Condition, lib::Status> {
    Condition condition{.name = node.attribute("name").as_string()};
    RETURN_OR_ASSIGN(condition.delay, read_number_or(node, "delay", 0.0));
    RETURN_OR_ASSIGN(std::string edge, read_text(node, "conditionEdge"));
    condition.edge = edge == "rising"    ? Condition::Edge::RISING
                     : edge == "falling" ? Condition::Edge::FALLING
                     : edge == "risingOrFalling"
                         ? Condition::Edge::RISING_OR_FALLING
                         : Condition::Edge::NONE;
    if (pugi::xml_node by_entity = node.child("ByEntityCondition")) {
      EntityCondition entity;
      pugi::xml_node triggering = by_entity.child("TriggeringEntities");
      RETURN_OR_ASSIGN(std::string rule,
                       read_text(triggering, "triggeringEntitiesRule"));
      entity.all = rule == "all";
      for (pugi::xml_node reference : triggering.children("EntityRef")) {
        RETURN_OR_ASSIGN(std::string name, read_text(reference, "entityRef"));
        entity.triggering.push_back(name);
      }
      RETURN_OR_ASSIGN(entity.condition,
                       read_entity_condition(
                           by_entity.child("EntityCondition").first_child()));
      condition.condition = std::move(entity);
    } else if (pugi::xml_node by_value = node.child("ByValueCondition")) {
      RETURN_OR_ASSIGN(ValueCondition value,
                       read_value_condition(by_value.first_child()));
      condition.condition = std::move(value);
    } else {
      return fail(node, condition.name + " has no condition");
    }
    return condition;
  }

  // A trigger, or none if it has no conditions.
  auto read_trigger(pugi::xml_node node)
      -> std::expected<std::optional<Trigger>, lib::Status> {
    if (!node || !node.child("ConditionGroup")) {
      return std::nullopt;
    }
    Trigger trigger;
    for (pugi::xml_node group : node.children("ConditionGroup")) {
      std::vector<Condition> conditions;
      for (pugi::xml_node condition : group.children("Condition")) {
        RETURN_OR_ASSIGN(Condition read, read_condition(condition));
        conditions.push_back(std::move(read));
      }
      trigger.groups.push_back(std::move(conditions));
    }
    return trigger;
  }

  //-- Storyboard -------------------------------------------------------------

  auto read_storyboard(pugi::xml_node node)
      -> std::expected<Storyboard, lib::Status> {
    Storyboard storyboard;
    for (pugi::xml_node actions :
         node.child("Init").child("Actions").children()) {
      std::string_view kind = actions.name();
      if (kind == "Private") {
        InitActions init;
        RETURN_OR_ASSIGN(init.entity, read_text(actions, "entityRef"));
        for (pugi::xml_node action : actions.children("PrivateAction")) {
          RETURN_OR_ASSIGN(PrivateAction read, read_private_action(action));
          init.actions.push_back(std::move(read));
        }
        storyboard.init.push_back(std::move(init));
      } else if (kind == "GlobalAction") {
        RETURN_OR_ASSIGN(GlobalAction read, read_global_action(actions));
        storyboard.global_init.push_back(std::move(read));
      } else {
        return refuse(actions);
      }
    }
    for (pugi::xml_node story : node.children("Story")) {
      RETURN_OR_ASSIGN(Story read, read_story(story));
      storyboard.stories.push_back(std::move(read));
    }
    RETURN_OR_ASSIGN(storyboard.stop, read_trigger(node.child("StopTrigger")));
    return storyboard;
  }

  auto read_story(pugi::xml_node node) -> std::expected<Story, lib::Status> {
    std::vector<Parameter> outer = scope_;
    std::vector<Parameter> local;
    RETURN_IF_UNEXPECTED(
        declare(node.child("ParameterDeclarations"), Out(local)));
    scope_.insert(scope_.end(), local.begin(), local.end());
    Story story{.name = node.attribute("name").as_string()};
    for (pugi::xml_node act : node.children("Act")) {
      auto read = read_act(act);
      if (!read) {
        scope_ = std::move(outer);
        return std::unexpected(read.error());
      }
      story.acts.push_back(std::move(*read));
    }
    scope_ = std::move(outer);
    return story;
  }

  auto read_act(pugi::xml_node node) -> std::expected<Act, lib::Status> {
    Act act{.name = node.attribute("name").as_string()};
    for (pugi::xml_node group : node.children("ManeuverGroup")) {
      ManeuverGroup read{.name = group.attribute("name").as_string()};
      RETURN_OR_ASSIGN(double executions,
                       read_number_or(group, "maximumExecutionCount", 1.0));
      read.maximum_executions = static_cast<int>(executions);
      for (pugi::xml_node actor : group.child("Actors").children("EntityRef")) {
        RETURN_OR_ASSIGN(std::string name, read_text(actor, "entityRef"));
        read.actors.push_back(name);
      }
      if (group.child("CatalogReference")) {
        return refuse(group.child("CatalogReference"));
      }
      for (pugi::xml_node maneuver : group.children("Maneuver")) {
        RETURN_OR_ASSIGN(Maneuver parsed, read_maneuver(maneuver));
        read.maneuvers.push_back(std::move(parsed));
      }
      act.groups.push_back(std::move(read));
    }
    RETURN_OR_ASSIGN(act.start, read_trigger(node.child("StartTrigger")));
    RETURN_OR_ASSIGN(act.stop, read_trigger(node.child("StopTrigger")));
    return act;
  }

  auto read_maneuver(pugi::xml_node node)
      -> std::expected<Maneuver, lib::Status> {
    Maneuver maneuver{.name = node.attribute("name").as_string()};
    for (pugi::xml_node event : node.children("Event")) {
      Event read{.name = event.attribute("name").as_string()};
      RETURN_OR_ASSIGN(std::string priority, read_text(event, "priority"));
      read.priority = priority == "skip"       ? Event::Priority::SKIP
                      : priority == "parallel" ? Event::Priority::PARALLEL
                                               : Event::Priority::OVERWRITE;
      RETURN_OR_ASSIGN(double executions,
                       read_number_or(event, "maximumExecutionCount", 1.0));
      read.maximum_executions = static_cast<int>(executions);
      for (pugi::xml_node action : event.children("Action")) {
        Action parsed{.name = action.attribute("name").as_string()};
        if (pugi::xml_node private_action = action.child("PrivateAction")) {
          RETURN_OR_ASSIGN(PrivateAction value,
                           read_private_action(private_action));
          parsed.action = std::move(value);
        } else if (pugi::xml_node global = action.child("GlobalAction")) {
          RETURN_OR_ASSIGN(GlobalAction value, read_global_action(global));
          parsed.action = std::move(value);
        } else {
          return refuse(action.first_child());
        }
        read.actions.push_back(std::move(parsed));
      }
      RETURN_OR_ASSIGN(read.start, read_trigger(event.child("StartTrigger")));
      maneuver.events.push_back(std::move(read));
    }
    return maneuver;
  }

  std::string directory_;
  std::span<const ParameterAssignment> assignments_;
  XmlDocument document_;
  std::vector<std::unique_ptr<XmlDocument>> catalogs_;
  std::vector<Parameter> scope_;
};

}  // namespace

auto evaluate_expression(std::string_view text,
                         const std::vector<Parameter>& parameters)
    -> std::expected<double, lib::Status> {
  return Expression{text, parameters}.evaluate();
}

auto parse_openscenario(std::string_view text, const std::string& directory,
                        std::span<const ParameterAssignment> assignments)
    -> std::expected<Scenario, lib::Status> {
  return Parser{directory, assignments}.parse(text);
}

auto load_openscenario(const std::string& path,
                       std::span<const ParameterAssignment> assignments)
    -> std::expected<Scenario, lib::Status> {
  RETURN_OR_ASSIGN(std::string text, read_text_file(path));
  return parse_openscenario(
      text, std::filesystem::path{path}.parent_path().string(), assignments);
}

auto parse_parameter_distribution(std::string_view text,
                                  const std::string& directory)
    -> std::expected<ParameterDistribution, lib::Status> {
  XmlDocument document;
  RETURN_IF_UNEXPECTED(document.load(std::string{text}));
  RETURN_OR_ASSIGN(pugi::xml_node scenario, document.find_root("OpenSCENARIO"));
  pugi::xml_node root = scenario.child("ParameterValueDistribution");
  if (!root) {
    return document.fail(scenario, "needs a ParameterValueDistribution");
  }
  std::string scenario_file =
      root.child("ScenarioFile").attribute("filepath").as_string();
  if (scenario_file.empty()) {
    return document.fail(root, "needs a ScenarioFile");
  }
  ParameterDistribution distribution{
      .scenario = (std::filesystem::path{directory} / scenario_file)
                      .lexically_normal()
                      .string()};
  if (pugi::xml_node stochastic = root.child("Stochastic")) {
    return document.refuse(stochastic);
  }
  pugi::xml_node deterministic = root.child("Deterministic");
  if (!deterministic) {
    return document.fail(root, "needs a distribution");
  }
  for (pugi::xml_node node : deterministic.children()) {
    std::vector<ParameterChoice> choices;
    std::string_view kind = node.name();
    if (kind == "DeterministicMultiParameterDistribution") {
      for (pugi::xml_node set :
           node.child("ValueSetDistribution").children("ParameterValueSet")) {
        ParameterChoice choice;
        for (pugi::xml_node assignment : set.children("ParameterAssignment")) {
          choice.push_back(
              {.name = assignment.attribute("parameterRef").as_string(),
               .value = assignment.attribute("value").as_string()});
        }
        choices.push_back(std::move(choice));
      }
    } else if (kind == "DeterministicSingleParameterDistribution") {
      std::string name = node.attribute("parameterName").as_string();
      if (name.empty()) {
        return document.fail(node, "needs a parameterName");
      }
      if (pugi::xml_node set = node.child("DistributionSet")) {
        for (pugi::xml_node element : set.children("Element")) {
          choices.push_back(
              {{.name = name,
                .value = element.attribute("value").as_string()}});
        }
      } else if (pugi::xml_node range = node.child("DistributionRange")) {
        RETURN_OR_ASSIGN(double step, document.read_number(range, "stepWidth"));
        pugi::xml_node limits = range.child("Range");
        RETURN_OR_ASSIGN(double lower,
                         document.read_number(limits, "lowerLimit"));
        RETURN_OR_ASSIGN(double upper,
                         document.read_number(limits, "upperLimit"));
        if (!(step > 0.0) || upper < lower) {
          return document.fail(
              range,
              "of " + name + " needs a positive step and a range lowest first");
        }
        // The steps that fit, allowing for the limits' rounding.
        auto steps = static_cast<std::size_t>(
            std::floor((upper - lower) / step * (1.0 + 1e-12)));
        for (std::size_t k = 0; k <= steps; ++k) {
          choices.push_back(
              {{.name = name,
                .value = std::format("{:.15g}",
                                     lower + static_cast<double>(k) * step)}});
        }
      } else {
        return document.refuse(node.first_child());
      }
    } else {
      return document.refuse(node);
    }
    if (choices.empty()) {
      return document.fail(node, "has no values");
    }
    distribution.distributions.push_back(std::move(choices));
  }
  return distribution;
}

auto load_parameter_distribution(const std::string& path)
    -> std::expected<ParameterDistribution, lib::Status> {
  RETURN_OR_ASSIGN(std::string text, read_text_file(path));
  return parse_parameter_distribution(
      text, std::filesystem::path{path}.parent_path().string());
}

}  // namespace simon::format
