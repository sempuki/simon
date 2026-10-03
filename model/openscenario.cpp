// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/openscenario.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <utility>

#include "base/core.hpp"
#include "pugixml.hpp"

template <>
const std::array<lib::StatusConditionEntry,
                 simon::model::openscenario::SCENARIO_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::model::openscenario::ScenarioError,
        simon::model::openscenario::SCENARIO_ERROR_COUNT>::conditions_ = {
        lib::StatusConditionEntry{"scenario unreadable"},
        lib::StatusConditionEntry{"scenario malformed"},
        lib::StatusConditionEntry{"scenario unsupported"},
};

namespace simon::model::openscenario {

auto Scenario::find_entity(std::string_view name) const -> const Entity* {
  auto found = std::ranges::find(entities, name, &Entity::name);
  return found == entities.end() ? nullptr : &*found;
}

namespace {

using Failure = std::unexpected<lib::Status>;
using lib::Out;

auto read_file(const std::string& path)
    -> std::expected<std::string, lib::Status> {
  std::ifstream file{path};
  if (!file) {
    return std::unexpected(
        lib::raise(ScenarioError::UNREADABLE, "cannot open " + path));
  }
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

auto parse_double(std::string_view text) -> std::optional<double> {
  while (!text.empty() &&
         std::isspace(static_cast<unsigned char>(text.front()))) {
    text.remove_prefix(1);
  }
  while (!text.empty() &&
         std::isspace(static_cast<unsigned char>(text.back()))) {
    text.remove_suffix(1);
  }
  if (!text.empty() && text.front() == '+') {
    text.remove_prefix(1);
  }
  double value = 0.0;
  auto [end, error] =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (error != std::errc{} || end != text.data() + text.size()) {
    return std::nullopt;
  }
  return value;
}

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

  auto evaluate() -> std::expected<double, std::string> {
    auto value = parse_expression();
    skip_space();
    if (value && at_ < text_.size()) {
      return std::unexpected("unexpected '" + std::string{text_.substr(at_)} +
                             "'");
    }
    return value;
  }

 private:
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

  auto parse_expression() -> std::expected<double, std::string> {
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

  auto parse_term() -> std::expected<double, std::string> {
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

  auto parse_unary() -> std::expected<double, std::string> {
    if (take('-')) {
      auto value = parse_unary();
      if (value) {
        *value = -*value;
      }
      return value;
    }
    return parse_primary();
  }

  auto parse_primary() -> std::expected<double, std::string> {
    skip_space();
    if (take('(')) {
      auto value = parse_expression();
      if (value && !take(')')) {
        return std::unexpected("missing ')'");
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
        return std::unexpected("no parameter " + std::string{name});
      }
      auto value = parse_double(found->value);
      if (!value) {
        return std::unexpected("parameter " + std::string{name} +
                               " is not a number");
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
    auto value = parse_double(text_.substr(start, at_ - start));
    if (!value) {
      return std::unexpected("expected a number at '" +
                             std::string{text_.substr(start)} + "'");
    }
    return *value;
  }

  std::string_view text_;
  const std::vector<Parameter>* parameters_ = nullptr;
  std::size_t at_ = 0;
};

// Reads one scenario, its parameters in scopes: the file's, then a story's
// or a catalog entry's, innermost last.
class Parser final {
 public:
  explicit Parser(std::string directory) : directory_{std::move(directory)} {}

  auto parse(std::string_view text) -> std::expected<Scenario, lib::Status> {
    pugi::xml_parse_result result =
        document_.load_buffer(text.data(), text.size());
    if (!result) {
      return fail(result.description());
    }
    pugi::xml_node root = document_.child("OpenSCENARIO");
    if (!root) {
      return fail("no OpenSCENARIO element");
    }
    Scenario scenario;
    RETURN_IF_UNEXPECTED(
        declare(root.child("ParameterDeclarations"), Out(scenario.parameters)));
    scope_ = scenario.parameters;
    RETURN_IF_UNEXPECTED(read_catalogs(root.child("CatalogLocations")));

    pugi::xml_node logic = root.child("RoadNetwork").child("LogicFile");
    if (!logic) {
      return fail("<RoadNetwork> needs a LogicFile");
    }
    RETURN_OR_ASSIGN(std::string road, text_of(logic, "filepath"));
    scenario.road_network = resolve(road);

    for (pugi::xml_node object : root.child("Entities").children()) {
      RETURN_OR_ASSIGN(Entity entity, read_entity(object));
      scenario.entities.push_back(std::move(entity));
    }
    RETURN_OR_ASSIGN(scenario.storyboard,
                     read_storyboard(root.child("Storyboard")));
    return scenario;
  }

 private:
  auto fail(std::string_view why) const -> Failure {
    return Failure{lib::raise(ScenarioError::MALFORMED, std::string{why})};
  }
  auto refuse(pugi::xml_node node) const -> Failure {
    return Failure{lib::raise(ScenarioError::UNSUPPORTED,
                              "<" + std::string{node.name()} + ">")};
  }

  auto resolve(const std::string& path) const -> std::string {
    std::filesystem::path relative{path};
    return relative.is_absolute()
               ? path
               : (std::filesystem::path{directory_} / relative)
                     .lexically_normal()
                     .string();
  }

  // Adds a scope's declarations to `parameters`, each value resolved in the
  // scope so far.
  auto declare(pugi::xml_node declarations, Out<std::vector<Parameter>> into)
      -> std::expected<void, lib::Status> {
    for (pugi::xml_node node : declarations.children("ParameterDeclaration")) {
      std::vector<Parameter> visible = scope_;
      visible.insert(visible.end(), into->begin(), into->end());
      std::swap(scope_, visible);
      auto value = text_of(node, "value");
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
  auto text_of(pugi::xml_node node, std::string_view name) const
      -> std::expected<std::string, lib::Status> {
    pugi::xml_attribute attribute = node.attribute(std::string{name}.c_str());
    if (!attribute) {
      return fail("<" + std::string{node.name()} + "> needs " +
                  std::string{name});
    }
    std::string text = attribute.as_string();
    if (text.starts_with("${") && text.ends_with("}")) {
      auto value = evaluate_expression(
          std::string_view{text}.substr(2, text.size() - 3), scope_);
      if (!value) {
        return fail("<" + std::string{node.name()} + "> " + std::string{name} +
                    ": " + value.error());
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
      return fail("no parameter " + text.substr(1));
    }
    return text;
  }

  auto number_of(pugi::xml_node node, std::string_view name) const
      -> std::expected<double, lib::Status> {
    RETURN_OR_ASSIGN(std::string text, text_of(node, name));
    auto value = parse_double(text);
    if (!value) {
      return fail("<" + std::string{node.name()} + "> " + std::string{name} +
                  " is not a number: " + text);
    }
    return *value;
  }

  auto number_or(pugi::xml_node node, std::string_view name,
                 double otherwise) const -> std::expected<double, lib::Status> {
    if (!node.attribute(std::string{name}.c_str())) {
      return otherwise;
    }
    return number_of(node, name);
  }

  auto flag_of(pugi::xml_node node, std::string_view name, bool otherwise) const
      -> std::expected<bool, lib::Status> {
    if (!node.attribute(std::string{name}.c_str())) {
      return otherwise;
    }
    RETURN_OR_ASSIGN(std::string text, text_of(node, name));
    return text == "true" || text == "1";
  }

  //-- Catalogs ---------------------------------------------------------------

  auto read_catalogs(pugi::xml_node locations)
      -> std::expected<void, lib::Status> {
    for (pugi::xml_node location : locations.children()) {
      RETURN_OR_ASSIGN(std::string path,
                       text_of(location.child("Directory"), "path"));
      std::filesystem::path directory{resolve(path)};
      if (!std::filesystem::is_directory(directory)) {
        continue;  // As esmini does, a missing catalog is only an error if
                   // used.
      }
      for (const auto& file : std::filesystem::directory_iterator{directory}) {
        if (file.path().extension() != ".xosc") {
          continue;
        }
        auto text = read_file(file.path().string());
        if (!text) {
          return std::unexpected(text.error());
        }
        auto document = std::make_unique<pugi::xml_document>();
        if (!document->load_string(text->c_str())) {
          return fail("catalog " + file.path().string() + " is not XML");
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
      pugi::xml_node root = document->child("OpenSCENARIO").child("Catalog");
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
      } else if (kind == "CatalogReference") {
        RETURN_OR_ASSIGN(std::string catalog, text_of(child, "catalogName"));
        RETURN_OR_ASSIGN(std::string name, text_of(child, "entryName"));
        pugi::xml_node vehicle = find_catalog_entry(catalog, name);
        if (!vehicle) {
          return fail("no catalog entry " + catalog + "/" + name);
        }
        if (std::string_view{vehicle.name()} != "Vehicle") {
          return refuse(vehicle);
        }
        // The entry's own parameters, overridden by the reference's.
        std::vector<Parameter> outer = scope_;
        std::vector<Parameter> local;
        RETURN_IF_UNEXPECTED(
            declare(vehicle.child("ParameterDeclarations"), Out(local)));
        for (pugi::xml_node assignment :
             child.child("ParameterAssignments").children()) {
          RETURN_OR_ASSIGN(std::string value, text_of(assignment, "value"));
          std::string reference =
              assignment.attribute("parameterRef").as_string();
          auto found = std::ranges::find(local, reference, &Parameter::name);
          if (found != local.end()) {
            found->value = value;
          }
        }
        scope_.insert(scope_.end(), local.begin(), local.end());
        auto read = read_vehicle(vehicle);
        scope_ = std::move(outer);
        if (!read) {
          return std::unexpected(read.error());
        }
        entity.vehicle = std::move(*read);
      } else if (kind == "ObjectController") {
        return refuse(child);
      } else {
        return refuse(child);
      }
    }
    return entity;
  }

  auto read_vehicle(pugi::xml_node node)
      -> std::expected<Vehicle, lib::Status> {
    Vehicle vehicle{.name = node.attribute("name").as_string(),
                    .category = node.attribute("vehicleCategory").as_string()};
    pugi::xml_node box = node.child("BoundingBox");
    RETURN_OR_ASSIGN(vehicle.center[0], number_of(box.child("Center"), "x"));
    RETURN_OR_ASSIGN(vehicle.center[1], number_of(box.child("Center"), "y"));
    RETURN_OR_ASSIGN(vehicle.center[2], number_of(box.child("Center"), "z"));
    RETURN_OR_ASSIGN(vehicle.dimensions[0],
                     number_of(box.child("Dimensions"), "length"));
    RETURN_OR_ASSIGN(vehicle.dimensions[1],
                     number_of(box.child("Dimensions"), "width"));
    RETURN_OR_ASSIGN(vehicle.dimensions[2],
                     number_of(box.child("Dimensions"), "height"));
    pugi::xml_node performance = node.child("Performance");
    RETURN_OR_ASSIGN(vehicle.max_speed, number_of(performance, "maxSpeed"));
    RETURN_OR_ASSIGN(vehicle.max_acceleration,
                     number_of(performance, "maxAcceleration"));
    RETURN_OR_ASSIGN(vehicle.max_deceleration,
                     number_of(performance, "maxDeceleration"));
    pugi::xml_node axles = node.child("Axles");
    RETURN_OR_ASSIGN(double front,
                     number_or(axles.child("FrontAxle"), "positionX", 0.0));
    RETURN_OR_ASSIGN(double rear,
                     number_or(axles.child("RearAxle"), "positionX", 0.0));
    vehicle.wheelbase = front - rear;
    // A trailer is an entity of its own, which simon does not tow.
    if (pugi::xml_node trailer = node.child("Trailer")) {
      RETURN_OR_ASSIGN(std::string reference,
                       text_of(trailer.child("TrailerRef"), "entityRef"));
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
      RETURN_OR_ASSIGN(std::string type, text_of(orientation, "type"));
      result.relative = type != "absolute";
    }
    RETURN_OR_ASSIGN(result.h, number_or(orientation, "h", 0.0));
    return result;
  }

  auto read_position(pugi::xml_node position)
      -> std::expected<Position, lib::Status> {
    pugi::xml_node node = position.first_child();
    std::string_view kind = node.name();
    if (kind == "WorldPosition") {
      WorldPosition world;
      RETURN_OR_ASSIGN(world.x, number_of(node, "x"));
      RETURN_OR_ASSIGN(world.y, number_of(node, "y"));
      RETURN_OR_ASSIGN(world.z, number_or(node, "z", 0.0));
      RETURN_OR_ASSIGN(world.h, number_or(node, "h", 0.0));
      return world;
    }
    if (kind == "LanePosition") {
      LanePosition lane;
      RETURN_OR_ASSIGN(lane.road, text_of(node, "roadId"));
      RETURN_OR_ASSIGN(double id, number_of(node, "laneId"));
      lane.lane = static_cast<int>(id);
      RETURN_OR_ASSIGN(lane.s, number_of(node, "s"));
      RETURN_OR_ASSIGN(lane.offset, number_or(node, "offset", 0.0));
      RETURN_OR_ASSIGN(lane.orientation, read_orientation(node));
      return lane;
    }
    if (kind == "RoadPosition") {
      RoadPosition road;
      RETURN_OR_ASSIGN(road.road, text_of(node, "roadId"));
      RETURN_OR_ASSIGN(road.s, number_of(node, "s"));
      RETURN_OR_ASSIGN(road.t, number_of(node, "t"));
      RETURN_OR_ASSIGN(road.orientation, read_orientation(node));
      return road;
    }
    if (kind == "RelativeRoadPosition") {
      RelativeRoadPosition relative;
      RETURN_OR_ASSIGN(relative.entity, text_of(node, "entityRef"));
      RETURN_OR_ASSIGN(relative.ds, number_of(node, "ds"));
      RETURN_OR_ASSIGN(relative.dt, number_of(node, "dt"));
      RETURN_OR_ASSIGN(relative.orientation, read_orientation(node));
      return relative;
    }
    if (kind == "RelativeLanePosition") {
      RelativeLanePosition relative;
      RETURN_OR_ASSIGN(relative.entity, text_of(node, "entityRef"));
      RETURN_OR_ASSIGN(double lanes, number_of(node, "dLane"));
      relative.lanes = static_cast<int>(lanes);
      RETURN_OR_ASSIGN(relative.ds, number_or(node, "ds", 0.0));
      RETURN_OR_ASSIGN(relative.offset, number_or(node, "offset", 0.0));
      RETURN_OR_ASSIGN(relative.orientation, read_orientation(node));
      return relative;
    }
    return refuse(node);
  }

  //-- Actions ----------------------------------------------------------------

  auto read_dynamics(pugi::xml_node node)
      -> std::expected<TransitionDynamics, lib::Status> {
    TransitionDynamics dynamics;
    RETURN_OR_ASSIGN(std::string shape, text_of(node, "dynamicsShape"));
    RETURN_OR_ASSIGN(DynamicsShape parsed, read_shape(node, shape));
    dynamics.shape = parsed;
    RETURN_OR_ASSIGN(std::string dimension, text_of(node, "dynamicsDimension"));
    if (dimension == "time") {
      dynamics.dimension = DynamicsDimension::TIME;
    } else if (dimension == "distance") {
      dynamics.dimension = DynamicsDimension::DISTANCE;
    } else if (dimension == "rate") {
      dynamics.dimension = DynamicsDimension::RATE;
    } else {
      return fail("unknown dynamicsDimension " + dimension);
    }
    RETURN_OR_ASSIGN(dynamics.value, number_of(node, "value"));
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
    return fail("<" + std::string{node.name()} + "> unknown shape " +
                std::string{shape});
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
        RETURN_OR_ASSIGN(double value, number_of(target, "value"));
        result.target = AbsoluteTargetSpeed{.value = value};
      } else if (std::string_view{target.name()} == "RelativeTargetSpeed") {
        RelativeTargetSpeed relative;
        RETURN_OR_ASSIGN(relative.entity, text_of(target, "entityRef"));
        RETURN_OR_ASSIGN(relative.value, number_of(target, "value"));
        RETURN_OR_ASSIGN(std::string type,
                         text_of(target, "speedTargetValueType"));
        relative.kind = type == "factor" ? RelativeTargetSpeed::Kind::FACTOR
                                         : RelativeTargetSpeed::Kind::DELTA;
        RETURN_OR_ASSIGN(relative.continuous,
                         flag_of(target, "continuous", false));
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
                         number_or(lateral, "targetLaneOffset", 0.0));
        pugi::xml_node target = lateral.child("LaneChangeTarget").first_child();
        if (std::string_view{target.name()} == "AbsoluteTargetLane") {
          RETURN_OR_ASSIGN(double lane, number_of(target, "value"));
          result.target = AbsoluteTargetLane{.lane = static_cast<int>(lane)};
        } else if (std::string_view{target.name()} == "RelativeTargetLane") {
          RETURN_OR_ASSIGN(std::string entity, text_of(target, "entityRef"));
          RETURN_OR_ASSIGN(double lanes, number_of(target, "value"));
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
                         flag_of(lateral, "continuous", false));
        pugi::xml_node dynamics = lateral.child("LaneOffsetActionDynamics");
        RETURN_OR_ASSIGN(std::string shape, text_of(dynamics, "dynamicsShape"));
        RETURN_OR_ASSIGN(result.shape, read_shape(dynamics, shape));
        RETURN_OR_ASSIGN(result.max_lateral_acceleration,
                         number_of(dynamics, "maxLateralAcc"));
        pugi::xml_node target = lateral.child("LaneOffsetTarget").first_child();
        if (std::string_view{target.name()} == "AbsoluteTargetLaneOffset") {
          RETURN_OR_ASSIGN(result.value, number_of(target, "value"));
        } else if (std::string_view{target.name()} ==
                   "RelativeTargetLaneOffset") {
          RETURN_OR_ASSIGN(std::string entity, text_of(target, "entityRef"));
          result.relative_to = entity;
          RETURN_OR_ASSIGN(result.value, number_of(target, "value"));
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
    return refuse(kind);
  }

  auto read_parameter_action(pugi::xml_node global)
      -> std::expected<ParameterAction, lib::Status> {
    pugi::xml_node node = global.child("ParameterAction");
    if (!node) {
      return refuse(global.first_child());
    }
    ParameterAction action;
    RETURN_OR_ASSIGN(action.parameter, text_of(node, "parameterRef"));
    pugi::xml_node change = node.first_child();
    std::string_view kind = change.name();
    if (kind == "SetAction") {
      action.kind = ParameterAction::Kind::SET;
      RETURN_OR_ASSIGN(action.value, text_of(change, "value"));
    } else if (kind == "ModifyAction") {
      pugi::xml_node rule = change.child("Rule").first_child();
      action.kind = std::string_view{rule.name()} == "AddValue"
                        ? ParameterAction::Kind::ADD
                        : ParameterAction::Kind::MULTIPLY;
      RETURN_OR_ASSIGN(action.value, text_of(rule, "value"));
    } else {
      return refuse(change);
    }
    return action;
  }

  //-- Conditions -------------------------------------------------------------

  auto read_rule(pugi::xml_node node) const
      -> std::expected<Rule, lib::Status> {
    RETURN_OR_ASSIGN(std::string rule, text_of(node, "rule"));
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
    return fail("unknown rule " + rule);
  }

  // Whether distances are along the road: alongRoute in 1.0, the road or
  // lane coordinate system after.
  auto read_along_road(pugi::xml_node node) const
      -> std::expected<bool, lib::Status> {
    if (node.attribute("alongRoute")) {
      return flag_of(node, "alongRoute", false);
    }
    if (node.attribute("coordinateSystem")) {
      RETURN_OR_ASSIGN(std::string system, text_of(node, "coordinateSystem"));
      return system == "road" || system == "lane";
    }
    return false;
  }

  auto read_entity_condition(pugi::xml_node node)
      -> std::expected<EntityConditionKind, lib::Status> {
    std::string_view kind = node.name();
    if (kind == "SpeedCondition") {
      SpeedCondition speed;
      RETURN_OR_ASSIGN(speed.value, number_of(node, "value"));
      RETURN_OR_ASSIGN(speed.rule, read_rule(node));
      return speed;
    }
    if (kind == "AccelerationCondition") {
      AccelerationCondition acceleration;
      RETURN_OR_ASSIGN(acceleration.value, number_of(node, "value"));
      RETURN_OR_ASSIGN(acceleration.rule, read_rule(node));
      return acceleration;
    }
    if (kind == "TimeHeadwayCondition") {
      TimeHeadwayCondition headway;
      RETURN_OR_ASSIGN(headway.entity, text_of(node, "entityRef"));
      RETURN_OR_ASSIGN(headway.value, number_of(node, "value"));
      RETURN_OR_ASSIGN(headway.freespace, flag_of(node, "freespace", false));
      RETURN_OR_ASSIGN(headway.along_road, read_along_road(node));
      RETURN_OR_ASSIGN(headway.rule, read_rule(node));
      return headway;
    }
    if (kind == "RelativeDistanceCondition") {
      RelativeDistanceCondition distance;
      RETURN_OR_ASSIGN(distance.entity, text_of(node, "entityRef"));
      RETURN_OR_ASSIGN(distance.value, number_of(node, "value"));
      RETURN_OR_ASSIGN(distance.freespace, flag_of(node, "freespace", false));
      RETURN_OR_ASSIGN(distance.along_road, read_along_road(node));
      RETURN_OR_ASSIGN(distance.rule, read_rule(node));
      RETURN_OR_ASSIGN(std::string type, text_of(node, "relativeDistanceType"));
      distance.kind =
          type == "longitudinal" ? RelativeDistanceCondition::Kind::LONGITUDINAL
          : type == "lateral"    ? RelativeDistanceCondition::Kind::LATERAL
                                 : RelativeDistanceCondition::Kind::CARTESIAN;
      return distance;
    }
    if (kind == "ReachPositionCondition") {
      ReachPositionCondition reach;
      RETURN_OR_ASSIGN(reach.tolerance, number_of(node, "tolerance"));
      RETURN_OR_ASSIGN(reach.position, read_position(node.child("Position")));
      return reach;
    }
    if (kind == "EndOfRoadCondition") {
      EndOfRoadCondition end;
      RETURN_OR_ASSIGN(end.duration, number_of(node, "duration"));
      return end;
    }
    if (kind == "OffroadCondition") {
      OffroadCondition offroad;
      RETURN_OR_ASSIGN(offroad.duration, number_of(node, "duration"));
      return offroad;
    }
    return refuse(node);
  }

  auto read_value_condition(pugi::xml_node node)
      -> std::expected<ValueCondition, lib::Status> {
    std::string_view kind = node.name();
    if (kind == "SimulationTimeCondition") {
      SimulationTimeCondition time;
      RETURN_OR_ASSIGN(time.value, number_of(node, "value"));
      RETURN_OR_ASSIGN(time.rule, read_rule(node));
      return time;
    }
    if (kind == "ParameterCondition") {
      ParameterCondition parameter;
      RETURN_OR_ASSIGN(parameter.parameter, text_of(node, "parameterRef"));
      RETURN_OR_ASSIGN(parameter.value, text_of(node, "value"));
      RETURN_OR_ASSIGN(parameter.rule, read_rule(node));
      return parameter;
    }
    if (kind == "StoryboardElementStateCondition") {
      StoryboardElementStateCondition state;
      RETURN_OR_ASSIGN(state.element, text_of(node, "storyboardElementRef"));
      RETURN_OR_ASSIGN(std::string type,
                       text_of(node, "storyboardElementType"));
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
        return fail("unknown storyboardElementType " + type);
      }
      state.type = found_type->second;
      RETURN_OR_ASSIGN(std::string name, text_of(node, "state"));
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
        return fail("unknown state " + name);
      }
      state.state = found_state->second;
      return state;
    }
    return refuse(node);
  }

  auto read_condition(pugi::xml_node node)
      -> std::expected<Condition, lib::Status> {
    Condition condition{.name = node.attribute("name").as_string()};
    RETURN_OR_ASSIGN(condition.delay, number_or(node, "delay", 0.0));
    RETURN_OR_ASSIGN(std::string edge, text_of(node, "conditionEdge"));
    condition.edge = edge == "rising"    ? Condition::Edge::RISING
                     : edge == "falling" ? Condition::Edge::FALLING
                     : edge == "risingOrFalling"
                         ? Condition::Edge::RISING_OR_FALLING
                         : Condition::Edge::NONE;
    if (pugi::xml_node by_entity = node.child("ByEntityCondition")) {
      EntityCondition entity;
      pugi::xml_node triggering = by_entity.child("TriggeringEntities");
      RETURN_OR_ASSIGN(std::string rule,
                       text_of(triggering, "triggeringEntitiesRule"));
      entity.all = rule == "all";
      for (pugi::xml_node reference : triggering.children("EntityRef")) {
        RETURN_OR_ASSIGN(std::string name, text_of(reference, "entityRef"));
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
      return fail("<Condition> " + condition.name + " has no condition");
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
        RETURN_OR_ASSIGN(init.entity, text_of(actions, "entityRef"));
        for (pugi::xml_node action : actions.children("PrivateAction")) {
          RETURN_OR_ASSIGN(PrivateAction read, read_private_action(action));
          init.actions.push_back(std::move(read));
        }
        storyboard.init.push_back(std::move(init));
      } else if (kind == "GlobalAction") {
        RETURN_OR_ASSIGN(ParameterAction read, read_parameter_action(actions));
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
                       number_or(group, "maximumExecutionCount", 1.0));
      read.maximum_executions = static_cast<int>(executions);
      for (pugi::xml_node actor : group.child("Actors").children("EntityRef")) {
        RETURN_OR_ASSIGN(std::string name, text_of(actor, "entityRef"));
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
      RETURN_OR_ASSIGN(std::string priority, text_of(event, "priority"));
      read.priority = priority == "skip"       ? Event::Priority::SKIP
                      : priority == "parallel" ? Event::Priority::PARALLEL
                                               : Event::Priority::OVERWRITE;
      RETURN_OR_ASSIGN(double executions,
                       number_or(event, "maximumExecutionCount", 1.0));
      read.maximum_executions = static_cast<int>(executions);
      for (pugi::xml_node action : event.children("Action")) {
        Action parsed{.name = action.attribute("name").as_string()};
        if (pugi::xml_node private_action = action.child("PrivateAction")) {
          RETURN_OR_ASSIGN(PrivateAction value,
                           read_private_action(private_action));
          parsed.action = std::move(value);
        } else if (pugi::xml_node global = action.child("GlobalAction")) {
          RETURN_OR_ASSIGN(ParameterAction value,
                           read_parameter_action(global));
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
  pugi::xml_document document_;
  std::vector<std::unique_ptr<pugi::xml_document>> catalogs_;
  std::vector<Parameter> scope_;
};

}  // namespace

auto evaluate_expression(std::string_view text,
                         const std::vector<Parameter>& parameters)
    -> std::expected<double, std::string> {
  return Expression{text, parameters}.evaluate();
}

auto parse_openscenario(std::string_view text, const std::string& directory)
    -> std::expected<Scenario, lib::Status> {
  return Parser{directory}.parse(text);
}

auto load_openscenario(const std::string& path)
    -> std::expected<Scenario, lib::Status> {
  RETURN_OR_ASSIGN(std::string text, read_file(path));
  return parse_openscenario(text,
                            std::filesystem::path{path}.parent_path().string());
}

}  // namespace simon::model::openscenario
