// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "scenario/storyboard.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <limits>
#include <numbers>
#include <type_traits>
#include <variant>

namespace simon::scenario {

using namespace model;

namespace {

constexpr std::size_t ROOT = 0;
constexpr std::size_t NO_ENTITY = std::numeric_limits<std::size_t>::max();
constexpr double SMALL = 1e-10;

auto parse_number(std::string_view text) -> std::optional<double> {
  double value = 0.0;
  auto [end, error] =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (error != std::errc{} || end != text.data() + text.size()) {
    return std::nullopt;
  }
  return value;
}

auto compare(double a, double b, Rule rule) -> bool {
  switch (rule) {
    case Rule::GREATER_THAN:
      return a > b;
    case Rule::LESS_THAN:
      return a < b;
    case Rule::EQUAL_TO:
      return std::abs(a - b) < SMALL;
    case Rule::GREATER_OR_EQUAL:
      return a >= b;
    case Rule::LESS_OR_EQUAL:
      return a <= b;
    case Rule::NOT_EQUAL_TO:
      return std::abs(a - b) >= SMALL;
  }
  return false;
}

// The domain a private action controls: speed, lateral position, or, for a
// teleport, neither.
enum class Domain : std::uint8_t { NONE, LONGITUDINAL, LATERAL };

auto domain_of(const PrivateAction* actor) -> Domain {
  if (actor == nullptr) {
    return Domain::NONE;
  }
  if (std::holds_alternative<SpeedAction>(*actor)) {
    return Domain::LONGITUDINAL;
  }
  if (std::holds_alternative<LaneChangeAction>(*actor) ||
      std::holds_alternative<LaneOffsetAction>(*actor) ||
      std::holds_alternative<FollowTrajectoryAction>(*actor)) {
    return Domain::LATERAL;
  }
  return Domain::NONE;
}

// Whether a private action is done as it starts: a teleport, or a route
// assigned.
auto is_instant(const PrivateAction& action) -> bool {
  return std::holds_alternative<TeleportAction>(action) ||
         std::holds_alternative<AssignRouteAction>(action);
}

}  // namespace

StoryboardPlayer::StoryboardPlayer(const Scenario& scenario,
                                   const RoadNetwork& network)
    : scenario_{&scenario},
      network_{&network},
      parameters_{scenario.parameters} {
  add(Element{.name = "storyboard",
              .stop_trigger = scenario.storyboard.stop
                                  ? &*scenario.storyboard.stop
                                  : nullptr});
  for (const InitActions& init : scenario.storyboard.init) {
    for (const PrivateAction& action : init.actions) {
      // Init actions stand outside the tree; their element only tracks them.
      add(Element{.type = StoryboardElementType::ACTION,
                  .name = "init",
                  .parent = NO_ENTITY,
                  .private_action = &action,
                  .entity = find_entity(init.entity)});
    }
  }
  for (const Story& story : scenario.storyboard.stories) {
    build(story);
  }
  // Each controller's cycle starts its delay after its reference's, the
  // references followed as far as they go.
  for (const TrafficSignalController& controller :
       scenario.signal_controllers) {
    ControllerState state{.controller = &controller};
    const TrafficSignalController* at = &controller;
    for (std::size_t depth = 0;
         at != nullptr && depth <= scenario.signal_controllers.size();
         ++depth) {
      state.offset += at->delay;
      auto reference =
          std::ranges::find(scenario.signal_controllers, at->reference,
                            &TrafficSignalController::name);
      at = at->reference.empty() ||
                   reference == scenario.signal_controllers.end()
               ? nullptr
               : &*reference;
    }
    controllers_.push_back(state);
  }
}

auto StoryboardPlayer::add(Element element) -> std::size_t {
  elements_.push_back(std::move(element));
  std::size_t index = elements_.size() - 1;
  std::size_t parent = elements_[index].parent;
  if (index != ROOT && parent != NO_ENTITY) {
    elements_[parent].children.push_back(index);
  }
  return index;
}

auto StoryboardPlayer::build(const Story& story) -> void {
  std::size_t story_index = add(Element{.type = StoryboardElementType::STORY,
                                        .name = story.name,
                                        .parent = ROOT});
  for (const Act& act : story.acts) {
    std::size_t act_index =
        add(Element{.type = StoryboardElementType::ACT,
                    .name = act.name,
                    .parent = story_index,
                    .start_trigger = act.start ? &*act.start : nullptr,
                    .stop_trigger = act.stop ? &*act.stop : nullptr});
    for (const ManeuverGroup& group : act.groups) {
      std::size_t group_index =
          add(Element{.type = StoryboardElementType::MANEUVER_GROUP,
                      .name = group.name,
                      .parent = act_index,
                      .maximum_executions = group.maximum_executions});
      for (const Maneuver& maneuver : group.maneuvers) {
        std::size_t maneuver_index =
            add(Element{.type = StoryboardElementType::MANEUVER,
                        .name = maneuver.name,
                        .parent = group_index});
        for (const Event& event : maneuver.events) {
          std::size_t event_index = add(
              Element{.type = StoryboardElementType::EVENT,
                      .name = event.name,
                      .parent = maneuver_index,
                      .maximum_executions = event.maximum_executions,
                      .start_trigger = event.start ? &*event.start : nullptr,
                      .event = &event});
          for (const Action& action : event.actions) {
            // A private action acts on each of the group's actors.
            if (std::holds_alternative<PrivateAction>(action.action)) {
              for (const std::string& actor : group.actors) {
                add(Element{
                    .type = StoryboardElementType::ACTION,
                    .name = action.name,
                    .parent = event_index,
                    .action = &action,
                    .private_action = &std::get<PrivateAction>(action.action),
                    .entity = find_entity(actor)});
              }
            } else {
              add(Element{.type = StoryboardElementType::ACTION,
                          .name = action.name,
                          .parent = event_index,
                          .action = &action,
                          .entity = NO_ENTITY});
            }
          }
        }
      }
    }
  }
}

auto StoryboardPlayer::find_entity(std::string_view name) const -> std::size_t {
  const auto& entities = scenario_->entities;
  auto found = std::ranges::find(entities, name, &Entity::name);
  return static_cast<std::size_t>(found - entities.begin());
}

auto StoryboardPlayer::find_parameter(std::string_view name) const
    -> const std::string* {
  auto found = std::ranges::find(parameters_, name, &Parameter::name);
  return found == parameters_.end() ? nullptr : &found->value;
}

auto StoryboardPlayer::find_signal_state(std::string_view signal) const
    -> const std::string* {
  auto found = std::ranges::find(signals_, signal, &TrafficSignalState::signal);
  return found == signals_.end() ? nullptr : &found->state;
}

auto StoryboardPlayer::find_phase(std::string_view controller) const
    -> const std::string* {
  auto found =
      std::ranges::find_if(controllers_, [&](const ControllerState& c) {
        return c.controller->name == controller;
      });
  if (found == controllers_.end() || found->phase == ControllerState::NONE) {
    return nullptr;
  }
  return &found->controller->phases[found->phase].name;
}

auto StoryboardPlayer::running() const -> bool {
  return elements_[ROOT].state != State::COMPLETE;
}

auto StoryboardPlayer::transitions() const -> const std::vector<Transition>& {
  return transitions_;
}

auto StoryboardPlayer::record(std::size_t index,
                              StoryboardElementState transition, double time)
    -> void {
  log_.emplace_back(index, transition);
  transitions_.push_back(Transition{.element = elements_[index].name,
                                    .transition = transition,
                                    .time = time});
}

//-- Lifecycle ----------------------------------------------------------------

auto StoryboardPlayer::start(double time) -> StoryboardOrders {
  orders_ = {};
  run_controllers(time);
  for (const GlobalAction& action : scenario_->storyboard.global_init) {
    apply(action, time);
  }
  for (std::size_t index = 0; index < elements_.size(); ++index) {
    Element& element = elements_[index];
    if (element.parent == NO_ENTITY) {
      element.state = State::RUNNING;
      orders_.starts.push_back(
          ActionOrder{.entity = element.entity,
                      .handle = static_cast<std::uint32_t>(index),
                      .action = element.private_action});
      if (is_instant(*element.private_action)) {
        instant_.push_back(index);
      }
    }
  }
  elements_[ROOT].state = State::RUNNING;
  for (std::size_t child : elements_[ROOT].children) {
    start_element(child, time);
  }
  return std::move(orders_);
}

// An element starts: to standby if it waits on a trigger, else running, its
// children started with it but for an event's, which its own start runs.
auto StoryboardPlayer::start_element(std::size_t index, double time) -> void {
  Element& element = elements_[index];
  if (element.type == StoryboardElementType::EVENT) {
    start_event(index, time);
    return;
  }
  if (element.state == State::INIT && element.start_trigger != nullptr) {
    element.state = State::STANDBY;
    return;
  }
  if (element.state == State::INIT || element.state == State::STANDBY ||
      element.executions > 0) {
    element.state = State::RUNNING;
    ++element.executions;
    record(index, StoryboardElementState::START_TRANSITION, time);
  }
  if (element.state != State::RUNNING) {
    return;
  }
  if (element.type == StoryboardElementType::ACTION) {
    const PrivateAction* actor = element.private_action;
    if (actor != nullptr) {
      orders_.starts.push_back(
          ActionOrder{.entity = element.entity,
                      .handle = static_cast<std::uint32_t>(index),
                      .action = actor});
      if (is_instant(*actor)) {
        instant_.push_back(index);
      }
    } else {
      apply(std::get<GlobalAction>(element.action->action), time);
      instant_.push_back(index);
    }
    return;
  }
  for (std::size_t child : std::vector<std::size_t>{element.children}) {
    start_element(child, time);
  }
}

// An event starts by its priority: overwriting ends its maneuver's running
// events, skipping waits while any runs, and parallel runs beside them. Its
// actions then end any running action of the same entity and domain.
auto StoryboardPlayer::start_event(std::size_t index, double time) -> void {
  Element& event = elements_[index];
  if (event.state == State::INIT && event.start_trigger != nullptr) {
    event.state = State::STANDBY;
    return;
  }
  const Element& maneuver = elements_[event.parent];
  if (event.state == State::RUNNING) {
    return;
  }
  Event::Priority priority = event.event->priority;
  if (priority == Event::Priority::SKIP &&
      std::ranges::any_of(maneuver.children, [&](std::size_t sibling) {
        return elements_[sibling].state == State::RUNNING;
      })) {
    return;
  }
  if (priority == Event::Priority::OVERWRITE) {
    for (std::size_t sibling : maneuver.children) {
      if (sibling != index && elements_[sibling].state == State::RUNNING) {
        end_element(sibling, time);
      }
    }
  }
  elements_[index].state = State::RUNNING;
  ++elements_[index].executions;
  record(index, StoryboardElementState::START_TRANSITION, time);
  if (elements_[index].children.empty()) {
    end_element(index, time);
    return;
  }
  for (std::size_t action :
       std::vector<std::size_t>{elements_[index].children}) {
    Domain domain = domain_of(elements_[action].private_action);
    std::size_t entity = elements_[action].entity;
    if (domain != Domain::NONE) {
      for (std::size_t i = 0; i < elements_.size(); ++i) {
        const Element& other = elements_[i];
        if (i == action || other.type != StoryboardElementType::ACTION ||
            other.state != State::RUNNING || other.entity != entity) {
          continue;
        }
        if (domain_of(other.private_action) == domain) {
          orders_.stops.push_back(static_cast<std::uint32_t>(i));
          end_element(i, time);
        }
      }
    }
    elements_[action].state = State::INIT;
    start_element(action, time);
  }
}

// An element ends: its running children end, and a maneuver group or event
// with executions left returns to standby, its triggers and children reset;
// anything else completes. A parent whose children are all complete ends.
auto StoryboardPlayer::end_element(std::size_t index, double time) -> void {
  Element& element = elements_[index];
  if (element.state == State::COMPLETE) {
    return;
  }
  for (std::size_t child : std::vector<std::size_t>{element.children}) {
    if (elements_[child].state == State::RUNNING) {
      end_element(child, time);
    }
  }
  if (elements_[index].state == State::COMPLETE) {
    return;
  }
  Element& ended = elements_[index];
  if (ended.state != State::RUNNING && ended.state != State::STANDBY) {
    return;
  }
  bool repeats = ended.type == StoryboardElementType::MANEUVER_GROUP ||
                 ended.type == StoryboardElementType::EVENT;
  if (repeats && ended.maximum_executions >= 0 &&
      ended.executions < ended.maximum_executions) {
    reset_trigger(ended.start_trigger);
    reset_trigger(ended.stop_trigger);
    ended.state = State::STANDBY;
    record(index, StoryboardElementState::END_TRANSITION, time);
    for (std::size_t child : std::vector<std::size_t>{ended.children}) {
      reset_element(child);
    }
  } else {
    ended.state = State::COMPLETE;
    record(index, StoryboardElementState::END_TRANSITION, time);
  }
  propagate(elements_[index].parent, time);
}

auto StoryboardPlayer::stop_element(std::size_t index, double time) -> void {
  for (std::size_t child :
       std::vector<std::size_t>{elements_[index].children}) {
    stop_element(child, time);
  }
  Element& element = elements_[index];
  if (element.state == State::INIT || element.state == State::COMPLETE) {
    return;
  }
  if (element.type == StoryboardElementType::ACTION &&
      element.state == State::RUNNING) {
    orders_.stops.push_back(static_cast<std::uint32_t>(index));
  }
  element.state = State::COMPLETE;
  record(index, StoryboardElementState::STOP_TRANSITION, time);
  if (index != ROOT) {
    propagate(element.parent, time);
  }
}

auto StoryboardPlayer::reset_element(std::size_t index) -> void {
  for (std::size_t child : elements_[index].children) {
    reset_element(child);
  }
  elements_[index].state = State::INIT;
  reset_trigger(elements_[index].start_trigger);
  reset_trigger(elements_[index].stop_trigger);
}

auto StoryboardPlayer::propagate(std::size_t index, double time) -> void {
  if (index == ROOT || index == NO_ENTITY) {
    return;
  }
  const Element& element = elements_[index];
  if (element.state == State::COMPLETE) {
    return;
  }
  if (std::ranges::all_of(element.children, [&](std::size_t child) {
        return elements_[child].state == State::COMPLETE;
      })) {
    end_element(index, time);
  }
}

//-- Global actions and traffic signals ---------------------------------------

auto StoryboardPlayer::apply(const GlobalAction& action, double time) -> void {
  if (const auto* set = std::get_if<ParameterAction>(&action)) {
    auto found =
        std::ranges::find(parameters_, set->parameter, &Parameter::name);
    if (found == parameters_.end()) {
      return;
    }
    auto now = parse_number(found->value);
    auto by = parse_number(set->value);
    if (set->kind == ParameterAction::Kind::SET || !now || !by) {
      found->value = set->value;
    } else {
      double value =
          set->kind == ParameterAction::Kind::ADD ? *now + *by : *now * *by;
      found->value = std::to_string(value);
    }
  } else if (const auto* signal =
                 std::get_if<TrafficSignalStateAction>(&action)) {
    set_signal(signal->signal, signal->state);
  } else {
    // The controller's cycle moves so that the phase starts now.
    const auto& jump = std::get<TrafficSignalControllerAction>(action);
    for (ControllerState& state : controllers_) {
      const std::vector<TrafficSignalPhase>& phases = state.controller->phases;
      auto phase =
          std::ranges::find(phases, jump.phase, &TrafficSignalPhase::name);
      if (state.controller->name != jump.controller || phase == phases.end()) {
        continue;
      }
      double before = 0.0;
      for (auto it = phases.begin(); it != phase; ++it) {
        before += it->duration;
      }
      state.offset = time - before;
      state.phase = ControllerState::NONE;
    }
    run_controllers(time);
  }
}

auto StoryboardPlayer::set_signal(const std::string& signal,
                                  const std::string& state) -> void {
  auto found = std::ranges::find(signals_, signal, &TrafficSignalState::signal);
  if (found == signals_.end()) {
    signals_.push_back(TrafficSignalState{.signal = signal, .state = state});
  } else {
    found->state = state;
  }
}

// The phase a controller is in at `time`: where `time` falls in its cycle,
// counted from its offset either way.
auto StoryboardPlayer::find_phase_index(const ControllerState& state,
                                        double time) const -> std::size_t {
  const std::vector<TrafficSignalPhase>& phases = state.controller->phases;
  double cycle = 0.0;
  for (const TrafficSignalPhase& phase : phases) {
    cycle += phase.duration;
  }
  if (cycle <= 0.0) {
    return 0;
  }
  double into = std::fmod(time - state.offset, cycle);
  if (into < 0.0) {
    into += cycle;
  }
  for (std::size_t i = 0; i < phases.size(); ++i) {
    if (into < phases[i].duration - SMALL) {
      return i;
    }
    into -= phases[i].duration;
  }
  return phases.size() - 1;
}

// Each controller entering a phase sets that phase's signal states, which
// hold until a signal action or another phase changes them.
auto StoryboardPlayer::run_controllers(double time) -> void {
  for (ControllerState& state : controllers_) {
    std::size_t phase = find_phase_index(state, time);
    if (phase == state.phase) {
      continue;
    }
    state.phase = phase;
    for (const TrafficSignalState& signal :
         state.controller->phases[phase].states) {
      set_signal(signal.signal, signal.state);
    }
  }
}

auto StoryboardPlayer::step(double time, std::span<const EntityState> entities,
                            std::span<const std::uint32_t> finished)
    -> StoryboardOrders {
  orders_ = {};
  entities_ = entities;
  run_controllers(time);
  for (std::uint32_t handle : finished) {
    if (handle < elements_.size() &&
        elements_[handle].state == State::RUNNING) {
      if (elements_[handle].parent == NO_ENTITY) {
        elements_[handle].state = State::COMPLETE;
      } else {
        end_element(handle, time);
      }
    }
  }
  evaluate_triggers(ROOT, time);
  finish_instant(time);
  return std::move(orders_);
}

// A teleport or a route ends, and a parameter set or traffic signal action
// stops, as esmini's do when they run.
auto StoryboardPlayer::finish_instant(double time) -> void {
  std::vector<std::size_t> instant = std::move(instant_);
  instant_.clear();
  for (std::size_t index : instant) {
    if (elements_[index].state != State::RUNNING) {
      continue;
    }
    if (elements_[index].private_action == nullptr) {
      stop_element(index, time);
    } else if (elements_[index].parent == NO_ENTITY) {
      elements_[index].state = State::COMPLETE;
    } else {
      end_element(index, time);
    }
  }
}

auto StoryboardPlayer::evaluate_triggers(std::size_t index, double time)
    -> void {
  Element& element = elements_[index];
  if (element.state == State::RUNNING && element.stop_trigger != nullptr &&
      fire(*element.stop_trigger, time)) {
    stop_element(index, time);
  }
  if (elements_[index].state == State::STANDBY &&
      (elements_[index].start_trigger == nullptr ||
       fire(*elements_[index].start_trigger, time))) {
    start_element(index, time);
  }
  if (elements_[index].state == State::RUNNING) {
    for (std::size_t child :
         std::vector<std::size_t>{elements_[index].children}) {
      evaluate_triggers(child, time);
    }
  }
}

//-- Triggers -----------------------------------------------------------------

auto StoryboardPlayer::fire(const Trigger& trigger, double time) -> bool {
  bool fired = false;
  for (const std::vector<Condition>& group : trigger.groups) {
    bool all = !group.empty();
    for (const Condition& condition : group) {
      all = evaluate(condition, time) && all;
    }
    fired = fired || all;
  }
  if (fired) {
    // Each condition starts over, its next value without an edge before it.
    for (const std::vector<Condition>& group : trigger.groups) {
      for (const Condition& condition : group) {
        for (auto& [which, memory] : memories_) {
          if (which == &condition) {
            memory.evaluated = false;
          }
        }
      }
    }
  }
  return fired;
}

auto StoryboardPlayer::reset_trigger(const Trigger* trigger) -> void {
  if (trigger == nullptr) {
    return;
  }
  for (const std::vector<Condition>& group : trigger->groups) {
    for (const Condition& condition : group) {
      for (auto& [which, memory] : memories_) {
        if (which == &condition) {
          memory = ConditionMemory{.seen = memory.seen};
        }
      }
    }
  }
}

// A condition's value: on its edge if it has one, needing a value before;
// and as it was `delay` seconds ago.
auto StoryboardPlayer::evaluate(const Condition& condition, double time)
    -> bool {
  auto found =
      std::ranges::find(memories_, &condition,
                        &std::pair<const Condition*, ConditionMemory>::first);
  if (found == memories_.end()) {
    memories_.emplace_back(&condition, ConditionMemory{});
    found = memories_.end() - 1;
  }
  ConditionMemory& memory = found->second;
  checking_ = &memory;
  bool raw = std::visit([&](const auto& kind) { return check(kind, time); },
                        condition.condition);
  memory.seen = log_.size();
  bool value = false;
  switch (condition.edge) {
    case Condition::Edge::NONE:
      value = raw;
      break;
    case Condition::Edge::RISING:
      value = memory.evaluated && raw && !memory.last;
      break;
    case Condition::Edge::FALLING:
      value = memory.evaluated && !raw && memory.last;
      break;
    case Condition::Edge::RISING_OR_FALLING:
      value = memory.evaluated && raw != memory.last;
      break;
  }
  memory.last = raw;
  memory.evaluated = true;
  memory.history.emplace_back(time, value);
  if (condition.delay <= 0.0) {
    return value;
  }
  bool delayed = false;
  for (const auto& [when, was] : memory.history) {
    if (when <= time - condition.delay + SMALL) {
      delayed = was;
    }
  }
  return delayed;
}

auto StoryboardPlayer::check(const ValueCondition& condition, double time)
    -> bool {
  struct Check final {
    auto operator()(const SimulationTimeCondition& c) const -> bool {
      return compare(time, c.value, c.rule);
    }
    auto operator()(const ParameterCondition& c) const -> bool {
      const std::string* value = player->find_parameter(c.parameter);
      if (value == nullptr) {
        return false;
      }
      auto a = parse_number(*value);
      auto b = parse_number(c.value);
      if (a && b) {
        return compare(*a, *b, c.rule);
      }
      bool equal = *value == c.value;
      return c.rule == Rule::EQUAL_TO       ? equal
             : c.rule == Rule::NOT_EQUAL_TO ? !equal
                                            : false;
    }
    auto operator()(const TrafficSignalCondition& c) const -> bool {
      const std::string* state = player->find_signal_state(c.signal);
      return state != nullptr && *state == c.state;
    }
    auto operator()(const TrafficSignalControllerCondition& c) const -> bool {
      const std::string* phase = player->find_phase(c.controller);
      return phase != nullptr && *phase == c.phase;
    }
    auto operator()(const StoryboardElementStateCondition& c) const -> bool {
      const auto& elements = player->elements_;
      auto found = std::ranges::find_if(elements, [&](const Element& e) {
        return e.type == c.type && e.name == c.element;
      });
      if (found == elements.end()) {
        return false;
      }
      auto index = static_cast<std::size_t>(found - elements.begin());
      switch (c.state) {
        case StoryboardElementState::STANDBY:
          return found->state == State::STANDBY;
        case StoryboardElementState::RUNNING:
          return found->state == State::RUNNING;
        case StoryboardElementState::COMPLETE:
          return found->state == State::COMPLETE;
        default:
          return std::any_of(
              player->log_.begin() +
                  static_cast<std::ptrdiff_t>(player->checking_->seen),
              player->log_.end(), [&](const auto& change) {
                return change.first == index && change.second == c.state;
              });
      }
    }
    const StoryboardPlayer* player = nullptr;
    double time = 0.0;
  };
  return std::visit(Check{.player = this, .time = time}, condition);
}

auto StoryboardPlayer::check(const EntityCondition& condition, double time)
    -> bool {
  (void)time;
  bool any = false;
  bool all = true;
  for (const std::string& name : condition.triggering) {
    std::size_t index = find_entity(name);
    if (index >= entities_.size()) {
      all = false;
      continue;
    }
    const EntityState& entity = entities_[index];
    bool holds = std::visit(
        [&](const auto& kind) -> bool {
          using Kind = std::decay_t<decltype(kind)>;
          if constexpr (std::is_same_v<Kind, SpeedCondition>) {
            return compare(entity.speed, kind.value, kind.rule);
          } else if constexpr (std::is_same_v<Kind, AccelerationCondition>) {
            return compare(entity.acceleration, kind.value, kind.rule);
          } else if constexpr (std::is_same_v<Kind, TimeHeadwayCondition>) {
            std::size_t other = find_entity(kind.entity);
            if (other >= entities_.size()) {
              return false;
            }
            double gap = compute_relative_distance(index, other, kind.distance);
            // Not defined with the other behind, or standing still.
            if (gap < 0.0 || entity.speed < SMALL) {
              return false;
            }
            return compare(std::abs(gap / entity.speed), kind.value, kind.rule);
          } else if constexpr (std::is_same_v<Kind,
                                              RelativeDistanceCondition>) {
            std::size_t other = find_entity(kind.entity);
            if (other >= entities_.size()) {
              return false;
            }
            double distance = std::abs(
                compute_relative_distance(index, other, kind.distance));
            return compare(distance, kind.value, kind.rule);
          } else if constexpr (std::is_same_v<Kind, ReachPositionCondition>) {
            RoadPlacement target = locate(kind.position, entities_);
            PlacementPose at = compute_placement_pose(*network_, target);
            return std::hypot(at.x - entity.pose.x, at.y - entity.pose.y) <
                   kind.tolerance;
          } else if constexpr (std::is_same_v<Kind, DistanceCondition>) {
            return compare(compute_distance_to(index, kind), kind.value,
                           kind.rule);
          } else if constexpr (std::is_same_v<Kind, EndOfRoadCondition>) {
            return entity.end_of_road >= kind.duration - SMALL;
          } else {
            return entity.offroad >= kind.duration - SMALL;
          }
        },
        condition.condition);
    any = any || holds;
    all = all && holds;
  }
  return condition.all ? all && !condition.triggering.empty() : any;
}

// esmini's Object::Distance to a position, either way: straight; along or
// across the entity's heading; or along or across its road, infinite on
// another road.
auto StoryboardPlayer::compute_distance_to(
    std::size_t from, const DistanceCondition& condition) const -> double {
  const EntityState& a = entities_[from];
  RoadPlacement target = locate(condition.position, entities_);
  PlacementPose b = compute_placement_pose(*network_, target);
  double dx = b.x - a.pose.x;
  double dy = b.y - a.pose.y;
  bool along_road = condition.distance.along_road;
  switch (condition.distance.kind) {
    case RelativeDistance::Kind::LONGITUDINAL:
      if (along_road) {
        return a.placement.road == target.road
                   ? std::abs(target.s - a.placement.s)
                   : std::numeric_limits<double>::infinity();
      }
      return std::abs(std::cos(a.pose.heading) * dx +
                      std::sin(a.pose.heading) * dy);
    case RelativeDistance::Kind::LATERAL: {
      if (along_road) {
        if (a.placement.road != target.road) {
          return std::numeric_limits<double>::infinity();
        }
        const Road& road = network_->roads[target.road];
        return std::abs(compute_placement_t(road, target) -
                        compute_placement_t(road, a.placement));
      }
      return std::abs(-std::sin(a.pose.heading) * dx +
                      std::cos(a.pose.heading) * dy);
    }
    default:
      return std::hypot(dx, dy);
  }
}

// esmini's Object::Distance: straight, signed by whether `to` is ahead along
// `from`'s heading; along or across `from`'s heading in its own coordinates;
// or along or across its road. Infinite on different roads.
auto StoryboardPlayer::compute_relative_distance(
    std::size_t from, std::size_t to, const RelativeDistance& distance) const
    -> double {
  const EntityState& a = entities_[from];
  const EntityState& b = entities_[to];
  double dx = b.pose.x - a.pose.x;
  double dy = b.pose.y - a.pose.y;
  double along = std::cos(a.pose.heading) * dx + std::sin(a.pose.heading) * dy;
  double across =
      -std::sin(a.pose.heading) * dx + std::cos(a.pose.heading) * dy;
  switch (distance.kind) {
    case RelativeDistance::Kind::LONGITUDINAL:
      return distance.along_road
                 ? compute_road_gap(from, to, distance.freespace)
                 : along;
    case RelativeDistance::Kind::LATERAL: {
      if (!distance.along_road) {
        return across;
      }
      if (a.placement.road != b.placement.road) {
        return std::numeric_limits<double>::infinity();
      }
      const Road& road = network_->roads[a.placement.road];
      return compute_placement_t(road, b.placement) -
             compute_placement_t(road, a.placement);
    }
    default:
      return (along > 0.0 ? 1.0 : -1.0) * std::hypot(dx, dy);
  }
}

// The gap from entity `from` to `to` along `from`'s road, positive with `to`
// ahead: between reference points, or between the nearest corners of their
// boxes, zero where they overlap along s, as esmini measures it.
auto StoryboardPlayer::compute_road_gap(std::size_t from, std::size_t to,
                                        bool freespace) const -> double {
  const EntityState& a = entities_[from];
  const EntityState& b = entities_[to];
  if (a.placement.road != b.placement.road) {
    return std::numeric_limits<double>::infinity();
  }
  double direction = runs_forward(a.placement) ? 1.0 : -1.0;
  if (!freespace) {
    return direction * (b.placement.s - a.placement.s);
  }
  const Road& road = network_->roads[a.placement.road];
  auto corners_s = [&](std::size_t entity) {
    const Vehicle& vehicle = scenario_->entities[entity].vehicle;
    const PlacementPose& pose = entities_[entity].pose;
    std::array<double, 4> s{};
    double half_length = 0.5 * vehicle.dimensions[0];
    double half_width = 0.5 * vehicle.dimensions[1];
    std::array<std::array<double, 2>, 4> corners{
        {{vehicle.center[0] + half_length, vehicle.center[1] + half_width},
         {vehicle.center[0] - half_length, vehicle.center[1] + half_width},
         {vehicle.center[0] - half_length, vehicle.center[1] - half_width},
         {vehicle.center[0] + half_length, vehicle.center[1] - half_width}}};
    for (std::size_t i = 0; i < 4; ++i) {
      double c = std::cos(pose.heading);
      double sn = std::sin(pose.heading);
      double x = pose.x + c * corners[i][0] - sn * corners[i][1];
      double y = pose.y + sn * corners[i][0] + c * corners[i][1];
      s[i] = find_road_coordinates(road, x * meter, y * meter)
                 .s.numerical_value_in(meter);
    }
    return s;
  };
  std::array<double, 4> mine = corners_s(from);
  std::array<double, 4> theirs = corners_s(to);
  double nearest = std::numeric_limits<double>::infinity();
  bool overlap = false;
  for (double s0 : mine) {
    bool ahead = false;
    bool behind = false;
    for (double s1 : theirs) {
      double ds = direction * (s1 - s0);
      if (std::abs(ds) < std::abs(nearest)) {
        nearest = ds;
      }
      (ds > 0.0 ? ahead : behind) = true;
    }
    overlap = overlap || (ahead && behind);
  }
  return overlap ? 0.0 : nearest;
}

//-- Positions ----------------------------------------------------------------

auto StoryboardPlayer::locate(const Position& position,
                              std::span<const EntityState> entities) const
    -> RoadPlacement {
  const RoadNetwork& network = *network_;
  auto road_index = [&](std::string_view id) {
    const Road* road = network.find_road(id);
    return road == nullptr
               ? std::size_t{0}
               : static_cast<std::size_t>(road - network.roads.data());
  };
  // Along the lane's travel by default, else as the orientation says: a
  // relative heading turns from the lane's travel, as esmini has it in
  // right-hand traffic.
  auto heading_of = [&](const RoadPlacement& placement,
                        const std::optional<Orientation>& orientation) {
    double travel = placement.lane > 0 ? std::numbers::pi : 0.0;
    if (!orientation) {
      return travel;
    }
    if (orientation->relative) {
      return std::remainder(travel + orientation->h, 2.0 * std::numbers::pi);
    }
    double road =
        compute_plan_point(network.roads[placement.road], placement.s * meter)
            .heading;
    return std::remainder(orientation->h - road, 2.0 * std::numbers::pi);
  };
  // The lane at (s, t), and t's offset from its middle.
  auto place_at = [&](std::size_t road, double s, double t) {
    const Road& r = network.roads[road];
    int lane = find_lane(r, s * meter, t * meter).value_or(-1);
    return RoadPlacement{.road = road,
                         .lane = lane,
                         .s = s,
                         .offset = t - compute_lane_center(r, s, lane)};
  };
  return std::visit(
      [&](const auto& p) -> RoadPlacement {
        using Kind = std::decay_t<decltype(p)>;
        if constexpr (std::is_same_v<Kind, WorldPosition>) {
          return find_placement(network, p.x, p.y, p.h)
              .value_or(RoadPlacement{});
        } else if constexpr (std::is_same_v<Kind, LanePosition>) {
          RoadPlacement placement{.road = road_index(p.road),
                                  .lane = p.lane,
                                  .s = p.s,
                                  .offset = p.offset};
          placement.heading = heading_of(placement, p.orientation);
          return placement;
        } else if constexpr (std::is_same_v<Kind, RoadPosition>) {
          RoadPlacement placement = place_at(road_index(p.road), p.s, p.t);
          placement.heading = heading_of(placement, p.orientation);
          return placement;
        } else if constexpr (std::is_same_v<Kind, RelativeRoadPosition>) {
          const RoadPlacement& reference =
              entities[find_entity(p.entity)].placement;
          const Road& road = network.roads[reference.road];
          RoadPlacement placement =
              place_at(reference.road, reference.s + p.ds,
                       compute_placement_t(road, reference) + p.dt);
          placement.heading = heading_of(placement, p.orientation);
          return placement;
        } else {
          const RoadPlacement& reference =
              entities[find_entity(p.entity)].placement;
          // dLane counts to the left of the reference's travel, skipping
          // lane 0.
          int lanes = runs_forward(reference) ? p.lanes : -p.lanes;
          int lane = reference.lane + lanes;
          if (lane == 0 || (lane > 0) != (reference.lane > 0)) {
            lane += lanes > 0 ? 1 : -1;
          }
          RoadPlacement placement{.road = reference.road,
                                  .lane = lane,
                                  .s = reference.s + p.ds,
                                  .offset = p.offset};
          placement.heading = heading_of(placement, p.orientation);
          return placement;
        }
      },
      position);
}

}  // namespace simon::scenario
