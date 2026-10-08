// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Watches traffic on an OpenDRIVE network, or an OpenSCENARIO scenario, in
// real time:
//
//   bazel run //application/automotive:viewer -- [roads.xodr | scenario.xosc]
//       [vehicles] [seed] [pedestrians] [--scale=N] [--frames=N]
//       [--screenshot=PATH]
//
// The map draws each lane by its type, each crosswalk, each stop line in
// its light's color, and every vehicle as its box and pedestrian as a dot.
// Traffic is colored by speed, and pedestrians by whether they walk, wait
// or cross; in a scenario the ego is blue, the others orange, and a vehicle
// touching the ego red, and pedestrians grey. A click on a vehicle follows it,
// the map keeping it in the middle; the panel reads out its state and the
// charts under the map trace it: in traffic its speed against the mean, in a
// scenario the ego's time to collision and gap as nuPlan measures them.
// Space pauses and resumes; Esc or Ctrl+Q quits. The map pans with the left
// mouse button and zooms with the wheel. See application/viewing.hpp for the
// window's options.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "application/arguments.hpp"
#include "application/automotive/road_drawing.hpp"
#include "application/automotive/scenario_batch.hpp"
#include "application/automotive/scenario_simulation.hpp"
#include "application/automotive/simulation.hpp"
#include "application/viewing.hpp"
#include "base/core.hpp"
#include "core/argument.hpp"
#include "core/math.hpp"
#include "engine/driver.hpp"
#include "imgui/imgui.h"
#include "implot/implot.h"
#include "model/collision.hpp"

namespace simon::automotive {
namespace {

using namespace std::chrono_literals;

constexpr ImVec4 BLUE{0.35f, 0.65f, 1.00f, 1.00f};
constexpr ImVec4 ORANGE{1.00f, 0.60f, 0.20f, 1.00f};
constexpr ImVec4 RED{1.00f, 0.25f, 0.25f, 1.00f};
constexpr ImVec4 YELLOW{1.00f, 0.85f, 0.25f, 1.00f};
constexpr ImVec4 GREY{0.60f, 0.62f, 0.65f, 1.00f};
constexpr ImVec4 GREEN{0.30f, 0.85f, 0.40f, 1.00f};
constexpr ImVec4 WHITE{0.95f, 0.95f, 0.95f, 1.00f};
constexpr ImVec4 CYAN{0.30f, 0.90f, 0.95f, 1.00f};

// The most border samples the map holds: the spacing grows on large
// networks so it holds no more.
constexpr double MOST_SAMPLES = 200'000.0;
// How long the charts look back.
constexpr double CHART_SECONDS = 60.0;
// How wide a scenario's map starts, about the ego.
constexpr double VIEW_WIDTH = 200.0;  // m.
// A pedestrian's dot, across.
constexpr double PEDESTRIAN_SIZE = 0.6;  // m.
// A traffic vehicle's width, which its driver does not carry.
constexpr double TRAFFIC_WIDTH = 1.8;  // m.

auto convert_color(ImVec4 color) -> ImU32 { return ImGui::GetColorU32(color); }

// A lane's fill by its OpenDRIVE type: road surfaces darkest, the rest
// lighter, and lanes nothing drives on faint.
auto find_lane_color(std::string_view type) -> ImU32 {
  if (type == "driving" || type == "entry" || type == "exit" ||
      type == "onRamp" || type == "offRamp" || type == "connectingRamp" ||
      type == "bidirectional") {
    return IM_COL32(58, 62, 68, 255);
  }
  if (type == "shoulder" || type == "border" || type == "stop" ||
      type == "parking" || type == "biking" || type == "restricted") {
    return IM_COL32(78, 80, 84, 255);
  }
  if (type == "sidewalk" || type == "walking" || type == "curb") {
    return IM_COL32(105, 104, 98, 255);
  }
  return IM_COL32(48, 54, 50, 255);
}

// The map: the roads, drawn once, and the vehicles' boxes over them, in a
// plot that pans and zooms, and can keep a followed vehicle in the middle.
class RoadMap final {
 public:
  explicit RoadMap(float scale) : scale_{scale} {}

  auto ready() const -> bool { return ready_; }

  // Draws `network`'s roads from now on, the map fitted to them.
  auto set_roads(const road::Map& network) -> void {
    double length = 0.0;
    for (const road::Road& road : network.roads) {
      length += road.length;
    }
    spacing_ = std::clamp(length / MOST_SAMPLES, 0.5, 20.0);
    drawing_ = draw_roads(network, spacing_);
    bounds_.clear();
    low_ = {.x = std::numeric_limits<double>::infinity(),
            .y = std::numeric_limits<double>::infinity()};
    high_ = {.x = -low_.x, .y = -low_.y};
    for (const LaneStrip& strip : drawing_.lanes) {
      Bounds bounds{.low = {.x = std::numeric_limits<double>::infinity(),
                            .y = std::numeric_limits<double>::infinity()},
                    .high = {.x = -std::numeric_limits<double>::infinity(),
                             .y = -std::numeric_limits<double>::infinity()}};
      for (const auto* side : {&strip.inner, &strip.outer}) {
        for (const model::Point2& point : *side) {
          bounds.low.x = std::min(bounds.low.x, point.x);
          bounds.low.y = std::min(bounds.low.y, point.y);
          bounds.high.x = std::max(bounds.high.x, point.x);
          bounds.high.y = std::max(bounds.high.y, point.y);
        }
      }
      low_ = {.x = std::min(low_.x, bounds.low.x),
              .y = std::min(low_.y, bounds.low.y)};
      high_ = {.x = std::max(high_.x, bounds.high.x),
               .y = std::max(high_.y, bounds.high.y)};
      bounds_.push_back(bounds);
    }
    ready_ = true;
    fit_ = 2;
  }

  // The map's next view, centered on what it follows, `width` meters wide.
  auto request_width(double width) -> void { requested_width_ = width; }

  // Opens the map, `height` tall, centered on `follow` if given. Draw
  // vehicles with draw_box, then close it with end.
  auto begin(float height, std::optional<model::Point2> follow) -> bool {
    if (!ImPlot::BeginPlot("##map", ImVec2(-1.0f, height),
                           ImPlotFlags_Equal | ImPlotFlags_NoMenus |
                               ImPlotFlags_NoLegend |
                               ImPlotFlags_NoBoxSelect)) {
      return false;
    }
    ImPlot::SetupAxes("x (m)", "y (m)");
    if (fit_ > 0 && ready_) {
      // Fitted twice: the plot's size is known from the second frame.
      double margin = 0.05 * std::max(high_.x - low_.x, high_.y - low_.y);
      set_view({.x = 0.5 * (low_.x + high_.x), .y = 0.5 * (low_.y + high_.y)},
               high_.x - low_.x + 2.0 * margin,
               high_.y - low_.y + 2.0 * margin);
      --fit_;
    } else if (follow) {
      double width = requested_width_.value_or(limits_.X.Max - limits_.X.Min);
      set_view(*follow, width, 0.0);
      requested_width_.reset();
    }
    ImPlot::SetupFinish();
    limits_ = ImPlot::GetPlotLimits();
    size_ = ImPlot::GetPlotSize();
    draw_ = ImPlot::GetPlotDrawList();
    ImPlot::PushPlotClipRect();
    draw_roads_now();

    // A click that did not drag picks the point under it.
    clicked_.reset();
    if (ImPlot::IsPlotHovered() &&
        ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
        ImGui::GetIO().MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] <
            16.0f * scale_ * scale_) {
      ImPlotPoint at = ImPlot::GetPlotMousePos();
      clicked_ = model::Point2{.x = at.x, .y = at.y};
    }
    return true;
  }

  // Draws `box` filled with `fill`, outlined with `outline` if given; a dot
  // where it would be smaller than a few pixels.
  auto draw_box(const model::OrientedBox& box, ImU32 fill,
                std::optional<ImU32> outline = std::nullopt) -> void {
    std::array<model::Point2, 4> corners = model::compute_corners(box);
    std::array<ImVec2, 4> pixels{};
    for (std::size_t i = 0; i < 4; ++i) {
      pixels[i] = ImPlot::PlotToPixels(corners[i].x, corners[i].y);
    }
    float size =
        std::hypot(pixels[0].x - pixels[1].x, pixels[0].y - pixels[1].y);
    if (size < 3.0f * scale_) {
      ImVec2 center = ImPlot::PlotToPixels(box.x, box.y);
      draw_->AddCircleFilled(center, 1.5f * scale_, fill);
      return;
    }
    draw_->AddQuadFilled(pixels[0], pixels[1], pixels[2], pixels[3], fill);
    if (outline) {
      draw_->AddQuad(pixels[0], pixels[1], pixels[2], pixels[3], *outline,
                     1.5f * scale_);
    }
  }

  // Draws a line from `a` to `b`, `width` meters wide but at least a pixel.
  auto draw_line(model::Point2 a, model::Point2 b, ImU32 color, double width)
      -> void {
    float pixels =
        static_cast<float>(width * size_.x / (limits_.X.Max - limits_.X.Min));
    draw_->AddLine(ImPlot::PlotToPixels(a.x, a.y),
                   ImPlot::PlotToPixels(b.x, b.y), color,
                   std::max(pixels, scale_));
  }

  // Draws a dot at `at`, `size` meters across but at least a few pixels.
  auto draw_dot(model::Point2 at, double size, ImU32 color) -> void {
    float pixels =
        static_cast<float>(size * size_.x / (limits_.X.Max - limits_.X.Min));
    draw_->AddCircleFilled(ImPlot::PlotToPixels(at.x, at.y),
                           std::max(0.5f * pixels, 1.5f * scale_), color);
  }

  // Draws the closed polygon through `corners`, filled.
  auto draw_polygon(std::span<const model::Point2> corners, ImU32 fill)
      -> void {
    std::vector<ImVec2> pixels;
    for (const model::Point2& corner : corners) {
      pixels.push_back(ImPlot::PlotToPixels(corner.x, corner.y));
    }
    draw_->AddConvexPolyFilled(pixels.data(), static_cast<int>(pixels.size()),
                               fill);
  }

  // Where a click without a drag landed this frame, if one did.
  auto clicked() const -> std::optional<model::Point2> { return clicked_; }

  auto end() -> void {
    ImPlot::PopPlotClipRect();
    ImPlot::EndPlot();
  }

 private:
  struct Bounds final {
    model::Point2 low;
    model::Point2 high;
  };

  // Shows at least `width` by `height` meters about `center`, a meter as
  // many pixels either way.
  auto set_view(model::Point2 center, double width, double height) -> void {
    double aspect =
        size_.y > 0.0f ? static_cast<double>(size_.x) / size_.y : 1.0;
    width = std::max(width, height * aspect);
    height = width / aspect;
    ImPlot::SetupAxesLimits(center.x - 0.5 * width, center.x + 0.5 * width,
                            center.y - 0.5 * height, center.y + 0.5 * height,
                            ImPlotCond_Always);
  }

  // Each lane in view, sampled no closer than two pixels apart.
  auto draw_roads_now() -> void {
    float width = ImPlot::GetPlotSize().x;
    double pixels_per_meter = width / (limits_.X.Max - limits_.X.Min);
    auto stride = static_cast<std::size_t>(std::max(
        1.0, std::floor(2.0 * scale_ / (spacing_ * pixels_per_meter))));
    auto visible = [&](const Bounds& b) {
      return b.high.x >= limits_.X.Min && b.low.x <= limits_.X.Max &&
             b.high.y >= limits_.Y.Min && b.low.y <= limits_.Y.Max;
    };
    auto pixel = [](const model::Point2& p) {
      return ImPlot::PlotToPixels(p.x, p.y);
    };
    ImU32 edge = IM_COL32(150, 152, 156, 160);
    for (std::size_t k = 0; k < drawing_.lanes.size(); ++k) {
      if (!visible(bounds_[k])) {
        continue;
      }
      const LaneStrip& strip = drawing_.lanes[k];
      ImU32 fill = find_lane_color(strip.type);
      std::size_t last = strip.inner.size() - 1;
      for (std::size_t i = 0; i < last; i += stride) {
        std::size_t j = std::min(i + stride, last);
        draw_->AddQuadFilled(pixel(strip.inner[i]), pixel(strip.inner[j]),
                             pixel(strip.outer[j]), pixel(strip.outer[i]),
                             fill);
      }
      if (strip.type == "driving") {
        for (std::size_t i = 0; i < last; i += stride) {
          std::size_t j = std::min(i + stride, last);
          draw_->AddLine(pixel(strip.outer[i]), pixel(strip.outer[j]), edge,
                         scale_);
        }
      }
    }
    ImU32 center = convert_color(YELLOW);
    for (const std::vector<model::Point2>& line : drawing_.center_lines) {
      for (std::size_t i = 0; i + 1 < line.size(); i += stride) {
        std::size_t j = std::min(i + stride, line.size() - 1);
        draw_->AddLine(pixel(line[i]), pixel(line[j]), center, scale_);
      }
    }
  }

  float scale_ = 1.0f;
  double spacing_ = 1.0;  // m between samples along each road.
  RoadDrawing drawing_;
  std::vector<Bounds> bounds_;  // By lane strip.
  model::Point2 low_;
  model::Point2 high_;
  ImPlotRect limits_;
  ImVec2 size_;  // The plot's, in pixels, as last drawn.
  ImDrawList* draw_ = nullptr;
  std::optional<model::Point2> clicked_;
  std::optional<double> requested_width_;
  bool ready_ = false;
  int fit_ = 0;  // Frames left to fit the map to the roads.
};

// A chart of recent values over simulated time; NaN where a value is
// undefined, which leaves a gap.
struct Trace final {
  auto append(double time, double value) -> void {
    times.push_back(time);
    values.push_back(value);
    while (!times.empty() && times.front() < time - CHART_SECONDS) {
      times.pop_front();
      values.pop_front();
    }
  }
  auto clear() -> void {
    times.clear();
    values.clear();
  }

  std::deque<double> times;
  std::deque<double> values;
};

// Plots `traces`, each with its label and color, on one chart `width` wide
// and `height` tall, the time axis the last CHART_SECONDS.
auto plot_traces(
    std::string_view id, std::string_view y_label,
    std::span<const std::pair<const Trace*, std::string_view>> traces,
    std::span<const ImVec4> colors, ImVec2 size, float scale) -> void {
  double end = 0.0;
  for (const auto& [trace, label] : traces) {
    if (!trace->times.empty()) {
      end = std::max(end, trace->times.back());
    }
  }
  if (!ImPlot::BeginPlot(std::string{id}.c_str(), size, ImPlotFlags_NoMenus)) {
    return;
  }
  ImPlot::SetupAxes("Time (s)", std::string{y_label}.c_str(), 0,
                    ImPlotAxisFlags_AutoFit);
  ImPlot::SetupAxisLimits(ImAxis_X1, end - CHART_SECONDS, end,
                          ImPlotCond_Always);
  ImPlot::SetupLegend(ImPlotLocation_NorthWest);
  for (std::size_t i = 0; i < traces.size(); ++i) {
    const auto& [trace, label] = traces[i];
    std::vector<double> times(trace->times.begin(), trace->times.end());
    std::vector<double> values(trace->values.begin(), trace->values.end());
    ImPlot::SetNextLineStyle(colors[i], scale);
    ImPlot::PlotLine(std::string{label}.c_str(), times.data(), values.data(),
                     static_cast<int>(times.size()), ImPlotLineFlags_SkipNaN);
  }
  ImPlot::EndPlot();
}

// Whether the session's run has configured its world.
template <typename SessionType>
auto check_running(SessionType& session) -> bool {
  engine::Phase phase = session.driver().driver().phase();
  return phase == engine::Phase::RUNNING || phase == engine::Phase::STOPPED;
}

//-- Traffic ------------------------------------------------------------------

using TrafficSession = viewing::Session<Simulation, Scenario>;

class TrafficViewer final {
 public:
  TrafficViewer(Scenario scenario, float scale)
      : scenario_{std::move(scenario)},
        desired_{static_cast<float>(
            scenario_.following.desired_speed.numerical_value_in(
                meter_per_second))},
        scale_{scale},
        map_{scale} {
    restart();
  }

  auto frame() -> void {
    session_->tick();
    bool running = check_running(*session_);
    if (running && !map_.ready()) {
      map_.set_roads(session_->simulation().network().map);
      set_furniture(session_->simulation().network());
    }
    if (running) {
      record();
    }
    viewing::draw_window(
        "Automotive", 320.0f * scale_, [&] { draw_controls(running); },
        [&] {
          float chart_height = 170.0f * scale_;
          float spacing = ImGui::GetStyle().ItemSpacing.y;
          draw_map(ImGui::GetContentRegionAvail().y - chart_height - spacing,
                   running);
          std::pair<const Trace*, std::string_view> traces[] = {
              {&mean_, "Mean"}, {&followed_speed_, "Followed"}};
          ImVec4 colors[] = {GREY, BLUE};
          plot_traces("##speed", "Speed (m/s)", traces, colors,
                      ImVec2(-1.0f, chart_height), scale_);
        });
  }

  auto quitting() const -> bool { return quitting_; }

 private:
  auto restart() -> void {
    scenario_.following.desired_speed =
        static_cast<double>(desired_) * meter_per_second;
    session_.reset();
    session_ = std::make_unique<TrafficSession>(
        scenario_, engine::Timing{.max_step = 100ms}, speed_);
    followed_.reset();
    mean_.clear();
    followed_speed_.clear();
  }

  auto record() -> void {
    const World& world = session_->simulation().world();
    double time = session_->seconds();
    if (!mean_.times.empty() && mean_.times.back() >= time) {
      return;  // Paused, or no step since the last frame.
    }
    double sum = 0.0;
    std::size_t count = 0;
    world.store_of<LaneState>().for_each([&](Entity, const LaneState& state) {
      sum += state.speed.numerical_value_in(meter_per_second);
      ++count;
    });
    mean_.append(time, count > 0 ? sum / static_cast<double>(count) : 0.0);
    const LaneState* state =
        followed_ ? world.store_of<LaneState>().maybe_component_of(*followed_)
                  : nullptr;
    followed_speed_.append(
        time, state ? state->speed.numerical_value_in(meter_per_second)
                    : std::numeric_limits<double>::quiet_NaN());
  }

  // Each stop line across its lane, and each crosswalk's outline, which
  // never move.
  auto set_furniture(const Network& network) -> void {
    stop_lines_.clear();
    for (const traffic::StopLine& line : network.signals.stop_lines()) {
      const road::Road& road = network.map.roads[line.lane.road];
      const road::LaneSection& section = road.lane_sections[line.lane.section];
      Length s = find_s_along(network, line.lane, line.along);
      int inner = line.lane.lane > 0 ? line.lane.lane - 1 : line.lane.lane + 1;
      auto at = [&](int id) {
        Vector3 p = eigen(road::compute_position(
            road, s, road::compute_lane_border(road, section, s, id)));
        return model::Point2{.x = p.x(), .y = p.y()};
      };
      stop_lines_.push_back(
          {.from = at(line.lane.lane), .to = at(inner), .group = line.group});
    }
    crosswalks_.clear();
    for (const road::Crosswalk& crosswalk : network.walking.crosswalks()) {
      const road::Road& road = network.map.roads[crosswalk.road];
      const road::RoadObject& object = road.objects[crosswalk.object];
      if (object.outlines.empty()) {
        continue;
      }
      std::vector<model::Point2> corners;
      for (const Position& corner :
           road::compute_outline(road, object, object.outlines.front())) {
        Vector3 p = eigen(corner);
        corners.push_back({.x = p.x(), .y = p.y()});
      }
      crosswalks_.push_back(std::move(corners));
    }
  }

  // The crosswalks, the stop lines in their lights' colors, and the
  // pedestrians: crossing cyan, standing orange, walking white.
  auto draw_furniture_and_walkers(const World& world) -> void {
    ImU32 zebra = IM_COL32(235, 235, 235, 70);
    for (const std::vector<model::Point2>& corners : crosswalks_) {
      map_.draw_polygon(corners, zebra);
    }
    std::vector<traffic::Aspect> aspects(
        session_->simulation().network().signals.groups().size(),
        traffic::Aspect::GREEN);
    world.store_of<SignalState>().for_each(
        [&](Entity, const SignalState& signal) {
          if (signal.group < aspects.size()) {
            aspects[signal.group] = signal.aspect;
          }
        });
    for (const StopLineDrawing& line : stop_lines_) {
      traffic::Aspect aspect = aspects[line.group];
      ImVec4 color = aspect == traffic::Aspect::GREEN    ? GREEN
                     : aspect == traffic::Aspect::YELLOW ? YELLOW
                                                         : RED;
      map_.draw_line(line.from, line.to, convert_color(color), 0.5);
    }
    const auto& commands = world.store_of<WalkCommand>();
    world.store_of<WalkState>().for_each(
        [&](Entity entity, const WalkState& state) {
          const RoadPose* pose =
              world.store_of<RoadPose>().maybe_component_of(entity);
          if (!pose) {
            return;
          }
          const WalkCommand* command = commands.maybe_component_of(entity);
          ImVec4 color = command && command->on                  ? CYAN
                         : state.speed < 0.05 * meter_per_second ? ORANGE
                                                                 : WHITE;
          Vector3 at = eigen(pose->position);
          map_.draw_dot({.x = at.x(), .y = at.y()}, PEDESTRIAN_SIZE,
                        convert_color(color));
        });
  }

  // A vehicle's box: its pose is its front bumper.
  auto convert_to_box(const RoadPose& pose, const Driver& driver) const
      -> model::OrientedBox {
    Vector3 front = eigen(pose.position);
    double heading = radians(pose.heading);
    double length = driver.length.numerical_value_in(meter);
    return {.x = front.x() - 0.5 * length * std::cos(heading),
            .y = front.y() - 0.5 * length * std::sin(heading),
            .heading = heading,
            .length = length,
            .width = TRAFFIC_WIDTH};
  }

  auto draw_controls(bool running) -> void {
    ImGui::SeparatorText("Traffic");
    ImGui::TextWrapped("%s", scenario_.roads.c_str());
    ImGui::InputInt("Vehicles", &scenario_.vehicles, 10, 100);
    ImGui::InputInt("Pedestrians", &scenario_.pedestrians, 10, 100);
    ImGui::InputScalar("Seed", ImGuiDataType_U64, &scenario_.seed);
    ImGui::SliderFloat("Desired", &desired_, 5.0f, 40.0f, "%.0f m/s");
    if (viewing::draw_run_buttons(InOut(quitting_))) {
      restart();
      return;
    }
    viewing::draw_time_controls(InOut(*session_), InOut(speed_), 30.0f);
    if (!running) {
      return;
    }

    const World& world = session_->simulation().world();
    double slowest = std::numeric_limits<double>::infinity();
    double fastest = 0.0;
    world.store_of<LaneState>().for_each([&](Entity, const LaneState& state) {
      double v = state.speed.numerical_value_in(meter_per_second);
      slowest = std::min(slowest, v);
      fastest = std::max(fastest, v);
    });
    std::size_t crossing = 0;
    std::size_t standing = 0;
    world.store_of<WalkCommand>().for_each(
        [&](Entity, const WalkCommand& command) {
          crossing += command.on ? 1 : 0;
          standing +=
              !command.on && command.speed < 0.05 * meter_per_second ? 1 : 0;
        });
    ImGui::SeparatorText("Pedestrians");
    ImGui::Text("Walking       %7zu", world.store_of<WalkState>().size());
    ImGui::Text("Crossing      %7zu", crossing);
    ImGui::Text("Standing      %7zu", standing);
    ImGui::SeparatorText("Vehicles");
    ImGui::Text("Driving       %7zu", world.store_of<LaneState>().size());
    if (!mean_.values.empty()) {
      ImGui::Text("Mean speed    %7.1f m/s", mean_.values.back());
      ImGui::Text("Slowest       %7.1f m/s", slowest);
      ImGui::Text("Fastest       %7.1f m/s", fastest);
    }
    ImPlot::ColormapScale("##speed", 0.0, top_speed(),
                          ImVec2(-1.0f, 90.0f * scale_), "%.0f m/s", 0,
                          ImPlotColormap_Viridis);

    ImGui::SeparatorText("Following");
    const LaneState* state =
        followed_ ? world.store_of<LaneState>().maybe_component_of(*followed_)
                  : nullptr;
    if (!state) {
      ImGui::TextWrapped("Click a vehicle to follow it.");
      return;
    }
    const DriveCommand& command =
        world.store_of<DriveCommand>().component_of(*followed_);
    const Network& network = session_->simulation().network();
    ImGui::Checkbox("Keep in view", &keep_in_view_);
    ImGui::Text("Road          %7s",
                network.map.roads[state->lane.road].id.c_str());
    ImGui::Text("Lane          %7d", state->lane.lane);
    ImGui::Text("s             %7.1f m", state->s.numerical_value_in(meter));
    ImGui::Text("Speed         %7.1f m/s",
                state->speed.numerical_value_in(meter_per_second));
    ImGui::Text(
        "Acceleration  %7.2f m/s^2",
        command.acceleration.numerical_value_in(meter_per_second_squared));
    ImGui::Text("Turns taken   %7u", state->turns);
    if (ImGui::Button("Stop following", ImVec2(-1.0f, 0.0f))) {
      followed_.reset();
    }
  }

  auto top_speed() const -> double {
    return static_cast<double>(desired_) * (1.0 + scenario_.speed_spread);
  }

  auto draw_map(float height, bool running) -> void {
    const World& world = session_->simulation().world();
    std::optional<model::Point2> follow;
    if (running && followed_ && keep_in_view_) {
      if (const RoadPose* pose =
              world.store_of<RoadPose>().maybe_component_of(*followed_)) {
        Vector3 at = eigen(pose->position);
        follow = model::Point2{.x = at.x(), .y = at.y()};
      }
    }
    if (!map_.begin(height, follow)) {
      return;
    }
    if (running) {
      draw_furniture_and_walkers(world);
      std::optional<model::Point2> clicked = map_.clicked();
      double nearest = std::numeric_limits<double>::infinity();
      world.store_of<RoadPose>().for_each(
          [&](Entity entity, const RoadPose& pose) {
            const Driver* driver =
                world.store_of<Driver>().maybe_component_of(entity);
            const LaneState* state =
                world.store_of<LaneState>().maybe_component_of(entity);
            if (!driver || !state) {
              return;
            }
            model::OrientedBox box = convert_to_box(pose, *driver);
            double v = state->speed.numerical_value_in(meter_per_second);
            ImU32 fill = convert_color(ImPlot::SampleColormap(
                static_cast<float>(std::clamp(v / top_speed(), 0.0, 1.0)),
                ImPlotColormap_Viridis));
            bool followed = followed_ && *followed_ == entity;
            map_.draw_box(
                box, fill,
                followed ? std::optional{convert_color(RED)} : std::nullopt);
            if (clicked) {
              double d = std::hypot(box.x - clicked->x, box.y - clicked->y);
              if (d < nearest && d < std::max(box.length, 10.0)) {
                nearest = d;
                followed_ = entity;
                keep_in_view_ = true;
              }
            }
          });
    }
    map_.end();
  }

  // A stop line across its lane, and its light's group.
  struct StopLineDrawing final {
    model::Point2 from;
    model::Point2 to;
    std::uint32_t group = 0;
  };

  Scenario scenario_;
  std::vector<StopLineDrawing> stop_lines_;
  std::vector<std::vector<model::Point2>> crosswalks_;
  float desired_ = 20.0f;  // m/s.
  float speed_ = 1.0f;
  float scale_ = 1.0f;
  RoadMap map_;
  std::unique_ptr<TrafficSession> session_;
  std::optional<Entity> followed_;
  bool keep_in_view_ = true;
  Trace mean_;
  Trace followed_speed_;
  bool quitting_ = false;
};

//-- Scenario -----------------------------------------------------------------

using ScenarioSession = viewing::Session<ScenarioSimulation, std::string>;

class ScenarioViewer final {
 public:
  ScenarioViewer(std::string path, float scale)
      : path_{std::move(path)}, scale_{scale}, map_{scale} {
    restart();
  }

  auto frame() -> void {
    session_->tick();
    bool running = check_running(*session_);
    if (running && !map_.ready()) {
      map_.set_roads(session_->simulation().roads());
      boxes_ = convert_entities_to_boxes(session_->simulation().scenario());
      zoom_ = true;
    }
    if (running) {
      record();
    }
    viewing::draw_window(
        "Automotive", 320.0f * scale_, [&] { draw_controls(running); },
        [&] {
          float chart_height = 170.0f * scale_;
          float spacing = ImGui::GetStyle().ItemSpacing.y;
          draw_map(ImGui::GetContentRegionAvail().y - chart_height - spacing,
                   running);
          float width = (ImGui::GetContentRegionAvail().x -
                         ImGui::GetStyle().ItemSpacing.x) /
                        2.0f;
          std::pair<const Trace*, std::string_view> ttc[] = {
              {&time_to_collision_, "Ego's time to collision"}};
          std::pair<const Trace*, std::string_view> gap[] = {
              {&gap_, "Ego's gap"}};
          ImVec4 ttc_color[] = {YELLOW};
          ImVec4 gap_color[] = {BLUE};
          plot_traces("##ttc", "TTC (s)", ttc, ttc_color,
                      ImVec2(width, chart_height), scale_);
          ImGui::SameLine();
          plot_traces("##gap", "Gap (m)", gap, gap_color,
                      ImVec2(width, chart_height), scale_);
        });
  }

  auto quitting() const -> bool { return quitting_; }

 private:
  auto restart() -> void {
    session_.reset();
    session_ = std::make_unique<ScenarioSession>(
        path_, engine::Timing{.max_step = 50ms}, speed_);
    map_ = RoadMap{scale_};
    boxes_.clear();
    samples_.clear();
    time_to_collision_.clear();
    gap_.clear();
    least_gap_ = std::numeric_limits<double>::infinity();
    least_time_to_collision_.reset();
    followed_ = 0;
  }

  auto record() -> void {
    double time = session_->seconds();
    // Vehicles are placed by the first step, and none moves while paused.
    if (time <= 0.0 || (!gap_.times.empty() && gap_.times.back() >= time)) {
      return;
    }
    samples_ = sample_entities(session_->simulation());
    if (samples_.empty()) {
      return;
    }
    SampleMeasures measures = measure_sample(samples_, boxes_);
    time_to_collision_.append(time,
                              measures.time_to_collision.value_or(
                                  std::numeric_limits<double>::quiet_NaN()));
    gap_.append(time, std::isfinite(measures.gap)
                          ? measures.gap
                          : std::numeric_limits<double>::quiet_NaN());
    least_gap_ = std::min(least_gap_, measures.gap);
    if (measures.time_to_collision) {
      least_time_to_collision_ = std::min(
          *measures.time_to_collision,
          least_time_to_collision_.value_or(*measures.time_to_collision));
    }
  }

  auto convert_to_box(std::size_t entity) const -> model::OrientedBox {
    const RunSample& sample = samples_[entity];
    const RunBox& box = boxes_[entity];
    return {.x = sample.x + box.center * std::cos(sample.heading),
            .y = sample.y + box.center * std::sin(sample.heading),
            .heading = sample.heading,
            .length = box.length,
            .width = box.width};
  }

  auto draw_controls(bool running) -> void {
    ImGui::SeparatorText("Scenario");
    ImGui::TextWrapped("%s", path_.c_str());
    if (viewing::draw_run_buttons(InOut(quitting_))) {
      restart();
      return;
    }
    viewing::draw_time_controls(InOut(*session_), InOut(speed_), 8.0f);
    if (!running || samples_.empty()) {
      return;
    }
    const ScenarioSimulation& simulation = session_->simulation();
    ImGui::Text("Storyboard    %s",
                simulation.player().running() ? "running" : "stopped");

    ImGui::SeparatorText("Ego");
    if (gap_.values.empty()) {
      return;
    }
    double gap = gap_.values.back();
    double ttc = time_to_collision_.values.back();
    ImGui::Text("Gap           %7.2f m", gap);
    ImGui::Text("Least gap     %7.2f m", least_gap_);
    if (std::isnan(ttc)) {
      ImGui::Text("Time to coll.       none");
    } else {
      ImGui::Text("Time to coll. %7.1f s", ttc);
    }
    if (least_time_to_collision_) {
      ImGui::Text("Least         %7.1f s", *least_time_to_collision_);
    }

    ImGui::SeparatorText("Vehicles");
    ImGui::Checkbox("Keep in view", &keep_in_view_);
    const auto& entities = simulation.scenario().entities;
    for (std::size_t i = 0; i < entities.size(); ++i) {
      ImGui::PushStyleColor(ImGuiCol_Text, i == 0 ? BLUE : ORANGE);
      std::string label = entities[i].name + "##" + std::to_string(i);
      if (ImGui::Selectable(label.c_str(), i == followed_)) {
        followed_ = i;
      }
      ImGui::PopStyleColor();
      ImGui::SameLine(160.0f * scale_);
      ImGui::Text("%6.1f m/s", samples_[i].speed);
    }
  }

  auto draw_map(float height, bool running) -> void {
    std::optional<model::Point2> follow;
    if (running && keep_in_view_ && followed_ < samples_.size()) {
      model::OrientedBox box = convert_to_box(followed_);
      follow = model::Point2{.x = box.x, .y = box.y};
      if (zoom_) {
        map_.request_width(VIEW_WIDTH);
        zoom_ = false;
      }
    }
    if (!map_.begin(height, follow)) {
      return;
    }
    if (running && samples_.size() == boxes_.size() && !samples_.empty()) {
      std::optional<model::Point2> clicked = map_.clicked();
      model::OrientedBox ego = convert_to_box(0);
      for (std::size_t i = 0; i < samples_.size(); ++i) {
        model::OrientedBox box = convert_to_box(i);
        bool touching = i > 0 && model::compute_gap(ego, box) == 0.0;
        bool walking = session_->simulation().scenario().entities[i].kind ==
                       scenario::Entity::Kind::PEDESTRIAN;
        ImU32 fill = convert_color(touching  ? RED
                                   : i == 0  ? BLUE
                                   : walking ? GREY
                                             : ORANGE);
        map_.draw_box(
            box, fill,
            i == followed_ ? std::optional{IM_COL32_WHITE} : std::nullopt);
        if (clicked && std::hypot(box.x - clicked->x, box.y - clicked->y) <
                           std::max(box.length, 5.0)) {
          followed_ = i;
          keep_in_view_ = true;
        }
      }
    }
    map_.end();
  }

  std::string path_;
  float speed_ = 1.0f;
  float scale_ = 1.0f;
  RoadMap map_;
  std::unique_ptr<ScenarioSession> session_;
  std::vector<RunBox> boxes_;       // By entity.
  std::vector<RunSample> samples_;  // By entity, after the last step.
  std::size_t followed_ = 0;        // An entity; the ego to begin with.
  bool keep_in_view_ = true;
  bool zoom_ = false;  // Zoom to VIEW_WIDTH about the followed vehicle.
  Trace time_to_collision_;
  Trace gap_;
  double least_gap_ = std::numeric_limits<double>::infinity();
  std::optional<double> least_time_to_collision_;
  bool quitting_ = false;
};

}  // namespace
}  // namespace simon::automotive

// How the viewer is called.
constexpr std::string_view USAGE =
    "automotive viewer [roads.xodr | scenario.xosc] [vehicles] [seed] "
    "[pedestrians] [--scale=N] [--frames=N] [--screenshot=PATH]";

auto main(int argc, char** argv) -> int {
  using namespace simon;
  application::Arguments arguments{argc, argv};
  viewing::WindowOptions options =
      viewing::read_window_options("Automotive", InOut(arguments));
  std::string file{arguments.text(0, "application/automotive/roads/ring.xodr")};
  if (file.ends_with(".xosc")) {
    if (arguments.report_error(USAGE)) {
      return EXIT_FAILURE;
    }
    return viewing::run(options, [&](float scale) {
      return automotive::ScenarioViewer{file, scale};
    });
  }
  automotive::Scenario scenario{
      .seed = static_cast<std::uint64_t>(arguments.integer(2, 1)),
      .roads = file,
      .vehicles = static_cast<int>(arguments.integer(1, 40)),
      .pedestrians = static_cast<int>(arguments.integer(3, 0)),
  };
  if (arguments.report_error(USAGE)) {
    return EXIT_FAILURE;
  }
  return viewing::run(options, [&](float scale) {
    return automotive::TrafficViewer{scenario, scale};
  });
}
