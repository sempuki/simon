// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Watches a flight scenario in real time, every fidelity level in one world:
//
//   bazel run //application/aeronautic:viewer -- [aircraft] [precise] [rigid]
//       [fighters] [seed] [--scale=N] [--frames=N] [--screenshot=PATH]
//
// Point-mass aircraft are dots; rigid 737s and F-16s are larger markers with
// trails. The panel follows one rigid aircraft: its route on the map, its air
// data, attitude, engine and surfaces, and a strip chart of its altitude and
// speed under the map. Space pauses and resumes; Esc or Ctrl+Q quits. The map
// pans with the left mouse button and zooms with the wheel. See
// application/viewing.hpp for the window's options.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "application/aeronautic/simulation.hpp"
#include "application/viewing.hpp"
#include "base/core.hpp"
#include "core/argument.hpp"
#include "core/math.hpp"
#include "engine/driver.hpp"
#include "imgui/imgui.h"
#include "implot/implot.h"

namespace simon::aeronautic {
namespace {

using namespace std::chrono_literals;
using Session = viewing::Session<Simulation, Scenario>;
using viewing::Scatter;

constexpr ImVec4 GREY{0.60f, 0.62f, 0.65f, 0.70f};
constexpr ImVec4 TEAL{0.40f, 0.80f, 0.80f, 0.90f};
constexpr ImVec4 BLUE{0.35f, 0.65f, 1.00f, 1.00f};
constexpr ImVec4 ORANGE{1.00f, 0.60f, 0.20f, 1.00f};
constexpr ImVec4 YELLOW{1.00f, 0.90f, 0.30f, 1.00f};
constexpr double RAD_TO_DEG = 180.0 / std::numbers::pi;

// How long a rigid aircraft's trail is, and how often it gains a point.
constexpr double TRAIL_SECONDS = 120.0;
constexpr double TRAIL_EVERY = 1.0;
// How long the strip chart looks back.
constexpr double CHART_SECONDS = 120.0;

// Appends `position` to `scatter`: its east and north.
auto append(const Position& position, InOut<Scatter> scatter) -> void {
  QuantityVector meters = position.numerical_value_in(meter);
  scatter->append(meters.x(), meters.y());
}

// One rigid aircraft's recent past: where it was, for its trail, and its
// altitude and speed, for the strip chart.
struct History final {
  Scatter trail;
  std::deque<double> times;  // Simulated seconds.
  std::deque<double> altitudes;
  std::deque<double> speeds;
  double last_trail = -1e9;
};

class Viewer final {
 public:
  // Draws everything `scale` times its base size.
  Viewer(Scenario scenario, float scale) : scenario_{scenario}, scale_{scale} {
    restart();
  }

  auto frame() -> void {
    session_->tick();
    record();
    viewing::draw_window(
        "Aeronautic", 320.0f * scale_, [&] { draw_controls(); },
        [&] {
          float chart_height = 170.0f * scale_;
          float spacing = ImGui::GetStyle().ItemSpacing.y;
          draw_map(ImGui::GetContentRegionAvail().y - chart_height - spacing);
          if (!rigid_.empty()) {
            draw_charts(histories_[std::min(selected_, rigid_.size() - 1)],
                        chart_height);
          }
        });
  }

  // Whether the Quit button was pressed.
  auto quitting() const -> bool { return quitting_; }

 private:
  auto restart() -> void {
    // The wind blows from `wind_from_`, so the air moves the other way.
    double from = wind_from_ / RAD_TO_DEG;
    scenario_.wind = model::WindField{
        .north_east_down = meters_per_second(
            -wind_speed_ * std::cos(from), -wind_speed_ * std::sin(from), 0.0),
        .turbulence = static_cast<model::Turbulence>(turbulence_),
    };
    session_.reset();  // Finish the old run before starting the new one.
    session_ = std::make_unique<Session>(
        scenario_, engine::Timing{.max_step = 20ms}, speed_);
    histories_.clear();
    rigid_.clear();
    selected_ = 0;
    fit_ = true;
  }

  // Lists the rigid aircraft, once the world has them, and adds to each one's
  // history.
  auto record() -> void {
    const World& world = session_->simulation().world();
    if (rigid_.empty()) {
      world.store_of<RigidBody>().for_each(
          [&](Entity entity, const RigidBody&) { rigid_.push_back(entity); });
      histories_.resize(rigid_.size());
    }
    double time = session_->seconds();
    for (std::size_t i = 0; i < rigid_.size(); ++i) {
      const AirState* state =
          world.store_of<AirState>().maybe_component_of(rigid_[i]);
      if (!state) {
        continue;
      }
      History& history = histories_[i];
      if (!history.times.empty() && history.times.back() >= time) {
        continue;  // Paused, or no step since the last frame.
      }
      if (time - history.last_trail >= TRAIL_EVERY) {
        append(state->position, InOut(history.trail));
        history.last_trail = time;
        if (history.trail.size() > TRAIL_SECONDS / TRAIL_EVERY) {
          history.trail.x.erase(history.trail.x.begin());
          history.trail.y.erase(history.trail.y.begin());
        }
      }
      history.times.push_back(time);
      history.altitudes.push_back(
          model::altitude_of(*state).numerical_value_in(meter));
      history.speeds.push_back(
          state->speed.numerical_value_in(meter_per_second));
      while (!history.times.empty() &&
             history.times.front() < time - CHART_SECONDS) {
        history.times.pop_front();
        history.altitudes.pop_front();
        history.speeds.pop_front();
      }
    }
  }

  // Whether `entity` is an F-16, as against a 737.
  auto is_fighter(Entity entity) const -> bool {
    const World& world = session_->simulation().world();
    const AircraftType* type =
        world.store_of<AircraftType>().maybe_component_of(entity);
    return type && type->data == session_->simulation().rigid_types().fighter;
  }

  auto label(std::size_t index) const -> std::string {
    return std::string{is_fighter(rigid_[index]) ? "F-16 " : "737 "} +
           std::to_string(index + 1);
  }

  auto draw_controls() -> void {
    const Simulation& simulation = session_->simulation();
    const World& world = simulation.world();

    ImGui::SeparatorText("Scenario");
    ImGui::InputScalar("Seed", ImGuiDataType_U64, &scenario_.seed);
    ImGui::InputInt("Aircraft", &scenario_.aircraft, 100, 1000);
    ImGui::InputInt("Runge-Kutta 4", &scenario_.precise);
    ImGui::InputInt("Rigid 737s", &scenario_.rigid);
    ImGui::InputInt("Rigid F-16s", &scenario_.fighters);
    ImGui::SliderFloat("Wind from", &wind_from_, 0.0f, 360.0f, "%.0f deg");
    ImGui::SliderFloat("Wind", &wind_speed_, 0.0f, 40.0f, "%.0f m/s");
    constexpr std::array<const char*, 4> TURBULENCE{"None", "Light", "Moderate",
                                                    "Severe"};
    ImGui::Combo("Turbulence", &turbulence_, TURBULENCE.data(),
                 static_cast<int>(TURBULENCE.size()));
    if (viewing::draw_run_buttons(InOut(quitting_))) {
      restart();
      return;
    }
    viewing::draw_time_controls(InOut(*session_), InOut(speed_), 60.0f);

    ImGui::SeparatorText("Aircraft");
    ImGui::TextColored(GREY, "Single pass     %zu",
                       world.store_of<FlightControls>().size() -
                           world.store_of<AirStateRate>().size());
    ImGui::TextColored(TEAL, "Runge-Kutta 4   %zu",
                       world.store_of<AirStateRate>().size());
    std::size_t fighters = 0;
    for (Entity entity : rigid_) {
      fighters += is_fighter(entity) ? 1 : 0;
    }
    ImGui::TextColored(BLUE, "Rigid 737s      %zu", rigid_.size() - fighters);
    ImGui::TextColored(ORANGE, "Rigid F-16s     %zu", fighters);
    ImGui::Text("Waypoints reached  %llu", static_cast<unsigned long long>(
                                               simulation.waypoints_reached()));
    ImGui::Text(
        "  by rigid aircraft  %llu",
        static_cast<unsigned long long>(simulation.rigid_waypoints_reached()));

    if (rigid_.empty()) {
      return;
    }
    ImGui::SeparatorText("Following");
    selected_ = std::min(selected_, rigid_.size() - 1);
    if (ImGui::BeginCombo("##rigid", label(selected_).c_str())) {
      for (std::size_t i = 0; i < rigid_.size(); ++i) {
        if (ImGui::Selectable(label(i).c_str(), i == selected_)) {
          selected_ = i;
        }
      }
      ImGui::EndCombo();
    }
    draw_readout(rigid_[selected_]);
  }

  auto draw_readout(Entity entity) -> void {
    const World& world = session_->simulation().world();
    const AirState* state =
        world.store_of<AirState>().maybe_component_of(entity);
    const RigidBody* body =
        world.store_of<RigidBody>().maybe_component_of(entity);
    const FlightSignals* signals =
        world.store_of<FlightSignals>().maybe_component_of(entity);
    const AircraftType* type =
        world.store_of<AircraftType>().maybe_component_of(entity);
    const Engines* engines =
        world.store_of<Engines>().maybe_component_of(entity);
    const BodyAcceleration* felt =
        world.store_of<BodyAcceleration>().maybe_component_of(entity);
    const Wind* wind = world.store_of<Wind>().maybe_component_of(entity);
    if (!state || !body || !signals || !type || !type->data || !engines ||
        !felt) {
      return;
    }
    const model::AircraftData& data = *type->data;
    model::Earth earth = model::Earth::flat();
    Matrix3 attitude =
        earth.convert_body_to_north_east_down(*body, seconds(0.0s));
    Vector3 uvw = model::compute_air_velocity(*body, earth,
                                              earth.place(*body, seconds(0.0s)),
                                              wind ? *wind : Wind{})
                      .numerical_value_in(meter_per_second)
                      .eigen();
    Vector3 ground =
        body->velocity.numerical_value_in(meter_per_second).eigen();
    double g =
        model::STANDARD_GRAVITY.numerical_value_in(meter_per_second_squared);
    auto signal = [&](std::string_view name) -> std::optional<double> {
      std::optional<std::size_t> found =
          model::find_signal(data.flight_controls, name);
      if (!found) {
        return std::nullopt;
      }
      return signals->values[*found];
    };

    ImGui::Text("Altitude      %7.0f m",
                model::altitude_of(*state).numerical_value_in(meter));
    ImGui::Text("Airspeed      %7.1f m/s",
                state->speed.numerical_value_in(meter_per_second));
    ImGui::Text("Ground speed  %7.1f m/s", std::hypot(ground.x(), ground.y()));
    if (wind) {
      Vector3 air =
          wind->north_east_down.numerical_value_in(meter_per_second).eigen();
      ImGui::Text("Wind          %7.1f m/s", std::hypot(air.x(), air.y()));
      ImGui::Text("Rising air    %7.1f m/s", -air.z());
    }
    ImGui::Text("Heading       %7.1f deg",
                radians(state->heading) * RAD_TO_DEG);
    ImGui::Text("Climb angle   %7.1f deg",
                radians(state->flight_path_angle) * RAD_TO_DEG);
    ImGui::Text("Bank          %7.1f deg",
                std::atan2(attitude(2, 1), attitude(2, 2)) * RAD_TO_DEG);
    ImGui::Text("Pitch         %7.1f deg",
                -std::asin(std::clamp(attitude(2, 0), -1.0, 1.0)) * RAD_TO_DEG);
    ImGui::Text("Alpha         %7.1f deg",
                std::atan2(uvw.z(), uvw.x()) * RAD_TO_DEG);
    ImGui::Text(
        "Load factor   %7.2f g",
        -felt->specific_force.numerical_value_in(meter_per_second_squared)
                .eigen()
                .z() /
            g);
    if (std::optional<double> throttle = signal("throttle_0")) {
      ImGui::Text("Throttle      %7.2f%s", *throttle,
                  engines->turbines[0].reheat ? "  reheat" : "");
    }
    double thrust = 0.0;
    for (std::size_t i = 0; i < data.engines.size(); ++i) {
      thrust += engines->turbines[i].thrust.numerical_value_in(newton);
    }
    ImGui::Text("Thrust        %7.1f kN", thrust / 1000.0);
    for (auto [name, shown] : {std::pair{"elevator", "Elevator"},
                               std::pair{"left_aileron", "Aileron"},
                               std::pair{"aileron-pos-rad", "Aileron"},
                               std::pair{"rudder", "Rudder"},
                               std::pair{"lef-pos-rad", "LE flaps"}}) {
      if (std::optional<double> value = signal(name)) {
        ImGui::Text("%-13s %7.1f deg", shown, *value * RAD_TO_DEG);
      }
    }
  }

  // The followed aircraft's altitude and airspeed, side by side, `height`
  // tall.
  auto draw_charts(const History& history, float height) -> void {
    if (history.times.empty()) {
      return;
    }
    std::vector<double> times(history.times.begin(), history.times.end());
    std::vector<double> altitudes(history.altitudes.begin(),
                                  history.altitudes.end());
    std::vector<double> speeds(history.speeds.begin(), history.speeds.end());
    int count = static_cast<int>(times.size());
    double end = times.back();
    float width =
        (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) /
        2.0f;
    if (ImPlot::BeginPlot("##altitude", ImVec2(width, height),
                          ImPlotFlags_NoLegend | ImPlotFlags_NoMenus)) {
      ImPlot::SetupAxes("Time (s)", "Altitude (m)", 0, ImPlotAxisFlags_AutoFit);
      ImPlot::SetupAxisLimits(ImAxis_X1, end - CHART_SECONDS, end,
                              ImPlotCond_Always);
      ImPlot::SetNextLineStyle(YELLOW, scale_);
      ImPlot::PlotLine("Altitude", times.data(), altitudes.data(), count);
      ImPlot::EndPlot();
    }
    ImGui::SameLine();
    if (ImPlot::BeginPlot("##speed", ImVec2(width, height),
                          ImPlotFlags_NoLegend | ImPlotFlags_NoMenus)) {
      ImPlot::SetupAxes("Time (s)", "Airspeed (m/s)", 0,
                        ImPlotAxisFlags_AutoFit);
      ImPlot::SetupAxisLimits(ImAxis_X1, end - CHART_SECONDS, end,
                              ImPlotCond_Always);
      ImPlot::SetNextLineStyle(YELLOW, scale_);
      ImPlot::PlotLine("Airspeed", times.data(), speeds.data(), count);
      ImPlot::EndPlot();
    }
  }

  // The map, `height` tall.
  auto draw_map(float height) -> void {
    const World& world = session_->simulation().world();
    if (!ImPlot::BeginPlot("##map", ImVec2(-1.0f, height),
                           ImPlotFlags_Equal | ImPlotFlags_NoMenus)) {
      return;
    }
    ImPlot::SetupAxes("East (m)", "North (m)");
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);

    Scatter simple;
    Scatter precise;
    Scatter airliners;
    Scatter fighters;
    world.store_of<AirState>().for_each([&](Entity entity,
                                            const AirState& state) {
      if (world.store_of<RigidBody>().maybe_component_of(entity)) {
        append(state.position,
               InOut(is_fighter(entity) ? fighters : airliners));
      } else if (world.store_of<AirStateRate>().maybe_component_of(entity)) {
        append(state.position, InOut(precise));
      } else {
        append(state.position, InOut(simple));
      }
    });
    if (fit_ &&
        simple.size() + precise.size() + airliners.size() + fighters.size() >
            0) {
      ImPlot::SetupAxesLimits(-20000.0, 20000.0, -20000.0, 20000.0,
                              ImPlotCond_Always);
      auto [low_x, high_x] = std::ranges::minmax(simple.x);
      auto [low_y, high_y] = std::ranges::minmax(simple.y);
      ImPlot::SetupAxesLimits(low_x - 20000.0, high_x + 20000.0,
                              low_y - 20000.0, high_y + 20000.0,
                              ImPlotCond_Always);
      fit_ = false;
    }

    // The followed aircraft's route first, so markers draw over it.
    if (!rigid_.empty()) {
      Entity followed = rigid_[std::min(selected_, rigid_.size() - 1)];
      if (const Route* route =
              world.store_of<Route>().maybe_component_of(followed)) {
        Scatter waypoints;
        for (const Position& waypoint : route->waypoints) {
          append(waypoint, InOut(waypoints));
        }
        append(route->waypoints.front(), InOut(waypoints));
        ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.9f, 0.3f, 0.5f), scale_);
        ImPlot::PlotLine("Route", waypoints.x.data(), waypoints.y.data(),
                         waypoints.size());
        Scatter next;
        append(route->waypoints[route->next % route->waypoints.size()],
               InOut(next));
        ImPlot::SetNextMarkerStyle(ImPlotMarker_Cross, 8.0f * scale_, YELLOW,
                                   2.0f * scale_, YELLOW);
        ImPlot::PlotScatter("Next waypoint", next.x.data(), next.y.data(), 1);
      }
    }
    for (std::size_t i = 0; i < histories_.size(); ++i) {
      const History& history = histories_[i];
      ImVec4 color = is_fighter(rigid_[i]) ? ORANGE : BLUE;
      color.w = 0.6f;
      ImPlot::SetNextLineStyle(color, scale_);
      ImPlot::PlotLine("Trails", history.trail.x.data(), history.trail.y.data(),
                       history.trail.size());
    }

    auto plot = [this](std::string_view label, const Scatter& scatter,
                       ImPlotMarker marker, float size, ImVec4 color) {
      viewing::plot_scatter(label, scatter, marker, size, color, color, scale_);
    };
    plot("Single pass", simple, ImPlotMarker_Circle, 1.5f, GREY);
    plot("Runge-Kutta 4", precise, ImPlotMarker_Circle, 2.5f, TEAL);
    plot("Rigid 737s", airliners, ImPlotMarker_Up, 6.0f, BLUE);
    plot("Rigid F-16s", fighters, ImPlotMarker_Diamond, 6.0f, ORANGE);
    ImPlot::EndPlot();
  }

  Scenario scenario_;
  float wind_from_ = 270.0f;  // Degrees from north.
  float wind_speed_ = 0.0f;   // m/s.
  int turbulence_ = 0;        // A model::Turbulence.
  float speed_ = 10.0f;
  float scale_ = 1.0f;
  std::unique_ptr<Session> session_;
  std::vector<Entity> rigid_;
  std::vector<History> histories_;  // By rigid aircraft, in rigid_'s order.
  std::size_t selected_ = 0;
  bool fit_ = true;  // Fit the map to the traffic on the next frame.
  bool quitting_ = false;
};

}  // namespace
}  // namespace simon::aeronautic

auto main(int argc, char** argv) -> int {
  using namespace simon;
  viewing::WindowOptions options{.title = "Aeronautic"};
  std::vector<std::string_view> arguments =
      viewing::parse_window_options(argc, argv, InOut(options));
  auto integer = [&](std::size_t index, std::int64_t fallback) {
    return viewing::parse_integer(arguments, index, fallback);
  };
  aeronautic::Scenario scenario{
      .seed = static_cast<std::uint64_t>(integer(4, 1)),
      .aircraft = static_cast<int>(integer(0, 2000)),
      .precise = static_cast<int>(integer(1, 20)),
      .rigid = static_cast<int>(integer(2, 4)),
      .fighters = static_cast<int>(integer(3, 4)),
  };
  return viewing::run(options, [&](float scale) {
    return aeronautic::Viewer{scenario, scale};
  });
}
