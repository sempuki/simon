// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Watches a defense scenario in real time:
//
//   bazel run //application/defense:viewer -- [seed] [--scale=N]
//       [--frames=N] [--screenshot=PATH]
//
// Space pauses and resumes; Esc or Ctrl+Q quits. The map pans with the left
// mouse button and zooms with the wheel. The interface scales with the
// display, 2x on a 4K screen at 100%; --scale overrides it. See
// application/viewing.hpp for the window's options.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "application/defense/simulation.hpp"
#include "application/viewing.hpp"
#include "base/core.hpp"
#include "engine/driver.hpp"
#include "framework/vocabulary.hpp"
#include "imgui/imgui.h"
#include "implot/implot.h"

namespace simon::defense {
namespace {

using namespace std::chrono_literals;
using WallClock = std::chrono::steady_clock;
using Session = viewing::Session<Simulation, Scenario>;
using viewing::Scatter;

constexpr ImVec4 RED{0.95f, 0.30f, 0.25f, 1.0f};
constexpr ImVec4 BLUE{0.35f, 0.65f, 1.00f, 1.0f};
constexpr ImVec4 GREEN{0.40f, 0.85f, 0.45f, 1.0f};
constexpr ImVec4 GREY{0.75f, 0.75f, 0.75f, 1.0f};
constexpr ImVec4 FAINT_GREEN{0.40f, 0.85f, 0.45f, 0.25f};
constexpr ImVec4 FAINT_BLUE{0.35f, 0.65f, 1.00f, 0.20f};
constexpr float EXPLOSION_SECONDS = 0.6f;

struct Point final {
  double x = 0.0;
  double y = 0.0;
};

auto point_of(const Kinematics& kinematics) -> Point {
  model::QuantityVector position =
      kinematics.position.numerical_value_in(model::meter);
  return Point{.x = position.x(), .y = position.y()};
}

// A ring drawn where a drone or interceptor disappeared. Blasts last one step,
// so the viewer never sees them; it infers them from disappearances instead.
struct Explosion final {
  Point center;
  double radius = 0.0;  // Meters, at the end of the animation.
  ImVec4 color;
  WallClock::time_point start;
};

auto plot_circle(const char* label, Point center, double radius, ImVec4 color,
                 float weight) -> void {
  constexpr int SEGMENTS = 64;
  std::array<double, SEGMENTS + 1> x{};
  std::array<double, SEGMENTS + 1> y{};
  for (int i = 0; i <= SEGMENTS; ++i) {
    double angle = 2.0 * std::numbers::pi * i / SEGMENTS;
    x[i] = center.x + radius * std::cos(angle);
    y[i] = center.y + radius * std::sin(angle);
  }
  ImPlot::SetNextLineStyle(color, weight);
  ImPlot::PlotLine(label, x.data(), y.data(), SEGMENTS + 1);
}

class Viewer final {
 public:
  // Draws everything `scale` times its base size.
  Viewer(std::uint64_t seed, float scale) : seed_{seed}, scale_{scale} {
    restart();
  }

  auto frame() -> void {
    session_->tick();
    notice_disappearances();
    viewing::draw_window(
        "Defense", 280.0f * scale_, [&] { draw_controls(); },
        [&] { draw_map(); });
  }

  // Whether the Quit button was pressed.
  auto quitting() const -> bool { return quitting_; }

 private:
  auto restart() -> void {
    session_.reset();  // Finish the old run before starting the new one.
    session_ = std::make_unique<Session>(
        Scenario{.seed = seed_}, engine::Timing{.max_step = 10ms}, speed_);
    last_seen_.clear();
    explosions_.clear();
  }

  // Compares this frame's drones and interceptors with the last frame's, and
  // adds an explosion for each one that disappeared.
  auto notice_disappearances() -> void {
    const World& world = session_->simulation().world();
    std::unordered_map<Name, Sighting> seen;
    auto look = [&]<typename ComponentType>(double radius, ImVec4 color) {
      world.store_of<ComponentType>().for_each(
          [&](Entity entity, const ComponentType&) {
            if (const Kinematics* kinematics =
                    world.store_of<Kinematics>().maybe_component_of(entity)) {
              seen.emplace(world.name_of(entity),
                           Sighting{.point = point_of(*kinematics),
                                    .radius = radius,
                                    .color = color});
            }
          });
    };
    look.template operator()<RedDrone>(
        session_->scenario().drone_warhead.radius.numerical_value_in(
            model::meter),
        RED);
    look.template operator()<Interceptor>(
        InterceptorDesign{}.warhead.radius.numerical_value_in(model::meter),
        BLUE);

    auto now = WallClock::now();
    for (const auto& [name, sighting] : last_seen_) {
      if (!seen.contains(name)) {
        explosions_.push_back(Explosion{.center = sighting.point,
                                        .radius = 2.0 * sighting.radius,
                                        .color = sighting.color,
                                        .start = now});
      }
    }
    last_seen_ = std::move(seen);
    std::erase_if(explosions_, [&](const Explosion& explosion) {
      return std::chrono::duration<float>(now - explosion.start).count() >
             EXPLOSION_SECONDS;
    });
  }

  auto draw_controls() -> void {
    const Simulation& simulation = session_->simulation();
    const World& world = simulation.world();

    ImGui::SeparatorText("Scenario");
    ImGui::InputScalar("Seed", ImGuiDataType_U64, &seed_);
    if (viewing::draw_run_buttons(InOut(quitting_))) {
      restart();
      return;
    }
    viewing::draw_time_controls(InOut(*session_), InOut(speed_), 20.0f);

    ImGui::SeparatorText("Red");
    ImGui::TextColored(RED, "Drones remaining  %zu",
                       world.store_of<RedDrone>().size());

    ImGui::SeparatorText("Blue");
    const Health* asset =
        world.store_of<Health>().maybe_component_of(simulation.asset());
    ImGui::TextColored(GREEN, "Asset health  %.0f / %.0f",
                       asset ? asset->points : 0.0,
                       session_->scenario().asset_health);
    ImGui::TextColored(BLUE, "Tracks  %zu", world.store_of<Track>().size());
    ImGui::TextColored(BLUE, "Interceptors in flight  %zu",
                       world.store_of<Interceptor>().size());
    ImGui::TextColored(BLUE, "Interceptors fired  %u",
                       simulation.interceptors_fired());

    ImGui::SeparatorText("Outcome");
    switch (simulation.outcome()) {
      case Outcome::UNDECIDED:
        ImGui::TextUnformatted("Undecided");
        break;
      case Outcome::BLUE_WINS:
        ImGui::TextColored(BLUE, "Blue wins");
        break;
      case Outcome::RED_WINS:
        ImGui::TextColored(RED, "Red wins");
        break;
    }
  }

  auto draw_map() -> void {
    const World& world = session_->simulation().world();
    if (!ImPlot::BeginPlot("##map", ImVec2(-1.0f, -1.0f),
                           ImPlotFlags_Equal | ImPlotFlags_NoMenus)) {
      return;
    }
    ImPlot::SetupAxes("East (m)", "North (m)");
    ImPlot::SetupAxesLimits(-7000.0, 7000.0, -7000.0, 7000.0, ImPlotCond_Once);
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);

    auto positions = [&]<typename ComponentType>() {
      Scatter scatter;
      world.store_of<ComponentType>().for_each(
          [&](Entity entity, const ComponentType&) {
            if (const Kinematics* kinematics =
                    world.store_of<Kinematics>().maybe_component_of(entity)) {
              Point point = point_of(*kinematics);
              scatter.append(point.x, point.y);
            }
          });
      return scatter;
    };

    // Coverage first, so markers draw over it.
    world.store_of<Radar>().for_each([&](Entity owner, const Radar& radar) {
      if (const Kinematics* kinematics =
              world.store_of<Kinematics>().maybe_component_of(owner)) {
        plot_circle("Radar coverage", point_of(*kinematics),
                    radar.range.numerical_value_in(model::meter), FAINT_GREEN,
                    scale_);
      }
    });
    world.store_of<Launcher>().for_each(
        [&](Entity owner, const Launcher& launcher) {
          if (const Kinematics* kinematics =
                  world.store_of<Kinematics>().maybe_component_of(owner)) {
            plot_circle("Launcher range", point_of(*kinematics),
                        launcher.range.numerical_value_in(model::meter),
                        FAINT_BLUE, scale_);
          }
        });

    Scatter tracks;
    world.store_of<Estimate>().for_each([&](Entity, const Estimate& estimate) {
      model::QuantityVector position =
          estimate.position.numerical_value_in(model::meter);
      tracks.append(position.x(), position.y());
    });

    auto plot = [this](std::string_view label, const Scatter& scatter,
                       ImPlotMarker marker, float size, ImVec4 fill,
                       ImVec4 outline) {
      viewing::plot_scatter(label, scatter, marker, size, fill, outline,
                            scale_);
    };
    plot("Asset", positions.template operator()<Asset>(), ImPlotMarker_Square,
         8.0f, GREEN, GREEN);
    plot("Radars", positions.template operator()<Radar>(), ImPlotMarker_Up,
         7.0f, GREEN, GREEN);
    plot("Launchers", positions.template operator()<Launcher>(),
         ImPlotMarker_Diamond, 7.0f, BLUE, BLUE);
    plot("Tracks", tracks, ImPlotMarker_Circle, 7.0f, ImVec4(0, 0, 0, 0), GREY);
    plot("Red drones", positions.template operator()<RedDrone>(),
         ImPlotMarker_Circle, 3.5f, RED, RED);
    plot("Interceptors", positions.template operator()<Interceptor>(),
         ImPlotMarker_Circle, 2.5f, BLUE, BLUE);

    draw_explosions();
    ImPlot::EndPlot();
  }

  auto draw_explosions() -> void {
    ImPlot::PushPlotClipRect();
    ImDrawList* draw = ImPlot::GetPlotDrawList();
    auto now = WallClock::now();
    for (const Explosion& explosion : explosions_) {
      float progress =
          std::chrono::duration<float>(now - explosion.start).count() /
          EXPLOSION_SECONDS;
      ImVec2 center =
          ImPlot::PlotToPixels(explosion.center.x, explosion.center.y);
      ImVec2 edge = ImPlot::PlotToPixels(
          explosion.center.x + explosion.radius * progress, explosion.center.y);
      ImVec4 color = explosion.color;
      color.w = 1.0f - progress;
      draw->AddCircle(center, std::max(2.0f * scale_, edge.x - center.x),
                      ImGui::GetColorU32(color), 32, 2.0f * scale_);
    }
    ImPlot::PopPlotClipRect();
  }

  struct Sighting final {
    Point point;
    double radius = 0.0;
    ImVec4 color;
  };

  std::uint64_t seed_;
  float speed_ = 4.0f;
  float scale_ = 1.0f;
  std::unique_ptr<Session> session_;
  std::unordered_map<Name, Sighting> last_seen_;
  std::vector<Explosion> explosions_;
  bool quitting_ = false;
};

}  // namespace
}  // namespace simon::defense

auto main(int argc, char** argv) -> int {
  using namespace simon;
  viewing::WindowOptions options{.title = "Defense"};
  std::vector<std::string_view> arguments =
      viewing::parse_window_options(argc, argv, InOut(options));
  auto seed =
      static_cast<std::uint64_t>(viewing::parse_integer(arguments, 0, 1));
  return viewing::run(
      options, [&](float scale) { return defense::Viewer{seed, scale}; });
}
