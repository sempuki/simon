// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Watches the balls in real time:
//
//   bazel run -c opt //application/hello:viewer -- [balls] [--scale=N]
//       [--frames=N] [--screenshot=PATH]
//
// Space pauses and resumes; Esc or Ctrl+Q quits. Each ball is colored by its
// speed. See application/viewing.hpp for the window's options.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

#include "application/arguments.hpp"
#include "application/hello/hello.hpp"
#include "application/viewing.hpp"
#include "base/core.hpp"
#include "core/argument.hpp"
#include "engine/driver.hpp"
#include "imgui/imgui.h"
#include "implot/implot.h"

namespace simon::hello {
namespace {

using Session = viewing::Session<Simulation, Scenario>;

constexpr double FASTEST_COLOR = 30.0;  // m/s, the top of the color scale.
constexpr ImVec4 WALL{0.75f, 0.75f, 0.75f, 1.0f};

class Viewer final {
 public:
  // Draws everything `scale` times its base size.
  Viewer(std::size_t balls, float scale) : scale_{scale} {
    scenario_.balls = std::min(balls, compute_capacity(scenario_));
    restart();
  }

  auto frame() -> void {
    session_->tick();
    viewing::draw_window(
        "Hello", 280.0f * scale_, [&] { draw_controls(); },
        [&] { draw_box(); });
  }

  // Whether the Quit button was pressed.
  auto quitting() const -> bool { return quitting_; }

  // When the run next needs a tick, so the window can sleep until then.
  auto next_wake() const
      -> std::optional<std::chrono::steady_clock::time_point> {
    return session_->next_wake();
  }

 private:
  auto restart() -> void {
    session_.reset();  // Finish the old run before starting the new one.
    session_ = std::make_unique<Session>(
        scenario_, engine::Timing{.max_step = STEP}, speed_);
  }

  auto draw_controls() -> void {
    const World& world = session_->simulation().world();

    ImGui::SeparatorText("Scenario");
    int balls = static_cast<int>(scenario_.balls);
    if (ImGui::InputInt("Balls", &balls, 100, 1000)) {
      scenario_.balls =
          std::clamp<std::size_t>(static_cast<std::size_t>(std::max(balls, 1)),
                                  1, compute_capacity(scenario_));
    }
    ImGui::InputScalar("Seed", ImGuiDataType_U64, &scenario_.seed);
    float restitution = static_cast<float>(scenario_.springiness.restitution);
    if (ImGui::SliderFloat("Restitution", &restitution, 0.05f, 1.0f, "%.2f")) {
      scenario_.springiness.restitution = restitution;
    }
    ImGui::TextDisabled("Changes apply on Restart.");
    if (viewing::draw_run_buttons(InOut(quitting_))) {
      restart();
      return;
    }
    viewing::draw_time_controls(InOut(*session_), InOut(speed_), 8.0f);

    ImGui::SeparatorText("Balls");
    ImGui::Text("Balls   %zu", world.store_of<Body>().size());
    ImGui::Text("Energy  %.1f MJ",
                compute_energy(world, session_->scenario().gravity)
                        .numerical_value_in(units::si::joule) /
                    1e6);

    std::vector<double> speeds;
    speeds.reserve(world.store_of<Kinematics>().size());
    world.store_of<Kinematics>().for_each([&](Entity, const Kinematics& ball) {
      speeds.push_back(
          magnitude(ball.velocity.numerical_value_in(meter_per_second)));
    });
    if (ImPlot::BeginPlot("Speeds", ImVec2(-1.0f, 200.0f * scale_),
                          ImPlotFlags_NoMenus | ImPlotFlags_NoLegend)) {
      ImPlot::SetupAxes("m/s", "balls", 0, ImPlotAxisFlags_AutoFit);
      ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, FASTEST_COLOR, ImPlotCond_Always);
      ImPlot::PlotHistogram("##speeds", speeds.data(),
                            static_cast<int>(speeds.size()), 30, 1.0,
                            ImPlotRange(0.0, FASTEST_COLOR));
      ImPlot::EndPlot();
    }
  }

  auto draw_box() -> void {
    const Simulation& simulation = session_->simulation();
    const World& world = simulation.world();
    const Box& box = simulation.scenario().box;
    double width = box.width.numerical_value_in(meter);
    double height = box.height.numerical_value_in(meter);

    if (!ImPlot::BeginPlot(
            "##box", ImVec2(-1.0f, -1.0f),
            ImPlotFlags_Equal | ImPlotFlags_NoMenus | ImPlotFlags_NoLegend)) {
      return;
    }
    ImPlot::SetupAxes("x (m)", "y (m)");
    ImPlot::SetupAxesLimits(-2.0, width + 2.0, -2.0, height + 2.0,
                            ImPlotCond_Once);

    std::vector<double> x{0.0, width, width, 0.0, 0.0};
    std::vector<double> y{0.0, 0.0, height, height, 0.0};
    ImPlot::SetNextLineStyle(WALL, 2.0f * scale_);
    ImPlot::PlotLine("Walls", x.data(), y.data(), static_cast<int>(x.size()));

    ImPlot::PushPlotClipRect();
    ImDrawList* draw = ImPlot::GetPlotDrawList();
    world.store_of<Body>().for_each([&](Entity entity, const Body& body) {
      const Kinematics& ball =
          world.store_of<Kinematics>().component_of(entity);
      QuantityVector at = ball.position.numerical_value_in(meter);
      double speed =
          magnitude(ball.velocity.numerical_value_in(meter_per_second));
      ImVec2 center = ImPlot::PlotToPixels(at.x(), at.y());
      ImVec2 edge = ImPlot::PlotToPixels(
          at.x() + body.radius.numerical_value_in(meter), at.y());
      ImVec4 color = ImPlot::SampleColormap(
          static_cast<float>(std::min(speed / FASTEST_COLOR, 1.0)),
          ImPlotColormap_Plasma);
      draw->AddCircleFilled(center, std::max(1.0f, edge.x - center.x),
                            ImGui::GetColorU32(color), 16);
    });
    ImPlot::PopPlotClipRect();
    ImPlot::EndPlot();
  }

  Scenario scenario_;
  float speed_ = 1.0f;
  float scale_ = 1.0f;
  std::unique_ptr<Session> session_;
  bool quitting_ = false;
};

}  // namespace
}  // namespace simon::hello

// How the viewer is called.
constexpr std::string_view USAGE =
    "hello viewer [balls] [--scale=N] [--frames=N] [--screenshot=PATH]";

auto main(int argc, char** argv) -> int {
  using namespace simon;
  application::Arguments arguments{argc, argv};
  viewing::WindowOptions options =
      viewing::read_window_options("Hello", InOut(arguments));
  auto balls = static_cast<std::size_t>(
      arguments.integer(0, static_cast<std::int64_t>(hello::Scenario{}.balls)));
  if (arguments.report_error(USAGE)) {
    return EXIT_FAILURE;
  }
  return viewing::run(options,
                      [&](float scale) { return hello::Viewer{balls, scale}; });
}
