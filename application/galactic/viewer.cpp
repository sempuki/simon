// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Watches galaxies collide in real time:
//
//   bazel run -c opt //application/galactic:viewer -- [disk bodies]
//       [--scale=N] [--frames=N] [--screenshot=PATH]
//
// The panel picks the scenario: two disk galaxies colliding, one alone, or
// Toomre and Toomre's encounter. Each galaxy's disk has its own color; its
// halo of dark matter can be shown faintly. Space pauses and resumes; Esc or
// Ctrl+Q quits. The map pans with the left mouse button and zooms with the
// wheel. See application/viewing.hpp for the window's options.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "application/galactic/simulation.hpp"
#include "application/viewing.hpp"
#include "base/core.hpp"
#include "engine/driver.hpp"
#include "framework/vocabulary.hpp"
#include "imgui/imgui.h"
#include "implot/implot.h"
#include "model/gravity.hpp"

namespace simon::galactic {
namespace {

using Session = viewing::Session<Simulation, Scenario>;
using viewing::Scatter;

constexpr std::array<ImVec4, 2> DISK_COLORS{ImVec4{0.40f, 0.70f, 1.00f, 1.0f},
                                            ImVec4{1.00f, 0.65f, 0.30f, 1.0f}};
constexpr ImVec4 HALO{0.60f, 0.60f, 0.65f, 0.25f};
constexpr ImVec4 MASS{1.00f, 1.00f, 1.00f, 1.0f};

enum class Shown { COLLISION, DISK, ENCOUNTER };
constexpr std::array<const char*, 3> SHOWN_NAMES{
    "Two galaxies colliding", "One disk galaxy", "Toomre and Toomre"};

enum class View { FACE_ON, EDGE_ON };

class Viewer final {
 public:
  // Draws everything `scale` times its base size.
  Viewer(std::size_t disk_bodies, float scale)
      : disk_bodies_{static_cast<int>(disk_bodies)}, scale_{scale} {
    restart();
  }

  auto frame() -> void {
    session_->tick();
    viewing::draw_window(
        "Galactic", 300.0f * scale_, [&] { draw_controls(); },
        [&] { draw_map(); });
  }

  // Whether the Quit button was pressed.
  auto quitting() const -> bool { return quitting_; }

 private:
  auto make_scenario() const -> Scenario {
    auto bodies = static_cast<std::size_t>(disk_bodies_);
    switch (shown_) {
      case Shown::COLLISION:
        return make_collision_scenario(make_standard_collision(bodies));
      case Shown::DISK:
        return make_standard_disk_scenario(bodies);
      case Shown::ENCOUNTER:
        break;
    }
    Scenario scenario = make_encounter_scenario(make_toomre_encounter());
    scenario.groups = {BodyGroup{.name = "masses", .first = 0, .count = 2}};
    return scenario;
  }

  auto restart() -> void {
    session_.reset();  // Finish the old run before starting the new one.
    Year step = shown_ == Shown::ENCOUNTER ? Year{250000} : Year{1000000};
    session_ = std::make_unique<Session>(
        make_scenario(), Timing{.max_step = step}, speed_ * million_years());
  }

  static auto million_years() -> double {
    return model::MEGAYEAR.numerical_value_in(model::second);
  }

  auto draw_controls() -> void {
    ImGui::SeparatorText("Scenario");
    int shown = static_cast<int>(shown_);
    if (ImGui::Combo("##shown", &shown, SHOWN_NAMES.data(),
                     static_cast<int>(SHOWN_NAMES.size()))) {
      shown_ = static_cast<Shown>(shown);
      restart();
      return;
    }
    if (shown_ != Shown::ENCOUNTER) {
      ImGui::InputInt("Disk bodies", &disk_bodies_, 500, 2000);
      disk_bodies_ = std::clamp(disk_bodies_, 100, 100000);
      ImGui::TextDisabled("Each halo has four times as many.");
    }
    if (viewing::draw_run_buttons(InOut(quitting_))) {
      restart();
      return;
    }

    if (ImGui::IsKeyPressed(ImGuiKey_Space) && !ImGui::GetIO().WantTextInput) {
      session_->toggle_pause();
    }
    ImGui::SeparatorText("Time");
    ImGui::Text("Simulated  %8.0f million years",
                session_->seconds() / million_years());
    if (ImGui::Button(
            session_->driver().paused() ? "Resume (space)" : "Pause (space)",
            ImVec2(-1.0f, 0.0f))) {
      session_->toggle_pause();
    }
    if (ImGui::SliderFloat("Myr per s", &speed_, 1.0f, 200.0f, "%.0f",
                           ImGuiSliderFlags_Logarithmic)) {
      session_->driver().set_speed(speed_ * million_years());
    }

    ImGui::SeparatorText("View");
    int view = static_cast<int>(view_);
    ImGui::RadioButton("Face on (x-y)", &view, 0);
    ImGui::RadioButton("Edge on (x-z)", &view, 1);
    view_ = static_cast<View>(view);
    if (shown_ != Shown::ENCOUNTER) ImGui::Checkbox("Show halos", &halos_);

    ImGui::SeparatorText("Bodies");
    const Simulation& simulation = session_->simulation();
    ImGui::Text("Bodies          %zu", simulation.bodies().size());
    ImGui::Text("Test particles  %zu", simulation.test_particles().size());
  }

  auto project(const Kinematics& kinematics) const -> ImPlotPoint {
    Vector3 p = kinematics.position.numerical_value_in(model::meter).eigen() /
                model::KILOPARSEC.numerical_value_in(model::meter);
    return view_ == View::FACE_ON ? ImPlotPoint{p.x(), p.y()}
                                  : ImPlotPoint{p.x(), p.z()};
  }

  auto draw_map() -> void {
    const Simulation& simulation = session_->simulation();
    const World& world = simulation.world();
    if (!ImPlot::BeginPlot("##map", ImVec2(-1.0f, -1.0f),
                           ImPlotFlags_Equal | ImPlotFlags_NoMenus)) {
      return;
    }
    ImPlot::SetupAxes("x (kpc)",
                      view_ == View::FACE_ON ? "y (kpc)" : "z (kpc)");
    ImPlot::SetupAxesLimits(-120.0, 120.0, -120.0, 120.0, ImPlotCond_Once);
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);

    auto collect = [&](const std::vector<Entity>& entities, std::size_t first,
                       std::size_t count) {
      Scatter scatter;
      const auto& kinematics = world.store_of<Kinematics>();
      for (std::size_t i = first; i < first + count && i < entities.size();
           ++i) {
        ImPlotPoint point = project(kinematics.component_of(entities[i]));
        scatter.append(point.x, point.y);
      }
      return scatter;
    };
    auto plot = [this](std::string_view label, const Scatter& scatter,
                       float size, ImVec4 color) {
      viewing::plot_scatter(label, scatter, ImPlotMarker_Circle, size, color,
                            color, scale_);
    };

    const std::vector<BodyGroup>& groups = simulation.scenario().groups;
    std::size_t disk = 0;
    for (const BodyGroup& group : groups) {
      bool is_halo = group.name.ends_with("halo");
      if (is_halo) {
        if (halos_) {
          plot(group.name,
               collect(simulation.bodies(), group.first, group.count), 1.0f,
               HALO);
        }
      } else if (group.name == "masses") {
        plot("Masses", collect(simulation.bodies(), group.first, group.count),
             4.0f, MASS);
      } else {
        plot(group.name, collect(simulation.bodies(), group.first, group.count),
             1.2f, DISK_COLORS[disk++ % DISK_COLORS.size()]);
      }
    }
    if (!simulation.test_particles().empty()) {
      plot("Disk",
           collect(simulation.test_particles(), 0,
                   simulation.test_particles().size()),
           2.0f, DISK_COLORS[0]);
    }
    ImPlot::EndPlot();
  }

  int disk_bodies_ = 2000;
  float scale_ = 1.0f;
  float speed_ = 20.0f;  // Million years per wall second.
  Shown shown_ = Shown::COLLISION;
  View view_ = View::FACE_ON;
  std::unique_ptr<Session> session_;
  bool halos_ = false;
  bool quitting_ = false;
};

}  // namespace
}  // namespace simon::galactic

auto main(int argc, char** argv) -> int {
  using namespace simon;
  viewing::WindowOptions options{.title = "Galactic"};
  std::vector<std::string_view> arguments =
      viewing::parse_window_options(argc, argv, InOut(options));
  auto disk_bodies =
      static_cast<std::size_t>(viewing::parse_integer(arguments, 0, 2000));
  return viewing::run(options, [&](float scale) {
    return galactic::Viewer{disk_bodies, scale};
  });
}
