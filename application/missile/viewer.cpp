// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

// Watches a missile scenario in real time:
//
//   bazel run //application/missile:viewer -- [seed]
//
// Space pauses and resumes. The map pans with the left mouse button and zooms
// with the wheel.

#include <SDL2/SDL.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <numbers>
#include <unordered_map>
#include <vector>

#include "application/missile/simulation.hpp"
#include "base/core.hpp"
#include "engine/driver.hpp"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_sdl2.h"
#include "imgui/imgui_impl_sdlrenderer2.h"
#include "implot/implot.h"

namespace simon::missile {
namespace {

using namespace std::chrono_literals;
using WallClock = std::chrono::steady_clock;
using Driver = engine::RealTimeDriver<Simulation>;

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

Point point_of(const Kinematics& kinematics) {
  model::Vector3d position =
      kinematics.position.numerical_value_in(model::meter);
  return Point{.x = position.x(), .y = position.y()};
}

// Positions of one group of markers, in the layout ImPlot wants.
struct Scatter final {
  std::vector<double> x;
  std::vector<double> y;

  void add(Point point) {
    x.push_back(point.x);
    y.push_back(point.y);
  }
  int size() const { return static_cast<int>(x.size()); }
};

// A ring drawn where a drone or interceptor disappeared. Blasts last one step,
// so the viewer never sees them; it infers them from disappearances instead.
struct Explosion final {
  Point center;
  double radius = 0.0;  // Meters, at the end of the animation.
  ImVec4 color;
  WallClock::time_point start;
};

void plot_circle(const char* label, Point center, double radius, ImVec4 color) {
  constexpr int SEGMENTS = 64;
  std::array<double, SEGMENTS + 1> x{};
  std::array<double, SEGMENTS + 1> y{};
  for (int i = 0; i <= SEGMENTS; ++i) {
    double angle = 2.0 * std::numbers::pi * i / SEGMENTS;
    x[i] = center.x + radius * std::cos(angle);
    y[i] = center.y + radius * std::sin(angle);
  }
  ImPlot::SetNextLineStyle(color, 1.0f);
  ImPlot::PlotLine(label, x.data(), y.data(), SEGMENTS + 1);
}

// One run of a scenario, paced to the wall clock.
class Session final {
 public:
  Session(Scenario scenario, double speed)
      : scenario_{scenario},
        simulation_{std::make_unique<Simulation>(scenario)},
        driver_{std::make_unique<Driver>(engine::Timing{.max_step = 10ms},
                                         speed, lib::Depend(*simulation_))} {}

  ~Session() {
    engine::Phase phase = driver_->driver().phase();
    if (phase == engine::Phase::RUNNING || phase == engine::Phase::STOPPED) {
      engine::FinishResult finished = driver_->finish();
      DECLARE_UNUSED(finished);
    }
  }

  void tick() {
    if (finished_) {
      return;
    }
    engine::PhaseResult result = driver_->tick();
    if (!result) {
      std::cerr << "Error: " << result.error().message() << "\n";
      finished_ = true;
    } else if (*result == engine::Flow::STOP) {
      finished_ = true;
    }
  }

  const Scenario& scenario() const { return scenario_; }
  const Simulation& simulation() const { return *simulation_; }
  Driver& driver() { return *driver_; }
  bool finished() const { return finished_; }

 private:
  Scenario scenario_;
  std::unique_ptr<Simulation> simulation_;
  std::unique_ptr<Driver> driver_;
  bool finished_ = false;
};

class Viewer final {
 public:
  explicit Viewer(std::uint64_t seed) : seed_{seed} { restart(); }

  void frame() {
    if (ImGui::IsKeyPressed(ImGuiKey_Space) && !ImGui::GetIO().WantTextInput) {
      toggle_pause();
    }
    session_->tick();
    notice_disappearances();

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::Begin("Missile", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);
    ImGui::BeginChild("Controls", ImVec2(280.0f, 0.0f),
                      ImGuiChildFlags_Borders);
    draw_controls();
    ImGui::EndChild();
    ImGui::SameLine();
    draw_map();
    ImGui::End();
  }

 private:
  void restart() {
    session_.reset();  // Finish the old run before starting the new one.
    session_ = std::make_unique<Session>(Scenario{.seed = seed_}, speed_);
    last_seen_.clear();
    explosions_.clear();
  }

  void toggle_pause() {
    Driver& driver = session_->driver();
    driver.paused() ? driver.resume() : driver.pause();
  }

  // Compares this frame's drones and interceptors with the last frame's, and
  // adds an explosion for each one that disappeared.
  void notice_disappearances() {
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

  void draw_controls() {
    const Simulation& simulation = session_->simulation();
    const World& world = simulation.world();
    Driver& driver = session_->driver();

    ImGui::SeparatorText("Scenario");
    ImGui::InputScalar("Seed", ImGuiDataType_U64, &seed_);
    if (ImGui::Button("Restart", ImVec2(-1.0f, 0.0f))) {
      restart();
      return;
    }

    ImGui::SeparatorText("Time");
    double seconds =
        std::chrono::duration<double>(driver.driver().now().time_since_epoch())
            .count();
    ImGui::Text("Simulated  %8.2f s", seconds);
    if (ImGui::Button(driver.paused() ? "Resume (space)" : "Pause (space)",
                      ImVec2(-1.0f, 0.0f))) {
      toggle_pause();
    }
    if (ImGui::SliderFloat("Speed", &speed_, 0.25f, 20.0f, "%.2fx",
                           ImGuiSliderFlags_Logarithmic)) {
      driver.set_speed(speed_);
    }

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

  void draw_map() {
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
              scatter.add(point_of(*kinematics));
            }
          });
      return scatter;
    };

    // Coverage first, so markers draw over it.
    world.store_of<Radar>().for_each([&](Entity owner, const Radar& radar) {
      if (const Kinematics* kinematics =
              world.store_of<Kinematics>().maybe_component_of(owner)) {
        plot_circle("Radar coverage", point_of(*kinematics),
                    radar.range.numerical_value_in(model::meter), FAINT_GREEN);
      }
    });
    world.store_of<Launcher>().for_each(
        [&](Entity owner, const Launcher& launcher) {
          if (const Kinematics* kinematics =
                  world.store_of<Kinematics>().maybe_component_of(owner)) {
            plot_circle("Launcher range", point_of(*kinematics),
                        launcher.range.numerical_value_in(model::meter),
                        FAINT_BLUE);
          }
        });

    Scatter tracks;
    world.store_of<Estimate>().for_each([&](Entity, const Estimate& estimate) {
      model::Vector3d position =
          estimate.position.numerical_value_in(model::meter);
      tracks.add(Point{.x = position.x(), .y = position.y()});
    });

    auto plot = [](const char* label, const Scatter& scatter,
                   ImPlotMarker marker, float size, ImVec4 fill,
                   ImVec4 outline) {
      ImPlot::SetNextMarkerStyle(marker, size, fill, 1.0f, outline);
      ImPlot::PlotScatter(label, scatter.x.data(), scatter.y.data(),
                          scatter.size());
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

  void draw_explosions() {
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
      draw->AddCircle(center, std::max(2.0f, edge.x - center.x),
                      ImGui::GetColorU32(color), 32, 2.0f);
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
  std::unique_ptr<Session> session_;
  std::unordered_map<Name, Sighting> last_seen_;
  std::vector<Explosion> explosions_;
};

}  // namespace
}  // namespace simon::missile

int main(int argc, char** argv) {
  using namespace simon;
  std::uint64_t seed = argc > 1 ? std::strtoull(argv[1], nullptr, 10) : 1;

  // Prefer Wayland: SDL2 defaults to X11, where this SDL build has no GPU
  // renderer (it ships GLES2 over EGL, not GLX).
  SDL_SetHint(SDL_HINT_VIDEODRIVER, "wayland,x11,windows,cocoa");
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
    std::cerr << "Error: " << SDL_GetError() << "\n";
    return EXIT_FAILURE;
  }
  SDL_Window* window = SDL_CreateWindow(
      "Missile", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 900,
      static_cast<SDL_WindowFlags>(SDL_WINDOW_RESIZABLE |
                                   SDL_WINDOW_ALLOW_HIGHDPI));
  SDL_Renderer* renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
  if (renderer == nullptr) {
    SDL_Log("No accelerated renderer (%s); falling back to software.",
            SDL_GetError());
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
  }
  if (renderer == nullptr) {
    SDL_Log("Error creating SDL_Renderer: %s", SDL_GetError());
    return EXIT_FAILURE;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGui::GetIO().IniFilename = nullptr;  // Nothing to save between runs.
  ImGui::StyleColorsDark();
  ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer2_Init(renderer);

  {
    missile::Viewer viewer{seed};
    bool done = false;
    while (!done) {
      SDL_Event event;
      while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL2_ProcessEvent(&event);
        if (event.type == SDL_QUIT ||
            (event.type == SDL_WINDOWEVENT &&
             event.window.event == SDL_WINDOWEVENT_CLOSE &&
             event.window.windowID == SDL_GetWindowID(window))) {
          done = true;
        }
      }
      ImGui_ImplSDLRenderer2_NewFrame();
      ImGui_ImplSDL2_NewFrame();
      ImGui::NewFrame();
      viewer.frame();
      ImGui::Render();
      SDL_SetRenderDrawColor(renderer, 20, 24, 28, 255);
      SDL_RenderClear(renderer);
      ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
      SDL_RenderPresent(renderer);
    }
  }

  ImGui_ImplSDLRenderer2_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return EXIT_SUCCESS;
}
