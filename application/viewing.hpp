// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <SDL2/SDL.h>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "engine/driver.hpp"
#include "framework/vocabulary.hpp"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_sdl2.h"
#include "imgui/imgui_impl_sdlrenderer2.h"
#include "implot/implot.h"

// A window for an application's viewer: SDL2, Dear ImGui and ImPlot, scaled
// to the display, running the viewer's frame until the user quits. Also the
// pieces every viewer draws: a run of its simulation paced to the wall clock,
// a side panel with run and time controls, and scatter plots of markers.
//
// Every viewer takes these options, besides its own arguments:
//
//   --scale=N        the interface's scale, chosen from the display otherwise
//   --frames=N       quits after N frames
//   --screenshot=P   saves the last frame to P, as a BMP
//
// Esc or Ctrl+Q quits, unless a text field has the keyboard.
namespace simon::viewing {

struct WindowOptions final {
  std::string title;
  float scale = 0.0f;  // Chosen from the display unless positive.
  int frames = 0;      // Until the user quits, unless positive.
  std::string screenshot;
};

// Takes the window options out of `argv`, and returns the rest.
inline auto parse_window_options(int argc, char** argv,
                                 InOut<WindowOptions> options)
    -> std::vector<std::string_view> {
  std::vector<std::string_view> rest;
  for (int i = 1; i < argc; ++i) {
    std::string_view argument = argv[i];
    if (argument.starts_with("--scale=")) {
      options->scale = std::strtof(argv[i] + 8, nullptr);
    } else if (argument.starts_with("--frames=")) {
      options->frames = std::atoi(argv[i] + 9);
    } else if (argument.starts_with("--screenshot=")) {
      options->screenshot = std::string{argument.substr(13)};
    } else {
      rest.push_back(argument);
    }
  }
  return rest;
}

// The integer argument at `index` of `arguments`, or `fallback` if there is
// none or it is not an integer.
inline auto parse_integer(std::span<const std::string_view> arguments,
                          std::size_t index, std::int64_t fallback)
    -> std::int64_t {
  if (index >= arguments.size()) {
    return fallback;
  }
  std::string_view argument = arguments[index];
  std::int64_t value = 0;
  auto [end, error] = std::from_chars(argument.data(),
                                      argument.data() + argument.size(), value);
  return error == std::errc{} && end == argument.data() + argument.size()
             ? value
             : fallback;
}

// Computes the interface's scale on `display`: its height in screen
// coordinates over 1080, to the nearest quarter, so a 4K display at 100% gives
// 2 and a 2880x1800 one gives 1.75. Screen coordinates already include the
// compositor's scale, so a 4K display set to 200% gives 1.
inline auto ui_scale(int display) -> float {
  SDL_Rect bounds{};
  if (SDL_GetDisplayBounds(display, &bounds) != 0 || bounds.h <= 0) {
    return 1.0f;
  }
  return std::clamp(std::round(bounds.h / 1080.0f * 4.0f) / 4.0f, 1.0f, 4.0f);
}

// Enlarges ImGui's and ImPlot's fonts, spacing and lines by `scale`. Fonts are
// rasterized at the scaled size, so text stays sharp.
inline auto scale_styles(float scale) -> void {
  ImGuiStyle& style = ImGui::GetStyle();
  style.ScaleAllSizes(scale);
  style.FontScaleDpi = scale;
  ImPlotStyle& plot = ImPlot::GetStyle();
  for (float* size :
       {&plot.LineWeight, &plot.MarkerSize, &plot.MarkerWeight,
        &plot.ErrorBarSize, &plot.ErrorBarWeight, &plot.DigitalBitHeight,
        &plot.DigitalBitGap, &plot.PlotBorderSize}) {
    *size *= scale;
  }
  for (ImVec2* size :
       {&plot.MajorTickLen, &plot.MinorTickLen, &plot.MajorTickSize,
        &plot.MinorTickSize, &plot.MajorGridSize, &plot.MinorGridSize,
        &plot.PlotPadding, &plot.LabelPadding, &plot.LegendPadding,
        &plot.LegendInnerPadding, &plot.LegendSpacing, &plot.MousePosPadding,
        &plot.AnnotationPadding, &plot.PlotDefaultSize, &plot.PlotMinSize}) {
    size->x *= scale;
    size->y *= scale;
  }
}

// Saves what `renderer` drew to `path`, as a BMP.
inline auto save_screenshot(SDL_Renderer* renderer, const std::string& path)
    -> bool {
  int width = 0;
  int height = 0;
  if (SDL_GetRendererOutputSize(renderer, &width, &height) != 0) {
    return false;
  }
  SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormat(
      0, width, height, 32, SDL_PIXELFORMAT_ARGB8888);
  if (surface == nullptr) {
    return false;
  }
  bool saved = SDL_RenderReadPixels(renderer, nullptr, SDL_PIXELFORMAT_ARGB8888,
                                    surface->pixels, surface->pitch) == 0 &&
               SDL_SaveBMP(surface, path.c_str()) == 0;
  SDL_FreeSurface(surface);
  return saved;
}

// Opens a window, makes a viewer by `make(scale)`, and draws its `frame()`
// until the user quits, it asks to with `quitting()`, or `options.frames`
// run out. Returns the process's exit code.
template <typename MakeType>
auto run(const WindowOptions& options, MakeType make) -> int {
  // Prefer Wayland: SDL2 defaults to X11, where this SDL build has no GPU
  // renderer (it ships GLES2 over EGL, not GLX).
  SDL_SetHint(SDL_HINT_VIDEODRIVER, "wayland,x11,windows,cocoa");
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
    std::cerr << "Error: " << SDL_GetError() << "\n";
    return EXIT_FAILURE;
  }
  float scale = options.scale > 0.0f ? options.scale : ui_scale(0);
  // 1280 by 900 at scale 1, and never more than most of the display.
  SDL_Rect usable{.x = 0, .y = 0, .w = 1280, .h = 900};
  SDL_GetDisplayUsableBounds(0, &usable);
  int width = std::min(static_cast<int>(1280 * scale), usable.w * 9 / 10);
  int height = std::min(static_cast<int>(900 * scale), usable.h * 9 / 10);
  SDL_Window* window =
      SDL_CreateWindow(options.title.c_str(), SDL_WINDOWPOS_CENTERED,
                       SDL_WINDOWPOS_CENTERED, width, height,
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
  scale_styles(scale);
  ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer2_Init(renderer);

  int code = EXIT_SUCCESS;
  {
    auto viewer = make(scale);
    bool done = false;
    for (int frame = 1; !done; ++frame) {
      SDL_Event event;
      while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL2_ProcessEvent(&event);
        if (event.type == SDL_QUIT ||
            (event.type == SDL_WINDOWEVENT &&
             event.window.event == SDL_WINDOWEVENT_CLOSE &&
             event.window.windowID == SDL_GetWindowID(window))) {
          done = true;
        }
        if (event.type == SDL_KEYDOWN && !ImGui::GetIO().WantTextInput &&
            (event.key.keysym.sym == SDLK_ESCAPE ||
             (event.key.keysym.sym == SDLK_q &&
              (event.key.keysym.mod & KMOD_CTRL) != 0))) {
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
      done = done || viewer.quitting() ||
             (options.frames > 0 && frame >= options.frames);
      if (done && !options.screenshot.empty() &&
          !save_screenshot(renderer, options.screenshot)) {
        std::cerr << "Error saving " << options.screenshot << ": "
                  << SDL_GetError() << "\n";
        code = EXIT_FAILURE;
      }
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
  return code;
}

// One run of a scenario under `RealTimeDriver`, paced to the wall clock. It
// finishes the run when it goes, so a viewer restarts by replacing it.
template <typename SimulationType, typename ScenarioType>
class Session final {
 public:
  using DriverType = engine::RealTimeDriver<SimulationType>;

  Session(ScenarioType scenario, engine::Timing timing, double speed)
      : scenario_{std::move(scenario)},
        simulation_{std::make_unique<SimulationType>(scenario_)},
        driver_{std::make_unique<DriverType>(timing, speed,
                                             Depend(*simulation_))} {}

  ~Session() {
    engine::Phase phase = driver_->driver().phase();
    if (phase == engine::Phase::RUNNING || phase == engine::Phase::STOPPED) {
      engine::FinishResult _ = driver_->finish();
    }
  }

  Session(const Session&) = delete;
  auto operator=(const Session&) -> Session& = delete;

  // Advances the run to the wall clock, until it stops or fails.
  auto tick() -> void {
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

  auto toggle_pause() -> void {
    driver_->paused() ? driver_->resume() : driver_->pause();
  }

  // Simulated time, in seconds.
  auto seconds() const -> double {
    return std::chrono::duration<double>(
               driver_->driver().now().time_since_epoch())
        .count();
  }

  auto scenario() const -> const ScenarioType& { return scenario_; }
  auto simulation() const -> const SimulationType& { return *simulation_; }
  auto driver() -> DriverType& { return *driver_; }
  auto finished() const -> bool { return finished_; }

 private:
  ScenarioType scenario_;
  std::unique_ptr<SimulationType> simulation_;
  std::unique_ptr<DriverType> driver_;
  bool finished_ = false;
};

// Fills the window: a side panel `width` wide, drawn by `panel`, and beside
// it the rest, drawn by `main`.
template <typename PanelType, typename MainType>
auto draw_window(std::string_view title, float width, PanelType panel,
                 MainType main) -> void {
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->WorkPos);
  ImGui::SetNextWindowSize(viewport->WorkSize);
  ImGui::Begin(std::string{title}.c_str(), nullptr,
               ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoSavedSettings |
                   ImGuiWindowFlags_NoBringToFrontOnFocus);
  ImGui::BeginChild("Controls", ImVec2(width, 0.0f), ImGuiChildFlags_Borders);
  panel();
  ImGui::EndChild();
  ImGui::SameLine();
  ImGui::BeginGroup();
  main();
  ImGui::EndGroup();
  ImGui::End();
}

// Draws the Restart and Quit buttons, sets `quitting` if Quit was pressed,
// and returns whether Restart was.
inline auto draw_run_buttons(InOut<bool> quitting) -> bool {
  bool restart = ImGui::Button("Restart", ImVec2(-1.0f, 0.0f));
  if (ImGui::Button("Quit (esc)", ImVec2(-1.0f, 0.0f))) {
    *quitting = true;
  }
  return restart;
}

// Draws the Time section for `session`: its simulated time, a button that
// pauses and resumes it, as Space does, and a slider that sets `speed` up to
// `fastest` times real time.
template <typename SessionType>
auto draw_time_controls(InOut<SessionType> session, InOut<float> speed,
                        float fastest) -> void {
  if (ImGui::IsKeyPressed(ImGuiKey_Space) && !ImGui::GetIO().WantTextInput) {
    session->toggle_pause();
  }
  ImGui::SeparatorText("Time");
  ImGui::Text("Simulated  %8.1f s", session->seconds());
  if (ImGui::Button(
          session->driver().paused() ? "Resume (space)" : "Pause (space)",
          ImVec2(-1.0f, 0.0f))) {
    session->toggle_pause();
  }
  if (ImGui::SliderFloat("Speed", &*speed, 0.25f, fastest, "%.2fx",
                         ImGuiSliderFlags_Logarithmic)) {
    session->driver().set_speed(*speed);
  }
}

// Positions of one group of markers, in the layout ImPlot wants.
struct Scatter final {
  auto append(double at_x, double at_y) -> void {
    x.push_back(at_x);
    y.push_back(at_y);
  }
  auto size() const -> int { return static_cast<int>(x.size()); }

  std::vector<double> x;
  std::vector<double> y;
};

// Plots `scatter` as markers of `size`, filled with `fill` and outlined with
// `outline`, both `scale` times their base size.
inline auto plot_scatter(std::string_view label, const Scatter& scatter,
                         ImPlotMarker marker, float size, ImVec4 fill,
                         ImVec4 outline, float scale) -> void {
  ImPlot::SetNextMarkerStyle(marker, size * scale, fill, scale, outline);
  ImPlot::PlotScatter(std::string{label}.c_str(), scatter.x.data(),
                      scatter.y.data(), scatter.size());
}

}  // namespace simon::viewing
