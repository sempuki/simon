// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <SDL2/SDL.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include "framework/vocabulary.hpp"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_sdl2.h"
#include "imgui/imgui_impl_sdlrenderer2.h"
#include "implot/implot.h"

// A window for an application's viewer: SDL2, Dear ImGui and ImPlot, scaled
// to the display, running the viewer's frame until the user quits.
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

}  // namespace simon::viewing
