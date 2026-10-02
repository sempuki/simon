// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include <SDL2/SDL.h>

#include <iostream>
#include <utility>

#include "application/hello/hello.hpp"
#include "engine/driver.hpp"
#include "framework/vocabulary.hpp"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_sdl2.h"
#include "imgui/imgui_impl_sdlrenderer2.h"

using namespace simon;

auto main(int, char**) -> int {
  // Setup SDL. Prefer Wayland: SDL2 defaults to X11, where this SDL build has
  // no GPU renderer (it ships GLES2 over EGL, not GLX).
  SDL_SetHint(SDL_HINT_VIDEODRIVER, "wayland,x11,windows,cocoa");
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) !=
      0) {
    std::cerr << "Error: " << SDL_GetError() << "\n";
    return -1;
  }

  // Setup window
  SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_RESIZABLE);
  SDL_Window* window = SDL_CreateWindow(
      "Dear ImGui SDL2+SDL_Renderer example", SDL_WINDOWPOS_CENTERED,
      SDL_WINDOWPOS_CENTERED, 720, 720, window_flags);

  // Setup SDL_Renderer instance
  SDL_Renderer* renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
  if (renderer == nullptr) {
    SDL_Log("No accelerated renderer (%s); falling back to software.",
            SDL_GetError());
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
  }
  if (renderer == nullptr) {
    SDL_Log("Error creating SDL_Renderer: %s", SDL_GetError());
    return 1;
  }
  SDL_RendererInfo renderer_info;
  if (SDL_GetRendererInfo(renderer, &renderer_info) == 0) {
    SDL_Log("Renderer: %s (%s)", renderer_info.name,
            SDL_GetCurrentVideoDriver());
  }

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& _ = ImGui::GetIO();

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  // Setup Platform/Renderer backends
  ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer2_Init(renderer);
  ImVec4 clear_color = ImVec4(0.35f, 0.45f, 0.50f, 1.00f);

  // Simulator: 10 ms steps, paced to the wall clock at five times real time.
  hello::Simulation simulation;
  engine::RealTimeDriver driver{
      engine::Timing{.max_step = std::chrono::milliseconds{10}}, 5.0,
      Depend(simulation)};

  bool done = false;

  // Main loop
  while (!done) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT) done = true;
      if (event.type == SDL_WINDOWEVENT &&
          event.window.event == SDL_WINDOWEVENT_CLOSE &&
          event.window.windowID == SDL_GetWindowID(window))
        done = true;
    }

    // Advance the simulation to wherever the wall clock has reached.
    engine::PhaseResult flow = driver.tick();
    if (!flow) {
      std::cerr << "Error: " << flow.error().message() << "\n";
      done = true;
    } else if (*flow == engine::Flow::STOP) {
      done = true;
    }
    const hello::World& world = simulation.world();
    const hello::Balls& balls = simulation.balls();

    // Start the Dear ImGui frame
    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    const ImU32 red = ImColor(ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
    const ImU32 blue = ImColor(ImVec4(0.0f, 0.0f, 1.0f, 1.0f));
    const ImU32 sides = 12;

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::Begin("Window", nullptr,
                 ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoDecoration |
                     ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings);
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    for (auto [entity, color] :
         {std::pair{balls.red, red}, std::pair{balls.blue, blue}}) {
      const model::Kinematics& kinematics =
          world.store_of<model::Kinematics>().component_of(entity);
      const hello::Collider& collider =
          world.store_of<hello::Collider>().component_of(entity);
      model::QuantityVector pixels =
          kinematics.position.numerical_value_in(model::meter);
      draw_list->AddCircleFilled(
          ImVec2(static_cast<float>(pixels.x()),
                 static_cast<float>(pixels.y())),
          static_cast<float>(collider.radius.numerical_value_in(model::meter)),
          color, sides);
    }
    ImGui::End();

    // Rendering
    ImGui::Render();
    SDL_SetRenderDrawColor(
        renderer, (Uint8)(clear_color.x * 255), (Uint8)(clear_color.y * 255),
        (Uint8)(clear_color.z * 255), (Uint8)(clear_color.w * 255));
    SDL_RenderClear(renderer);
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
    SDL_RenderPresent(renderer);
  }

  if (engine::FinishResult finished = driver.finish(); !finished) {
    std::cerr << "Error: " << finished.error().message() << "\n";
  }

  // Cleanup
  ImGui_ImplSDLRenderer2_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
