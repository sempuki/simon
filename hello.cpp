#include <SDL2/SDL.h>

#include <iostream>

#include "base/contract.hpp"
#include "base/time.hpp"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_sdl2.h"
#include "imgui/imgui_impl_sdlrenderer2.h"
#include "simulation.hpp"

using namespace simon;
using framework::Entity;
using framework::EntityName;

int main(int, char**) {
  // Setup SDL. Prefer Wayland: SDL2 defaults to X11, where this SDL build has
  // no GPU renderer (it ships GLES2 over EGL, not GLX).
  SDL_SetHint(SDL_HINT_VIDEODRIVER, "wayland,x11,windows,cocoa");
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) {
    std::cerr << "Error: " << SDL_GetError() << "\n";
    return -1;
  }

  // Setup window
  SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_RESIZABLE);
  SDL_Window* window = SDL_CreateWindow("Dear ImGui SDL2+SDL_Renderer example",
                                        SDL_WINDOWPOS_CENTERED,
                                        SDL_WINDOWPOS_CENTERED,
                                        720,
                                        720,
                                        window_flags);

  // Setup SDL_Renderer instance
  SDL_Renderer* renderer =
    SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
  if (renderer == nullptr) {
    SDL_Log("No accelerated renderer (%s); falling back to software.", SDL_GetError());
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
  }
  if (renderer == nullptr) {
    SDL_Log("Error creating SDL_Renderer: %s", SDL_GetError());
    return 1;
  }
  SDL_RendererInfo renderer_info;
  if (SDL_GetRendererInfo(renderer, &renderer_info) == 0) {
    SDL_Log("Renderer: %s (%s)", renderer_info.name, SDL_GetCurrentVideoDriver());
  }

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  (void)io;

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  // Setup Platform/Renderer backends
  ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer2_Init(renderer);
  ImVec4 clear_color = ImVec4(0.35f, 0.45f, 0.50f, 1.00f);

  // Simulator
  TimePoint curr_time;
  Simulation simulation;

  auto* ball_a = simulation.create();
  auto* ball_b = simulation.create();

  ball_a->component<component::Environment>()->wind = {-1.0, 0.0, 0.0};
  ball_a->component<component::Physical>()->wind_resistance_factor = 0.4;
  ball_a->component<component::Physical>()->radius = 10.0;
  ball_a->component<component::Controls>()->acceleration = {0.0, 9.8, 0.0};
  ball_a->component<component::Movement>()->position = {360.0, 100.0, 0.0};
  ball_a->component<component::Movement>()->velocity = {10.0, -10.0, 0.0};

  ball_b->component<component::Physical>()->radius = 10.0;
  ball_b->component<component::Movement>()->position = {360.0, 600.0, 0.0};

  auto* ball_a_movement = ball_a->component<component::Movement>();
  auto* ball_a_physical = ball_a->component<component::Physical>();
  auto* ball_b_movement = ball_b->component<component::Movement>();
  auto* ball_b_physical = ball_b->component<component::Physical>();

  bool done = false;
  simulation.events.subscribe<Collision>([&](TimePoint /*time*/, const Collision& event) {
    ASSERT((event.a == ball_a_physical || event.a == ball_b_physical) &&
           (event.b == ball_a_physical || event.b == ball_b_physical));
    done = true;
  });

  // Main loop
  while (!done) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT) done = true;
      if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE &&
          event.window.windowID == SDL_GetWindowID(window))
        done = true;
    }

    // Start simulation
    TimePoint slice_time{curr_time + Simulation::STEP_SIZE};
    for (; curr_time < slice_time; curr_time += Simulation::STEP_SIZE) {
      simulation(curr_time, Simulation::STEP_SIZE);
    }

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
    ImGui::Begin("Window",
                 nullptr,
                 ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoDecoration |
                   ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddCircleFilled(ImVec2(ball_a_movement->position[0], ball_a_movement->position[1]),
                               ball_a_physical->radius,
                               red,
                               sides);
    draw_list->AddCircleFilled(ImVec2(ball_b_movement->position[0], ball_b_movement->position[1]),
                               ball_b_physical->radius,
                               blue,
                               sides);
    ImGui::End();

    // Rendering
    ImGui::Render();
    SDL_SetRenderDrawColor(renderer,
                           (Uint8)(clear_color.x * 255),
                           (Uint8)(clear_color.y * 255),
                           (Uint8)(clear_color.z * 255),
                           (Uint8)(clear_color.w * 255));
    SDL_RenderClear(renderer);
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
    SDL_RenderPresent(renderer);
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

