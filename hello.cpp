#include <SDL2/SDL.h>

#include <iostream>

#include "backends/imgui_impl_sdl.h"
#include "backends/imgui_impl_sdlrenderer.h"
#include "base/contract.hpp"
#include "base/time.hpp"
#include "component/controls.hpp"
#include "component/environment.hpp"
#include "component/movement.hpp"
#include "component/physical.hpp"
#include "framework/component_system.hpp"
#include "framework/entity.hpp"
#include "framework/event_queue.hpp"
#include "imgui.h"

using namespace simon;
using framework::Entity;
using framework::EntityName;

struct Collision final {
  component::Physical* a = nullptr;
  component::Physical* b = nullptr;
};

bool has_collision(component::Physical* a, component::Physical* b) {
  auto distance = static_cast<Vec3>(a->movement->position - b->movement->position).norm();
  return distance <= (a->radius + b->radius);
}

struct DetectSphericalCollision : public framework::ComputeBase<component::Physical> {
  void prepare(component::Physical* current) { others.push_back(current); }
  void operator()(component::Physical* current,
                  TimePoint time,
                  Duration step,
                  framework::EventQueue* events) {
    for (auto* other : others) {
      if (current != other && has_collision(current, other)) {
        events->publish<Collision>(time, current, other);
      }
    }
  }
  void resolve(component::Physical* current) { others.clear(); }
  std::vector<component::Physical*> others;
};

auto compute_acceleration(component::Movement* m) {
  return m->controls->acceleration +
         (m->physical->wind_resistance_factor * (m->environment->wind - m->velocity));
}

struct ForwardEulerMovement : public framework::ComputeBase<component::Movement> {
  void operator()(component::Movement* movement,
                  TimePoint time,
                  Duration step,
                  framework::EventQueue* events) {
    auto prev = *movement;
    auto& next = *movement;
    auto acceleration = compute_acceleration(movement);

    next.velocity = prev.velocity + acceleration * step.count();
    next.position = prev.position + prev.velocity * step.count();
  }
};

struct TrapezoidMovement : public framework::ComputeBase<component::Movement> {
  void operator()(component::Movement* movement,
                  TimePoint time,
                  Duration step,
                  framework::EventQueue* events) {
    auto prev = *movement;
    auto& next = *movement;
    auto acceleration = compute_acceleration(movement);

    next.velocity = prev.velocity + acceleration * step.count();
    next.position = prev.position + (prev.velocity + next.velocity) * step.count() * 0.5;
  }
};

struct RungeKutta2Movement : public framework::ComputeBase<component::Movement> {
  void operator()(component::Movement* movement,
                  TimePoint time,
                  Duration step,
                  framework::EventQueue* events) {
    auto prev = *movement;
    auto& next = *movement;
    auto acceleration = compute_acceleration(movement);

    // k1.velocity = prev.velocity + acceleration * step;
    // mid.velocity = prev.velocity + k1.velocity * step * 0.5;
    // k2.velocity = mid.velocity + acceleration * step;

    next.velocity = prev.velocity + acceleration * step.count();
    next.position =
      prev.position +
      ((prev.velocity + (prev.velocity + acceleration * step.count()) * step.count() * 0.5) +
       acceleration * step.count()) *
        step.count();
  }
};

struct ComputeMovement : public RungeKutta2Movement {};

class Simulation final {
 public:
  static constexpr Duration STEP_SIZE{0.1};
  static constexpr double SUB_STEP_FACTOR{0.1};

  Entity* create() {
    entities_.emplace_back(std::make_unique<Entity>());

    auto* entity = entities_.back().get();
    auto* controls = controls_.attach(entity);
    auto* environment = environment_.attach(entity);
    auto* physical = physical_.attach(entity);
    auto* movement = movement_.attach(entity);

    movement->environment = environment;
    movement->physical = physical;
    movement->controls = controls;
    physical->movement = movement;

    return entity;
  }

  void operator()(TimePoint time, Duration step) {
    static auto do_substep =
      [](auto& system, TimePoint time, Duration step, framework::EventQueue* events) {
        Duration substep = step * SUB_STEP_FACTOR;
        for (TimePoint stop = time + step; time < stop; time += substep) {
          system(time, substep, events);
        }
      };

    events.process_until(time);
    do_substep(environment_, time, step, &events);
    do_substep(physical_, time, step, &events);
    do_substep(controls_, time, step, &events);
    do_substep(movement_, time, step, &events);
  }

  framework::EventQueue events;

 private:
  framework::ComponentSystem<component::Controls, framework::ComputeNone> controls_;
  framework::ComponentSystem<component::Environment, framework::ComputeNone> environment_;
  framework::ComponentSystem<component::Physical, DetectSphericalCollision> physical_;
  framework::ComponentSystem<component::Movement, ComputeMovement> movement_;
  std::vector<std::unique_ptr<Entity>> entities_;
};

int main(int, char**) {
  // Setup SDL
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
    SDL_Log("Error creating SDL_Renderer!");
    return 0;
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
  ImGui_ImplSDLRenderer_Init(renderer);
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
  auto* ball_b_physical = ball_a->component<component::Physical>();

  bool done = false;
  simulation.events.subscribe<Collision>([&](TimePoint time, const Collision& event) {
    ASSERT(event.a == ball_a_physical && event.b == ball_b_physical);
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
    ImGui_ImplSDLRenderer_NewFrame();
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
    ImGui_ImplSDLRenderer_RenderDrawData(ImGui::GetDrawData());
    SDL_RenderPresent(renderer);
  }

  // Cleanup
  ImGui_ImplSDLRenderer_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}

