// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Watches a robotic model in real time:
//
//   bazel run //application/robotic:viewer -- [model.xml] [--scale=N]
//       [--frames=N] [--screenshot=PATH]
//
// MuJoCo's humanoid unless a model is given. Space pauses and resumes; Esc
// or Ctrl+Q quits. The view turns about its target with the left mouse
// button, pans with the right, and zooms with the wheel. See
// application/viewing.hpp for the window's options.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <string>
#include <string_view>
#include <vector>

#include "application/robotic/simulation.hpp"
#include "application/viewing.hpp"
#include "base/core.hpp"
#include "core/argument.hpp"
#include "core/math.hpp"
#include "engine/driver.hpp"
#include "imgui/imgui.h"

namespace simon::robotic {
namespace {

using namespace std::chrono_literals;
using Session = viewing::Session<Simulation, Scenario>;
using framework::Entity;
using model::GeomFrame;
using model::GeomType;

constexpr std::string_view HUMANOID = "3rd_party/mujoco/humanoid.xml";

// A perspective camera turning about a target, z up.
struct Camera final {
  Vector3 target{0.0, 0.0, 0.8};
  double yaw = -0.6;    // rad, about z.
  double pitch = 0.35;  // rad, above the horizon.
  double distance = 4.0;
  double fov = 0.9;  // rad, vertical.

  // The camera's position and axes: right, up and forward.
  auto eye() const -> Vector3 { return target + backward() * distance; }
  auto backward() const -> Vector3 {
    return {std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw),
            std::sin(pitch)};
  }
  auto right() const -> Vector3 { return {-std::sin(yaw), std::cos(yaw), 0.0}; }
  auto up() const -> Vector3 {
    Vector3 b = backward();
    Vector3 r = right();
    return {b[1] * r[2] - b[2] * r[1], b[2] * r[0] - b[0] * r[2],
            b[0] * r[1] - b[1] * r[0]};
  }
};

// Where points land on the screen, and how far ahead of the camera.
class Projection final {
 public:
  Projection(const Camera& camera, ImVec2 origin, ImVec2 size)
      : eye_{camera.eye()},
        right_{camera.right()},
        up_{camera.up()},
        forward_{camera.backward() * (-1.0)},
        center_{origin.x + size.x / 2, origin.y + size.y / 2},
        focal_{size.y / 2 / std::tan(camera.fov / 2)} {}

  auto depth(const Vector3& p) const -> double {
    return (p - eye_).dot(forward_);
  }
  auto visible(const Vector3& p) const -> bool { return depth(p) > NEAR; }
  auto at(const Vector3& p) const -> ImVec2 {
    Vector3 d = p - eye_;
    double z = std::max(d.dot(forward_), NEAR);
    return {static_cast<float>(center_.x + focal_ * d.dot(right_) / z),
            static_cast<float>(center_.y - focal_ * d.dot(up_) / z)};
  }
  auto size(double length, const Vector3& p) const -> float {
    return static_cast<float>(focal_ * length / std::max(depth(p), NEAR));
  }
  auto eye() const -> const Vector3& { return eye_; }

 private:
  static constexpr double NEAR = 0.05;  // m.

  Vector3 eye_;
  Vector3 right_;
  Vector3 up_;
  Vector3 forward_;
  ImVec2 center_;
  double focal_ = 1.0;
};

// A color for each tree, shaded by how squarely a face meets the light.
auto tree_color(std::uint32_t tree) -> std::array<float, 3> {
  constexpr std::array<std::array<float, 3>, 6> PALETTE{
      {{0.85f, 0.62f, 0.40f},
       {0.40f, 0.65f, 0.95f},
       {0.45f, 0.80f, 0.50f},
       {0.90f, 0.45f, 0.45f},
       {0.75f, 0.55f, 0.90f},
       {0.90f, 0.80f, 0.40f}}};
  return PALETTE[tree % PALETTE.size()];
}

auto shade(const std::array<float, 3>& color, double lit) -> ImU32 {
  auto f = static_cast<float>(0.35 + 0.65 * std::clamp(lit, 0.0, 1.0));
  return ImGui::ColorConvertFloat4ToU32(
      ImVec4{color[0] * f, color[1] * f, color[2] * f, 1.0f});
}

class Viewer final {
 public:
  Viewer(std::string model, float scale)
      : model_{std::move(model)}, scale_{scale} {
    restart();
  }

  auto frame() -> void {
    session_->tick();
    viewing::draw_window(
        "Robotic", 280.0f * scale_, [&] { draw_controls(); },
        [&] { draw_scene(); });
  }

  auto quitting() const -> bool { return quitting_; }

 private:
  auto restart() -> void {
    session_.reset();
    session_ = std::make_unique<Session>(
        Scenario{.model = model_}, engine::Timing{.max_step = 20ms}, speed_);
  }

  auto draw_controls() -> void {
    ImGui::SeparatorText("Model");
    ImGui::TextWrapped("%s", model_.c_str());
    if (viewing::draw_run_buttons(InOut(quitting_))) {
      restart();
      return;
    }
    viewing::draw_time_controls(InOut(*session_), InOut(speed_), 4.0f);
    const Simulation& simulation = session_->simulation();
    if (!simulation.ready()) {
      return;
    }
    ImGui::SeparatorText("Constraints");
    const ConstraintSolution& solution = simulation.constraints();
    ImGui::Text("Trees     %zu", simulation.mechanics().trees().size());
    ImGui::Text("Contacts  %zu", simulation.contacts().size());
    ImGui::Text("Islands   %u", solution.islands);
    ImGui::Text("Rows      %u", solution.rows);
    ImGui::Text("Solver    %u iterations", solution.iterations);
    ImGui::Checkbox("Show contacts", &show_contacts_);
    ImGui::SeparatorText("View");
    ImGui::Checkbox("Follow the trees", &follow_);
    ImGui::TextWrapped("Left drag turns, right drag pans, the wheel zooms.");
  }

  auto steer(ImVec2 size) -> void {
    ImGuiIO& io = ImGui::GetIO();
    if (!ImGui::IsItemHovered() && !ImGui::IsItemActive()) {
      return;
    }
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
      camera_.yaw -= io.MouseDelta.x * 0.006;
      camera_.pitch =
          std::clamp(camera_.pitch + io.MouseDelta.y * 0.006, -1.5, 1.5);
    }
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
      double pan = camera_.distance / size.y * 2 * std::tan(camera_.fov / 2);
      camera_.target = camera_.target +
                       camera_.right() * (-io.MouseDelta.x * pan) +
                       camera_.up() * (io.MouseDelta.y * pan);
    }
    if (io.MouseWheel != 0) {
      camera_.distance = std::clamp(
          camera_.distance * std::pow(0.9, io.MouseWheel), 0.3, 200.0);
    }
  }

  auto draw_scene() -> void {
    ImVec2 origin = ImGui::GetCursorScreenPos();
    ImVec2 size = ImGui::GetContentRegionAvail();
    size.x = std::max(size.x, 1.0f);
    size.y = std::max(size.y, 1.0f);
    ImGui::InvisibleButton(
        "scene", size,
        ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    steer(size);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->PushClipRect(origin, ImVec2{origin.x + size.x, origin.y + size.y},
                       true);
    draw->AddRectFilled(origin, ImVec2{origin.x + size.x, origin.y + size.y},
                        IM_COL32(26, 30, 36, 255));
    const Simulation& simulation = session_->simulation();
    if (simulation.ready()) {
      if (follow_) {
        follow(simulation);
      }
      Projection projection{camera_, origin, size};
      draw_geoms(simulation, projection, draw);
      if (show_contacts_) {
        for (const model::Contact& contact : simulation.contacts()) {
          if (projection.visible(contact.pos)) {
            draw->AddCircleFilled(projection.at(contact.pos), 3.0f * scale_,
                                  IM_COL32(240, 70, 60, 255));
          }
        }
      }
    }
    draw->PopClipRect();
  }

  // Turns the camera's target toward the trees' mean center, gently.
  auto follow(const Simulation& simulation) -> void {
    Vector3 sum = Vector3::Zero();
    std::size_t count = 0;
    simulation.world().store_of<TreeBound>().for_each(
        [&](Entity, const TreeBound& bound) {
          if (bound.radius > 0) {
            sum += bound.center;
            ++count;
          }
        });
    if (count > 0) {
      Vector3 mean = sum * (1.0 / static_cast<double>(count));
      camera_.target += (mean - camera_.target) * 0.05;
    }
  }

  // Planes as grids, then every other geom from the farthest to the
  // nearest, its faces lit from above.
  auto draw_geoms(const Simulation& simulation, const Projection& projection,
                  ImDrawList* draw) const -> void {
    const model::ArticulatedModel& m = simulation.mechanics().model();
    std::vector<GeomFrame> frames = simulation.read_geom_frames();
    std::vector<std::uint32_t> order;
    for (std::uint32_t g = 0; g < m.geoms.size(); ++g) {
      if (m.geoms[g].type == GeomType::PLANE) {
        draw_plane(frames[g], projection, draw);
      } else if (projection.visible(frames[g].pos)) {
        order.push_back(g);
      }
    }
    std::ranges::sort(order, [&](std::uint32_t a, std::uint32_t b) {
      return projection.depth(frames[a].pos) > projection.depth(frames[b].pos);
    });
    const std::vector<model::Tree>& trees = simulation.mechanics().trees();
    for (std::uint32_t g : order) {
      const model::Geom& geom = m.geoms[g];
      auto tree = static_cast<std::uint32_t>(
          std::ranges::upper_bound(trees, geom.body, {},
                                   &model::Tree::first_body) -
          trees.begin() - 1);
      draw_geom(geom, frames[g], tree_color(geom.body == 0 ? 5 : tree),
                projection, draw);
    }
  }

  auto draw_plane(const GeomFrame& frame, const Projection& projection,
                  ImDrawList* draw) const -> void {
    constexpr int LINES = 40;
    constexpr double SPACING = 0.5;  // m.
    Vector3 x = frame.mat.col(0);
    Vector3 y = frame.mat.col(1);
    // Centered under the camera's target, on the plane.
    Vector3 offset = camera_.target - frame.pos;
    double u0 = std::round(offset.dot(x) / SPACING) * SPACING;
    double v0 = std::round(offset.dot(y) / SPACING) * SPACING;
    double half = LINES / 2 * SPACING;
    for (int i = -LINES / 2; i <= LINES / 2; ++i) {
      for (int axis = 0; axis < 2; ++axis) {
        const Vector3& along = axis == 0 ? x : y;
        const Vector3& across = axis == 0 ? y : x;
        double c = (axis == 0 ? v0 : u0) + i * SPACING;
        double a = axis == 0 ? u0 : v0;
        Vector3 base = frame.pos + across * c + along * a;
        draw_line(base + along * (-half), base + along * half, projection,
                  IM_COL32(70, 80, 95, 255), 1.0f, draw);
      }
    }
  }

  // A segment, cut where it passes behind the camera.
  auto draw_line(Vector3 a, Vector3 b, const Projection& projection,
                 ImU32 color, float thickness, ImDrawList* draw) const -> void {
    constexpr double NEAR = 0.1;
    double da = projection.depth(a);
    double db = projection.depth(b);
    if (da < NEAR && db < NEAR) {
      return;
    }
    if (da < NEAR) {
      a += (b - a) * ((NEAR - da) / (db - da));
    } else if (db < NEAR) {
      b += (a - b) * ((NEAR - db) / (da - db));
    }
    draw->AddLine(projection.at(a), projection.at(b), color,
                  thickness * scale_);
  }

  auto draw_geom(const model::Geom& geom, const GeomFrame& frame,
                 const std::array<float, 3>& color,
                 const Projection& projection, ImDrawList* draw) const -> void {
    const Vector3& s = geom.size;
    Vector3 light{0.3, 0.2, 0.93};
    switch (geom.type) {
      case GeomType::SPHERE:
      case GeomType::ELLIPSOID: {
        double radius =
            geom.type == GeomType::SPHERE ? s[0] : (s[0] + s[1] + s[2]) / 3;
        float r = projection.size(radius, frame.pos);
        draw->AddCircleFilled(projection.at(frame.pos), r, shade(color, 0.8),
                              32);
        draw->AddCircle(projection.at(frame.pos), r, shade(color, 0.2), 32,
                        scale_);
        break;
      }
      case GeomType::CAPSULE: {
        Vector3 axis = frame.mat.col(2) * s[1];
        Vector3 a = frame.pos - axis;
        Vector3 b = frame.pos + axis;
        float ra = projection.size(s[0], a);
        float rb = projection.size(s[0], b);
        ImU32 fill = shade(color, 0.75);
        ImU32 edge = shade(color, 0.2);
        ImVec2 pa = projection.at(a);
        ImVec2 pb = projection.at(b);
        draw->AddLine(pa, pb, edge, ra + rb + 2 * scale_);
        draw->AddCircleFilled(pa, ra + scale_, edge, 24);
        draw->AddCircleFilled(pb, rb + scale_, edge, 24);
        draw->AddLine(pa, pb, fill, ra + rb);
        draw->AddCircleFilled(pa, ra, fill, 24);
        draw->AddCircleFilled(pb, rb, fill, 24);
        break;
      }
      case GeomType::BOX:
        draw_prism(box_corners(frame, s), 4, color, light, projection, draw);
        break;
      case GeomType::CYLINDER:
        draw_prism(cylinder_corners(frame, s), CYLINDER_SIDES, color, light,
                   projection, draw);
        break;
      default:
        break;
    }
  }

  static constexpr int CYLINDER_SIDES = 16;

  // A prism's corners: the bottom ring, then the top ring.
  static auto box_corners(const GeomFrame& frame, const Vector3& s)
      -> std::vector<Vector3> {
    std::vector<Vector3> corners;
    constexpr std::array<std::array<double, 2>, 4> RING{
        {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}};
    for (double z : {-1.0, 1.0}) {
      for (const auto& [u, v] : RING) {
        corners.push_back(frame.pos + frame.mat.col(0) * (u * s[0]) +
                          frame.mat.col(1) * (v * s[1]) +
                          frame.mat.col(2) * (z * s[2]));
      }
    }
    return corners;
  }

  static auto cylinder_corners(const GeomFrame& frame, const Vector3& s)
      -> std::vector<Vector3> {
    std::vector<Vector3> corners;
    for (double z : {-1.0, 1.0}) {
      for (int k = 0; k < CYLINDER_SIDES; ++k) {
        double angle = 2 * std::numbers::pi * k / CYLINDER_SIDES;
        corners.push_back(frame.pos +
                          frame.mat.col(0) * (s[0] * std::cos(angle)) +
                          frame.mat.col(1) * (s[0] * std::sin(angle)) +
                          frame.mat.col(2) * (z * s[1]));
      }
    }
    return corners;
  }

  // The faces of a prism of `sides` sides that face the camera.
  auto draw_prism(const std::vector<Vector3>& corners, int sides,
                  const std::array<float, 3>& color, const Vector3& light,
                  const Projection& projection, ImDrawList* draw) const
      -> void {
    Vector3 center = Vector3::Zero();
    for (const Vector3& c : corners) {
      center += c * (1.0 / static_cast<double>(corners.size()));
    }
    auto face = [&](const std::vector<Vector3>& points) {
      Vector3 middle = Vector3::Zero();
      for (const Vector3& p : points) {
        middle += p * (1.0 / static_cast<double>(points.size()));
      }
      Vector3 normal = middle - center;
      double length = std::sqrt(normal.dot(normal));
      if (length <= 0 || normal.dot(projection.eye() - middle) <= 0) {
        return;
      }
      for (const Vector3& p : points) {
        if (!projection.visible(p)) {
          return;
        }
      }
      std::vector<ImVec2> pixels;
      for (const Vector3& p : points) {
        pixels.push_back(projection.at(p));
      }
      draw->AddConvexPolyFilled(pixels.data(), static_cast<int>(pixels.size()),
                                shade(color, normal.dot(light) / length));
    };
    for (int k = 0; k < sides; ++k) {
      int next = (k + 1) % sides;
      face({corners[k], corners[next], corners[sides + next],
            corners[sides + k]});
    }
    std::vector<Vector3> bottom(corners.begin(), corners.begin() + sides);
    std::vector<Vector3> top(corners.begin() + sides, corners.end());
    face(bottom);
    face(top);
  }

  std::string model_;
  float scale_ = 1.0f;
  float speed_ = 1.0f;
  bool quitting_ = false;
  bool show_contacts_ = true;
  bool follow_ = true;
  Camera camera_;
  std::unique_ptr<Session> session_;
};

}  // namespace
}  // namespace simon::robotic

auto main(int argc, char** argv) -> int {
  using namespace simon;
  viewing::WindowOptions options{.title = "Robotic"};
  std::vector<std::string_view> arguments =
      viewing::parse_window_options(argc, argv, InOut(options));
  std::string model{arguments.empty() ? robotic::HUMANOID : arguments[0]};
  return viewing::run(
      options, [&](float scale) { return robotic::Viewer{model, scale}; });
}
