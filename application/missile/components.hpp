// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>

#include "engine/rate_gate.hpp"
#include "framework/archetype.hpp"
#include "framework/entity.hpp"
#include "framework/name.hpp"
#include "framework/step.hpp"
#include "framework/world.hpp"
#include "model/kinematics.hpp"
#include "model/units.hpp"

// Red drones fly at a protected asset. Blue radars track them, blue launchers
// fire interceptors, and every kill is a blast applied to Health.
namespace simon::missile {

using framework::Duration;
using framework::Entity;
using framework::Name;
using framework::Step;
using framework::TimePoint;
using model::AccelerationMagnitude;
using model::Control;
using model::Kinematics;
using model::Length;
using model::Position;
using model::Rate;
using model::Speed;
using model::Velocity;

struct Health final {
  double points = 1.0;
};

// Detonates within `fuse` of its target, creating a Blast.
struct Warhead final {
  Length fuse = 0.0 * model::meter;
  Length radius = 0.0 * model::meter;
  double damage = 1.0;
};

// Damages every Health within `radius`. Lives for one step.
struct Blast final {
  Length radius = 0.0 * model::meter;
  double damage = 1.0;
  Name source;  // The warhead's entity.
};

struct RedDrone final {
  Entity target;  // The asset it flies at.
  Speed cruise = 0.0 * model::meter_per_second;
  AccelerationMagnitude agility = 0.0 * model::meter_per_second_squared;
};

// Marks a red drone that has a track, so radars do not track it twice.
struct Tracked final {
  Entity track;
};

struct Asset final {};

struct Radar final {
  Length range = 0.0 * model::meter;
  engine::RateGate scan;
  bool scanned = false;  // Whether it scanned this step.
};

// What blue believes about a red drone.
struct Track final {
  Entity target;
  Position position = model::meters(0.0, 0.0, 0.0);
  Velocity velocity = model::meters_per_second(0.0, 0.0, 0.0);
  TimePoint last_seen{};
  Entity engaged_by;          // The launcher engaging it, if any.
  TimePoint engaged_until{};  // When the engagement lapses if it has not ended.
};

struct Launcher final {
  Length range = 0.0 * model::meter;
  std::uint32_t inventory = 0;
  Duration reload{};
  TimePoint ready_at{};
  Entity proposal;  // The track it proposes to engage this step.
};

struct Interceptor final {
  Entity target;  // The red drone it homes on.
  double navigation_gain = 4.0;
  Speed speed = 0.0 * model::meter_per_second;
  AccelerationMagnitude agility = 0.0 * model::meter_per_second_squared;
  Length seeker_range = 0.0 * model::meter;
  TimePoint expires_at{};
};

using World =
    framework::World<Kinematics, Control, Health, Warhead, Blast, RedDrone,
                     Tracked, Asset, Radar, Track, Launcher, Interceptor>;

namespace archetype {

using framework::Allows;
using framework::Archetype;
using framework::Requires;

struct Asset final
    : Archetype<"asset", Requires<Kinematics, Health, missile::Asset>> {};
struct Radar final : Archetype<"radar", Requires<Kinematics, missile::Radar>> {
};
struct Launcher final
    : Archetype<"launcher", Requires<Kinematics, missile::Launcher>> {};
struct RedDrone final
    : Archetype<
          "red drone",
          Requires<Kinematics, Control, Health, Warhead, missile::RedDrone>,
          Allows<Tracked>> {};
struct Interceptor final
    : Archetype<"interceptor",
                Requires<Kinematics, Control, Warhead, missile::Interceptor>> {
};
struct Track final : Archetype<"track", Requires<missile::Track>> {};
struct Blast final : Archetype<"blast", Requires<Kinematics, missile::Blast>> {
};

}  // namespace archetype

}  // namespace simon::missile
