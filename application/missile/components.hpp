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

// The entity this one flies at: the asset for a red drone, the drone for an
// interceptor. A sibling both archetypes require, so a system that needs only
// the target, such as TriggerWarheads, reads 8 bytes.
struct Target final {
  Entity entity;
};

struct RedDrone final {
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

// Blue's belief about a red drone: the drone, and when a radar last saw it.
// A track's estimate and engagement are siblings its archetype requires, so
// systems that need one of them read only that.
struct Track final {
  Entity target;
  TimePoint last_seen{};
};

// Blue's estimate of a tracked drone's motion.
struct Estimate final {
  Position position = model::meters(0.0, 0.0, 0.0);
  Velocity velocity = model::meters_per_second(0.0, 0.0, 0.0);
};

// The launcher engaging a track, if any, and until when.
struct Engagement final {
  Entity engaged_by;
  TimePoint engaged_until{};  // When the engagement lapses if it has not ended.
};

struct Launcher final {
  Length range = 0.0 * model::meter;
  std::uint32_t inventory = 0;
  Duration reload{};
  TimePoint ready_at{};
  Entity proposal;  // The track it proposes to engage this step.
};

// Keeps a launcher from engaging. Operators impose and lift it over a sector;
// see hold_weapons and free_weapons.
struct WeaponsHold final {};

struct Interceptor final {
  double navigation_gain = 4.0;
  Speed speed = 0.0 * model::meter_per_second;
  AccelerationMagnitude agility = 0.0 * model::meter_per_second_squared;
  Length seeker_range = 0.0 * model::meter;
  TimePoint expires_at{};
};

namespace archetype {

using framework::Allows;
using framework::Archetype;
using framework::Requires;

struct Asset final                                                   //
    : Archetype<"asset",                                             //
                Requires<Kinematics, Health, missile::Asset>> {};    //
struct Radar final                                                   //
    : Archetype<"radar",                                             //
                Requires<Kinematics, missile::Radar>> {};            //
struct Launcher final                                                //
    : Archetype<"launcher",                                          //
                Requires<Kinematics, missile::Launcher>,             //
                Allows<WeaponsHold>> {};                             //
struct RedDrone final                                                //
    : Archetype<"red drone",                                         //
                Requires<Kinematics,                                 //
                         Control,                                    //
                         Health,                                     //
                         Warhead,                                    //
                         Target,                                     //
                         missile::RedDrone>,                         //
                Allows<Tracked>> {};                                 //
struct Interceptor final                                             //
    : Archetype<"interceptor",                                       //
                Requires<Kinematics,                                 //
                         Control,                                    //
                         Warhead,                                    //
                         Target,                                     //
                         missile::Interceptor>> {};                  //
struct Track final                                                   //
    : Archetype<"track",                                             //
                Requires<missile::Track, Estimate, Engagement>> {};  //
struct Blast final                                                   //
    : Archetype<"blast",                                             //
                Requires<Kinematics, missile::Blast>> {};            //

}  // namespace archetype

using World = framework::World<
    Kinematics,
    framework::TypeList<Control, Health, Warhead, Blast, Target, RedDrone,
                        Tracked, Asset, Radar, Track, Estimate, Engagement,
                        Launcher, WeaponsHold, Interceptor>,
    framework::TypeList<archetype::Asset, archetype::Radar, archetype::Launcher,
                        archetype::RedDrone, archetype::Interceptor,
                        archetype::Track, archetype::Blast>>;

}  // namespace simon::missile
