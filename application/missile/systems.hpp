// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>
#include <cstddef>
#include <optional>
#include <utility>

#include "application/missile/components.hpp"
#include "base/core.hpp"
#include "framework/system.hpp"
#include "model/guidance.hpp"
#include "model/motion.hpp"

namespace simon::missile {

using framework::System;
using framework::SystemList;
using framework::TypeList;
using namespace std::chrono_literals;

// Each system here names its access type as a member, `using LocalWorld =
// WorldAccess<ThisSystem>;`, so builder calls such as `detach<Tracked>()` need
// no `template` keyword.
template <typename SystemType>
using WorldAccess = framework::WorldAccess<SystemType, World>;

inline Length distance_between(const Position& a, const Position& b) {
  return norm(a - b);
}

//-- Sensing ------------------------------------------------------------------

// Each radar that scans this step creates a track for every red drone in range
// that has none. The drone is marked Tracked first; if another radar marked it
// earlier in this step, the builder refuses and the new track is dropped.
struct ScanRadars final : System<Radar, const Kinematics> {
  using LocalWorld = WorldAccess<ScanRadars>;
  using AllowComponentList = TypeList<Kinematics, RedDrone, Tracked>;

  void operator()(LocalWorld& world, Entity, Radar& radar,
                  const Kinematics* kinematics, Step step) {
    radar.scanned = false;
    if (!kinematics || !radar.scan.fire(step)) {
      return;
    }
    radar.scanned = true;
    const auto& drones = world.store_of<RedDrone>();
    for (std::size_t i = 0; i < drones.size(); ++i) {
      Entity drone = drones.owner(i);
      const Kinematics* drone_kinematics =
          world.try_component_of<Kinematics>(drone);
      if (world.try_component_of<Tracked>(drone) || !drone_kinematics ||
          distance(*kinematics, *drone_kinematics) > radar.range) {
        continue;
      }
      auto track = world.create<archetype::Track>()
                       .with(Track{.target = drone,
                                   .position = drone_kinematics->position,
                                   .velocity = drone_kinematics->velocity,
                                   .last_seen = step.time})
                       .build();
      if (!track) {
        continue;
      }
      if (!world.change(drone).attach(Tracked{.track = *track}).build()) {
        // Another radar got there first.
        auto dropped = world.destroy(*track).build();
        DECLARE_UNUSED(dropped);
      }
    }
  }
};

// Each track updates itself from any radar that scanned its target this step.
// The radars are perfect for now: the estimate is the truth.
struct UpdateTracks final : System<Track> {
  using LocalWorld = WorldAccess<UpdateTracks>;
  using SequenceAfterSystemList = SystemList<ScanRadars>;
  using AllowComponentList = TypeList<Kinematics, Radar>;

  void operator()(LocalWorld& world, Entity, Track& track, Step step) {
    const Kinematics* target = world.try_component_of<Kinematics>(track.target);
    if (!target) {
      return;
    }
    const auto& radars = world.store_of<Radar>();
    for (std::size_t i = 0; i < radars.size(); ++i) {
      const Kinematics* radar =
          world.try_component_of<Kinematics>(radars.owner(i));
      if (radars.data(i).scanned && radar &&
          distance(*radar, *target) <= radars.data(i).range) {
        track.position = target->position;
        track.velocity = target->velocity;
        track.last_seen = step.time;
        return;
      }
    }
  }
};

// Destroys tracks whose target is gone or has not been seen for `timeout`, and
// unmarks a surviving target so it can be tracked again.
struct DropStaleTracks final : System<const Track> {
  using LocalWorld = WorldAccess<DropStaleTracks>;
  using SequenceAfterSystemList = SystemList<UpdateTracks>;

  void operator()(LocalWorld& world, Entity self, const Track& track,
                  Step step) {
    bool gone = !world.alive(track.target);
    bool stale = step.time - track.last_seen > timeout;
    if (!gone && !stale) {
      return;
    }
    auto destroyed = world.destroy(self).build();
    DECLARE_UNUSED(destroyed);
    if (!gone) {
      auto unmarked = world.change(track.target).detach<Tracked>().build();
      DECLARE_UNUSED(unmarked);
    }
  }

  Duration timeout = 5s;
};

using Sensing = SystemList<ScanRadars, UpdateTracks, DropStaleTracks>;

//-- Engagement ---------------------------------------------------------------

// Each ready launcher proposes the nearest unengaged track in range.
struct ProposeEngagements final : System<Launcher, const Kinematics> {
  using LocalWorld = WorldAccess<ProposeEngagements>;
  using SequenceAfterSystemList = SystemList<DropStaleTracks>;
  using AllowComponentList = TypeList<Track>;

  void operator()(LocalWorld& world, Entity, Launcher& launcher,
                  const Kinematics* kinematics, Step step) {
    launcher.proposal = Entity{};
    TimePoint now = step.time;
    if (!kinematics || launcher.inventory == 0 || now < launcher.ready_at) {
      return;
    }
    const auto& tracks = world.store_of<Track>();
    std::optional<Length> nearest;
    for (std::size_t i = 0; i < tracks.size(); ++i) {
      const Track& track = tracks.data(i);
      bool engaged = track.engaged_by != Entity{} && now < track.engaged_until;
      Length range = distance_between(track.position, kinematics->position);
      if (!engaged && range <= launcher.range &&
          (!nearest || range < *nearest)) {
        nearest = range;
        launcher.proposal = tracks.owner(i);
      }
    }
  }
};

// Each unengaged track accepts the nearest launcher that proposed it. Ties go
// to the launcher that comes first in iteration order.
struct ResolveEngagements final : System<Track> {
  using LocalWorld = WorldAccess<ResolveEngagements>;
  using SequenceAfterSystemList = SystemList<ProposeEngagements>;
  using AllowComponentList = TypeList<Launcher, Kinematics>;

  void operator()(LocalWorld& world, Entity self, Track& track, Step step) {
    TimePoint now = step.time;
    if (track.engaged_by != Entity{} && now < track.engaged_until) {
      return;
    }
    const auto& launchers = world.store_of<Launcher>();
    std::optional<Length> nearest;
    for (std::size_t i = 0; i < launchers.size(); ++i) {
      const Kinematics* launcher =
          world.try_component_of<Kinematics>(launchers.owner(i));
      if (launchers.data(i).proposal != self || !launcher) {
        continue;
      }
      Length range = distance_between(track.position, launcher->position);
      if (!nearest || range < *nearest) {
        nearest = range;
        track.engaged_by = launchers.owner(i);
        track.engaged_until = now + engagement;
      }
    }
  }

  Duration engagement = 30s;  // About an interceptor's flight time.
};

// What every interceptor is built with.
struct InterceptorDesign final {
  Speed speed = 150.0 * model::meter_per_second;
  AccelerationMagnitude agility = 300.0 * model::meter_per_second_squared;
  double navigation_gain = 4.0;
  Length seeker_range = 1000.0 * model::meter;
  Duration lifetime = 30s;
  Warhead warhead{.fuse = 15.0 * model::meter,
                  .radius = 25.0 * model::meter,
                  .damage = 5.0};
};

// Launchers whose proposal was accepted build an interceptor aimed at the
// track, under themselves.
struct LaunchInterceptors final : System<Launcher, const Kinematics> {
  using LocalWorld = WorldAccess<LaunchInterceptors>;
  using SequenceAfterSystemList = SystemList<ResolveEngagements>;
  using AllowComponentList = TypeList<Track>;

  void operator()(LocalWorld& world, Entity self, Launcher& launcher,
                  const Kinematics* kinematics, Step step) {
    Entity proposal = std::exchange(launcher.proposal, Entity{});
    const Track* track = world.try_component_of<Track>(proposal);
    if (!kinematics || !track || track->engaged_by != self) {
      return;
    }
    TimePoint now = step.time;
    model::Displacement aim = track->position - kinematics->position;
    Length range = norm(aim);
    Velocity velocity = range > 0.0 * model::meter
                            ? aim * (design.speed / range)
                            : model::meters_per_second(0.0, 0.0, 0.0);
    auto interceptor =
        world.create<archetype::Interceptor>()
            .under(self)
            .with(Kinematics{.position = kinematics->position,
                             .velocity = velocity})
            .with(Control{})
            .with(design.warhead)
            .with(Interceptor{.target = track->target,
                              .navigation_gain = design.navigation_gain,
                              .speed = design.speed,
                              .agility = design.agility,
                              .seeker_range = design.seeker_range,
                              .expires_at = now + design.lifetime})
            .build();
    if (interceptor) {
      --launcher.inventory;
      launcher.ready_at = now + launcher.reload;
    }
  }

  InterceptorDesign design;
};

using Engagement =
    SystemList<ProposeEngagements, ResolveEngagements, LaunchInterceptors>;

//-- Guidance -----------------------------------------------------------------

// Proportional navigation toward the target, holding speed. When the target
// is gone, retargets the nearest red drone within seeker range, or
// self-destructs. Also self-destructs when its flight time runs out.
struct GuideInterceptors final
    : System<Interceptor, const Kinematics, Control> {
  using LocalWorld = WorldAccess<GuideInterceptors>;
  using SequenceAfterSystemList = SystemList<LaunchInterceptors>;
  using AllowComponentList = TypeList<Kinematics, RedDrone>;

  void operator()(LocalWorld& world, Entity self, Interceptor& interceptor,
                  const Kinematics* kinematics, Control* control, Step step) {
    if (!kinematics || !control) {
      return;
    }
    if (step.time >= interceptor.expires_at) {
      auto destroyed = world.destroy(self).build();
      DECLARE_UNUSED(destroyed);
      return;
    }
    const Kinematics* target =
        world.try_component_of<Kinematics>(interceptor.target);
    if (!target) {
      target = retarget(interceptor, *kinematics, world);
    }
    if (!target) {
      auto destroyed = world.destroy(self).build();
      DECLARE_UNUSED(destroyed);
      return;
    }
    constexpr Rate SPEED_RESPONSE = 2.0 * model::per_second;
    control->acceleration = model::limit(
        model::proportional_navigation(*kinematics, *target,
                                       interceptor.navigation_gain) +
            model::hold_speed(*kinematics, interceptor.speed, SPEED_RESPONSE),
        interceptor.agility);
  }

 private:
  static const Kinematics* retarget(Interceptor& interceptor,
                                    const Kinematics& kinematics,
                                    LocalWorld& world) {
    const auto& drones = world.store_of<RedDrone>();
    const Kinematics* nearest = nullptr;
    for (std::size_t i = 0; i < drones.size(); ++i) {
      const Kinematics* drone =
          world.try_component_of<Kinematics>(drones.owner(i));
      if (drone && distance(kinematics, *drone) <= interceptor.seeker_range &&
          (!nearest ||
           distance(kinematics, *drone) < distance(kinematics, *nearest))) {
        nearest = drone;
        interceptor.target = drones.owner(i);
      }
    }
    return nearest;
  }
};

// Red drones steer at their target at cruise speed.
struct SteerRedDrones final
    : System<const RedDrone, const Kinematics, Control> {
  using LocalWorld = WorldAccess<SteerRedDrones>;
  using AllowComponentList = TypeList<Kinematics>;

  void operator()(LocalWorld& world, Entity, const RedDrone& drone,
                  const Kinematics* kinematics, Control* control) {
    if (!kinematics || !control) {
      return;
    }
    constexpr Rate RESPONSE = 1.0 * model::per_second;
    const Kinematics* target = world.try_component_of<Kinematics>(drone.target);
    control->acceleration =
        target ? model::limit(model::steer_toward(*kinematics, target->position,
                                                  drone.cruise, RESPONSE),
                              drone.agility)
               : model::meters_per_second_squared(0.0, 0.0, 0.0);
  }
};

//-- Blasts -------------------------------------------------------------------

// A warhead within its fuse distance of its target detonates: it creates a
// Blast and destroys itself.
struct TriggerWarheads final : System<const Warhead, const Kinematics,
                                      const Interceptor, const RedDrone> {
  using LocalWorld = WorldAccess<TriggerWarheads>;
  using SequenceAfterSystemList = SystemList<model::Integrate>;
  using AllowComponentList = TypeList<Kinematics>;

  void operator()(LocalWorld& world, Entity self, const Warhead& warhead,
                  const Kinematics* kinematics, const Interceptor* interceptor,
                  const RedDrone* drone) {
    Entity target = interceptor ? interceptor->target
                    : drone     ? drone->target
                                : Entity{};
    const Kinematics* target_kinematics =
        world.try_component_of<Kinematics>(target);
    if (!kinematics || !target_kinematics ||
        distance(*kinematics, *target_kinematics) > warhead.fuse) {
      return;
    }
    auto blast = world.create<archetype::Blast>()
                     .under(self)
                     .with(Kinematics{.position = kinematics->position})
                     .with(Blast{.radius = warhead.radius,
                                 .damage = warhead.damage,
                                 .source = world.name_of(self)})
                     .build();
    DECLARE_UNUSED(blast);
    auto destroyed = world.destroy(self).build();
    DECLARE_UNUSED(destroyed);
  }
};

// Every entity with Health takes damage from each blast it is inside, and is
// destroyed when its health runs out. Victims are the batch; each writes only
// itself.
struct ApplyBlasts final : System<Health, const Kinematics> {
  using LocalWorld = WorldAccess<ApplyBlasts>;
  using SequenceAfterSystemList = SystemList<TriggerWarheads>;
  using AllowComponentList = TypeList<Blast, Kinematics>;

  void operator()(LocalWorld& world, Entity self, Health& health,
                  const Kinematics* kinematics) {
    if (!kinematics) {
      return;
    }
    const auto& blasts = world.store_of<Blast>();
    for (std::size_t i = 0; i < blasts.size(); ++i) {
      const Kinematics* blast =
          world.try_component_of<Kinematics>(blasts.owner(i));
      if (blast && distance(*kinematics, *blast) <= blasts.data(i).radius) {
        health.points -= blasts.data(i).damage;
      }
    }
    if (health.points <= 0.0) {
      auto destroyed = world.destroy(self).build();
      DECLARE_UNUSED(destroyed);
    }
  }
};

// Blasts live for one step.
struct ExpireBlasts final : System<const Blast> {
  using LocalWorld = WorldAccess<ExpireBlasts>;
  using SequenceAfterSystemList = SystemList<ApplyBlasts>;

  void operator()(LocalWorld& world, Entity self, const Blast&) {
    auto destroyed = world.destroy(self).build();
    DECLARE_UNUSED(destroyed);
  }
};

using Blasts = SystemList<TriggerWarheads, ApplyBlasts, ExpireBlasts>;

//-- Schedule -----------------------------------------------------------------

using Schedule = SystemList<Sensing, Engagement, GuideInterceptors,
                            SteerRedDrones, model::Motion, Blasts>;
using Scheduler = framework::Scheduler<World, Schedule>;

}  // namespace simon::missile
