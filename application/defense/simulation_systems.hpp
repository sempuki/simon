// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "application/defense/simulation_components.hpp"
#include "base/core.hpp"
#include "framework/system.hpp"
#include "model/guidance.hpp"

namespace simon::defense {

using framework::System;
using framework::SystemList;
using framework::TypeList;
using namespace std::chrono_literals;

// Each system here names its projected world as a member, `using SystemWorld
// = ProjectedWorld<ThisSystem>;`, so builder calls such as `detach<Tracked>()`
// need no `template` keyword.
template <typename SystemType>
using ProjectedWorld = framework::ProjectedWorld<SystemType, World>;

inline auto distance_between(const Position& a, const Position& b) -> Length {
  return norm(a - b);
}

//-- Sensing ------------------------------------------------------------------

// Each radar decides whether it scans this step. What a scan finds is decided
// by the drones and tracks themselves (DetectDrones, UpdateTracks), so each
// entity writes only itself.
struct ScanRadars final  //
    : System<Radar> {
  using SystemWorld = ProjectedWorld<ScanRadars>;

  auto operator()(SystemWorld&, Entity, Radar& radar, Step step) -> void {
    radar.scanned = radar.scan.fire(step).has_value();
  }
};

// The radars that scanned this step, indexed so a query looks only at radars
// near it. Systems build one in `prepare`.
class ScanningRadars final {
 public:
  // Collects and indexes the radars that scanned. Returns whether any did.
  template <typename ProjectedWorldType>
  auto collect(ProjectedWorldType& world) -> bool {
    scanning_.clear();
    longest_ = 0.0 * meter;
    const auto& radars = world.template store_of<Radar>();
    radars.for_each([&](Entity owner, const Radar& radar) {
      const Kinematics* kinematics =
          world.template maybe_component_of<Kinematics>(owner);
      if (radar.scanned && kinematics) {
        scanning_.push_back(
            Scanning{.radar = kinematics, .range = radar.range});
        longest_ = std::max(longest_, radar.range);
      }
    });
    if (scanning_.empty()) {
      return false;
    }
    if (!index_) {
      index_.emplace(radars.capacity());
    }
    index_->rebuild(scanning_.size(), [&](std::size_t slot) {
      return model::coordinates(*scanning_[slot].radar);
    });
    return true;
  }

  // Whether a radar that scanned is within its range of `target`. Looks only
  // as far as the longest range.
  auto cover(const Kinematics& target) const -> bool {
    if (scanning_.empty()) {
      return false;
    }
    return index_
        ->nearest(model::coordinates(target),
                  longest_.numerical_value_in(meter),
                  [&](std::uint32_t slot) {
                    const Scanning& scanning = scanning_[slot];
                    return distance(*scanning.radar, target) <= scanning.range;
                  })
        .has_value();
  }

 private:
  struct Scanning final {
    const Kinematics* radar = nullptr;  // Valid until the next sync point.
    Length range = 0.0 * meter;
  };

  std::vector<Scanning> scanning_;
  Length longest_ = 0.0 * meter;
  std::optional<framework::SpatialIndex> index_;  // Sized on first use.
};

// Each untracked red drone that a scanning radar covers creates its own track
// and marks itself Tracked. The drone writes only itself and creates at most
// one track, so however many radars see it, it gets one.
struct DetectDrones final       //
    : System<const RedDrone,    //
             const Kinematics,  //
             const Tracked> {
  using SystemWorld = ProjectedWorld<DetectDrones>;
  using SequenceAfterSystemList = SystemList<ScanRadars>;
  using AllowComponentList = TypeList<Kinematics, Radar>;

  // Steps without a scan have nothing to detect.
  auto prepare(SystemWorld& world) -> bool { return radars_.collect(world); }

  auto operator()(SystemWorld& world, Entity self,  //
                  const RedDrone&,                  //
                  const Kinematics* kinematics,     //
                  const Tracked* tracked,           //
                  Step step) -> void {
    if (tracked || !kinematics || !radars_.cover(*kinematics)) {
      return;
    }
    auto track = world.create<archetype::Track>()
                     .with(Track{.target = self, .last_seen = step.time})
                     .with(Estimate{.position = kinematics->position,
                                    .velocity = kinematics->velocity})
                     .with(Engagement{})
                     .build();
    if (!track) {
      return;
    }
    if (!world.change(self).attach(Tracked{.track = *track}).build()) {
      auto _ = world.destroy(*track).build();
    }
  }

 private:
  ScanningRadars radars_;
};

// Each track updates its estimate from any radar that scanned its target this
// step. The radars are perfect for now: the estimate is the truth.
struct UpdateTracks final  //
    : System<Track,        //
             Estimate> {
  using SystemWorld = ProjectedWorld<UpdateTracks>;
  using SequenceAfterSystemList = SystemList<DetectDrones>;
  using AllowComponentList = TypeList<Kinematics, Radar>;

  // Steps without a scan have nothing to update.
  auto prepare(SystemWorld& world) -> bool { return radars_.collect(world); }

  auto operator()(SystemWorld& world, Entity,  //
                  Track& track,                //
                  Estimate* estimate,          //
                  Step step) -> void {
    const Kinematics* target =
        world.maybe_component_of<Kinematics>(track.target);
    if (!estimate || !target || !radars_.cover(*target)) {
      return;
    }
    estimate->position = target->position;
    estimate->velocity = target->velocity;
    track.last_seen = step.time;
  }

 private:
  ScanningRadars radars_;
};

// Destroys tracks whose target is gone or has not been seen for `timeout`, and
// unmarks a surviving target so it can be tracked again.
struct DropStaleTracks final  //
    : System<const Track> {
  using SystemWorld = ProjectedWorld<DropStaleTracks>;
  using SequenceAfterSystemList = SystemList<UpdateTracks>;

  auto operator()(SystemWorld& world, Entity self,  //
                  const Track& track,               //
                  Step step) -> void {
    bool gone = !world.alive(track.target);
    bool stale = step.time - track.last_seen > timeout;
    if (!gone && !stale) {
      return;
    }
    auto _ = world.destroy(self).build();
    if (!gone) {
      auto _ = world.change(track.target).detach<Tracked>().build();
    }
  }

  Duration timeout = 5s;
};

using Sensing =
    SystemList<ScanRadars, DetectDrones, UpdateTracks, DropStaleTracks>;

//-- Engagement ---------------------------------------------------------------

// Each ready launcher proposes the nearest unengaged track in range, unless
// it is under a weapons hold.
//
// Tracks are not in the world's spatial index: a track's position is blue's
// estimate, and UpdateTracks could not write a track Kinematics while reading
// its target's. So this system indexes the estimates itself, on the first
// query in a step: most steps no launcher is ready, and building it in
// prepare every step costs four times as much. This relies on the
// per-entity loop running on one thread.
struct ProposeEngagements final  //
    : System<Launcher,           //
             const Kinematics,   //
             const WeaponsHold> {
  using SystemWorld = ProjectedWorld<ProposeEngagements>;
  using SequenceAfterSystemList = SystemList<DropStaleTracks>;
  using AllowComponentList = TypeList<Estimate, Engagement>;

  auto prepare(SystemWorld&) -> void { indexed_ = false; }

  auto operator()(SystemWorld& world, Entity,    //
                  Launcher& launcher,            //
                  const Kinematics* kinematics,  //
                  const WeaponsHold* hold,       //
                  Step step) -> void {
    launcher.proposal = Entity{};
    TimePoint now = step.time;
    if (!kinematics || hold || launcher.inventory == 0 ||
        now < launcher.ready_at) {
      return;
    }
    const auto& estimates = world.store_of<Estimate>();
    if (!indexed_) {
      if (!tracks_) {
        tracks_.emplace(estimates.capacity());
      }
      tracks_->rebuild([&](auto&& insert) {
        estimates.for_each_slot(
            [&](std::uint32_t slot, Entity, const Estimate& estimate) {
              insert(slot, model::coordinates(estimate.position));
            });
      });
      indexed_ = true;
    }
    std::optional<std::uint32_t> nearest = tracks_->nearest(
        model::coordinates(kinematics->position),
        launcher.range.numerical_value_in(meter), [&](std::uint32_t slot) {
          const Engagement* engagement =
              world.maybe_component_of<Engagement>(estimates.owner_at(slot));
          return engagement && (engagement->engaged_by == Entity{} ||
                                now >= engagement->engaged_until);
        });
    if (nearest) {
      launcher.proposal = estimates.owner_at(*nearest);
    }
  }

 private:
  std::optional<framework::SpatialIndex> tracks_;  // Sized on first use.
  bool indexed_ = false;
};

// Each unengaged track accepts the nearest launcher that proposed it. Ties go
// to the launcher that comes first in iteration order.
struct ResolveEngagements final  //
    : System<Engagement,         //
             const Estimate> {
  using SystemWorld = ProjectedWorld<ResolveEngagements>;
  using SequenceAfterSystemList = SystemList<ProposeEngagements>;
  using AllowComponentList = TypeList<Launcher, Kinematics>;

  // Indexes this step's proposals by track, once, so each track finds its
  // proposers without scanning every launcher. Launchers are indexed in store
  // order, so ties go to the launcher that comes first. Steps without
  // proposals have nothing to resolve.
  auto prepare(SystemWorld& world) -> bool {
    proposals_.clear();
    world.store_of<Launcher>().for_each(
        [&](Entity owner, const Launcher& launcher) {
          if (launcher.proposal != Entity{}) {
            proposals_.push_back(
                Proposal{.track = launcher.proposal.index, .launcher = owner});
          }
        });
    std::ranges::stable_sort(proposals_, {}, &Proposal::track);
    return !proposals_.empty();
  }

  auto operator()(SystemWorld& world, Entity self,  //
                  Engagement& engagement,           //
                  const Estimate* estimate,         //
                  Step step) -> void {
    TimePoint now = step.time;
    if (!estimate ||
        (engagement.engaged_by != Entity{} && now < engagement.engaged_until)) {
      return;
    }
    auto [begin, end] =
        std::ranges::equal_range(proposals_, self.index, {}, &Proposal::track);
    std::optional<Length> nearest;
    for (auto proposal = begin; proposal != end; ++proposal) {
      const Kinematics* launcher =
          world.maybe_component_of<Kinematics>(proposal->launcher);
      if (!launcher || world.store_of<Launcher>()
                               .component_of(proposal->launcher)
                               .proposal != self) {
        continue;  // A stale index entry for a reused entity index.
      }
      Length range = distance_between(estimate->position, launcher->position);
      if (!nearest || range < *nearest) {
        nearest = range;
        engagement.engaged_by = proposal->launcher;
        engagement.engaged_until = now + duration;
      }
    }
  }

  Duration duration = 30s;  // About an interceptor's flight time.

 private:
  struct Proposal final {
    std::uint32_t track = 0;  // The proposed track's entity index.
    Entity launcher;
  };

  std::vector<Proposal> proposals_;
};

// The parameters every interceptor is built with.
struct InterceptorDesign final {
  Speed speed = 150.0 * meter_per_second;
  AccelerationMagnitude agility = 300.0 * meter_per_second_squared;
  double navigation_gain = 4.0;
  Length seeker_range = 1000.0 * meter;
  Duration lifetime = 30s;
  Warhead warhead{.fuse = 15.0 * meter, .radius = 25.0 * meter, .damage = 5.0};
};

// Launchers whose proposal was accepted build an interceptor aimed at the
// track, under themselves.
struct LaunchInterceptors final  //
    : System<Launcher,           //
             const Kinematics> {
  using SystemWorld = ProjectedWorld<LaunchInterceptors>;
  using SequenceAfterSystemList = SystemList<ResolveEngagements>;
  using AllowComponentList = TypeList<Track, Estimate, Engagement>;

  auto operator()(SystemWorld& world, Entity self,  //
                  Launcher& launcher,               //
                  const Kinematics* kinematics,     //
                  Step step) -> void {
    Entity proposal = std::exchange(launcher.proposal, Entity{});
    const Track* track = world.maybe_component_of<Track>(proposal);
    const Estimate* estimate = world.maybe_component_of<Estimate>(proposal);
    const Engagement* engagement =
        world.maybe_component_of<Engagement>(proposal);
    if (!kinematics || !track || !estimate || !engagement ||
        engagement->engaged_by != self) {
      return;
    }
    TimePoint now = step.time;
    Displacement aim = estimate->position - kinematics->position;
    Length range = norm(aim);
    Velocity velocity = range > 0.0 * meter ? aim * (design.speed / range)
                                            : meters_per_second(0.0, 0.0, 0.0);
    auto interceptor =
        world.create<archetype::Interceptor>()
            .under(self)
            .with(Kinematics{.position = kinematics->position,
                             .velocity = velocity})
            .with(Control{})
            .with(design.warhead)
            .with(Target{.entity = track->target})
            .with(Interceptor{.navigation_gain = design.navigation_gain,
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

using Engaging =
    SystemList<ProposeEngagements, ResolveEngagements, LaunchInterceptors>;

//-- Guidance -----------------------------------------------------------------

// Proportional navigation toward the target, holding speed. When the target
// is gone, retargets the nearest red drone within seeker range, or
// self-destructs. Also self-destructs when its flight time runs out.
struct GuideInterceptors final   //
    : System<const Interceptor,  //
             const Kinematics,   //
             Control,            //
             Target> {
  using SystemWorld = ProjectedWorld<GuideInterceptors>;
  using SequenceAfterSystemList = SystemList<LaunchInterceptors>;
  using AllowComponentList = TypeList<Kinematics, RedDrone>;

  auto operator()(SystemWorld& world, Entity self,  //
                  const Interceptor& interceptor,   //
                  const Kinematics* kinematics,     //
                  Control* control,                 //
                  Target* target_of,                //
                  Step step) -> void {
    if (!kinematics || !control || !target_of) {
      return;
    }
    if (step.time >= interceptor.expires_at) {
      auto _ = world.destroy(self).build();
      return;
    }
    const Kinematics* target =
        world.maybe_component_of<Kinematics>(target_of->entity);
    if (!target) {
      target = retarget(interceptor, *kinematics, *target_of, world);
    }
    if (!target) {
      auto _ = world.destroy(self).build();
      return;
    }
    constexpr Rate SPEED_RESPONSE = 2.0 * per_second;
    control->acceleration = model::limit(
        model::compute_proportional_navigation(*kinematics, *target,
                                               interceptor.navigation_gain) +
            model::hold_speed(*kinematics, interceptor.speed, SPEED_RESPONSE),
        interceptor.agility);
  }

 private:
  static auto retarget(const Interceptor& interceptor,
                       const Kinematics& kinematics, Target& target,
                       SystemWorld& world) -> const Kinematics* {
    std::optional<Entity> nearest = world.nearest(
        kinematics, interceptor.seeker_range,
        [&](Entity candidate, const Kinematics&) {
          return world.maybe_component_of<RedDrone>(candidate) != nullptr;
        });
    if (!nearest) {
      return nullptr;
    }
    target.entity = *nearest;
    return &world.component_of<Kinematics>(*nearest);
  }
};

// Red drones steer at their target at cruise speed.
struct SteerRedDrones final     //
    : System<const RedDrone,    //
             const Kinematics,  //
             Control,           //
             const Target> {
  using SystemWorld = ProjectedWorld<SteerRedDrones>;
  using AllowComponentList = TypeList<Kinematics>;

  auto operator()(SystemWorld& world, Entity,    //
                  const RedDrone& drone,         //
                  const Kinematics* kinematics,  //
                  Control* control,              //
                  const Target* target_of) -> void {
    if (!kinematics || !control || !target_of) {
      return;
    }
    constexpr Rate RESPONSE = 1.0 * per_second;
    const Kinematics* target =
        world.maybe_component_of<Kinematics>(target_of->entity);
    control->acceleration =
        target ? model::limit(model::steer_toward(*kinematics, target->position,
                                                  drone.cruise, RESPONSE),
                              drone.agility)
               : meters_per_second_squared(0.0, 0.0, 0.0);
  }
};

//-- Motion -------------------------------------------------------------------

// Moves each entity under its Control by the midpoint rule. Entities
// without a Control coast.
struct Integrate final    //
    : System<Kinematics,  //
             const Control> {
  auto operator()(auto&, Entity,           //
                  Kinematics& kinematics,  //
                  const Control* control,  //
                  Step step) const -> void {
    model::integrate_midpoint(
        control ? control->acceleration : meters_per_second_squared(0, 0, 0),
        seconds(step.dt), InOut(kinematics));
  }
};

//-- Blasts -------------------------------------------------------------------

// A warhead within its fuse distance of its target detonates: it creates a
// Blast and destroys itself.
//
// Every warhead checks its fuse every step, and few detonate, so the check is
// all the call operator does: small enough to inline into the loop. The
// builders that detonate are out of line and marked cold.
struct TriggerWarheads final    //
    : System<const Warhead,     //
             const Kinematics,  //
             const Target> {
  using SystemWorld = ProjectedWorld<TriggerWarheads>;
  using SequenceAfterSystemList = SystemList<Integrate>;
  using AllowComponentList = TypeList<Kinematics>;

  auto operator()(SystemWorld& world, Entity self,  //
                  const Warhead& warhead,           //
                  const Kinematics* kinematics,     //
                  const Target* target) -> void {
    const Kinematics* target_kinematics =
        target ? world.maybe_component_of<Kinematics>(target->entity) : nullptr;
    if (kinematics && target_kinematics &&
        within_distance(*kinematics, *target_kinematics, warhead.fuse)) {
      detonate(world, self, warhead, *kinematics);
    }
  }

 private:
  [[gnu::cold, gnu::noinline]] static auto detonate(
      SystemWorld& world, Entity self, const Warhead& warhead,
      const Kinematics& kinematics) -> void {
    auto _ = world.create<archetype::Blast>()
                 .under(self)
                 .with(Kinematics{.position = kinematics.position})
                 .with(Blast{.radius = warhead.radius,
                             .damage = warhead.damage,
                             .source = world.name_of(self)})
                 .build();
    auto _ = world.destroy(self).build();
  }
};

// Every entity with Health takes damage from each blast it is inside, and is
// destroyed when its health runs out. Victims are the batch; each writes only
// itself.
struct ApplyBlasts final  //
    : System<Health,      //
             const Kinematics> {
  using SystemWorld = ProjectedWorld<ApplyBlasts>;
  using SequenceAfterSystemList = SystemList<TriggerWarheads>;
  using AllowComponentList = TypeList<Blast, Kinematics>;

  // Collects this step's blasts, once, so each victim reads a short array
  // instead of walking the Blast store. Most steps have none, and then there
  // is nothing to apply.
  auto prepare(SystemWorld& world) -> bool {
    blasts_.clear();
    world.store_of<Blast>().for_each([&](Entity owner, const Blast& blast) {
      if (const Kinematics* center =
              world.maybe_component_of<Kinematics>(owner)) {
        blasts_.push_back(Burst{
            .center = center, .radius = blast.radius, .damage = blast.damage});
      }
    });
    return !blasts_.empty();
  }

  auto operator()(SystemWorld& world, Entity self,  //
                  Health& health,                   //
                  const Kinematics* kinematics) -> void {
    if (!kinematics) {
      return;
    }
    for (const Burst& burst : blasts_) {
      if (distance(*kinematics, *burst.center) <= burst.radius) {
        health.points -= burst.damage;
      }
    }
    if (health.points <= 0.0) {
      auto _ = world.destroy(self).build();
    }
  }

 private:
  struct Burst final {
    const Kinematics* center = nullptr;  // Valid until the next sync point.
    Length radius = 0.0 * meter;
    double damage = 0.0;
  };
  std::vector<Burst> blasts_;
};

// Blasts live for one step.
struct ExpireBlasts final  //
    : System<const Blast> {
  using SystemWorld = ProjectedWorld<ExpireBlasts>;
  using SequenceAfterSystemList = SystemList<ApplyBlasts>;

  auto operator()(SystemWorld& world, Entity self, const Blast&) -> void {
    auto _ = world.destroy(self).build();
  }
};

using Blasts = SystemList<TriggerWarheads, ApplyBlasts, ExpireBlasts>;

//-- Schedule -----------------------------------------------------------------

using Schedule = SystemList<Sensing, Engaging, GuideInterceptors,
                            SteerRedDrones, Integrate, Blasts>;
using Scheduler = framework::Scheduler<World, Schedule>;

}  // namespace simon::defense
