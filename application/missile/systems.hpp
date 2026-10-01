// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

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

// Each radar decides whether it scans this step. What a scan finds is decided
// by the drones and tracks themselves (DetectDrones, UpdateTracks), so each
// entity writes only itself.
struct ScanRadars final : System<Radar> {
  using LocalWorld = WorldAccess<ScanRadars>;

  void operator()(LocalWorld&, Entity, Radar& radar, Step step) {
    radar.scanned = radar.scan.fire(step).has_value();
  }
};

// The radars that scanned this step, indexed so a query looks only at radars
// near it. Systems build one in `prepare`.
class ScanningRadars final {
 public:
  // Collects and indexes the radars that scanned. Returns whether any did.
  template <typename WorldAccessType>
  bool collect(WorldAccessType& world) {
    scanning_.clear();
    longest_ = 0.0 * model::meter;
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
      index_.emplace(radars.capacity(), CELL_SIZE);
    }
    index_->rebuild(scanning_.size(), [&](std::size_t slot) {
      return model::coordinates(*scanning_[slot].radar);
    });
    return true;
  }

  // Whether a radar that scanned is within its range of `target`. Looks only
  // as far as the longest range.
  bool cover(const Kinematics& target) const {
    if (scanning_.empty()) {
      return false;
    }
    return index_
        ->nearest(model::coordinates(target),
                  longest_.numerical_value_in(model::meter),
                  [&](std::uint32_t slot) {
                    const Scanning& scanning = scanning_[slot];
                    return distance(*scanning.radar, target) <= scanning.range;
                  })
        .has_value();
  }

 private:
  static constexpr double CELL_SIZE = 4000.0;  // Meters, about a radar's range.

  struct Scanning final {
    const Kinematics* radar = nullptr;  // Valid until the next sync point.
    Length range = 0.0 * model::meter;
  };
  std::vector<Scanning> scanning_;
  Length longest_ = 0.0 * model::meter;
  std::optional<framework::SpatialIndex> index_;  // Sized on first use.
};

// Each untracked red drone that a scanning radar covers creates its own track
// and marks itself Tracked. The drone writes only itself and creates at most
// one track, so however many radars see it, it gets one.
struct DetectDrones final
    : System<const RedDrone, const Kinematics, const Tracked> {
  using LocalWorld = WorldAccess<DetectDrones>;
  using SequenceAfterSystemList = SystemList<ScanRadars>;
  using AllowComponentList = TypeList<Kinematics, Radar>;

  // Steps without a scan have nothing to detect.
  bool prepare(LocalWorld& world) { return radars_.collect(world); }

  void operator()(LocalWorld& world, Entity self, const RedDrone&,
                  const Kinematics* kinematics, const Tracked* tracked,
                  Step step) {
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
      auto dropped = world.destroy(*track).build();
      DECLARE_UNUSED(dropped);
    }
  }

 private:
  ScanningRadars radars_;
};

// Each track updates its estimate from any radar that scanned its target this
// step. The radars are perfect for now: the estimate is the truth.
struct UpdateTracks final : System<Track, Estimate> {
  using LocalWorld = WorldAccess<UpdateTracks>;
  using SequenceAfterSystemList = SystemList<DetectDrones>;
  using AllowComponentList = TypeList<Kinematics, Radar>;

  // Steps without a scan have nothing to update.
  bool prepare(LocalWorld& world) { return radars_.collect(world); }

  void operator()(LocalWorld& world, Entity, Track& track, Estimate* estimate,
                  Step step) {
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
struct ProposeEngagements final
    : System<Launcher, const Kinematics, const WeaponsHold> {
  using LocalWorld = WorldAccess<ProposeEngagements>;
  using SequenceAfterSystemList = SystemList<DropStaleTracks>;
  using AllowComponentList = TypeList<Estimate, Engagement>;

  void prepare(LocalWorld&) { indexed_ = false; }

  void operator()(LocalWorld& world, Entity, Launcher& launcher,
                  const Kinematics* kinematics, const WeaponsHold* hold,
                  Step step) {
    launcher.proposal = Entity{};
    TimePoint now = step.time;
    if (!kinematics || hold || launcher.inventory == 0 ||
        now < launcher.ready_at) {
      return;
    }
    const auto& estimates = world.store_of<Estimate>();
    if (!indexed_) {
      if (!tracks_) {
        tracks_.emplace(estimates.capacity(), CELL_SIZE);
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
        launcher.range.numerical_value_in(model::meter),
        [&](std::uint32_t slot) {
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
  static constexpr double CELL_SIZE = 250.0;       // Meters.
  std::optional<framework::SpatialIndex> tracks_;  // Sized on first use.
  bool indexed_ = false;
};

// Each unengaged track accepts the nearest launcher that proposed it. Ties go
// to the launcher that comes first in iteration order.
struct ResolveEngagements final : System<Engagement, const Estimate> {
  using LocalWorld = WorldAccess<ResolveEngagements>;
  using SequenceAfterSystemList = SystemList<ProposeEngagements>;
  using AllowComponentList = TypeList<Launcher, Kinematics>;

  // Indexes this step's proposals by track, once, so each track finds its
  // proposers without scanning every launcher. Launchers are indexed in store
  // order, so ties still go to the launcher that comes first. Steps without
  // proposals have nothing to resolve.
  bool prepare(LocalWorld& world) {
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

  void operator()(LocalWorld& world, Entity self, Engagement& engagement,
                  const Estimate* estimate, Step step) {
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
  using AllowComponentList = TypeList<Track, Estimate, Engagement>;

  void operator()(LocalWorld& world, Entity self, Launcher& launcher,
                  const Kinematics* kinematics, Step step) {
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
    model::Displacement aim = estimate->position - kinematics->position;
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
struct GuideInterceptors final
    : System<const Interceptor, const Kinematics, Control, Target> {
  using LocalWorld = WorldAccess<GuideInterceptors>;
  using SequenceAfterSystemList = SystemList<LaunchInterceptors>;
  using AllowComponentList = TypeList<Kinematics, RedDrone>;

  void operator()(LocalWorld& world, Entity self,
                  const Interceptor& interceptor, const Kinematics* kinematics,
                  Control* control, Target* target_of, Step step) {
    if (!kinematics || !control || !target_of) {
      return;
    }
    if (step.time >= interceptor.expires_at) {
      auto destroyed = world.destroy(self).build();
      DECLARE_UNUSED(destroyed);
      return;
    }
    const Kinematics* target =
        world.maybe_component_of<Kinematics>(target_of->entity);
    if (!target) {
      target = retarget(interceptor, *kinematics, *target_of, world);
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
  static const Kinematics* retarget(const Interceptor& interceptor,
                                    const Kinematics& kinematics,
                                    Target& target, LocalWorld& world) {
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
struct SteerRedDrones final
    : System<const RedDrone, const Kinematics, Control, const Target> {
  using LocalWorld = WorldAccess<SteerRedDrones>;
  using AllowComponentList = TypeList<Kinematics>;

  void operator()(LocalWorld& world, Entity, const RedDrone& drone,
                  const Kinematics* kinematics, Control* control,
                  const Target* target_of) {
    if (!kinematics || !control || !target_of) {
      return;
    }
    constexpr Rate RESPONSE = 1.0 * model::per_second;
    const Kinematics* target =
        world.maybe_component_of<Kinematics>(target_of->entity);
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
struct TriggerWarheads final
    : System<const Warhead, const Kinematics, const Target> {
  using LocalWorld = WorldAccess<TriggerWarheads>;
  using SequenceAfterSystemList = SystemList<model::Integrate>;
  using AllowComponentList = TypeList<Kinematics>;

  void operator()(LocalWorld& world, Entity self, const Warhead& warhead,
                  const Kinematics* kinematics, const Target* target) {
    const Kinematics* target_kinematics =
        target ? world.maybe_component_of<Kinematics>(target->entity) : nullptr;
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

  // Collects this step's blasts, once, so each victim reads a short array
  // instead of walking the Blast store. Most steps have none, and then there
  // is nothing to apply.
  bool prepare(LocalWorld& world) {
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

  void operator()(LocalWorld& world, Entity self, Health& health,
                  const Kinematics* kinematics) {
    if (!kinematics) {
      return;
    }
    for (const Burst& burst : blasts_) {
      if (distance(*kinematics, *burst.center) <= burst.radius) {
        health.points -= burst.damage;
      }
    }
    if (health.points <= 0.0) {
      auto destroyed = world.destroy(self).build();
      DECLARE_UNUSED(destroyed);
    }
  }

 private:
  struct Burst final {
    const Kinematics* center = nullptr;  // Valid until the next sync point.
    Length radius = 0.0 * model::meter;
    double damage = 0.0;
  };
  std::vector<Burst> blasts_;
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

using Schedule = SystemList<Sensing, Engaging, GuideInterceptors,
                            SteerRedDrones, model::Motion, Blasts>;
using Scheduler = framework::Scheduler<World, Schedule>;

}  // namespace simon::missile
