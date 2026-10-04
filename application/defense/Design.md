# defense design

Red drones fly toward a protected asset. Blue radars track them, blue launchers
fire interceptors, and the run ends when red is defeated or the asset is
destroyed. `bazel run //application/defense -- <seed>` runs one scenario as
fast as possible and prints the outcome.

Components (`application/defense/simulation_components.hpp`):

| Component | Holds |
|---|---|
| `Kinematics`, `Control` | From `model/`: position and velocity, and the commanded acceleration |
| `Health` | Hit points |
| `Warhead` | Fuse distance, blast radius, damage |
| `Blast` | Radius, damage, the warhead's Name; lives for one step |
| `Target` | What a red drone (the asset) or an interceptor (a drone) flies at |
| `RedDrone` | Cruise speed, agility |
| `Tracked` | Marks a red drone that has a track, and names the track |
| `Asset` | Marks the protected asset |
| `Radar` | Range, a `RateGate` for the scan, whether it scanned this step |
| `Track` | Target entity and when a radar last saw it |
| `Estimate` | A track's estimated position and velocity |
| `Engagement` | The launcher engaging a track, if any, and until when |
| `Launcher` | Range, inventory, reload time, ready time, this step's proposal |
| `WeaponsHold` | Marks a launcher an operator has held; launchers allow it |
| `Interceptor` | Navigation gain, speed, agility, seeker range, flight time |

Each component has an archetype in `defense::archetype` (asset, radar,
launcher, red drone, interceptor, track, blast).

Tracks are entities, with `Track`, `Estimate` and `Engagement` required by
their archetype. A drone creates its own track when a scanning radar covers it,
tracks update themselves from the radars that scanned their target, launchers
engage them, and later they are what an interop layer would publish.

Interceptors and red drones both carry a `Warhead`. An interceptor reaching its
target and a drone reaching the asset are the same event: a blast, applied to
every `Health` within its radius.

Schedule (`application/defense/simulation_systems.hpp`):

| System | Does |
|---|---|
| `ScanRadars` | Each radar decides whether its scan fires this step |
| `DetectDrones` | Each untracked red drone that a scanning radar covers creates its track and marks itself `Tracked`. Skips steps without a scan. |
| `UpdateTracks` | Each track updates its own estimate from a radar that scanned its target. Radars are perfect for now. Skips steps without a scan. |
| `DropStaleTracks` | Destroys tracks whose target is gone or unseen for 5 s, and unmarks a surviving target |
| `ProposeEngagements` | Each ready launcher with inventory and no `WeaponsHold` proposes the nearest unengaged track in range |
| `ResolveEngagements` | Each unengaged track accepts the nearest launcher that proposed it, for 30 s |
| `LaunchInterceptors` | Launchers whose proposal was accepted build an interceptor under themselves, aimed at the track |
| `GuideInterceptors` | Proportional navigation plus speed hold into `Control`. Retargets the nearest red drone within seeker range when the target is gone; self-destructs when there is none or its flight time is up. |
| `SteerRedDrones` | Steers drones at their target at cruise speed into `Control` |
| `Motion` | Integrates `Control` into `Kinematics` |
| `TriggerWarheads` | A warhead within fuse distance of its target creates a Blast and destroys itself |
| `ApplyBlasts` | Every `Health` inside a blast takes its damage, and is destroyed at zero |
| `ExpireBlasts` | Destroys every blast; they live for one step |

Operator commands (`application/defense/simulation.hpp`) are query forms. Each
selects what it applies to, changes all of it or none, and returns how many
entities it affected:

```cpp
// Holds every launcher in a sector, so none engages until freed.
world->change()
    .each<archetype::Launcher>()
    .within(Kinematics{.position = sector.center}, sector.radius)
    .lacking<WeaponsHold>()  // Already held, or held by a pending command.
    .attach(WeaponsHold{})
    .build();
```

`free_weapons` detaches `WeaponsHold` from the held launchers in a sector,
except those inside sectors it is told to keep held, and
`destruct_interceptors` destroys every interceptor in flight in one. A hold is
a component, so holding a launcher is a structural change, and
`ProposeEngagements` reads it as an optional sibling. `Simulation` forwards
each command to its world. Like every builder, a command applies at the next
sync point.

A scenario can order weapons holds ahead of time (`Scenario::holds`), which
exercises the event queue:

```cpp
scenario.holds = {TimedHold{.sector = {.radius = 1000.0 * meter},
                            .from = TimePoint{20s}, .lasting = 30s}};
```

`configure` starts a timer for each hold. When it fires, the simulation holds
the sector and starts a second timer for `lasting`. That timer publishes
`WeaponsHoldExpired`, and the simulation's subscriber frees the sector, except
where another hold is still in force, so overlapping holds end with the last of
them. `Simulation::step` delivers the events due by the step's time before it
runs the schedule, and `Simulation::events()` lets anyone else subscribe.

Launchers under another hold are never freed, rather than freed and held again
in the same batch, so nothing churns through the `WeaponsHold` store.

Decisions made while building it:

- **The outcome is decided by the simulation, not a system.** Systems cannot
  stop a run, so `Simulation::step` checks the world after each step: red wins
  when the asset is gone, blue when no red drones remain.
- **No `Team` component yet.** Red is already expressed by `RedDrone`, and
  nothing needs a team separately.
- **Engagements expire.** A track stays engaged for about an interceptor's
  flight time, so a missed intercept frees it to be engaged again.
- **Retargeting policy:** an interceptor whose target is gone takes the nearest
  red drone within its seeker range, and otherwise self-destructs.
- **Reload is a ready time, not a `RateGate`,** because it means "not before
  time T", not periodic work.
- **Randomness is `model::Random`,** which converts `std::mt19937_64`'s raw
  bits itself: the engine's output is fixed by the standard, but the standard
  distributions are not, so this is the same on every platform.
- **Spatial queries use the world's index.** `GuideInterceptors` retargets with
  `nearest()`. Detection and track updates ask an index of the radars that
  scanned this step (`ScanningRadars`).
- **Tracks are not in the world's spatial index.** A track's position is blue's
  estimate. Giving tracks a `Kinematics` would have them coast between radar
  updates, but `UpdateTracks` would then write track `Kinematics` while reading
  its target's, which the rule against writing and reading one component
  forbids. `ProposeEngagements` owns a `SpatialIndex` over the estimates
  instead, rebuilt on the first query of each step.
- **`UpdateTracks` indexes the radars that scanned,** in `prepare`, so a track
  checks only the radars near its target.
- **Radars scan together unless a site asks otherwise.** `SiteBuilder::
  scanning_in_turn()` (`Scenario::radars_in_turn`, `defense_benchmark
  --in-turn`) staggers each site's radars over the scan period. At 100,000
  drones over 500 steps it lowers the slowest step from 137 to 105 ms, but
  raises the average from 2.21 to 2.89 ms. Every step with a scan walks every
  track and every untracked drone, and in turn there are ten such steps per
  period instead of one: `UpdateTracks` goes from 0.19 to 0.90 ms per step.
  The slowest step barely moves because it is the first scan, when every
  drone creates its track, and each site's first radar still covers most of
  them. Marking the covered tracks in `prepare`, from the scanning radars'
  side, only brought `UpdateTracks` back to 0.76 ms, so it was not kept.
- **Systems name their projected world** with a member alias,
  `using SystemWorld = ProjectedWorld<ThisSystem>;`, so builder
  calls with explicit template arguments, such as `detach<Tracked>()`, need no
  `template` keyword. It is `SystemWorld`, not `LocalWorld`, because "local"
  already means the local Cartesian frame and, across processes, what this
  process owns as against replicas.

The test runs whole scenarios under `BatchDriver` (blue wins by default, red
wins without launchers or with too few interceptors, and the same seed repeats
exactly) and each rule on a small world (one track per drone however many radars
see it, one launch per contested track, reload, retargeting, self-destruct,
blast damage).

`bazel run //application/defense:viewer -- <seed>` watches a scenario under
`RealTimeDriver`. An ImPlot map shows the asset, radars and launchers with
their coverage, red drones, interceptors, tracks at their estimated positions,
and explosions. A side panel shows time, counts, the asset's health and the
outcome, with pause (also Space), a speed slider and restart with a seed.
Blasts last a single step, so the viewer never sees one; it draws an
explosion wherever a drone or interceptor disappears. That is presentation
only and changes nothing in the simulation.

## Following the framework's practices

defense follows the practices in
[Using the framework well](../../framework/Design.md#using-the-framework-well),
and measures what they are worth against a layout that does not:

- **`Kinematics` is position and velocity, 48 bytes.** Orientation is a
  separate `Orientation` component, which nothing in defense needs, and the
  commanded acceleration is in `Control`. With both inside, it is 112 bytes.
- **`Track` is three components its archetype requires:** `Track` (target and
  when it was last seen, 16 bytes), `Estimate` (position and velocity, 48) and
  `Engagement` (launcher and until when, 16). `DropStaleTracks` reads only
  `Track`, `ResolveEngagements` only `Engagement` and, for proposed tracks,
  `Estimate`.
- **Drones detect themselves.** `ScanRadars` only decides which radars scan.
  `DetectDrones` lets each untracked drone ask an index of the radars that
  scanned, and create its own track; `UpdateTracks` uses the same index.
- **Idle steps do nothing.** `DetectDrones`, `UpdateTracks`,
  `ResolveEngagements` and `ApplyBlasts` skip their loops when no radar scanned,
  no launcher proposed or nothing exploded.

At 100,000 drones (ms per step), against fat components and radars that create
tracks:

| | Without them | With them |
|---|---:|---:|
| Idle, 500 steps | 6.33 | 2.87 |
| Idle, 200 steps | 9.49 | 3.01 |
| 4 contending threads, 200 steps | 42.4 | 5.53 |

The step is 2.2 times faster idle and 7.7 times faster under contention, where
it slows 1.8 times rather than 4.5. The cost per entity is flat from 1,000 to
100,000 drones (12.9 to 13.4 ns per entity-step), and the outcomes are the
same.

Drones creating their own tracks also means each drone has at most one. When
radars create tracks, ten radars that cover a drone on one scan each create a
track before marking it `Tracked`; the nine that lose destroy theirs, but a
destroyed entity's slot is freed only at the next sync. At ten sites the first
scan then runs out of entity capacity and tracks 5,000 of 10,000 drones.

`defense_benchmark` prints `framework::bytes_per_entity_v`, the bytes a
system's loop can read per entity, beside each system's time:

| System | B/entity | System | B/entity |
|---|---:|---|---:|
| `SteerRedDrones` | 104 | `TriggerWarheads` | 88 |
| `Integrate` | 80 | `DropStaleTracks` | 24 |
| `DetectDrones` | 88 | `ResolveEngagements` | 72 |

A `Target` component that both archetypes require holds a warhead's target,
so `TriggerWarheads` names `Warhead`, `Kinematics` and `Target`: 88 bytes,
against 152 when it names `Interceptor` (48 bytes) and `RedDrone` (24) for
their targets. The time is the same within noise (0.57 to 0.60 ms per step at
100,000 drones): the runner passes a null `Interceptor` to drones without
reading it, and most of the time is each drone looking up its target's
`Kinematics`. The gain is a simpler system and one copy of the target.

## Roadmap

1. **Headless (done).** The components and schedule above, with a
   `BatchDriver` test that checks a deterministic outcome for a fixed seed.
2. **Viewer (done).** An ImGui and ImPlot view under `RealTimeDriver`.
