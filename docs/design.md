# simon design

simon is a toolkit for building entity-component simulations, plus the
applications that drive its development.

This document records the architecture, the decisions behind it, and the
roadmap. Update it when a decision changes.

## Goals

In priority order:

1. Architectural simplicity.
2. A robust toolkit for any simulation.
3. A high-performance entity-component system.
4. A cool demo.

When goals conflict, the higher one wins.

## How we work

We build applications and keep the roadmap in mind. We do not build the
framework first and look for applications afterwards. A piece of the toolkit
exists because an application needed it, and it moves into the toolkit once a
second use appears or the roadmap clearly calls for it.

We prefer existing, modern libraries for anything outside the core. The core
(entities, stores, systems, schedules and drivers) is ours, because its layout
and handle rules are the point of the project.

## Terminology

| Term | Meaning |
|---|---|
| Entity | An object with a name/identity suitable for quick comparison, canonical lookup and local aliasing. An `Entity` value is the local alias; its `Name` is the canonical identity. |
| Component | A type with a name/identity suitable for debugging. |
| Entity-component | A single object of a component type, associated with a single entity. |

## Architecture

The architecture is idiomatic ECS with no exceptions:

| Concept | Rule |
|---|---|
| Entity | An identity and nothing else: `{index, generation}`. It holds no data and no pointers. |
| Component | Plain data. No behaviour, no virtual functions, no base class required. Each component type has a debug name. |
| Store | Exactly one per component. It is the array of that component's entity-components. |
| System | Every non-trivial function is a system. A system iterates component arrays through stores. |
| Builder | The user-facing language for asking the simulator to do something specific. A builder turns a request into commands. |
| Command | A typed, low-level structural change: create, add, remove, destroy. Commands are applied to stores. |
| Schedule | A type listing systems in execution order. Schedules compose. |
| World | The entity database. A factory of builders (the only way to write) and a query interface (the only way to read). Configured by a `Spatial` model. |
| Driver | Owns time and steps the world's schedule. |

Two pairs factor the details out of the parts people write:

```
System  ──uses──▶  Store     Store hides indices, layout and lookup from systems.
Builder ──emits──▶ Command   Command hides store mutation and ordering from builders.
```

Anything that does not fit one of these rows needs a design change recorded
here before it goes in.

```
          Driver ── advance_to(T) ──▶ Simulation::step(time, dt)
                                            │
                                            ▼
                                  Schedule<SystemA, SystemB, ...>
                                            │  each system iterates
                                            ▼
     World ── entity table ── Store<Kinematics> ── Store<Radar> ── ...
       │                             ▲
       └── command buffer ───────────┘  applied at sync points
```

### Entities

An `Entity` is 8 bytes and trivially copyable:

```cpp
struct Entity {
  std::uint32_t index;
  std::uint32_t generation;
};
```

The world keeps a table of generations indexed by `index`. Destroying an
entity bumps its generation and returns the index to a free list. Any `Entity`
value that still names the old generation is stale, and every lookup through
it fails the generation check.

The entity table has a fixed capacity chosen when the world is created. It
never reallocates.

Free indices are reused first-in first-out, so a destroyed index waits as long
as possible before a new entity takes it.

### Components

A component is a plain struct:

```cpp
struct Kinematics {
  Vec3 position = Vec3::Zero();
  Quat orientation = Quat::Identity();
  Vec3 velocity = Vec3::Zero();
  Vec3 acceleration = Vec3::Zero();
};
```

`Kinematics` is the toolkit's default `Spatial` model (see
[The world](#the-world)).

Every field has a default initializer. Eigen types in particular do not
initialize themselves.

A component's debug name comes from its type (`lib::to_type_string<T>()`), so
logs and debuggers can say which component an entity-component belongs to.

State and the commands that change it are separate components. Guidance writes
a `Control`, and `Integrate` turns `Control` into motion:

```cpp
struct Control {
  Vec3 acceleration = Vec3::Zero();
};
```

A component that depends on another entity stores that `Entity`:

```cpp
struct Interceptor {
  Entity target;
  double navigation_gain = 4.0;
  double max_acceleration = 80.0;
};
```

### Stores

Each component has one `Store<T>`, which holds all of its entity-components.
Stores keep that array dense so that large simulations fit in cache. On the development machine (Zen
3, 512 KiB L2 per core, 32 MiB L3 per chiplet), 5,000 agents with a 72-byte
`Kinematics` take 360 KiB. That fits in one core's L2. With 50% holes the same
store takes 720 KiB and spills into L3.

The leading design keeps holes out of the component array by indexing through
the entity:

```
Store<Kinematics>
  index[entity.index] ─▶ {dense position, generation}    8 bytes per entity slot
  owner[dense]        ─▶ Entity                           dense
  data[dense]         ─▶ Kinematics                       dense; systems iterate this
```

- **Iterate.** Walk `data` (and `owner` when the entity is needed) from start
  to end. The index is never touched.
- **Add.** Append to `data` and `owner`, and write the index entry.
- **Remove.** Move the last element into the gap, fix the moved element's
  index entry, and clear the removed one. The array stays dense after every
  removal.
- **Look up.** `index[e.index]` checks the generation, then reads `data`. The
  index for 5,000 entities is 40 KiB and stays hot.
- **Capacity.** Every array is allocated once, at a capacity chosen when the
  world is created. Stores never reallocate, so references into them stay
  valid until the element is removed or moved by a removal.

A hole now costs 8 bytes in the index, instead of a whole component in the
array.

This design is still to be confirmed by measurement. The alternative is a
stable-slot store, where components never move, destroyed slots are reused by
new components, and a handle dereference is one load. Step 1 of the roadmap
benchmarks both. Systems use the same store interface either way (add, remove,
look up, iterate), so the choice stays inside `Store`.

### Handles and references

A handle has three duties, carried over from the older simulator:

1. Hide indices and addresses from the component programmer.
2. Never allow a memory error.
3. Never fail silently.

`Entity` is the handle. A system reads another entity's component through the
store:

```cpp
const Kinematics* target = kinematics.try_get(interceptor.target);  // null if gone
const Kinematics& target = kinematics.get(interceptor.target);      // contract check if gone
```

`try_get` is the explicit path for "this may have disappeared". `get` fails a
contract check on a stale entity, so misuse stops loudly. Neither can read
freed memory, because store memory lives as long as the world.

Raw pointers or references returned by a lookup are only valid until the next
sync point. Systems must not keep them across steps.

Nothing registers handles or patches them. The older simulator's self-patching
handle met the same three duties, but every copy made a virtual call and a hash
insert to follow objects that reallocation had moved. Fixed capacity removes
the reallocation, and the generation check replaces the patching.

### Names, identities and queries

Many entities and components come and go during a run, and every one of them
must be debuggable. Names and identities exist for that.

**A name** is a magic number compared in a vaguely type-safe way. `Name<Tag>`
wraps an integer, so an entity name cannot be compared with, say, an event
name. Comparing names is an integer compare.

**An identity** is what issues names. The older simulator drew identities from
static counters, so a name said nothing about where its entity came from, and
the numbers depended on construction order. In simon an identity belongs to
the builder that created the entity. Each builder has an identity scope, and
scopes nest:

```
scenario
└── blue/launcher/2                 created by the scenario's launcher builder
    └── blue/launcher/2/interceptor/7   created by launcher 2's interceptor builder
```

- **Deterministic.** The same scenario and seed produce the same names, in any
  run and on any machine. Replays and logs line up.
- **Traceable.** A name leads back to the builder that made the entity, and
  that builder's own name leads to its creator.
- **Cheap on the hot path.** The name is an integer derived from the scope
  path. The readable path is kept in a cold-path table in the world, for logs
  and debuggers.
- **Portable.** Two processes that load the same scenario derive the same
  names. That makes names usable as the identity across processes (see
  [Distribution](#distribution)).

A `Name` component holds an entity's name, so a name is ordinary component
data. The world keeps an index from name to entity:

```cpp
std::optional<Entity> missile = world.find(names.path("blue/launcher/2/interceptor/7"));
```

**A component query** finds every entity with a set of components, optionally
filtered:

```cpp
world.query<const Kinematics, const Team>()
    .where([](const Kinematics&, const Team& team) { return team == Team::Red; })
    .for_each([&](Entity e, const Kinematics& k, const Team&) { ... });
```

A query iterates the smallest store in the set and looks up the rest.

The older simulator also let code ask for a named thing before it existed and
get a callback when it appeared. That needed the `DependencyTracker`, about
750 lines of string-keyed tables with callbacks that could not be unregistered.
It existed because scenarios were parsed in arbitrary order. simon replaces it
with two simpler rules:

- **Scenario loading is two-phase.** Create every named entity first, then
  resolve references by name.
- **At runtime, look names up when they are used.** Code that must react to a
  new entity subscribes to an `EntityCreated` event.

### Builders and commands

Builders are the user-facing language of the toolkit. A builder is a type-safe,
friendly way to ask the simulator for something specific:

```cpp
build.ego().with_missiles(4).at(site).build();
build.destroy().team(Team::Red).within(radius, of(asset)).build();
build.connect().cosimulator("red-side").lockstep().lookahead(10ms).build();
```

A builder hides important type details without losing them. It may convert or
erase types internally (a query, a set of archetypes, a transport), and the
caller never sees those details. What leaves the builder is always a list of
typed commands, so nothing is lost.

A builder is a grammar. An utterance can only be validated once it is
complete, so a builder accumulates the utterance, validates it at its terminal
call, and only then emits commands:

```
build.ego().with_missiles(4).at(site).build()
└──────────── utterance ─────────────┘   │
                                         ├─ validate the complete utterance
                                         └─ emit commands:
                                              create(e) add(Kinematics) add(Launcher)
                                              name(e, "blue/ego/1") ...
```

Builders and commands are not isomorphic. One utterance can emit many
commands, and "destroy all red within a radius" emits a number of commands
that depends on the world at the time.

Rules for builders:

- **Validate the whole utterance before emitting anything.** Structure is
  checked at compile time where the grammar allows it: each step can return a
  builder of a different type, so an incomplete or contradictory utterance
  fails when `.build()` compiles. Data-dependent rules (a name must exist, a
  radius must be positive) are checked at runtime.
- **The terminal call is explicit, `[[nodiscard]]`, and returns a status.** A
  builder never does work in its destructor, and a failure is never only
  logged.
- **One builder type per request.** A builder does not switch between modes.
- **Builders issue identities.** Entities a builder creates are named inside
  its identity scope (see above).
- **Creating reserves the entity immediately,** so the returned `Entity` can be
  stored before the commands are applied.

Commands are the low-level vocabulary: create, add, remove, destroy, name.
They are typed, go into the world's command buffer, and are applied at sync
points in the order they were recorded. The stores never change shape while a
system iterates them.

Sync points sit between systems. A system's commands are applied before the
next system runs, so later systems in the same step see the change.

Systems use builders too. `AssignLaunchers` asks its launcher's interceptor
builder for a new interceptor, and the builder emits the commands.

### Systems

A system declares its structure as a type. `System<Drive, Optional...>` is a
variadic template: the first component drives the loop, and every following
component is optional and belongs to the same entity.

```cpp
template <typename Drive, typename... Optional>
struct System {
  using DriveType = Drive;
  using OptionalTypes = TypeList<Optional...>;
};

struct GuideInterceptors : System<const Interceptor, const Kinematics, Control> {
  using After = Systems<UpdateTracks>;
  using Lookups = Stores<Kinematics>;   // other entities, always read-only

  void operator()(Entity self, const Interceptor& interceptor,
                  const Kinematics* kinematics, Control* control,
                  Context<GuideInterceptors>& context) const {
    if (!kinematics || !control) return;
    const Kinematics* target = context.lookup<Kinematics>(interceptor.target);
    if (!target) { ... }
    control->acceleration = model::proportional_navigation(*kinematics, *target, interceptor);
  }
};
```

The framework reads only the `System<...>` arguments. It walks the driving
store and passes each entity's optional entity-components:

```cpp
auto& drive = world.store<typename S::DriveType>();   // const Store<Interceptor>& when Drive is const
for (std::size_t i = 0; i < drive.size(); ++i) {
  Entity e = drive.owner(i);
  system(e, drive.data(i), world.store<Optional>().try_get(e)..., context);
}
```

Given a system driven by `A` with an optional `B`, and entities `x`, `y` and
`z` holding `(a)`, `(b)` and `(a, b)`:

| Entity | Holds | The system |
|---|---|---|
| `x` | `(a)` | runs with `b == nullptr` |
| `y` | `(b)` | does not run |
| `z` | `(a, b)` | runs with its `b` |

Rules:

- **The driving component comes in by reference** and runs the loop, in the
  driving store's dense order.
- **Every following component comes in by pointer** and is null when the entity
  lacks it. A system that needs it checks explicitly (`if (!b) return;`), so a
  missing component is visible in the code, never silently skipped.
- **Constness is declared in the `System` type, nowhere else.** Components are
  written plainly (`System<Interceptor, Control>`). To read a component without
  writing it, declare it `const` (`System<const Interceptor, Control>`). The
  framework then takes a `const Store<T>*` for it, and a const store only hands
  out `const T&` and `const T*`. The call operator must accept what the
  declaration implies; a mismatch is a compile error.
- **Other entities are reached through declared lookups.** `using Lookups =
  Stores<...>;` lists the stores a system may read by entity, and
  `Context<S>::lookup<T>(e)` returns `const T*`. Lookups are always read-only,
  and asking for an undeclared store does not compile. (This form is proposed;
  see [Open decisions](#open-decisions).)
- **`Context<S>`** also carries the `Step` (see [Time](#time-and-drivers)) and
  the builders the system may use.
- **Optional stages** (`prepare` before the main loop, `resolve` after it)
  are detected at compile time and cost nothing when absent. The older
  simulator did the same with stage tags.

Because the structure is a type, it composes. A system can be generic
(`template <typename Body> struct Integrate : System<Kinematics, const Control,
const Body>`), wrapped (`EveryN<10, S>`), or generated (an interop layer could
declare `Replicate<T> : System<T, const Replica<T>>` for every component it
publishes). A later builder built on expression templates could assemble
`System<...>` types the same way.

**Cost.** Each optional component costs one probe of its store's index per row
(an array read and a generation compare). That is the same lookup the older
simulator made by hand through the entity, minus its hash map and
`dynamic_cast`. The declared type adds no cost and buys visibility: the
compiler can enforce the write rule, check ordering, and print what each system
reads and writes. If a hot pair measures too slow, the world can keep an owned
group for it (entities holding both packed at the front of both stores, in the
same order), which removes the probes without changing system code.

**Stages or separate systems.** Every non-trivial function is a system, but a
system may do several passes:

- Use stages (or a state variable with a switch) when the passes are one
  concern over the same driving component.
- Use separate systems when a sync point is needed between the passes, when
  the driving component differs, or when a pass is worth testing or reusing
  alone.

Detonation needs three systems for those reasons. Blast entities must exist
before victims look them up, which needs a sync point, and the victims are
driven by `Health` while the warheads are driven by `Warhead`.

The framework derives each system's reads and writes from its `System` type
and its `Lookups`. The compiler enforces them: a system declared with
`const Radar` cannot write radar state.

The physics inside a system should be a free function in `model/`
(`model::proportional_navigation` above), testable without a world.

### Relations between entities

ECS favours batch processing: a system walks arrays and treats every row alike.
Simulations cannot escape relations between entities, though. A warhead
detonates and must damage everything within its radius. An interceptor chases a
target. Two launchers want the same track. ECS lets users choose how to handle
relations, which is a strength, so simon needs a clear best practice and a
design that makes misuse hard.

**The rule: write only your own entity.** A system writes only the
entity-components of the entity it is running for: the driving one and the
optional ones. Everything it reaches through a `Store` lookup is read-only. To affect another entity, a system either uses a builder (create,
edit, destroy) or leaves data that the other entity's own system reads.

When the natural loop runs the wrong way, invert it. Make the side being
written the batch, and look up the side being read.

#### Example: detonation

```
TriggerWarheads   driven by Warhead, reads Kinematics*   writes its own warhead state
   │  on trigger:  build.blast().at(position).radius(r).yield(y).source(self).build()
   │               build.destroy(self).build()
   ▼  sync point: this step's Blast entities exist
ApplyBlasts       driven by Health, reads Kinematics*    victims are the batch; each writes only itself
   │  asks the world for blasts within range, read-only
   │  if destroyed: build.destroy(self).build()
   ▼
ExpireBlasts      driven by Blast                        destroys blasts older than one step
```

- **The blast is an entity** with `Blast` and `Kinematics` entity-components,
  named inside the warhead's builder scope. It can be debugged and drawn, and
  it has the same shape as a DIS Detonation PDU or an HLA interaction. If a
  victim lives in another process, the blast is the only thing that could reach
  it.
- **Victims write only themselves.** Two blasts hitting one victim, or a victim
  destroyed partway through, cannot produce order-dependent results. Each
  victim sums its own damage.
- **"Everything within the radius"** is a world query. The world's spatial
  index is updated once per step after `Motion` and read-only for the rest of
  the step.
- **Victims that must react** set state on themselves (for example `Damaged`)
  or publish an event if something rare needs to know.

#### Patterns, simplest first

| Relation | Pattern | Example |
|---|---|---|
| One-to-one reference | Store an `Entity`; look it up read-only | An interceptor reads its target's `Kinematics` |
| One-to-many by space or predicate | A shared per-step index or a query, read-only | Blast radius, radar coverage |
| Affecting another entity | Emit an entity or command; the target's own system applies it | Blasts, new tracks |
| Conflicting claims | Propose, then resolve | Each launcher writes a proposal on itself; `ResolveEngagements` iterates tracks and picks one |
| Many-to-many | A relation entity holding both `Entity` values | A radar observing a track |

#### Guarding against misuse

- **Lookups are always read-only.** There is no way to declare a writable
  lookup, so "loop over blasts and write each victim" cannot be written. The
  error message points at the inversion pattern.
- **A system cannot both write `T` and look it up.** Declaring a non-const `T`
  in the `System` type and `T` in `Lookups` fails to compile. Reading an array
  partway through writing it gives results that depend on iteration order.
  This is why guidance writes `Control` and reads `Kinematics`.
- **Structural changes to other entities go through builders.** Their commands
  apply at the next sync point, so nothing is destroyed partway through an
  iteration.
- **Pointers from lookups are valid only until the next sync point.** Debug
  builds can wrap them to catch one kept longer.
- **The event queue is for rare events.** Per-step traffic belongs in entities
  and components.
- **The world answers spatial queries,** so nobody writes an O(N²) scan for
  lack of an index.

The rule is the same as the authority rule in [Distribution](#distribution):
only the owner writes. Code written this way is already shaped to run across
processes.

### Schedules

A schedule is a type. Its template argument order is the execution order, and
the compiler holds it fixed. This is what makes a step deterministic and
replayable.

Running a schedule is a fold expression over its systems. There is no type
erasure and no virtual call:

```cpp
template <typename... Ss>
void run(Systems<Ss...>, World& world, Step step) {
  (run_system<Ss>(world, step), ...);   // each call is a direct, inlinable instantiation
}
```

Each system is listed once, in the schedule type.

Schedules compose:

```cpp
// Toolkit pieces.
using Motion  = Systems<Integrate, UpdateSpatialIndex>;
using Sensing = Systems<ScanRadars, UpdateTracks, DropStaleTracks>;
using Blasts  = Systems<TriggerWarheads, ApplyBlasts, ExpireBlasts>;

// Application schedules.
using HelloSystems   = Systems<ApplyWind, Motion, DetectCollisions>;
using DefenseSystems = Systems<Sensing, ProposeEngagements, ResolveEngagements,
                               LaunchInterceptors, GuideInterceptors, SteerRedDrones,
                               Motion, Blasts, CheckOutcome>;
```

- **Nested schedules flatten** at compile time into one list.
- **Ordering constraints are checked** on the flattened list. A system that
  declares `using After = Systems<X>;` fails the build if it is scheduled
  before `X`.
- **Rate adapters are schedules too.** For example, `EveryN<10, Sensing>` runs
  a group every tenth step and is still a type the compiler can inline.
- **The schedule can be printed.** A test or startup flag prints the flattened
  list with each system's reads, writes and constraints, generated from the
  type. This recovers the discoverability a type list otherwise costs.
- **Sub-schedules can be tested alone** against a small world.

The older simulator also fixed order by template arguments, but kept the
dependencies that justified the order in comments. simon keeps the type list
and makes the dependencies part of the type.

### The world

A world is the entity database. It has two halves:

1. **A factory of builders.** Builders are the only way to write. A builder
   validates its utterance and emits commands, and the world applies them at
   the next sync point, so the database is populated when the commands are
   evaluated and no store changes shape while a system iterates it.
2. **A query interface over the result.** Queries are the only way to read:
   by name, by component, by space and by relation. A system's per-entity loop
   is the most basic query ("every `A`, with optional `B`"), compiled down to
   array walks.

```
          write                                       read
build.ego()...build() ─▶ commands ─▶ World ─▶ by name       world.find(name)
build.destroy()...     (applied at            by component  System<A, B...>, world.query<...>()
build.connect()...      sync points)          by space      world.within(center, radius), world.nearest(p)
                                              by relation   world.parent(e), world.children(e)
```

| Database | World |
|---|---|
| Primary key | `Entity` (local alias) and `Name` (canonical identity) |
| Tables | Stores, one per component |
| Indexes | The name index, the spatial index, the transform hierarchy |
| Transactions | Command buffers, applied at sync points in recorded order |
| Business logic | Systems |

A world holds every entity-component its builders created, the mappings from
names to entities, and any relationships between entities. Relationships are
ordinary data: `Entity` fields in components, or relation entities (see
[Relations between entities](#relations-between-entities)).

#### Configuration

A world is configured at compile time by a `Spatial` model and its component
list:

```cpp
template <typename S>
concept Spatial = requires(const S& a, const S& b) {
  { distance(a, b) } -> std::convertible_to<double>;
  { pose(a) } -> std::convertible_to<Pose>;
};

template <Spatial S, typename... Components>
class World;

using DefenseWorld = World<Kinematics,
                           Name, Team, Control, Health, Warhead, Blast,
                           RedDrone, Asset, Radar, Track, Launcher, Interceptor>;
```

- **`Spatial` is the world's only configuration concept.** It needs a distance
  and a pose. The toolkit ships `Kinematics`, a local Cartesian 3D model, as
  the default, so interop layers share one notion of position unless an
  application opts out. Other models (geodetic for DIS, 2D, a grid, a network
  where distance is hop count) fit without changing the core.
- **The component list is closed per build.** `world.store<T>()` resolves at
  compile time, nothing is type-erased, and the inliner sees every hot-path
  call. An application cannot add a component type at runtime, which none of
  our use cases need.

#### Indexes

The world owns two trees. Both are implementation details, exposed only as
queries:

- **The spatial index** answers `within()` and `nearest()` over every entity
  with the `Spatial` component. Its structure (uniform grid, BVH, k-d tree) is
  the world's choice, measured and swappable. Users never supply a tree type.
- **The transform hierarchy** answers `parent()` and `children()`, and composes
  poses for mounted entities such as a radar on a vehicle. It is built from
  `pose()` and a parent relation.

Indexes are updated at defined points, so every query within a step sees a
consistent snapshot:

- Created and destroyed entities enter and leave the indexes at sync points.
- Moved entities are re-indexed by a world-provided system,
  `UpdateSpatialIndex`, which the schedule places after `Motion`. The world
  maintains its indexes, and the schedule says when.

Entities replicated from another process enter the same indexes, so a spatial
query finds a red drone whether red is simulated locally or remotely.

#### Several worlds

A world is an object, so a process can hold several. One use stands out for
defense: the blue side's track picture as its own world, holding tracks built
from sensor reports and queried the same way as ground truth. It is a natural
home for perception error later.

An entity belongs to exactly one world. References across worlds use names,
the same rule as references across processes.

Only one world type, the Cartesian spatial world, is built until a second is
needed.

### Determinism

A run is reproducible from its scenario and seed. That requires:

- A fixed schedule order (the schedule type).
- A stable iteration order within a system. Dense stores change order only on
  removal, and removal happens only at sync points, so the order depends only
  on the history of commands.
- Commands applied in recorded order.
- Integer time (see below).
- No iteration over unordered containers on the hot path.
- Random numbers drawn from generators the simulation owns and seeds.

## Extensible edges

Large simulations connect to other input formats and co-simulators, so we
decide deliberately which edges are extensible and what polymorphism each uses.

The principle is templates inside the step, where code runs once per
entity-component, and runtime polymorphism at the edges, which are crossed once
per step or once at load. Builders sit between the two. They take type-erased
input from an edge, validate it, and emit typed commands.

| Edge | Crossed | Polymorphism | Why |
|---|---|---|---|
| Systems and schedules | Per entity-component | Static: `System<...>`, `Systems<...>` | Hot path; must inline |
| World configuration (`Spatial`, component list) | Compile time | Static: `World<S, ...>` | Hot path; closed per build |
| Input formats (scenario files, other schemas) | At load | Runtime: a reader interface feeding builders | New formats without recompiling the core |
| Builders | At load or at sync points | Static interface; may type-erase internally | User-facing grammar that emits typed commands |
| Co-simulator connections (scatter/gather, DIS, HLA) | Once per step | Runtime: a channel interface | Chosen by configuration; cost amortized over the step |
| Drivers | Once per step | Runtime: chosen from configuration | Outer loop; batch, real-time or lockstep picked at startup |
| Component serialization | At the edge | Compile-time traits per component, called through the runtime channel | Typed codecs, reachable through the type-erased edge |

## Time and drivers

### Time

Time is simulation time, never wall time. It is passed explicitly as a `Step`
to everything that needs it:

```cpp
struct Step {
  TimePoint time;   // start of this step
  Duration dt;      // length of this step
};
```

There is no global clock.

Time is counted in `int64` nanoseconds. Integer time is exact, which
determinism needs, and it removes a class of substep drift bugs. simon already
fixed one caused by accumulating `double` substeps. An `int64` of nanoseconds
covers about 292 years.

`lib::SimClock` currently counts `double` seconds and changes to `int64`
nanoseconds in step 2. Physics code converts a `Duration` to `double` seconds
at the point of use.

### Lifecycle

Every simulation follows one lifecycle, whatever drives it:

```
configure ─▶ initialize ─▶ step ─▶ step ─▶ ... ─▶ finalize
                             └── any step may return Stop
```

Each phase returns `Continue` or `Stop`. Errors are statuses and are never
used as control flow. (The older engine sometimes used `AbortedError` to mean
"exit the loop".) The lifecycle is a small enum. It needs no state machine
classes.

### Drivers

A simulation is anything that implements the lifecycle and
`step(Step) -> Continue | Stop`. A driver owns time and makes the simulation go.

Every driver uses one contract, `advance_to(T)`. The driver is told a target
time and substeps toward it at no more than the maximum `dt`, clamping the last
substep so it lands exactly on `T`. Drivers differ only in where `T` comes
from:

| Driver | Source of `T` | Use |
|---|---|---|
| `BatchDriver` | A fixed end time, as fast as possible | Tests, batch runs |
| `RealTimeDriver` | The wall clock, paced | The demo, DIS-style interop |
| `LockstepDriver` | An external peer grants the next time | Multiple processes, HLA-style time management |

The contract carries an optional lookahead, the promise that this simulation
will not produce events earlier than `now + lookahead`. HLA time management
needs it.

`BatchDriver` and `RealTimeDriver` come first. `LockstepDriver` waits until a
second process exists.

### Rate gates

A component that works at a lower rate than the step (a radar scanning at
10 Hz on a 100 Hz step) holds a `RateGate` value. Its system asks the gate
whether to run this step.

When it runs, the work receives the elapsed time since the gate last fired, not
the driver's `dt`. The catch-up policy (fire once and skip missed periods, or
fire once per missed period) is an explicit parameter. The older simulator's
`PeriodicStep` passed the driver's `dt`, which was wrong for any gated work.

### Events

The event queue carries rare, discrete happenings: a launch, an intercept, an
entity created. It is not for anything that happens every step, which belongs
in components.

Events are delivered in time order, ties in publish order, with the event's own
time. simon's current `EventQueue` already does this.

## Distribution

Nothing here is implemented yet. The core is designed so that these can be
added later without changing the simulation's interface.

### Future use cases

- **Red and blue on different computers.** Each side simulates its own
  entities and sees the other side's as replicas.
- **DIS (IEEE 1278).** Entity State PDUs broadcast over UDP, dead reckoning
  for remote entities, Fire and Detonation PDUs, wall-clock pacing.
- **HLA (IEEE 1516).** A runtime infrastructure (RTI), a federation object
  model (FOM) of object classes and interactions, per-attribute ownership, and
  logical time management with lookahead.

### What the core provides for them

| Need | Core design |
|---|---|
| Who may write a piece of state | **Authority per component.** Each component on an entity is either owned locally or a replica of remote state. Per-component authority maps to HLA attribute ownership. A component is roughly an HLA attribute group. |
| Where state crosses a boundary | **Scatter and gather at step boundaries only.** Gather publishes locally owned components that changed. Scatter applies remote updates to replicas. |
| Identity across processes | **Names.** `Entity` values mean nothing outside their process. Each interop layer maps its external identity (a DIS `{site, application, entity}` triple, an HLA object instance name) to our names. Neither format enters the core. |
| Time across processes | **`advance_to(T)` with lookahead.** It is the same shape as HLA's time advance request and grant. The barrier rule for peers is "advance to the minimum of the peers' times". |
| Remote entities between updates | **Dead reckoning as a system** that extrapolates replica components each step, using the DIS algorithms. |
| Discrete interactions | **Events** that carry the names or entities involved. They map to DIS Fire/Detonation PDUs and HLA interactions. |
| Coordinates | **A local Cartesian frame with a stated geodetic origin.** Conversion to geocentric coordinates (DIS) happens at the interop boundary. |

Transport, liveness heartbeats and any controller process live outside the
core. Candidate libraries when we get there are Open-DIS for DIS and OpenRTI or
Portico for HLA.

## Repository layout

```
simon/
  core/          Entity, Store, World, Spatial, builders, command buffer, System, schedules
  drive/         Step, lifecycle, drivers, RateGate, EventQueue
  model/         Reusable physics: kinematics, sensing, guidance (free functions)
  app/
    hello/       Two bouncing balls, the first application
    defense/     Red drones against blue radars and launchers
  docs/          This document
  2nd_party/lib  Shared core libraries (submodule)
```

`framework/`, `component/` and the top-level `simulation.hpp` are the current
prototype. They are retired once `app/hello` runs on the new core.

## Applications

### hello

Two balls under gravity and wind that stop when they collide. It is the
smallest complete use of the architecture: a handful of components, a motion
schedule, a collision system and a real-time driver.

### defense

Red drones fly toward a protected asset. Blue radars detect them, blue
launchers fire interceptors, and the run ends when red is defeated or the asset
is destroyed.

Components:

| Component | Holds |
|---|---|
| `Name`, `Team` | Identity and side |
| `Kinematics` | Position, velocity, acceleration |
| `Control` | Commanded acceleration, written by guidance and steering |
| `Health` | Hit points; damage taken this step |
| `Warhead` | Fuse radius, blast radius, yield |
| `Blast` | Radius, yield, source name; lives for one step |
| `RedDrone` | Goal position, cruise speed |
| `Asset` | Marks the protected asset |
| `Radar` | Range, field of view, `RateGate` for the scan |
| `Track` | Target entity, estimated position and velocity, last seen time, engaging launcher |
| `Launcher` | Inventory, reload `RateGate`, engagement range, current proposal |
| `Interceptor` | Target entity, navigation gain, acceleration limit |

Tracks are entities. Radars create them, tracks update themselves from what the
radars can see, launchers engage them, and later they are what an interop layer
would publish.

Interceptors and red drones both carry a `Warhead`. An interceptor reaching its
target and a drone reaching the asset are the same event: a blast, applied to
every `Health` within its radius.

Schedule:

| System | Does |
|---|---|
| `ScanRadars` | Creates a track for each red drone in coverage that has none |
| `UpdateTracks` | Each track updates its own estimate from the radars that can see its target |
| `DropStaleTracks` | Destroys tracks not seen recently or whose target is gone |
| `ProposeEngagements` | Each ready launcher proposes the best unengaged track in range |
| `ResolveEngagements` | Each track accepts one proposal |
| `LaunchInterceptors` | Launchers whose proposal was accepted build an interceptor |
| `GuideInterceptors` | Proportional navigation into `Control`; retargets or self-destructs when the target is gone |
| `SteerRedDrones` | Steers drones toward the asset into `Control` |
| `Motion` | Integrates `Control` into `Kinematics` and updates the world's spatial index |
| `Blasts` | Triggers warheads, applies blast damage, expires blasts |
| `CheckOutcome` | Returns `Stop` when red is defeated or the asset is destroyed |

This application exercises the parts of the toolkit that matter most:
references to entities that disappear, relations between entities, creation
and destruction in the middle of a run, rate gates, and thousands of agents.

## Libraries

| Need | Library |
|---|---|
| Linear algebra | Eigen (have) |
| Tests and benchmarks | Catch2, including its benchmarks (have) |
| UI | Dear ImGui (have), ImPlot for the top-down view |
| Window and input | SDL2 now; SDL3 when it is in the Bazel Central Registry |
| Profiling, later | Tracy |

C++26 reflection would remove some boilerplate, but GCC 16 supports it and
Clang 22 does not. We will revisit it when both compilers do.

## Roadmap

Each step ends with a working application and passing tests.

1. **Core ECS.** `Entity`, `Store`, `World`, systems, schedules, builders and
   commands, with tests. Benchmark the dense store against the stable-slot
   store at 1k, 10k and 100k entities with 0–75% churn, for iteration and
   random lookup, and record the choice here. Port `hello` onto the core and
   retire the prototype.
2. **Drivers.** Switch `lib::SimClock` to `int64` nanoseconds. Add `Step`, the
   lifecycle, `advance_to`, `BatchDriver`,
   `RealTimeDriver` and `RateGate`. Run `hello` under the real-time driver.
3. **Defense, headless.** The components and schedule above, with a
   `BatchDriver` test that checks a deterministic outcome for a fixed seed.
4. **Defense demo.** An ImGui and ImPlot view under `RealTimeDriver`.
5. **Performance.** Profile at thousands to hundreds of thousands of agents.
   Consider struct-of-arrays layout inside hot components only if measurements
   call for it.

Later: `LockstepDriver` and a second process, scenario files with two-phase
loading, parent-child transforms (a radar mounted on a vehicle), and DIS or HLA
interop.

## Open decisions

- **Store layout.** Dense with an entity index, or stable slots. Decided by the
  step 1 benchmark.
- **Retargeting policy** for interceptors whose target disappears. To be
  settled while building defense.
- **Spatial index structure.** A uniform grid is the likely start; to be
  measured with defense at scale.
- **Cross-entity lookups.** Proposed: `using Lookups = Stores<...>;` on the
  system, reached through `Context<S>::lookup<T>(e)`, always read-only.

## Lessons from the older simulator

simon takes ideas from an older simulator its author wrote. What we kept:

- Plain-data components, and physics as free functions testable without the
  ECS.
- Computes as template functors with optional stages, so the hot path inlines.
- Execution order fixed by a type list.
- `(time, step)` passed explicitly, with no global clock.
- A driver that owns time, around a simulation that only steps.
- Rate gates as values inside components.
- An event queue for rare events only.
- Sensors as entities with their own pose.
- Strongly typed internal IDs, kept separate from external IDs.
- Names as typed magic numbers, so everything can be debugged.
- Builders as the type-safe, user-friendly way to ask for specific things,
  hiding type details without losing them.

What we changed, and why:

| Old | New | Why |
|---|---|---|
| Self-patching handles registered with their pool | Generation-checked `Entity` handles and fixed-capacity stores | Same three duties. Every old handle copy made a virtual call and a hash insert, only to track reallocation. |
| Pools with holes, iterated through a rebuilt `vector<T*>` | Dense stores iterated directly | Holes and pointer chasing defeat the cache the architecture exists to use. |
| Components with vtables, and entities holding a hash map of components | Plain structs, and entities that are only IDs | Idiomatic ECS. The entity map needed a hash lookup and a `dynamic_cast` per access. |
| `DependencyTracker` with deferred callbacks | Two-phase loading, lookup by name at use time, and an `EntityCreated` event | Deferred resolution existed only because of arbitrary construction order. |
| Identities drawn from static counters | Identities owned by the builder that creates the entity | Names become deterministic and traceable to their creator. |
| Builders with several modes that built in the destructor and type-erased through `dynamic_cast` | One builder per request with an explicit, checked terminal call that emits typed commands | Keeps the builder as the user-facing language. Invalid utterances were only caught at runtime, and failures were only logged. |
| Order by template arguments, dependencies in comments | Order by template arguments, dependencies declared in `System` types and checked | Keeps the compiler-fixed order and makes the dependencies checkable. |
| One system per component type, with `ComputeNone` fillers | Systems that iterate any set of components; stores exist without a system | A store is data. It does not need a system to exist. |
| Scenarios assembled from string paths into an external proto schema | Code-first scenarios now; scenario files with two-phase loading later | Removes about 45 near-identical attach functions. |
| Multi-process state machines and liveness inside the engine | A small lifecycle enum and `advance_to` in the core; transport outside it | Keeps what every simulator needs and leaves deployment concerns at the edge. |
