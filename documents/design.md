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

**You pay only for what you use.** simon favors scale over accuracy by
default. A feature that adds accuracy or capability, such as a multi-stage
integrator or 6-DOF dynamics, is opted into through a type: a schedule
element, an archetype's components or a trait. It may be slower for whoever
uses it, but a simulation that does not use it runs no slower, holds no more
bytes per entity and compiles no more code. Such features add their own types
instead of fields to shared ones (`Step`, `Entity`, `Kinematics`), keep no
world-wide bookkeeping, and live in their own Bazel targets. missile uses none
of them, so `missile_benchmark` and `bytes_per_entity_v` check the rule: a
feature missile does not use must not change either.

## How we work

We build applications and keep the roadmap in mind. We do not build the
framework first and look for applications afterwards. A piece of the toolkit
exists because an application needed it, and it moves into the toolkit once a
second use appears or the roadmap clearly calls for it.

We prefer existing, modern libraries for anything outside the framework. The framework
(entities, stores, systems, schedules and drivers) is ours, because its layout
and handle rules are the point of the project.

## Conventions

**Reference arguments are marked at the call site** with `lib`'s wrappers:

| Wrapper | The callee | Call site |
|---|---|---|
| `Out<T>` | writes the argument, and may ignore it | `parse(text, Out(result))` |
| `InOut<T>` | reads and writes the argument | `scheduler.step(InOut(world), step)` |
| `Depend<T>` | keeps a reference that can dangle | `BatchDriver{Depend(simulation), timing}` |

Call sites use the constructor function, `Out(x)`, `InOut(x)` or `Depend(x)`,
and let it deduce the type; only parameters name it.

**Parameters read inputs first, then what the callee writes or keeps, then what
it only writes:** `(In..., InOut..., Out)`, with `Depend` among the `InOut`s.
`integrate_midpoint(acceleration, dt, InOut(kinematics))`,
`scheduler.step(step, InOut(world))`, `BatchDriver{timing, Depend(simulation)}`.
A system's call operator is the exception: the framework sets its order.

**Return a value or write an `Out`, whichever suits the type.** An `Out`
parameter fills an object the caller already owns, so it suits types that are
large or never move, such as a world.

**Errors are `std::expected<T, lib::Status>`, propagated with `lib`'s macros**
instead of an `if` per call:

```cpp
RETURN_IF_UNEXPECTED(build_world(scenario_, lib::Out(world_)));
ASSIGN_OR_RETURN(asset_, build_scenario(scenario_, lib::InOut(world_)));
```

`RETURN_IF_UNEXPECTED` returns the error, if any, from the enclosing function.
`ASSIGN_OR_RETURN` does the same and otherwise moves the value into an existing
variable or a new declaration (`Entity asset`).

**Functions declare a trailing return type:** `auto f(X) -> Y`, `void` ones
included (`auto step(const Step& step) -> void`). Constructors, destructors and
conversion operators cannot, and lambdas keep their deduced returns. lib's
`.clang-tidy`, which simon and volcano link to, checks it with
`modernize-use-trailing-return-type`; that check skips `void` functions.

**A getter that may return null is named `maybe_*`:**
`maybe_component_of<Health>(entity)` returns a pointer, null if the entity has
no Health. Its non-null counterpart drops the prefix, `component_of`, and
returns a reference.

**Headers hold what must be inline; `.cpp` files hold the rest.** Templates,
and code on hot paths that the inliner must see (a system's call operator,
`Vector3`, the spatial index's per-query helpers), stay in headers. Setup,
string formatting and once-a-step code (`build_scenario`, the site builder's
`build()`, `Simulation`, name parsing) go in `.cpp` files.

**A class has one `public:` section, then one `protected:`, then one
`private:`.** Compile-time checks and aliases may come first, before
`public:`, unlabeled.

**Every pointer is initialized,** `Type* pointer = nullptr;`, even when a
constructor always sets it.

**Implementation details live in a namespace named `internal`.**

**Unused names say so without comments.** Leave a parameter unnamed when its
type says what it is, as a system's `(SystemWorld&, Entity, ...)` does; no
compiler warns about an unnamed parameter. Name it with `[[maybe_unused]]`
when the name carries meaning the type does not, or when only some
instantiations use it. Never write `/*name*/`. Bind a result kept only to be
ignored, such as a builder's refusal, to C++26's `_`, which may be declared
again in the same scope: `auto _ = world.destroy(self).build();`. `_` does not
apply to parameters, which C++26 leaves out because they can be unnamed.

A plain `T&` parameter is only for what the language or the framework decides:
operators, and a system's call operator, whose entity-components arrive by
reference with constness declared in `System<...>`. Computing wrappers from
that constness would be awkward, and an `In<T>` for const parameters would stray
too far from common C++. `const T&` needs no wrapper.

**Template type parameters end with `Type`** (or `Types` for a pack):
`ComponentType`, `ComponentTypes...`, `WorldType`. Concepts are named for the
property they check, as adjectives where they read well: `Spatial`,
`Archetypal`.

**Types that are lists of types end with `List`:** `TypeList<...>`,
`SystemList<...>` (a schedule), `ComponentList`, `AllowComponentList`,
`SequenceAfterSystemList`. Metafunctions that produce one say so:
`write_list_of_t<SystemType>`.

**Accessors are named for what they return, never `get`:** `component_of` and
`maybe_component_of` (the `try_` form returns a pointer that may be null),
`store_of`, `name_of`. Where arguments of different strong types ask different
questions, one name is overloaded: `find_name_of(Identity)` and
`find_name_of(Alias)`.

A class that takes `Depend<T>` stores a plain pointer, so its hot path does not
pay for `CheckedPointer`'s null check on every access.

## Terminology

| Term | Meaning |
|---|---|
| Entity | An object with a name/identity suitable for quick comparison, canonical lookup and local aliasing. An `Entity` value is the local alias; its `Name` is the canonical identity. |
| Component | A type with a name/identity suitable for debugging. |
| Entity-component | A single object of a component type, associated with a single entity. |
| Archetype | What sort of object an entity is: a name, and the components an entity of it must and may have. Every entity is created from one. |
| Sibling | Another entity-component of the same entity, as seen from one of its components. In `System<Kinematics, const Control>`, each entity's `Control` is a sibling of its `Kinematics`. A sibling the entity's archetype requires exists for the entity's whole life; one the archetype only allows may be attached and detached. |

## Architecture

The architecture is idiomatic ECS with no exceptions:

| Concept | Rule |
|---|---|
| Entity | An identity and nothing else: `{index, generation}`. It holds no data and no pointers. |
| Component | Plain data. No behaviour, no virtual functions, no base class required. Each component type has a debug name. |
| ComponentStore | Exactly one per component. It is the array of that component's entity-components. |
| System | Every non-trivial function is a system. A system iterates component arrays through stores. |
| Builder | The user-facing language for asking the simulator to do something specific. A builder turns a request into commands. |
| Command | A typed, low-level structural change: create, attach, detach, destroy. Commands are applied to stores. |
| Schedule | A type listing systems in execution order. Schedules compose. |
| World | The entity database. A factory of builders (the only way to write) and a query interface (the only way to read). Configured by a `Spatial` model. |
| Driver | Owns time and steps the world's schedule. |

Two pairs factor the details out of the parts people write:

```
System  ──uses──▶  ComponentStore     ComponentStore hides indices, layout and lookup from systems.
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
     World ── entity table ── ComponentStore<Kinematics> ── ComponentStore<Radar> ── ...
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
struct Kinematics final {
  Position position = meters(0.0, 0.0, 0.0);
  Velocity velocity = meters_per_second(0.0, 0.0, 0.0);
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

Each component has one `ComponentStore<T>`, which holds all of its entity-components.
Stores keep that array dense so that large simulations fit in cache. On the development machine (Zen
3, 512 KiB L2 per core, 32 MiB L3 per chiplet), 5,000 agents with a 72-byte
`Kinematics` take 360 KiB. That fits in one core's L2. With 50% holes the same
store takes 720 KiB and spills into L3.

A store keeps holes out of its data by indexing through the entity, and
divides the data into segments, one per archetype that requires the component
(see [Archetype segments](#archetype-segments)):

```
ComponentStore<Kinematics>
  index[entity.index] ─▶ {slot, generation}    8 bytes per entity slot
  segments            ─▶ [ drone chunks | interceptor chunks | ... | allowed chunks ]
  owner[slot]         ─▶ Entity                dense within each segment
  data[slot]          ─▶ Kinematics            dense within each segment; systems iterate this
```

- **Iterate.** Walk each segment's chunks from start to end. The index is
  never touched. Code outside systems walks a store with
  `store.for_each([](Entity owner, const T& component) { ... })`.
- **Append.** Write at the end of the entity's segment, taking a chunk from
  the pool when the last one is full, and write the index entry.
- **Erase.** Move the segment's last element into the gap, fix the moved
  element's index entry, clear the erased one, and return the last chunk to
  the pool when it empties. Every segment stays dense after every erasure.
- **Look up.** `index[e.index]` checks the generation, then reads the slot. The
  index for 5,000 entities is 40 KiB and stays hot.
- **Capacity.** The pool is allocated once, at a capacity chosen when the world
  is created, plus a partly filled chunk per segment. Stores never reallocate,
  so references into them stay valid until the element is erased or moved by
  an erasure.

A hole costs 8 bytes in the index, instead of a whole component in the array.

**Decided by measurement: dense.** The alternative was a stable-slot store,
where each entity index has a fixed slot, nothing moves, and a lookup is one
load, but destroyed entities leave holes. `framework/component_store_benchmark.cpp`
compares the two with a 72-byte component, destroying a random fraction of the
population (as when drones are shot down). On the development machine, with
`-c opt`:

| Entities | Churn | Dense iterate | Stable iterate | Dense lookup | Stable lookup |
|---:|---:|---:|---:|---:|---:|
| 10,000 | 0% | 0.88 | 1.17 | 1.25 | 1.19 |
| 10,000 | 75% | 0.72 | 2.73 | 0.95 | 0.94 |
| 100,000 | 0% | 0.85 | 1.13 | 1.67 | 1.13 |
| 100,000 | 50% | 0.93 | 4.86 | 1.52 | 1.14 |
| 100,000 | 75% | 0.84 | 6.16 | 1.50 | 1.16 |
| 1,000,000 | 75% | 1.06 | 14.78 | 7.34 | 5.99 |

Times are nanoseconds per live entity (iterate) or per random lookup.

- **Iteration** stays near 1 ns per entity in the dense store at every churn
  level. The stable-slot store slows in proportion to its holes: 3 to 7 times
  slower at 75% churn from 100,000 entities up.
- **Lookup** is 10 to 35% faster in the stable-slot store, which saves one load.

Systems iterate far more than they look up, and a missile run is mostly agents
dying, so the dense store wins. Run the benchmark with
`bazel run -c opt //framework:component_store_benchmark`.

#### Sibling components

A system walks its driving component and reaches each entity's siblings (see
[Terminology](#terminology)) through `maybe_component_of`. Siblings the
archetype requires are the common case, since a drone's `Control` exists for
as long as its `Kinematics` does. The older simulator's handles resolved such a
sibling once, at creation. The store before segments looked it up on every
step.

`framework/system_benchmark.cpp` measures that lookup against a handle (a
pointer resolved at creation) and against "structural" access, where both
arrays share an order and are walked together. A 72-byte driving component
reads a 24-byte sibling; times are nanoseconds per entity, uncontended:

| Entities | Sibling order | Lookup | Handle | Structural | No sibling |
|---:|---|---:|---:|---:|---:|
| 100,000 | aligned | 1.85 | 1.48 | 1.3–2.4 | 0.84 |
| 100,000 | shuffled | 3.33 | 2.44 | | 0.84 |
| 1,000,000 | aligned | 6.1 | 5.6 | 5.1 | 3.8 |
| 1,000,000 | shuffled | 14–16 | 12–13.6 | | 3.9 |

- **The lookup itself costs about 0.4 ns per entity** over a handle when both
  stores fit in cache, and 8 to 30% at a million entities.
- **Order matters more than lookup against handle.** When the sibling store's
  order diverges from the driving store's, both lookup and handle slow by about
  2.3 times, because each access lands on a random cache line.
- **Under contention** (`--contend`, 31 threads streaming 256 MB each),
  everything at a million entities costs 150 to 660 ns per entity, and lookup
  against handle stays within about 15%. Cache misses decide the cost.

So the question is how to keep siblings in a shared order under churn, without
asking users to manage it.

#### Layouts under churn

`framework/churn_benchmark.cpp` replays one schedule of operations against
six layouts. The population is constant; half the births live 1 to 4 steps
(like blasts) and the rest an average of 500 (like drones). An entity's
archetype is the set of siblings it is created with. It may attach and detach
only siblings its archetype allows, as `Tracked` is attached to a drone
partway through its life. Each step is timed as structural changes, layout
maintenance and iteration.

| Layout | What it does |
|---|---|
| dense | The store before segments: one array per component, swap-erase, and a lookup per sibling |
| sorted | Dense, and each store is sorted by entity index at sync points once 1 in 8 of it is out of order |
| group | An EnTT-style owning group: entities with both components sit at the same positions at the front of both stores, kept there by swaps |
| hybrid | A group, plus sorting the stores it does not own into the owner's order |
| generational | A settled region in entity order and a nursery in append order; survivors are merged into the settled region |
| segmented | One chunked segment per archetype in each store, lined up across the archetype's stores (below) |

The case that matters is two systems competing for one store, as `Integrate`
(Kinematics, Control) and `ApplyBlasts` (Kinematics, Health) both want
Kinematics. One sibling is on 80% of entities and the other on 50%.
Nanoseconds per entity-step, uncontended, including structural changes and
maintenance:

| Entities, sibling size | dense | sorted | group | hybrid | segmented |
|---|---:|---:|---:|---:|---:|
| 100,000, 24 B | 22.3 | 14.8 | 11.6 | 16.1 | **4.2** |
| 100,000, 256 B | 68.6 | 46.3 | 20.0 | 31.5 | **16.7** |
| 1,000,000, 24 B | 88.4 | 25.5 | 33.8 | 28.3 | **18.1** |
| 1,000,000, 256 B | 122.2 | 69.7 | 57.8 | 63.7 | **32.6** |

With a single pair, the best case for a group:

| Entities, sibling size | dense | sorted | group | generational | segmented |
|---|---:|---:|---:|---:|---:|
| 100,000, 24 B | 5.3 | 4.5 | 2.1 | 4.7 | **1.9** |
| 100,000, 256 B | 11.2 | 9.5 | **6.3** | 15.1 | 7.6 |
| 100,000, 1 KB | 18.2 | 24.5 | **13.3** | 28.0 | 14.3 |
| 1,000,000, 24 B | 18.3 | 10.3 | **5.8** | 9.0 | 7.4 |
| 1,000,000, 256 B | 23.1 | 23.7 | **13.4** | 26.7 | 16.4 |

Run-to-run noise is about 10%. An earlier contended run at 100,000 entities
(before toggles were limited to allowed siblings) kept the same ranking:
dense 570, sorted 283, group 169, generational 284 ns per entity-step with
24-byte siblings.

What each layout taught:

- **Sorting** helps small components and hurts large ones: a comparison sort
  moves every component about log n times, so with 1 KB siblings it costs more
  than it saves. It also pauses, up to 880 ms in one contended step. A
  permutation sort would move each component once, but the pauses and the
  tuning threshold remain. Dropped.
- **Groups** are the fastest single pair, but a store can belong to only one
  group, so the system that loses the store is left with lookups in an order
  the group scrambles: 2 times slower than sorting at a million entities. They
  also need the user, or a solver, to choose owners.
- **The hybrid** speeds up the loser, and spends the gain sorting.
- **The generational layout** doubles memory and loses with large components.
  Entity lifetimes here do follow the generational hypothesis, but the layout
  answers "when to compact", which is not the question.
- **Segments** win whenever systems compete, because no store has an owner.
  They trail a group only where a group also swallows siblings that were
  attached later; segments leave those in a sparse store.

#### Archetype segments

Locality comes from correlations the world already knows. Components present
together are declared by an archetype's `Requires`. Components read together
are declared by each `System<Driving, Others...>`. The older simulator's
handles modeled the same dependency at creation. Segments use the archetype
directly:

```
ComponentStore<Kinematics>: [ drone 0..n      | interceptor 0..m | radar | launcher | blast ]
ComponentStore<Control>:    [ drone 0..n      | interceptor 0..m ]
ComponentStore<Health>:     [ drone 0..n      | asset ]
```

- **Each store is divided into one segment per archetype that has the
  component.** Within an archetype, every store orders its segment the same
  way, so drone i's `Kinematics` and `Control` sit at local index i of the drone
  segment in both. The shared index is the handle: nothing points, nothing
  dangles, and the same operation moves both.
- **Segments are lists of fixed-size chunks** drawn from a pool each store
  allocates once. A segment grows by taking a chunk and shrinks by returning
  one, so growth never moves another segment's data, and the only waste is one
  partly filled chunk per segment. Every store in a world uses the same chunk
  size, a power of two of at most 1,024 entries (smaller in small worlds), so
  local index i is the same chunk and offset in each.
- **Every archetype requires `EntityArchetype` and allows `Parent`.** The last
  segment of each store holds entities whose archetype only allows the
  component, whether they were created with it or it was attached later.
- **Creation appends to the archetype's segment; destruction moves the
  segment's last entity into the gap in each of its stores.** Structural cost
  matches swap-erase.
- **The runner walks segment by segment,** and knows at compile time, per
  archetype, whether each other component is required (a pointer to the same
  slot of the matching chunk), absent (a null pointer, so that branch compiles
  away), or allowed (a lookup). In the last segment every other component is
  looked up. A contract check confirms, once per segment, that each required
  sibling's segment has the same size. This needs the world to list its
  archetypes (see [Configuration](#configuration)). Archetypes name only
  components, so there is no cycle with systems.
- **The builder protects the alignment.** A `change` that detaches a component
  the entity's archetype requires fails with `BuildError::COMPONENT_REQUIRED`,
  and one that attaches a component the archetype neither requires nor allows
  fails with `BuildError::COMPONENT_NOT_PERMITTED`.
- **Components an archetype only `Allows` stay in sparse stores** with
  lookups. We accept that components outside the archetype do not get the
  automatic layout. Archetype-table ECSs (Unity DOTS, flecs) move the whole
  entity to another table instead, which is where their 10 to 30 times slower
  attach and detach comes from.
- **Later, hierarchical locality.** Names such as `/blue/radars` could order
  segments within each store, so a system that walks every blue archetype
  streams one range. This is the slab arrangement games have used for decades,
  applied to segments that already exist.
- **Determinism.** Segment order follows the command sequence, so
  resimulation repeats it. A checkpoint saves segments as they are, with no
  extra bookkeeping.

Measured with `system_benchmark` after the change (nanoseconds per entity,
uncontended; "allowed" is a sibling attached after creation, "required" one in
the archetype's segment):

| Entities | Allowed, aligned | Allowed, shuffled | Required | Handle | Structural |
|---:|---:|---:|---:|---:|---:|
| 100,000 | 1.92 | 3.17 | 1.51 | 1.47 | 1.3–2.2 |
| 1,000,000 | 6.03 | 13.46 | 5.18 | 5.54 | 5.17 |

A required sibling now costs what a handle or a structural walk costs. Over
3,000 steps at 10,000 drones, the missile step went from 0.738 to 0.691 ms:
`Integrate` from 0.060 to 0.041, `TriggerWarheads` from 0.105 to 0.070 and
`SteerRedDrones` from 0.078 to 0.067. `ApplyBlasts` first went from 0.040 to
0.056, because it walked the few blasts once per victim and a segmented walk
has more fixed cost than an array. It now collects the step's blasts once in
`prepare`, and takes 0.026, bringing the step to 0.66 ms.

### Handles and references

A handle has three duties, carried over from the older simulator:

1. Hide indices and addresses from the component programmer.
2. Never allow a memory error.
3. Never fail silently.

`Entity` is the handle. A system reads another entity's component through the
store:

```cpp
const Kinematics* target = world.maybe_component_of<Kinematics>(interceptor.target);  // null if gone
const Kinematics& target = world.component_of<Kinematics>(interceptor.target);      // contract check if gone
```

`maybe_component_of` is the explicit path for "this may have disappeared".
`component_of` fails a contract check on a stale entity, so misuse stops
loudly. `ComponentStore` has the same pair. Neither can read
freed memory, because store memory lives as long as the world.

Raw pointers or references returned by `maybe_component_of` or
`component_of` are only valid until the next
sync point. Systems must not keep them across steps.

Nothing registers handles or patches them. The older simulator's self-patching
handle met the same three duties, but every copy made a virtual call and a hash
insert to follow objects that reallocation had moved. Fixed capacity removes
the reallocation, and the generation check replaces the patching.

The older handle also made a structural sibling, such as a drone's `Control`,
free to reach on every step. Archetype segments (see
[Sibling components](#sibling-components)) recover that without a handle: a
component the archetype requires sits at the entity's own index in the
sibling's store.

### Names, identities and aliases

Many entities and components come and go during a run, and every one of them
must be debuggable from a console, comparable in a hot loop, and findable by
what people call it. Everything in a world (the world itself, its archetypes,
components, systems, entities and entity-components) is known three ways:

| | For | Example | Cost |
|---|---|---|---|
| **Name** `{kind, instance}` | Hot loops, logs, crossing processes | `{Entity, 2}` | Two `uint32_t`, one compare |
| **Identity** | Debugging from a console | `/world/1/entity/2/component/3` | Computed from the Name |
| **Alias** | Meaning only people bring | `"ego"`, `"Luke Skywalker"` | A multimap, off the hot path |

**A Name** is a pair of integers. `kind` says what is named; `instance` says
which one:

| Named | Name | Identity |
|---|---|---|
| World | `{World, 1}` | `/world/1` |
| Archetype | `{Archetype, 0}` | `/world/1/archetype/0` |
| Component | `{Component, 3}` | `/world/1/component/3` |
| System | `{System, 4}` | `/world/1/system/4` |
| Entity | `{Entity, 2}` | `/world/1/entity/2` |
| Entity-component | `{EntityComponent + 3, 2}` | `/world/1/entity/2/component/3` |

- **Each component type is its own kind for its entity-components,** so an
  entity-component's Name fits in the same two integers and its Identity
  follows from it.
- **A component's instance** is its position in the world's component list. A
  system's is its position in the flattened schedule. An archetype's is the
  order in which the world first saw it.
- **An entity's instance is a serial number in creation order, never reused.**
  `Entity` indices are recycled; entity Names are not. A Name stays meaningful
  after its entity is destroyed, the same scenario gives the same Names on every
  run and machine, and Names can cross processes.
- **The world's number is given when it is built**
  (`World::set_up().numbered(n)`). There is no global counter; the older
  simulator's static identity source had to be set before anything could be
  constructed.

`Entity` and Name are both eight bytes and compare in one instruction. An
`Entity` is the local alias systems use to reach stores. A Name is the stable
identity for logs, events and anything leaving the process.

**An Identity** is a REST-like path computed from a Name, so the world stores
no strings for it. `world.identity_of(name)` formats one and
`world.find_name_of(Identity{path})` parses one back, returning nothing for anything that does not exist in this
world. `world.describe(name)` gives a one-line summary for a console:

```
/world/1/entity/0 (red) archetype ball: Kinematics, Control, Thrust, Wind, Drag, Collider, Collision
```

**An Alias** is a string with meaning only people bring, such as "ego" or "Luke
Skywalker". Aliases are many-to-many: several entities can share "red drone",
and one entity can be both "ego" and "Luke Skywalker".
`world.find_name_of(Alias{"ego"})` returns every Name with it, in the order the
aliases were given.

`Identity` and `Alias` are distinct strong types (`TaggedString<Tag>`). Each
converts implicitly from anything that converts to `std::string_view`, but
neither converts to the other, so `find_name_of` is one overloaded name and the
argument's type picks the query. A bare string literal is ambiguous there, so
callers say which they mean.

- `create<Ball>("Luke Skywalker")` gives the new entity its first alias.
- `change(e).alias("ego")` and `change(e).unalias("ego")` add and remove more.
- Component types are aliased by their type names, qualified and short, so
  `/world/1/component/3` is also findable as `Kinematics`.
- Archetypes are aliased by their names, so the ball archetype is findable as
  "ball".
- Destroying an entity drops its aliases.

Aliases are an index beside the stores, not store shape, so alias changes take
effect immediately at `build()` instead of at the next sync point.

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

Builders are the user-facing language of the toolkit: a type-safe, friendly way
to ask the simulator for something specific. They form a fluent grammar, so
their names are chosen as a grammar, from the sentences users should be able to
say:

```
In the world, create a ball known as Luke Skywalker, under squadron 1, with kinematics and a collider.
In the world, create an interceptor under launcher 2, with kinematics.
In the world, change ball 0: attach health, detach drag, and call it "ego".
In the world, destroy ball 0.
```

```cpp
world.create<Ball>("Luke Skywalker").under(squadron).with(Kinematics{...}).with(Collider{...}).build();
world.create<Interceptor>().under(launcher).with(Kinematics{...}).build();
world.change(ball).attach(Health{}).detach<Drag>().alias("ego").build();
world.destroy(ball).build();
```

The grammar:

```
utterance  := world . verb . complement* . build()
verb       := create<ArchetypeType>(alias?) | change(entity) | destroy(entity)
complement := under(entity)                create: at most once, before any with
            | with(component)              create: what the new entity starts with
            | attach(component)            change
            | detach<Component>()          change
            | alias("...")                 change
            | unalias("...")               change: at least one of the four
```

- **`create<ArchetypeType>(alias)` carries both archetype and entity information.**
  The archetype (see below) supplies its name and the components the entity
  requires and permits. The optional argument is the entity's first alias. The
  world issues the entity's Name.
- **`under(parent)` records the entity it was created under** (an interceptor
  under its launcher) as a built-in `Parent` entity-component. That is a
  relation, not part of the entity's identity, and it is what the world's
  transform hierarchy will build on.
- **`with` declares a new entity's starting components. `attach` and `detach`
  change a live one.**
- **`build()` ends every utterance.** It returns `std::expected<Entity,
  Status>` for `create` and `std::expected<void, Status>` for the others.

#### Archetypes

Every entity is created from an archetype, which states what sort of object the
entity is and which components it must and may have. (In this document "class"
only ever means a C++ class.)

```cpp
struct Ball : Archetype<"ball", Requires<Kinematics, Collider, Collision>,
                        Allows<Control, Thrust, Wind, Drag>> {};
```

The archetype is recorded on the entity as a built-in `EntityArchetype`
entity-component. `world.archetype_of(e)` returns the archetype's Name, and
`world.aliases_of(archetype)` its name. That is the information an IO layer needs to
map entities to foreign object types, such as HLA object classes or DIS entity
types, and it helps debugging.

#### Validation

A builder is a grammar, and an utterance can only be validated once it is
complete. A builder accumulates the utterance, validates it at `build()`, and
only then emits commands.

- **Word order and composition are checked at compile time.** Each step returns
  a builder of a different type, so only well-formed sentences compile:
  - `under` after `with` does not compile.
  - `with` of a component the archetype neither requires nor allows fails with
    "The entity's archetype neither requires nor allows this component."
  - `build()` before every required component is declared fails with "The
    entity lacks a component its archetype requires."
  - A `change` that attaches, detaches, aliases and unaliases nothing does not
    compile.
- **Data-dependent rules are checked by `build()` at runtime, against the state
  the world will be in once pending commands apply:** the entity (and any
  parent) is alive and not about to be destroyed, a component to attach is
  absent, permitted by the entity's archetype and its store has room, a
  component to detach is present and not required by the archetype, an alias is
  non-empty and not already given, an alias to take was given. A refused
  utterance returns a `lib::Status` whose condition is a `BuildError`
  (compare with `status == lib::watch(BuildError::ALIAS_NOT_GIVEN)`), and
  emits nothing. Because every command was validated this way, applying
  commands at a sync point cannot fail halfway through a batch.
- **A store's room counts what is pending, in order.** Commands apply in the
  order they were recorded, so each store's plan keeps a running growth:
  every attach adds one, and every detach, and every destroy of an entity that
  will have the component, takes one away. An attach fits when the store's
  size plus that growth is below its capacity, so a detach or destroy
  recorded earlier makes room for it, and one recorded later does not.
  Entity capacity does not count pending destroys, since creating reserves
  the entity at once.
- **The terminal call is explicit and `[[nodiscard]]`.** A builder never does
  work in its destructor, and a failure is never only logged.
- **One builder type per verb.** A builder does not switch between modes.
- **Creating reserves the entity and its Name immediately,** so the returned
  `Entity` can be stored, and children can be created under it, before the
  commands are applied.

`lib::Status` keeps each incident's message in a ring buffer of 16 per
condition domain, and that buffer is not thread-safe. Inspect a returned
`Status` where it is returned, or keep a `detach_copy()`. The simulation is
single-threaded today; this needs revisiting before it is not.

#### Domain builders

A builder hides important type details without losing them. It may convert or
erase types internally (a query, a set of archetypes), and the caller never
sees those details. What leaves the builder is always a list of typed commands,
so nothing is lost.

Domain builders extend the same grammar with verbs of their own. Missile builds
each defended site with one:

```cpp
std::expected<Entity, Status> asset =
    create_site(origin, lib::Depend(world))
        .protecting(Health{.points = 30.0})
        .watched_by(10, radar, 500.0 * meter)
        .defended_by(50, launcher, 300.0 * meter)
        .attacked_by(1000, drone, warhead, Ring{.radius = 3500.0 * meter,
                                               .width = 1000.0 * meter},
                     lib::Depend(random))
        .build();
```

`create_site` returns a builder whose type the caller never names, and
`build()` becomes an entity utterance for each asset, radar, launcher and
drone. Builders and commands are not isomorphic: one utterance emits many
commands.

**Domain builders are atomic, through transactions.** `world.transaction()`
opens one. While it is open, every planned change records how to undo itself:
entities reserved and their names, aliases, planned attachments, detachments
and destructions, and their commands. `commit()` keeps them; a transaction that
ends without committing rolls them back, newest first, so an early return
undoes the whole utterance and leaves no capacity reserved:

```cpp
auto transaction = world_->transaction();
ASSIGN_OR_RETURN(Entity asset, world_->create<archetype::Asset>("asset")...build());
RETURN_IF_UNEXPECTED(world_->create<archetype::Radar>()...build());
transaction.commit();
```

Transactions nest, and an inner commit keeps its work only if every enclosing
transaction commits. Every transaction must end before `sync()`. Undo steps are
recorded only while a transaction is open, so ordinary builders pay nothing.
Registering an archetype on its first creation is not undone; it describes what
the world can hold, not what the utterance planned.

**Query forms** select entities and apply one verb to all of them, atomically.
Destroy and change have query forms. Destroy:

```cpp
std::expected<std::size_t, Status> destroyed =
    world.destroy()
        .each<archetype::RedDrone>()  // An archetype, or a component.
        .within(asset_kinematics, 500.0 * meter)
        .where([&](Entity drone) { return drone != spared; })
        .build();
```

`each` comes first, selecting by archetype (walking that archetype's segment)
or by component (walking its store). `within` narrows through the spatial
index, and each `where` narrows by a predicate. `having<T>()` and
`lacking<T>()` narrow by whether an entity will have a component once pending
commands apply, which is the state a builder validates against. `build()`
plans every destruction in one transaction, skips entities already planned for
destruction, and returns how many it destroyed.

Change selects in the same words, then changes in the words of the
single-entity `change`:

```cpp
std::expected<std::size_t, Status> changed =
    world.change()
        .each<archetype::RedDrone>()
        .within(asset_kinematics, 4000.0 * meter)
        .attach(Tracked{})
        .detach<Health>()
        .alias("hostile")
        .build();
```

Every selected entity gets a copy of each attached component. `build()` makes
the change to each entity in one transaction. If the world refuses it for any
entity (an archetype that doesn't permit the component, a full store, an alias
already given), nothing changes and `build()` returns that Status. The same
compile-time checks apply as for one entity: the component is in the world's
list, it isn't built in, and each is attached or detached once. A change that
attaches, detaches and aliases nothing doesn't compile.

Both query forms share `Query`, which holds the selection: the chosen archetype
or component, the `within` region and the predicates. Selection happens before
the transaction opens, so a predicate sees the world as it was before the
utterance.

#### IO is not a builder verb

The world has no verb for connecting to a foreign simulator. The IO layer is
distinct: the simulation has no idea where entities and components came from,
only that they are there. The programmer who adapts a foreign simulator at the
IO layer uses the same builders as anyone else, and connection setup belongs
to the IO layer and the driver.

#### Commands

Commands are the low-level vocabulary: attach, detach, destroy. They are typed,
go into the world's command buffer, and are applied at sync points in the order
they were recorded. The stores never change shape while a system iterates them.

Sync points sit between systems. A system's commands are applied before the
next system runs, so later systems in the same step see the change.

Builders validate an utterance against the planned state: what the world will
be once pending commands apply. Each component keeps a plan of which entities
will gain or lose it and how many entity-components are waiting to be attached,
and the world tracks which entities will be destroyed. So a second radar trying
to mark a drone `Tracked` in the same step is refused, although the first mark
is not applied yet, and applying a batch at `sync()` cannot fail.

Two things happen when `build()` succeeds rather than at sync:

- **Creation reserves the entity and its Name,** so the returned `Entity` can be
  stored, and children created under it, before the commands apply. Its
  capacity is in use until then, even if the entity is destroyed in the same
  batch.
- **Aliases change immediately.** They are an index beside the stores, not
  store shape.

Commands are an in-process type, a `std::variant` generated from the world's
component list. Logging, replaying or sending them to another process (the DIS
and HLA direction) would build on them, and is not built.

Systems use builders too, through their `ProjectedWorld`:
`world.create<Interceptor>().under(self).with(...).build()`. Inside a system
whose access parameter is `auto&`, the free-function form avoids the
`template` keyword: `create<Interceptor>(lib::InOut(world))`.

`ProjectedWorld` has the query forms as well, `world.destroy()` and
`world.change()`, limited to what the system declares it reads. Selecting by a
component reads that component's store, so the component must be in the
system's `AllowComponentList`. `within` reads the spatial index, so the
spatial component must be there too. Selecting by archetype reads nothing the
system must declare. A query form selects across every entity, so a system
uses it from `prepare` or `resolve`, which run once per step, and not from the
per-entity call:

```cpp
struct ClearOrigin final  //
    : System<const Health> {
  using AllowComponentList = TypeList<Position>;  // within reads Position.
  auto prepare(auto& world) -> bool {
    destroyed = world.destroy()
                    .template each<Body>()
                    .within(Position{0.0}, 2.0)
                    .build();
    return false;  // Nothing to do per entity.
  }
  ...
};
```

The query builders carry a read policy as a template parameter. The world's
query forms use `ReadAnything`; `ProjectedWorld` passes one that answers from the
system's `AllowComponentList`, and `each` and `within` check it with a
`static_assert`.

### Systems

A system declares its structure as a type. `System<DrivingComponentType, OtherComponentTypes...>` is a
variadic template: the first component drives the loop, and every following
component is optional and belongs to the same entity.

```cpp
template <typename DrivingComponentType, typename... OtherComponentTypes>
struct System {
  using DrivingComponent = DrivingComponentType;
  using OtherComponentList = TypeList<OtherComponentTypes...>;
};

struct GuideInterceptors final   //
    : System<const Interceptor,  //
             const Kinematics,   //
             Control> {
  using SequenceAfterSystemList = SystemList<UpdateTracks>;
  using AllowComponentList = TypeList<Kinematics>;   // other entities, always read-only
  using SystemWorld = ProjectedWorld<GuideInterceptors>;

  auto operator()(SystemWorld& world, Entity self,  //
                  const Interceptor& interceptor,   //
                  const Kinematics* kinematics,     //
                  Control* control,                 //
                  Step step) const -> void {
    if (!kinematics || !control) return;
    const Kinematics* target = world.maybe_component_of<Kinematics>(interceptor.target);
    if (!target) { ... }
    control->acceleration = model::proportional_navigation(*kinematics, *target, interceptor);
  }
};
```

The framework reads only the `System<...>` arguments. It walks the driving
store and passes each entity's optional entity-components. In outline, for the
entities whose archetype only allows the driving component:

```cpp
auto& drive = world.store_of<typename S::DrivingComponent>();   // const ComponentStore<Interceptor>& when const
for (each chunk in the last segment of drive)
  for (std::size_t i = 0; i < chunk.size; ++i) {
    Entity e = chunk.owners[i];
    system(access, e, chunk.components[i],
           world.store_of<OtherComponentTypes>().maybe_component_of(e)..., step);
  }
```

In each archetype's segment, a sibling the archetype requires comes from the
same slot of the matching chunk instead of a lookup (see
[Archetype segments](#archetype-segments)).

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
  framework then takes a `const ComponentStore<T>*` for it, and a const store only hands
  out `const T&` and `const T*`. The call operator must accept what the
  declaration implies; a mismatch is a compile error.
- **Other entities are reached through an allow list.** `using AllowComponentList =
  TypeList<...>;` opts the system in to reading those stores by entity.
  `world.maybe_component_of<T>(e)` returns `const T*`, `world.component_of<T>(e)`
  returns `const T&` with a contract check, and `world.store_of<T>()` returns
  the whole read-only store. Reads through the allow list are always
  read-only, and asking for a component not on it does not compile.
- **Owners of some components can be left out.** `using ExcludeComponentList =
  TypeList<...>;` keeps the system from running for any entity that has one
  of them. The runner decides per archetype at compile time: it skips the
  segment of an archetype that requires an excluded component, looks the
  component up for an archetype that only allows it, and checks nothing for
  one that cannot have it. A system that excludes nothing pays nothing, and
  one cannot both name and exclude a component. Fidelity levels use it: the
  flight application's single-pass `Fly` excludes the `AirStateRate` that
  only Runge-Kutta aircraft have.
- **The call order is `(ProjectedWorld& world, Entity self, driving component,
  other components..., Step step)`.** `ProjectedWorld<S, W>` is the system's
  opt-in access to the world: its allow list, spatial and name queries, and
  builders. The `Step` (see [Time](#time-and-drivers)) is passed by value, on
  the stack, and is optional: the scheduler passes it only if the call
  operator takes it. Time is not world data, so it is not in `ProjectedWorld`.
- **Optional stages** (`prepare(world[, step])` before the main loop,
  `resolve(world[, step])` after it) are detected at compile time and cost
  nothing when absent. The older
  simulator did the same with stage tags.

**Any callable can be a system.** Systems often keep state between steps, so
besides structs, a system can be made from a lambda or any other callable. The
structure still lives in a type; the callable keeps its own state in its
captures:

```cpp
auto integrate = framework::system<Kinematics, const Control>(
    [](auto&, Entity, Kinematics& kinematics, const Control* control, Step step) { ... });
auto count = framework::system<const Health>(
    [seen = 0](auto&, Entity, const Health&) mutable { ++seen; });
auto guide = framework::system<const Interceptor, const Kinematics, Control>(
    framework::TypeList<Kinematics>{},  // AllowComponentList.
    [](auto& world, Entity, const Interceptor&, const Kinematics*, Control*) { ... });
```

A struct system keeps its state in members, which the scheduler owns. Either
way, each scheduled system is one object that lives as long as its scheduler.

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
and its `AllowComponentList`. The compiler enforces them: a system declared with
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
optional ones. Everything it reaches through a `ComponentStore` lookup is read-only. To affect another entity, a system either uses a builder (create,
edit, destroy) or leaves data that the other entity's own system reads.

When the natural loop runs the wrong way, invert it. Make the side being
written the batch, and look up the side being read.

#### Example: detonation

```
TriggerWarheads   driven by Warhead, reads Kinematics*   writes its own warhead state
   │  on trigger:  world.create<Blast>().under(self).with(Kinematics{...}).with(Blast{...}).build()
   │               world.destroy(self).build()
   ▼  sync point: this step's Blast entities exist
ApplyBlasts       driven by Health, reads Kinematics*    victims are the batch; each writes only itself
   │  asks the world for blasts within range, read-only
   │  if destroyed: world.destroy(self).build()
   ▼
ExpireBlasts      driven by Blast                        destroys blasts older than one step
```

- **The blast is an entity** with `Blast` and `Kinematics` entity-components,
  created under the warhead, so its `Parent` records where it came from. It can be debugged and drawn, and
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
| One-to-one reference | ComponentStore an `Entity`; look it up read-only | An interceptor reads its target's `Kinematics` |
| One-to-many by space or predicate | A shared per-step index or a query, read-only | Blast radius, radar coverage |
| Affecting another entity | Emit an entity or command; the target's own system applies it | Blasts, new tracks |
| Conflicting claims | Propose, then resolve | Each launcher writes a proposal on itself; `ResolveEngagements` iterates tracks and picks one |
| Many-to-many | A relation entity holding both `Entity` values | A radar observing a track |

#### Guarding against misuse

- **Reads through the allow list are always read-only.** There is no way to
  declare a writable one, so "loop over blasts and write each victim" cannot be written. The
  error message points at the inversion pattern.
- **A system cannot both write `T` and look it up.** Declaring a non-const `T`
  in the `System` type and `T` in its `AllowComponentList` fails to compile. Reading an array
  partway through writing it gives results that depend on iteration order.
  This is why guidance writes `Control` and reads `Kinematics`.
- **Structural changes to other entities go through builders.** Their commands
  apply at the next sync point, so nothing is destroyed partway through an
  iteration.
- **Pointers from `maybe_component_of` are valid only until the next sync point.** Debug
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
auto run(SystemList<Ss...>, World& world, Step step) -> void {
  (run_system<Ss>(world, step), ...);   // each call is a direct, inlinable instantiation
}
```

Each system is listed once, in the schedule type.

A schedule is also a value that holds its systems, because systems made from
lambdas with captures cannot be default-constructed. Schedules of
default-constructible systems need no value:

```cpp
framework::Scheduler<World, SystemList<ApplyForces, model::Motion>> scheduler;   // Structs only.

auto schedule = SystemList{count, SystemList<ApplyForces, model::Motion>{}, guide};
framework::Scheduler<World, decltype(schedule)> scheduler{schedule};         // With lambdas.
```

Schedules compose:

```cpp
// Toolkit pieces.
using Motion  = SystemList<Integrate>;
using Sensing = SystemList<ScanRadars, UpdateTracks, DropStaleTracks>;
using Blasts  = SystemList<TriggerWarheads, ApplyBlasts, ExpireBlasts>;

// Application schedules.
using HelloSystems   = SystemList<ApplyWind, Motion, DetectCollisions>;
using MissileSystems = SystemList<Sensing, ProposeEngagements, ResolveEngagements,
                               LaunchInterceptors, GuideInterceptors, SteerRedDrones,
                               Motion, Blasts, CheckOutcome>;
```

- **Nested schedules flatten** at compile time into one list.
- **Ordering constraints are checked** on the flattened list. A system that
  declares `using SequenceAfterSystemList = SystemList<X>;` fails the build if it is scheduled
  before `X`. `SequenceAfterSystemList` is about order only: if `X` is not in the schedule, it
  imposes nothing, so a sub-schedule can run alone.
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
          write                                           read
world.create<C>()...build() ─▶ commands ─▶ World ─▶ by name       world.find_name_of(identity)
world.change(e)...build()   (applied at            by component  System<A, B...>, world.query<...>()
world.destroy(e).build()     sync points)          by space      world.within(center, radius, visit), world.nearest(center, radius, accept)
                                                   by relation   world.parent(e), world.children(e)
```

We use database terms for ECS concepts where they fit:

| Database | World |
|---|---|
| Primary key | `Entity` (local alias) and `Name` (canonical identity) |
| Row | An entity, its fields spread across the stores of its components |
| Column (of a column store) | A store: one component for every entity that has it |
| Schema | The world's component list and archetype list, fixed at compile time |
| `NOT NULL` and nullable columns | An archetype's `Requires` and `Allows`; builders refuse any other component |
| Partitions | Archetype segments: each store is partitioned by archetype, in the same order in every store, so a row's fields sit at the same slot |
| Indexes | Each store's entity index, the name index, the spatial index, the transform hierarchy |
| Selection (`WHERE`) | `where`, `within`, `having` and `lacking` in queries and query forms |
| Projection | `ProjectedWorld`: the stores a system declares, read-only beyond its own row |
| Left outer join | A system's loop: `System<A, B, C>` runs for every `A`, with `B` and `C` null where absent |
| Driving table | `DrivingComponent`: the store the loop scans row by row |
| Merge join | A sibling the archetype requires, read from the same slot of the matching chunk |
| Index nested-loop join | A sibling the archetype only allows, looked up through its store's index |
| Anti-join (`NOT EXISTS`) | `ExcludeComponentList`: rows that have any of these components are left out |
| Foreign key | An `Entity` field in a component, followed read-only through `AllowComponentList` |
| Transactions | Commands, applied at sync points in recorded order; `world.transaction()` makes an utterance atomic and rolls it back unless committed |
| Stored procedures | Systems |

A world holds every entity-component its builders created, the mappings from
Names to entities, the aliases, and any relationships between entities. The
built-in components `EntityArchetype` and `Parent` are always in a world's
component list; applications do not list them. Relationships are
ordinary data: `Entity` fields in components, or relation entities (see
[Relations between entities](#relations-between-entities)).

#### Configuration

A world is configured at compile time by a `Spatial` model, its component
list and its archetype list:

```cpp
template <typename Type>
concept Spatial = requires(const Type& a, const Type& b) {
  { distance(a, b) < distance(a, b) } -> std::convertible_to<bool>;
  pose(a);
  { coordinates(a) } -> std::same_as<Coordinates>;  // std::array<double, 3>
  { coordinate_length(a, distance(a, b)) } -> std::same_as<double>;
};

template <Spatial SpatialType, typename ComponentListType,
          typename ArchetypeListType>
class World;

using World = framework::World<
    Kinematics,
    TypeList<Control, Health, Warhead, Blast, RedDrone, Tracked, Asset, Radar,
             Track, Launcher, Interceptor>,
    TypeList<archetype::Asset, archetype::Radar, archetype::Launcher,
             archetype::RedDrone, archetype::Interceptor, archetype::Track,
             archetype::Blast>>;
```

- **`Spatial` is the world's only configuration concept.** It needs a distance
  (which may carry units, and only needs to be ordered), a pose, and plain
  coordinates for the spatial index. `coordinate_length` converts a distance
  into the same unit as the coordinates; `Kinematics` uses meters. The toolkit ships `Kinematics`, a local Cartesian 3D model, as
  the default, so interop layers share one notion of position unless an
  application opts out. Other models (geodetic for DIS, 2D, a grid, a network
  where distance is hop count) fit without changing the framework.
- **Every world is built by a builder,** which `World::set_up()` returns. The
  world's type says what it can hold; the builder says how much, so the world
  is decoupled from how it is configured, and its configuration is a private
  detail of the two:

  ```cpp
  World world;  // Empty: it holds nothing until built.
  std::expected<void, Status> built =
      World::set_up()
          .numbered(1)
          .holding<archetype::RedDrone>(drones)
          .holding<archetype::Track>(drones)
          .holding<archetype::Blast>(drones + interceptors)
          .build(lib::Out(world));
  ```

  - `holding<A>(n)` says the world holds `n` more entities of archetype `A`
    alive at once; holdings add up. The entity capacity is the total, and each
    component's store holds as many as every archetype that requires or allows
    the component, so `Tracked` is sized for red drones and `Radar` for radars,
    instead of every store for every entity. Capacity is pooled, not reserved
    per archetype: one archetype may use another's slack.
  - The spatial index sizes its cells at every rebuild to how densely the
    entities lie (see below). `cells_of(size)` fixes the cell edge instead,
    converted with `coordinate_length`.
  - `build(lib::Out(world))` fills the caller's world, discarding everything it
    held, and returns `std::expected<void, Status>`. It refuses a given cell
    size that is not positive (`CELL_SIZE_INVALID`) and holding more than a store's
    32-bit slots can index (`CAPACITY_TOO_LARGE`), and a refused plan leaves
    the world as it was.
  - **A world never moves.** Builders, `ProjectedWorld` and domain builders keep a
    pointer to it and its stores never reallocate, so nothing that refers to a
    world can dangle while it lives. Its default constructor makes an empty
    world, and the builder fills it in place through a private `initialize`,
    the one description of how a configuration makes a world.
  - Applications hold their world as a member and build it in the `configure`
    phase, so a plan too big for a world fails that phase with the builder's
    Status: missile's `build_world(scenario, Out(world))` says how many of each
    archetype a scenario holds.
- **The archetype list says what the world can create,** and orders each
  store's segments. It is checked against the component list at compile time:
  every archetype's components must be in it, and `create<A>()` of an archetype
  not in it does not compile. Archetypes are declared before the world, since
  they name only components.
- **The component list is closed per build.** `world.store_of<T>()` resolves at
  compile time, nothing is type-erased, and the inliner sees every hot-path
  call. An application cannot declare a component type at runtime, which none of
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

The spatial index is a uniform grid, in `framework/spatial_index.hpp`:

- **Cells hash into a fixed table of buckets,** so the grid is unbounded and
  its memory follows its capacity, not the space it covers.
- **A rebuild is a counting sort** into one flat array of entries, each a
  point's coordinates beside its slot in the spatial store. A query reads a few
  runs of that array and never touches store data for points it rejects.
  Nothing allocates after construction.
- **`within(center, radius, visit)`** visits the cells the radius covers,
  clipped to the cells that hold points. When that box has more cells than
  there are points, it reads every entry instead.
- **`nearest(center, radius, accept)`** searches outward in rings of cells.
  Each cell is skipped when its nearest face is farther than the best match so
  far, and the search stops when a whole ring is. Ties go to the lowest slot.
- **Queries are deterministic:** cells are visited in a fixed order, and a
  bucket's entries in the order the rebuild gave them, which for the world's
  index is its walk of the spatial store, segment by segment.

The world rebuilds the index lazily, on the first query after anything could
have moved:

- A system that writes the spatial component marks it stale, because the
  scheduler hands it the mutable store. No system can write the spatial
  component and query space in the same run; the existing rule against writing
  and reading one component already forbids it.
- A sync that attaches, detaches or destroys a spatial entity-component marks
  it stale.

So no schedule entry maintains it, and a query always sees the positions of the
last sync. Queries are not const on the world, because they may rebuild.

The grid is also a toolkit class that a system can own over positions that are
not the world's spatial component. `ProposeEngagements` indexes track
estimates this way (see [missile](#missile)).

Measured with the missile benchmark over the same 100 steps at 100,000 drones
(ms per step):

| | Total | ScanRadars | ProposeEngagements |
|---|---:|---:|---:|
| Linear scans | 516 | 51 | 462 |
| Grid, ring search only | 430 | 84 | 342 |
| Grid, skipping cells beyond the best match | 162 | 85 | 73 |

`ScanRadars` got slower. That benchmark scaled radars with drones inside a
fixed band, so every radar's 4 km range covered nearly every drone and the grid
filtered nothing, while visiting in grid order cost random store accesses.
Compare runs only over the same number of steps (`--steps N`): radars scan once
a second, so averages over different windows mix different amounts of scanning.

The benchmark now keeps density constant instead. `Scenario::sites` builds
copies of the whole site (asset, radars, launchers, drones) on a grid 20 km
apart, and the benchmark uses one site per 1,000 drones, each with 10 radars
and 50 launchers. One site builds exactly the single-site scenario. Over 500
steps, after archetype segments:

| Drones | Sites | ms per step | ns per entity-step |
|---:|---:|---:|---:|
| 1,000 | 1 | 0.039 | 18.0 |
| 10,000 | 10 | 0.651 | 32.2 |
| 100,000 | 100 | 6.885 | 34.3 |

From 10,000 to 100,000 drones the cost per entity is nearly flat; the step
from 1,000 is the working set leaving the core's caches.

`UpdateTracks` did not scale at first: every radar scans on the same steps,
and each track checked every scanning radar in the world, so it went from
0.011 to 0.824 ms per step for ten times the population. Its `prepare` now
indexes the step's scanning radars in a `SpatialIndex` with 4 km cells (about
a radar's range), and each track asks for the nearest one whose range covers
its target. That takes 0.018 and 0.218 ms per step, growing with the
population. At 10,000 drones the old loop over 100 radars was cheaper; a query
has fixed cost. With 1 km cells the query visited dozens of empty cells and
took 0.499 ms at 100,000 drones, so cell size matters as much as the index.

So the index picks it. A `SpatialIndex` built without a cell size sizes its
cells at every rebuild for about one point per cell over the box the points
span, counting only the axes they spread along, so points on a plane get
square cells and height is ignored. Choosing costs one pass over the points.
`ProposeEngagements` had hard-coded 250 m cells for its track index. A
launcher wants the nearest track nobody has engaged, which is often near its
3 km range, so each search crossed about 540 cells and found about 57 tracks
in them. With cells sized to the tracks it went from 0.635 to 0.268 ms per
step at 100,000 drones, and from 0.054 to 0.021 ms at 10,000. The radar index
already had a good size by hand and did not change. The world's own index sizes
itself the same way unless `cells_of` fixes it.

Under contention (`missile_benchmark --contend=N`, N threads each streaming
over 256 MB), over 200 steps including the first radar scan, ms per step and
the slowdown against an idle machine:

| Contending threads | 10,000 drones | 100,000 drones |
|---:|---:|---:|
| 0 | 1.20 | 9.4 |
| 4 | 1.82 (1.5×) | 40.8 (4.3×) |
| 16 | 14.7 (12×) | 141 (15×) |
| 31 | 30.6 (26×) | 466 (49×) |

- **Light contention separates the sizes.** With 4 threads, 10,000 drones slow
  1.5 times and 100,000 slow 4.3 times. The smaller working set is probably
  still mostly in the 32 MiB L3; the larger one is several times bigger and
  depends on memory bandwidth, which the neighbors take. This is the bar in
  step 5: at scale the simulation is bandwidth-bound, so the bytes each entity
  touches per step are what to reduce.
- **At 16 and 31 threads the benchmark also competes for cores.** The machine
  has 32 hardware threads, so those runs measure CPU contention as well as
  memory contention, and overstate a realistic neighbor. 4 to 8 threads is
  probably closer.
- **This window runs slower than the 500-step table above,** because it
  includes the step where every drone gets a track: 9.4 ms idle at 100,000
  drones here, against 6.3 ms over 500 steps.

Entities replicated from another process enter the same indexes, so a spatial
query finds a red drone whether red is simulated locally or remotely.

#### Several worlds

A world is an object, so a process can hold several. One use stands out for
the missile application: the blue side's track picture as its own world, holding tracks built
from sensor reports and queried the same way as ground truth. It is a natural
home for perception error later.

An entity belongs to exactly one world. References across worlds use names,
the same rule as references across processes.

Only one world type, the Cartesian spatial world, is built until a second is
needed.

### Determinism

A run is reproducible from its scenario and seed. That requires:

- A fixed schedule order (the schedule type).
- A stable iteration order within a system. Systems walk segments in archetype
  order, and a segment changes order only on erasure, which happens only at
  sync points, so the order depends only on the history of commands.
- Commands applied in recorded order.
- Integer time (see below).
- No iteration over unordered containers on the hot path.
- Random numbers drawn from generators the simulation owns and seeds.
- Spatial queries that visit in a fixed order and break ties by store position.
- A checkpoint that saves every store's order as it is, so a resimulation from
  it iterates the same way. Layouts that reorder on a heuristic (such as sorting
  once a store is disordered) would also have to save their bookkeeping, one
  more reason archetype segments were chosen.

## Using the framework well

The applications are examples of how to use the framework, so they follow
these practices, and the framework tries to make each one the easy path. They
come from measuring the missile simulation at 100,000 drones on an idle and a
contended machine (see [Indexes](#indexes)). At scale the simulation is bound
by memory bandwidth, so most of them are about the bytes each entity touches
per step.

### Make components thin, and require what is always there

Size a component by what systems read together, not by what it means. Split
fields read every step from fields read rarely, and do not store data nothing
reads, such as a copy of a value another component already holds.

Archetype segments make splitting free. Components an archetype requires sit
at the same slot of their stores, so a system reaches a required sibling
without a lookup (see [Archetype segments](#archetype-segments)). A fat
component used to save lookups; with segments it only costs bandwidth, because
every system that reads one field streams the whole component.

```cpp
// Thin components an archetype requires: Steer, Integrate and the spatial index
// each stream only what they read.
struct Kinematics final { Position position; Velocity velocity; };
struct Orientation final { Quaternion orientation; };   // Only where needed.
struct Drone final : Archetype<"drone", Requires<Kinematics, Control, Health>> {};
```

### Allow only what comes and goes

Put a component in `Requires` when every entity of the archetype has it for
its whole life, and in `Allows` only when it is attached and detached during
life. Allowed components live in a sparse segment and are reached by lookup,
which is the right cost for something rare. `Tracked`, attached to a drone
when it is first seen, is allowed for that reason. A component every entity
always has, declared as allowed, costs a lookup on every step for nothing.

### Let the entity being written drive the loop, and query what is near it

A system writes only the entity it is called for (see
[Relations between entities](#relations-between-entities)). Pick the driving
component so that each entity's work is local: the entity asks an index about
the few things near it, never "for each X, visit every Y".

- **Build a small view once per step in `prepare`.** `UpdateTracks` indexes
  the radars that scanned this step, and `ApplyBlasts` collects the step's
  blasts, so each track or victim reads a short array or asks an index instead
  of walking a store.
- **Use the world's spatial index for the spatial component, and a system's
  own `SpatialIndex` for other positions,** as `ProposeEngagements` does for
  track estimates. Let the index size its cells: built without a cell size,
  it sizes them to the points at every rebuild. Hand-picked sizes were wrong
  both ways in missile: cells a quarter of the radar range made `UpdateTracks`
  twice as slow, and `ProposeEngagements`' 250 m cells made it 2.4 times
  slower than cells sized to its tracks.
- **Invert a loop when the side being written is the side with sparse work.**
  A sensor that visits every target in range, to change a few of them, is the
  inverted loop: the targets that need changing should ask the sensors. In
  missile, `DetectDrones` lets each untracked drone ask an index of the radars
  that scanned, then create its own track.

### Do nothing on steps with nothing to do

A system whose work is rate-gated (radar scans) or event-driven (blasts) should
not touch every entity on the steps in between. A `prepare` stage that returns
`bool` skips the per-entity loop when it returns false; `resolve` still runs.

```cpp
// Steps without a scan have nothing to detect.
auto prepare(SystemWorld& world) -> bool { return radars_.collect(world); }
```

### Keep the per-entity call small

The runner calls a system once per entity, and the compiler inlines that call
into the loop only if it is small. A call operator that also holds rare work,
such as builders that create or destroy, may not be inlined, and then every
entity pays for a function call. Keep the check every entity makes in the call
operator, and move the rare work into a function marked cold:

```cpp
auto operator()(SystemWorld& world, Entity self,  //
                const Warhead& warhead,           //
                const Kinematics* kinematics,     //
                const Target* target) -> void {
  const Kinematics* target_kinematics =
      target ? world.maybe_component_of<Kinematics>(target->entity) : nullptr;
  if (kinematics && target_kinematics &&
      within_distance(*kinematics, *target_kinematics, warhead.fuse)) {
    detonate(world, self, warhead, *kinematics);  // Rare.
  }
}

[[gnu::cold, gnu::noinline]] static auto detonate(...) -> void;
```

A check most entities fail should be cheap: `within_distance` compares squared
distances, so it takes no square root. `model::limit` does the same for the
common case, a command already within the limit, and takes a square root only
to scale one down. That took `SteerRedDrones` from 0.72 to 0.59 ms per step at
100,000 drones, and `GuideInterceptors`, which also limits its command, from
0.166 to 0.152 ms.

`TriggerWarheads` had both problems. perf showed its call operator as a
separate function, because the same function built the Blast. Moving the
builders out took it from 0.548 to 0.32 ms per step at 100,000 drones, and
comparing squares instead of calling `distance` took it to 0.23 ms, 2.4 times
faster overall. Systems whose rare work is small, such as `DropStaleTracks`,
were already inlined; check perf before splitting one.

### Measure each system, idle and contended

Time every system at the population you care about, on an idle machine and
under `--contend=N`, over the same number of steps, and check the bytes each
loop reads per entity (`framework::bytes_per_entity_v`).

Measure with both compilers too. Their inliners disagree, and a function that
is not inlined into a hot loop shows up as its own line in `perf report`. Two
cases so far:

- **Contract checks.** `lib`'s `CHECK_*` macros used to expand, at every check,
  into a `std::source_location`, a `std::format` and a `throw`. GCC discounts
  that cold branch when it decides what to inline; Clang does not, so it left
  small checked functions such as `InOut`'s `operator->` out of line, called
  several times per entity by `Integrate`. The failure now happens in one cold,
  never-inlined function, `lib::internal::do_contract_failure`, so a check is a
  compare and a call. At 100,000 drones, Clang's `Integrate` went from 0.96 to
  0.24 ms per step and its whole step from 3.92 to 3.14 ms; GCC's went from
  3.23 to 3.17 ms.
- **Hot framework helpers.** Clang declined to inline
  `SpatialIndex::visit_cell`, which a nearest search calls from several places;
  `[[gnu::always_inline]]` takes `ProposeEngagements` from 0.77 to 0.62 ms
  under Clang and changes nothing under GCC.

GCC, for its part, stopped inlining mp-units' `Position - Position` into
`SteerRedDrones` and `TriggerWarheads` when the site builder added call sites,
which costs it about 8%; that was accepted. With both fixes Clang runs the
100,000-drone step in about 3.0 ms and GCC in about 3.2 ms. Contention shows which
systems are bandwidth-bound: at 100,000 drones, four streaming neighbors slowed
`ScanRadars` 2.6 times but `TriggerWarheads` 12 times.

**Use `model::max`, `min` and `clamp` on quantities in hot code.**
`std::max` and its relatives compare through mp-units' `<=>`, and GCC 16
compiles that to branches and stack spills instead of one `maxsd`. When the
flight model moved from raw `double`s to quantities, one `std::max` on a speed
made `Fly` 8% slower, and forcing functions inline did not recover it.
`model::max`, `min` and `clamp` (in `model/units.hpp`) take quantities and
compare their numbers, so the code keeps its units. With them, the typed
flight model runs as fast as the `double` one did under GCC (7.24 against 7.25
ms per step at 100,000 aircraft), and within 2% under Clang. `Actuate` and
`FlyAutopilot`, which already clamped quantities, got 6% and 3% faster.

### Choose fidelity per archetype

Most entities in a large scenario need little fidelity, and a few need a lot.
Give each level its own archetype and systems, so every system walks one dense
segment and no entity branches on how accurate it is:

| Level | State | Use |
|---|---|---|
| Kinematic | `Kinematics` and a commanded acceleration in `Control` | Crowds, distant traffic, missile's drones |
| Point-mass flight path | Above, plus speed, flight-path angle, heading and bank that follow commands with a lag | Most aircraft and missiles |
| 6-DOF | Above, plus attitude, body rates and full aerodynamics, often under `Continuous` | The few entities whose handling matters |

Model fast dynamics away instead of integrating them with small steps. A
first-order lag advanced by its exact solution, `x += (u - x)(1 - e^(-dt/τ))`,
is stable at any step, and so is a filter discretized by the bilinear
transform. Promoting an entity to a higher level means destroying it and
creating it again as the other archetype, so it stays in a dense segment.

### What we do not recommend

Each of these trades architectural simplicity, our first goal, for gains the
practices above already give:

- Merging systems to save passes over a store.
- Groups or orderings the user has to manage.
- Packing several components into one struct to keep them together.
- Giving up units or `double` for speed.

### Bringing missile in line

The missile simulation followed these practices only partly, and changing it
to follow them is the measurement of what they are worth:

- **`Kinematics` is position and velocity (48 bytes, was 112).** Its
  orientation moved to a separate `Orientation` component, which nothing in
  missile needs, and the acceleration it stored, which nothing read, is gone;
  the commanded acceleration is in `Control`.
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

At 100,000 drones (ms per step):

| | Before | After |
|---|---:|---:|
| Idle, 500 steps | 6.33 | 2.87 |
| Idle, 200 steps | 9.49 | 3.01 |
| 4 contending threads, 200 steps | 42.4 | 5.53 |

The step is 2.2 times faster idle and 7.7 times faster under contention, where
it now slows 1.8 times instead of 4.5. The cost per entity is flat from 1,000
to 100,000 drones (12.9 to 13.4 ns per entity-step). Outcomes are unchanged.

The change also fixed a bug the old layout hid. When ten radars covered a drone
on the first scan, each created a track for it before trying to mark it
`Tracked`; the nine that lost destroyed theirs, but a destroyed entity's slot
is freed only at the next sync. At ten sites the first scan ran out of entity
capacity and silently tracked 5,000 of 10,000 drones. Each drone now creates at
most one track.

`framework::bytes_per_entity_v<System>` gives the bytes a system's loop can
read per entity: the owner, the driving component and every other component it
names. It is an upper bound, since a sibling the entity's archetype cannot have
costs nothing, and reads of other entities come on top. `missile_benchmark`
prints it beside each system's time:

| System | B/entity | System | B/entity |
|---|---:|---|---:|
| `SteerRedDrones` | 104 | `TriggerWarheads` | 88 |
| `Integrate` | 80 | `DropStaleTracks` | 24 |
| `DetectDrones` | 88 | `ResolveEngagements` | 72 |

`TriggerWarheads` first named both `Interceptor` (48 bytes) and `RedDrone`
(24), because either held a warhead's target, so its report said 152. A
`Target` component that both archetypes require now holds it, and the system
names `Warhead`, `Kinematics` and `Target`: 88 bytes. Its time did not change
(0.57 to 0.60 ms per step at 100,000 drones, within noise). The runner already
passed a null `Interceptor` to drones without reading it, so the drone loop
read about 104 bytes before; most of the remaining time is each drone looking
up its target's `Kinematics`. The gain is a simpler system and one less copy of
the target.

## Extensible edges

Large simulations connect to other input formats and co-simulators, so we
decide deliberately which edges are extensible and what polymorphism each uses.

The principle is templates inside the step, where code runs once per
entity-component, and runtime polymorphism at the edges, which are crossed once
per step or once at load. Builders sit between the two. They take type-erased
input from an edge, validate it, and emit typed commands.

| Edge | Crossed | Polymorphism | Why |
|---|---|---|---|
| Systems and schedules | Per entity-component | Static: `System<...>`, `SystemList<...>` | Hot path; must inline |
| World configuration (`Spatial`, component list) | Compile time | Static: `World<S, ...>` | Hot path; closed per build |
| Input formats (scenario files, other schemas) | At load | Runtime: a reader interface feeding builders | New formats without recompiling the framework |
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

There is no global clock. Current time always arrives through `Step`, from the
driver.

Time uses `std::chrono`, counted in `int64` nanoseconds:

```cpp
struct SimTime;   // tag only: no now(), no state
using Duration  = std::chrono::nanoseconds;
using TimePoint = std::chrono::time_point<SimTime, Duration>;
```

- **Integer time is exact,** which determinism needs, and it removes a class of
  substep drift bugs. simon already fixed one caused by accumulating `double`
  substeps. An `int64` of nanoseconds covers about 292 years.
- **`SimTime` is a tag.** It exists so that a simulation time point and a
  wall-clock time point are different types. `RealTimeDriver` handles both,
  and subtracting a `steady_clock` time point from a `TimePoint` does not
  compile. `SimTime` has no `now()`, because a global clock would give systems
  a hidden second source of time.
- **Durations convert to `double` seconds** at the point of use in physics,
  through the units library (see [Units](#units)).

Strictly, the standard asks a `time_point`'s clock parameter to meet the clock
requirements, which include `now()`. GCC, Clang and MSVC do not enforce it. If
that ever matters, `TimePoint` becomes a small type of our own wrapping
`nanoseconds` since the start of the run, with the same arithmetic rules.

These live in `lib/base/time.hpp` as `lib::SimTime`, `lib::Duration` and
`lib::TimePoint` (step 2 replaced the old `double`-second `SimClock`).

### Units

Every physical quantity carries its unit in its type. Time uses `std::chrono`.
Everything else uses [mp-units](https://mpusz.github.io/mp-units/) (2.5.0,
in the Bazel Central Registry):

```cpp
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

quantity<isq::length[m]> range = 20 * km;
quantity<isq::speed[m / s]> cruise = 40 * m / s;
auto travel = range / cruise;                      // a time quantity
std::chrono::nanoseconds as_chrono = to_chrono_duration(travel);
```

- **mp-units is heading into the standard.** It is the reference
  implementation of the ISO proposal for quantities and units, targeting
  C++29.
- **Quantity kinds.** Beyond meters against feet, it can tell a displacement
  from an altitude from a range where we want that strictness.
- **Affine positions.** `quantity_point` is to positions what `time_point` is
  to time. Two positions subtract to a displacement, and adding two positions
  does not compile.
- **`std::chrono` interop.** Durations and time points convert both ways, so
  `SimTime` plugs straight in.
- **Angles** (radians, degrees) help with the geodetic coordinates DIS needs
  later.

The costs are heavier templates, slower compiles and long error messages.
[Au](https://aurora-opensource.github.io/au) (0.6.0, in the Bazel Central
Registry) is the fallback if those costs hurt: C++14, fast to compile, good
errors, `std::chrono` interop, and less expressive.

**Vectors are the open question.** Our vectors are Eigen types, and units and
linear algebra do not mix easily:

| Approach | Problem |
|---|---|
| A quantity whose representation is an Eigen vector (`quantity<isq::displacement[m], Vec3>`) | Eigen's arithmetic returns expression types, which may need wrapping or `.eval()` |
| An Eigen vector of quantities (`Matrix<quantity<m>, 3, 1>`) | Eigen assumes one scalar type, and products change units (m × m = m²) |
| Units at the boundaries: components and APIs carry quantities; `model/` kernels unwrap to Eigen doubles | Always possible; costs some typing at each call site |

**Spike results (step 1).** The spike is on branch `spike/units-eigen`
(`spike/units/`).

- **A thin `Vec3` wrapper over Eigen works as a representation.** Every
  operator evaluates back to `Vec3`, so mp-units never sees an Eigen expression
  type. Plain `Eigen::Vector3d` fails to compile for that reason.
- **Generated code matches raw Eigen** at `-O2`: integration is identical
  instruction for instruction, and proportional navigation differs only in
  register allocation.
- **Compile time** rises from 1.5 s to 3.8 s for one translation unit, almost
  all of it the mp-units headers. Including only the headers needed brings the
  headers' share down to about 1.85 s.
- **Units at the boundaries saves nothing.** Components still need `Vec3` to
  hold vector quantities, and every boundary needs an unwrap call.
- **Gaps to fill ourselves:** mp-units 2.5 has no dot or cross product for
  vector quantities (about 15 lines), and `r / |r|` is a dimensionless scalar
  to mp-units, so unit vectors are written as a division by range at the end.
- **Clang 22 cannot compile ISQ quantity types** (`quantity<isq::velocity[m/s],
  ...>`), even over `double`. It is a Clang regression (llvm/llvm-project
  #175831, mp-units #798). Quantities in plain SI units (`quantity<m/s, Vec3>`)
  compile on both compilers.

**Decision: plain SI units, over a `Vector3` wrapper, everywhere.**
`model/units.hpp` holds the wrapper, the vector algebra (`dot`, `cross`,
`norm`), a `seconds()` conversion from `std::chrono`, and every quantity type as
an alias:

```cpp
using Length = quantity<meter, double>;
using Time = quantity<second, double>;
using Rate = quantity<one / second, double>;
using Displacement = quantity<meter, Vector3>;
using Velocity = quantity<meter / second, Vector3>;
using Acceleration = quantity<meter / square(second), Vector3>;
using Position = Displacement;   // From the world origin.
```

- Plain units compile on both GCC 16 and Clang 22, and catch the common bug:
  adding meters to meters per second does not compile.
- They do not tell a position from a displacement, or an altitude from a
  range. Positions are displacements from the world origin, not affine
  `quantity_point`s.
- Moving to ISQ quantity kinds once Clang is fixed changes the aliases in
  `units.hpp`, not the code that uses them.
- `framework` stays free of units: the `Spatial` concept only needs distances to be
  ordered, so `World::within` takes whatever `distance()` returns.

### Lifecycle

Every simulation follows one lifecycle, whatever drives it:

```
configure ─▶ initialize ─▶ step ─▶ step ─▶ ... ─▶ finalize
                             └── any step may return Stop
```

Each phase returns `std::expected<Flow, Status>` (`PhaseResult`), where `Flow`
is `CONTINUE` or `STOP`; `finalize` returns `std::expected<void, Status>`.
Errors are statuses and are never used as control flow. (The older engine
sometimes used `AbortedError` to mean "exit the loop".)

A simulation is anything with `step(const Step&) -> PhaseResult` (the
`Simulation` concept in `engine/lifecycle.hpp`). `configure`, `initialize` and
`finalize` are optional; a driver calls them when they exist. Where a driver is
in the lifecycle is a small enum, `Phase`: `NEW`, `RUNNING`, `STOPPED`,
`FINISHED`. It needs no state machine classes.

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

`engine/driver.hpp` implements the contract once, in `Driver`, which owns the
simulation's time and phase. The other drivers wrap it:

- **`BatchDriver::run(end)`** runs the whole lifecycle to `end`, or until the
  simulation stops, and returns the time reached. `finalize` runs even after an
  error.
- **`RealTimeDriver::tick()`** advances to wherever the wall clock has reached;
  an application calls it once per frame. `run()` ticks and sleeps until the
  simulation stops, for headless use. The wall clock is a template parameter,
  so tests drive it by hand.
- **`RealTimeDriver` can pause, resume and change speed** (`pause()`,
  `resume()`, `set_speed()`). Each re-anchors the wall clock at the
  simulation's current time, so paused wall time is never caught up and a new
  speed never makes simulated time jump.
- **Real-time runs are deterministic.** `RealTimeDriver` only ever targets whole
  multiples of the maximum step, so the wall clock decides when steps happen,
  never how long they are. A test checks that irregular wall-clock ticks take
  exactly the steps a batch run takes.

`hello` runs under `RealTimeDriver` at five times real time; its test runs the
same simulation under `BatchDriver` and checks that two runs end at the same
time with bit-identical state.

The contract will carry an optional lookahead, the promise that this
simulation will not produce events earlier than `now + lookahead`. HLA time
management needs it, so it arrives with `LockstepDriver`, which waits until a
second process exists.

### Rate gates

A component that works at a lower rate than the step (a radar scanning at
10 Hz on a 100 Hz step) holds a `RateGate` value. Its system asks the gate
whether to run this step.

`RateGate::fire(step)` returns a `Firing` when the gate fires, and nothing
otherwise:

- The gate fires on the first step it is asked about, then on each step that
  contains one of its period boundaries. Steps are half-open, `[time,
  time + dt)`.
- `Firing::elapsed` is the time since the gate last fired (zero the first
  time), never the driver's `dt`. The older simulator's `PeriodicStep` passed
  the driver's `dt`, which was wrong for any gated work.
- The catch-up policy is explicit. `CatchUp::SKIP` fires once and drops missed
  periods. `CatchUp::EVERY` fires once and reports, in `Firing::periods`, how
  many periods fell in the step, so the work can run once per period.
- **Gates can be staggered.** `RateGate{period, catch_up, first}` fires first
  on the step that contains `first`, and on each period after it. If every
  radar's gate fires on the same step, that step does all the scanning; giving
  each radar a different `first` spreads the work over the period. The gate
  already keeps its next firing time, so this adds no field.

### Continuous state

Most simulations at scale integrate each step once, with one evaluation of
what drives the motion: `Integrate` turns a `Control` into `Kinematics` by the
midpoint rule. That is the default, and it costs one pass over the state.

Some users want more accuracy for some entities, such as an aircraft whose
handling matters, and accept that it costs more. They opt in with a schedule
element, `Continuous`, in `framework/continuous.hpp` (its own Bazel target).
A simulation that does not list one pays nothing for it.

```cpp
using Dynamics = framework::Continuous<
    framework::RungeKutta4,
    TypeList<Kinematics>,                               // The continuous state.
    SystemList<Gravity, Aerodynamics, EquationsOfMotion>>;  // Its derivatives.

using AircraftSystems = SystemList<Sensors, FlightControl, Dynamics, CheckOutcome>;
```

- **A continuous component names its rate component** and can be advanced
  along it. The `ContinuousState` concept asks for `typename T::RateComponent`,
  `advance(state, rate, dt)`, and `rate + rate` and `weight * rate` on the
  rate. `advance` is a free function so a state that is not a vector space,
  such as an attitude quaternion, can use the exponential map and renormalize.
- **Only the integrator writes continuous state.** Derivative systems are
  ordinary systems that write rate components (or force components that
  another derivative system sums into a rate). `Continuous` fails to compile
  if one of them writes a component in its state list.
- **Each method is a Butcher tableau as a type:** `Euler`, `Midpoint` and
  `RungeKutta4`. For each stage the integrator sets every entity's state to the
  stage's trial state, in place, runs the derivative systems at the stage's
  time, and keeps the rates. It then advances every state from where it began
  the step by the weighted rates. One-stage methods keep no copies.
- **Every entity advances in lockstep.** A derivative system that reads another
  entity's state through its allow list (a seeker reading its target) sees
  that entity's trial state for the same stage, which is what a coupled
  integration needs.
- **Derivative systems run once per stage, so they must not plan structural
  changes.** There is no sync point between stages, and a contract check fails
  if one records a command. Stage time arrives in the usual `Step`, with
  `time` at the stage and `dt` the whole step.
- **Spatial queries see the stage's state.** The integrator writes the spatial
  component, which marks the index stale, so a derivative system that queries
  space rebuilds the index once per stage. Only simulations that query space
  from a derivative system pay for that.
- **An archetype opts in by having the rate.** One that requires the state
  and the rate is integrated, reaching the rate at the same slot. One that
  requires the state but cannot have the rate is skipped at compile time, so
  a cheaper fidelity level in the same world can advance the same state its
  own way. An archetype may not require the state and only allow the rate. An
  entity whose archetype only allows the state has its rate looked up, and
  keeps its state if it has none.
- **The integrator keeps its copies of state and rates itself,** sized to the
  state store's capacity on its first step. A Runge-Kutta 4 integrator over a
  48-byte state at 100,000 entities keeps about 24 MB and passes over the
  state about five times a step. That cost is the opt-in.

Multistep methods (Adams-Bashforth), which evaluate once per step but keep
rates from earlier steps, are not built. Their history must follow an entity
through swap-erase, so it will be a component the archetype requires
(`RateHistory<T, N>`), not the integrator's copies.

### Events

The event queue carries rare, discrete happenings: a launch, an intercept, an
entity created. It is not for anything that happens every step, which belongs
in components.

Events are delivered in time order, ties in publish order, with the event's own
time. simon's current `EventQueue` already does this. missile's timed weapons
holds use it: a timer starts each hold, and a second timer raises
`WeaponsHoldExpired` when it ends.

## Distribution

Nothing here is implemented yet. The framework is designed so that these can be
added later without changing the simulation's interface.

### Future use cases

- **Red and blue on different computers.** Each side simulates its own
  entities and sees the other side's as replicas.
- **DIS (IEEE 1278).** Entity State PDUs broadcast over UDP, dead reckoning
  for remote entities, Fire and Detonation PDUs, wall-clock pacing.
- **HLA (IEEE 1516).** A runtime infrastructure (RTI), a federation object
  model (FOM) of object classes and interactions, per-attribute ownership, and
  logical time management with lookahead.

### What the framework provides for them

| Need | Framework design |
|---|---|
| Who may write a piece of state | **Authority per component.** Each component on an entity is either owned locally or a replica of remote state. Per-component authority maps to HLA attribute ownership. A component is roughly an HLA attribute group. |
| Where state crosses a boundary | **Scatter and gather at step boundaries only.** Gather publishes locally owned components that changed. Scatter applies remote updates to replicas. |
| Identity across processes | **Names.** `Entity` values mean nothing outside their process. Each interop layer maps its external identity (a DIS `{site, application, entity}` triple, an HLA object instance name) to our names. Neither format enters the framework. |
| Time across processes | **`advance_to(T)` with lookahead.** It is the same shape as HLA's time advance request and grant. The barrier rule for peers is "advance to the minimum of the peers' times". |
| Remote entities between updates | **Dead reckoning as a system** that extrapolates replica components each step, using the DIS algorithms. |
| Discrete interactions | **Events** that carry the names or entities involved. They map to DIS Fire/Detonation PDUs and HLA interactions. |
| Coordinates | **A local Cartesian frame with a stated geodetic origin.** Conversion to geocentric coordinates (DIS) happens at the interop boundary. |

Transport, liveness heartbeats and any controller process live outside the
framework. Candidate libraries when we get there are Open-DIS for DIS and OpenRTI or
Portico for HLA.

## Repository layout

```
simon/
  framework/     Entity, ComponentStore, World, Spatial, names, builders, commands, Step, System, schedules
  engine/        Lifecycle, drivers, RateGate, EventQueue
  model/         Reusable physics: kinematics, guidance, flight paths, atmosphere, control blocks (free functions)
  application/
    hello/       Two bouncing balls, the first application
    missile/     Red drones against blue radars, launchers and interceptors
    flight/      Aircraft flying routes under an autopilot, at two fidelity levels
  documents/     This document
  2nd_party/lib  Shared core libraries (submodule)
```

The prototype (`framework/`, `component/` and the top-level `simulation.hpp`)
was retired in step 1. Its event queue moved to `engine/`.

## Applications

### hello

Two balls under gravity and wind that stop when they collide. It is the
smallest complete use of the architecture: a handful of components, a motion
schedule, a collision system and a real-time driver.

### missile

Red drones fly toward a protected asset. Blue radars track them, blue launchers
fire interceptors, and the run ends when red is defeated or the asset is
destroyed. `bazel run //application/missile -- <seed>` runs one scenario as
fast as possible and prints the outcome.

Components (`application/missile/components.hpp`):

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

Each component has an archetype in `missile::archetype` (asset, radar,
launcher, red drone, interceptor, track, blast).

Tracks are entities, with `Track`, `Estimate` and `Engagement` required by
their archetype. A drone creates its own track when a scanning radar covers it,
tracks update themselves from the radars that scanned their target, launchers
engage them, and later they are what an interop layer would publish.

Interceptors and red drones both carry a `Warhead`. An interceptor reaching its
target and a drone reaching the asset are the same event: a blast, applied to
every `Health` within its radius.

Schedule (`application/missile/systems.hpp`):

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

Operator commands (`application/missile/simulation.hpp`) are query forms. Each
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
  scanning_in_turn()` (`Scenario::radars_in_turn`, `missile_benchmark
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

`bazel run //application/missile:viewer -- <seed>` watches a scenario under
`RealTimeDriver`. An ImPlot map shows the asset, radars and launchers with
their coverage, red drones, interceptors, tracks at their estimated positions,
and explosions. A side panel shows time, counts, the asset's health and the
outcome, with pause (also Space), a speed slider and restart with a seed.
Blasts last a single step, so the viewer never sees one; it draws an
explosion wherever a drone or interceptor disappears. That is presentation
only and changes nothing in the simulation.

### flight

Aircraft fly closed routes of waypoints under an autopilot, spread over an
area that grows with their number so traffic density stays the same. It is
the application for flight control algorithms of the class JSBSim runs, and
it shows fidelity as an opt-in: most aircraft fly a single-pass model, and
those whose archetype opts in are integrated with Runge-Kutta 4.
`bazel run //application/flight -- <aircraft> <precise> <seed>` flies one
scenario headless and prints how many waypoints were reached.

The code is in three layers:

| Layer | Holds |
|---|---|
| `framework/continuous.hpp` | `Continuous`, generic over any `ContinuousState` |
| `model/` | `flight_path.hpp` (the point-mass state, its rate equations, a single-pass integrator and autopilot laws), `atmosphere.hpp`, `control.hpp` (lags, rate limits, PI control, tables) |
| `application/flight/` | The aircraft components and archetypes, the systems, the scenario and the simulation |

The model is point-mass flight path: an `AirState` of position, speed,
flight-path angle and heading, flown by commanding load factor, bank and
throttle. Lift is a load factor times weight, drag comes from a drag polar,
and thrust falls with the standard atmosphere's density. The atmosphere's
layers are in geopotential altitude, as the 1976 standard and JSBSim have
them, so its density is within 10^-5 of JSBSim's up to 20 km. `AirState` is the
world's spatial component, so no copy of the position is kept anywhere else.

Components (`application/flight/components.hpp`):

| Component | Holds |
|---|---|
| `AirState` | From `model/`: position, speed, flight-path angle, heading |
| `AirStateRate` | Its rate; only precise aircraft have it |
| `FlightControls` | The load factor, bank and throttle the airframe has actually reached |
| `Commands` | What the autopilot commands |
| `Airframe` | What the dynamics read: mass, wing area, drag polar, thrust |
| `Handling` | What the autopilot and actuators read: limits, roll rate, lags |
| `Autopilot` | The altitude, heading and speed it holds, and its throttle integral |
| `Route` | Four waypoints, the speed to fly them, the next one and how many were reached |

Schedule (`application/flight/systems.hpp`):

| System | Does |
|---|---|
| `FollowRoute` | Once a second, aims each autopilot at its route's next waypoint, and moves on within 3 km of it |
| `FlyAutopilot` | Ten times a second, turns targets into commands: bank for heading, load factor for altitude through a commanded flight-path angle, throttle for speed by PI control |
| `Actuate` | Every step, moves the controls toward the commands through exact lags and a roll-rate limit |
| `Fly` | Every step, advances each single-pass aircraft in one semi-implicit pass |
| `Precise` | `Continuous<RungeKutta4, TypeList<AirState>, SystemList<PointMassRates>>` for precise aircraft |

Decisions made while building it:

- **Fidelity is per archetype.** `Aircraft` and `PreciseAircraft` require
  the same components, and the precise one also requires `AirStateRate`.
  `Continuous` integrates only archetypes that can have the rate, and `Fly`
  excludes the rate, so the runner skips the precise segment whole. `Fly` first
  named the rate as an optional sibling and returned when it was present.
  Clang did not inline the call operator, so it paid a call per precise
  aircraft to return: 0.23 ms per step at 100,000, against 0.06 ms with GCC.
  Excluding takes both to nothing.
- **Guidance and control run at their own rates, gated once per system** in
  `prepare`, so the steps in between skip their loops. Per-entity staggered
  gates would spread the work, at the cost of a gate in each component and a
  check per entity per step.
- **Speed comes first.** The steepest climb the autopilot commands shrinks as
  the aircraft falls below its target speed, and is zero 20 m/s below it, so
  a long climb at altitude never trades away more speed than that.
- **No ground, no wind, no stall.** Routes stay between 3 and 9 km.

The test checks the autopilot (holding altitude, speed and heading, and
turning the short way to a waypoint behind), that a closed route is flown,
that a single-pass and a Runge-Kutta aircraft flying the same route end within
100 m of each other after five minutes, and that a scenario repeats exactly
from its seed. `accuracy_test` compares the model with JSBSim (see
[Accuracy against JSBSim](#accuracy-against-jsbsim)).

`bazel run -c opt //application/flight:flight_benchmark` flies 500 steps of
20 ms at each population, once with every aircraft on the single-pass model
and once with every aircraft on Runge-Kutta 4. GCC, ms per step:

| Aircraft | Single pass | ns per entity-step | Runge-Kutta 4 | ns per entity-step |
|---:|---:|---:|---:|---:|
| 1,000 | 0.07 | 68 | 0.17 | 167 |
| 10,000 | 0.72 | 72 | 1.87 | 187 |
| 100,000 | 7.25 | 73 | 19.1 | 191 |
| 100,000, 4 contending threads | 8.00 | 80 | 29.9 | 299 |

- **The single-pass model is bound by computation.** Cost per aircraft is
  nearly flat from 1,000 to 100,000, and four contending threads slow it only
  1.1 times. `Fly` is three quarters of the step, and three `sincos` calls are
  most of `Fly`.
- **Runge-Kutta 4 costs 2.6 times as much,** and is bound by memory under
  contention (1.6 times slower), because its copies of the start state and
  four stages' rates stream through memory every step.
- **Two changes took the single-pass step from 8.34 to 7.25 ms,** and the
  Runge-Kutta 4 step from 29.8 to 19.1 ms. The rates take each sine and
  cosine once, and `fly` gets the new velocity from the rate's derivative,
  not more trigonometry. `wrap` calls `std::remainder` only when a heading
  leaves [-π, π]. `StandardAirTable`, the atmosphere tabulated every 100 m of
  geopotential altitude (within a few parts in 10^5), replaced the power and
  exponential of `standard_air`; it saved 6% of `Fly`, and 20% of Runge-Kutta
  4, which evaluates the air four times.
- **Clang is about 13% slower** (8.3 and 22.7 ms at 100,000).

#### Accuracy against JSBSim

`accuracy_test` measures how far the point-mass model drifts from JSBSim's
737. `reference/jsbsim_737.py` trims the 737 at 6000 m and 200 m/s and flies
it for 640 s under a small autopilot of its own. It flies level, turns at 30°
of bank, climbs at 3°, makes a descending turn at 25° and accelerates to
220 m/s. Every 0.2 s it records where the 737 is, and the load factor, bank
and throttle a point mass needs to fly the same path. The test replays those
controls through `Fly` and `Precise` and compares the paths. Both see the same
controls, so simon's autopilot and actuators play no part, and the drift
belongs to the model and the integrator.

The replay keeps three effects out of the comparison:

- **The Earth.** Load factor and bank come from the rates of flight-path angle
  and heading that the 737 flew, so simon's flat, non-rotating Earth sees the
  same turns. Taken from JSBSim's forces instead, they need 0.056 m/s² less
  lift on average than a flat Earth expects. Gravity falling with altitude is
  0.012 m/s² of that, and the Earth's rotation and curvature most of the
  rest. Replayed open loop, that error puts the aircraft 10 km off in
  altitude after 640 s. In a running simulation the autopilot closes the loop
  and absorbs it.
- **Fuel.** The 737 burns 1.2% of its mass, and `Airframe` has a fixed one,
  so the throttle replays thrust per kilogram of the starting mass.
- **The engine.** The throttle is JSBSim's thrust along the velocity, scaled
  to simon's thrust law, so drag is the only force left to simon.

The airframe is the 737's mass and wing area, with a drag polar fitted to 100
JSBSim trims over the scenario's envelope (3 to 9 km, 160 to 240 m/s, level
and in turns up to 45° of bank). CD0 is 0.0224 and K is 0.0807, and the worst
trim is 9.7% off. The largest drift over the flight, which covers 130 km:

| Step | Model | Position | Altitude | Speed | Heading |
|---:|---|---:|---:|---:|---:|
| 20 ms | Single pass | 1.78 km | 51 m | 3.3 m/s | 1.2° |
| 20 ms | Runge-Kutta 4 | 1.77 km | 50 m | 3.3 m/s | 1.2° |
| 200 ms | Single pass | 1.85 km | 62 m | 3.6 m/s | 1.3° |
| 200 ms | Runge-Kutta 4 | 1.76 km | 50 m | 3.2 m/s | 1.3° |
| 1 s | Single pass | 2.05 km | 94 m | 4.5 m/s | 1.9° |
| 1 s | Runge-Kutta 4 | 1.66 km | 53 m | 3.2 m/s | 1.9° |

- **The drag polar is most of the drift.** Its speed error changes the turn
  rate, which bends the path away. In a turn the 737 holds more elevator
  against its pitch rate, and its elevator drag rises from 0.0027 to 0.0038 at
  30° of bank. A polar in CL alone cannot follow that. A polar fitted to level
  trims alone drifts 4.8 km. A Python copy of the replay that takes JSBSim's
  own drag drifts 55 m, so the equations and integrators are not the limit.
- **Runge-Kutta 4 buys little at the steps simon runs.** At 20 ms the two
  models differ by 10 m against a drift of 1.8 km. At 1 s Runge-Kutta 4 holds
  altitude to 53 m against 94 m. A better drag model would be worth more than
  either integrator.

The test holds the drift at 20 ms and 1 s with about 25% headroom. To
regenerate the reference, install JSBSim's Python package and NumPy, and run
`python application/flight/reference/jsbsim_737.py`. It prints the fitted
airframe, which the test keeps as constants.

#### Rigid aircraft

The rigid aircraft archetype is the highest fidelity level: six degrees of
freedom, flown by control surfaces, engines and fuel, from an aircraft
described as data. It is opt in. Point-mass aircraft in the same world cost
the same as before (7.32 against 7.27 ms per step at 100,000), because each
rigid system is driven by a component only rigid aircraft have, and `Fly`
excludes rigid bodies.

An aircraft comes from JSBSim. `tools/jsbsim/convert.py` turns a JSBSim
aircraft's XML into simon's aircraft format, in SI units, and
`model/aircraft_data` reads it back, refusing bad files with the line at
fault. A converted aircraft holds its metrics, mass balance, fuel tanks,
turbines, flight control system and aerodynamics. The 737 is
`application/flight/aircraft/737.aircraft`.

| Layer | Holds |
|---|---|
| `model/aerodynamics` | The coefficient build-up: terms of a constant, state variables and tables, summed by axis, and turned into body loads about the center of mass |
| `model/flight_control` | Flight control blocks over named signals: summers, gains, scheduled gains, surface scales and kinematic actuators |
| `model/turbine` | JSBSim's turbine: spools, thrust from idle to military, fuel flow |
| `model/earth` | The WGS84 ellipsoid, J2 gravitation, the Earth's rotation, geodetic conversion |
| `model/rigid_body` | Stevens and Lewis's equations of motion in an inertial frame, as a `ContinuousState` |
| `model/rigid_aircraft` | The flat or round Earth, air data, mass balance, engines and fuel, and the body's rate |
| `application/flight` | The archetype and its systems |

Each step a rigid aircraft runs `RunFlightControls`, `RunEngines`, then
`Rigid` (Runge-Kutta 4 over `RigidAircraftRates`), then `BurnFuel` and
`FollowRigidBody`, which keeps its `AirState` on its body for the rest of
the world. The Earth is flat unless the systems are built with a round
one; round, the inertial frame is ECI and the world's local frame is the
plane tangent to the ellipsoid at an origin.

Each layer is checked against JSBSim's 737 (see
`application/flight/reference/README.md`):

| Test | Checks | Agreement |
|---|---|---|
| `aero_test` | Aerodynamic forces and moments at 240 states, from ground effect to Mach 0.94 | 5e-14 |
| `rigid_body_test` | Equations of motion, air data and mass balance at 480 states, at the equator and 60° north | 7e-16 to 5e-13; see below |
| `turbine_test` | Spools, thrust and fuel flow, frame by frame through throttle steps | 1e-13 |
| `flight_control_test` | Surface positions, frame by frame through command sweeps and extensions | 4e-15 |
| `check_case_test` | The whole aircraft, open loop for 30 s from JSBSim's trim | See below |

The check cases fly from JSBSim's trim at 6 km, 30° north, through a trim
hold, elevator, aileron and rudder doublets, and a throttle step, round the
turning Earth. The reference is JSBSim at 0.5 ms, where its integrators and
frame lags have converged. The largest distance from it over 30 s:

| Case | simon at 0.5 ms | simon at 8 ms | JSBSim at 8 ms |
|---|---:|---:|---:|
| Trim hold | 2.1 mm | 2.8 mm | 1.6 mm |
| Elevator doublet | 0.9 mm | 1.3 mm | 8.9 mm |
| Aileron doublet | 2.2 mm | 6.1 mm | 11.7 mm |
| Rudder doublet | 2.4 cm | 3.8 cm | 41.5 cm |
| Throttle step | 3.9 mm | 6.0 cm | 11.9 cm |

simon's physics and JSBSim's agree to millimeters, the first column. At the
same step, simon's Runge-Kutta 4 is closer to the converged answer than
JSBSim's mixed Euler and Adams-Bashforth integrators in every case with an
input, by 11 times after the rudder doublet.

Matching JSBSim is how simon's physics is checked, and it is the starting
point. Where JSBSim takes a shortcut, simon does not, and the difference is
measured:

- **No frame lags.** JSBSim's induced drag reads the frame before's lift
  coefficient, its rate of angle of attack the frame before's acceleration,
  and its flight controls the frame before's air data. simon sums lift
  before drag and forces before moments, so all three are the step's own.
- **Exact geodetic altitude.** JSBSim's is a one-step approximation, 2.5 cm
  off at 60° north and 6 km up. simon's is Heikkinen's closed form, which
  agrees with an iteration run to convergence.
- **Exact units.** JSBSim turns pounds into slugs by a rounded 32.174049,
  1.4e-8 off, and keeps its atmosphere's constants in English units, 8.5e-6
  off in density. simon uses the definitions and the 1976 standard.

`bazel run -c opt //application/flight:rigid_benchmark` flies rigid 737s
at 8 ms steps, each on its own. GCC, per aircraft per step:

| Aircraft | Flat Earth | Round Earth |
|---:|---:|---:|
| 100 | 1.83 µs | 2.80 µs |
| 1,000 | 1.77 µs | 2.82 µs |
| 10,000 | 1.77 µs | 2.80 µs |

- **The cost is flat with population,** so one thread flies about 4,500
  rigid 737s in real time over a flat Earth, and 2,800 round one. `Rigid`
  is three quarters to four fifths of it: four stages, each building up the
  aerodynamics.
- **JSBSim takes 9.2 µs a frame** for one 737, timed through its Python
  module from its own trim, with one instance per aircraft. Its frame does
  work simon's does not, such as ground reactions and its property tree, so
  the comparison is rough; simon evaluates the aircraft four times a step to
  JSBSim's once and is still 3.3 times faster round the Earth.
- **The round Earth costs 60% more.** Each stage finds the body's place on
  it once, its geodetic position, local frame and the Earth's turn, in
  `Earth::place`, which took the round Earth from 3.57 to 2.81 µs.

Building it turned up JSBSim behaviors that a comparison has to allow for,
all noted where they matter: a frame starts by moving the state on, so
everything read after a frame belongs together; its mass balance runs
before its engines burn; its `inertia/ixy` and `iyz` properties are the
tensor's elements negated but `ixz` is not; and its kinematic actuators keep
the frame time the model loaded with.

#### Mixed fidelity

Every level flies in one world. A scenario's `rigid` count makes some of
its aircraft rigid 737s, and they fly the same kind of routes as the rest:
`FollowRoute` sets their autopilot's targets as it sets every aircraft's.
`bazel run //application/flight -- <aircraft> <precise> <rigid> <seed>` flies
one, over a flat Earth so that every level shares the world's frame.

Rigid aircraft start from JSBSim's trim in cruise (`RigidTrim`) and fly their
surfaces with `FlySurfaces`, an autopilot kept small on purpose. It takes the
point-mass autopilot's laws for the bank a heading needs, the flight-path
angle an altitude needs and putting speed first, and flies them with three
lines: aileron from the bank error with roll damping, elevator from the
flight-path angle error with pitch damping, the pull a turn needs and a
bounded integral, and throttle from the speed error about the trim. It
banks up to 45°, so a 737 at 200 m/s turns on about 4 km, near the 3 km at
which `FollowRoute` captures a waypoint.

At the world's 20 ms step, rigid aircraft stay within 15 cm of the converged
JSBSim reference after the 30 s check cases, and within 9 cm after the rudder
doublet, against JSBSim's 41.5 cm at 8 ms. Over 10 minutes of routes they
keep between 3 and 8.3 km and between 197 and 236 m/s, and reach about 4.4
waypoints each to the point-mass aircraft's 6.5: a 737 turns wider than the
point-mass jet.

`flight_benchmark` adds a mixed population, 1% on Runge-Kutta 4 and 0.1%
rigid. At 100,000 aircraft, GCC, 20 ms steps:

| Aircraft | Systems | ms per step | Share |
|---|---|---:|---:|
| 98,900 single pass | `FollowRoute`, `FlyAutopilot`, `Actuate`, `Fly` | 7.17 | 95.2% |
| 1,000 Runge-Kutta 4 | `Continuous(AirState)` | 0.17 | 2.3% |
| 100 rigid 737s | `FlySurfaces` to `FollowRigidBody` | 0.19 | 2.5% |
| All | | 7.53 | |

One thread runs it 2.7 times faster than real time. Each level costs what
its own aircraft cost, and nothing more: the single-pass aircraft run as
fast as they do alone (7.25 ms at 100,000), because each level's systems are
driven by components only its aircraft have.

## Libraries

| Need | Library |
|---|---|
| Linear algebra | Eigen (have) |
| Units | `std::chrono` for time; mp-units in plain SI units for everything else |
| Tests | Catch2 (have) |
| Benchmarks | A small `std::chrono` harness per benchmark, printing one table per question. Benchmarks sit beside what they measure, like tests; only ones that measure several things go in a common directory. |
| UI | Dear ImGui, and ImPlot (0.17) for the missile viewer's map |
| Window and input | SDL2 now; SDL3 when it is in the Bazel Central Registry |
| Profiling, later | Tracy |

C++26 reflection would eliminate some boilerplate, but GCC 16 supports it and
Clang 22 does not. We will revisit it when both compilers do.

## Roadmap

Each step ends with a working application and passing tests.

1. **Framework (done).** `Entity`, `ComponentStore`, `World`, systems, schedules, builders and
   commands, with tests. Benchmark the dense store against the stable-slot
   store at 1k, 10k and 100k entities with 0–75% churn, for iteration and
   random lookup, and record the choice here. Run the units spike (mp-units
   quantities over Eigen vectors) and record that choice too. Port `hello` onto
   the framework and retire the prototype.
2. **Drivers (done).** Replace `lib::SimClock` with the `SimTime` tag and `int64`
   nanoseconds. Add `Step`, the lifecycle, `advance_to`, `BatchDriver`,
   `RealTimeDriver` and `RateGate`. Run `hello` under the real-time driver.
3. **Missile, headless (done).** The components and schedule above, with a
   `BatchDriver` test that checks a deterministic outcome for a fixed seed.
4. **Missile viewer (done).** An ImGui and ImPlot view under `RealTimeDriver`.
5. **Performance (in progress).** The bar is not "thousands of agents on the
   development machine". It is scale on cloud machines whose cache and memory
   bandwidth are contended by other heavy loads, and we have not met it yet.
   Benchmarks run idle and under `--contend`.
   - Done: `missile_benchmark` (per-system cost at 1k to 100k drones, at
     constant density),
     prepare-time indexes in `UpdateTracks` and `ResolveEngagements`, the
     spatial index, `system_benchmark` (the cost of reaching a sibling) and
     `churn_benchmark` (layouts under churn).
   - Done: archetype segments (see [Stores](#stores)).
   - Done: the missile simulation measured idle and contended (see
     [Indexes](#indexes)). At 100,000 drones, 4 contending threads slow it
     4.3 times.
   - Done: missile follows the practices in
     [Using the framework well](#using-the-framework-well): 2.2 times faster
     idle and 7.7 times faster under contention at 100,000 drones.
   - Done: `bytes_per_entity_v`, reported per system by `missile_benchmark`.
   Consider struct-of-arrays layout inside hot components only if measurements
   call for it.
6. **Flight dynamics (in progress).** Run flight control algorithms of the
   class JSBSim runs, at scale, with fidelity as an opt-in (see
   [Choose fidelity per archetype](#choose-fidelity-per-archetype)).
   - Done: staggered rate gates.
   - Done: control blocks in `model/`: exact first-order lags, rate limits,
     PI control, and lookup tables with linear interpolation.
   - Done: `Continuous` with Euler, midpoint and Runge-Kutta 4 (see
     [Continuous state](#continuous-state)). missile, which does not use it,
     runs as before: 2.31 against 2.32 ms per step at 100,000 drones.
   - Done: the standard atmosphere, a point-mass flight-path model, and the
     [flight](#flight) application flying it at two fidelity levels.
   - Done: `flight_benchmark`, idle and contended, for both levels (see
     [flight](#flight)).
   - Done: accuracy against JSBSim's 737 (see
     [Accuracy against JSBSim](#accuracy-against-jsbsim)).
   - Done: rigid aircraft, six degrees of freedom from JSBSim aircraft
     converted to data, round a WGS84 Earth or over a flat one, checked
     against JSBSim layer by layer and whole (see
     [Rigid aircraft](#rigid-aircraft)).
   - Done: every level in one world, rigid aircraft flying routes by a small
     surface autopilot (see [Mixed fidelity](#mixed-fidelity)).
   - Next for rigid aircraft: a trim of simon's own, more JSBSim aircraft
     and the flight control components they need, and many rigid aircraft
     batched in one segment.
   - Later: Adams-Bashforth with rate history, many replicas of a scenario
     in one world, world snapshots, and trim tables computed offline. JSBSim,
     run offline, stays the reference each level's accuracy is measured
     against.

Later: `LockstepDriver` and a second process, scenario files with two-phase
loading, parent-child transforms (a radar mounted on a vehicle), and DIS or HLA
interop.

## Open decisions

- **ISQ quantity kinds** (and affine positions) once a Clang release compiles
  them.
- **Segment chunk size.** At most 1,024 entries today; to be measured with
  large components, where a chunk spans a megabyte.
- **Hierarchical locality names** (`/blue/radars`) for ordering segments,
  once a system walks several archetypes together.
- **A realistic contention load.** `--contend` runs a streaming thread on every
  spare core, close to the worst case on the development machine, and
  `--contend=N` runs N. Which N resembles a busy cloud neighbor is still to be
  decided; 4 to 8 on the development machine avoids also taking its cores.
- **Step rates that divide a second.** Time is integer nanoseconds, so 120 Hz
  (8,333,333.3 ns) is not exact, and rate ratios such as 120 to 40 Hz drift.
  Rates that divide 10^9 (100, 125, 200, 250, 500 Hz) are exact. Whether to
  support others with a rational step is undecided.

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
| Multi-process state machines and liveness inside the engine | A small lifecycle enum and `advance_to` in the framework; transport outside it | Keeps what every simulator needs and leaves deployment concerns at the edge. |
