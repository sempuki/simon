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

We prefer existing, modern libraries for anything outside the framework. The framework
(entities, stores, systems, schedules and drivers) is ours, because its layout
and handle rules are the point of the project.

## Conventions

**Reference arguments are marked at the call site** with `lib`'s wrappers:

| Wrapper | The callee | Call site |
|---|---|---|
| `Out<T>` | writes the argument, and may ignore it | `parse(text, Out(result))` |
| `InOut<T>` | reads and writes the argument | `scheduler.step(InOut(world), step)` |
| `Depend<T>` | keeps a reference that can dangle | `WorldAccess{Depend<World>{world}}` |

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
`try_component_of` (the `try_` form returns a pointer that may be null),
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

## Architecture

The architecture is idiomatic ECS with no exceptions:

| Concept | Rule |
|---|---|
| Entity | An identity and nothing else: `{index, generation}`. It holds no data and no pointers. |
| Component | Plain data. No behaviour, no virtual functions, no base class required. Each component type has a debug name. |
| Store | Exactly one per component. It is the array of that component's entity-components. |
| System | Every non-trivial function is a system. A system iterates component arrays through stores. |
| Builder | The user-facing language for asking the simulator to do something specific. A builder turns a request into commands. |
| Command | A typed, low-level structural change: create, attach, detach, destroy. Commands are applied to stores. |
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
- **Append.** Push onto `data` and `owner`, and write the index entry.
- **Erase.** Move the last element into the gap, fix the moved element's
  index entry, and clear the erased one. The array stays dense after every
  erasure.
- **Look up.** `index[e.index]` checks the generation, then reads `data`. The
  index for 5,000 entities is 40 KiB and stays hot.
- **Capacity.** Every array is allocated once, at a capacity chosen when the
  world is created. Stores never reallocate, so references into them stay
  valid until the element is erased or moved by an erasure.

A hole now costs 8 bytes in the index, instead of a whole component in the
array.

**Decided by measurement: dense.** The alternative was a stable-slot store,
where each entity index has a fixed slot, nothing moves, and a lookup is one
load, but destroyed entities leave holes. `framework/store_benchmark.cpp`
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
`bazel run -c opt //framework:store_benchmark`.

### Handles and references

A handle has three duties, carried over from the older simulator:

1. Hide indices and addresses from the component programmer.
2. Never allow a memory error.
3. Never fail silently.

`Entity` is the handle. A system reads another entity's component through the
store:

```cpp
const Kinematics* target = world.try_component_of<Kinematics>(interceptor.target);  // null if gone
const Kinematics& target = world.component_of<Kinematics>(interceptor.target);      // contract check if gone
```

`try_component_of` is the explicit path for "this may have disappeared".
`component_of` fails a contract check on a stale entity, so misuse stops
loudly. `Store` has the same pair. Neither can read
freed memory, because store memory lives as long as the world.

Raw pointers or references returned by `try_component_of` or
`component_of` are only valid until the next
sync point. Systems must not keep them across steps.

Nothing registers handles or patches them. The older simulator's self-patching
handle met the same three duties, but every copy made a virtual call and a hash
insert to follow objects that reallocation had moved. Fixed capacity removes
the reallocation, and the generation check replaces the patching.

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
- **The world's number is given when it is constructed**
  (`WorldConfiguration::number`). There is no global counter; the older
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
  absent and its store has room, a component to detach is present, an alias is
  non-empty and not already given, an alias to take was given. A refused
  utterance returns a `lib::Status` whose condition is a `BuildError`
  (compare with `status == lib::watch(BuildError::ALIAS_NOT_GIVEN)`), and
  emits nothing. Because every command was validated this way, applying
  commands at a sync point cannot fail halfway through a batch.
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

Domain builders extend the same grammar with verbs of their own, for example
`world.create<Ego>("ego").with_missiles(4).at(site).build()`, or a query form
of destroy, `world.destroy().each<Team>(Team::Red).within(500 * m, of(asset))
.build()`. Builders and commands are not isomorphic: one utterance can emit
many commands, and a query emits a number that depends on the world at the
time. Domain verbs arrive when an application needs them.

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

Systems use builders too, through their `WorldAccess`:
`world.create<Interceptor>().under(self).with(...).build()`. Inside a system
whose access parameter is `auto&`, the free-function form avoids the
`template` keyword: `create<Interceptor>(lib::InOut(world))`.

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

struct GuideInterceptors : System<const Interceptor, const Kinematics, Control> {
  using SequenceAfterSystemList = SystemList<UpdateTracks>;
  using AllowComponentList = TypeList<Kinematics>;   // other entities, always read-only
  using LocalWorld = WorldAccess<GuideInterceptors>;

  void operator()(LocalWorld& world, Entity self,
                  const Interceptor& interceptor, const Kinematics* kinematics,
                  Control* control, Step step) const {
    if (!kinematics || !control) return;
    const Kinematics* target = world.try_component_of<Kinematics>(interceptor.target);
    if (!target) { ... }
    control->acceleration = model::proportional_navigation(*kinematics, *target, interceptor);
  }
};
```

The framework reads only the `System<...>` arguments. It walks the driving
store and passes each entity's optional entity-components:

```cpp
auto& drive = world.store_of<typename S::DrivingComponent>();   // const Store<Interceptor>& when const
for (std::size_t i = 0; i < drive.size(); ++i) {
  Entity e = drive.owner(i);
  system(access, e, drive.data(i), world.store_of<OtherComponentTypes>().try_component_of(e)..., step);
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
- **Other entities are reached through an allow list.** `using AllowComponentList =
  TypeList<...>;` opts the system in to reading those stores by entity.
  `world.try_component_of<T>(e)` returns `const T*`, `world.component_of<T>(e)`
  returns `const T&` with a contract check, and `world.store_of<T>()` returns
  the whole read-only store. Reads through the allow list are always
  read-only, and asking for a component not on it does not compile.
- **The call order is `(WorldAccess& world, Entity self, driving component,
  other components..., Step step)`.** `WorldAccess<S, W>` is the system's
  opt-in access to the world: its allow list, spatial and name queries, and
  builders. The `Step` (see [Time](#time-and-drivers)) is passed by value, on
  the stack, and is optional: the scheduler passes it only if the call
  operator takes it. Time is not world data, so it is not in `WorldAccess`.
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
optional ones. Everything it reaches through a `Store` lookup is read-only. To affect another entity, a system either uses a builder (create,
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
| One-to-one reference | Store an `Entity`; look it up read-only | An interceptor reads its target's `Kinematics` |
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
- **Pointers from `try_component_of` are valid only until the next sync point.** Debug
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
void run(SystemList<Ss...>, World& world, Step step) {
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
using Motion  = SystemList<Integrate, UpdateSpatialIndex>;
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
world.destroy(e).build()     sync points)          by space      world.within(center, radius), world.nearest(p)
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
Names to entities, the aliases, and any relationships between entities. The
built-in components `EntityArchetype` and `Parent` are always in a world's
component list; applications do not list them. Relationships are
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

using MissileWorld = World<Kinematics,
                           Team, Control, Health, Warhead, Blast,
                           RedDrone, Asset, Radar, Track, Launcher, Interceptor>;
```

- **`Spatial` is the world's only configuration concept.** It needs a distance
  and a pose. The toolkit ships `Kinematics`, a local Cartesian 3D model, as
  the default, so interop layers share one notion of position unless an
  application opts out. Other models (geodetic for DIS, 2D, a grid, a network
  where distance is hop count) fit without changing the framework.
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

Indexes are updated at defined points, so every query within a step sees a
consistent snapshot:

- Created and destroyed entities enter and leave the indexes at sync points.
- Moved entities are re-indexed by a world-provided system,
  `UpdateSpatialIndex`, which the schedule places after `Motion`. The world
  maintains its indexes, and the schedule says when.

Step 1 answers `within()` with a linear scan of the spatial store, which is
always current, so `UpdateSpatialIndex` does not exist yet. The real index and
its system arrive with the missile application, where the population makes a scan too slow.

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
- A stable iteration order within a system. Dense stores change order only on
  erasure, and erasure happens only at sync points, so the order depends only
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

**Decision: plain SI units, over a `Vector3d` wrapper, everywhere.**
`model/units.hpp` holds the wrapper, the vector algebra (`dot`, `cross`,
`norm`), a `seconds()` conversion from `std::chrono`, and every quantity type as
an alias:

```cpp
using Length = quantity<meter, double>;
using Time = quantity<second, double>;
using Rate = quantity<one / second, double>;
using Displacement = quantity<meter, Vector3d>;
using Velocity = quantity<meter / second, Vector3d>;
using Acceleration = quantity<meter / square(second), Vector3d>;
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

### Events

The event queue carries rare, discrete happenings: a launch, an intercept, an
entity created. It is not for anything that happens every step, which belongs
in components.

Events are delivered in time order, ties in publish order, with the event's own
time. simon's current `EventQueue` already does this.

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
  framework/     Entity, Store, World, Spatial, names, builders, commands, Step, System, schedules
  engine/        Lifecycle, drivers, RateGate, EventQueue
  model/         Reusable physics: kinematics, sensing, guidance (free functions)
  application/
    hello/       Two bouncing balls, the first application
    missile/     Red drones against blue radars, launchers and interceptors
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
| `Kinematics`, `Control` | From `model/` |
| `Health` | Hit points |
| `Warhead` | Fuse distance, blast radius, damage |
| `Blast` | Radius, damage, the warhead's Name; lives for one step |
| `RedDrone` | Target entity (the asset), cruise speed, agility |
| `Tracked` | Marks a red drone that has a track, and names the track |
| `Asset` | Marks the protected asset |
| `Radar` | Range, a `RateGate` for the scan, whether it scanned this step |
| `Track` | Target entity, estimated position and velocity, last seen, engaging launcher and until when |
| `Launcher` | Range, inventory, reload time, ready time, this step's proposal |
| `Interceptor` | Target entity, navigation gain, speed, agility, seeker range, flight time |

Each component has an archetype in `missile::archetype` (asset, radar,
launcher, red drone, interceptor, track, blast).

Tracks are entities. Radars create them, tracks update themselves from the
radars that scanned their target, launchers engage them, and later they are
what an interop layer would publish.

Interceptors and red drones both carry a `Warhead`. An interceptor reaching its
target and a drone reaching the asset are the same event: a blast, applied to
every `Health` within its radius.

Schedule (`application/missile/systems.hpp`):

| System | Does |
|---|---|
| `ScanRadars` | Each radar whose scan fires marks every untracked red drone in range `Tracked` and creates its track. If another radar marked the drone earlier in the step, the builder refuses the mark and the new track is destroyed. |
| `UpdateTracks` | Each track updates its own estimate from a radar that scanned its target. Radars are perfect for now. |
| `DropStaleTracks` | Destroys tracks whose target is gone or unseen for 5 s, and unmarks a surviving target |
| `ProposeEngagements` | Each ready launcher with inventory proposes the nearest unengaged track in range |
| `ResolveEngagements` | Each unengaged track accepts the nearest launcher that proposed it, for 30 s |
| `LaunchInterceptors` | Launchers whose proposal was accepted build an interceptor under themselves, aimed at the track |
| `GuideInterceptors` | Proportional navigation plus speed hold into `Control`. Retargets the nearest red drone within seeker range when the target is gone; self-destructs when there is none or its flight time is up. |
| `SteerRedDrones` | Steers drones at their target at cruise speed into `Control` |
| `Motion` | Integrates `Control` into `Kinematics` |
| `TriggerWarheads` | A warhead within fuse distance of its target creates a Blast and destroys itself |
| `ApplyBlasts` | Every `Health` inside a blast takes its damage, and is destroyed at zero |
| `ExpireBlasts` | Destroys every blast; they live for one step |

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
- **Entity lookups use a linear scan for now.** A few hundred entities do not
  need the spatial index; step 5 measures when they do.
- **Systems name their concrete access type** with a member alias,
  `using LocalWorld = WorldAccess<ThisSystem>;`, so builder
  calls with explicit template arguments, such as `detach<Tracked>()`, need no
  `template` keyword.

The test runs whole scenarios under `BatchDriver` (blue wins by default, red
wins without launchers or with too few interceptors, and the same seed repeats
exactly) and each rule on a small world (one track per drone however many radars
see it, one launch per contested track, reload, retargeting, self-destruct,
blast damage).

## Libraries

| Need | Library |
|---|---|
| Linear algebra | Eigen (have) |
| Units | `std::chrono` for time; mp-units in plain SI units for everything else |
| Tests | Catch2 (have) |
| Benchmarks | A small `std::chrono` harness per benchmark, printing one table per question. Benchmarks sit beside what they measure, like tests; only ones that measure several things go in a common directory. |
| UI | Dear ImGui (have), ImPlot for the top-down view |
| Window and input | SDL2 now; SDL3 when it is in the Bazel Central Registry |
| Profiling, later | Tracy |

C++26 reflection would eliminate some boilerplate, but GCC 16 supports it and
Clang 22 does not. We will revisit it when both compilers do.

## Roadmap

Each step ends with a working application and passing tests.

1. **Framework (done).** `Entity`, `Store`, `World`, systems, schedules, builders and
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
4. **Missile demo.** An ImGui and ImPlot view under `RealTimeDriver`.
5. **Performance.** Profile at thousands to hundreds of thousands of agents.
   Consider struct-of-arrays layout inside hot components only if measurements
   call for it.

Later: `LockstepDriver` and a second process, scenario files with two-phase
loading, parent-child transforms (a radar mounted on a vehicle), and DIS or HLA
interop.

## Open decisions

- **ISQ quantity kinds** (and affine positions) once a Clang release compiles
  them.
- **Spatial index structure.** A uniform grid is the likely start; to be
  measured with the missile application at scale.

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
