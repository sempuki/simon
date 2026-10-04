# automotive design

Vehicles drive a network of roads, closed loop at the level of objects:
traffic and a vehicle under test, without sensors. It proves the framework
can carry a driving simulation comparable to the best open source, as
aeronautic does against JSBSim, with each claim checked against an open
reference: roads against libOpenDRIVE, vehicles against CommonRoad's models
and Chrono::Vehicle, traffic against SUMO, scenarios against esmini, and
metrics against nuPlan. It was built in eight steps (see the
[Roadmap](#roadmap)): roads, traffic, a scale benchmark, tires and vehicle
dynamics against CommonRoad, maneuvers against Chrono, scenarios against
esmini, metrics and batch runs, and a viewer.

## Roads

`model/road` holds roads as ASAM OpenDRIVE describes them, and
`format/opendrive` reads them from OpenDRIVE files with pugixml. A road is a
reference line of lines, arcs, spirals and parametric cubics, with an
elevation, a superelevation and a lane offset along it, and lane sections
whose lanes have widths that are cubics in s. A point on a road is (s, t, h):
s along the reference line in the plane, t across it to the left, h up from
the surface. simon finds a point's position, each lane's borders, the lane
at a point, and the road coordinates of a position. It reads what geometry
needs, and refuses the deprecated poly3 and lanes given by borders, as
libOpenDRIVE does.

- **Each geometry is evaluated exactly.** A line and an arc are in closed
  form, the arc's chord written so it holds as its curvature goes to zero. A
  spiral's position is the integral of its tangent, whose heading is
  quadratic in s: the Fresnel integral, by 8-point Gauss-Legendre quadrature
  in parts that turn at most half a radian, where the rule's error is far
  below rounding. It matches the Fresnel integrals' tabulated values to
  1e-15.
- **s is arc length on a parametric cubic too.** A paramPoly3's s is its arc
  length, found by quadrature of its speed and Newton's method. A file's
  length and the curve's arc length often differ a little, so s runs over
  the curve in proportion to its arc length, from its start at 0 to its end
  at the file's length.
- **Lane borders sum the lanes' widths** from the center out, each evaluated
  at s, plus the lane offset.
- **The lane graph follows travel.** `model/lane_graph` links each lane to
  the lanes traffic moves into from it: across lane sections, across road
  links by their contact points, and from a junction's incoming lanes into its
  connecting roads, as OpenDRIVE's links give them. Traffic keeps right, so a
  lane right of the reference line runs with s and one left of it against s.
  A lane's successors are a contiguous run of a sorted array, found by binary
  search.
- **Road coordinates are found by projection.** Each piece is sampled every 2
  m and every tenth of a radian, and the nearest sample refined by Newton's
  method on the tangent's component of the offset.
- **Traffic control is read with the roads.** Each road's signals keep their
  place, whether they change, the direction of travel and lanes they hold for,
  and their country's catalog entry, such as Germany's 1000001 for a traffic
  light and 206 for a stop sign. Objects keep their outlines, and
  `compute_outline` places their corners: a corner in road coordinates on the
  road, and one in the object's own frame turned by its heading, pitch and
  roll from the road's axes at the object. Junctions keep their priorities
  between connecting roads and their controllers, and the network keeps the
  controllers that group signals; OpenDRIVE says which signals change
  together, not when.

`opendrive_reference_test` checks simon against libOpenDRIVE on test roads
with every geometry, elevation, superelevation and changing lanes, one of
them at map coordinates; a signalized four-way junction with a light and a
crosswalk on each approach; a T whose minor road gives way; a climbing,
leaning curve with crosswalks in road coordinates and in their own turned
frame; CARLA's Town01, 98 roads as RoadRunner writes them; and two of
esmini's roads with signs (see `application/automotive/reference/README.md`):

| Check | Agreement |
|---|---:|
| Positions on lines, arcs and spirals, through elevation and superelevation, on and off the surface | 1.0e-13 m |
| Lane borders | 8e-14 m |
| The lane at each lane's middle | 5,879 of 5,880 |
| The lane graph, through both junctions too | All 337 edges |
| Positions on parametric cubics, against an exact arc length | 1.2e-13 m |
| 20 signals: every field, every lane validity, and where each stands | Exact |
| The corners of 6 crosswalks' outlines | 1.6e-14 m |
| Junction priorities and controllers | All 6 |

libOpenDRIVE finds a parametric cubic's arc length through a table of chords
made to 1 cm, and is 5.4 mm from the exact arc length; simon is at rounding.
The one lane they disagree on is where a lane opens from no width: libOpenDRIVE
keys lanes by their outer border, so a lane of no width that shares its
neighbor's border can take a point in the neighbor's middle.

## Vehicles

`model/single_track` holds CommonRoad's single-track models, each axle's
wheels lumped into one on the center line. The kinematic model rolls without
slip: its state is the rear axle's position, the steering angle, the speed and
the heading, and it is driven by a steering rate and an acceleration, which
the vehicle's limits bound as CommonRoad bounds them: the steering's range and
rate, braking, the engine's power above a switching speed, and the speed's
range. Its state and rate plug into `Continuous`.

`single_track_test` checks it against CommonRoad's reference model, for its
three vehicles, a Ford Escort, a BMW 320i and a VW Vanagon:

| Check | Agreement |
|---|---:|
| Rates at 1,200 states and inputs, a quarter on a limit | 2.3e-16, relative |
| Paths over 20 s of steering and acceleration steps, by the same Runge-Kutta 4 at 0.05 s | 1.9e-13 m |
| The same paths at 0.05 s against 0.0005 s | 0.23 mm |

Three dynamic models follow CommonRoad's, for fidelity where the kinematic
model is not enough, each opt in:

- **The dynamic single-track model** (`model/single_track`) follows the
  center of gravity, which slips sideways and yaws under each axle's lateral
  force, linear in its slip angle, with the axle's load shifted by braking
  and speeding up.
- **The drift model** adds each axle's wheel spin, and its tires follow the
  Magic Formula in combined slip, so it brakes, spins its wheels and drifts.
- **The multibody model** (`model/multibody`), after the US Department of
  Transportation's vehicle dynamics, has a sprung body that yaws, rolls,
  pitches and heaves on its suspension over two unsprung axles, four wheels,
  and compliant pins at each axle's roll axis: 29 states.

`model/tire` holds the Magic Formula 5.2 in CommonRoad's subset: pure and
combined slip, every scaling factor 1, turn slip left out. `model/vehicle`
holds every parameter of CommonRoad's vehicles, and the limits on how they
are driven. At a crawl, where slips are undefined, each model drives as the
kinematic model about the center of gravity.

simon implements four corrections to CommonRoad, each from its source:

- **The tire's vertical shift is added after the sine**, as Pacejka has it.
  CommonRoad adds F_z p_vx1 inside the sine, as an angle, which brakes a tire
  rolling free with about 600 N under 8 kN.
- **The side force longitudinal slip induces turns with kappa**, the Magic
  Formula's slip. CommonRoad passes its own slip, -kappa, there, though it
  negates it for the pure longitudinal force.
- **At a crawl the slip angle changes at its derivative.** beta =
  atan(tan(delta) b / l), and CommonRoad's rate squares tan(delta) a second
  time.
- **The multibody model's crawling yaw rate changes with the slip angle**,
  where CommonRoad reads the roll angle.

simon also bounds two cases where CommonRoad's multibody model has no
answer: a tire off the ground pushes nothing, where CommonRoad's pulls with
a negative load, and a wheel's slip is divided by at least 0.1 m/s of ground
speed, as CommonRoad's drift model divides it, where the multibody model
divides by zero when reversing.

`vehicle_dynamics_test` checks all of it against CommonRoad, its models
patched with the four corrections (see
`application/automotive/reference/commonroad_dynamic.py`), for its three
vehicles:

| Check | Agreement |
|---|---:|
| Tire forces at 2,000 slips, slip angles, cambers and loads | 2.9e-15, relative |
| CommonRoad's own tire forces, uncorrected | 618 N longitudinal, 885 N lateral |
| Dynamic single-track rates at 900 states, from a crawl to 45 m/s and reversing | 6.4e-15, relative |
| Drift rates at 900 states, wheels slipping and locked | 3.3e-14, relative |
| Multibody rates at 592 states, the suspension displaced about rest | 1.9e-14, relative |
| Paths through a double lane change, braking and speeding up, 8 s, by the same Runge-Kutta 4 at 0.001 s | 2.9e-14 m |
| The same paths at 0.001 s against 0.0001 s: dynamic, drift, multibody | 4e-11 m, 4.2 µm, 0.52 mm |

## Maneuvers against Chrono

Project Chrono's Chrono::Vehicle models a vehicle as a full multibody system.
Its Sedan has a double wishbone front suspension, a multilink rear, a rack
and pinion, front-wheel drive through an engine map and gearbox, and Pac02
tires, Chrono's Magic Formula 5.2. simon drives its own models with the
Sedan's parameters through the handling maneuvers the standards define, and
measures both as the standards measure them
(`application/automotive/reference/chrono_reference.cpp` records Chrono).

**The tire.** `model/tire` holds the whole of the Magic Formula 5.2's steady
state, pure and combined slip and the aligning moment, and `format/tire_file`
reads it from TNO tire property files (.tir). A tire on the right is the
left one mirrored about its wheel plane, so that a pair's asymmetries cancel.
CommonRoad's subset stays a tire of its own, since its cornering stiffness
is linear in load and it mirrors through the camber's sign. Chrono's Pac02
departs from Pacejka in five places, and `tire_test` measures each:

| Check | Agreement |
|---|---:|
| Forces and aligning moment, without camber, where Chrono keeps the formula | 2.2e-5 of the load |
| Beyond Chrono's clamp of B x at pi/2 - 0.01, which flattens the curve past its peak | 7.7% of the load |
| At a camber of 0.03 rad, where Chrono's lateral friction rises by 1 + p_dy3 gamma^2 rather than falling by 1 - p_dy3 gamma^2 | 0.48% of the load |
| Combined by Chrono's default friction ellipsis rather than Pacejka's weighting | 60% of the load, at large slips both ways |

The 2.2e-5 is Chrono's 0.1 added to each B's denominator. Chrono also gives
the trail's equivalent slip angle in combined slip kappa's sign rather than
the slip angle's, divides its slips by the speed plus 0.1 m/s, and in a
vehicle holds camber at zero whatever the wheel's lean.

**The models.** Driving the Sedan showed what CommonRoad's models leave out
once the tire is a real one, and simon adds it, opt in through the tire:

- **Each axle's tires each take half its load.** The Magic Formula's
  cornering stiffness grows less than linearly with load, so one tire at an
  axle's whole load corners differently; CommonRoad's linear tire hides it.
- **The tires' aligning moments yaw the vehicle.** At the Sedan's
  pneumatic trail they are worth about 0.15 deg/g of understeer, the largest
  single term.
- **Each wheel can be steered on its own** in the multibody model, by toe,
  Ackermann and roll steer. CommonRoad's model steers both front wheels
  alike and the rear not at all.

The multibody model is in SAE's axes, x forward, y right and z down, as
CommonRoad writes it; its left wheels sit at -y.

**The Sedan's parameters.** The drift model reads the Sedan's mass, inertia
and axle positions from Chrono's assembly. The multibody model's suspension
comes from Chrono's own, measured as a kinematics and compliance rig would:
the Sedan at rest, braked, pressed down by 3 kN and rolled by 2 kN m, gives
each corner's rate, 16.3 kN/m front and 61 kN/m rear, and each axle's roll
stiffness; the ramp at 80 km/h gives each axle's lateral load transfer, and
so its roll center. The Sedan's tire is used with its camber terms zeroed,
as Chrono uses it. Each model is steered by Chrono's front wheels' mean
angle and held to Chrono's speed; the multibody model is also steered, in
one comparison, as Chrono's suspension steers each wheel.

`maneuver_test` checks them against Chrono, through steady-state circular
driving at 80 and 100 km/h with a slowly increasing steer (ISO 4138), a step
steer to 4 m/s^2 at 80 km/h (ISO 7401), and sines with dwell at 80 km/h at
2.5 and 5 times the steer for 0.3 g (FMVSS 126):

| Metric | Chrono | Multibody, each wheel steered | Multibody | Drift single-track |
|---|---:|---:|---:|---:|
| Understeer gradient at 80 km/h, deg/g | 0.159 | 0.151 | 0.131 | 0.132 |
| Understeer gradient at 100 km/h, deg/g | 0.186 | 0.142 | 0.130 | 0.125 |
| Step steer's steady yaw rate, rad/s | 0.1783 | 0.1781 | 0.1792 | 0.1803 |
| Step steer's yaw rate response time, s | 0.08 | 0.08 | 0.08 | 0.08 |
| Step steer's overshoot | 28.8% | 29.0% | 27.3% | 29.3% |
| Sine with dwell at 2.5 times: lateral displacement at 1.07 s, m | 2.36 | 2.18 | 2.15 | 2.12 |
| Sine with dwell at 5 times | Spins | Spins | Spins | Spins |

Both pass FMVSS 126's yaw rate ratios at 2.5 times, near zero 1 s after the
steer, and neither would pass at 5 times without stability control. Past
the limit they spin differently: Chrono's clamp holds its tires' force near
the peak where the formula's falls away, so simon's Sedan spins sooner. At
100 km/h Chrono understeers more than at 80 km/h and simon's models do
not; that difference, up to 0.04 deg/g, is not yet explained.

## Scenarios against esmini

`format/openscenario` reads ASAM OpenSCENARIO 1.x scenarios with pugixml:
parameters, substituted as $name and evaluated as ${...} expressions;
vehicles from catalogs; positions on lanes and roads, relative to entities
and in the world; speed, lane change, lane offset, teleport and parameter
actions; and conditions on time, speed, acceleration, headway, distance,
position, the end of the road, parameters and the storyboard's own states.
Anything else that changes what happens, a controller, a route or a
trajectory among them, is refused. A vehicle towing a
trailer is refused too, since esmini makes the trailer an entity of its own.

`scenario/storyboard` runs the storyboard: its elements' states and
transitions, events by priority and execution count, and triggers, each
condition on its edge and after its delay. `model/road_placement` places a
vehicle by road, lane, s and offset, either way along any lane, and moves it
along its path at its t, s changing by the distance over 1 - kappa t. In the
ECS, `RunStoryboard` evaluates the storyboard and gives each vehicle its
orders, `ControlSpeed` runs speed actions, `MoveOnRoad` runs lateral actions
and teleports or carries a vehicle along its lane, and `PlaceOnRoad` puts it
in the world (`application/automotive/scenario_simulation.hpp`).

Where the standard leaves the details open, simon follows esmini, the open
OpenSCENARIO player:

- **Each step evaluates every trigger on the world as the last step left
  it**, and an action started in a step also runs in it, in the storyboard's
  order. A lane change before a speed action in the storyboard moves at the
  speed before it.
- **A transition is stretched to the vehicle's limits.** A speed action
  whose shape would accelerate or decelerate past the vehicle's performance
  takes longer, its peak rate at the limit; a rate dimension is the
  transition's peak rate.
- **A lane change keeps the vehicle's path length,** its speed times the
  step, and takes its lateral motion from it, the heading turned along the
  path.
- **A storyboard element's transition counts at a condition's next
  evaluation,** in the same step if the condition comes later; an edge needs
  a value before it; a trigger that fires starts its conditions over.
- **A teleport or a parameter set ends after the step's triggers,** and a
  vehicle the storyboard teleports stays where it was put for the step.
- **A condition measures distance in the triggering entity's own
  coordinates by default:** along its heading, across it, or straight,
  signed by whether the other is ahead; or along or across its road. 1.0's
  alongRoute is longitudinal along the road. Between bounding boxes
  (freespace), simon measures only along the road and refuses the rest.

simon evaluates every condition of a group each step, where esmini stops at
the first false one but for those with delays, which keeps every edge's
history current.

`scenario_test` plays three of esmini's scenarios and checks every entity
at every step against esmini 3.8.2 at 0.05 s
(`application/automotive/reference/esmini_scenarios.py`):

| Scenario | Steps | Position | Heading | Speed |
|---|---:|---:|---:|---:|
| A cut-in on a straight road: a headway trigger, a sinusoidal lane change and braking | 322 | 6.8e-7 m | 4.9e-7 rad | 4.6e-14 m/s |
| A cut-in on a curved highway, at a speed relative to the ego's | 440 | 1.2 mm | 8.3e-7 rad | 8.3e-14 m/s |
| Lane changes across a curve, into the oncoming lane, over and over: reaching positions, the end of the road, teleports, two acts, repeated events | 3,669 | 7.0e-7 m | 1.3e-6 rad | 0.0033 m/s |

esmini logs to six decimals, which bounds the agreement on straight roads.
On e6mini's curves the 1.2 mm is the two road models' geometry. The lane
changes' speed differs in two steps, where esmini reports a vehicle
teleported away from the end of its road at a standstill for a step; its
next step and every position agree. Each storyboard stops on the same step
as esmini's.

## Metrics and batch runs

`model/collision` treats vehicles as boxes in the plane. Two boxes overlap
when no axis among their edges' normals separates their projections, the
separating axis theorem; boxes that touch overlap, as GEOS counts them.
Apart, their gap is the least distance from a corner of either to an edge
of the other. `collision_test` checks 2,000 pairs, from a car's size to a
truck's at any heading, a quarter of them a hair from touching, against
Shapely 2.1.2 on GEOS 3.13.1, which nuPlan uses: every overlap agrees, and
every gap to 1e-12 m.

`model/driving_metrics` measures a run as nuPlan's devkit (1.2.2) defines
it:

- **Comfort.** The accelerations along and across the heading, smoothed by
  a Savitzky-Golay filter over 8 samples; the jerks, their derivatives over
  15; the yaw rate and acceleration, the heading's first and second
  derivatives over 5; each rounded to 8 decimals, and each strictly within
  nuPlan's bounds at every sample. The filter is SciPy's `savgol_filter`,
  its edges and even windows included.
- **Time to collision.** The ego and the vehicles ahead of it, within 30
  degrees of its heading, carried at constant speed along their headings
  in steps of 0.1 s up to 3 s, until the ego's box overlaps one's. A run
  stays within bound while every sample's time exceeds 0.95 s.

`metrics_test` checks both on esmini's three scenarios against nuPlan's own
code (`application/automotive/reference/nuplan_metrics.py`): the comfort
signals to 5e-8, a unit of nuPlan's rounding carried through a derivative,
and every time to collision exactly. nuPlan's map-dependent parts are left
out: which collisions are at fault, the drivable area, driving direction,
and choosing tracks by lane, for which simon takes the vehicles ahead as
nuPlan's `is_agent_ahead` does.

`format/openscenario` reads deterministic parameter value distributions:
value sets, sets of values, and ranges, whose values are the lower limit
and each step on it, written to 15 significant digits so the steps'
rounding does not show. Stochastic distributions are refused.
`scenario/parameter_distribution` numbers the permutations as esmini does,
the last distribution varying fastest, and each permutation's values
replace the scenario file's declarations before it is read.
`application/automotive/scenario_batch` plays every permutation on a pool
of threads, each run in its own world, and measures its ego, the scenario's
first entity, sampled after each step.

`batch_test` runs esmini's parameter distribution over the curved
highway's cut-in, 12 permutations of two vehicle sets, two ego speeds and
three speed factors, against esmini 3.8.2 and against nuPlan's code on
esmini's runs:

| Check | Result |
|---|---|
| Each permutation's parameter values | The same as esmini's, in the same order |
| Every entity at every step, 5,392 steps | 1.1 mm, as the cut-in alone; speeds to esmini's six decimals |
| Each entity's box, from the vehicle its permutation chose | Exact |
| Time to collision at every sample | The same at all 5,392 |
| The gap between the ego's box and the other's at every sample | 1.1 mm |
| Each run's verdicts: comfort, time to collision within bound | The same |
| One thread or four | Identical measures |

In every permutation the ego, which has no controller, runs into the
vehicle that cuts in and brakes, in esmini as in simon. On one thread the
12 runs take 70 ms.

## Drivers

`model/traffic` holds the drivers' models: the Intelligent Driver Model for
following, in the form of Treiber and Kesting's *Traffic Flow Dynamics*, its
desired gap never less than the minimum gap, and MOBIL for changing lanes, by
its symmetric criterion with a bias toward the right lane. MOBIL weighs a
change by the accelerations it brings this driver and, by its politeness,
both followers, and refuses one that brakes the new follower harder than its
safe deceleration.

Its authors' own implementation, movsim's traffic-simulation.de, is a
variant: a linear free-road term above the desired speed, the gap held at
the minimum gap, braking capped at 18 m/s^2, and a MOBIL whose safe
deceleration changes with speed and which leaves out the old follower.
`traffic_test` checks simon against it where the two agree with the published
models, and checks a platoon against SUMO:

| Check | Agreement |
|---|---:|
| IDM accelerations at 1,000 gaps and speeds, for four drivers, against movsim | 1e-14 |
| MOBIL decisions at 1,000 sets of accelerations, to either side, against movsim | All 1,000 |
| Five IDM followers behind a leader that brakes and speeds up, 80 s, simon by Runge-Kutta 4 at 0.1 s against SUMO at 0.001 s | 1.07 cm |

SUMO's Euler update is first order: at its usual 0.1 s step it is 1.06 m from
its own converged platoon, so the 1.07 cm is its error at 0.001 s; simon at
0.1 s is within 0.1 mm of its own converged platoon.

## Traffic

`application/automotive` drives vehicles on a network read from OpenDRIVE.
A vehicle's state is in lane coordinates: its lane, the s of its front bumper,
its speed, and how many lanes it has entered. It follows its lane's middle,
which is the kinematic single-track model with its steering set by the lane's
curvature, so lane coordinates lose nothing, and its place in the world
follows from the road's geometry.

Components (`application/automotive/simulation_components.hpp`):

| Component | Holds |
|---|---|
| `VehiclePose` | The world's spatial component: the front bumper's position and the heading |
| `LaneState` | The lane, s, speed and lanes entered |
| `Driver` | The IDM and MOBIL drivers, the vehicle's length, and the seed that picks its way at forks |
| `DriveCommand` | The step's acceleration, and a lane to change to |

Schedule (`application/automotive/simulation_systems.hpp`):

| System | Does |
|---|---|
| `Decide` | Every step, finds each driver's leader and accelerates by IDM; once a second, weighs the lanes beside it by MOBIL |
| `Drive` | Every step, changes lane if decided and moves along the lane, entering the next lane on the vehicle's way past its end |
| `FollowLane` | Every step, puts each vehicle in the world at its lane's middle |

- **A driver reads its neighbors from an index.** `Decide` writes only the
  command, so it may read every vehicle's lane state; its `prepare` sorts
  them by lane and distance along. A vehicle's leader and follower in its own
  lane sit beside it in the index, found through a table from entity to
  place; in other lanes they are binary searches. A leader is the next vehicle in the lane, else the first in the
  lanes the vehicle's way leads into, within 250 m; a dead end is a leader
  standing still.
- **A vehicle's way is fixed by its seed.** At a fork the lane is picked by
  SplitMix64 of the driver's seed and the number of lanes it has entered, so
  the same lane is picked each time it is looked ahead to and taken.
- **The step holds the acceleration** and moves exactly under it, never
  backward: a vehicle that would stop within the step stops where it would.
- **Merges at junctions are not resolved.** Vehicles on different incoming
  lanes see each other only once in the same lane, and nothing gives way.
  Signals, priorities and pedestrians are later work.

`automotive_test` drives a ring of two roads, two lanes each way, and CARLA's
Town01. On the ring, 40 vehicles circulate for 300 s at 18.4 m/s, never
closer than 1.09 m, and change lanes 129 times, more of them ending in the
right lane than the left; the same seed repeats exactly. In Town01, 60
vehicles enter 1,573 lanes through its junctions in 120 s, at 10.2 of their
11 m/s.

## Tactical layer

Planned, step by step (see the [Roadmap](#roadmap)). IDM decides how hard to
accelerate behind one leader; the tactical layer decides where a vehicle
must stop, and pedestrians decide when to cross.

**Everything a vehicle stops for is a stopped leader.** The tactical layer
never sets an acceleration. It finds the first point ahead where the vehicle
must stop and hands IDM a standing leader of no length there; IDM's leader is
the nearer of that and the real one. Treiber and Kesting model a red light
this way (*Traffic Flow Dynamics*), and SUMO stops its vehicles at junctions
the same way.

The points are **conflict zones**, fixed data built from the network when it
is read and sorted by s along each lane:

| Zone | Where | Blocks when |
|---|---|---|
| Stop line | Where a signal's validity starts on a lane | Its group shows red, or yellow and the vehicle can still stop comfortably |
| Conflict area | Where a junction's connecting lane crosses or merges with a foe | A foe with priority would arrive within the driver's critical gap |
| Crosswalk | Where a crosswalk's outline crosses the lane | A pedestrian is on it or has committed to it |

- **Zones are model data, not entities.** They never move and are many, so
  they live in a table keyed by lane, as the lane graph does. What changes,
  signal states and who is on a crosswalk, is in entities.
- **A vehicle commits** once it can no longer stop at a zone with its
  comfortable deceleration, and then ignores it, so it neither dithers in the
  dilemma zone nor brakes inside the junction. At yellow it stops if stopping
  needs less than its comfortable deceleration, as Treiber and Kesting's
  drivers do.
- **Yielding is gap acceptance.** A driver on a lane that yields asks when the
  first vehicle with priority reaches each conflict area ahead, and the area
  blocks while that is sooner than its critical gap t_c; a driver following
  another through the same gap needs only the follow-up time t_f, as the
  Highway Capacity Manual's two-way stop control defines them. Each driver
  draws its own. Against random priority traffic the minor stream's capacity
  is then Siegloch's, c = (3600 / t_f) exp(-q_p (t_c - t_f / 2) / 3600).
- **Deadlocks resolve by arrival.** At an all-way stop the first to arrive
  goes first, ties going to the lower Name, so runs repeat.
- **`Decide` stays one system.** Its `prepare` adds signal states, crosswalk
  occupancy and each conflict area's priority arrivals to its index; each
  vehicle then searches its lane's zones once and writes only its own
  command and `Tactical` state.

**Traffic lights** are `SignalController` entities, one per OpenDRIVE
`<controller>`, holding a fixed-time `SignalProgram` and its `SignalState`,
advanced by `RunSignals`. OpenDRIVE says which signals a controller groups
but not their timing, which comes from OpenSCENARIO's
`TrafficSignalController`, a scenario parameter, or a default plan. Actuated
signals are a later opt-in.

**Pedestrians** walk a graph of sidewalk lanes, crossings from crosswalk
outlines, and links where sidewalks meet at junctions, in one dimension as
vehicles drive lanes, so they cost what traffic does; a social force model
(Helbing and Molnar, 1995) is a later opt-in for crowds.

| Layer | Decision | Model |
|---|---|---|
| Strategic | Where to go | A destination and the shortest path on the walking graph, from the seed |
| Tactical | Whether to cross now | At a signal, walk or don't walk, with each person's compliance; elsewhere, gap acceptance against the next vehicle, the critical gap the crossing's length over the walking speed plus a start-up time, as in the Highway Capacity Manual |
| Operational | How to move | Each person's desired speed, slowing behind the person ahead on the same edge |

A pedestrian who commits to a crossing occupies it in the next index, and
vehicles see a stopped leader there. Whether vehicles yield at an
unsignalized crosswalk, and which side traffic drives on, are parameters of
the network, by default right-hand traffic that yields. Vehicles and
pedestrians share one spatial component, `RoadPose`.

## Viewer

`bazel run -c opt //application/automotive:viewer -- <file>` watches traffic
on an OpenDRIVE network, or plays an OpenSCENARIO scenario, under
`RealTimeDriver`; the file's extension decides which. The map draws each
lane by its OpenDRIVE type, the center line where lanes run either way, and
every vehicle as its box; a box smaller than a few pixels becomes a dot, and
the lanes are sampled no closer than two pixels apart, so a network as large
as the scale benchmark's 100 rings draws whole. Traffic is colored by speed.
In a scenario the ego is blue, the others orange, and a vehicle touching the
ego red, and the map starts 200 m wide about the ego. A click on a vehicle
follows it, the map keeping it in the middle; the panel reads out its lane,
s, speed and acceleration in traffic, and in a scenario every vehicle's
speed and the ego's gap and time to collision, which charts under the map
trace as nuPlan measures them (`measure_sample`, shared with the batch
runs).

`road_drawing` samples each lane section's lane borders along the road, at
most a spacing apart on the reference line, without the UI. `road_drawing_test`
checks the ring's driving lanes against the exact area of the polygon their
chords inscribe, to 1e-12, that every lane of Town01 is drawn, and that the
ring's halves meet. `viewer_traffic_test` and `viewer_scenario_test` run the
viewer for 60 frames on Town01's traffic and on the curved highway's cut-in.

## Scale

`automotive_benchmark` drives 1,000, 10,000 and 100,000 vehicles on 100
rings of 1 km radius, three lanes each way, 3,770 km of lanes in all
(`roads/rings.xodr`). Drivers want 30 m/s, spread uniformly by 10%, and
start at 25 m/s; steps are 0.1 s. After 60 s of settling it times 300 steps,
each system on its own, on one thread. `reference/sumo_benchmark.py` drives
the same network, converted by netconvert, in SUMO 1.27.1: the same IDM
drivers, SUMO's LC2013 for changing lanes, one thread, no output, no
teleporting and no TraCI, timed by SUMO's own duration over the same 300
steps. Both run on an AMD Ryzen 9 5900XT.

| Vehicles | simon, ns per vehicle-step | SUMO, ns per vehicle-step | SUMO / simon | Mean speed at the end, simon / SUMO |
|---:|---:|---:|---:|---:|
| 1,000 | 174 | 1,433 | 8.2 | 29.9 / 29.8 m/s |
| 10,000 | 234 | 7,140 | 31 | 29.3 / 29.5 m/s |
| 100,000 | 248 | 6,510 | 26 | 20.4 / 22.2 m/s |

At 100,000 vehicles simon steps in 25 ms, four times faster than real time
at 0.1 s steps; SUMO takes 651 ms, 6.5 times slower than real time. simon's
step splits into `Decide` 57%, `FollowLane` 39% and `Drive` 3%. `Decide`'s
cost is mostly its index, sorted each step, and the IDM's arithmetic;
`FollowLane`'s is the road geometry, each vehicle's place found from its
lane's middle on an arc.

The two drive differently in detail, so the comparison is of cost at the
same load. SUMO's lane changing is LC2013, simon's
MOBIL, and SUMO updates by Euler where simon holds the acceleration over the
step. At 100,000 vehicles, 26.5 vehicles a lane-kilometer, traffic is near
the IDM's capacity at 30 m/s, and both slow down.

Waymax is not measured. Its scenarios come from the Waymo Open Motion
Dataset, whose license allows only non-commercial use, and its throughput
depends on the accelerator it runs on, so no figure for it is quoted here.

GPUDrive (aa48a43) is not measured, for the same two reasons. It drives the
same dataset, at most 64 agents a world, on the Madrona engine (8645708;
Shacklett et al., "An Extensible, Data-Oriented Architecture for
High-Performance, Many-World Simulation", SIGGRAPH 2023), and its authors
give 1 million frames a second on a GPU. Madrona is an ECS in C++ that steps
thousands of small worlds at once. On a GPU each archetype is one table for
every world, each row carrying its world's ID, so one pass of a system
covers all worlds; sorting by world and compacting deleted rows are steps
of its task graph. On a CPU each world runs its own task graph as one job on
a pool of threads, as `scenario_batch` plays each permutation in its own
world. Given a maximum per world, Madrona's CPU tables hold every world in
one allocation, each world's rows in a block of their own, so a world
resets by zeroing its count. simon's segments could hold one replica each
in the same way, which would give the roadmap's world snapshots and
replicas of a scenario a place in the store.

## Roadmap

**Automotive (done).** Closed-loop driving at the level of objects, each
claim checked against an open reference:

1. Roads, read from OpenDRIVE and checked against libOpenDRIVE.
2. The lane graph, against libOpenDRIVE's routing graph; the kinematic
   single-track model, against CommonRoad's; IDM and MOBIL, against movsim
   and SUMO; traffic on lanes in the ECS.
3. A scale benchmark against SUMO, 26 times SUMO's speed at 100,000
   vehicles.
4. The dynamic and drift single-track models, the Magic Formula tire and the
   multibody model, against CommonRoad's, with four corrections.
5. The whole Magic Formula 5.2 and tire property files, against Chrono's
   Pac02; the Sedan through the ISO handling maneuvers and FMVSS 126 against
   Chrono::Vehicle, with the tires' aligning moments, each wheel steered, and
   each axle's tires at half its load.
6. OpenSCENARIO's storyboard, actions and conditions in the ECS, against
   esmini step by step.
7. nuPlan's comfort and time to collision, on boxes checked against GEOS;
   parameter distributions played in batches on threads, against esmini and
   nuPlan.
8. A viewer for traffic and for scenarios.

**Tactical layer (in progress).** Where vehicles must stop, and pedestrians
who decide when to cross (see [Tactical layer](#tactical-layer)):

1. Done: signals, controllers, junction priorities and crosswalks, read
   from OpenDRIVE and checked against libOpenDRIVE.
2. Conflict zones, stopped leaders, commitment and fixed-time signals,
   against movsim, Webster's uniform delay and SUMO.
3. Right of way at junctions: priority, give-way and stop, gap acceptance and
   deadlock, against Siegloch's capacity and SUMO.
4. The walking graph, and pedestrians walking routes on it.
5. Crossing decisions and vehicles yielding, against the Highway Capacity
   Manual's pedestrian delay and SUMO.
6. OpenSCENARIO's signal controllers and pedestrians, against esmini.
7. The scale benchmark with signals and pedestrians, against SUMO, and the
   viewer.
