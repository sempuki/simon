# aeronautic design

Aircraft fly closed routes of waypoints under an autopilot, spread over an
area that grows with their number so traffic density stays the same. It is
the application for flight control algorithms of the class JSBSim runs, and
it shows fidelity as an opt-in: most aircraft fly a single-pass model, and
those whose archetype opts in are integrated with Runge-Kutta 4.
`bazel run //application/aeronautic -- <aircraft> <precise> <seed>` flies one
scenario headless and prints how many waypoints were reached.

The code is in three layers:

| Layer | Holds |
|---|---|
| `framework/continuous.hpp` | `Continuous`, generic over any `ContinuousState` |
| `model/` | `flight_path.hpp` (the point-mass state, its rate equations, a single-pass integrator and autopilot laws), `atmosphere.hpp`, `control.hpp` (lags, rate limits, PI control, tables) |
| `application/aeronautic/` | The aircraft components and archetypes, the systems, the scenario and the simulation |

The model is point-mass flight path: an `AirState` of position, speed,
flight-path angle and heading, flown by commanding load factor, bank and
throttle. Lift is a load factor times weight, drag comes from a drag polar,
and thrust falls with the standard atmosphere's density. The atmosphere's
layers are in geopotential altitude, as the 1976 standard and JSBSim have
them, so its density is within 10^-5 of JSBSim's up to 20 km. `AirState` is the
world's spatial component, so no copy of the position is kept anywhere else.

Components (`application/aeronautic/simulation_components.hpp`):

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

Schedule (`application/aeronautic/simulation_systems.hpp`):

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
  excludes the rate, so the runner skips the precise segment whole. Naming
  the rate as an optional sibling and returning when it is present would cost
  a call per precise aircraft under Clang, which does not inline the call
  operator: 0.23 ms per step at 100,000, against 0.06 ms with GCC. Excluding
  costs nothing under both.
- **Guidance and control run at their own rates, gated once per system** in
  `prepare`, so the steps in between skip their loops. Per-entity staggered
  gates would spread the work, at the cost of a gate in each component and a
  check per entity per step.
- **Speed comes first.** The steepest climb the autopilot commands shrinks as
  the aircraft falls below its target speed, and is zero 20 m/s below it, so
  a long climb at altitude never trades away more speed than that.
- **No ground and no stall.** Routes stay between 3 and 9 km. Wind is opt
  in (see [Wind and turbulence](#wind-and-turbulence)).

The test checks the autopilot (holding altitude, speed and heading, and
turning the short way to a waypoint behind), that a closed route is flown,
that a single-pass and a Runge-Kutta aircraft flying the same route end within
100 m of each other after five minutes, and that a scenario repeats exactly
from its seed. `accuracy_test` compares the model with JSBSim (see
[Accuracy against JSBSim](#accuracy-against-jsbsim)).

`bazel run -c opt //application/aeronautic:aeronautic_benchmark` flies 500 steps of
20 ms at each population, once with every aircraft on the single-pass model
and once with every aircraft on Runge-Kutta 4. GCC, ms per step:

| Aircraft | Single pass | ns per entity-step | Runge-Kutta 4 | ns per entity-step |
|---:|---:|---:|---:|---:|
| 1,000 | 0.06 | 59 | 0.17 | 170 |
| 10,000 | 0.63 | 63 | 1.89 | 189 |
| 100,000 | 6.27 | 63 | 19.2 | 192 |
| 100,000, 4 contending threads | 7.25 | 73 | 32.1 | 321 |

- **The single-pass model is bound by computation.** Cost per aircraft is
  nearly flat from 1,000 to 100,000, and four contending threads slow it only
  1.2 times. `Fly` is three quarters of the step, and three `sincos` calls are
  most of `Fly`.
- **Runge-Kutta 4 costs 3.1 times as much,** and is bound by memory under
  contention (1.7 times slower), because its copies of the start state and
  four stages' rates stream through memory every step.
- **The single pass takes 6.27 ms and Runge-Kutta 4 19.2 ms.** The rates
  take each sine and cosine once, and `fly` gets the new velocity from the
  rate's derivative, not more trigonometry, and its direction's cosine by a
  square root rather than `hypot`. `wrap` calls `std::remainder` only when a heading
  leaves [-π, π]. `StandardAirTable`, the atmosphere tabulated every 100 m of
  geopotential altitude (within a few parts in 10^5), stands in for the
  power and exponential of `standard_air`, which would cost 6% more in `Fly`
  and 20% more in Runge-Kutta 4, which evaluates the air four times.
- **Clang is about 13% slower** (8.3 and 22.7 ms at 100,000).

## Accuracy against JSBSim

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

The airframe is the 737's mass and wing area, with a drag polar
CD = CD0 + K1 CL + K CL^2 fitted to 100 JSBSim trims over the scenario's
envelope (3 to 9 km, 160 to 240 m/s, level and in turns up to 45° of bank).
CD0 is 0.0156, K1 is 0.0302 and K is 0.0519, and the worst trim is 3.4% off.
The largest drift over the flight, which covers 130 km:

| Step | Model | Position | Altitude | Speed | Heading |
|---:|---|---:|---:|---:|---:|
| 20 ms | Single pass | 472 m | 53 m | 2.0 m/s | 0.7° |
| 20 ms | Runge-Kutta 4 | 476 m | 52 m | 2.0 m/s | 0.7° |
| 200 ms | Single pass | 434 m | 65 m | 1.8 m/s | 0.5° |
| 200 ms | Runge-Kutta 4 | 470 m | 52 m | 2.0 m/s | 0.7° |
| 1 s | Single pass | 436 m | 100 m | 2.5 m/s | 0.9° |
| 1 s | Runge-Kutta 4 | 475 m | 55 m | 2.2 m/s | 0.9° |

- **The drag polar is most of the drift.** Its speed error changes the turn
  rate, which bends the path away. A Python copy of the replay that takes
  JSBSim's own drag drifts 55 m, so the equations and integrators are not
  the limit.
- **The polar has a linear term.** The 737's least drag falls at a lift
  other than zero, as a cambered wing's does, so CD0 + K CL^2 fits the trims
  only to 9.7% and drifts 1.8 km. K1 costs one multiply-add a step, and
  aircraft without it set it to zero.
- **The polar leaves out pitch rate.** A turn holds elevator against its
  pitch rate, and a drag term in pitch rate brings the speed error down to
  1.2 m/s, but the path drifts further, 640 m, and every aircraft would pay
  for it.
- **Runge-Kutta 4 buys little at the steps simon runs.** At 20 ms the two
  models differ by 4 m against a drift of 470 m. At 1 s Runge-Kutta 4 holds
  altitude to 55 m against 100 m.

The test holds the drift at 20 ms and 1 s with about 25% headroom. To
regenerate the reference, install JSBSim's Python package and NumPy, and run
`python application/aeronautic/reference/jsbsim_737.py`. It prints the fitted
airframe, which the test keeps as constants.

## Rigid aircraft

The rigid aircraft archetype is the highest fidelity level: six degrees of
freedom, flown by control surfaces, engines and fuel, from an aircraft
described as data. It is opt in. Point-mass aircraft in the same world cost
what they cost alone (see [Mixed fidelity](#mixed-fidelity)), because each
rigid system is driven by a component only rigid aircraft have, and `Fly`
excludes rigid bodies.

An aircraft comes from JSBSim. `tools/jsbsim/convert.py` turns a JSBSim
aircraft's XML into simon's aircraft format, in SI units, and
`format/aircraft_file` reads it back, refusing bad files with the line at
fault. A converted aircraft holds its metrics, mass balance, fuel tanks,
turbines, flight control system and aerodynamics. The 737 is
`3rd_party/jsbsim/737.aircraft`, and the F-16 is `f16.aircraft` beside it (see [The F-16](#the-f-16)).

| Layer | Holds |
|---|---|
| `model/aerodynamics` | The coefficient build-up: terms of a constant, inputs and tables of inputs, summed by axis, and turned into body loads about the center of mass. An input is a state variable or a flight control signal, such as a surface's deflection, so an aircraft's surfaces need no code |
| `model/flight_control` | Flight control blocks over named signals: summers, gains, scheduled gains, surface scales, kinematic actuators, switches, PIDs and functions |
| `model/turbine` | JSBSim's turbine: spools, thrust from idle to military and through reheat to maximum, fuel flow |
| `model/earth` | The WGS84 ellipsoid, J2 gravitation, the Earth's rotation, geodetic conversion |
| `model/rigid_body` | Stevens and Lewis's equations of motion in an inertial frame, as a `ContinuousState` |
| `model/frames` | The flat or round Earth, a body's place on it, and its motion relative to the air and the wind |
| `model/mass_balance` | Fuel tanks, and the mass, center of mass and inertia they give with the empty aircraft |
| `model/propulsion` | The engines' air, their spools and thrust at the throttles, and the fuel they burn |
| `model/sensing` | The air data, attitude, motion and pilot's accelerations the flight controls read |
| `model/rigid_aircraft` | The aerodynamics' inputs and the body's rate, and a header that includes the four above |
| `application/aeronautic` | The archetype and its systems |

Each step a rigid aircraft runs `RunFlightControls`, `RunEngines`, then
`Rigid` (Runge-Kutta 4 over `RigidAircraftRates`), then `BurnFuel` and
`FollowRigidBody`, which keeps its `AirState` on its body for the rest of
the world. `RunFlightControls` runs every step, or at a fixed period if it is
given one, as a digital flight control computer runs at its own rate. Each
engine's throttle is a flight control signal, `throttle_<n>`, set from its
command before the blocks run, as JSBSim sets it, so a block may change it. The Earth is flat unless the systems are built with a round
one; round, the inertial frame is ECI and the world's local frame is the
plane tangent to the ellipsoid at an origin.

Each layer is checked against JSBSim's 737 (see
`application/aeronautic/reference/README.md`):

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

`bazel run -c opt //application/aeronautic:rigid_benchmark` flies rigid 737s
at 8 ms steps, each on its own. GCC, per aircraft per step:

| Aircraft | Flat Earth | Round Earth |
|---:|---:|---:|
| 100 | 1.22 µs | 2.03 µs |
| 1,000 | 1.16 µs | 2.02 µs |
| 10,000 | 1.19 µs | 2.06 µs |

- **The cost is flat with population,** so one thread flies about 6,900
  rigid 737s in real time over a flat Earth, and 3,900 round one. `Rigid`
  is two thirds to seven tenths of it: four stages, each building up the
  aerodynamics.
- **A stage finds the body's motion once.** It turns the attitude into a
  matrix and finds the air velocity and rate from it, takes the wind angles'
  sines and cosines as ratios of the air velocity, and finds the rate of
  angle of attack from the force per unit mass and a remainder the stage
  fixes. Whether an aircraft's forces read that rate is found when it is
  read.
- **JSBSim takes 9.2 µs a frame** for one 737, timed through its Python
  module from its own trim, with one instance per aircraft. Its frame does
  work simon's does not, such as ground reactions and its property tree, so
  the comparison is rough; simon evaluates the aircraft four times a step to
  JSBSim's once and is still 4.6 times faster round the Earth, and 7.9
  times over a flat one.
- **The round Earth costs 75% more.** Each stage finds the body's place on
  it once, its altitude, local frame and the Earth's turn, in `Earth::place`.
  `wgs84::locate` gives the local frame's sines and cosines straight from
  Heikkinen's closed form, with no angles to take them of.
- **The mass balance sums six terms.** `BurnFuel` recomputes it every step
  as the fuel burns, summing the inertia tensor's six distinct terms rather
  than a matrix per mass: 50 ns a step for the 737.

Building it turned up JSBSim behaviors that a comparison has to allow for,
all noted where they matter: a frame starts by moving the state on, so
everything read after a frame belongs together; its mass balance runs
before its engines burn; its `inertia/ixy` and `iyz` properties are the
tensor's elements negated but `ixz` is not; and its kinematic actuators keep
the frame time the model loaded with.

## The F-16

JSBSim's F-16 shows that the converter and the models were not fitted to
the 737. The same code converts it, and the same tests check it. It brings
what the 737 lacks:

- **Fly-by-wire flight controls.** They close loops on roll rate, on pitch
  rate and load factor, and on yaw rate and lateral acceleration, through
  three PIDs, eleven switches and a function. `model/flight_control` gains those three
  kinds of block.
- **Reheat.** A throttle past 1 lights it, and the flight controls double
  the pilot's throttle, so half throttle is military power.
- **A pilot.** The pilot is a point mass in the mass balance, and the flight
  controls feel the pilot's accelerations at the eye point.

The aerodynamics read flight control signals by name, so the F-16's
surfaces (combined aileron, leading-edge flaps, flaperons, speedbrake) are
data and need no code. The converter maps JSBSim's built-in surfaces:
`-rad` and `-deg` to the deflection in radians, `-norm` to
`<surface>_norm`, and `mag-` to `|surface|`, the magnitude. A block works in
the aircraft's own units, and where a signal it reads or writes is in other
units, such as knots or degrees, the converter gives it a scale, and turns
the numbers a switch compares into SI. Ground contacts, the hook, pushback
and the canopy are left out: weight on wheels is always 0.

The flight controls read the state they need, which `sense_flight_state`
finds: air data, calibrated airspeed by JSBSim's pitot formulas, ground
speed, body velocity, attitude, and the pilot's accelerations. An aircraft
finds only what its flight controls read, so the 737 pays nothing for the
F-16's. The pilot's accelerations come from the step before:
`RigidAircraftRates` keeps what each body feels at every stage, and the last
stage's is left after `Rigid`. A step's own would need the surfaces the
flight controls are about to set. JSBSim's flight controls read the frame
before's air data and accelerations two frames old.

Two of JSBSim's behaviors are part of the model. A kinematic block with an
output starts each frame from the output's value, and the F-16's yaw PID
writes the rudder's position just before its actuator moves it. JSBSim's
trim runs the PIDs before it has an airspeed, so they integrate, and their
integral cannot be read; the references zero it after the trim, as simon
starts. The F-16's PID triggers
hold their integrals in flight, so in flight its PIDs are proportional and
derivative.

Each layer agrees with JSBSim to rounding: the aerodynamics to 9e-15 at 300
states up to Mach 1.36, every one of the 60 flight control blocks to 2e-16
on every frame of three flights, the engine to 1e-13 through reheat and out,
and the mass balance to JSBSim's rounded slug.

The F-16's flight controls run at a fixed rate. Its control laws
differentiate the pilot's commands and then clip them, so run at every frame
they change with the frame and do not converge. A digital flight control
computer runs at its own rate whatever the dynamics do: the F-16's runs
every 8 ms in simon (`RunFlightControls` given a period) and in the
reference, which flies JSBSim at 0.125 ms with each channel run every 64th
frame. The throttle's channel runs every frame, because JSBSim sets the
throttle to its command each frame before the channels run; the command
changes only on the computer's frames, so the result is the same. Every
flight starts from JSBSim's trim at 8 ms, because JSBSim's trim depends on
its frame: at 0.125 ms its pitch trim differs by 6e-5.

The largest distance from JSBSim at 0.125 ms over 30 s:

| Case | simon at 0.5 ms | simon at 8 ms | JSBSim at 8 ms |
|---|---:|---:|---:|
| Trim hold | 3.7 mm | 3.7 mm | 0.05 mm |
| Pitch doublet | 2.1 mm | 2.1 mm | 1.5 cm |
| Roll doublet | 5.6 cm | 5.6 cm | 3.53 m |
| Rudder doublet | 3.7 mm | 3.7 mm | 0.8 mm |
| Throttle step into reheat | 1.1 cm | 14.7 cm | 63.2 cm |

- **simon has converged at 8 ms** in every case but the step into reheat,
  where the engine's thrust, held for a step, is the error.
- **The physics agrees to millimeters,** as the 737's does, and to 6 cm after
  the roll doublet, where JSBSim has itself converged only to about 5 cm.
- **At 8 ms simon is closer than JSBSim** after the roll doublet, by 63
  times, and the throttle step, by 4 times. In the hold and the rudder
  doublet JSBSim at 8 ms is closer, both being under simon's floor of 4 mm.

A rigid 737 costs 1.16 µs per aircraft-step flat and 2.02 µs round at 1,000;
the F-16's sensing costs it nothing.

## Trim

`model/trim` finds the attitude, controls and throttle at which a rigid
aircraft flies a steady, straight path. Six unknowns balance six
accelerations, paired as JSBSim's full trim pairs them: angle of attack the
acceleration along body z, throttle along body x, pitch trim the pitch, bank
the acceleration along body y, aileron the roll and rudder the yaw. There is
no sideslip. Newton's method solves them together, with a numerical Jacobian
and a line search, to residual accelerations of 1e-13. A trim that needs a
control past its limit fails as saturated.

Each evaluation settles the flight controls with the airframe.
`settle_flight_controls` runs every block as it stands when its inputs hold
still, kinematic blocks at their inputs and PIDs seeing no rate. The F-16's
flight controls feel the accelerations their own surfaces make, so the blocks
and the airframe are settled in turn until they agree.

Round the Earth a trim is level over the Earth: the body turns as the local
north-east-down frame turns under it as it moves (`Earth::level_rate`), so
its altitude and speed hold. Straight in space, as JSBSim's trim is, the path
climbs as the Earth curves away, 2.8 m in 6 km. A `FlightCondition` asks for
either.

Trimmed straight in space at the check cases' condition, simon's controls
agree with JSBSim's to a few parts in 10^5, the tolerance JSBSim's trim stops
at (`trim_test`). Flown level for 30 s at 8 ms, the largest change in
altitude and airspeed (`check_case_test`):

| Trim | 737 | F-16 |
|---|---:|---:|
| JSBSim's | 4.0 m, 0.19 m/s | 2.1 m, 0.081 m/s |
| simon's | 2.0 m, 0.10 m/s | 16 cm, 0.013 m/s |
| simon's, mass held | 3.7 mm, 0.2 mm/s | 2.4 mm, 0.1 mm/s |

Burning fuel lightens the aircraft, which climbs and speeds up; with the mass
held, the trim alone is measured.

## Mixed fidelity

Every level flies in one world. A scenario's `rigid` and `fighters` counts
make some of its aircraft rigid 737s and rigid F-16s, and they fly the same
kind of routes as the rest: `FollowRoute` sets their autopilot's targets as it
sets every aircraft's. `bazel run //application/aeronautic -- <aircraft>
<precise> <rigid> <fighters> <seed>` flies one, over a flat Earth so that
every level shares the world's frame.

Rigid aircraft start from simon's trim in cruise (`trim_in_cruise`; see
[Trim](#trim)), one for each type, since over a flat Earth a trim holds
wherever an aircraft is and whichever way it heads. They fly their
surfaces with `FlySurfaces`, an autopilot kept small on purpose. It takes the
point-mass autopilot's laws for the bank a heading needs, the flight-path
angle an altitude needs and putting speed first, and flies them with three
lines: aileron from the bank error with roll damping, elevator from the
flight-path angle error with pitch damping, the pull a turn needs and a
bounded integral, and throttle from the speed error about the trim. Each
aircraft carries its gains in its `SurfaceAutopilot`. A 737 banks up to 45°,
so at 200 m/s it turns on about 4 km, near the 3 km at which `FollowRoute`
captures a waypoint. An F-16 flies the same gains through its fly-by-wire,
whose stick commands rates and load, and banks up to 60°, turning on about
2 km. Its flight controls run every 20 ms step.

At the world's 20 ms step, rigid aircraft stay within 15 cm of the converged
JSBSim reference after the 30 s check cases, and within 9 cm after the rudder
doublet, against JSBSim's 41.5 cm at 8 ms. Over 10 minutes of routes they
keep between 3 and 8.3 km and between 197 and 236 m/s, and reach about 4.4
waypoints each to the point-mass aircraft's 6.5: a 737 turns wider than the
point-mass jet. F-16s keep between 4.0 and 7.7 km and between 200 and 238
m/s, and reach about 6 waypoints each.

`aeronautic_benchmark` adds a mixed population, 1% on Runge-Kutta 4 and 0.1%
rigid. At 100,000 aircraft, GCC, 20 ms steps:

| Aircraft | Systems | ms per step | Share |
|---|---|---:|---:|
| 98,900 single pass | `FollowRoute`, `FlyAutopilot`, `Actuate`, `Fly` | 6.20 | 95.3% |
| 1,000 Runge-Kutta 4 | `Continuous(AirState)` | 0.18 | 2.7% |
| 100 rigid 737s | `FlySurfaces` to `FollowRigidBody` | 0.13 | 2.0% |
| All | | 6.50 | |

One thread runs it 3.1 times faster than real time. Each level costs what
its own aircraft cost, and nothing more: the single-pass aircraft run as
fast as they do alone (6.27 ms at 100,000), because each level's systems are
driven by components only its aircraft have.

`bazel run -c opt //application/aeronautic:viewer` watches a mixed world under
`RealTimeDriver`, by default 2,000 aircraft with 20 on Runge-Kutta 4, four
rigid 737s and four rigid F-16s, at ten times real time. An ImPlot map draws
each level its own way, rigid aircraft with trails, and the route of the
rigid aircraft the panel follows. The panel shows that aircraft's air data,
attitude, load factor, engine and surfaces, and strip charts of its altitude
and airspeed sit under the map. The aeronautic viewer's panel also sets the
scenario's wind and turbulence.

The viewers share `application/viewing.hpp`: the window and its frame loop,
which take `--scale`, and `--frames` and `--screenshot` for running a viewer
with nobody watching; a `Session` that runs a scenario under `RealTimeDriver`
and finishes it when replaced; the side panel with its Restart, Quit, pause
and speed controls; and scatter plots of markers. A viewer draws only what is
its own. Each viewer's `viewer_test` runs it for 60 frames under SDL's dummy video
driver, so a viewer that no longer builds, starts or draws fails the tests.

## Wind and turbulence

Wind is opt in, like every level. A scenario's `wind` is a
`model::WindField`: a steady wind, the same everywhere, and turbulence of a
severity. By default the air is still, no aircraft has a `Wind`, and every
level costs what it would without wind: 1.16 µs per rigid aircraft-step at
1,000 over a flat Earth, and 6.50 ms per step for the mixed 100,000.

In moving air each aircraft has a `Wind`: the air's velocity relative to the
Earth, in the local north-east-down frame, and the rotation turbulence gives
it, held over a step. `MoveAir` sets it each step. A point-mass aircraft's
`AirState` is its motion through the air, so `DriftWithWind` adds the wind to
its position after `Fly` and `Precise`. Rigid aircraft in wind are an
archetype of their own, `RigidAircraftInWind`, which requires a `Wind`. The
rigid systems read it as a sibling that the still archetype lacks, so a rigid
aircraft in still air never looks for one. The rate, the flight controls' air
data and the engines' air are compiled apart for still air for the same
reason. A rigid aircraft in wind that also has `Gusts` flies through
turbulence. It starts trimmed for still air and moving with the air, which
over a flat Earth is the same trim.

A rigid aircraft's air velocity is its velocity less the air's,
R^T (v - W x r - u) with u the wind in the inertial frame, and its rate
relative to the air leaves out the turbulence's rotation. The rate of angle
of attack reads the rate of the air velocity in body axes, which counts the
wind turning in body axes as the body turns:

    d/dt R^T (v - W x r - u) = f / m + R^T (g - W x v - W x u) - w x R^T (v - W x r - u)

`rigid_aircraft_test` checks it against a central difference along a body's
motion, over a flat Earth and a round one. JSBSim's `FGAuxiliary` takes the
rate of angle of attack from the rate of the velocity over the ground, which
leaves out w x R^T u.

The check cases add a wind: from JSBSim's trim, a wind of 8 m/s north,
12 m/s west and 2 m/s down starts at 1.004 s, between two of the F-16's
flight control frames, and holds. The largest distance from JSBSim's
converged flight over 30 s:

| Aircraft | simon at 0.5 ms | simon at 8 ms | JSBSim at 8 ms |
|---|---:|---:|---:|
| 737 | 92 cm | 96 cm | 11.7 cm |
| F-16 | 4.4 mm | 3.8 cm | 2.4 cm |

The F-16's aerodynamics do not read the rate of angle of attack, and in wind
it agrees with JSBSim to millimeters, as in still air. The 737's pitching
moment does, and it parts from JSBSim by 92 cm; flown with JSBSim's rate, it
agrees to 1.7 cm. The wind starts between 8 ms steps, so at 8 ms simon and
JSBSim each start it 4 ms off the reference.

Turbulence follows MIL-F-8785C. The gusts along the path (u), across it (v)
and down (w) have its Dryden spectra, and the roll, pitch and yaw gusts
follow from them over the wing span. The intensities come from its
low-altitude model up to 1,000 ft and its table of intensities by
probability of exceedance above 2,000 ft. Light, moderate and severe are
exceeded with probabilities of 10^-2, 10^-3 and 10^-5. The turbulence is
frozen in the air, and each aircraft meets it along its own path at its
airspeed, from its own seed.

- **Each filter is sampled exactly.** A step draws the filter's next state
  from the distribution the continuous filter reaches over the step, so the
  gusts' variance and correlation are the specification's at any step.
  `wind_test` checks them against Dryden's at 20 ms and at 250 ms. JSBSim's
  MIL-F-8785C turbulence advances its filters by Euler steps and takes the
  second-order v and w spectra as first-order, so its turbulence changes
  with its frame.
- **Turbulence needs no time to build up.** The first draw starts each filter
  from its steady distribution.
- **The pitch and yaw gusts have the air's own signs,** q = -dw/dx and
  r = dv/dx, through lags of 4b / (pi V) and 3b / (pi V).
- **The noise needs no generator state.** It is SplitMix64, counted by step
  from the aircraft's seed, so a run repeats exactly from its seed.

`aeronautic_test` flies 737s, F-16s and point-mass aircraft in a 15 m/s wind and
moderate turbulence for five minutes, and checks that they keep to the
routes' envelope and reach waypoints. It also checks that a point-mass
aircraft in wind flies as it does in still air, carried by the wind.

## Roadmap

**Flight dynamics (in progress).** Run flight control algorithms of the
class JSBSim runs, at scale, with fidelity as an opt-in (see
[Choose fidelity per archetype](../../documents/design.md#choose-fidelity-per-archetype)).

- Done: control blocks in `model/`: exact first-order lags, rate limits,
  PI control, and lookup tables with linear interpolation.
- Done: the standard atmosphere, a point-mass flight-path model, and this
  application flying it at two fidelity levels.
- Done: `aeronautic_benchmark`, idle and contended, for both levels.
- Done: accuracy against JSBSim's 737 (see
  [Accuracy against JSBSim](#accuracy-against-jsbsim)).
- Done: rigid aircraft, six degrees of freedom from JSBSim aircraft
  converted to data, round a WGS84 Earth or over a flat one, checked
  against JSBSim layer by layer and whole (see
  [Rigid aircraft](#rigid-aircraft)).
- Done: every level in one world, rigid aircraft flying routes by a small
  surface autopilot (see [Mixed fidelity](#mixed-fidelity)).
- Done: a second, very different aircraft, JSBSim's F-16, with
  fly-by-wire flight controls and reheat, converted by the same code and
  checked the same way (see [The F-16](#the-f-16)).
- Done: a trim of simon's own, level over the round Earth (see
  [Trim](#trim)).
- Done: an aeronautic viewer for the mixed world (see
  [Mixed fidelity](#mixed-fidelity)).
- Done: wind and MIL-F-8785C turbulence, opt in (see
  [Wind and turbulence](#wind-and-turbulence)).
- Parked: many rigid aircraft batched in one segment, until more than the
  6,900 one thread flies in real time are needed.
- Later: trim tables computed offline. JSBSim, run offline, stays the
  reference each level's accuracy is measured against.
