# simon

Simulation Out of the Box: a C++26 framework for entity-component simulations
that run at scale, with higher fidelity as an opt-in that costs nothing for
simulations that don't use it.

## What this proves

simon is a framework for entity-component simulations. Its flight
application shows that the framework can carry a flight simulator comparable
to [JSBSim](https://github.com/JSBSim-Team/jsbsim), a widely used open-source
flight dynamics model. The application's physics matches JSBSim's layer by
layer, and one thread flies a hundred thousand aircraft at mixed fidelity.
Each claim below names the command that checks it. Timings are GCC builds on
one core of an AMD Ryzen 9 5900XT. The details are in
[documents/design.md](documents/design.md#flight).

### The physics matches JSBSim

An offline converter (`tools/jsbsim/convert.py`) turns JSBSim's 737 into
simon's own aircraft format. The application rebuilds each layer of the
aircraft, and each layer is tested against tables recorded from JSBSim:

| Test | Checks | Agreement |
|---|---|---:|
| `aero_test` | Aerodynamic forces and moments at 240 states, from ground effect to Mach 0.94 | 5e-14 |
| `rigid_body_test` | Equations of motion, air data and mass balance at 480 states, at the equator and 60° north | 5e-13 |
| `turbine_test` | Spools, thrust and fuel flow, frame by frame through throttle steps | 1e-13 |
| `flight_control_test` | Surface positions, frame by frame through command sweeps | 4e-15 |

These differences are rounding in double precision.

```sh
bazel test //application/flight:aero_test //application/flight:rigid_body_test \
    //application/flight:turbine_test //application/flight:flight_control_test
```

### The whole aircraft matches JSBSim, and beats it at the same step

`check_case_test` flies the 737 open loop for 30 s from JSBSim's trim at 6 km,
round the rotating WGS84 Earth, through three doublets and a throttle step.
The reference is JSBSim at 0.5 ms, where its integrators have converged. The
largest distance from that reference over 30 s:

| Case | simon at 0.5 ms | simon at 8 ms | JSBSim at 8 ms |
|---|---:|---:|---:|
| Trim hold | 2.1 mm | 2.8 mm | 1.6 mm |
| Elevator doublet | 0.9 mm | 1.3 mm | 8.9 mm |
| Aileron doublet | 2.2 mm | 6.1 mm | 11.7 mm |
| Rudder doublet | 2.4 cm | 3.8 cm | 41.5 cm |
| Throttle step | 3.9 mm | 6.0 cm | 11.9 cm |

The first column shows the application's physics agreeing with JSBSim's to
millimeters. At the same 8 ms step, the framework's Runge-Kutta 4 lands closer
to the converged answer in every case with an input, and 11 times closer after
the rudder doublet. The application also leaves out three of JSBSim's
shortcuts. It has no one-frame lags in induced drag, angle-of-attack rate or
flight-control inputs, it computes geodetic altitude exactly, and it uses exact
unit constants.

```sh
bazel test //application/flight:check_case_test   # about 90 s
```

### It runs faster than JSBSim

| Rigid 737s | Flat Earth | Round Earth |
|---:|---:|---:|
| 100 | 1.83 µs | 2.80 µs |
| 1,000 | 1.77 µs | 2.82 µs |
| 10,000 | 1.77 µs | 2.80 µs |

The cost per aircraft-step stays flat with population. JSBSim takes 9.2 µs a
frame for one 737, timed through its Python module. The comparison is rough,
since JSBSim's frame also runs ground reactions and its property tree. Even so,
the application evaluates the aircraft four times a step to JSBSim's once and
is 3.3 times faster round the Earth. One thread flies about 4,500 rigid 737s
in real time over a flat Earth.

```sh
bazel run -c opt //application/flight:rigid_benchmark
```

### It scales

Point-mass aircraft fly routes under an autopilot, at 20 ms steps:

| Aircraft | Single pass | Runge-Kutta 4 |
|---:|---:|---:|
| 1,000 | 0.07 ms | 0.17 ms |
| 10,000 | 0.72 ms | 1.87 ms |
| 100,000 | 7.25 ms | 19.1 ms |

A single-pass aircraft costs about 73 ns a step at any population, so 100,000
of them run 2.8 times faster than real time.

```sh
bazel run -c opt //application/flight:flight_benchmark
```

### Fidelity costs only the aircraft that use it

The framework makes fidelity a choice per archetype. A level's systems run
only on entities whose components opt in, so a simulation pays nothing for a
level it doesn't use. One world at 100,000 aircraft:

| Aircraft | ms per step | Share |
|---|---:|---:|
| 98,900 single pass | 7.17 | 95.2% |
| 1,000 Runge-Kutta 4 | 0.17 | 2.3% |
| 100 rigid 737s | 0.19 | 2.5% |
| All | 7.53 | |

The single-pass aircraft cost the same per aircraft as they do alone. The
rigid 737s fly the same routes as everyone else, under a deliberately small
autopilot, and at the world's 20 ms step they stay within 15 cm of the
converged JSBSim reference.

```sh
bazel run -c opt //application/flight:flight_benchmark   # the mixed population
bazel run -c opt //application/flight -- 10000 100 10    # 10 minutes of flight in about 25 s
```

### Anyone can regenerate the references

Each JSBSim table comes from a script in
[application/flight/reference](application/flight/reference/README.md). The
tests compare against the committed tables, so they run without JSBSim. To
regenerate a table, run its script after `pip install jsbsim numpy`.

### What it does not show yet

- **A second aircraft.** Every JSBSim comparison uses the 737. A very
  different aircraft would show that the converter and models generalize.
- **Point-mass accuracy.** The point-mass model drifts 1.8 km from JSBSim
  over a 130 km flight (`accuracy_test`). Its fitted drag polar causes most of
  that drift. The model suits traffic at scale, and the rigid model suits
  handling.
- **Ground, wind and stall.** Routes stay between 3 and 9 km.

## Build

Requires [Bazelisk](https://github.com/bazelbuild/bazelisk) (installed as
`bazel`); the Bazel release is pinned in `.bazelversion`. All dependencies,
SDL2 included, come from the Bazel Central Registry, except the shared core
libraries in the [lib](https://github.com/sempuki/lib) submodule:

```sh
git clone --recurse-submodules git@github.com:sempuki/simon.git
# or, in an existing clone:
git submodule update --init

bazel test //...
bazel run //application/hello
bazel run //application/missile:viewer   # watch a missile scenario
bazel run //application/missile -- 7    # run seed 7 headless
bazel run //application/flight -- 1000 100   # 1,000 aircraft, 100 on RK4
bazel run -c opt //application/flight:flight_benchmark
```

Code targets C++26; flags come from `@lib//bazel:copts.bzl`.

The architecture, its decisions and the roadmap are in
[documents/design.md](documents/design.md).

## Editor setup

clangd needs a `compile_commands.json`, and the headers it names must stay put.
Bazel's execution root does not: every build relinks it to only the external
repositories that build needed. lib's `bazel/lsp_mirror.py` builds in an output
base of its own, copies the headers clangd reads into `.lsp/mirror/` (ignored
by git and Bazel), and writes `compile_commands.json` against that mirror, so
builds and compiler switches never disturb your editor:

```sh
python3 2nd_party/lib/bazel/lsp_mirror.py                  # build the mirror now
python3 2nd_party/lib/bazel/lsp_mirror.py --if-stale       # only if anything changed
python3 2nd_party/lib/bazel/lsp_mirror.py --watch 60       # check every minute
python3 2nd_party/lib/bazel/lsp_mirror.py --install-hooks  # after checkout, merge, rebase
```

`--if-stale` takes a fraction of a second when nothing changed, so it is cheap
to run often. Restart clangd (`:LspRestart` in Neovim) after the first build.
