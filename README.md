# simon

Simulation Out of the Box: a C++26 framework for entity-component simulations
that run at scale, with higher fidelity as an opt-in that costs nothing for
simulations that don't use it.

simon is a framework for building simulations in any domain. Each
application in this repository is a simulation built on it, checked against
the best open-source simulator of its domain, to show the framework can
carry a world-class simulation of that kind.

## The framework

| Directory | Holds |
|---|---|
| [framework/](framework) | Entities, component stores in archetype segments, the world, systems and schedules, names, builders and commands, spatial indexes |
| [engine/](engine) | The lifecycle, batch and real-time drivers, rate gates, continuous state with Euler, midpoint and Runge-Kutta 4, events |
| [model/](model) | Physics and maths shared by applications, as free functions: frames, the atmosphere and Earth, rigid bodies, aircraft and engines, roads and lanes, tires, vehicles and drivers |
| [format/](format) | Readers into model data: OpenDRIVE, OpenSCENARIO, tire property files, converted aircraft |
| [scenario/](scenario) | OpenSCENARIO scenarios, parameter distributions, and the player that runs their storyboards |

Fidelity is chosen per archetype. A system runs only on entities whose
components opt in, so a world can mix cheap and precise entities, and a
simulation pays nothing for a level it doesn't use. Every physical quantity
carries its unit in its type, and time is integer nanoseconds, so runs
repeat exactly.

The architecture, its decisions and the roadmap are in
[framework/Design.md](framework/Design.md).

## The applications

| Application | Simulates | Checked against |
|---|---|---|
| [aeronautic](application/aeronautic/README.md) | Aircraft flying routes, from point masses to rigid 737s and F-16s with their flight controls, in one world | JSBSim |
| [automotive](application/automotive/README.md) | Roads, traffic, vehicle dynamics and scenarios, at the level of objects | libOpenDRIVE, CommonRoad, Chrono::Vehicle, SUMO, esmini, nuPlan |
| [robotic](application/robotic/README.md) | Articulated rigid bodies on joints with contact, friction, limits, tendons and actuators: MuJoCo's humanoid and Menagerie's robots | MuJoCo, MuJoCo Menagerie |
| [defense](application/defense/README.md) | Red drones against blue radars, launchers and interceptors | Its own benchmarks, idle and under contention |
| [hello](application/hello/README.md) | Balls bouncing in a box under gravity, the smallest complete use of the framework | |

Some headline results, each with the command that checks it in the
application's README:

- **aeronautic.** Each layer of the 737 and the F-16 matches JSBSim to
  rounding in double precision, and whole flights stay within millimeters
  of JSBSim's converged solution. One thread flies 100,000 point-mass
  aircraft 3.2 times faster than real time, and a rigid 737 4.6 times faster
  than JSBSim flies one.
- **automotive.** Roads match libOpenDRIVE to 8e-14 m, vehicle models match
  CommonRoad's to rounding, the Sedan matches Chrono's handling maneuvers,
  and esmini's scenarios play out to its log's precision. One thread steps
  100,000 vehicles in 19 ms, 34 times faster than SUMO on the same network.

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
bazel run -c opt //application/hello:viewer   # watch balls bounce in a box
bazel run //application/defense:viewer   # watch a defense scenario
bazel run -c opt //application/aeronautic:viewer   # watch the aeronautic world
bazel run -c opt //application/automotive:viewer   # watch traffic on a ring
bazel run //application/defense -- 7    # run seed 7 headless
bazel run //application/aeronautic -- 1000 100   # 1,000 aircraft, 100 on RK4
bazel run -c opt //application/aeronautic:aeronautic_benchmark
```

Code targets C++26; flags come from `@lib//bazel:copts.bzl`.

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

## Credits

simon is built on open-source work, and thanks everyone behind it. Each
application's README credits the projects it measures itself against or
follows, and `NOTICE.md` keeps the copyright notices and terms of the code
simon translates. The libraries simon builds on:

| Library | Used for | License |
|---|---|---|
| [Eigen](https://eigen.tuxfamily.org) | Linear algebra | MPL-2.0 |
| [mp-units](https://github.com/mpusz/mp-units) | Units of measure | MIT |
| [pugixml](https://pugixml.org) | Reading XML: OpenDRIVE, OpenSCENARIO, MJCF | MIT |
| [Dear ImGui](https://github.com/ocornut/imgui) and [ImPlot](https://github.com/epezent/implot) | The viewers | MIT |
| [SDL2](https://www.libsdl.org) | The viewers' windows | Zlib |
| [Catch2](https://github.com/catchorg/Catch2) | Tests | BSL-1.0 |

Vendored data, each with its source's license, is listed in
[3rd_party/README.md](3rd_party/README.md).
