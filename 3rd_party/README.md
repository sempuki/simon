# Third-party software

Everything simon takes from outside its own code comes through this
directory. BUILD files depend on libraries by `//3rd_party:<name>`, and tests
read vendored data from the subdirectories, each with its source's license.

## Libraries

From the [Bazel Central Registry](https://registry.bazel.build), at the
versions `MODULE.bazel` pins.

| Target | Library | Version | License | Used by |
|---|---|---|---|---|
| `eigen` | [Eigen](https://eigen.tuxfamily.org) | 5.0.1 | MPL-2.0 | `framework/`, `model/` |
| `mp-units` | [mp-units](https://github.com/mpusz/mp-units) | 2.5.0 | MIT | `model/` |
| `pugixml` | [pugixml](https://pugixml.org) | 1.15 | MIT | `format/` |
| `imgui`, `imgui_platform_sdl2`, `imgui_renderer_sdl2` | [Dear ImGui](https://github.com/ocornut/imgui) | 1.92.6-docking | MIT | `application/` viewers |
| `implot` | [ImPlot](https://github.com/epezent/implot) | 0.17 | MIT | `application/` viewers |
| `sdl2` | [SDL](https://www.libsdl.org) | 2.32.0 | Zlib | `application/` viewers |

[Catch2](https://github.com/catchorg/Catch2) 3.15.1 (BSL-1.0) runs the tests,
through `@lib//base:testing` in `2nd_party/lib`.

## Data

| Directory | Target | Holds | From | License |
|---|---|---|---|---|
| `carla/` | `carla` | `Town01.xodr`, a town of 98 roads | CARLA's [OpenDRIVE test files](https://github.com/carla-simulator/opendrive-test-files) | MIT, `carla/LICENSE` |
| `chrono/` | `chrono` | `Sedan_Pac02Tire.tir`, the Sedan's PAC2002 tire | [Project Chrono](https://projectchrono.org)'s data | BSD-3-Clause, `chrono/LICENSE` |
| `esmini/` | `esmini` | Four scenarios and a parameter distribution in `xosc/`, among them one with traffic lights and pedestrians, their roads in `xodr/` with two roads with signs from esmini's unit tests, and the vehicle and pedestrian catalogs | [esmini](https://github.com/esmini/esmini)'s resources | MPL-2.0, `esmini/LICENSE` |
| `jsbsim/` | `jsbsim` | `737.aircraft` and `f16.aircraft`, converted by `tools/jsbsim/convert.py` | [JSBSim](https://github.com/JSBSim-Team/jsbsim)'s aircraft (see `jsbsim/README.md`) | GPL, named in each file's header |

## References

The tools simon is checked against (JSBSim, libOpenDRIVE, CommonRoad, Chrono,
SUMO, esmini, Shapely and nuPlan, among others) are not part of its build.
The scripts that run them, and their versions and licenses, are listed in
`application/aeronautic/reference/README.md` and
`application/automotive/reference/README.md`.
