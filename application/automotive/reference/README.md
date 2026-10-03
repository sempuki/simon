# Reference data

The tables here check simon's roads against
[libOpenDRIVE](https://github.com/pageldev/libOpenDRIVE) (Apache-2.0), an
OpenDRIVE library that renders and queries road networks, on the roads in
`../roads`, and its vehicles against CommonRoad's reference models:

| Road | From | Shows |
|---|---|---|
| `curves.xodr` | `make_test_roads.py` | Lines, spirals and arcs of either hand, elevation, superelevation, a lane offset, and lane widths that change and open over three lane sections |
| `paramPoly3.xodr` | `make_test_roads.py` | Parametric cubics over arcLength and normalized ranges, one at map coordinates 500 km east and 5,400 km north |
| `ring.xodr` | `make_test_roads.py` | Two half circles leading into each other, two lanes each way, for traffic |
| `Town01.xodr` | CARLA's [OpenDRIVE test files](https://github.com/carla-simulator/opendrive-test-files) (MIT, see `../roads/LICENSE-CARLA`) | A town of 98 roads, as RoadRunner writes them |

| Script | Table | Test |
|---|---|---|
| `make_test_roads.py` | `../roads/curves.xodr`, `../roads/paramPoly3.xodr` | `opendrive_reference_test` |
| `libopendrive_reference.cpp` | `libopendrive_positions.csv`, `libopendrive_borders.csv`, `libopendrive_lanes.csv`, `libopendrive_successors.csv` (its routing graph) | `opendrive_reference_test` |
| `commonroad_kinematic.py` | `commonroad_parameters.csv`, `commonroad_rates.csv`, `commonroad_paths.csv`: CommonRoad's kinematic single-track model ([commonroad-vehicle-models](https://commonroad.in.tum.de), BSD) | `single_track_test` |
| `exact_param_poly3.py` | `exact_positions.csv`: the parametric cubics by exact arc length | `opendrive_reference_test` |

`make_test_roads.py` and `exact_param_poly3.py` need NumPy, and
`commonroad_kinematic.py` also needs `pip install commonroad-vehicle-models`. Each script says
how to run it in its docstring; `libopendrive_reference.cpp` is built beside
libOpenDRIVE with CMake, outside simon's build.
