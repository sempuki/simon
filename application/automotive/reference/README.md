# Reference data

The tables here check simon's roads against
[libOpenDRIVE](https://github.com/pageldev/libOpenDRIVE) (Apache-2.0), an
OpenDRIVE library that renders and queries road networks, on the roads in
`../roads` and `3rd_party/carla`, and its vehicles against CommonRoad's reference models:

| Road | From | Shows |
|---|---|---|
| `curves.xodr` | `make_test_roads.py` | Lines, spirals and arcs of either hand, elevation, superelevation, a lane offset, and lane widths that change and open over three lane sections |
| `paramPoly3.xodr` | `make_test_roads.py` | Parametric cubics over arcLength and normalized ranges, one at map coordinates 500 km east and 5,400 km north |
| `ring.xodr` | `make_test_roads.py` | Two half circles leading into each other, two lanes each way, for traffic |
| `rings.xodr` | `make_test_roads.py` | 100 rings of 1 km radius, three lanes each way, for the scale benchmark |
| `signalized.xodr` | `make_test_roads.py` | Four 100 m arms with sidewalks meeting in a junction, each approach with a traffic light at its stop line and a crosswalk, two in road coordinates and two in their own frame; two controllers group the lights |
| `priority.xodr` | `make_test_roads.py` | A T whose minor road gives way, with the junction's priorities, a give-way sign and a speed limit |
| `crosswalks.xodr` | `make_test_roads.py` | A curve that climbs and leans, with a crosswalk turned, pitched and rolled in its own frame and one in road coordinates |
| `3rd_party/chrono/Sedan_Pac02Tire.tir` | Project Chrono's data (BSD-3-Clause, see `3rd_party/chrono/LICENSE`) | The Sedan's tire, a 245/40 R 18, as a TNO property file in the PAC2002 format |
| `3rd_party/esmini/` | esmini's resources (MPL-2.0, see `3rd_party/esmini/LICENSE`) | Three scenarios, cut-in_simple, cut-in and lane_change_simple, cut-in's parameter distribution, cut-in_parameter_set, their roads, straight_500m, e6mini and curve_r100, and the vehicle catalog; and two roads with signs from its unit tests, some_signs and lane_offset_intersection |
| `3rd_party/carla/Town01.xodr` | CARLA's [OpenDRIVE test files](https://github.com/carla-simulator/opendrive-test-files) (MIT, see `3rd_party/carla/LICENSE`) | A town of 98 roads, as RoadRunner writes them |

| Script | Table | Test |
|---|---|---|
| `make_test_roads.py` | `../roads/curves.xodr`, `../roads/paramPoly3.xodr`, `../roads/ring.xodr`, `../roads/rings.xodr`, `../roads/signalized.xodr`, `../roads/priority.xodr`, `../roads/crosswalks.xodr` | `opendrive_reference_test`, `automotive_test`, `automotive_benchmark` |
| `libopendrive_reference.cpp` | `libopendrive_positions.csv`, `libopendrive_borders.csv`, `libopendrive_lanes.csv`, `libopendrive_successors.csv` (its routing graph), `libopendrive_signals.csv`, `libopendrive_objects.csv` (outline corners), `libopendrive_junctions.csv` (priorities and controllers) | `opendrive_reference_test` |
| `commonroad_kinematic.py` | `commonroad_rates.csv`, `commonroad_paths.csv`: CommonRoad's kinematic single-track model ([commonroad-vehicle-models](https://commonroad.in.tum.de), BSD) | `single_track_test` |
| `commonroad_dynamic.py` | `commonroad_vehicles.csv`: every parameter of CommonRoad's vehicles; `commonroad_tires.csv`, `commonroad_dynamic_rates.csv`, `commonroad_dynamic_paths.csv`: its tire and its dynamic, drift and multibody models, with four corrections patched in | `single_track_test`, `vehicle_dynamics_test` |
| `movsim_reference.js` | `movsim_idm.csv`, `movsim_mobil.csv`: IDM and MOBIL as their authors implement them ([traffic-simulation.de](https://github.com/movsim/traffic-simulation-de), GPL-3.0) | `traffic_test` |
| `sumo_platoon.py` | `sumo_platoon.csv`: a platoon of IDM followers in [SUMO](https://eclipse.dev/sumo) (EPL-2.0), at 0.1 s and 0.001 s | `traffic_test` |
| `chrono_reference.cpp` | `chrono_tires.csv`, `chrono_sedan.csv`, `chrono_maneuvers.csv`: [Project Chrono](https://projectchrono.org)'s Pac02 tire, and its Sedan at rest and through the handling maneuvers (BSD-3-Clause, see `3rd_party/chrono/LICENSE`) | `tire_test`, `maneuver_test` |
| `esmini_scenarios.py` | `esmini_scenarios.csv`: [esmini](https://github.com/esmini/esmini)'s scenarios played in esmini, every entity at every step (MPL-2.0, see `3rd_party/esmini/LICENSE`); `esmini_permutations.csv`, `esmini_parameters.csv`: the 12 permutations of cut-in_parameter_set, every entity at every step with its box, and the parameter values esmini gave each | `scenario_test`, `batch_test` |
| `shapely_boxes.py` | `shapely_boxes.csv`: 2,000 pairs of boxes, whether they intersect and how far apart they are, by [Shapely](https://shapely.readthedocs.io) 2.1.2 on GEOS 3.13.1 (BSD-3-Clause, LGPL-2.1) | `collision_test` |
| `nuplan_metrics.py` | `nuplan_metrics.csv`: the ego's comfort signals and time to collision in esmini's scenarios, by [nuPlan](https://github.com/motional/nuplan-devkit)'s devkit 1.2.2 (Apache-2.0); `nuplan_permutations.csv`, `nuplan_verdicts.csv`: its time to collision and gap at every sample of esmini's permutations, and each run's verdicts | `metrics_test`, `batch_test` |
| `sumo_benchmark.py` | None: SUMO's time per vehicle-step on `rings.xodr`, printed | `automotive_benchmark`, by comparison |
| `exact_param_poly3.py` | `exact_positions.csv`: the parametric cubics by exact arc length | `opendrive_reference_test` |

`make_test_roads.py` and `exact_param_poly3.py` need NumPy, and
`commonroad_kinematic.py` and `commonroad_dynamic.py` also need `pip install commonroad-vehicle-models`,
`sumo_platoon.py` `pip install eclipse-sumo traci`, and `sumo_benchmark.py`
`pip install eclipse-sumo`. `movsim_reference.js`
runs under Node.js beside a clone of movsim. Each script says
how to run it in its docstring; `libopendrive_reference.cpp` is built beside
libOpenDRIVE with CMake, outside simon's build, and `chrono_reference.cpp`
beside Chrono 10.0.0, built with its vehicle module, the same way;
`esmini_scenarios.py` runs esmini 3.8.2, built from source;
`shapely_boxes.py` needs `pip install shapely numpy`, and `nuplan_metrics.py`
a clone of nuPlan's devkit.
