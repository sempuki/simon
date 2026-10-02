# Reference data

The tables here come from flying [JSBSim](https://github.com/JSBSim-Team/jsbsim)
(LGPL 2.1 or later) with its 737 and F-16 models. Dave Culp and Aeromatic wrote
the 737 and Erik Hofman the F-16, and released them under the GPL. The 737's
authors say it is meant for educational and entertainment purposes only. The tests compare simon against these tables.

Each script regenerates its table, and says how in its docstring. The scripts
need JSBSim's Python package (`pip install jsbsim numpy`).

| Script | Table | Test |
|---|---|---|
| `jsbsim_737.py` | The 737's path and the point-mass controls that fly it | `accuracy_test` |
| `jsbsim_737_aero.py` | The 737's aerodynamic loads at recorded states | `aero_test` |
| `jsbsim_737_rigid_body.py` | The 737's equations of motion, air data and mass balance at recorded states | `rigid_body_test` |
| `jsbsim_737_turbine.py` | The 737's turbines, frame by frame through throttle changes | `turbine_test` |
| `jsbsim_737_flight_control.py` | The 737's flight controls, frame by frame through command sweeps | `flight_control_test` |
| `jsbsim_737_check_cases.py` | The whole 737, trimmed and flown open loop through doublets and a throttle step, at 8 ms and 0.5 ms | `check_case_test` |
| `jsbsim_f16_aero.py` | The F-16's aerodynamic loads at recorded states | `aero_test` |
| `jsbsim_f16_turbine.py` | The F-16's turbine, frame by frame into reheat and out | `turbine_test` |
| `jsbsim_f16_flight_control.py` | Every block of the F-16's flight controls, frame by frame through three flights | `flight_control_test` |
| `jsbsim_f16_check_cases.py` | The whole F-16, trimmed and flown through doublets and a throttle step into reheat, at 8 ms and 0.125 ms, its flight controls every 8 ms | `check_case_test` |
