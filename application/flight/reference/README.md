# Reference data

The tables here come from flying [JSBSim](https://github.com/JSBSim-Team/jsbsim)
(LGPL 2.1 or later) with its 737 model. Dave Culp and Aeromatic wrote the model
and released it under the GPL. Its authors say it is meant for educational and
entertainment purposes only. The tests compare simon against these tables.

Each script regenerates its table, and says how in its docstring. The scripts
need JSBSim's Python package (`pip install jsbsim numpy`).

| Script | Table | Test |
|---|---|---|
| `jsbsim_737.py` | The 737's path and the point-mass controls that fly it | `accuracy_test` |
| `jsbsim_737_aero.py` | The 737's aerodynamic loads at recorded states | `aero_test` |
| `jsbsim_737_rigid_body.py` | The 737's equations of motion at recorded states | `rigid_body_test` |
