# Reference data

The tables here check galactic's gravity and time stepping against
[REBOUND](https://rebound.readthedocs.io) (GPL-3.0):

| Script | Writes | Read by |
|---|---|---|
| `rebound_gravity.py` | `rebound_forces.csv`: two random clusters of bodies, 64 softened and 16 not, with the accelerations REBOUND sums for them; `rebound_leapfrog.csv`: the softened cluster every 10 of 400 leapfrog steps, kick-drift-kick, each kick taking REBOUND's accelerations | `gravity_test` |

`pip install rebound numpy` (REBOUND 5.2.2) runs the scripts.
