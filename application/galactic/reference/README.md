# Reference data

The tables here check galactic's gravity and time stepping against
[REBOUND](https://rebound.readthedocs.io) (GPL-3.0):

| Script | Writes | Read by |
|---|---|---|
| `rebound_gravity.py` | `rebound_forces.csv`: two random clusters of bodies, 64 softened and 16 not, with the accelerations REBOUND sums for them; `rebound_leapfrog.csv`: the softened cluster every 10 of 400 leapfrog steps, kick-drift-kick, each kick taking REBOUND's accelerations | `gravity_test` |
| `rebound_tree.py` | `rebound_tree.csv`: a Plummer sphere of 2,000 bodies, with the accelerations REBOUND sums for them directly and by its tree at opening angles of 0.25, 0.5, 0.75 and 1 | `tree_test` |
| `rebound_encounter.py` | `rebound_encounter.csv`: Toomre and Toomre's flat direct encounter, two masses of 10^11 suns and 120 test particles, every 400 of 8,000 steps of 250,000 years | `encounter_test` |

`pip install rebound numpy` (REBOUND 5.2.2) runs the scripts.
