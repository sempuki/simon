# robotic

The robotic application shows that simon's framework can carry articulated
rigid bodies with contact, as MuJoCo simulates them: the first system that
solves many entities together. The details and the roadmap are in
[Design.md](Design.md).

It reads models from MuJoCo's MJCF and compiles them as MuJoCo does: every
mass, inertia, frame, joint and shape of its test models is within 1e-12
of MuJoCo's. At 400 random poses of every primitive pair, it finds all
5,465 of MuJoCo's contacts, each within 4e-14. Everything that steps is
held to MuJoCo's accuracy. One step from MuJoCo's own states lands within
how far a few units in the last place move MuJoCo's step, and a whole run
within how far rounding moves MuJoCo's own run. Where its constraint
solver stops elsewhere than MuJoCo's, it is no farther than MuJoCo's from
the step solved to convergence. Without constraints, each
run is no farther than MuJoCo's from Runge–Kutta 4 at a fiftieth of the
timestep. Its dynamics meet this through a chaotic double pendulum's 3,000
steps, staying within 6e-14 of MuJoCo; so do its actuators, and a
cart-pole balanced by MuJoCo's own linear quadratic regulator. Its
constraint solver, by Newton's method and by PGS, meets it for a rolling
sphere, a sliding box, a resting stack and joints against their limits;
the box slides as far as Coulomb friction allows, and the sphere rolls at
five sevenths of its speed. MuJoCo's humanoid, with its tendons, compiles
within 1e-12 and falls as MuJoCo's does to rounding, and so do four robots
of MuJoCo Menagerie, Go1, H1, UR5e and ANYmal C, from their own MJCF. Ten
thousand loose bodies and a thousand humanoids step within 1.25x of
MuJoCo's time on one thread.

```sh
bazel run //application/robotic:viewer
```

```sh
bazel test //application/robotic/...
```

## Credits

robotic is built to match [MuJoCo](https://github.com/google-deepmind/mujoco),
and much of it is MuJoCo's own code, translated. Thank you to Emanuel
Todorov, Google DeepMind and MuJoCo's contributors for making the engine
open, and to the MuJoCo Menagerie's maintainers and the robot makers who
released their models.

| Project | What robotic takes from it | License |
|---|---|---|
| [MuJoCo](https://github.com/google-deepmind/mujoco) 3.14.0 | The reference every check runs against. Its MJCF compiler, smooth dynamics, collision detection, convex collider, constraint solvers, islands and tendons are translated in `format/mjcf`, `model/articulated/articulated*` and the simulation's systems (see `NOTICE.md`). Its humanoid is vendored in `3rd_party/mujoco` | Apache-2.0 |
| [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) | The MJCF of Unitree's Go1 and H1, Universal Robots' UR5e (by ROS-Industrial) and ANYbotics' ANYmal C, vendored without meshes in `3rd_party/menagerie` | BSD-3-Clause, each model's own |

The papers behind the algorithms are cited in
[model/REFERENCES.md](../../model/REFERENCES.md).
