# robotic

The robotic application shows that simon's framework can carry articulated
rigid bodies with contact, as MuJoCo simulates them: the first system that
solves many entities together. It is in progress; the details and the
roadmap are in [Design.md](Design.md).

Its models come first: read from MuJoCo's MJCF and compiled as MuJoCo
compiles them, every mass, inertia, frame, joint and shape of its test
models equals MuJoCo's to the last bit. Its dynamics without contact do
too, step by step, through a chaotic double pendulum's 3,000 steps; a
quaternion's integration parts by at most 2e-14. So do its actuators, and a
cart-pole balanced by MuJoCo's own linear quadratic regulator. So do its
contacts: at 400 random poses of every primitive pair, all 5,465 of MuJoCo's
contacts, each to the last bit. Its constraint solver, by Newton's method,
steps a rolling sphere, a sliding box, a resting stack and joints against
their limits within 1e-11 of MuJoCo; the box slides as far as Coulomb
friction allows, and the sphere rolls at five sevenths of its speed.
MuJoCo's humanoid, with its tendons, compiles to the last bit and falls
within 1e-11 of MuJoCo's. Ten thousand loose bodies and a thousand
humanoids step within 1.2x of MuJoCo's time on one thread. Four robots of
MuJoCo Menagerie, Go1, H1, UR5e and ANYmal C, run from their own MJCF
within 1e-11 of MuJoCo.

```sh
bazel run //application/robotic:viewer
```

```sh
bazel test //application/robotic/...
```
