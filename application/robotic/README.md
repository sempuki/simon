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
contacts, each to the last bit.

```sh
bazel test //application/robotic/...
```
