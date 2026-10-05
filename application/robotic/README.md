# robotic

The robotic application shows that simon's framework can carry articulated
rigid bodies with contact, as MuJoCo simulates them: the first system that
solves many entities together. It is in progress; the details and the
roadmap are in [Design.md](Design.md).

Its models come first: read from MuJoCo's MJCF and compiled as MuJoCo
compiles them, every mass, inertia, frame, joint and shape of its test
models equals MuJoCo's to the last bit. Its dynamics without contact do
too, step by step, through a chaotic double pendulum's 3,000 steps; a
quaternion's integration parts by at most 2e-14.

```sh
bazel test //application/robotic/...
```
