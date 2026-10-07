# hello

Balls bouncing off each other and the walls of a box, under gravity. It is
the smallest complete use of the architecture: four components, an archetype,
a schedule of three systems, a scenario, a headless runner and a viewer.

Each step, `Integrate` moves every ball under gravity.
`DetectContacts` then has each ball find the balls it touches through the
world's spatial query and sum their forces, and the walls', into its own
`Contact`. `ApplyContacts` then has each ball take that force. A system
writes only the entity it runs for, and every ball reads the same state, so
the two forces of a touching pair are exactly equal and opposite in whatever
order the balls run, and momentum is conserved to rounding.

Contacts are linear springs and dashpots, as in the discrete element method,
sized so that each contact lasts 50 ms and gives back `restitution` times the
speed it took. At one tenth of a contact, the 5 ms step keeps an elastic
run's energy to a few percent over minutes. In the default box, 1,500 balls
settle into an atmosphere: dense at the floor, thinning with height.

```sh
bazel run -c opt //application/hello:viewer          # watch 1,500 balls
bazel run -c opt //application/hello:viewer -- 3000  # watch 3,000
bazel run -c opt //application/hello -- 1500 60 0.5  # 60 s at restitution 0.5, headless
bazel test //application/hello/...
```

The tests check that equal balls meeting head on swap velocities, that a
glancing blow keeps momentum to rounding and energy to 1%, that balls bounce
off the walls, that an elastic run keeps its energy to 3% and an inelastic
one loses it, and that two runs end bit for bit the same.
