# robotic design

Articulated rigid bodies on joints, with contact, friction and actuators:
robot arms, legged robots, humanoids and piles of loose objects. It proves
the framework can carry the first kind of system it has not yet carried, a
solve that couples entities: contacts tie bodies together, and each step
solves them at once. MuJoCo (Todorov, Erez and Tassa, "MuJoCo: A physics
engine for model-based control", IROS 2012; Apache-2.0) is the reference.
It is being built in eight steps (see the [Roadmap](#roadmap)); models read
and compiled as MuJoCo compiles them, their dynamics without constraints,
their actuators and control, their contacts, the constraint solver,
MuJoCo's humanoid, and scale against MuJoCo with a viewer are done.

## Choices

Each choice says what it is, why, and where it comes from.

1. **Generalized coordinates.** Each kinematic tree is its joints'
   positions and velocities; joints are exact, and only contacts, limits
   and equalities are constraints. Matching MuJoCo to rounding needs its
   formulation, and long chains do not drift. Maximal coordinates, a free
   body per link with joints as constraints, as Bullet, PhysX and ODE have
   it, fit an ECS more naturally and scale for loose bodies, but joints drift.
   The dynamics follow Featherstone, *Rigid Body Dynamics Algorithms*, 2008:
   the composite rigid body algorithm, recursive Newton–Euler, and LDLᵀ
   along the tree.
2. **An entity is a tree.** A robot is one entity; each loose body is its
   own, on a free joint. The model's layout, axes and shapes are read-only
   data, as automotive's network is; an entity holds its state. A robot's
   state is tightly coupled within, and trees meet only through contacts,
   so the entity's boundary lies where coupling is weakest.
3. **State sized at compile time, per archetype.** A free body has 7 and 6
   numbers, a humanoid 28 and 27, each archetype's components sized to its
   own, inline and contiguous, so ten thousand loose bodies stream as
   vehicles do; a pool of vectors of any length would bring back scattered
   reads. You pay only for what you use.
4. **Contacts solved by island, through propose and resolve.** `prepare`
   joins entities in contact into islands, by union and find, solves each,
   and leaves results in scratch; each entity's step applies its own. A
   system so writes only its own entity, and islands, independent, run in
   parallel and repeat exactly.
5. **MuJoCo's soft convex contacts** (Todorov, "Convex and
   analytically-invertible dynamics with contacts and constraints", ICRA
   2014): solref and solimp, the pyramidal friction cone, and the Newton and
   PGS solvers; the elliptic cone and CG later, opt in.
6. **Collision from primitives first:** plane, sphere, capsule, box and
   cylinder, by MuJoCo's analytic colliders, the broad phase on simon's
   spatial index over bounding spheres; the pairs MuJoCo sends to its general
   convex collider (ellipsoids, a cylinder with a capsule, a box or a
   cylinder) and convex meshes, by GJK and EPA, later, opt in.
7. **Semi-implicit Euler at 2 ms first,** MuJoCo's default; implicitfast and
   Runge–Kutta 4 later, opt in.
8. **MJCF first,** MuJoCo's format, its example models Apache-2.0; URDF
   later.
9. **Matched to rounding without contact,** as aeronautic against JSBSim; with
   contact, to the solver's tolerance over a short horizon, since contact
   is chaotic, and by physical checks beyond: a box sliding as far as
   Coulomb friction gives, a stack at rest within a set penetration, energy
   kept where nothing takes it away.

## Models

`format/mjcf` reads MJCF and compiles it into `model/articulated`'s
`ArticulatedModel` as MuJoCo 3.14.0's compiler does, in its order of
operations: default classes, each from its parent, and `childclass`;
attribute vectors given in part; angles in degrees unless the compiler says
radians; orientations by quaternion, axis and angle, Euler angles on moving
or fixed axes, x and y axes, or z axis; capsules, cylinders, boxes and
ellipsoids by `fromto`; each geom's mass from its density or mass and its
moments from its shape; a body's inertia combined from its geoms by the
parallel axis theorem and put on principal axes by Jacobi rotations kept
as a quaternion, or given explicitly, a full tensor turned to the body's
axes first; joints' limits from their ranges, positions at rest and spring
references; and each degree of freedom's place in the tree. Anything that
would change how a model moves and that simon does not yet run is refused:
spatial tendons and tendon springs, equalities, meshes, explicit contact
pairs, actuators other than motors, position and velocity servos and
general actuators with fixed gains, and integrators other than Euler. Fixed
tendons, a sum of joints' positions with limits and dry friction, and
contact exclusions between bodies are read. What only shows a model is left out. A body's inertial frame
and a geom's frame within 1e-6 of the body's frame, or of the inertial
frame, are snapped to it, as MuJoCo snaps them; poses in the world then
come from the same arithmetic.

`model_test` compiles eleven test models with every compiler feature (a
pendulum, a double pendulum, a cart-pole, a tumbling free body of three
offset geoms, a model of default classes and every orientation, a stack of
boxes, an actuated arm, a model of every primitive pair, a rolling sphere,
sliding bodies and limited joints) and MuJoCo's humanoid, with its tendons
and exclusions, and checks every compiled value against MuJoCo's
(`reference/mujoco_models.py`): every one is equal, to the last bit.

## Dynamics

`model/articulated_dynamics` steps one tree as MuJoCo steps a model, in
MuJoCo 3.14.0's order of operations: forward kinematics, each joint turning
or sliding its body from the parent's frame; each body's inertia and each
degree of freedom's motion in a frame at the tree's center of mass; the
mass matrix by the composite rigid body algorithm, armature on its
diagonal; its LDLᵀ along the tree, each row only over its ancestors; the
bias forces, gravity and the velocity products, by recursive Newton–Euler;
joint springs, about their reference positions, quaternions differenced as
angular velocities; dof dampers; and motors, each control clamped and times
its gear. Semi-implicit Euler then advances velocities by the accelerations
and positions by the new velocities, quaternions turned by the angular
velocity; where a dof of the model is damped or actuated, the step solves
M + h B, as MuJoCo's does, so that dampers stay stable at any step.
MuJoCo's implicitfast integrator is there too: it solves M - h D, D the
derivative of the dampers' and actuators' forces in the velocities, and
for a lone free body its own 6 by 6 system with the derivative of the
velocity products, by LU. Runge–Kutta 4 and the fully implicit integrator
are refused: the first runs the whole step, collision and solver
included, four times, and the second needs the velocity products'
derivative for every tree. MuJoCo also keeps a
fixed inertia for dofs whose bodies never turn; simon recomputes it, which
can differ in the last bit.

In the ECS, an entity is a tree, of one of three archetypes by its
capacity: a single body of at most 6 degrees of freedom, a small tree of
at most 4 bodies and 8, and a large tree of at most 16 and 32. Each tree
takes the smallest that fits, so a loose body streams half the bytes it
would as a small tree. Each archetype's state and work are sized at compile
time, inline in its components. `Forward` computes each tree's poses, its
factored mass matrix and its accelerations into its `TreeDynamics`;
`Bound` its sphere, the world's spatial component; and `Integrate` steps
its `TreeState`. `Collide` finds the contacts and `Solve` the constraint
forces between them.

`dynamics_test` steps six cases with constraints off, and checks every
position and velocity at every step against MuJoCo
(`reference/mujoco_dynamics.py`):

| Case | Steps | Position | Velocity |
|---|---:|---:|---:|
| A pendulum | 2,000 | equal | equal |
| A double pendulum, chaotic | 3,000 | equal | equal |
| A free body of three geoms, tumbling without gravity | 2,000 | 2.0e-15 m | 3.8e-15 m/s |
| Ball, slide and hinge joints with springs, implicit dampers and armature | 1,000 | 8.9e-16 | 1.0e-14 |
| A cart-pole pushed by its motor | 200 | equal | equal |
| Five boxes falling, five trees | 500 | equal | equal |

Where only hinges and slides move, every step equals MuJoCo's to the last
bit; where a quaternion is in play, its integration and normalization part
from MuJoCo's by an ulp or two.

## Control

An actuator drives a hinge or slide by a fixed gain times its control,
plus, for an affine bias, a constant and terms in its joint's length and
velocity, each times the gear; the force is clamped to its range and acts
on the joint times the gear. MJCF's motor has gain 1 and no bias; its
position servo gain kp and bias 0, -kp, -kv; its velocity servo gain kv and
bias 0, 0, -kv; its general actuator says so itself. An actuator's damping
joins its joint's, times the gear squared, in the passive forces and in the
implicit Euler, as MuJoCo has it; a servo's kv is explicit, as MuJoCo's is,
so a stiff servo on a light link needs a step to suit it. Activation
dynamics and gain types other than fixed are refused.

Controls are their own component, `TreeControl`, so that `Control` writes
them while `Integrate` writes the state. `Control` applies a linear state
feedback, u = u0 - K (x - x0), x the model's positions and velocities, each
tree's controls from its own state, where the scenario gives one; else the
controls hold.

`control_test` checks two cases against MuJoCo, every step
(`reference/mujoco_control.py`): an arm of four joints under two position
servos, one geared and one taking its gain from a default class, a
velocity servo with damping and a general actuator against its force range,
for 3 s; and the cart-pole balanced from 0.2 rad by the discrete linear
quadratic regulator of MuJoCo's own linearization, for 15 s, the pole
upright within a microradian at the end. Both are equal to MuJoCo's to the
last bit.

## Contacts

`model/articulated_collision` finds contacts as MuJoCo 3.14.0 does. Two
bodies may touch unless they are on one rigid assembly (a body without
joints is welded to its parent), neither can move, or one's assembly is the
other's parent's; two geoms, if one's contact type meets the other's
affinity and their bounding spheres overlap within their margins and gaps.
MuJoCo's analytic colliders then run in its order of operations, the geom
of lower type first: a plane with a sphere, a capsule, a cylinder or a box;
a sphere with a sphere, a capsule, a cylinder or a box; a capsule with a
capsule or a box; and a box with a box, by the separating axis test, faces
preferred on near-ties, then either the nearest points of two edges or the
other box's face clipped to the reference face. Each contact takes its
parameters from the geom of higher priority, or else the larger condim,
the larger frictions and an even mix of solref and solimp, and its frame
from its normal. A model with a pair only MuJoCo's general convex collider
handles is refused.

`Collide` runs once a step, after `Bound`, on the whole world: it places
every geom from its tree's poses, then collides the world's geoms and the
planes with every body, each tree's bodies with each other, and the trees
whose spheres overlap, found in the world's spatial index; the contacts,
ordered by their bodies as MuJoCo orders its body pairs, are shared with
the systems after it, and each tree's `Touching` counts its own.

`collision_test` poses `models/collisions.xml`, two planes (one tilted on a
body that cannot move), two spheres, two capsules, a cylinder, two boxes,
a box that cannot move and an arm whose links overlap their parents with a
body welded to one, at 400 random poses, and checks every contact against
MuJoCo's (`reference/mujoco_contacts.py`): all 5,465 contacts are there,
between the same geoms, and every distance, position, frame, dimension,
friction and soft parameter is equal to MuJoCo's to the last bit.

## Constraints

`model/articulated_constraint` solves an island's constraints as MuJoCo
3.14.0 does (Todorov, ICRA 2014). Each constraint is a soft row of the
Jacobian: a dof's or a tendon's dry friction, a joint's or a tendon's limit
once within its margin (a hinge, a slide or a tendon on either side, a ball
past its largest angle), and a contact, frictionless, a pyramidal friction cone of 2 (dim - 1) edges,
or an elliptic cone of dim directions, torsional and rolling friction among
them. Each row's regularization comes
from the inverse inertia it sees, the bodies' and dofs' at rest, and its
impedance from solimp at its distance; its reference acceleration from
solref's stiffness and damping. The forces minimize a convex cost in the
accelerations: Newton's method, its Hessian M + Jᵀ D J over the active rows
by Cholesky, its line search exact on the piecewise quadratic, as MuJoCo's;
or projected Gauss–Seidel on the dual, its rows swept in MuJoCo's shuffled
order with Nesterov's momentum. An elliptic cone's cost has three zones,
above the cone, below it, and between, where it is quadratic in the
distance to the cone's surface; Newton's method takes the cone's own
Hessian there, and projected Gauss–Seidel solves each cone's friction as
one block by a quadratically constrained quadratic program. Each step
starts from the last step's accelerations where they cost less, as
MuJoCo's warmstart does. The CG solver is refused.

`Solve` runs after `Collide`, once a step, on the whole world, in its
`prepare`: it gathers each tree's mass matrix and smooth accelerations,
joins the trees that contacts and tendons touch into islands by union and
find, and
solves each island on its own, densely, since an island's rows and dofs are
few; each tree learns its island. `Integrate` then steps each tree at its
island's accelerations, or, where any dof of the model is damped, at its
smooth and constraint forces through M + h B, as MuJoCo does.

`constraint_test` checks twenty-one cases against MuJoCo, every position and
velocity at every step (`reference/mujoco_constraints.py`):

| Case | Steps | Newton, position | Newton, velocity | PGS, position | PGS, velocity |
|---|---:|---:|---:|---:|---:|
| A sphere dropped, sliding, then rolling | 1,500 | 9.1e-13 m | 6.4e-12 m/s | 5.4e-15 | 7.5e-14 |
| A box sliding to rest; a capsule rolling with torsional and rolling friction | 1,000 | 1.7e-14 | 9.8e-14 | 3.9e-7 | 1.1e-5 |
| Five boxes stacked at rest | 1,000 (PGS 200) | 4.9e-16 | 3.8e-14 | 5.4e-9 | 6.9e-7 |
| Hinge, ball and slide limits, a margin, dry friction | 1,500 | 6.7e-15 | 4.8e-14 | 6.7e-15 | 5.7e-14 |
| MuJoCo's humanoid falling from standing | 400 | 1.3e-13 | 1.1e-11 | 6.4e-14 | 5.2e-12 |
| The humanoid falling, its 21 motors held at controls | 400 | 2.8e-13 | 7.6e-12 | | |
| The sphere, elliptic cone | 1,500 | 3.1e-14 | 1.5e-13 | 5.8e-15 | 2.8e-14 |
| The box and capsule, elliptic cone | 1,000 | 1.1e-7 | 6.6e-6 | 2.5e-7 | 1.1e-5 |
| The stack, elliptic cone | 1,000 | 0 | 3.8e-17 | | |
| The driven humanoid, elliptic cone | 400 | 3.6e-13 | 2.0e-11 | | |
| The actuated arm, implicitfast | 1,500 | equal | equal | | |
| The tumbling free body, implicitfast | 1,000 | 7.8e-16 | 8.9e-16 | | |
| The sphere, implicitfast | 1,500 | 1.7e-14 | 9.2e-14 | | |
| The driven humanoid, implicitfast | 400 | 2.8e-13 | 7.6e-12 | | |

Newton's method converges, and so matches MuJoCo to rounding grown over
the run, but where a body comes to stick, the tolerance it stops at can
tip it a step apart from MuJoCo's: the capsule, on the elliptic cone. Projected Gauss–Seidel stops at its tolerance; where a sweep ends
a step earlier or later than MuJoCo's, by rounding, the two part by its
tolerance, and a stack's resting contacts then come and go apart, so the
stack is compared over 0.4 s.

It also checks physics, by both solvers: the box slides 0.5078 m from 2 m/s
where Coulomb friction of 0.4 gives 0.5097 m; a sphere sliding at 2 m/s on
the floor rolls at 1.4279 m/s, within 0.05% of five sevenths of 2 m/s, its
spin matching; the stack rests within 2.1 mm of its height, still to 1e-12
m/s by Newton's method, and within 5 mm/s by PGS, as MuJoCo's own PGS
leaves it. MuJoCo's humanoid, falling for 20 s, lies on the floor 0.070 m
high, as MuJoCo's does, its tendons and limits holding.

## Scale

`robotic_benchmark` times scenes `reference/make_scenes.py` writes, and
`reference/mujoco_benchmark.py` times MuJoCo 3.14.0 on the same files, both
on one thread, `-c opt`, 200 steps of 2 ms (5 ms for the humanoids) from
rest, the bodies landing partway through:

| Scene | simon | MuJoCo | Ratio |
|---|---:|---:|---:|
| 1,000 loose boxes, spheres and capsules falling onto the floor | 1.45 ms/step | 1.36 | 1.07 |
| 10,000 of them | 17.8 | 15.1 | 1.18 |
| 100 humanoids falling, 2 m apart | 2.49 | 2.21 | 1.13 |
| 1,000 humanoids | 31.4 | 26.2 | 1.20 |

Both end with the same contacts, 19,111 for the 10,000 bodies and 8,000 for
the humanoids, and take about as many solver iterations an island, 1.1 to
1.2. MuJoCo is faster by its sparse mass matrix and Jacobian; simon keeps
each tree's mass matrix dense at its archetype's capacity, and each
island's Hessian dense, so a large island, a pile of hundreds of bodies
touching, would cost the cube of its size. Every island is independent, so
they could be solved in parallel; the framework runs a system's `prepare`
on one thread, so they are solved one after another.

## Viewer

`viewer` watches a model in real time, MuJoCo's humanoid unless it is given
another: each geom in perspective, boxes and cylinders by their faces lit
from above, capsules and spheres outlined, planes as grids, the contacts as
dots; the view turns, pans and zooms with the mouse and follows the trees.
Its panel shows the trees, contacts, islands, rows and solver iterations
of the step, and pauses, restarts and speeds the run.

## Roadmap

1. Done: MJCF read and compiled, against MuJoCo's compiled model.
2. Done: dynamics without contact, springs, dampers and motors: forward
   kinematics, the mass matrix, bias forces, LDLᵀ and Euler; a pendulum, a
   double pendulum, a tumbling free body, springs and dampers, a cart-pole
   and falling boxes, each step against MuJoCo. Joint limits move to step
   5, since MuJoCo solves them with contacts.
3. Done: position, velocity and general actuators, actuator damping, and
   a linear state feedback balancing a cart-pole, against MuJoCo.
4. Done: collision of primitives and the broad phase, against MuJoCo's
   contacts.
5. Done: the constraint solver: soft contacts, joint limits and dry
   friction, Newton and PGS, by island; a sphere rolling, a box sliding, a
   stack, against MuJoCo and physics.
6. Done: MuJoCo's humanoid, with its fixed tendons and contact exclusions,
   compiled to the last bit and stepped within 1e-11 of MuJoCo through its
   fall. The Apache-2.0 models of MuJoCo Menagerie move to step 8: each uses
   the elliptic cone, implicitfast, cylinders that only the convex collider
   handles, or meshes.
7. Done: ten thousand loose bodies and a thousand humanoids within 1.2x of
   MuJoCo on one thread, and the viewer. Islands in parallel wait on the
   framework running `prepare` on more than one thread.
8. Opt-in fidelity: implicitfast and Runge–Kutta 4, the elliptic cone, convex
   meshes and the pairs MuJoCo's convex collider handles; then MuJoCo
   Menagerie's robots.
