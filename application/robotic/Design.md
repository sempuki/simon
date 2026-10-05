# robotic design

Articulated rigid bodies on joints, with contact, friction and actuators:
robot arms, legged robots, humanoids and piles of loose objects. It proves
the framework can carry the first kind of system it has not yet carried, a
solve that couples entities: contacts tie bodies together, and each step
solves them at once. MuJoCo (Todorov, Erez and Tassa, "MuJoCo: A physics
engine for model-based control", IROS 2012; Apache-2.0) is the reference.
It is being built in eight steps (see the [Roadmap](#roadmap)); models read
and compiled as MuJoCo compiles them, their dynamics without constraints,
their actuators and control, and their contacts are done.

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
tendons, equalities, meshes, contact pairs and exclusions, and actuators
other than motors, position and velocity servos and general actuators with
fixed gains. What only shows a model is left out. A body's inertial frame
and a geom's frame within 1e-6 of the body's frame, or of the inertial
frame, are snapped to it, as MuJoCo snaps them; poses in the world then
come from the same arithmetic.

`model_test` compiles eight test models with every compiler feature (a
pendulum, a double pendulum, a cart-pole, a tumbling free body of three
offset geoms, a model of default classes and every orientation, a stack of
boxes, an actuated arm, and a model of every primitive pair) and checks all
3,954 compiled values of 2,276 fields against MuJoCo's
(`reference/mujoco_models.py`): every one is equal, to the last bit.
MuJoCo's humanoid is refused, for its contact exclusions and tendons, until
step 6.

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
velocity; where a dof is damped or actuated, the step solves M + h B, as
MuJoCo's does, so that dampers stay stable at any step. MuJoCo also keeps a
fixed inertia for dofs whose bodies never turn; simon recomputes it, which
can differ in the last bit.

In the ECS, an entity is a tree, of one of two archetypes by its capacity:
a small tree, at most 4 bodies and 8 degrees of freedom, and a large tree,
at most 16 and 32. Each archetype's state and work are sized at compile
time, inline in its components. `Forward` computes each tree's poses, its
factored mass matrix and its accelerations into its `TreeDynamics`;
`Bound` its sphere, the world's spatial component; and `Integrate` steps
its `TreeState`. `Collide` finds the contacts between them, and the
constraint solver will come after it.

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
5. The constraint solver: soft contacts and joint limits, friction, Newton
   and PGS, by island; a sphere rolling, a box sliding, a stack.
6. Whole robots: MuJoCo's humanoid, with its tendons and contact
   exclusions, and Apache-2.0 models from MuJoCo Menagerie.
7. Scale and the viewer: ten thousand loose bodies and a thousand
   humanoids against MuJoCo on one thread, islands in parallel.
8. Opt-in fidelity: implicitfast and Runge–Kutta 4, the elliptic cone, convex
   meshes.
