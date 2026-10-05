# robotic design

Articulated rigid bodies on joints, with contact, friction and actuators:
robot arms, legged robots, humanoids and piles of loose objects. It proves
the framework can carry the first kind of system it has not yet carried, a
solve that couples entities: contacts tie bodies together, and each step
solves them at once. MuJoCo (Todorov, Erez and Tassa, "MuJoCo: A physics
engine for model-based control", IROS 2012; Apache-2.0) is the reference.
It is being built in eight steps (see the [Roadmap](#roadmap)); the first,
models read and compiled as MuJoCo compiles them, is done.

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
   cylinder, analytic where MuJoCo's are, the broad phase on simon's spatial
   index over bounding spheres; convex meshes by GJK and EPA later, opt in.
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
other than motors. What only shows a model is left out.

`model_test` compiles six test models with every compiler feature (a
pendulum, a double pendulum, a cart-pole, a tumbling free body of three
offset geoms, a model of default classes and every orientation, and a stack
of boxes) and checks all 1,114 compiled values against MuJoCo's
(`reference/mujoco_models.py`): every one is equal, to the last bit.
MuJoCo's humanoid is refused, for its contact exclusions and tendons, until
step 6.

## Roadmap

1. Done: MJCF read and compiled, against MuJoCo's compiled model.
2. Dynamics without contact: forward kinematics, the mass matrix, bias
   forces, LDLᵀ and Euler; a pendulum, a double pendulum and a free body
   spinning, each step against MuJoCo.
3. Limits, springs, damping and actuators: a cart-pole under control.
4. Collision: primitives and the broad phase, against MuJoCo's contacts.
5. The constraint solver: soft contacts, friction, Newton and PGS, by
   island; a sphere rolling, a box sliding, a stack.
6. Whole robots: MuJoCo's humanoid, with its tendons and contact
   exclusions, and Apache-2.0 models from MuJoCo Menagerie.
7. Scale and the viewer: ten thousand loose bodies and a thousand
   humanoids against MuJoCo on one thread, islands in parallel.
8. Opt-in fidelity: implicitfast and Runge–Kutta 4, the elliptic cone, convex
   meshes.
