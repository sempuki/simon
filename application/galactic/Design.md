# galactic design

Galaxies as bodies under each other's gravity, up to two disk galaxies
colliding and merging. It proves the framework can carry a force that every
entity exerts on every other, at any distance: the other applications'
entities meet only their neighbors or the bodies of their own tree.
[REBOUND](https://rebound.readthedocs.io) (Rein and Liu, *Astronomy &
Astrophysics* 537, A128, 2012; GPL-3.0) is the reference. It was built in
seven steps (see the [Roadmap](#roadmap)), all done: direct gravity and the leapfrog,
Plummer's sphere, Barnes and Hut's tree, Toomre and Toomre's restricted
encounters, disk galaxies, the collision with its runner and viewer, and
scale against REBOUND.

## Choices

Each choice says what it is, why, and where it comes from.

1. **A body stands for many stars.** A body is an entity with a position,
   velocity, mass and the acceleration gravity gives it. A galaxy of 10^11
   stars is sampled by 10^4 to 10^6 bodies, which sample its mass
   distribution, as every collisionless galaxy code does (Hernquist 1993).
   Close encounters between bodies are an artifact of that sampling, so
   gravity is softened.
2. **Plummer softening.** Gravity between bodies is G m d / (|d|^2 +
   e^2)^(3/2), REBOUND's softening, so the two can be compared exactly.
   Dehnen (*MNRAS* 324, 2001) shows that kernels of finite extent err less;
   they are an opt-in for later.
3. **Three fidelities of gravity, chosen per archetype and per run.** A
   test particle (an archetype without a mass) feels gravity and exerts
   none, as Toomre and Toomre's stars do, and costs one pass over the
   bodies that pull. Bodies with mass get gravity by direct summation,
   exact to rounding at N^2, or by Barnes and Hut's tree at N log N. A run
   chooses direct or tree; both are in the schedule, and the one not chosen
   skips its loop in `prepare`, so it costs nothing.
4. **The tree is built once a step and walked by each body.** The tree
   system gathers the bodies that pull and builds an octree in `prepare`;
   each body then walks it on its own and writes only its own `Gravity`.
   That is the framework's propose pattern: shared read-only data made
   once, then a per-entity pass in any order. Its forces are not exactly
   equal and opposite, so momentum drifts by the tree's error, which the
   tests measure. Dehnen's fast multipole method keeps momentum exact and is
   the natural opt-in after it.
5. **The leapfrog in kick-drift-kick form, one step for every body.** It is
   symplectic and time-reversible, so orbits' energy oscillates without
   drifting (Hairer, Lubich and Wanner), and it takes one force evaluation
   a step. GADGET-2 (Springel, *MNRAS* 364, 2005) steps galaxies the same
   way. REBOUND's own leapfrog is drift-kick-drift, so the reference scripts
   kick and drift themselves, taking REBOUND's accelerations. Block steps,
   shorter for bodies deep in a galaxy, are for later.
6. **Time counts whole Julian years.** simon's default tick, int64
   nanoseconds, covers 292 years; galaxies need billions. A simulation now
   declares its tick (`using Tick = Year;`), and its drivers and steps count
   in it, exactly, while physics converts `step.dt` to seconds as before
   (see "Time" in [framework/Design.md](../../framework/Design.md)).
7. **SI units under the hood, astronomical at the edges.** Galaxies fit in
   doubles: a kiloparsec is 3 x 10^19 m and a galaxy 10^41 kg. The
   constants are CODATA 2018's G, the IAU's astronomical unit and parsec,
   and the IAU's nominal solar mass parameter over G. Galaxy codes set G = 1
   instead; simon keeps its one convention.
8. **Initial conditions from their papers.** Plummer's sphere is sampled by
   Aarseth, Hénon and Wielen's appendix (1974), and disk galaxies by
   Hernquist's (1993) moments of the collisionless Boltzmann equation, with
   two changes: a Hernquist (1990) halo, whose mass and potential are in
   closed form, instead of his isothermal one, and no bulge. The disk and
   the halo are each centered on their own: the halo outweighs the disk
   tenfold, and its sampling noise moves its center of mass by kiloparsecs.

## Gravity

`model/gravity/gravity` gathers the bodies that pull, those with a `PointMass`, into
a list once a step. `SumGravity` sums every one's pull on each body in the
list's order, with REBOUND's operations in REBOUND's order, so the two agree
to the last bit: REBOUND's loop over pairs happens to add each body's terms
in the same order. `TreeGravity` builds its octree over the same list: a
cube around the bodies, split into octants, stably, until each cell holds
one body; each cell holds its mass and center of mass. A body opens a cell
whose width is more than the opening angle times its distance to the cell's
center of mass, REBOUND's test, and takes an unopened cell as a point there.

At an opening angle of 0.5, on 2,000 bodies of a Plummer sphere, the tree's
median force error is 0.29% and its 99th percentile 1.6%, against REBOUND's
0.25% and 1.7%; REBOUND's root is a cube about the origin, simon's bounds the
bodies, so their cells differ. Opened all the way it equals direct summation
to 4 x 10^-15.

## Initial conditions

Plummer's sphere: 4,000 bodies fall within 5% of its mass profile at five
radii and within 5% of virial equilibrium, and 256 of them keep their
energy to 1.5 x 10^-4 and their half-mass radius within 2.5% for ten
crossing times.

Disk galaxies: Hernquist's recipe sets the halo's dispersion by the Jeans
equation in the halo's and disk's mass, with Gaussian speeds below 0.95 of
escape, and the disk's by its surface density (σ_z^2 = π G Σ z_0), Toomre's
Q at a reference radius (σ_R ∝ e^(-R/2h)), the epicyclic approximation for
σ_φ and asymmetric drift for the mean rotation. The disk's own pull is
Freeman's (1970) thin exponential disk. The standard galaxy is a disk of
5 x 10^10 suns, 3 kpc in scale and 0.6 kpc thick, in a halo of 5 x 10^11
suns with a scale of 10 kpc, cut off at 100 kpc. Its proportions follow
Hernquist's: a disk 0.2 of its scale length thick, in a halo that outweighs
it at every radius but its center. A disk half as thick, or one that
dominates its inner halo, starts swelling within 50 million years; this one
keeps its half-mass radius and thickness within 0.5% for 100 million years
at 10,000 bodies. At 20,000, over 500 million years, its half-mass radius
stays within 7% and its disk thickens by 16%, slowly, as Hernquist found his
disks inflate by discreteness.

## Encounters

Toomre and Toomre's restricted encounters (*ApJ* 178, 1972) place a mass
with their disk of 120 test particles, in rings of 12 to 36 at 0.2 to 0.6
of the pericenter distance, and a bare companion on a parabolic orbit,
from Barker's equation in closed form. Their flat direct passage matches
REBOUND bit for bit for a billion years either side of pericenter. At
converged steps, of 15,625 years and finer, the companion captures 27 of
the particles, to their 28; their rings' phase is not given, and turning
every ring half a spacing gives 28. Captured particles plunge within a
kiloparsec of the companion on orbits of eccentricity 0.9, so one global
step must be short; Toomre and Toomre stepped each particle on its own.

## Collision

Two standard galaxies, taken as points of their whole mass for their
orbit, start 600 million years before passing within 15 kpc, the first face
on to the orbit and the second tilted 45 degrees, on the tree at an
opening angle of 0.6, softened by 0.24 kpc, in steps of a million years.
They pass within 15.3 kpc at 600 million years, swing out to 52 kpc at
900, fall back through each other at 1,200, and have merged, their centers
within 5 kpc, by 1,300. At the first apocenter a fifth of the disks is more
than 30 kpc from its galaxy's center, in tails and bridges. Energy drifts by
0.36% over 2 billion years in steps of a million years, and by 0.11% in
steps of half a million.

REBOUND's tree, run from the same start at the same opening angle and
softening by its own leapfrog, follows the same arc: within 0.5 kpc of
simon's on the way in, a pericenter of 14.9 kpc to simon's 15.3, an
apocenter of 50.2 kpc to 51.8, and the same merger by 1,300 million years.
After the second passage the two runs part, as chaotic runs do; simon's own
runs at a million and half a million years part as much from each other,
9.5 kpc apart against 13.5 at 1,200 million years.

## Scale

On one thread, against REBOUND's pip build, which has no OpenMP and so runs
on one thread too, each stepping the same three cases by the leapfrog with
one force evaluation a step:

| Case | simon | REBOUND | |
|---|---|---|---|
| Direct summation, 2,000 bodies | 12.7 ms a step | 9.6 ms | 1.3x slower |
| Direct summation, 10,000 bodies | 311 ms | 240 ms | 1.3x slower |
| Tree at an opening angle of 0.5, 10,000 bodies | 81.5 ms | 113 ms | 1.4x faster |
| Tree, 100,000 bodies | 1.37 s | 2.27 s | 1.7x faster |
| Two masses and 100,000 test particles | 1.12 ms | 1.12 ms | the same |
| Two masses and 1,000,000 test particles | 17.6 ms | 35.2 ms | 2.0x faster |

Direct summation is slower because each body writes only its own
acceleration, so each pair is computed twice, once for each body, where
REBOUND computes it once and applies it to both. Per pair, simon is 1.5x
faster. Writing only its own entity is what lets bodies run in any order and
in parallel. The tree and the test particles cost each body one walk or one
pass, and there simon is faster outright, more so as N grows.

## Viewer

`bazel run -c opt //application/galactic:viewer` shows the collision, a
galaxy alone, or Toomre and Toomre's encounter, face on or edge on, each
galaxy's disk in its own color and its halo faintly if asked, at a chosen
number of million years a second.

## Roadmap

1. Bodies, units, direct summation and the leapfrog, against REBOUND and
   Kepler. Done.
2. Plummer's sphere. Done.
3. Barnes and Hut's tree, against REBOUND's. Done.
4. Toomre and Toomre's restricted encounters, against REBOUND and their
   counts. Done.
5. Disk galaxies after Hernquist. Done.
6. The collision, its runner and viewer. Done.
7. Scale: 10^6 test particles and 10^5 bodies on the tree, against
   single-thread REBOUND. Done.

Later, each an opt-in: direct summation over each pair once, written to
both bodies through a resolve stage; threads; the fast multipole method, for exact momentum and
O(N); quadrupole moments in the tree's cells; block time steps; softening
by kernels of finite extent; bulges; and `Continuous`, `RateGate` and the
event queue counting in a simulation's own tick.
