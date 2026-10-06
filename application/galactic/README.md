# galactic

Bodies under each other's gravity, on the way to galaxies colliding. A body
stands for many stars or much dark matter: a galaxy of 10^11 stars is
sampled by 10^4 to 10^6 bodies, as galaxy codes do.

Gravity between bodies is softened by Plummer's kernel, and the leapfrog
steps it in kick-drift-kick form, as GADGET-2 does. Time counts whole Julian
years, so a run can last billions of years.

- **Gravity matches REBOUND to the last bit.** Summed directly over random
  clusters, softened and not, every body's acceleration equals REBOUND's
  exactly, because both add each pull in the same order with the same
  operations.
- **So does the leapfrog.** 400 steps of a 64-body cluster end where
  REBOUND's accelerations, kicked and drifted the same way, put them, bit for
  bit.
- **A Kepler orbit closes.** A body on a 10 kpc orbit of eccentricity 0.5
  around 10^11 solar masses returns to pericenter within 1.1 pc after one
  orbit of 4,000 steps, and the error falls fourfold when the steps double.

```sh
bazel test //application/galactic/...
python application/galactic/reference/rebound_gravity.py   # regenerate the tables
```
