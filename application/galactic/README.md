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

- **A Plummer sphere stays in equilibrium.** 4,000 bodies sampled by
  Aarseth, Hénon and Wielen's method fall within 5% of Plummer's mass
  profile at five radii and start within 5% of virial equilibrium. 256 of
  them, softened by 0.05 of the scale radius and run for ten crossing times
  at 128 steps each, keep their energy to 1.5 x 10^-4, their momentum and
  angular momentum to rounding, their virial ratio at 1.04 and their
  half-mass radius within 2.5%.
- **Barnes and Hut's tree errs as REBOUND's does.** On 2,000 bodies of a
  Plummer sphere at an opening angle of 0.5, its median force error is 0.29%
  and its 99th percentile 1.6%, to REBOUND's 0.25% and 1.7%, and so on at
  0.25, 0.75 and 1. With every cell opened it equals direct summation to
  4 x 10^-15. Its forces are not equal and opposite, so over ten crossing
  times momentum drifts by 9 x 10^-4 of the sphere's, and energy by
  9 x 10^-4, where direct summation keeps them to rounding and 1.5 x 10^-4.
- **Toomre and Toomre's encounter plays out as they found it.** Two masses
  of 10^11 suns pass within 25 kpc on a parabolic orbit, the first carrying
  their disk of 120 test particles, which feel gravity and exert none. Its
  outer ring turns in their 544.2 million years. Stepped by REBOUND's
  accelerations, the whole encounter, a billion years either side of
  pericenter, matches it bit for bit. The companion tears off a bridge and
  captures 27 of the particles, where they report 28; their rings' phase
  is not given, and turning every ring half a spacing gives 28.

```sh
bazel test //application/galactic/...
python application/galactic/reference/rebound_gravity.py   # regenerate the tables
```

## Credits

galactic measures itself against [REBOUND](https://rebound.readthedocs.io).
Thank you to Hanno Rein, Shangfei Liu and REBOUND's contributors for an
N-body code that is open, documented and easy to run beside another.

| Project | What galactic takes from it | License |
|---|---|---|
| [REBOUND](https://github.com/hannorein/rebound) 5.2.2 | The reference every check runs against, through its Python module. Its direct summation, which `model/gravity` follows in its order of operations (see `NOTICE.md`), and its tree's opening test, which simon's tree shares | GPL-3.0 |
| [GADGET-2](https://wwwmpa.mpa-garching.mpg.de/gadget/) | The leapfrog's kick-drift-kick form, from its paper; no code | |

The papers behind the physics are cited in
[model/REFERENCES.md](../../model/REFERENCES.md).
