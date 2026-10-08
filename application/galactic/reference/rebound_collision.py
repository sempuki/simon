# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Runs galactic's standard collision in REBOUND, for the arc of its merger.

Two disk galaxies, each 2,000 disk and 8,000 halo bodies, start from the
state simon's runner writes, and REBOUND (https://rebound.readthedocs.io,
GPL-3.0) runs them on its Barnes and Hut tree at simon's opening angle and
softening, by its own leapfrog, in steps of a million years, for 2 billion
years. Gravity is chaotic, so the two runs part body by body; what they
share is the merger's arc. This script writes rebound_collision.csv: every
10 million years, the distance between the galaxies' centers, each found as
simon's runner finds it, from the galaxy's disk.

REBOUND 5.2.2:

  pip install rebound numpy
  bazel run -c opt //application/galactic -- 2000 2000 \\
      --start=/tmp/collision_start.csv
  python application/galactic/reference/rebound_collision.py \\
      /tmp/collision_start.csv
"""

import csv
import math
import os
import sys

import numpy
import rebound

HERE = os.path.dirname(os.path.abspath(__file__))

G = 6.67430e-11  # CODATA 2018.
PARSEC = 149597870700.0 * (648000.0 / math.pi)  # In simon's order.
KILOPARSEC = 1000.0 * PARSEC
YEAR = 365.25 * 86400.0

SOFTENING = 0.24 * KILOPARSEC
OPENING_ANGLE = 0.6
STEP = 1e6 * YEAR
MILLION_YEARS = 2000


def find_center(positions, masses):
    """The center of mass, then three times that of the bodies within 10 kpc
    of it, as simon's compute_group_center finds it."""
    center = numpy.average(positions, axis=0, weights=masses)
    for _ in range(3):
        near = numpy.linalg.norm(positions - center, axis=1) <= 10 * KILOPARSEC
        if near.any():
            center = numpy.average(positions[near], axis=0,
                                   weights=masses[near])
    return center


def main(start):
    with open(start) as file:
        rows = list(csv.DictReader(file))
    groups = numpy.array([int(row['group']) for row in rows])
    masses = numpy.array([float(row['m']) for row in rows])

    simulation = rebound.Simulation()
    simulation.G = G
    simulation.root_size = 4000 * KILOPARSEC
    simulation.boundary = 'open'
    simulation.gravity = 'tree'
    simulation.opening_angle2 = OPENING_ANGLE ** 2
    simulation.softening = SOFTENING
    simulation.integrator = 'leapfrog'
    simulation.dt = STEP
    for row in rows:
        simulation.add(m=float(row['m']), x=float(row['x']),
                       y=float(row['y']), z=float(row['z']),
                       vx=float(row['vx']), vy=float(row['vy']),
                       vz=float(row['vz']))

    positions = numpy.zeros((len(rows), 3))
    with open(os.path.join(HERE, 'rebound_collision.csv'), 'w',
              newline='') as file:
        out = csv.writer(file, lineterminator='\n')
        out.writerow(['million_years', 'separation_kpc'])
        for myr in range(0, MILLION_YEARS + 1, 10):
            simulation.integrate(myr * 1e6 * YEAR)
            simulation.serialize_particle_data(xyz=positions)
            first = groups == 0
            second = groups == 2
            separation = numpy.linalg.norm(
                find_center(positions[first], masses[first]) -
                find_center(positions[second], masses[second]))
            out.writerow([myr, f'{separation / KILOPARSEC:.9g}'])


if __name__ == '__main__':
    main(sys.argv[1])
