# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Computes Barnes-Hut tree gravity in REBOUND, for tree_test.

A Plummer sphere of 2,000 bodies, sampled here by Aarseth, Henon and
Wielen's method, gets its gravity from REBOUND (https://rebound.readthedocs.io,
GPL-3.0) by direct summation and by REBOUND's tree at four opening angles.
This script writes rebound_tree.csv, in SI units, to 17 significant digits:
each body's position and mass, its direct acceleration, and its tree
acceleration at each opening angle.

REBOUND's tree opens a cell when its width exceeds the opening angle times
the distance to the cell's center of mass, and takes each unopened cell as a
point at its center of mass (it is built without quadrupoles). Its root is a
cube around the origin, so its cells differ from simon's, which bound the
bodies; the two are compared by their errors against direct summation.

REBOUND 5.2.2:

  pip install rebound numpy
  python application/galactic/reference/rebound_tree.py
"""

import csv
import ctypes
import math
import os

import numpy
import rebound

HERE = os.path.dirname(os.path.abspath(__file__))

G = 6.67430e-11  # CODATA 2018.
PARSEC = 149597870700.0 * (648000.0 / math.pi)  # In simon's order.
KILOPARSEC = 1000.0 * PARSEC
SOLAR_MASS = 1.3271244e20 / G

BODIES = 2000
MASS = 1e10 * SOLAR_MASS
SCALE = KILOPARSEC
SOFTENING = 0.01 * KILOPARSEC
ANGLES = [0.25, 0.5, 0.75, 1.0]


def sample_direction(random, length):
    z = (1.0 - 2.0 * random.random()) * length
    across = math.sqrt(length * length - z * z)
    angle = 2.0 * math.pi * random.random()
    return across * math.cos(angle), across * math.sin(angle), z


def sample_plummer(seed):
    random = numpy.random.default_rng(seed)
    bodies = []
    for _ in range(BODIES):
        r = 1.0 / math.sqrt(random.random() ** (-2.0 / 3.0) - 1.0)
        bodies.append(sample_direction(random, r * SCALE))
    return bodies


def make_simulation(bodies, gravity, angle, box):
    simulation = rebound.Simulation()
    simulation.G = G
    simulation.softening = SOFTENING
    if gravity == 'tree':
        simulation.root_size = box  # One root box, centered on the origin.
        simulation.boundary = 'open'
        simulation.opening_angle2 = angle * angle
    simulation.gravity = gravity
    for x, y, z in bodies:
        simulation.add(m=MASS / BODIES, x=x, y=y, z=z)
    rebound.clibrebound.reb_simulation_update_acceleration(
        ctypes.byref(simulation))
    return [(p.ax, p.ay, p.az) for p in simulation.particles]


def main():
    bodies = sample_plummer(11)
    box = 2.2 * max(max(abs(c) for c in body) for body in bodies)
    direct = make_simulation(bodies, 'basic', 0.0, box)
    trees = [make_simulation(bodies, 'tree', angle, box) for angle in ANGLES]

    with open(os.path.join(HERE, 'rebound_tree.csv'), 'w', newline='') as file:
        out = csv.writer(file, lineterminator='\n')
        header = ['x', 'y', 'z', 'm', 'softening', 'ax', 'ay', 'az']
        for angle in ANGLES:
            header += [f'tree_{angle:g}_{axis}' for axis in 'xyz']
        out.writerow(header)
        for i, (x, y, z) in enumerate(bodies):
            row = [repr(x), repr(y), repr(z), repr(MASS / BODIES),
                   repr(SOFTENING)] + [repr(a) for a in direct[i]]
            for tree in trees:
                row += [repr(a) for a in tree[i]]
            out.writerow(row)


if __name__ == '__main__':
    main()
