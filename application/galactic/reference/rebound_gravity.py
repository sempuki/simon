# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Computes softened gravity and leapfrog steps in REBOUND, for gravity_test.

REBOUND (https://rebound.readthedocs.io, GPL-3.0) computes, by direct
summation, the gravity on random clusters of bodies, and steps one cluster
by the leapfrog in its kick-drift-kick form, each kick taking REBOUND's
accelerations. This script writes, in SI units, to 17 significant digits:

  rebound_forces.csv    each case's bodies, their masses and accelerations
  rebound_leapfrog.csv  the stepped cluster's bodies every 10 steps; step 0
                        is the start

REBOUND's own leapfrog is drift-kick-drift; galaxy codes such as GADGET-2
use kick-drift-kick, as simon does, so the kicks and drifts are done here.

REBOUND 5.2.2:

  pip install rebound numpy
  python application/galactic/reference/rebound_gravity.py
"""

import csv
import ctypes
import math
import os

import numpy
import rebound

HERE = os.path.dirname(os.path.abspath(__file__))

G = 6.67430e-11  # CODATA 2018.
PARSEC = 149597870700.0 * (648000.0 / math.pi)
KILOPARSEC = 1000.0 * PARSEC
SOLAR_MASS = 1.3271244e20 / G
MEGAYEAR = 1e6 * 365.25 * 86400.0

# Name, bodies, softening, seed.
CASES = [
    ('cluster', 64, 0.1 * KILOPARSEC, 1),
    ('unsoftened', 16, 0.0, 2),
]
STEPPED = 'cluster'
STEP = 50000 * 365.25 * 86400.0  # 0.05 Myr, exactly as simon's years give it.
STEPS = 400


def make_simulation(bodies, softening, seed):
    random = numpy.random.default_rng(seed)
    simulation = rebound.Simulation()
    simulation.G = G
    simulation.gravity = 'basic'
    simulation.softening = softening
    for _ in range(bodies):
        x, y, z = random.normal(0.0, KILOPARSEC, 3)
        vx, vy, vz = random.normal(0.0, 100e3, 3)
        mass = random.uniform(1e8, 1e9) * SOLAR_MASS
        simulation.add(m=mass, x=x, y=y, z=z, vx=vx, vy=vy, vz=vz)
    return simulation


def update_acceleration(simulation):
    rebound.clibrebound.reb_simulation_update_acceleration(
        ctypes.byref(simulation))


def kick(simulation, dt):
    for p in simulation.particles:
        p.vx += p.ax * dt
        p.vy += p.ay * dt
        p.vz += p.az * dt


def drift(simulation, dt):
    for p in simulation.particles:
        p.x += p.vx * dt
        p.y += p.vy * dt
        p.z += p.vz * dt


def state(p):
    return [repr(v) for v in (p.x, p.y, p.z, p.vx, p.vy, p.vz)]


def main():
    with open(os.path.join(HERE, 'rebound_forces.csv'), 'w',
              newline='') as file:
        out = csv.writer(file, lineterminator='\n')
        out.writerow(['case', 'softening', 'body', 'x', 'y', 'z', 'vx', 'vy',
                      'vz', 'm', 'ax', 'ay', 'az'])
        for name, bodies, softening, seed in CASES:
            simulation = make_simulation(bodies, softening, seed)
            update_acceleration(simulation)
            for i, p in enumerate(simulation.particles):
                out.writerow([name, repr(softening), i] + state(p) +
                             [repr(p.m), repr(p.ax), repr(p.ay), repr(p.az)])

    with open(os.path.join(HERE, 'rebound_leapfrog.csv'), 'w',
              newline='') as file:
        out = csv.writer(file, lineterminator='\n')
        out.writerow(['step', 'body', 'x', 'y', 'z', 'vx', 'vy', 'vz'])
        _, bodies, softening, seed = next(
            case for case in CASES if case[0] == STEPPED)
        simulation = make_simulation(bodies, softening, seed)
        update_acceleration(simulation)
        for step in range(STEPS + 1):
            if step % 10 == 0:
                for i, p in enumerate(simulation.particles):
                    out.writerow([step, i] + state(p))
            kick(simulation, 0.5 * STEP)
            drift(simulation, STEP)
            update_acceleration(simulation)
            kick(simulation, 0.5 * STEP)


if __name__ == '__main__':
    main()
