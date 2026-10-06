# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Times REBOUND's gravity on one thread, for comparison with
galactic_benchmark: the same three cases, built the same way, stepped by
REBOUND's leapfrog, which computes the accelerations once a step as simon's
does. The pip wheel of REBOUND is built without OpenMP, so it runs on one
thread.

  python application/galactic/reference/rebound_benchmark.py \\
      direct|tree|restricted N [steps]

REBOUND 5.2.2:

  pip install rebound numpy
"""

import math
import sys
import time

import numpy
import rebound

G = 6.67430e-11  # CODATA 2018.
PARSEC = 149597870700.0 * (648000.0 / math.pi)  # In simon's order.
KILOPARSEC = 1000.0 * PARSEC
SOLAR_MASS = 1.3271244e20 / G
YEAR = 365.25 * 86400.0


def sample_direction(random, length):
    z = (1.0 - 2.0 * random.random()) * length
    across = math.sqrt(length * length - z * z)
    angle = 2.0 * math.pi * random.random()
    return across * math.cos(angle), across * math.sin(angle), z


def add_plummer(simulation, bodies):
    """Positions as galactic_benchmark samples them; velocities do not
    change the cost of a step, so the bodies start at rest."""
    random = numpy.random.default_rng(1)
    mass = 1e10 * SOLAR_MASS
    for _ in range(bodies):
        r = 1.0 / math.sqrt(random.random() ** (-2.0 / 3.0) - 1.0)
        x, y, z = sample_direction(random, r * KILOPARSEC)
        simulation.add(m=mass / bodies, x=x, y=y, z=z)


def add_restricted(simulation, particles):
    mass = 1e11 * SOLAR_MASS
    simulation.add(m=mass)
    simulation.add(m=mass, x=50 * KILOPARSEC)
    rings = 100
    for ring in range(rings):
        r = (2.0 + 18.0 * ring / (rings - 1)) * KILOPARSEC
        speed = math.sqrt(G * mass / r)
        count = particles // rings
        for i in range(count):
            angle = 2.0 * math.pi * i / count
            simulation.add(m=0.0, x=r * math.cos(angle), y=r * math.sin(angle),
                           vx=-speed * math.sin(angle),
                           vy=speed * math.cos(angle))
    simulation.N_active = 2
    simulation.testparticle_type = 0


def main():
    kind = sys.argv[1]
    count = int(sys.argv[2])
    steps = int(sys.argv[3]) if len(sys.argv) > 3 else 10

    simulation = rebound.Simulation()
    simulation.G = G
    simulation.integrator = 'leapfrog'
    simulation.dt = 10000 * YEAR
    if kind == 'tree':
        simulation.root_size = 2000 * KILOPARSEC
        simulation.boundary = 'open'
        simulation.gravity = 'tree'
        simulation.opening_angle2 = 0.25
        simulation.softening = 0.01 * KILOPARSEC
        add_plummer(simulation, count)
    elif kind == 'direct':
        simulation.gravity = 'basic'
        simulation.softening = 0.01 * KILOPARSEC
        add_plummer(simulation, count)
    else:
        simulation.gravity = 'basic'
        simulation.softening = 0.1 * KILOPARSEC
        add_restricted(simulation, count)
    bodies = simulation.N

    simulation.step()
    start = time.perf_counter()
    simulation.steps(steps)
    seconds = time.perf_counter() - start
    print(f'{kind} {bodies}: {seconds / steps * 1e3:.3f} ms/step, '
          f'{seconds / steps / bodies * 1e9:.0f} ns/body-step')


if __name__ == '__main__':
    main()
