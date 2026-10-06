# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Steps Toomre and Toomre's flat direct encounter in REBOUND, for
encounter_test.

Two masses of 10^11 suns pass on a parabolic orbit within 25 kpc; the first
carries Toomre and Toomre's disk of 120 test particles in rings of 12, 18,
24, 30 and 36 at 0.2 to 0.6 of the pericenter distance, turning the same way
as the orbit (A. Toomre and J. Toomre, ApJ 178, 623, 1972, section II and
figure 2). REBOUND (https://rebound.readthedocs.io, GPL-3.0) gives the
accelerations, its two masses active and the particles pulling on nothing,
and this script kicks and drifts them, kick-drift-kick as simon does, from a
billion years before pericenter to a billion after, in steps of 250,000
years. It writes rebound_encounter.csv, in SI units, to 17 significant
digits: every body every 400 steps, the masses first; step 0 is the start.

REBOUND 5.2.2:

  pip install rebound
  python application/galactic/reference/rebound_encounter.py
"""

import csv
import ctypes
import math
import os

import rebound

HERE = os.path.dirname(os.path.abspath(__file__))

G = 6.67430e-11  # CODATA 2018.
PARSEC = 149597870700.0 * (648000.0 / math.pi)  # In simon's order.
KILOPARSEC = 1000.0 * PARSEC
SOLAR_MASS = 1.3271244e20 / G
YEAR = 365.25 * 86400.0

MASS = 1e11 * SOLAR_MASS
PERICENTER = 25.0 * KILOPARSEC
BEFORE = 1e9 * YEAR
SOFTENING = 0.1 * KILOPARSEC
STEP = 250000 * YEAR
STEPS = 8000


def parabolic_separation(time):
    """Barker's equation in closed form."""
    mu = G * (MASS + MASS)
    q = PERICENTER
    rate = math.sqrt(mu / (2.0 * q * q * q))
    a = 1.5 * rate * time
    b = math.cbrt(a + math.sqrt(a * a + 1.0))
    d = b - 1.0 / b
    d_rate = rate / (1.0 + d * d)
    return ((q * (1.0 - d * d), 2.0 * q * d, 0.0),
            (-2.0 * q * d * d_rate, 2.0 * q * d_rate, 0.0))


def make_bodies():
    (x, y, z), (vx, vy, vz) = parabolic_separation(-BEFORE)
    victim = (-0.5 * x, -0.5 * y, -0.5 * z, -0.5 * vx, -0.5 * vy, -0.5 * vz)
    companion = (0.5 * x, 0.5 * y, 0.5 * z, 0.5 * vx, 0.5 * vy, 0.5 * vz)
    particles = []
    gm = G * MASS
    for ring in range(5):
        r = (0.2 + 0.1 * ring) * PERICENTER
        count = 12 + 6 * ring
        speed = math.sqrt(gm * r * r / (r * r + SOFTENING * SOFTENING) ** 1.5)
        for i in range(count):
            angle = 2.0 * math.pi * i / count
            c, s = math.cos(angle), math.sin(angle)
            particles.append((victim[0] + r * c, victim[1] + r * s, victim[2],
                              victim[3] - speed * s, victim[4] + speed * c,
                              victim[5]))
    return victim, companion, particles


def main():
    victim, companion, particles = make_bodies()
    simulation = rebound.Simulation()
    simulation.G = G
    simulation.gravity = 'basic'
    simulation.softening = SOFTENING
    for x, y, z, vx, vy, vz in (victim, companion):
        simulation.add(m=MASS, x=x, y=y, z=z, vx=vx, vy=vy, vz=vz)
    for x, y, z, vx, vy, vz in particles:
        simulation.add(m=0.0, x=x, y=y, z=z, vx=vx, vy=vy, vz=vz)
    simulation.N_active = 2
    simulation.testparticle_type = 0

    def update():
        rebound.clibrebound.reb_simulation_update_acceleration(
            ctypes.byref(simulation))

    with open(os.path.join(HERE, 'rebound_encounter.csv'), 'w',
              newline='') as file:
        out = csv.writer(file, lineterminator='\n')
        out.writerow(['step', 'body', 'x', 'y', 'z', 'vx', 'vy', 'vz'])
        update()
        for step in range(STEPS + 1):
            if step % 400 == 0:
                for i, p in enumerate(simulation.particles):
                    out.writerow([step, i] + [repr(v) for v in (
                        p.x, p.y, p.z, p.vx, p.vy, p.vz)])
            if step == STEPS:
                break
            for p in simulation.particles:
                p.vx += p.ax * (0.5 * STEP)
                p.vy += p.ay * (0.5 * STEP)
                p.vz += p.az * (0.5 * STEP)
            for p in simulation.particles:
                p.x += p.vx * STEP
                p.y += p.vy * STEP
                p.z += p.vz * STEP
            update()
            for p in simulation.particles:
                p.vx += p.ax * (0.5 * STEP)
                p.vy += p.ay * (0.5 * STEP)
                p.vz += p.az * (0.5 * STEP)


if __name__ == '__main__':
    main()
