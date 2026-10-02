# Copyright 2022 -- CONTRIBUTORS. See LICENSE.

"""Flies JSBSim's 737 as the reference for the flight model's accuracy.

Writes jsbsim_737.csv, which accuracy_test replays through Fly and Precise, and
prints the airframe fitted to the 737, which accuracy_test holds as constants.

The 737 starts trimmed at 6000 m and 200 m/s, heading north from the equator,
and a small autopilot flies it through level cruise, a 30 degree turn, a climb,
a descending turn and an acceleration. Every 0.2 s the table records where it
is and the controls a point mass needs to fly the same path:

  load factor, bank  from the rates of flight-path angle and heading it flew,
                     so the table takes a flat, non-rotating earth's view of
                     the path and earth effects do not count against simon;
  throttle           from the thrust along the velocity, per kilogram of the
                     fitted mass, so fuel burn does not count either.

JSBSim (https://github.com/JSBSim-Team/jsbsim, LGPL 2.1 or later) does the
flying; its 737 model is by Dave Culp and Aeromatic, under the GPL, and is
meant for educational and entertainment purposes only.

Drag is left to simon's drag polar, fitted to JSBSim's level and turning trims
over the scenario's envelope. Run with JSBSim's Python package and NumPy:

  pip install jsbsim numpy
  python application/flight/reference/jsbsim_737.py
"""

import math
import os

import jsbsim
import numpy as np

FT = 0.3048  # m.
LBF = 4.4482216152605  # N.
SLUG = 14.593902937  # kg.
G0 = 9.80665  # m/s^2.

# The sea-level thrust at full throttle. It only scales the throttle.
THRUST = 200000.0  # N.

# Maneuvers: (from time s, flight-path angle deg, bank deg, speed m/s).
PLAN = [(0, 0, 0, 200), (60, 0, 30, 200), (200, 0, 0, 200), (260, 3, 0, 200),
        (380, 0, 0, 200), (440, -2, -25, 200), (560, 0, 0, 220)]
END = 640.0
SAMPLE = 0.2  # s.


def standard_density(altitude):
    """The International Standard Atmosphere's density, in the troposphere,
    at a geometric altitude."""
    altitude = 6356766.0 * altitude / (6356766.0 + altitude)  # Geopotential.
    temperature = 288.15 - 0.0065 * altitude
    pressure = 101325.0 * (temperature / 288.15) ** (G0 / (0.0065 * 287.052874))
    return pressure / (287.052874 * temperature)


def trimmed(altitude, speed, bank=0.0):
    """The 737 trimmed in level flight, or in a level turn at `bank` deg."""
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    fdm.load_model('737')
    fdm.set_dt(1.0 / 120.0)
    fdm['ic/h-sl-ft'] = altitude / FT
    fdm['ic/vt-fps'] = speed / FT
    fdm['ic/gamma-deg'] = 0.0
    fdm['ic/lat-gc-deg'] = 0.0
    fdm['ic/long-gc-deg'] = 0.0
    fdm['ic/psi-true-deg'] = 0.0
    fdm['ic/phi-deg'] = 0.0
    fdm['ic/beta-deg'] = 0.0
    fdm['gear/gear-cmd-norm'] = 0
    fdm['fcs/flap-cmd-norm'] = 0
    fdm.run_ic()
    fdm['propulsion/set-running'] = -1
    for _ in range(10):
        fdm.run()
    fdm.do_trim(1)
    if bank:
        fdm['ic/phi-deg'] = bank
        fdm.run_ic()
        fdm.do_trim(5)
    return fdm


def fit_drag_polar():
    """CD0 and K of CD = CD0 + K CL^2, over level and turning trims."""
    lift, drag = [], []
    for altitude in [3000, 4500, 6000, 7500, 9000]:
        for speed in [160, 180, 200, 220, 240]:
            for bank in [0, 15, 30, 45]:
                try:
                    fdm = trimmed(altitude, speed, bank)
                except RuntimeError:
                    continue
                area = abs(fdm['aero/qbar-area'])
                lift.append(abs(fdm['forces/fwz-aero-lbs']) / area)
                drag.append(abs(fdm['forces/fwx-aero-lbs']) / area)
    lift, drag = np.array(lift), np.array(drag)
    terms = np.c_[np.ones_like(lift), lift**2]
    (zero_lift, induced), *_ = np.linalg.lstsq(terms, drag, rcond=None)
    worst = np.abs(terms @ [zero_lift, induced] / drag - 1.0).max()
    return zero_lift, induced, len(lift), worst


def command(time):
    current = PLAN[0]
    for maneuver in PLAN:
        if time >= maneuver[0]:
            current = maneuver
    return current


def clamp(value, low, high):
    return max(low, min(high, value))


def fly():
    """Flies the plan, recording every frame."""
    keys = ['velocities/v-east-fps', 'velocities/v-north-fps',
            'velocities/v-down-fps', 'position/h-sl-meters',
            'inertia/mass-slugs', 'attitude/phi-rad', 'attitude/theta-rad',
            'attitude/psi-rad', 'forces/fbx-prop-lbs', 'forces/fby-prop-lbs',
            'forces/fbz-prop-lbs']
    fdm = trimmed(6000.0, 200.0)
    dt = fdm.get_delta_t()
    throttle_trim = fdm['fcs/throttle-cmd-norm[0]']
    elevator_trim = fdm['fcs/elevator-cmd-norm']
    climb_integral = speed_integral = 0.0
    start = fdm.get_sim_time()  # Trimming ran the clock.
    frames = [[0.0] + [fdm[key] for key in keys]]
    while fdm.get_sim_time() - start < END - dt / 2:
        _, climb, bank, speed = command(fdm.get_sim_time() - start)
        climb, bank = math.radians(climb), math.radians(bank)
        phi = fdm['attitude/phi-rad']
        gamma = fdm['flight-path/gamma-rad']
        v = fdm['velocities/vt-fps'] * FT

        # Roll toward the bank at no more than 0.08 rad/s.
        roll_rate = clamp(0.5 * (bank - phi), -0.08, 0.08)
        fdm['fcs/aileron-cmd-norm'] = clamp(
            2.0 * (roll_rate - fdm['velocities/p-rad_sec']), -1, 1)

        # Pitch toward the flight-path angle, plus the pull a turn needs.
        turn_pitch_rate = G0 / v * math.sin(phi) * math.tan(phi)
        pitch_rate = clamp(0.3 * (climb - gamma) + turn_pitch_rate, -0.05, 0.05)
        climb_integral += (climb - gamma) * dt
        fdm['fcs/elevator-cmd-norm'] = clamp(
            elevator_trim - 4.0 * (pitch_rate - fdm['velocities/q-rad_sec']) -
            climb_integral, -1, 1)

        # No sideslip, and the yaw rate of a coordinated turn.
        fdm['fcs/rudder-cmd-norm'] = clamp(
            -4.0 * fdm['aero/beta-rad'] +
            2.0 * (fdm['velocities/r-rad_sec'] - G0 / v * math.sin(phi)), -1, 1)

        # Speed by throttle.
        speed_integral += (speed - v) * dt
        throttle = clamp(throttle_trim + 0.05 * (speed - v) +
                         0.005 * speed_integral, 0, 1)
        fdm['fcs/throttle-cmd-norm[0]'] = throttle
        fdm['fcs/throttle-cmd-norm[1]'] = throttle

        fdm.run()
        frames.append([fdm.get_sim_time() - start] + [fdm[key] for key in keys])
    return np.array(frames), dt


def body_to_local(force, phi, theta, psi):
    """Body-axis forces in east, north and up."""
    cf, sf = np.cos(phi), np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cp, sp = np.cos(psi), np.sin(psi)
    x, y, z = force.T
    north = cp * ct * x + (cp * st * sf - sp * cf) * y + (cp * st * cf + sp * sf) * z
    east = sp * ct * x + (sp * st * sf + cp * cf) * y + (sp * st * cf - cp * sf) * z
    down = -st * x + ct * sf * y + ct * cf * z
    return np.c_[east, north, -down]


def main():
    zero_lift, induced, trims, worst = fit_drag_polar()
    frames, dt = fly()
    time = frames[:, 0]
    velocity = frames[:, 1:4] * FT * [1, 1, -1]  # East, north, up.
    altitude = frames[:, 4]
    mass = frames[:, 5] * SLUG
    phi, theta, psi = frames[:, 6], frames[:, 7], frames[:, 8]
    thrust = body_to_local(frames[:, 9:12] * LBF, phi, theta, psi)

    # The path in a flat local frame: velocity integrated over the ground.
    position = np.zeros_like(velocity)
    position[1:] = np.cumsum(
        0.5 * (velocity[1:] + velocity[:-1]) * np.diff(time)[:, None], axis=0)
    position[:, 2] = altitude
    speed = np.linalg.norm(velocity, axis=1)
    gamma = np.arcsin(velocity[:, 2] / speed)
    heading = np.unwrap(np.arctan2(velocity[:, 0], velocity[:, 1]))

    # The load factor and bank that turn the point mass as the 737 turned.
    lift_up = np.gradient(gamma, time) * speed / G0 + np.cos(gamma)
    lift_side = np.gradient(heading, time) * speed * np.cos(gamma) / G0
    load_factor = np.hypot(lift_up, lift_side)
    bank = np.arctan2(lift_side, lift_up)

    # Thrust along the velocity, per kilogram of the starting mass.
    along = (thrust * velocity).sum(axis=1) / speed
    throttle = along * (mass[0] / mass) / (
        THRUST * standard_density(altitude) / 1.225)

    every = int(round(SAMPLE / dt))
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'jsbsim_737.csv')
    with open(path, 'w') as table:
        table.write('time,load_factor,bank,throttle,x,y,z,speed,'
                    'flight_path_angle,heading\n')
        for i in range(0, len(time), every):
            table.write('%.1f,%.5f,%.5f,%.5f,%.1f,%.1f,%.1f,%.3f,%.5f,%.5f\n' % (
                time[i], load_factor[i], bank[i], throttle[i], *position[i],
                speed[i], gamma[i], heading[i]))

    print('wrote', path)
    print('mass %.1f kg, wing area %.3f m^2, thrust %.0f N' %
          (mass[0], 1171.0 * FT**2, THRUST))
    print('drag polar over %d trims: CD0 %.5f K %.4f, worst error %.1f%%' % (trims, zero_lift, induced, 100 * worst))
    print('fuel burned: %.1f%% of the mass' % (100 * (1 - mass[-1] / mass[0])))


if __name__ == '__main__':
    main()
