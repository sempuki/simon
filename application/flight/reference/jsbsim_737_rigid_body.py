# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Records JSBSim's 737 equations of motion, for rigid_body_test.

Writes jsbsim_737_rigid_body.csv: at states through two maneuvering flights,
one at the equator heading north and one at 60 degrees north heading east,
the state in the Earth-centered inertial frame, the forces and moments that
act on it, its mass properties, the rates of change JSBSim found, and the
air data it read. All of
it is read after a frame: JSBSim starts each frame by moving the state on,
then finds the forces and rates of the state it moved to.

  pip install jsbsim numpy
  python application/flight/reference/jsbsim_737_rigid_body.py
"""

import math
import os

import jsbsim
import numpy as np

FT = 0.3048
LBF = 4.4482216152605
SLUG = LBF / FT  # kg: a slug is a pound-force second squared per foot.
SLUG_FT2 = SLUG * FT * FT

STATE = [
    ('time', 'simulation/sim-time-sec', 1.0),
    ('earth_angle', 'position/epa-rad', 1.0),
    ('x', 'position/eci-x-ft', FT),
    ('y', 'position/eci-y-ft', FT),
    ('z', 'position/eci-z-ft', FT),
    ('vx', 'velocities/eci-x-fps', FT),
    ('vy', 'velocities/eci-y-fps', FT),
    ('vz', 'velocities/eci-z-fps', FT),
    ('p', 'velocities/pi-rad_sec', 1.0),
    ('q', 'velocities/qi-rad_sec', 1.0),
    ('r', 'velocities/ri-rad_sec', 1.0),
]
DERIVED = [
    ('force_x', 'forces/fbx-total-lbs', LBF),
    ('force_y', 'forces/fby-total-lbs', LBF),
    ('force_z', 'forces/fbz-total-lbs', LBF),
    ('moment_x', 'moments/l-total-lbsft', LBF * FT),
    ('moment_y', 'moments/m-total-lbsft', LBF * FT),
    ('moment_z', 'moments/n-total-lbsft', LBF * FT),
    ('mass', 'inertia/mass-slugs', SLUG),
    ('ixx', 'inertia/ixx-slugs_ft2', SLUG_FT2),
    ('iyy', 'inertia/iyy-slugs_ft2', SLUG_FT2),
    ('izz', 'inertia/izz-slugs_ft2', SLUG_FT2),
    ('ixy', 'inertia/ixy-slugs_ft2', SLUG_FT2),
    ('ixz', 'inertia/ixz-slugs_ft2', SLUG_FT2),
    ('iyz', 'inertia/iyz-slugs_ft2', SLUG_FT2),
    # The inertial acceleration, in the inertial frame.
    ('ax', 'accelerations/uidot-ft_sec2', FT),
    ('ay', 'accelerations/vidot-ft_sec2', FT),
    ('az', 'accelerations/widot-ft_sec2', FT),
    # The rate of the inertial body rates, in body axes.
    ('pdot', 'accelerations/pidot-rad_sec2', 1.0),
    ('qdot', 'accelerations/qidot-rad_sec2', 1.0),
    ('rdot', 'accelerations/ridot-rad_sec2', 1.0),
    # Air data, and the center of mass and fuel it was found with.
    ('alpha', 'aero/alpha-rad', 1.0),
    ('beta', 'aero/beta-rad', 1.0),
    ('airspeed', 'velocities/vt-fps', FT),
    ('mach', 'velocities/mach', 1.0),
    ('dynamic_pressure', 'aero/qbar-psf', LBF / FT**2),
    ('p_air', 'velocities/p-aero-rad_sec', 1.0),
    ('q_air', 'velocities/q-aero-rad_sec', 1.0),
    ('r_air', 'velocities/r-aero-rad_sec', 1.0),
    ('altitude', 'position/h-sl-meters', 1.0),
    ('height_over_span', 'aero/h_b-mac-ft', 1.0),
    ('alpha_rate', 'aero/alphadot-rad_sec', 1.0),
    ('cg_x', 'inertia/cg-x-in', 0.0254),
    ('cg_y', 'inertia/cg-y-in', 0.0254),
    ('cg_z', 'inertia/cg-z-in', 0.0254),
]
# The fuel the frame's mass balance used, which is read before the frame:
# the engines burn after the mass balance is found.
FUEL = [('fuel_%d' % i, 'propulsion/tank[%d]/contents-lbs' % i, 0.45359237)
        for i in range(3)]


def quaternion(matrix):
    """The unit quaternion (w, x, y, z) of a rotation matrix, by Shepperd's
    method, which takes the largest of the four components first."""
    m = np.asarray(matrix)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    candidates = [trace, m[0, 0], m[1, 1], m[2, 2]]
    largest = int(np.argmax(candidates))
    if largest == 0:
        w = math.sqrt(1.0 + trace) / 2.0
        q = [w, (m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w),
             (m[1, 0] - m[0, 1]) / (4 * w)]
    elif largest == 1:
        x = math.sqrt(1.0 + 2 * m[0, 0] - trace) / 2.0
        q = [(m[2, 1] - m[1, 2]) / (4 * x), x, (m[0, 1] + m[1, 0]) / (4 * x),
             (m[0, 2] + m[2, 0]) / (4 * x)]
    elif largest == 2:
        y = math.sqrt(1.0 + 2 * m[1, 1] - trace) / 2.0
        q = [(m[0, 2] - m[2, 0]) / (4 * y), (m[0, 1] + m[1, 0]) / (4 * y), y,
             (m[1, 2] + m[2, 1]) / (4 * y)]
    else:
        z = math.sqrt(1.0 + 2 * m[2, 2] - trace) / 2.0
        q = [(m[1, 0] - m[0, 1]) / (4 * z), (m[0, 2] + m[2, 0]) / (4 * z),
             (m[1, 2] + m[2, 1]) / (4 * z), z]
    q = np.array(q)
    return q / np.linalg.norm(q)


def body_to_inertial(fdm):
    """The rotation from body axes to the inertial frame, as a quaternion."""
    ecef_to_body = np.array(fdm.get_propagate().get_Tec2b())
    angle = fdm['position/epa-rad']
    inertial_to_ecef = np.array([[math.cos(angle), math.sin(angle), 0.0],
                                 [-math.sin(angle), math.cos(angle), 0.0],
                                 [0.0, 0.0, 1.0]])
    inertial_to_body = ecef_to_body @ inertial_to_ecef
    return quaternion(inertial_to_body.T)


def start(latitude, heading):
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    fdm.load_model('737')
    fdm.set_dt(1.0 / 120.0)
    fdm['ic/h-sl-ft'] = 6000.0 / FT
    fdm['ic/vt-fps'] = 200.0 / FT
    fdm['ic/alpha-deg'] = 2.0
    fdm['ic/lat-geod-deg'] = latitude
    fdm['ic/long-gc-deg'] = 10.0
    fdm['ic/psi-true-deg'] = heading
    fdm.run_ic()
    fdm['propulsion/set-running'] = -1
    return fdm


def fly(fdm, seconds, every, rows):
    for i in range(int(seconds * 120)):
        t = i / 120.0
        fdm['fcs/elevator-cmd-norm'] = 0.2 * math.sin(1.1 * t)
        fdm['fcs/aileron-cmd-norm'] = 0.5 * math.sin(0.8 * t)
        fdm['fcs/rudder-cmd-norm'] = 0.3 * math.sin(0.6 * t)
        fdm['fcs/throttle-cmd-norm[0]'] = 0.8
        fdm['fcs/throttle-cmd-norm[1]'] = 0.6
        fuel = [fdm[p] * scale for _, p, scale in FUEL]
        fdm.run()
        if i % every == 0:
            rows.append([fdm[p] * scale for _, p, scale in STATE] +
                        list(body_to_inertial(fdm)) +
                        [fdm[p] * scale for _, p, scale in DERIVED] + fuel)


def main():
    rows = []
    fly(start(0.0, 0.0), 20.0, 10, rows)
    fly(start(60.0, 90.0), 20.0, 10, rows)
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'jsbsim_737_rigid_body.csv')
    names = ([name for name, _, _ in STATE] + ['qw', 'qx', 'qy', 'qz'] +
             [name for name, _, _ in DERIVED] + [name for name, _, _ in FUEL])
    with open(path, 'w') as table:
        table.write(','.join(names) + '\n')
        for row in rows:
            table.write(','.join('%.17g' % value for value in row) + '\n')
    print('wrote %d states to %s' % (len(rows), path))


if __name__ == '__main__':
    main()
