# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Flies JSBSim's 737 through open-loop check cases, for check_case_test.

The 737 is trimmed in cruise at 6 km and 200 m/s, 30 degrees north, heading
northeast, and from there flown open loop for 30 s in each of five cases:
holding the trim, and an elevator, aileron and rudder doublet and a throttle
step on top of it (see `command`). Each case is flown twice: at a frame of
8 ms, simon's step, and at 0.5 ms, where JSBSim's integrators and its frame
lags have converged, as the reference both are judged against.

Writes jsbsim_737_check_initial.csv, the trimmed state and controls, and
jsbsim_737_check_cases.csv, every 0.2 s of every case at both frames: the
state in the Earth-centered inertial frame.

The command before each frame is the one at the frame's end, so that the
state a frame ends at moves under the command of its own time, as it does in
simon.

  pip install jsbsim numpy
  python application/flight/reference/jsbsim_737_check_cases.py
"""

import math
import os

import jsbsim
import numpy as np

FT = 0.3048
LBM = 0.45359237
COARSE = 8000  # Microseconds.
FINE = 500
SECONDS = 30
SAMPLE = 200000  # Microseconds: a multiple of both frames.

CASES = ['hold', 'elevator', 'aileron', 'rudder', 'throttle']


def command(case, microseconds):
    """What each case adds to the trimmed elevator, aileron and rudder
    commands and throttle, at a time in whole microseconds."""
    t = microseconds
    doublet = (1 if 1000000 <= t < 2000000 else
               -1 if 2000000 <= t < 3000000 else 0)
    return {
        'elevator': 0.05 * doublet if case == 'elevator' else 0.0,
        'aileron': 0.2 * doublet if case == 'aileron' else 0.0,
        'rudder': 0.2 * doublet if case == 'rudder' else 0.0,
        'throttle': 0.2 if case == 'throttle' and t >= 1000000 else 0.0,
    }


def quaternion(matrix):
    """The unit quaternion (w, x, y, z) of a rotation matrix (Shepperd)."""
    m = np.asarray(matrix)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    largest = int(np.argmax([trace, m[0, 0], m[1, 1], m[2, 2]]))
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


def state(fdm):
    """Time, ECI position and velocity, body-to-ECI quaternion and inertial
    body rates, in SI."""
    angle = fdm['position/epa-rad']
    inertial_to_ecef = np.array([[math.cos(angle), math.sin(angle), 0.0],
                                 [-math.sin(angle), math.cos(angle), 0.0],
                                 [0.0, 0.0, 1.0]])
    inertial_to_body = np.array(fdm.get_propagate().get_Tec2b()) @ inertial_to_ecef
    return ([fdm['position/eci-%s-ft' % a] * FT for a in 'xyz'] +
            [fdm['velocities/eci-%s-fps' % a] * FT for a in 'xyz'] +
            list(quaternion(inertial_to_body.T)) +
            [fdm['velocities/%si-rad_sec' % a] for a in 'pqr'])


TRIMMED = ['elevator', 'aileron', 'rudder', 'pitch_trim', 'roll_trim',
           'yaw_trim', 'throttle_0', 'throttle_1']
TRIM_PROPERTIES = ['fcs/elevator-cmd-norm', 'fcs/aileron-cmd-norm',
                   'fcs/rudder-cmd-norm', 'fcs/pitch-trim-cmd-norm',
                   'fcs/roll-trim-cmd-norm', 'fcs/yaw-trim-cmd-norm',
                   'fcs/throttle-cmd-norm[0]', 'fcs/throttle-cmd-norm[1]']


def trimmed(frame):
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    # Before loading: kinematic components keep the frame time they load with.
    fdm.set_dt(frame / 1e6)
    fdm.load_model('737')
    fdm['ic/h-sl-ft'] = 6000.0 / FT
    fdm['ic/vt-fps'] = 200.0 / FT
    fdm['ic/gamma-deg'] = 0.0
    fdm['ic/lat-geod-deg'] = 30.0
    fdm['ic/long-gc-deg'] = 10.0
    fdm['ic/psi-true-deg'] = 45.0
    fdm['gear/gear-cmd-norm'] = 0.0
    fdm['fcs/flap-cmd-norm'] = 0.0
    fdm.run_ic()
    fdm['propulsion/set-running'] = -1
    fdm.do_trim(1)
    return fdm


def fly(case, frame):
    fdm = trimmed(frame)
    trim = [fdm[p] for p in TRIM_PROPERTIES]
    initial = state(fdm) + trim + [
        fdm['propulsion/tank[%d]/contents-lbs' % i] * LBM for i in range(3)]
    rows = [[CASES.index(case), frame, 0.0] + state(fdm)]
    frames = SECONDS * 1000000 // frame
    for k in range(frames):
        t = (k + 1) * frame  # The frame's end.
        offset = command(case, t)
        fdm['fcs/elevator-cmd-norm'] = trim[0] + offset['elevator']
        fdm['fcs/aileron-cmd-norm'] = trim[1] + offset['aileron']
        fdm['fcs/rudder-cmd-norm'] = trim[2] + offset['rudder']
        fdm['fcs/throttle-cmd-norm[0]'] = trim[6] + offset['throttle']
        fdm['fcs/throttle-cmd-norm[1]'] = trim[7] + offset['throttle']
        fdm.run()
        if t % SAMPLE == 0:
            rows.append([CASES.index(case), frame, t / 1e6] + state(fdm))
    return initial, rows


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    initial = None
    rows = []
    for case in CASES:
        for frame in (COARSE, FINE):
            start, flown = fly(case, frame)
            if frame == COARSE:
                if initial is None:
                    initial = start
                # Every case starts from the same trim.
                assert np.allclose(initial, start, rtol=0, atol=1e-9), case
            rows += flown
            print(case, frame, 'done')

    state_names = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'qw', 'qx', 'qy', 'qz',
                   'p', 'q', 'r']
    with open(os.path.join(here, 'jsbsim_737_check_initial.csv'), 'w') as table:
        table.write(','.join(state_names + TRIMMED +
                             ['fuel_0', 'fuel_1', 'fuel_2']) + '\n')
        table.write(','.join('%.17g' % v for v in initial) + '\n')
    with open(os.path.join(here, 'jsbsim_737_check_cases.csv'), 'w') as table:
        table.write(','.join(['case', 'frame_us', 'time'] + state_names) + '\n')
        for row in rows:
            table.write(','.join('%.17g' % v for v in row) + '\n')
    print('wrote %d rows' % len(rows))


if __name__ == '__main__':
    main()
