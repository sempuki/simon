# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Records JSBSim's 737 flight control system, for flight_control_test.

Writes jsbsim_737_flight_control.csv: every frame of 15 s at 60 Hz, the
pilot's commands, the state the flight controls read, and the surface
positions they set. The commands sweep the elevator, ailerons and rudder,
step the trims, and lower the flaps and gear and raise the speedbrake and
spoilers part way. JSBSim runs its flight controls before it finds the
frame's air data, so the state they read is the frame before's, which is read
before each frame; the surfaces are read after it.

  pip install jsbsim
  python application/aeronautic/reference/jsbsim_737_flight_control.py
"""

import math
import os

import jsbsim

DT = 1.0 / 60.0

COMMANDS = [
    ('elevator_command', 'fcs/elevator-cmd-norm'),
    ('aileron_command', 'fcs/aileron-cmd-norm'),
    ('rudder_command', 'fcs/rudder-cmd-norm'),
    ('flaps_command', 'fcs/flap-cmd-norm'),
    ('gear_command', 'gear/gear-cmd-norm'),
    ('speedbrake_command', 'fcs/speedbrake-cmd-norm'),
    ('spoilers_command', 'fcs/spoiler-cmd-norm'),
    ('pitch_trim_command', 'fcs/pitch-trim-cmd-norm'),
    ('roll_trim_command', 'fcs/roll-trim-cmd-norm'),
    ('yaw_trim_command', 'fcs/yaw-trim-cmd-norm'),
]
STATE = [
    ('mach', 'velocities/mach'),
    ('yaw_rate', 'velocities/r-aero-rad_sec'),
]
SURFACES = [
    ('elevator', 'fcs/elevator-pos-rad'),
    ('left_aileron', 'fcs/left-aileron-pos-rad'),
    ('right_aileron', 'fcs/right-aileron-pos-rad'),
    ('rudder', 'fcs/rudder-pos-rad'),
    ('flaps_norm', 'fcs/flap-pos-norm'),
    ('gear', 'gear/gear-pos-norm'),
    ('speedbrake_norm', 'fcs/speedbrake-pos-norm'),
    ('spoilers_norm', 'fcs/spoiler-pos-norm'),
]


def commands(t):
    return {
        'fcs/elevator-cmd-norm': 0.6 * math.sin(1.3 * t),
        'fcs/aileron-cmd-norm': 0.9 * math.sin(0.7 * t),
        'fcs/rudder-cmd-norm': 0.8 * math.sin(0.5 * t),
        'fcs/flap-cmd-norm': 0.0 if t < 1.0 else (0.6 if t < 10.0 else 0.2),
        'gear/gear-cmd-norm': 1.0 if 2.0 < t < 9.0 else 0.0,
        'fcs/speedbrake-cmd-norm': 0.5 if t > 5.0 else 0.0,
        'fcs/spoiler-cmd-norm': 0.3 if t > 7.0 else 0.0,
        'fcs/pitch-trim-cmd-norm': 0.2 if t > 3.0 else 0.0,
        'fcs/roll-trim-cmd-norm': -0.1 if t > 4.0 else 0.0,
        'fcs/yaw-trim-cmd-norm': 0.15 if t > 6.0 else 0.0,
    }


def main():
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    # Before loading: kinematic components keep the frame time they load with.
    fdm.set_dt(DT)
    fdm.load_model('737')
    fdm['ic/h-sl-ft'] = 6000.0 / 0.3048
    fdm['ic/vt-fps'] = 200.0 / 0.3048
    fdm['ic/alpha-deg'] = 2.0
    fdm['gear/gear-cmd-norm'] = 0.0
    fdm.run_ic()
    fdm['propulsion/set-running'] = -1

    rows = []
    for i in range(int(15.0 / DT)):
        for name, value in commands(i * DT).items():
            fdm[name] = value
        state = [fdm[p] for _, p in STATE]
        fdm.run()
        rows.append([fdm[p] for _, p in COMMANDS] + state +
                    [fdm[p] for _, p in SURFACES])

    names = [n for n, _ in COMMANDS] + [n for n, _ in STATE] + [n for n, _ in SURFACES]
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'jsbsim_737_flight_control.csv')
    with open(path, 'w') as table:
        table.write(','.join(names) + '\n')
        for row in rows:
            table.write(','.join('%.15g' % value for value in row) + '\n')
    print('wrote %d frames to %s' % (len(rows), path))


if __name__ == '__main__':
    main()
