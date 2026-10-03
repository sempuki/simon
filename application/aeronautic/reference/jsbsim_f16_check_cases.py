# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Flies JSBSim's F-16 through open-loop check cases, for check_case_test.

The F-16 is trimmed in cruise at 6 km and 200 m/s, 30 degrees north, heading
northeast, and from there flown for 30 s in each of six cases: holding the
trim, a stick doublet in pitch and in roll, a rudder doublet, and a throttle
step into reheat on top of it (see `command`), and holding the trim into a
wind that starts just after 1 s (see `WIND`). The pilot's commands
are open loop; the fly-by-wire flight controls close their own loops on rate
and load factor. Each case is flown at 8 ms, and at 0.125 ms, where JSBSim has
converged to a millimeter or so, and to 5 cm after the roll doublet. The flight
controls run every 8 ms at both, as a digital flight control computer runs at
its own rate: at 0.125 ms each channel but the throttle's runs every 64th
frame. The F-16's control laws differentiate the pilot's commands and then clip
them, so at JSBSim's own frame they change with the frame and never converge.
JSBSim's trim at 0.125 ms comes to the same state as at 8 ms, but its pitch
trim differs by a few parts in 10^5, which the F-16 feels; so every flight
takes its commands from the trim at 8 ms.

Writes jsbsim_f16_check_initial.csv, the trimmed state and controls and the
mass properties; jsbsim_f16_check_signals.csv, every flight control signal the
trim leaves; and jsbsim_f16_check_cases.csv, every 0.2 s of every case at both
frames. After the trim each PID's integral is zeroed, as simon's start, because
JSBSim does not show it.

  pip install jsbsim numpy
  python application/aeronautic/reference/jsbsim_f16_check_cases.py
"""

import os
import re
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ElementTree

import jsbsim
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'tools', 'jsbsim'))
import convert  # noqa: E402
from jsbsim_737_check_cases import state  # noqa: E402

FT = 0.3048
IN = 0.0254
LBM = 0.45359237
SLUG_FT2 = convert.SLUG_FT2
COARSE = 8000  # Microseconds.
FINE = 125
SECONDS = 30
SAMPLE = 200000  # Microseconds: a multiple of both frames.

CASES = ['hold', 'elevator', 'aileron', 'rudder', 'throttle', 'wind']
# The wind in the wind case: north, east and down, in m/s, from WIND_START
# microseconds, between two of the F-16's flight control frames, so that
# each frame reads the air its state moved through.
WIND = (8.0, -12.0, 2.0)
WIND_START = 1004000
PIDS = ['roll-rate-pid', 'g-load-pid', 'yaw-load-pid']


def command(case, microseconds):
    """What each case adds to the trimmed stick, rudder and throttle
    commands, at a time in whole microseconds."""
    t = microseconds
    doublet = (1 if 1000000 <= t < 2000000 else
               -1 if 2000000 <= t < 3000000 else 0)
    return {
        'elevator': 0.05 * doublet if case == 'elevator' else 0.0,
        'aileron': 0.2 * doublet if case == 'aileron' else 0.0,
        'rudder': 0.2 * doublet if case == 'rudder' else 0.0,
        'throttle': 0.4 if case == 'throttle' and t >= 1000000 else 0.0,
    }


TRIMMED = ['elevator', 'aileron', 'rudder', 'pitch_trim', 'roll_trim',
           'yaw_trim', 'throttle_0']
TRIM_PROPERTIES = ['fcs/elevator-cmd-norm', 'fcs/aileron-cmd-norm',
                   'fcs/rudder-cmd-norm', 'fcs/pitch-trim-cmd-norm',
                   'fcs/roll-trim-cmd-norm', 'fcs/yaw-trim-cmd-norm',
                   'fcs/throttle-cmd-norm']
MASS = [('mass', 'inertia/mass-slugs', convert.SLUG),
        ('cg_x', 'inertia/cg-x-in', IN), ('cg_y', 'inertia/cg-y-in', IN),
        ('cg_z', 'inertia/cg-z-in', IN),
        ('ixx', 'inertia/ixx-slugs_ft2', SLUG_FT2),
        ('iyy', 'inertia/iyy-slugs_ft2', SLUG_FT2),
        ('izz', 'inertia/izz-slugs_ft2', SLUG_FT2),
        ('ixz', 'inertia/ixz-slugs_ft2', SLUG_FT2)]


def signals():
    """Each flight control signal JSBSim's F-16 has: its components' outputs,
    and the properties they write, by simon's name, with the factor to SI."""
    path = os.path.join(jsbsim.get_default_root_dir(), 'aircraft', 'f16',
                        'f16.xml')
    controls = ElementTree.parse(path).getroot().find('flight_control')
    props = []
    for channel in controls.findall('channel'):
        for component in channel:
            props.append(convert.component_property(component.get('name')))
            props += [o.text.strip() for o in component.findall('output')]
    result = {}
    for prop in props:
        name, scale = convert.flight_signal(prop)
        result.setdefault(name, (prop, scale))
    return result


def aircraft_path(frame):
    """A directory holding the F-16, its flight control channels run every
    COARSE microseconds at frames of `frame` microseconds."""
    rate = COARSE // frame
    if rate == 1:
        return os.path.join(jsbsim.get_default_root_dir(), 'aircraft')
    directory = os.path.join(tempfile.gettempdir(), 'jsbsim_f16_execrate_%d' % rate)
    if not os.path.isdir(directory):
        shutil.copytree(os.path.join(jsbsim.get_default_root_dir(), 'aircraft',
                                     'f16'),
                        os.path.join(directory, 'f16'))
        path = os.path.join(directory, 'f16', 'f16.xml')
        with open(path) as source:
            text = source.read()
        with open(path, 'w') as target:
            # Not the throttle's: JSBSim sets the throttle to its command
            # every frame before the channels run, and the channel, a gain,
            # would double it only on the computer's frames. Its command
            # changes only on the computer's frames, so running it every
            # frame is the same.
            target.write(re.sub(r'<channel name="(?!Throttle")',
                                '<channel execrate="%d" name="' % rate, text))
    return directory


def trimmed(frame):
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    # Before loading: components keep the frame time they load with.
    fdm.set_dt(frame / 1e6)
    fdm.set_aircraft_path(aircraft_path(frame))
    fdm.load_model('f16')
    fdm['ic/h-sl-ft'] = 6000.0 / FT
    fdm['ic/vt-fps'] = 200.0 / FT
    fdm['ic/gamma-deg'] = 0.0
    fdm['ic/lat-geod-deg'] = 30.0
    fdm['ic/long-gc-deg'] = 10.0
    fdm['ic/psi-true-deg'] = 45.0
    fdm['gear/gear-cmd-norm'] = 0.0
    fdm.run_ic()
    fdm['propulsion/set-running'] = -1
    fdm.do_trim(1)
    for pid in PIDS:
        fdm['fcs/%s/initial-integrator-value' % pid] = 0.0
    return fdm


def fly(case, frame, flight_signals, commands=None):
    fdm = trimmed(frame)
    if commands is not None:
        for prop, value in zip(TRIM_PROPERTIES, commands):
            fdm[prop] = value
    trim = [fdm[p] for p in TRIM_PROPERTIES]
    initial = (state(fdm) + trim +
               [fdm['propulsion/tank[%d]/contents-lbs' % i] * LBM
                for i in range(4)] +
               [fdm[p] * scale for _, p, scale in MASS])
    trimmed_signals = [fdm[prop] * scale
                       for prop, scale in flight_signals.values()]
    rows = [[CASES.index(case), frame, 0.0] + state(fdm)]
    frames = SECONDS * 1000000 // frame
    for k in range(frames):
        t = (k + 1) * frame  # The frame's end.
        offset = command(case, t)
        fdm['fcs/elevator-cmd-norm'] = trim[0] + offset['elevator']
        fdm['fcs/aileron-cmd-norm'] = trim[1] + offset['aileron']
        fdm['fcs/rudder-cmd-norm'] = trim[2] + offset['rudder']
        fdm['fcs/throttle-cmd-norm'] = trim[6] + offset['throttle']
        blowing = case == 'wind' and t >= WIND_START
        for axis, speed in zip(('north', 'east', 'down'), WIND):
            fdm['atmosphere/wind-%s-fps' % axis] = (speed / FT if blowing
                                                    else 0.0)
        fdm.run()
        if t % SAMPLE == 0:
            rows.append([CASES.index(case), frame, t / 1e6] + state(fdm))
    return initial, trimmed_signals, rows


def main():
    flight_signals = signals()
    initial = None
    trimmed_signals = None
    commands = None
    rows = []
    for case in CASES:
        for frame in (COARSE, FINE):
            start, start_signals, flown = fly(case, frame, flight_signals,
                                              commands)
            if commands is None:
                commands = start[13:13 + len(TRIMMED)]
            if frame == COARSE:
                if initial is None:
                    initial, trimmed_signals = start, start_signals
                # Every case starts from the same trim.
                assert np.allclose(initial, start, rtol=0, atol=1e-9), case
                assert np.allclose(trimmed_signals, start_signals, rtol=0,
                                   atol=1e-9), case
            rows += flown
            print(case, frame, 'done')

    state_names = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'qw', 'qx', 'qy', 'qz',
                   'p', 'q', 'r']
    with open(os.path.join(HERE, 'jsbsim_f16_check_initial.csv'), 'w') as table:
        table.write(','.join(state_names + TRIMMED +
                             ['fuel_%d' % i for i in range(4)] +
                             [name for name, _, _ in MASS]) + '\n')
        table.write(','.join('%.17g' % v for v in initial) + '\n')
    with open(os.path.join(HERE, 'jsbsim_f16_check_signals.csv'), 'w') as table:
        table.write(','.join(flight_signals) + '\n')
        table.write(','.join('%.17g' % v for v in trimmed_signals) + '\n')
    with open(os.path.join(HERE, 'jsbsim_f16_check_cases.csv'), 'w') as table:
        table.write(','.join(['case', 'frame_us', 'time'] + state_names) + '\n')
        for row in rows:
            table.write(','.join('%.17g' % v for v in row) + '\n')
    print('wrote %d rows' % len(rows))


if __name__ == '__main__':
    main()
