# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Records JSBSim's F-16 flight control system, for flight_control_test.

Writes jsbsim_f16_flight_control.csv: every frame of three flights at 120 Hz,
the pilot's commands, the state the flight controls read, and every
component's output. The first flight cruises while the stick, rudder and
throttle sweep, the speedbrake opens and the gear cycles; the second is slow,
below the speed that lowers the trailing-edge flaps, and pulls to high angle
of attack; the third is supersonic.

JSBSim moves the state on at the start of a frame, then runs its flight
controls, then finds the frame's air data. So the flight controls read the
frame's attitude and body velocity, which are read after it, and the frame
before's air data and accelerations, which are read before it.

Columns use simon's names and SI units, from tools/jsbsim/convert.py.

  pip install jsbsim
  python application/flight/reference/jsbsim_f16_flight_control.py
"""

import math
import os
import sys
import xml.etree.ElementTree as ElementTree

import jsbsim

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'tools', 'jsbsim'))
import convert  # noqa: E402

DT = 1.0 / 120.0

COMMANDS = ['fcs/elevator-cmd-norm', 'fcs/aileron-cmd-norm',
            'fcs/rudder-cmd-norm', 'gear/gear-cmd-norm',
            'fcs/speedbrake-cmd-norm', 'fcs/pitch-trim-cmd-norm',
            'fcs/yaw-trim-cmd-norm', 'fcs/throttle-cmd-norm']
# State the flight controls read, by whether it is the frame's or the frame
# before's.
FRAME = ['attitude/pitch-rad', 'attitude/roll-rad', 'velocities/u-fps',
         'velocities/v-fps']
BEFORE = ['velocities/mach', 'velocities/p-aero-rad_sec',
          'velocities/q-aero-rad_sec', 'velocities/r-aero-rad_sec',
          'aero/alpha-rad', 'velocities/vc-kts', 'velocities/vg-fps',
          'accelerations/n-pilot-y-norm', 'accelerations/n-pilot-z-norm']
# Outputs to properties the components do not name themselves.
OUTPUTS = ['fcs/aileron-pos-rad', 'fcs/left-aileron-pos-norm',
           'fcs/left-aileron-pos-rad', 'fcs/right-aileron-pos-rad',
           'fcs/elevator-pos-norm', 'fcs/elevator-pos-rad',
           'fcs/rudder-pos-norm', 'fcs/rudder-pos-rad', 'gear/gear-pos-norm',
           'fcs/speedbrake-pos-rad', 'fcs/speedbrake-pos-norm',
           'fcs/throttle-pos-norm']


def components():
    """The property each of the F-16's flight control components sets."""
    path = os.path.join(jsbsim.get_default_root_dir(), 'aircraft', 'f16',
                        'f16.xml')
    controls = ElementTree.parse(path).getroot().find('flight_control')
    return [convert.component_property(c.get('name'))
            for channel in controls.findall('channel') for c in channel]


def column(prop):
    """simon's name for `prop`, and the factor from JSBSim's units to SI."""
    return convert.flight_signal(prop)


def start(altitude, speed, alpha):
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    # Before loading: components keep the frame time they load with.
    fdm.set_dt(DT)
    fdm.load_model('f16')
    fdm['ic/h-sl-ft'] = altitude / convert.FT
    fdm['ic/vt-fps'] = speed / convert.FT
    fdm['ic/alpha-deg'] = alpha
    fdm['gear/gear-cmd-norm'] = 0.0
    fdm.run_ic()
    fdm['propulsion/set-running'] = -1
    # run_ic runs the flight controls before it finds the air data, so it
    # reads no airspeed, and the PIDs integrate. Each flight starts with them
    # at zero, as simon's do.
    for pid in ['roll-rate-pid', 'g-load-pid', 'yaw-load-pid']:
        fdm['fcs/%s/initial-integrator-value' % pid] = 0.0
    return fdm


def fly(run, fdm, seconds, commands, props, rows):
    for i in range(int(seconds / DT)):
        for name, value in commands(i * DT).items():
            fdm[name] = value
        before = [fdm[p] for p in BEFORE]
        fdm.run()
        values = dict(zip(BEFORE, before))
        for p in COMMANDS + FRAME + props:
            values[p] = fdm[p]
        rows.append([run] + [values[p] * column(p)[1] for p in
                             COMMANDS + FRAME + BEFORE + props])


def main():
    props = components()
    props += [p for p in OUTPUTS if p not in props]
    rows = []
    fly(0, start(6000.0, 220.0, 2.0), 15.0, lambda t: {
        'fcs/elevator-cmd-norm': 0.4 * math.sin(1.3 * t),
        'fcs/aileron-cmd-norm': 0.6 * math.sin(0.7 * t),
        'fcs/rudder-cmd-norm': 0.5 * math.sin(0.5 * t),
        'gear/gear-cmd-norm': 1.0 if 2.0 < t < 9.0 else 0.0,
        'fcs/speedbrake-cmd-norm': 1.0 if t > 5.0 else 0.0,
        'fcs/pitch-trim-cmd-norm': -0.1 if t > 3.0 else 0.0,
        'fcs/yaw-trim-cmd-norm': 0.1 if t > 6.0 else 0.0,
        'fcs/throttle-cmd-norm': 0.5 + 0.5 * math.sin(0.4 * t)}, props, rows)
    fly(1, start(3000.0, 110.0, 6.0), 10.0, lambda t: {
        'fcs/elevator-cmd-norm': -min(t / 3.0, 1.0),
        'fcs/aileron-cmd-norm': 0.3 * math.sin(1.1 * t),
        'fcs/throttle-cmd-norm': 0.6}, props, rows)
    fly(2, start(10000.0, 330.0, 2.0), 5.0, lambda t: {
        'fcs/elevator-cmd-norm': 0.2 * math.sin(2.0 * t),
        'fcs/aileron-cmd-norm': -0.4 * math.sin(1.5 * t),
        'fcs/rudder-cmd-norm': 0.3 * math.sin(t),
        'fcs/throttle-cmd-norm': 1.0}, props, rows)

    names = ['run'] + [column(p)[0] for p in COMMANDS + FRAME + BEFORE + props]
    path = os.path.join(HERE, 'jsbsim_f16_flight_control.csv')
    with open(path, 'w') as table:
        table.write(','.join(names) + '\n')
        for row in rows:
            table.write(','.join('%.17g' % value for value in row) + '\n')
    print('wrote %d frames to %s' % (len(rows), path))


if __name__ == '__main__':
    main()
