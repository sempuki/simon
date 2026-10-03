# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Records JSBSim's F-16 aerodynamics, for aero_test to check simon's against.

Writes jsbsim_f16_aero.csv: at states through three short flights, what the
aerodynamics read and what JSBSim made of them. The flights are low and slow
with the gear down, in ground effect; at cruise; slow with the stick full aft,
to high angle of attack; and supersonic, with the speedbrake out. The stick
moves throughout, and the fly-by-wire flight controls move the surfaces, the
leading-edge flaps and the flaperons.

Every column is read after a frame: JSBSim derives angle of attack, dynamic
pressure and the rest from the state at the frame's start, then runs the
flight control system and the aerodynamics, then moves the state on.

  pip install jsbsim
  python application/aeronautic/reference/jsbsim_f16_aero.py
"""

import math
import os

import jsbsim

FT = 0.3048
IN = 0.0254
LBF = 4.4482216152605
PSF = LBF / (FT * FT)

COLUMNS = [
    # What the aerodynamics read.
    ('alpha', 'aero/alpha-rad', 1.0),
    ('beta', 'aero/beta-rad', 1.0),
    ('mach', 'velocities/mach', 1.0),
    ('dynamic_pressure', 'aero/qbar-psf', PSF),
    ('span_over_twice_speed', 'aero/bi2vel', 1.0),
    ('chord_over_twice_speed', 'aero/ci2vel', 1.0),
    ('roll_rate', 'velocities/p-aero-rad_sec', 1.0),
    ('pitch_rate', 'velocities/q-aero-rad_sec', 1.0),
    ('yaw_rate', 'velocities/r-aero-rad_sec', 1.0),
    ('height_over_span', 'aero/h_b-mac-ft', 1.0),
    ('elevator', 'fcs/elevator-pos-rad', 1.0),
    ('aileron-pos-rad', 'fcs/aileron-pos-rad', 1.0),
    ('rudder', 'fcs/rudder-pos-rad', 1.0),
    ('lef-pos-rad', 'fcs/lef-pos-rad', 1.0),
    ('flaperon-mix-rad', 'fcs/flaperon-mix-rad', 1.0),
    ('speedbrake', 'fcs/speedbrake-pos-rad', 1.0),
    ('gear', 'gear/gear-pos-norm', 1.0),
    # The center of mass, structural frame.
    ('cg_x', 'inertia/cg-x-in', IN),
    ('cg_y', 'inertia/cg-y-in', IN),
    ('cg_z', 'inertia/cg-z-in', IN),
    # Wind-axis forces: drag, side force and lift.
    ('drag', 'forces/fwx-aero-lbs', LBF),
    ('side', 'forces/fwy-aero-lbs', LBF),
    ('lift', 'forces/fwz-aero-lbs', LBF),
    # Body-axis forces, and moments about the center of mass.
    ('force_x', 'forces/fbx-aero-lbs', LBF),
    ('force_y', 'forces/fby-aero-lbs', LBF),
    ('force_z', 'forces/fbz-aero-lbs', LBF),
    ('moment_x', 'moments/l-aero-lbsft', LBF * FT),
    ('moment_y', 'moments/m-aero-lbsft', LBF * FT),
    ('moment_z', 'moments/n-aero-lbsft', LBF * FT),
]


def start(altitude, speed, gear):
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    fdm.load_model('f16')
    fdm.set_dt(1.0 / 120.0)
    fdm['ic/h-sl-ft'] = altitude / FT
    fdm['ic/vt-fps'] = speed / FT
    fdm['ic/alpha-deg'] = 4.0
    fdm['ic/lat-gc-deg'] = 0.0
    fdm['ic/long-gc-deg'] = 0.0
    fdm['gear/gear-cmd-norm'] = gear
    fdm.run_ic()
    fdm['propulsion/set-running'] = -1
    return fdm


def fly(fdm, seconds, every, rows, commands):
    frames = int(seconds * 120)
    for i in range(frames):
        t = i / 120.0
        for name, value in commands(t).items():
            fdm[name] = value
        fdm.run()
        if i % every == 0:
            rows.append([fdm[p] * scale for _, p, scale in COLUMNS])


def main():
    rows = []
    # Low and slow, gear down, in ground effect, the flaperons out.
    fly(start(8.0, 80.0, 1.0), 2.0, 4, rows, lambda t: {
        'fcs/elevator-cmd-norm': -0.3 + 0.2 * math.sin(4 * t),
        'fcs/aileron-cmd-norm': 0.3 * math.sin(3 * t),
        'fcs/rudder-cmd-norm': 0.2 * math.sin(2 * t),
        'fcs/throttle-cmd-norm': 0.6})
    # Cruise, the stick moving, pulling to high angle of attack.
    fly(start(6000.0, 220.0, 0.0), 10.0, 12, rows, lambda t: {
        'fcs/elevator-cmd-norm': -0.5 * math.sin(0.8 * t) ** 2,
        'fcs/aileron-cmd-norm': 0.4 * math.sin(0.9 * t),
        'fcs/rudder-cmd-norm': 0.3 * math.sin(0.7 * t),
        'fcs/throttle-cmd-norm': 0.5})
    # Slow, the stick full aft, to high angle of attack.
    fly(start(3000.0, 120.0, 0.0), 6.0, 12, rows, lambda t: {
        'fcs/elevator-cmd-norm': -1.0,
        'fcs/aileron-cmd-norm': 0.2 * math.sin(1.1 * t),
        'fcs/throttle-cmd-norm': 0.4})
    # Supersonic, in reheat, speedbrake out.
    fly(start(10000.0, 400.0, 0.0), 4.0, 6, rows, lambda t: {
        'fcs/elevator-cmd-norm': 0.1 * math.sin(2 * t),
        'fcs/aileron-cmd-norm': -0.3 * math.sin(1.5 * t),
        'fcs/rudder-cmd-norm': 0.2 * math.sin(t),
        'fcs/speedbrake-cmd-norm': 1.0 if t > 1.0 else 0.0,
        'fcs/throttle-cmd-norm': 1.0})

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'jsbsim_f16_aero.csv')
    with open(path, 'w') as table:
        table.write(','.join(name for name, _, _ in COLUMNS) + '\n')
        for row in rows:
            table.write(','.join('%.17g' % value for value in row) + '\n')
    print('wrote %d states to %s' % (len(rows), path))


if __name__ == '__main__':
    main()
