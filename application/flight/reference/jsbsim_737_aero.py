# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Records JSBSim's 737 aerodynamics, for aero_test to check simon's against.

Writes jsbsim_737_aero.csv: at states through three short flights, what the
aerodynamics read and what JSBSim made of them. The flights are low and slow
with flaps, gear and spoilers out, in ground effect; at cruise; and fast,
above the drag rise, with the speedbrake out. The controls move throughout.

Every column is read after a frame: JSBSim derives angle of attack, dynamic
pressure and the rest from the state at the frame's start, then runs the
flight control system and the aerodynamics, then moves the state on. JSBSim's
lift coefficient lags a frame: its induced drag uses the frame before's.

  pip install jsbsim
  python application/flight/reference/jsbsim_737_aero.py
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
    ('alpha_rate', 'aero/alphadot-rad_sec', 1.0),
    ('lift_coefficient_squared', 'aero/cl-squared', 1.0),
    ('height_over_span', 'aero/h_b-mac-ft', 1.0),
    ('elevator', 'fcs/elevator-pos-rad', 1.0),
    ('elevator_magnitude', 'fcs/mag-elevator-pos-rad', 1.0),
    ('left_aileron', 'fcs/left-aileron-pos-rad', 1.0),
    ('right_aileron', 'fcs/right-aileron-pos-rad', 1.0),
    ('rudder', 'fcs/rudder-pos-rad', 1.0),
    ('flaps_norm', 'fcs/flap-pos-norm', 1.0),
    ('gear', 'gear/gear-pos-norm', 1.0),
    ('speedbrake_norm', 'fcs/speedbrake-pos-norm', 1.0),
    ('spoilers_norm', 'fcs/spoiler-pos-norm', 1.0),
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


def start(altitude, speed, flaps, gear):
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    fdm.load_model('737')
    fdm.set_dt(1.0 / 120.0)
    fdm['ic/h-sl-ft'] = altitude / FT
    fdm['ic/vt-fps'] = speed / FT
    fdm['ic/alpha-deg'] = 3.0
    fdm['ic/lat-gc-deg'] = 0.0
    fdm['ic/long-gc-deg'] = 0.0
    fdm['fcs/flap-cmd-norm'] = flaps
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
    # Low and slow, flaps and gear out, in ground effect, spoilers coming out.
    fly(start(12.0, 75.0, 1.0, 1.0), 2.0, 4, rows, lambda t: {
        'fcs/elevator-cmd-norm': -0.3 + 0.2 * math.sin(4 * t),
        'fcs/aileron-cmd-norm': 0.3 * math.sin(3 * t),
        'fcs/rudder-cmd-norm': 0.2 * math.sin(2 * t),
        'fcs/spoiler-cmd-norm': 1.0 if t > 1.0 else 0.0,
        'fcs/throttle-cmd-norm[0]': 0.6, 'fcs/throttle-cmd-norm[1]': 0.6})
    # Cruise, the controls moving.
    fly(start(6000.0, 200.0, 0.0, 0.0), 10.0, 12, rows, lambda t: {
        'fcs/elevator-cmd-norm': 0.15 * math.sin(1.3 * t),
        'fcs/aileron-cmd-norm': 0.4 * math.sin(0.9 * t),
        'fcs/rudder-cmd-norm': 0.3 * math.sin(0.7 * t),
        'fcs/throttle-cmd-norm[0]': 0.7, 'fcs/throttle-cmd-norm[1]': 0.7})
    # Above the drag rise, speedbrake out.
    fly(start(9000.0, 285.0, 0.0, 0.0), 4.0, 6, rows, lambda t: {
        'fcs/elevator-cmd-norm': 0.1 * math.sin(2 * t),
        'fcs/aileron-cmd-norm': -0.3 * math.sin(1.5 * t),
        'fcs/rudder-cmd-norm': 0.2 * math.sin(t),
        'fcs/speedbrake-cmd-norm': 1.0 if t > 1.0 else 0.0,
        'fcs/throttle-cmd-norm[0]': 0.9, 'fcs/throttle-cmd-norm[1]': 0.9})

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'jsbsim_737_aero.csv')
    with open(path, 'w') as table:
        table.write(','.join(name for name, _, _ in COLUMNS) + '\n')
        for row in rows:
            table.write(','.join('%.17g' % value for value in row) + '\n')
    print('wrote %d states to %s' % (len(rows), path))


if __name__ == '__main__':
    main()
