# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Records JSBSim's 737 turbines, for turbine_test.

Writes jsbsim_737_turbine.csv: every frame of 25 s at 60 Hz, the air the
engines breathe, their throttles, and their spool speeds, thrust and fuel
flow. The engines start settled; then the throttles step and ramp, one engine
differently from the other, while the 737 climbs and slows, so the air
changes too. Every column is read after its frame, which finds the engines'
air, runs them, and burns their fuel.

  pip install jsbsim
  python application/aeronautic/reference/jsbsim_737_turbine.py
"""

import math
import os

import jsbsim

FT = 0.3048
LBF = 4.4482216152605
LBM = 0.45359237
DT = 1.0 / 60.0

AIR = [
    ('time', 'simulation/sim-time-sec', 1.0),
    ('mach', 'velocities/mach', 1.0),
    ('density_altitude', 'atmosphere/density-altitude', FT),
    ('density_ratio', 'atmosphere/sigma', 1.0),
    ('temperature', 'atmosphere/T-R', 1.0 / 1.8),
]
ENGINE = [
    ('throttle', 'fcs/throttle-pos-norm[%d]', 1.0),
    ('n1', 'propulsion/engine[%d]/n1', 1.0),
    ('n2', 'propulsion/engine[%d]/n2', 1.0),
    ('thrust', 'propulsion/engine[%d]/thrust-lbs', LBF),
    ('fuel_flow', 'propulsion/engine[%d]/fuel-flow-rate-pps', LBM),
]


def throttle(engine, t):
    """Steps, then a ramp, then a cut, differently for each engine."""
    if engine == 0:
        if t < 2.0:
            return 0.6
        if t < 8.0:
            return 1.0
        if t < 14.0:
            return 1.0 - (t - 8.0) / 6.0
        return 0.3
    return 0.6 if t < 4.0 else (0.1 if t < 12.0 else 0.9)


def main():
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    fdm.load_model('737')
    fdm.set_dt(DT)
    fdm['ic/h-sl-ft'] = 3000.0 / FT
    fdm['ic/vt-fps'] = 220.0 / FT
    fdm['ic/gamma-deg'] = 4.0
    fdm['ic/alpha-deg'] = 4.0
    fdm['fcs/throttle-cmd-norm[0]'] = 0.6
    fdm['fcs/throttle-cmd-norm[1]'] = 0.6
    fdm.run_ic()
    fdm['propulsion/set-running'] = -1

    rows = []
    names = ([name for name, _, _ in AIR] +
             ['%s_%d' % (name, e) for e in range(2) for name, _, _ in ENGINE])
    for i in range(int(25.0 / DT)):
        t = i * DT
        for engine in range(2):
            fdm['fcs/throttle-cmd-norm[%d]' % engine] = throttle(engine, t)
        fdm['fcs/elevator-cmd-norm'] = -0.15
        fdm.run()
        row = [fdm[p] * scale for _, p, scale in AIR]
        for engine in range(2):
            row += [fdm[p % engine] * scale for _, p, scale in ENGINE]
        rows.append(row)

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'jsbsim_737_turbine.csv')
    with open(path, 'w') as table:
        table.write(','.join(names) + '\n')
        for row in rows:
            table.write(','.join('%.15g' % value for value in row) + '\n')
    print('wrote %d frames to %s' % (len(rows), path))


if __name__ == '__main__':
    main()
