# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Records JSBSim's F-16 turbine, for turbine_test.

Writes jsbsim_f16_turbine.csv: every frame of 30 s at 120 Hz, the air the
engine breathes, its throttle, and its spool speeds, thrust and fuel flow. The
engine starts settled; then the throttle steps into reheat part way and fully,
back out, and in again, while the F-16 climbs, so the air changes too. The
flight controls double the pilot's throttle, so a throttle past 1 lights the
reheat. Every column is read after its frame, which finds the engines' air,
runs them, and burns their fuel.

  pip install jsbsim
  python application/aeronautic/reference/jsbsim_f16_turbine.py
"""

import os

import jsbsim

FT = 0.3048
LBF = 4.4482216152605
LBM = 0.45359237
DT = 1.0 / 120.0

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


def throttle(t):
    """The pilot's throttle: military, then reheat part way and fully, back
    to dry, into reheat again, and to idle."""
    for until, value in [(2.0, 0.5), (6.0, 0.75), (10.0, 1.0), (14.0, 0.6),
                         (18.0, 0.4), (19.0, 0.9), (24.0, 0.55)]:
        if t < until:
            return value
    return 0.1


def main():
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    fdm.load_model('f16')
    fdm.set_dt(DT)
    fdm['ic/h-sl-ft'] = 6000.0 / FT
    fdm['ic/vt-fps'] = 250.0 / FT
    fdm['ic/gamma-deg'] = 4.0
    fdm['ic/alpha-deg'] = 3.0
    fdm['fcs/throttle-cmd-norm'] = 0.5
    fdm.run_ic()
    fdm['propulsion/set-running'] = -1

    rows = []
    names = [name for name, _, _ in AIR] + ['%s_0' % name for name, _, _ in ENGINE]
    for i in range(int(30.0 / DT)):
        fdm['fcs/throttle-cmd-norm'] = throttle(i * DT)
        fdm.run()
        row = [fdm[p] * scale for _, p, scale in AIR]
        row += [fdm[p % 0] * scale for _, p, scale in ENGINE]
        rows.append(row)

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'jsbsim_f16_turbine.csv')
    with open(path, 'w') as table:
        table.write(','.join(names) + '\n')
        for row in rows:
            table.write(','.join('%.17g' % value for value in row) + '\n')
    print('wrote %d frames to %s' % (len(rows), path))


if __name__ == '__main__':
    main()
