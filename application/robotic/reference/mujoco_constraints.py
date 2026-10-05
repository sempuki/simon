# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Steps robotic's test models in MuJoCo with contacts, limits and dry
friction, for constraint_test.

MuJoCo (https://mujoco.org, Apache-2.0) steps each case below from its
starting state, by Newton's method or projected Gauss-Seidel, and this script
writes mujoco_constraints.csv: for each case and step, the positions and
velocities after it, space-separated, to 17 significant digits; step 0 is the
start.

MuJoCo 3.14.0:

  pip install mujoco
  python application/robotic/reference/mujoco_constraints.py
"""

import csv
import os

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', 'models')
HUMANOID = os.path.join(HERE, '..', '..', '..', '3rd_party', 'mujoco',
                        'humanoid.xml')

NEWTON = mujoco.mjtSolver.mjSOL_NEWTON
PGS = mujoco.mjtSolver.mjSOL_PGS
ELLIPTIC = mujoco.mjtCone.mjCONE_ELLIPTIC
DRIVEN = [0.3 * ((7 * k) % 11 - 5) / 5 for k in range(21)]

# Name, model, steps, solver, starting positions and velocities to set by
# address, and optionally controls and the friction cone.
CASES = [
    ('rolling', 'rolling.xml', 1500, NEWTON, {}, {0: 2.0}),
    ('sliding', 'sliding.xml', 1000, NEWTON, {}, {0: 2.0, 7: 1.0}),
    ('stack', 'boxes.xml', 1000, NEWTON, {}, {}),
    ('limits', 'limits.xml', 1500, NEWTON, {0: 0.3, 5: -0.1},
     {0: 3.0, 1: 2.0, 2: -1.0, 4: 1.5, 5: 2.0}),
    ('rolling by PGS', 'rolling.xml', 1500, PGS, {}, {0: 2.0}),
    ('sliding by PGS', 'sliding.xml', 1000, PGS, {}, {0: 2.0, 7: 1.0}),
    ('stack by PGS', 'boxes.xml', 1000, PGS, {}, {}),
    ('limits by PGS', 'limits.xml', 1500, PGS, {0: 0.3, 5: -0.1},
     {0: 3.0, 1: 2.0, 2: -1.0, 4: 1.5, 5: 2.0}),
    ('humanoid falling', HUMANOID, 400, NEWTON, {}, {}),
    ('humanoid driven', HUMANOID, 400, NEWTON, {}, {}, DRIVEN),
    ('humanoid falling by PGS', HUMANOID, 400, PGS, {}, {}),
    ('rolling elliptic', 'rolling.xml', 1500, NEWTON, {}, {0: 2.0}, None,
     ELLIPTIC),
    ('sliding elliptic', 'sliding.xml', 1000, NEWTON, {},
     {0: 2.0, 7: 1.0}, None, ELLIPTIC),
    ('stack elliptic', 'boxes.xml', 1000, NEWTON, {}, {}, None, ELLIPTIC),
    ('humanoid driven elliptic', HUMANOID, 400, NEWTON, {}, {}, DRIVEN,
     ELLIPTIC),
    ('rolling elliptic by PGS', 'rolling.xml', 1500, PGS, {}, {0: 2.0},
     None, ELLIPTIC),
    ('sliding elliptic by PGS', 'sliding.xml', 1000, PGS, {},
     {0: 2.0, 7: 1.0}, None, ELLIPTIC),
]


def text(values):
    return ' '.join(repr(float(v)) for v in values)


def main():
    with open(os.path.join(HERE, 'mujoco_constraints.csv'), 'w',
              newline='') as f:
        out = csv.writer(f, lineterminator='\n')
        out.writerow(['case', 'step', 'qpos', 'qvel'])
        for name, file, steps, solver, qpos, qvel, *rest in CASES:
            ctrl = rest[0] if rest else None
            model = mujoco.MjModel.from_xml_path(os.path.join(MODELS, file))
            model.opt.solver = solver
            if len(rest) > 1:
                model.opt.cone = rest[1]
            data = mujoco.MjData(model)
            for address, value in qpos.items():
                data.qpos[address] = value
            for address, value in qvel.items():
                data.qvel[address] = value
            if ctrl:
                data.ctrl[:] = ctrl
            out.writerow([name, 0, text(data.qpos), text(data.qvel)])
            for step in range(1, steps + 1):
                mujoco.mj_step(model, data)
                out.writerow([name, step, text(data.qpos), text(data.qvel)])
            print(name, steps, 'steps,', data.ncon, 'contacts,', data.nefc,
                  'rows at the end')


if __name__ == '__main__':
    main()
