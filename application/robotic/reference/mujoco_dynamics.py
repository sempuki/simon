# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Steps robotic's test models in MuJoCo without constraints, for
dynamics_test.

MuJoCo (https://mujoco.org, Apache-2.0) steps each case below from its
starting state, its contacts and limits turned off, and this script writes
mujoco_dynamics.csv: for each case and step, the positions and velocities
after it, space-separated, to 17 significant digits; step 0 is the start.

MuJoCo 3.14.0:

  pip install mujoco
  python application/robotic/reference/mujoco_dynamics.py
"""

import csv
import os

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', 'models')

# Model, steps, starting positions and velocities to set by address, and
# controls.
CASES = [
    ('pendulum', 'pendulum.xml', 2000, {0: 0.7}, {}, []),
    ('double pendulum', 'double_pendulum.xml', 3000, {0: 1.2, 1: -0.4}, {},
     []),
    ('free body', 'free_body.xml', 2000, {},
     {0: 0.3, 1: -0.1, 2: 0.2, 3: 4.0, 4: 0.5, 5: 1.5}, []),
    ('features', 'features.xml', 1000,
     {0: 0.1, 1: 0.9, 2: 0.2, 3: -0.3, 4: 0.25, 5: 0.3, 6: 0.5},
     {2: 1.0}, []),
    ('cartpole', 'cartpole.xml', 200, {1: 0.05}, {}, [0.3]),
    ('boxes', 'boxes.xml', 500, {}, {7: 0.5, 23: -1.0}, []),
]


def text(values):
    return ' '.join(repr(float(v)) for v in values)


def main():
    with open(os.path.join(HERE, 'mujoco_dynamics.csv'), 'w', newline='') as f:
        out = csv.writer(f, lineterminator='\n')
        out.writerow(['case', 'step', 'qpos', 'qvel'])
        for name, file, steps, qpos, qvel, ctrl in CASES:
            model = mujoco.MjModel.from_xml_path(os.path.join(MODELS, file))
            model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
            data = mujoco.MjData(model)
            for address, value in qpos.items():
                data.qpos[address] = value
            for address, value in qvel.items():
                data.qvel[address] = value
            data.ctrl[:] = ctrl
            out.writerow([name, 0, text(data.qpos), text(data.qvel)])
            for step in range(1, steps + 1):
                mujoco.mj_step(model, data)
                out.writerow([name, step, text(data.qpos), text(data.qvel)])
            print(name, steps, 'steps')


if __name__ == '__main__':
    main()
