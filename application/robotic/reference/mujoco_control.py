# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Steps robotic's controlled cases in MuJoCo without constraints, for
control_test.

MuJoCo (https://mujoco.org, Apache-2.0) steps an arm under position,
velocity and general actuators held at constant controls, and a cart-pole
balanced from 0.2 rad by a linear state feedback: the discrete linear
quadratic regulator of MuJoCo's own linearization about the upright pole
(mjd_transitionFD), found by iterating the Riccati equation, applied each
step before the step from the state as it stands, in the order simon's
Control system sums it. This script writes:

  mujoco_control.csv   for each case and step, positions, velocities and
                       controls after it, space-separated, to 17
                       significant digits; step 0 is the start
  mujoco_feedback.csv  the cart-pole's gains, row by row, reference state
                       and offset

MuJoCo 3.14.0 and numpy:

  pip install mujoco numpy
  python application/robotic/reference/mujoco_control.py
"""

import csv
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', 'models')


def text(values):
    return ' '.join(repr(float(v)) for v in values)


def regulator(model):
    """The discrete LQR gains about the model's rest state."""
    data = mujoco.MjData(model)
    nv, nu = model.nv, model.nu
    a = np.zeros((2 * nv, 2 * nv))
    b = np.zeros((2 * nv, nu))
    mujoco.mjd_transitionFD(model, data, 1e-6, True, a, b, None, None)
    q = np.diag([1.0, 10.0, 1.0, 1.0])
    r = np.diag([0.1])
    p = q.copy()
    for _ in range(5000):
        k = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)
        p = q + a.T @ p @ (a - b @ k)
    return k


def control(model, data, gains, reference, offset):
    """u = u0 - K (x - x0), summed as simon sums it."""
    nq = model.nq
    out = []
    for row in range(model.nu):
        u = offset[row]
        for i in range(nq):
            u -= gains[row][i] * (data.qpos[i] - reference[i])
        for i in range(model.nv):
            u -= gains[row][nq + i] * (data.qvel[i] - reference[nq + i])
        out.append(u)
    return out


def run(out, name, file, steps, qpos, ctrl, law=None):
    model = mujoco.MjModel.from_xml_path(os.path.join(MODELS, file))
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
    data = mujoco.MjData(model)
    for address, value in qpos.items():
        data.qpos[address] = value
    data.ctrl[:] = ctrl
    out.writerow([name, 0, text(data.qpos), text(data.qvel), text(data.ctrl)])
    for step in range(1, steps + 1):
        if law is not None:
            data.ctrl[:] = control(model, data, *law)
        mujoco.mj_step(model, data)
        out.writerow([name, step, text(data.qpos), text(data.qvel),
                      text(data.ctrl)])
    print(name, steps, 'steps', 'ending at', data.qpos)


def main():
    cartpole = mujoco.MjModel.from_xml_path(os.path.join(MODELS, 'cartpole.xml'))
    cartpole.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
    gains = regulator(cartpole)
    reference = [0.0] * (cartpole.nq + cartpole.nv)
    offset = [0.0] * cartpole.nu
    with open(os.path.join(HERE, 'mujoco_feedback.csv'), 'w', newline='') as f:
        out = csv.writer(f, lineterminator='\n')
        out.writerow(['case', 'gains', 'reference', 'offset'])
        out.writerow(['balance', text(gains.flat), text(reference),
                      text(offset)])
    with open(os.path.join(HERE, 'mujoco_control.csv'), 'w', newline='') as f:
        out = csv.writer(f, lineterminator='\n')
        out.writerow(['case', 'step', 'qpos', 'qvel', 'ctrl'])
        run(out, 'arm', 'arm.xml', 1500, {0: 0.3, 1: -0.5},
            [0.8, -1.2, 2.0, 0.5])
        run(out, 'balance', 'cartpole.xml', 1500, {1: 0.2}, [0.0],
            (gains.tolist(), reference, offset))
    print('gains', gains)


if __name__ == '__main__':
    main()
