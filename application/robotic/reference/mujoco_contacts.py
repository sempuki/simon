# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Finds the contacts of robotic's collision models in MuJoCo, for
collision_test.

MuJoCo (https://mujoco.org, Apache-2.0) runs its collision detection on
models/collisions.xml at 400 poses drawn at random (seed 1): every loose
body and the arm's base anywhere in a 0.7 m by 0.7 m by 0.65 m box, turned
at random, the arm's joints within a radian. This script writes
mujoco_contacts.csv: a row of positions for each pose, then a row for each
contact MuJoCo finds there: its geoms, distance, position, frame, dimension,
friction, solref, solimp, margin it includes, and whether the gap excludes
it, to 17 significant digits. It does the same for models/convex.xml, the
pairs MuJoCo's convex collider takes, into mujoco_convex.csv.

MuJoCo 3.14.0:

  pip install mujoco
  python application/robotic/reference/mujoco_contacts.py
"""

import csv
import os

import mujoco
import numpy

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', 'models')
POSES = 400


def text(values):
    return ' '.join(repr(float(v)) for v in values)


def main():
    write('collisions.xml', 'mujoco_contacts.csv')
    write('convex.xml', 'mujoco_convex.csv')


def write(model_file, table):
    model = mujoco.MjModel.from_xml_path(os.path.join(MODELS, model_file))
    data = mujoco.MjData(model)
    rng = numpy.random.default_rng(1)
    with open(os.path.join(HERE, table), 'w', newline='') as f:
        out = csv.writer(f, lineterminator='\n')
        out.writerow(['pose', 'kind', 'values'])
        total = 0
        for pose in range(POSES):
            for j in range(model.njnt):
                address = model.jnt_qposadr[j]
                if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                    data.qpos[address:address + 3] = rng.uniform(
                        [-0.35, -0.35, -0.05], [0.35, 0.35, 0.6])
                    quat = rng.normal(size=4)
                    data.qpos[address + 3:address + 7] = quat / numpy.linalg.norm(
                        quat)
                else:
                    data.qpos[address] = rng.uniform(-1.0, 1.0)
            mujoco.mj_forward(model, data)
            out.writerow([pose, 'qpos', text(data.qpos)])
            for c in data.contact[:data.ncon]:
                out.writerow([
                    pose, 'contact',
                    text([c.geom[0], c.geom[1], c.dist, *c.pos, *c.frame,
                          c.dim, *c.friction, *c.solref, *c.solimp,
                          c.includemargin, c.exclude])
                ])
            total += data.ncon
        print(model_file, POSES, 'poses,', total, 'contacts')


if __name__ == '__main__':
    main()
