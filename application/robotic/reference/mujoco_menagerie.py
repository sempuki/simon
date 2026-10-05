# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Steps four MuJoCo Menagerie robots in MuJoCo, for menagerie_test.

MuJoCo (https://mujoco.org, Apache-2.0) steps each robot's scene from its
home keyframe, if it has one, its actuators held at the keyframe's
controls, for 1000 steps, and this script writes mujoco_menagerie.csv: for
each robot a start row of positions and controls, then for each step the
positions and velocities after it, space-separated, to 17 significant
digits; step 0 is the start.

The robots are read with their meshes from a clone of Menagerie at
4d038b3 (git clone https://github.com/google-deepmind/mujoco_menagerie
~/.cache/simon-reference/mujoco_menagerie); robotic's 3rd_party/menagerie
holds their MJCF without the meshes, which only show them.

MuJoCo 3.14.0:

  pip install mujoco
  python application/robotic/reference/mujoco_menagerie.py
"""

import csv
import os

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
MENAGERIE = os.path.expanduser('~/.cache/simon-reference/mujoco_menagerie')
ROBOTS = ['unitree_go1', 'unitree_h1', 'universal_robots_ur5e',
          'anybotics_anymal_c']
STEPS = 1000


def text(values):
    return ' '.join(repr(float(v)) for v in values)


def main():
    with open(os.path.join(HERE, 'mujoco_menagerie.csv'), 'w',
              newline='') as f:
        out = csv.writer(f, lineterminator='\n')
        out.writerow(['robot', 'step', 'qpos', 'qvel'])
        for robot in ROBOTS:
            model = mujoco.MjModel.from_xml_path(
                os.path.join(MENAGERIE, robot, 'scene.xml'))
            data = mujoco.MjData(model)
            if model.nkey > 0:
                mujoco.mj_resetDataKeyframe(model, data, 0)
            out.writerow([robot, 'start', text(data.qpos), text(data.ctrl)])
            out.writerow([robot, 0, text(data.qpos), text(data.qvel)])
            for step in range(1, STEPS + 1):
                mujoco.mj_step(model, data)
                out.writerow([robot, step, text(data.qpos), text(data.qvel)])
            print(robot, STEPS, 'steps,', data.ncon, 'contacts,', data.nefc,
                  'rows at the end, height', data.qpos[2])


if __name__ == '__main__':
    main()
