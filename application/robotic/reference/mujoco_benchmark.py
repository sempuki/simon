# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Times MuJoCo stepping a scene on one thread, for comparison with
robotic_benchmark: steps it for a second of simulated time and prints the
wall time per step and per tree.

  python application/robotic/reference/mujoco_benchmark.py SCENE.xml [steps]
"""

import sys
import time

import mujoco


def main():
    model = mujoco.MjModel.from_xml_path(sys.argv[1])
    steps = int(sys.argv[2]) if len(sys.argv) > 2 else 500
    data = mujoco.MjData(model)
    trees = model.ntree
    mujoco.mj_step(model, data)
    start = time.perf_counter()
    for _ in range(steps):
        mujoco.mj_step(model, data)
    seconds = time.perf_counter() - start
    print(f'{sys.argv[1]}: {trees} trees, {seconds / steps * 1e3:.3f} ms/step,'
          f' {seconds / steps / trees * 1e9:.0f} ns/tree-step,'
          f' {data.ncon} contacts, {data.nefc} rows at the end')


if __name__ == '__main__':
    main()
