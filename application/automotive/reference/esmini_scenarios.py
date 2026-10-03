# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Plays esmini's scenarios in esmini, for scenario_test.

esmini (https://github.com/esmini/esmini, MPL-2.0), an OpenSCENARIO player,
plays each scenario in 3rd_party/esmini/xosc headless, at fixed steps of 0.05 s,
logging every entity each step. This script writes esmini_scenarios.csv:
for each scenario, step and entity, its position and heading in the world,
its speed, and its road, lane, s and offset from the lane's middle.

esmini 3.8.2, built from source without its viewer, OSI, SUMO or tests:

  cmake -S esmini -B build -DUSE_OSG=OFF -DUSE_OSI=OFF -DUSE_SUMO=OFF \\
        -DUSE_GTEST=OFF
  python application/automotive/reference/esmini_scenarios.py \\
         build/EnvironmentSimulator/Applications/esmini/esmini
"""

import csv
import os
import subprocess
import sys
import tempfile

SCENARIOS = ['cut-in_simple.xosc', 'cut-in.xosc', 'lane_change_simple.xosc']
STEP = 0.05


def main():
    esmini = sys.argv[1]
    here = os.path.dirname(os.path.abspath(__file__))
    scenarios = os.path.join(here, '..', '..', '..', '3rd_party', 'esmini',
                             'xosc')
    with open(os.path.join(here, 'esmini_scenarios.csv'), 'w', newline='') as out:
        out.write('scenario,time,entity,x,y,heading,speed,lane,s,offset\n')
        for name in SCENARIOS:
            with tempfile.TemporaryDirectory() as directory:
                log = os.path.join(directory, 'log.csv')
                subprocess.run([esmini, '--headless', '--osc',
                                os.path.join(scenarios, name),
                                '--fixed_timestep', str(STEP),
                                '--csv_logger', log, '--disable_log'],
                               check=True, capture_output=True, cwd=directory)
                with open(log) as f:
                    lines = f.read().splitlines()
                # A header of several lines, then one row a step: for each
                # entity, 31 columns from its name.
                rows = [line for line in lines if line[:1].isdigit()]
                for row in rows:
                    cells = [c.strip() for c in row.split(',')]
                    time = float(cells[1])
                    for at in range(2, len(cells) - 30, 31):
                        entity = cells[at]
                        if not entity:
                            continue
                        x, y = float(cells[at + 11]), float(cells[at + 12])
                        speed = float(cells[at + 2])
                        s = float(cells[at + 20])
                        lane = int(float(cells[at + 22]))
                        offset = float(cells[at + 23])
                        heading = float(cells[at + 24])
                        out.write('%s,%.2f,%s,%.12g,%.12g,%.12g,%.12g,%d,%.12g,%.12g\n'
                                  % (name, time, entity, x, y, heading, speed,
                                     lane, s, offset))
            print(name, len(rows), 'steps')


if __name__ == '__main__':
    main()
