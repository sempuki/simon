# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Plays esmini's scenarios in esmini, for scenario_test and batch_test.

esmini (https://github.com/esmini/esmini, MPL-2.0), an OpenSCENARIO player,
plays each scenario in 3rd_party/esmini/xosc headless, at fixed steps of
0.05 s, logging every entity each step. This script writes:

  esmini_scenarios.csv     for each scenario, step and entity, its position
                           and heading in the world, its speed, and its road,
                           lane, s and offset from the lane's middle
  esmini_permutations.csv  the same for each of the 12 permutations of
                           cut-in_parameter_set.xosc, each entity's box, its
                           length, width and center ahead of its reference
                           point, in place of the road
  esmini_parameters.csv    the parameter values esmini gave each permutation

esmini 3.8.2, built from source without its viewer, OSI, SUMO or tests:

  cmake -S esmini -B build -DUSE_OSG=OFF -DUSE_OSI=OFF -DUSE_SUMO=OFF \\
        -DUSE_GTEST=OFF
  python application/automotive/reference/esmini_scenarios.py \\
         build/EnvironmentSimulator/Applications/esmini/esmini
"""

import csv
import glob
import os
import subprocess
import sys
import tempfile

SCENARIOS = ['cut-in_simple.xosc', 'cut-in.xosc', 'lane_change_simple.xosc',
             'traffic_lights.xosc']
DISTRIBUTION = 'cut-in_parameter_set.xosc'
PERMUTATIONS = 12
STEP = 0.05


def play(esmini, scenario, options=()):
    """Plays a scenario, returning its log's rows, each a step's time and
    its entities' columns by name, and esmini's own log's lines."""
    with tempfile.TemporaryDirectory() as directory:
        subprocess.run([esmini, '--headless', '--osc', scenario,
                        '--fixed_timestep', str(STEP),
                        '--csv_logger', 'log.csv', *options],
                       check=True, capture_output=True, cwd=directory)
        [log] = glob.glob(os.path.join(directory, 'log*.csv'))
        with open(log) as f:
            lines = f.read().splitlines()
        messages = []
        for text in glob.glob(os.path.join(directory, 'log*.txt')):
            with open(text) as f:
                messages += f.read().splitlines()
    # A header of several lines, then one row a step: for each entity, 31
    # columns from its name.
    steps = []
    for row in (line for line in lines if line[:1].isdigit()):
        cells = [c.strip() for c in row.split(',')]
        entities = {}
        for at in range(2, len(cells) - 30, 31):
            if cells[at]:
                entities[cells[at]] = cells[at:at + 31]
        steps.append((float(cells[1]), entities))
    return steps, messages


def main():
    esmini = sys.argv[1]
    here = os.path.dirname(os.path.abspath(__file__))
    scenarios = os.path.join(here, '..', '..', '..', '3rd_party', 'esmini',
                             'xosc')
    with open(os.path.join(here, 'esmini_scenarios.csv'), 'w', newline='') as out:
        out.write('scenario,time,entity,x,y,heading,speed,lane,s,offset\n')
        for name in SCENARIOS:
            steps, _ = play(esmini, os.path.join(scenarios, name),
                            ['--disable_log'])
            for time, entities in steps:
                for entity, c in entities.items():
                    out.write('%s,%.2f,%s,%.12g,%.12g,%.12g,%.12g,%d,%.12g,%.12g\n'
                              % (name, time, entity, float(c[11]), float(c[12]),
                                 float(c[24]), float(c[2]), int(float(c[22])),
                                 float(c[20]), float(c[23])))
            print(name, len(steps), 'steps')

    with open(os.path.join(here, 'esmini_permutations.csv'), 'w') as out, \
            open(os.path.join(here, 'esmini_parameters.csv'), 'w') as values:
        out.write('permutation,time,entity,x,y,heading,speed,length,width,center\n')
        values.write('permutation,name,value\n')
        for permutation in range(PERMUTATIONS):
            steps, messages = play(
                esmini, os.path.join(scenarios, DISTRIBUTION),
                ['--param_permutation', str(permutation)])
            # After "Parameter permutation k/n", a line "   name: value" each.
            at = next(i for i, m in enumerate(messages)
                      if 'Parameter permutation' in m) + 1
            while at < len(messages) and messages[at].startswith('[] [info]    '):
                name, value = messages[at][len('[] [info]    '):].split(': ')
                values.write('%d,%s,%s\n' % (permutation, name, value))
                at += 1
            for time, entities in steps:
                for entity, c in entities.items():
                    out.write('%d,%.2f,%s,%s\n' % (
                        permutation, time, entity, ','.join(
                            '%.12g' % float(c[i])
                            for i in (11, 12, 24, 2, 8, 9, 5))))
            print(DISTRIBUTION, permutation, len(steps), 'steps')


if __name__ == '__main__':
    main()
