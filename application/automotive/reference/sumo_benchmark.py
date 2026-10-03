# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Times SUMO driving the traffic automotive_benchmark drives, for the
comparison in design.md.

SUMO (https://eclipse.dev/sumo, EPL-2.0) converts rings.xodr with netconvert,
with a speed limit of 30 m/s, and drives 1,000, 10,000 and 100,000 vehicles on it, each following by SUMO's
IDM (v0 = 30 m/s spread uniformly by 10%, T = 1 s, s0 = 2 m, a = 1 m/s^2,
b = 1.5 m/s^2, delta = 4) and changing lanes by SUMO's default LC2013. The
vehicles start at 25 m/s, spread at random over the lanes, and each loops its
ring. SUMO runs on one thread with 0.1 s steps, as simon does, and without
teleporting, output or TraCI. Each population runs twice, for 60 s and for
90 s, and the difference in SUMO's own reported duration is the time for 300
steps of settled traffic, with loading left out.

  pip install eclipse-sumo
  python application/automotive/reference/sumo_benchmark.py [vehicles...]
"""

import os
import random
import re
import subprocess
import sys
import tempfile

import sumo
import sumolib

HERE = os.path.dirname(os.path.abspath(__file__))
ROADS = os.path.join(HERE, '..', 'roads', 'rings.xodr')
SETTLING = 60.0
MEASURED = 30.0
STEP = 0.1


def binary(name):
    return os.path.join(sumo.SUMO_HOME, 'bin', name)


def loops(net):
    """Each edge's loop around its ring, starting from the edge."""
    found = {}
    for edge in net.getEdges():
        if edge.getFunction() == 'internal' or edge.getID() in found:
            continue
        cycle = [edge]
        while True:
            (onward,) = cycle[-1].getOutgoing()
            if onward == edge:
                break
            cycle.append(onward)
        for i, e in enumerate(cycle):
            found[e.getID()] = [c.getID() for c in cycle[i:] + cycle[:i]]
    return found


def routes(net, vehicles, path):
    rng = random.Random(1)
    cycles = loops(net)
    edges = [net.getEdge(e) for e in sorted(cycles)]
    lanes = [(e, i) for e in edges for i in range(e.getLaneNumber())]
    weights = [e.getLanes()[i].getLength() for e, i in lanes]
    taken = {}
    with open(path, 'w') as f:
        f.write('<routes>\n'
                '  <vType id="idm" carFollowModel="IDM" length="4.5" minGap="2" '
                'accel="1" decel="1.5" emergencyDecel="9" tau="1" delta="4" '
                'maxSpeed="60" '
                'laneChangeModel="LC2013"/>\n')
        for e in edges:
            f.write('  <route id="r%s" edges="%s"/>\n'
                    % (e.getID(), ' '.join(cycles[e.getID()] * 3)))
        for n in range(vehicles):
            while True:
                edge, lane = rng.choices(lanes, weights)[0]
                pos = rng.uniform(0.0, edge.getLanes()[lane].getLength())
                others = taken.setdefault((edge.getID(), lane), [])
                if all(abs(o - pos) >= 11.5 for o in others):
                    others.append(pos)
                    break
            f.write('  <vehicle id="v%d" type="idm" route="r%s" depart="0" '
                    'departLane="%d" departPos="%.2f" departSpeed="25" '
                    'speedFactor="%.4f" insertionChecks="none"/>\n'
                    % (n, edge.getID(), lane, pos, rng.uniform(0.9, 1.1)))
        f.write('</routes>\n')


def duration(net, routes_path, end):
    result = subprocess.run(
        [binary('sumo'), '-n', net, '-r', routes_path, '--step-length', str(STEP),
         '--end', str(end), '--time-to-teleport', '-1', '--no-step-log', 'true',
         '--no-warnings', 'true', '--duration-log.statistics', 'true',
         '--collision.action', 'warn', '--threads', '1'],
        check=True, capture_output=True, text=True)
    text = result.stdout + result.stderr
    running = re.search(r'Running: (\d+)', text)
    return (float(re.search(r'Duration: ([\d.]+)\s*s', text).group(1)),
            int(running.group(1)) if running else -1)


def main():
    populations = [int(a) for a in sys.argv[1:]] or [1000, 10000, 100000]
    with tempfile.TemporaryDirectory() as directory:
        net = os.path.join(directory, 'rings.net.xml')
        subprocess.run([binary('netconvert'), '--opendrive-files', ROADS,
                        '--speed.minimum', '30', '-o', net],
                       check=True, capture_output=True)
        network = sumolib.net.readNet(net)
        for vehicles in populations:
            path = os.path.join(directory, 'routes.rou.xml')
            routes(network, vehicles, path)
            settled, _ = duration(net, path, SETTLING)
            whole, running = duration(net, path, SETTLING + MEASURED)
            steps = MEASURED / STEP
            seconds = whole - settled
            print('%d vehicles (%d running): %.3f ms/step %.1f ns/vehicle-step'
                  % (vehicles, running, 1e3 * seconds / steps,
                     1e9 * seconds / (steps * vehicles)))


if __name__ == '__main__':
    main()
