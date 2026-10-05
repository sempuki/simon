# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Times SUMO driving the traffic automotive_benchmark drives, for the
comparison in application/automotive/Design.md.

SUMO (https://eclipse.dev/sumo, EPL-2.0) converts rings.xodr with netconvert,
with a speed limit of 30 m/s, and drives 1,000, 10,000 and 100,000 vehicles on it, each following by SUMO's
IDM (v0 = 30 m/s spread uniformly by 10%, T = 1 s, s0 = 2 m, a = 1 m/s^2,
b = 1.5 m/s^2, delta = 4) and changing lanes by SUMO's default LC2013. The
vehicles start at 25 m/s, spread at random over the lanes, and each loops its
ring. SUMO runs on one thread with 0.1 s steps, as simon does, and without
teleporting, output or TraCI. Each population runs twice, for 60 s and for
90 s, and the difference in SUMO's own reported duration is the time for 300
steps of settled traffic, with loading left out.

With --grid it converts grid.xodr instead, its sidewalks by SUMO's
OpenDRIVE type maps and with crossings and walking areas at every junction,
and gives each junction's lights simon's cycle: 30 s green, then 2 s and 3 s
of yellow, for each of its two groups. Each population is vehicles:
pedestrians. The vehicles follow by the same IDM at 50 km/h, spread
uniformly by 10%, each on a random walk through the grid as simon's drivers
take random turns, starting at 10 m/s spread at random over the lanes. Each
pedestrian walks from a random place on a sidewalk to a random sidewalk,
by SUMO's default striping model, at Weidmann's walking speeds.

  pip install eclipse-sumo
  python application/automotive/reference/sumo_benchmark.py [vehicles...]
  python application/automotive/reference/sumo_benchmark.py --grid \
      [vehicles:pedestrians...]
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
GRID = os.path.join(HERE, '..', 'roads', 'grid.xodr')
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


def convert_grid(directory):
    """grid.xodr as a SUMO network, its lights on simon's cycle."""
    net = os.path.join(directory, 'grid.net.xml')
    types = os.path.join(sumo.SUMO_HOME, 'data', 'typemap')
    subprocess.run([binary('netconvert'), '--opendrive-files', GRID,
                    '--type-files',
                    os.path.join(types, 'opendriveNetconvert.typ.xml') + ',' +
                    os.path.join(types, 'opendriveNetconvertPedestrians.typ.xml'),
                    '--opendrive.signal-groups', 'true',
                    '--crossings.guess', 'true', '--walkingareas', 'true',
                    '-o', net],
                   check=True, capture_output=True,
                   env=dict(os.environ, SUMO_HOME=sumo.SUMO_HOME))
    with open(net) as f:
        text = f.read()

    def retime(match):
        state = match.group(2)
        seconds = 3 if 'y' in state else 30 if int(match.group(1)) >= 10 else 2
        return '<phase duration="%d" state="%s"' % (seconds, state)

    text = re.sub(r'<phase duration="(\d+)"\s+state="([^"]*)"', retime, text)
    with open(net, 'w') as f:
        f.write(text)
    return net


def grid_routes(net, vehicles, pedestrians, path):
    """Vehicles on random walks and pedestrians walking to random places."""
    rng = random.Random(1)
    roads = [e for e in net.getEdges()
             if e.getFunction() == '' and e.allows('passenger')]
    walks = [e for e in net.getEdges()
             if e.getFunction() == '' and e.allows('pedestrian')]

    def onward(edge):
        return [e for e in edge.getOutgoing() if e.allows('passenger')]

    taken = {}
    with open(path, 'w') as f:
        f.write('<routes>\n'
                '  <vType id="idm" carFollowModel="IDM" length="4.5" minGap="2" '
                'accel="1" decel="1.5" emergencyDecel="9" tau="1" delta="4" '
                'laneChangeModel="LC2013"/>\n'
                '  <vType id="walker" vClass="pedestrian" '
                'speedFactor="normc(1,0.194,0.373,1.866)"/>\n')
        weights = [e.getLength() for e in roads]
        for n in range(vehicles):
            while True:
                edge = rng.choices(roads, weights)[0]
                lane = next(i for i, l in enumerate(edge.getLanes())
                            if l.allows('passenger'))
                pos = rng.uniform(0.0, edge.getLength())
                others = taken.setdefault(edge.getID(), [])
                if all(abs(o - pos) >= 11.5 for o in others):
                    others.append(pos)
                    break
            way = [edge]
            while len(way) < 300:
                choices = onward(way[-1])
                if not choices:
                    break
                way.append(rng.choice(choices))
            f.write('  <vehicle id="v%d" type="idm" depart="0" departLane="%d" '
                    'departPos="%.2f" departSpeed="10" speedFactor="%.4f" '
                    'insertionChecks="none"><route edges="%s"/></vehicle>\n'
                    % (n, lane, pos, rng.uniform(0.9, 1.1),
                       ' '.join(e.getID() for e in way)))
        weights = [e.getLength() for e in walks]
        for n in range(pedestrians):
            start = rng.choices(walks, weights)[0]
            end = rng.choice(walks)
            f.write('  <person id="p%d" type="walker" depart="0" '
                    'departPos="%.2f"><walk from="%s" to="%s"/></person>\n'
                    % (n, rng.uniform(0.0, start.getLength()), start.getID(),
                       end.getID()))
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


def mean_speed(net, routes_path, end, directory):
    """The running vehicles' mean speed at `end`, from a run of its own with
    SUMO's summary output, so that the timed runs write nothing."""
    summary = os.path.join(directory, 'summary.xml')
    subprocess.run(
        [binary('sumo'), '-n', net, '-r', routes_path, '--step-length', str(STEP),
         '--end', str(end), '--time-to-teleport', '-1', '--no-step-log', 'true',
         '--no-warnings', 'true', '--collision.action', 'warn', '--threads', '1',
         '--summary-output', summary],
        check=True, capture_output=True)
    with open(summary) as f:
        speeds = re.findall(r'meanSpeed="([-\d.]+)"', f.read())
    return float(speeds[-1])


def main_grid(populations):
    with tempfile.TemporaryDirectory(dir=os.path.expanduser('~/.cache')) as directory:
        net = convert_grid(directory)
        network = sumolib.net.readNet(net)
        for vehicles, pedestrians in populations:
            path = os.path.join(directory, 'routes.rou.xml')
            grid_routes(network, vehicles, pedestrians, path)
            settled, _ = duration(net, path, SETTLING)
            whole, running = duration(net, path, SETTLING + MEASURED)
            steps = MEASURED / STEP
            seconds = whole - settled
            speed = mean_speed(net, path, SETTLING + MEASURED, directory)
            print('%d vehicles and %d pedestrians (%d vehicles running): '
                  '%.3f ms/step %.1f ns/entity-step, vehicles ending at %.1f m/s'
                  % (vehicles, pedestrians, running, 1e3 * seconds / steps,
                     1e9 * seconds / (steps * (vehicles + pedestrians)), speed))


def main():
    if sys.argv[1:2] == ['--grid']:
        populations = [tuple(int(n) for n in a.split(':')) for a in sys.argv[2:]]
        main_grid(populations or [(1000, 1000), (10000, 10000), (10000, 100000)])
        return
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
