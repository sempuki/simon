# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Drives a queue of Intelligent Driver Model vehicles to a red light and
away on green in SUMO, for signal_test.

SUMO (https://eclipse.dev/sumo, EPL-2.0) drives ten vehicles on one straight
lane, 1 km, with a traffic light at 500 m, as ../roads/light.xodr has it. The
light is red for 30 s and then green. The vehicles start at 400 m and every
35 m behind, at 15 m/s, and follow by SUMO's IDM, with v0 = 20 m/s, T = 1 s,
s0 = 2 m, a = 1 m/s^2, b = 1.5 m/s^2, delta = 4, 4.5 m long. SUMO runs at
0.001 s with its IDM taking one iteration a step, where its Euler update has
converged to the model's equations. Time counts from the step in which SUMO
inserts the vehicles. Writes sumo_signal.csv: every vehicle's front bumper
position along the road and its speed every 0.1 s.

  pip install eclipse-sumo traci
  python application/automotive/reference/sumo_signal.py
"""

import os
import subprocess
import tempfile

import sumo
import traci

VEHICLES = 10
END = 55.0
SAMPLE = 0.1
STEP = 0.001


def network(directory):
    with open(os.path.join(directory, 'road.nod.xml'), 'w') as f:
        f.write('<nodes><node id="a" x="0" y="0"/>'
                '<node id="b" x="500" y="0" type="traffic_light"/>'
                '<node id="c" x="1000" y="0"/></nodes>\n')
    with open(os.path.join(directory, 'road.edg.xml'), 'w') as f:
        f.write('<edges><edge id="in" from="a" to="b" numLanes="1" speed="60"/>'
                '<edge id="out" from="b" to="c" numLanes="1" speed="60"/></edges>\n')
    with open(os.path.join(directory, 'road.tll.xml'), 'w') as f:
        f.write('<tlLogics><tlLogic id="b" type="static" programID="fixed" offset="0">'
                '<phase duration="30" state="r"/><phase duration="1000" state="G"/>'
                '</tlLogic></tlLogics>\n')
    net = os.path.join(directory, 'road.net.xml')
    subprocess.run([os.path.join(sumo.SUMO_HOME, 'bin', 'netconvert'),
                    '--node-files', os.path.join(directory, 'road.nod.xml'),
                    '--edge-files', os.path.join(directory, 'road.edg.xml'),
                    '--tllogic-files', os.path.join(directory, 'road.tll.xml'),
                    '--no-internal-links', 'true',
                    '--output-file', net], check=True, capture_output=True)
    routes = os.path.join(directory, 'routes.rou.xml')
    with open(routes, 'w') as f:
        f.write('<routes>\n'
                '  <vType id="idm" carFollowModel="IDM" length="4.5" minGap="2" accel="1" '
                'decel="1.5" emergencyDecel="9" tau="1" delta="4" stepping="%g" '
                'maxSpeed="20" speedFactor="1" speedDev="0"/>\n'
                '  <route id="along" edges="in out"/>\n'
                '</routes>\n' % STEP)
    return net, routes


def run(net, routes):
    traci.start([os.path.join(sumo.SUMO_HOME, 'bin', 'sumo'), '-n', net, '-r', routes,
                 '--step-length', '%g' % STEP, '--no-step-log', 'true',
                 '--no-warnings', 'true'])
    names = ['vehicle%d' % i for i in range(VEHICLES)]
    for i, name in enumerate(names):
        traci.vehicle.add(name, 'along', typeID='idm', depart='0',
                          departPos='%g' % (400.0 - 35.0 * i), departSpeed='15',
                          departLane='0')
    # SUMO inserts vehicles during a step; time counts from their insertion.
    traci.simulationStep()
    rows = []
    every = int(round(SAMPLE / STEP))
    for k in range(int(round(END / STEP)) + 1):
        if k % every == 0:
            for i, name in enumerate(names):
                edge = traci.vehicle.getRoadID(name)
                position = traci.vehicle.getLanePosition(name)
                if edge == 'out':
                    position += 500.0
                rows.append((k * STEP, i, position, traci.vehicle.getSpeed(name)))
        traci.simulationStep()
    traci.close()
    return rows


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    with tempfile.TemporaryDirectory() as directory:
        rows = run(*network(directory))
    with open(os.path.join(here, 'sumo_signal.csv'), 'w') as out:
        out.write('time,vehicle,position,speed\n')
        for t, i, x, v in rows:
            out.write('%.17g,%d,%.17g,%.17g\n' % (t, i, x, v))
    print('wrote', len(rows), 'rows')


if __name__ == '__main__':
    main()
