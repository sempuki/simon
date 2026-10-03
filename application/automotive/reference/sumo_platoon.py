# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Drives a platoon of Intelligent Driver Model followers in SUMO, for
traffic_test.

SUMO (https://eclipse.dev/sumo, EPL-2.0) drives five followers behind a
leader on one straight lane. The leader's speed is set each step by a
schedule: 25 m/s until 10 s, braking at 1 m/s^2 to 10 m/s, holding it until
40 s, and speeding up at 0.8 m/s^2 back to 25 m/s, for 80 s in all. The
followers start 40 m apart at 25 m/s and follow by SUMO's IDM, with
v0 = 33.33 m/s, T = 1 s, s0 = 2 m, a = 1 m/s^2, b = 1.5 m/s^2, delta = 4.
SUMO runs twice: at its usual 0.1 s step, and at 0.001 s with its IDM taking
one iteration a step, where its Euler update has converged to the model's
equations. Time counts from the step in which SUMO inserts the platoon. Writes
sumo_platoon.csv: every vehicle's front bumper position and speed every
0.1 s, at both steps.

  pip install eclipse-sumo traci
  python application/automotive/reference/sumo_platoon.py
"""

import os
import subprocess
import tempfile

import sumo
import traci

LENGTH = 5.0
FOLLOWERS = 5
END = 80.0
SAMPLE = 0.1


def leader_speed(t):
    if t < 10.0:
        return 25.0
    if t < 25.0:
        return max(10.0, 25.0 - (t - 10.0))
    if t < 40.0:
        return 10.0
    return min(25.0, 10.0 + 0.8 * (t - 40.0))


def network(directory):
    with open(os.path.join(directory, 'road.nod.xml'), 'w') as f:
        f.write('<nodes><node id="a" x="0" y="0"/><node id="b" x="6000" y="0"/></nodes>\n')
    with open(os.path.join(directory, 'road.edg.xml'), 'w') as f:
        f.write('<edges><edge id="road" from="a" to="b" numLanes="1" speed="60"/></edges>\n')
    net = os.path.join(directory, 'road.net.xml')
    subprocess.run([os.path.join(sumo.SUMO_HOME, 'bin', 'netconvert'),
                    '--node-files', os.path.join(directory, 'road.nod.xml'),
                    '--edge-files', os.path.join(directory, 'road.edg.xml'),
                    '--output-file', net], check=True, capture_output=True)
    with open(os.path.join(directory, 'types.rou.xml'), 'w') as f:
        f.write('<routes>\n'
                '  <vType id="leader" length="%g" minGap="2" accel="10" decel="10" '
                'emergencyDecel="10" maxSpeed="60" speedFactor="1" speedDev="0" sigma="0"/>\n'
                '  <vType id="idm" carFollowModel="IDM" length="%g" minGap="2" accel="1" '
                'decel="1.5" emergencyDecel="9" tau="1" delta="4" stepping="%s" '
                'maxSpeed="33.33" speedFactor="1" speedDev="0"/>\n'
                '  <route id="along" edges="road"/>\n'
                '</routes>\n' % (LENGTH, LENGTH, '{stepping}'))
    return net


def run(net, directory, step):
    routes = os.path.join(directory, 'routes.rou.xml')
    with open(os.path.join(directory, 'types.rou.xml')) as f:
        text = f.read().replace('{stepping}', '%g' % (0.25 if step >= 0.01 else step))
    with open(routes, 'w') as f:
        f.write(text)
    traci.start([os.path.join(sumo.SUMO_HOME, 'bin', 'sumo'), '-n', net, '-r', routes,
                 '--step-length', '%g' % step, '--no-step-log', 'true',
                 '--collision.action', 'warn', '--no-warnings', 'true'])
    names = ['leader'] + ['follower%d' % i for i in range(FOLLOWERS)]
    for i, name in enumerate(names):
        traci.vehicle.add(name, 'along', typeID='leader' if i == 0 else 'idm',
                          depart='0', departPos='%g' % (300.0 - 45.0 * i),
                          departSpeed='25', departLane='0')
    traci.vehicle.setSpeedMode('leader', 0)
    # SUMO inserts vehicles during a step; time counts from their insertion.
    traci.simulationStep()
    rows = []
    steps = int(round(END / step))
    every = int(round(SAMPLE / step))
    for k in range(steps + 1):
        if k % every == 0:
            for i, name in enumerate(names):
                rows.append((step, k * step, i, traci.vehicle.getLanePosition(name),
                             traci.vehicle.getSpeed(name)))
        traci.vehicle.setSpeed('leader', leader_speed((k + 1) * step))
        traci.simulationStep()
    traci.close()
    return rows


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    with tempfile.TemporaryDirectory() as directory:
        net = network(directory)
        rows = run(net, directory, 0.1) + run(net, directory, 0.001)
    with open(os.path.join(here, 'sumo_platoon.csv'), 'w') as out:
        out.write('step,time,vehicle,position,speed\n')
        for step, t, i, x, v in rows:
            out.write('%g,%.17g,%d,%.17g,%.17g\n' % (step, t, i, x, v))
    print('wrote', len(rows), 'rows')


if __name__ == '__main__':
    main()
