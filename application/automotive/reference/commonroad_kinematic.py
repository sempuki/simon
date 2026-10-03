# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Records CommonRoad's kinematic single-track model, for single_track_test.

CommonRoad's vehicle models (https://commonroad.in.tum.de, BSD) define the
kinematic single-track model by its rates, vehicle_dynamics_ks, with the
steering and acceleration limits it applies to its inputs, for three vehicles
(Althoff and Wuersching, "CommonRoad: Vehicle Models", 2020): 1, a Ford
Escort; 2, a BMW 320i; 3, a VW Vanagon. This script writes:

  commonroad_parameters.csv  each vehicle's parameters the model reads
  commonroad_rates.csv       the rates at 400 states and inputs per vehicle,
                             a quarter of them past a limit
  commonroad_paths.csv       each vehicle driven 20 s through a steering sine
                             and acceleration steps held over 0.05 s each, by
                             classic Runge-Kutta 4, every 0.1 s, at steps of
                             0.05 s and 0.0005 s

  pip install commonroad-vehicle-models numpy
  python application/automotive/reference/commonroad_kinematic.py
"""

import math
import os

import numpy as np
from vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.parameters_vehicle3 import parameters_vehicle3
from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks

VEHICLES = {1: parameters_vehicle1(), 2: parameters_vehicle2(), 3: parameters_vehicle3()}
SAMPLE = 0.1  # s between recorded states.
END = 20.0


def inputs(t):
    """Steering rate and acceleration at time t: a steering sine, as a lane
    change asks, and acceleration steps past the engine's limit and the
    braking limit and back, each held over 0.05 s, so that every step sees the same inputs
    and the finest step converges to the exact path under them."""
    t = math.floor(t / 0.05 + 1e-9) * 0.05
    steering_rate = 0.05 * math.sin(0.8 * t)
    acceleration = 3.0 if t < 5.0 else -1.5 if t < 9.0 else 14.0 if t < 11.0 else -12.0 if t < 13.0 else 0.5
    return [steering_rate, acceleration]


def drive(p, dt):
    """Runge-Kutta 4 with the inputs at each step's start."""
    x = [0.0, 0.0, 0.0, 10.0, 0.3]
    rows = [(0.0, list(x))]
    steps = int(round(END / dt))
    every = int(round(SAMPLE / dt))
    f = lambda state, u: np.array(vehicle_dynamics_ks(list(state), u, p))
    for k in range(steps):
        u = inputs(k * dt)
        x0 = np.array(x)
        k1 = f(x0, u)
        k2 = f(x0 + 0.5 * dt * k1, u)
        k3 = f(x0 + 0.5 * dt * k2, u)
        k4 = f(x0 + dt * k3, u)
        x = list(x0 + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4))
        if (k + 1) % every == 0:
            rows.append(((k + 1) * dt, list(x)))
    return rows


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, 'commonroad_parameters.csv'), 'w') as out:
        out.write('vehicle,length,width,a,b,steering_min,steering_max,'
                  'steering_rate_min,steering_rate_max,a_max,v_switch,v_min,v_max\n')
        for vehicle, p in VEHICLES.items():
            out.write('%d,%s\n' % (vehicle, ','.join('%.17g' % v for v in (
                p.l, p.w, p.a, p.b, p.steering.min, p.steering.max,
                p.steering.v_min, p.steering.v_max, p.longitudinal.a_max,
                p.longitudinal.v_switch, p.longitudinal.v_min, p.longitudinal.v_max))))

    random = np.random.default_rng(7)
    with open(os.path.join(here, 'commonroad_rates.csv'), 'w') as out:
        out.write('vehicle,x,y,steering,v,heading,steering_rate,acceleration,'
                  'dx,dy,dsteering,dv,dheading\n')
        for vehicle, p in VEHICLES.items():
            for i in range(400):
                steering = random.uniform(-1.1, 1.1) * p.steering.max
                v = random.uniform(-20.0, 55.0)
                state = [random.uniform(-100, 100), random.uniform(-100, 100),
                         steering, v, random.uniform(-math.pi, math.pi)]
                u = [random.uniform(-0.6, 0.6), random.uniform(-14.0, 14.0)]
                if i % 4 == 0:  # On a limit, to take each branch.
                    edge = random.integers(4)
                    if edge == 0:
                        state[2] = p.steering.max
                    elif edge == 1:
                        state[2] = p.steering.min
                    elif edge == 2:
                        state[3] = p.longitudinal.v_max
                    else:
                        state[3] = p.longitudinal.v_min
                rates = vehicle_dynamics_ks(list(state), list(u), p)
                out.write('%d,%s\n' % (vehicle, ','.join(
                    '%.17g' % v for v in state + u + list(rates))))

    with open(os.path.join(here, 'commonroad_paths.csv'), 'w') as out:
        out.write('vehicle,step,time,x,y,steering,v,heading\n')
        for vehicle, p in VEHICLES.items():
            for dt in (0.05, 0.0005):
                for t, x in drive(p, dt):
                    out.write('%d,%.17g,%.17g,%s\n' % (
                        vehicle, dt, t, ','.join('%.17g' % v for v in x)))
            print('vehicle', vehicle, 'done')


if __name__ == '__main__':
    main()
