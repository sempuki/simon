# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Records CommonRoad's dynamic vehicle models and their tire, for
tire_test and dynamics_test.

CommonRoad's vehicle models (https://commonroad.in.tum.de, BSD) define three
models with tires (Althoff and Wuersching, "CommonRoad: Vehicle Models",
2020): the single-track model, vehicle_dynamics_st, with tires linear in slip;
the single-track drift model, vehicle_dynamics_std, with the Magic Formula
tire and wheel spin; and the multibody model, vehicle_dynamics_mb, with a
sprung body, two unsprung axles, four wheels and their suspension. Their tire
is a subset of Pacejka's Magic Formula 5.2 (Pacejka, Tire and Vehicle
Dynamics), in utils/tire_model.

simon corrects four slips. CommonRoad's longitudinal slip is
s = 1 - R omega / u, the negative of the Magic Formula's kappa, and the pure
longitudinal force negates it but the combined lateral force does not, which
turns the side force longitudinal slip induces the wrong way. The pure
longitudinal force adds its vertical shift inside the sine rather than after
it. And at a crawl, where each model drives as the kinematic model, the slip
angle beta = atan(tan(delta) b / l) changes at

  b / l delta' / (cos(delta)^2 (1 + (tan(delta) b / l)^2))

where CommonRoad squares tan(delta) a second time; the multibody model's
yaw rate then changes with the slip angle, which CommonRoad's multibody
model takes from its roll angle. This script records the tire as CommonRoad
has it and corrected, and the models corrected, their source patched where
they differ:

  commonroad_vehicles.csv      every parameter of vehicles 1, 2 and 3, by name
  commonroad_tires.csv         tire forces at 2,000 slips and loads
  commonroad_dynamic_rates.csv each model's rates at up to 300 states and
                               inputs per vehicle, leaving out those where
                               the multibody model divides by a wheel's ground
                               speed under 0.1 m/s or lifts a wheel
  commonroad_dynamic_paths.csv each model driven 8 s through a steering sine
                               and acceleration steps held over 0.01 s each,
                               by classic Runge-Kutta 4, every 0.1 s, at steps
                               of 0.001 s and 0.0001 s

  pip install commonroad-vehicle-models numpy
  python application/automotive/reference/commonroad_dynamic.py
"""

import dataclasses
import inspect
import math
import os

import numpy as np
from vehiclemodels.init_mb import init_mb
from vehiclemodels.init_std import init_std
from vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.parameters_vehicle3 import parameters_vehicle3
import vehiclemodels.utils.tire_model as tire_model
import vehiclemodels.vehicle_dynamics_mb
import vehiclemodels.vehicle_dynamics_st
import vehiclemodels.vehicle_dynamics_std


MULTIBODY_CRAWL_YAW = (
    '        dd_psi = 1 / lwb * (u[1] * math.cos(x[6]) * math.tan(x[2]) -\n'
    '                            x[3] * math.sin(x[6]) * d_beta * math.tan(x[2]) +\n'
    '                            x[3] * math.cos(x[6]) * u[0] / math.cos(x[2]) ** 2)\n')


def correct_crawl(module, name):
    """The model `name` from `module`, its crawling slip angle's rate with
    tan(delta) squared once, and for the multibody model its crawling yaw
    rate's change with the slip angle, where CommonRoad reads the roll
    angle."""
    source = inspect.getsource(getattr(module, name))
    wrong = '(1 + (math.tan(x[2]) ** 2 * p.b / lwb) ** 2)'
    right = '(1 + (math.tan(x[2]) * p.b / lwb) ** 2)'
    assert source.count(wrong) == 1
    source = source.replace(wrong, right)
    if name == 'vehicle_dynamics_mb':
        assert source.count(MULTIBODY_CRAWL_YAW) == 1
        source = source.replace(MULTIBODY_CRAWL_YAW, MULTIBODY_CRAWL_YAW.replace(
            'x[6]', 'beta'))
    namespace = dict(vars(module))
    exec(source, namespace)
    return namespace[name]


VEHICLES = {1: parameters_vehicle1(), 2: parameters_vehicle2(), 3: parameters_vehicle3()}
MODELS = {
    'st': correct_crawl(vehiclemodels.vehicle_dynamics_st, 'vehicle_dynamics_st'),
    'std': correct_crawl(vehiclemodels.vehicle_dynamics_std, 'vehicle_dynamics_std'),
    'mb': correct_crawl(vehiclemodels.vehicle_dynamics_mb, 'vehicle_dynamics_mb'),
}
SAMPLE = 0.1  # s between recorded states.
HOLD = 0.01  # s each input is held.
END = 8.0

ORIGINAL = {name: getattr(tire_model, name) for name in (
    'formula_longitudinal', 'formula_lateral', 'formula_longitudinal_comb',
    'formula_lateral_comb')}


def corrected_longitudinal(kappa, gamma, F_z, p):
    """formula_longitudinal with its vertical shift after the sine."""
    kappa = -kappa
    S_hx = p.p_hx1
    S_vx = F_z * p.p_vx1
    kappa_x = kappa + S_hx
    mu_x = p.p_dx1 * (1 - p.p_dx3 * gamma ** 2)
    C_x = p.p_cx1
    D_x = mu_x * F_z
    E_x = p.p_ex1
    K_x = F_z * p.p_kx1
    B_x = K_x / (C_x * D_x)
    return D_x * math.sin(C_x * math.atan(
        B_x * kappa_x - E_x * (B_x * kappa_x - math.atan(B_x * kappa_x)))) + S_vx


def corrected_lateral_comb(kappa, alpha, gamma, mu_y, F_z, F0_y, p):
    """formula_lateral_comb with CommonRoad's slip turned into kappa."""
    return ORIGINAL['formula_lateral_comb'](-kappa, alpha, gamma, mu_y, F_z, F0_y, p)


def use_tire(corrected):
    tire_model.formula_longitudinal = (
        corrected_longitudinal if corrected else ORIGINAL['formula_longitudinal'])
    tire_model.formula_lateral_comb = (
        corrected_lateral_comb if corrected else ORIGINAL['formula_lateral_comb'])


def tire_forces(s, alpha, gamma, F_z, p):
    """The combined forces at CommonRoad's slip s, as its models compute them."""
    F0_x = tire_model.formula_longitudinal(s, gamma, F_z, p)
    F0_y, mu_y = tire_model.formula_lateral(alpha, gamma, F_z, p)
    return (tire_model.formula_longitudinal_comb(s, alpha, F0_x, p),
            tire_model.formula_lateral_comb(s, alpha, gamma, mu_y, F_z, F0_y, p))


def parameters(p):
    """Every scalar parameter, by name: steering_ and longitudinal_ for those
    groups, tire_ for the tire's."""
    rows = []
    for field in dataclasses.fields(p):
        value = getattr(p, field.name)
        if isinstance(value, (int, float)):
            rows.append((field.name, float(value)))
    for group in ('steering', 'longitudinal', 'tire'):
        part = getattr(p, group)
        for field in dataclasses.fields(part):
            value = getattr(part, field.name)
            if isinstance(value, (int, float)):
                rows.append(('%s_%s' % (group, field.name), float(value)))
    return rows


def start(model, p, speed):
    """Straight ahead at `speed`, wheels rolling, suspension settled."""
    core = [0.0, 0.0, 0.0, speed, 0.0, 0.0, 0.0]
    if model == 'st':
        return core
    if model == 'std':
        return init_std(core, p)
    return init_mb(core, p)


def inputs(t):
    """Steering rate and acceleration at time t, held over 0.01 s: two
    periods of a steering sine of 0.04 rad, as a double lane change asks, and
    braking, a coast and speeding up."""
    t = math.floor(t / HOLD + 1e-9) * HOLD
    steering_rate = 0.1 * math.cos(2.5 * t) if t < 4.0 * math.pi / 2.5 else 0.0
    acceleration = -4.0 if 1.0 <= t < 2.5 else 0.0 if t < 5.0 else 2.0
    return [steering_rate, acceleration]


def drive(model, p, dt):
    """Runge-Kutta 4 with the inputs at each step's start."""
    x = np.array(start(model, p, 20.0))
    rows = [(0.0, list(x))]
    steps = int(round(END / dt))
    every = int(round(SAMPLE / dt))
    f = lambda state, u: np.array(MODELS[model](list(state), list(u), p))
    for k in range(steps):
        u = inputs(k * dt)
        k1 = f(x, u)
        k2 = f(x + 0.5 * dt * k1, u)
        k3 = f(x + 0.5 * dt * k2, u)
        k4 = f(x + dt * k3, u)
        x = x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        if (k + 1) % every == 0:
            rows.append(((k + 1) * dt, list(x)))
    return rows


def sample_state(model, p, random):
    """A state near a vehicle's motion: any heading and steering, speed from
    a crawl to fast, slip, yaw rate, and for the drift and multibody models
    wheels slipping and suspension displaced about rest."""
    speed = random.choice([random.uniform(0.0, 0.3), random.uniform(0.3, 45.0)])
    if random.uniform() < 0.1:
        speed = -speed
    core = [random.uniform(-100, 100), random.uniform(-100, 100),
            random.uniform(-0.3, 0.3), speed, random.uniform(-math.pi, math.pi),
            random.uniform(-0.8, 0.8), random.uniform(-0.15, 0.15)]
    if model == 'st':
        return core
    x = start(model, p, core[3])
    x[:7] = core if model == 'std' else x[:7]
    if model == 'std':
        for i in (7, 8):
            x[i] *= 1.0 + random.uniform(-0.3, 0.3)
            if random.uniform() < 0.05:
                x[i] = -random.uniform(0.0, 1.0)
        return x
    x[0], x[1], x[2], x[4], x[5] = core[0], core[1], core[2], core[4], core[5]
    x[10] = core[3] * math.tan(core[6])
    for i in (6, 8):  # The body's roll and pitch.
        x[i] = random.uniform(-0.05, 0.05)
    for i in (13, 18):  # The axles' roll.
        x[i] = random.uniform(-0.01, 0.01)
    for i in (7, 9, 14, 19):  # Their rates.
        x[i] = random.uniform(-0.3, 0.3)
    x[11] = random.uniform(-0.03, 0.03)
    x[12] = random.uniform(-0.2, 0.2)
    for i in (15, 20):  # Unsprung lateral velocities.
        x[i] = x[10] + random.uniform(-0.2, 0.2)
    for i in (16, 21):  # Tire compression about rest.
        x[i] *= 1.0 + random.uniform(-0.4, 0.4)
    for i in (17, 22):
        x[i] = random.uniform(-0.2, 0.2)
    for i in range(23, 27):
        x[i] *= 1.0 + random.uniform(-0.2, 0.2)
        if random.uniform() < 0.03:
            x[i] = -random.uniform(0.0, 1.0)
    for i in (27, 28):
        x[i] = random.uniform(-0.005, 0.005)
    return x


def lifted_wheel(x, p):
    """Whether a tire of the multibody model is off the ground, where
    CommonRoad's tire pulls with a negative load and simon's pushes
    nothing."""
    loads = [x[16] + p.R_w * (math.cos(x[13]) - 1) - 0.5 * p.T_f * math.sin(x[13]),
             x[16] + p.R_w * (math.cos(x[13]) - 1) + 0.5 * p.T_f * math.sin(x[13]),
             x[21] + p.R_w * (math.cos(x[18]) - 1) - 0.5 * p.T_r * math.sin(x[18]),
             x[21] + p.R_w * (math.cos(x[18]) - 1) + 0.5 * p.T_r * math.sin(x[18])]
    return min(loads) <= 0.0


def slow_wheel(x, p):
    """Whether the multibody model divides a slip by a wheel's ground speed
    under 0.1 m/s, where simon bounds it as the drift model does."""
    if abs(x[3]) < 0.1:
        return False
    front = (x[10] + p.a * x[5]) * math.sin(x[2])
    speeds = [(x[3] + 0.5 * p.T_f * x[5]) * math.cos(x[2]) + front,
              (x[3] - 0.5 * p.T_f * x[5]) * math.cos(x[2]) + front,
              x[3] + 0.5 * p.T_r * x[5], x[3] - 0.5 * p.T_r * x[5]]
    return min(speeds) < 0.1


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, 'commonroad_vehicles.csv'), 'w') as out:
        out.write('vehicle,name,value\n')
        for vehicle, p in VEHICLES.items():
            for name, value in parameters(p):
                out.write('%d,%s,%.17g\n' % (vehicle, name, value))

    random = np.random.default_rng(11)
    tire = VEHICLES[1].tire
    with open(os.path.join(here, 'commonroad_tires.csv'), 'w') as out:
        out.write('load,kappa,alpha,gamma,fx,fy,original_fx,original_fy\n')
        for i in range(2000):
            load = random.uniform(500.0, 8000.0)
            s = random.uniform(-1.0, 1.0) * (0.05 if i % 2 else 0.6)
            alpha = random.uniform(-1.0, 1.0) * (0.05 if i % 3 else 0.5)
            gamma = 0.0 if i % 4 else random.uniform(-0.1, 0.1)
            use_tire(True)
            fx, fy = tire_forces(s, alpha, gamma, load, tire)
            use_tire(False)
            ox, oy = tire_forces(s, alpha, gamma, load, tire)
            out.write('%s\n' % ','.join('%.17g' % v for v in (
                load, -s, alpha, gamma, fx, fy, ox, oy)))

    use_tire(True)
    skipped = {model: 0 for model in MODELS}
    with open(os.path.join(here, 'commonroad_dynamic_rates.csv'), 'w') as out:
        out.write('model,vehicle,steering_rate,acceleration,state...,rate...\n')
        for model, dynamics in MODELS.items():
            for vehicle, p in VEHICLES.items():
                for i in range(300):
                    x = sample_state(model, p, random)
                    u = [random.uniform(-0.6, 0.6), random.uniform(-14.0, 14.0)]
                    rates = dynamics(list(x), list(u), p)
                    if (not all(math.isfinite(v) for v in rates) or
                            model == 'mb' and (slow_wheel(x, p) or
                                               lifted_wheel(x, p))):
                        skipped[model] += 1
                        continue
                    out.write('%s,%d,%s\n' % (model, vehicle, ','.join(
                        '%.17g' % v for v in u + list(x) + list(rates))))

    print('states left out:', skipped)

    with open(os.path.join(here, 'commonroad_dynamic_paths.csv'), 'w') as out:
        out.write('model,vehicle,step,time,state...\n')
        for model in MODELS:
            for vehicle, p in VEHICLES.items():
                for dt in (0.001, 0.0001):
                    for t, x in drive(model, p, dt):
                        out.write('%s,%d,%.17g,%.17g,%s\n' % (
                            model, vehicle, dt, t, ','.join('%.17g' % v for v in x)))
                print(model, 'vehicle', vehicle, 'done', flush=True)


if __name__ == '__main__':
    main()
