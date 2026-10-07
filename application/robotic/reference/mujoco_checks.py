# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Measures what robotic's tests hold simon to, beside MuJoCo's own runs.

MuJoCo (https://mujoco.org, Apache-2.0) replays every case of
mujoco_dynamics.py, mujoco_control.py, mujoco_constraints.py and
mujoco_menagerie.py, and this script writes:

  mujoco_local.csv      At up to 50 steps of each run: the state there, the
                        state one step later, stepped from fresh data so that
                        nothing but that state carries over, and how far that
                        step moves when its state is nudged by a few units in
                        the last place (its spread). A test steps simon once
                        from the same state.
  mujoco_spread.csv     For each case and step, how far rounding alone moves
                        MuJoCo's run by then: the largest difference so far
                        between its run and the same run nudged after every
                        step.
  mujoco_solved.csv     For each case with constraints, the run with every
                        step's constraints solved to convergence, by Newton's
                        method to a tolerance of 1e-15 in up to 1,000
                        iterations, at every fifth step. mujoco_local.csv
                        likewise gives each local step solved so. Where
                        simon's solver stops elsewhere than MuJoCo's, a test
                        holds it to being no farther from these than MuJoCo
                        is.
  mujoco_contact_spread.csv
                        For each pose of mujoco_contacts.csv and
                        mujoco_convex.csv, how far rounding moves MuJoCo's
                        contacts there: the largest difference in a distance,
                        position or frame between its contacts and those of
                        the pose nudged, or, for the convex collider, those
                        found with its tolerance halved, since where EPA
                        stops is rounding's to choose too; and whether
                        either changes which contacts there are.
  mujoco_converged.csv  For each case without constraints, the converged
                        solution at each step: Runge-Kutta 4 at a fiftieth of
                        the timestep. A test holds simon's error from it to
                        MuJoCo's own.

Positions and velocities are space-separated, to 17 significant digits.

MuJoCo 3.14.0:

  pip install mujoco numpy
  python application/robotic/reference/mujoco_checks.py
"""

import csv
import os

import mujoco
import numpy as np

import mujoco_constraints
import mujoco_control
import mujoco_dynamics
import mujoco_menagerie

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', 'models')
SAMPLES = 50
NUDGES = 4          # Nudged runs a case's run spread is the largest of.
ULPS = 4.0          # How far a nudge moves each number.
CONVERGED = 50      # Substeps a converged step takes.
STRIDE = 5          # Steps between the solved runs' recorded states.
SEED = 20261006


def text(values):
    return ' '.join(repr(float(v)) for v in values)


class Case:
    """A run: how to build its model and starting data, and its controls."""

    def __init__(self, name, steps, make_model, qpos=None, qvel=None,
                 ctrl=None, law=None, key=False):
        self.name = name
        self.steps = steps
        self.make_model = make_model
        self.qpos = qpos or {}
        self.qvel = qvel or {}
        self.ctrl = ctrl
        self.law = law
        self.key = key

    def start(self, model):
        data = mujoco.MjData(model)
        if self.key and model.nkey > 0:
            mujoco.mj_resetDataKeyframe(model, data, 0)
        for address, value in self.qpos.items():
            data.qpos[address] = value
        for address, value in self.qvel.items():
            data.qvel[address] = value
        if self.ctrl is not None:
            data.ctrl[:] = self.ctrl
        return data

    def step(self, model, data):
        if self.law is not None:
            data.ctrl[:] = mujoco_control.control(model, data, *self.law)
        mujoco.mj_step(model, data)


def unconstrained(file):
    def make():
        model = mujoco.MjModel.from_xml_path(os.path.join(MODELS, file))
        model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
        return model
    return make


def constrained(file, solver, cone=None, integrator=None):
    def make():
        path = file if os.path.isabs(file) else os.path.join(MODELS, file)
        model = mujoco.MjModel.from_xml_path(path)
        model.opt.solver = solver
        if cone is not None:
            model.opt.cone = cone
        if integrator is not None:
            model.opt.integrator = integrator
        return model
    return make


def robot(name):
    def make():
        return mujoco.MjModel.from_xml_path(
            os.path.join(mujoco_menagerie.MENAGERIE, name, 'scene.xml'))
    return make


def read_law():
    with open(os.path.join(HERE, 'mujoco_feedback.csv')) as f:
        row = list(csv.reader(f))[1]
    gains = [float(x) for x in row[1].split()]
    reference = [float(x) for x in row[2].split()]
    offset = [float(x) for x in row[3].split()]
    width = len(reference)
    return ([gains[i:i + width] for i in range(0, len(gains), width)],
            reference, offset)


def cases():
    found = []
    for name, file, steps, qpos, qvel, ctrl in mujoco_dynamics.CASES:
        found.append(('dynamics', Case(name, steps, unconstrained(file), qpos,
                                       qvel, ctrl or None)))
    found.append(('control', Case('arm', 1500, unconstrained('arm.xml'),
                                  {0: 0.3, 1: -0.5},
                                  ctrl=[0.8, -1.2, 2.0, 0.5])))
    found.append(('control', Case('balance', 1500,
                                  unconstrained('cartpole.xml'), {1: 0.2},
                                  ctrl=[0.0], law=read_law())))
    for name, file, steps, solver, qpos, qvel, *rest in \
            mujoco_constraints.CASES:
        ctrl = rest[0] if rest else None
        cone = rest[1] if len(rest) > 1 else None
        integrator = rest[2] if len(rest) > 2 else None
        found.append(('constraints', Case(
            name, steps, constrained(file, solver, cone, integrator), qpos,
            qvel, ctrl)))
    for name in mujoco_menagerie.ROBOTS:
        found.append(('menagerie', Case(name, mujoco_menagerie.STEPS,
                                        robot(name), key=True)))
    return found


def nudge(values, generator):
    """Each value moved by up to ULPS units in the last place, either way;
    zeros by as much as a value of a thousandth would be."""
    scale = (np.abs(values) + 1e-3) * np.finfo(float).eps * ULPS
    return values + scale * generator.uniform(-1.0, 1.0, len(values))


def run(case, model, nudged=None):
    """The case's states at every step, nudged after each by `nudged`, a
    random generator, if given."""
    data = case.start(model)
    states = [(data.qpos.copy(), data.qvel.copy())]
    for _ in range(case.steps):
        case.step(model, data)
        if nudged is not None:
            data.qpos[:] = nudge(data.qpos, nudged)
            data.qvel[:] = nudge(data.qvel, nudged)
        states.append((data.qpos.copy(), data.qvel.copy()))
    return states


def apart(a, b):
    return (max(np.max(np.abs(x[0] - y[0])) for x, y in zip(a, b)),
            max(np.max(np.abs(x[1] - y[1])) for x, y in zip(a, b)))


def step_from(case, model, qpos, qvel):
    """One step from fresh data at `qpos` and `qvel`."""
    data = case.start(model)
    data.qpos[:] = qpos
    data.qvel[:] = qvel
    case.step(model, data)
    return data.qpos.copy(), data.qvel.copy()


def solved(case):
    """The case's model with its constraints solved to convergence."""
    model = case.make_model()
    model.opt.solver = mujoco.mjtSolver.mjSOL_NEWTON
    model.opt.tolerance = 1e-15
    model.opt.iterations = 1000
    model.opt.ls_tolerance = 1e-15
    model.opt.ls_iterations = 1000
    return model


def converged(case, model, coarse):
    """Runge-Kutta 4 at a fiftieth of the timestep, at every coarse step."""
    fine = case.make_model()
    fine.opt.timestep = model.opt.timestep / CONVERGED
    fine.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4
    data = case.start(fine)
    states = [(data.qpos.copy(), data.qvel.copy())]
    for _ in range(case.steps):
        for _ in range(CONVERGED):
            case.step(fine, data)
        states.append((data.qpos.copy(), data.qvel.copy()))
    return states


def find_contacts(model, data, qpos):
    """The contacts at `qpos`: geoms, and distance, position and frame,
    sorted by geoms and then position, as collision_test sorts them."""
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    found = [((int(c.geom[0]), int(c.geom[1])),
              np.concatenate([[c.dist], c.pos, c.frame]))
             for c in data.contact[:data.ncon]]
    found.sort(key=lambda c: (c[0], tuple(c[1][1:4])))
    return found


def write_contact_spreads(out, table, model_file, generator):
    model = mujoco.MjModel.from_xml_path(os.path.join(MODELS, model_file))
    data = mujoco.MjData(model)
    # The same model, its convex collider stopping at half the tolerance.
    finer = mujoco.MjModel.from_xml_path(os.path.join(MODELS, model_file))
    finer.opt.ccd_tolerance = model.opt.ccd_tolerance / 2
    finer_data = mujoco.MjData(finer)
    with open(os.path.join(HERE, table)) as f:
        rows = [r for r in csv.reader(f)][1:]
    for pose, kind, values in rows:
        if kind != 'qpos':
            continue
        qpos = np.array([float(x) for x in values.split()])
        base = find_contacts(model, data, qpos)
        spread = 0.0
        recount = False
        others = [find_contacts(finer, finer_data, qpos)]
        others += [find_contacts(model, data, nudge(qpos, generator))
                   for _ in range(NUDGES)]
        for moved in others:
            if [c[0] for c in moved] != [c[0] for c in base]:
                recount = True
                continue
            for (_, a), (_, b) in zip(base, moved):
                spread = max(spread, float(np.max(np.abs(a - b))))
        out.writerow([table, pose, repr(spread), int(recount)])


def main():
    generator = np.random.default_rng(SEED)
    with open(os.path.join(HERE, 'mujoco_contact_spread.csv'), 'w',
              newline='') as f:
        out = csv.writer(f, lineterminator='\n')
        out.writerow(['table', 'pose', 'spread', 'recount'])
        write_contact_spreads(out, 'mujoco_contacts.csv', 'collisions.xml',
                              generator)
        write_contact_spreads(out, 'mujoco_convex.csv', 'convex.xml',
                              generator)
    local = open(os.path.join(HERE, 'mujoco_local.csv'), 'w', newline='')
    spread = open(os.path.join(HERE, 'mujoco_spread.csv'), 'w', newline='')
    smooth = open(os.path.join(HERE, 'mujoco_converged.csv'), 'w',
                  newline='')
    local_out = csv.writer(local, lineterminator='\n')
    spread_out = csv.writer(spread, lineterminator='\n')
    smooth_out = csv.writer(smooth, lineterminator='\n')
    exact = open(os.path.join(HERE, 'mujoco_solved.csv'), 'w', newline='')
    exact_out = csv.writer(exact, lineterminator='\n')
    exact_out.writerow(['case', 'step', 'qpos', 'qvel'])
    local_out.writerow(['case', 'step', 'qpos', 'qvel', 'next_qpos',
                        'next_qvel', 'spread_position', 'spread_velocity',
                        'solved_qpos', 'solved_qvel'])
    spread_out.writerow(['case', 'step', 'position', 'velocity'])
    smooth_out.writerow(['case', 'step', 'qpos', 'qvel'])
    for kind, case in cases():
        model = case.make_model()
        reference = run(case, model)
        with_constraints = kind in ('constraints', 'menagerie')
        exact_model = solved(case) if with_constraints else None
        if with_constraints:
            for step, (qpos, qvel) in enumerate(run(case, exact_model)):
                if step % STRIDE == 0:
                    exact_out.writerow([case.name, step, text(qpos),
                                        text(qvel)])

        # The envelope: at each step, the largest difference by then.
        envelope = [(0.0, 0.0)] * len(reference)
        for _ in range(NUDGES):
            nudged = run(case, model, generator)
            for k in range(len(reference)):
                envelope[k] = tuple(map(max, envelope[k],
                                        apart([reference[k]], [nudged[k]])))
        for k in range(1, len(envelope)):
            envelope[k] = tuple(map(max, envelope[k], envelope[k - 1]))
        for k, (position, velocity) in enumerate(envelope):
            spread_out.writerow([case.name, k, repr(float(position)),
                                 repr(float(velocity))])
        run_spread = envelope[-1]

        local_spread = (0.0, 0.0)
        samples = sorted({round(k * (case.steps - 1) / (SAMPLES - 1))
                          for k in range(SAMPLES)})
        for k in samples:
            qpos, qvel = reference[k]
            after = step_from(case, model, qpos, qvel)
            here = (0.0, 0.0)
            for _ in range(NUDGES):
                moved = step_from(case, model, nudge(qpos, generator),
                                  nudge(qvel, generator))
                here = tuple(map(max, here, apart([after], [moved])))
            settled = (step_from(case, exact_model, qpos, qvel)
                       if with_constraints else ([], []))
            local_out.writerow([case.name, k, text(qpos), text(qvel),
                                text(after[0]), text(after[1]),
                                repr(float(here[0])), repr(float(here[1])),
                                text(settled[0]), text(settled[1])])
            local_spread = tuple(map(max, local_spread, here))

        if kind in ('dynamics', 'control') and case.law is None:
            for step, (qpos, qvel) in enumerate(converged(case, model,
                                                          reference)):
                smooth_out.writerow([case.name, step, text(qpos),
                                     text(qvel)])
        print(f'{case.name}: run spread {run_spread[0]:.1e} '
              f'{run_spread[1]:.1e}, local spread {local_spread[0]:.1e} '
              f'{local_spread[1]:.1e}')
    local.close()
    exact.close()
    spread.close()
    smooth.close()


if __name__ == '__main__':
    main()
