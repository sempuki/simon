# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Compiles robotic's test models with MuJoCo, for model_test.

MuJoCo (https://mujoco.org, Apache-2.0) compiles each model in
application/robotic/models, and this script writes mujoco_models.csv: for
each model, element (option, body, jnt, dof, geom, actuator, qpos0,
qpos_spring), index
and field, the compiled values, space-separated, to 17 significant digits.

MuJoCo 3.14.0:

  pip install mujoco
  python application/robotic/reference/mujoco_models.py
"""

import csv
import glob
import os

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', 'models')

FIELDS = {
    'body': ['parentid', 'rootid', 'pos', 'quat', 'ipos', 'iquat', 'mass',
             'inertia', 'jntadr', 'jntnum', 'dofadr', 'dofnum', 'geomadr',
             'geomnum', 'sameframe'],
    'jnt': ['type', 'bodyid', 'pos', 'axis', 'limited', 'range', 'stiffness',
            'qposadr', 'dofadr', 'solref', 'solimp', 'margin'],
    'dof': ['bodyid', 'jntid', 'parentid', 'armature', 'damping',
            'frictionloss', 'solref', 'solimp'],
    'geom': ['type', 'bodyid', 'size', 'pos', 'quat', 'friction', 'condim',
             'contype', 'conaffinity', 'solref', 'solimp', 'margin', 'gap',
             'priority', 'sameframe'],
    'actuator': ['trntype', 'trnid', 'gear', 'ctrlrange', 'ctrllimited',
                 'forcerange', 'forcelimited', 'damping', 'dyntype',
                 'gaintype', 'biastype', 'gainprm', 'biasprm'],
}
SIZES = {'body': 'nbody', 'jnt': 'njnt', 'dof': 'nv', 'geom': 'ngeom',
         'actuator': 'nu'}


def text(values):
    values = list(values.flat) if hasattr(values, 'flat') else [values]
    return ' '.join(repr(float(v)) for v in values)


def main():
    with open(os.path.join(HERE, 'mujoco_models.csv'), 'w', newline='') as f:
        out = csv.writer(f, lineterminator='\n')
        out.writerow(['model', 'element', 'index', 'field', 'values'])
        for path in sorted(glob.glob(os.path.join(MODELS, '*.xml'))):
            name = os.path.basename(path)
            model = mujoco.MjModel.from_xml_path(path)
            option = model.opt
            for field in ['timestep', 'gravity', 'integrator', 'cone', 'solver',
                          'iterations', 'tolerance']:
                out.writerow([name, 'option', 0, field,
                              text(getattr(option, field))])
            out.writerow([name, 'qpos0', 0, 'values', text(model.qpos0)])
            out.writerow([name, 'qpos_spring', 0, 'values',
                          text(model.qpos_spring)])
            for element, fields in FIELDS.items():
                for index in range(getattr(model, SIZES[element])):
                    for field in fields:
                        values = getattr(model, element + '_' + field)[index]
                        out.writerow([name, element, index, field, text(values)])
            print(name, model.nbody, 'bodies')


if __name__ == '__main__':
    main()
