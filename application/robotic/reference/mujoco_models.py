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
import numpy

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', 'models')
THIRD_PARTY = os.path.join(HERE, '..', '..', '..', '3rd_party', 'mujoco')
# Menagerie's robots, with their meshes, as robotic's 3rd_party holds them
# without (git clone https://github.com/google-deepmind/mujoco_menagerie).
MENAGERIE = os.path.expanduser('~/.cache/simon-reference/mujoco_menagerie')
ROBOTS = ['unitree_go1', 'unitree_h1', 'universal_robots_ur5e',
          'anybotics_anymal_c']

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
    'tendon': ['limited', 'range', 'margin', 'frictionloss', 'solref_lim',
               'solimp_lim', 'solref_fri', 'solimp_fri'],
    'actuator': ['trntype', 'trnid', 'gear', 'ctrlrange', 'ctrllimited',
                 'forcerange', 'forcelimited', 'damping', 'dyntype',
                 'gaintype', 'biastype', 'gainprm', 'biasprm'],
}
SIZES = {'body': 'nbody', 'jnt': 'njnt', 'dof': 'nv', 'geom': 'ngeom',
         'tendon': 'ntendon', 'actuator': 'nu'}


def text(values):
    values = list(values.flat) if hasattr(values, 'flat') else [values]
    return ' '.join(repr(float(v)) for v in values)


def main():
    with open(os.path.join(HERE, 'mujoco_models.csv'), 'w', newline='') as f:
        out = csv.writer(f, lineterminator='\n')
        out.writerow(['model', 'element', 'index', 'field', 'values'])
        paths = sorted(glob.glob(os.path.join(MODELS, '*.xml')))
        paths.append(os.path.join(THIRD_PARTY, 'humanoid.xml'))
        paths += [os.path.join(MENAGERIE, robot, 'scene.xml') for robot in ROBOTS]
        for path in paths:
            name = os.path.basename(path)
            if path.startswith(MENAGERIE):
                name = os.path.relpath(path, MENAGERIE)
            model = mujoco.MjModel.from_xml_path(path)
            option = model.opt
            for field in ['timestep', 'gravity', 'integrator', 'cone', 'solver',
                          'iterations', 'tolerance', 'impratio']:
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
            for index in range(model.ntendon):
                adr = model.tendon_adr[index]
                num = model.tendon_num[index]
                out.writerow([name, 'tendon', index, 'joints',
                              text(model.wrap_objid[adr:adr + num])])
                out.writerow([name, 'tendon', index, 'coefficients',
                              text(model.wrap_prm[adr:adr + num])])
            for index in range(model.nexclude):
                signature = int(model.exclude_signature[index])
                out.writerow([name, 'exclude', index, 'bodies',
                              text(numpy.array([signature >> 16, signature & 0xFFFF]))])
            print(name, model.nbody, 'bodies')


if __name__ == '__main__':
    main()
