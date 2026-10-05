# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Writes the scenes robotic_benchmark and mujoco_benchmark.py time.

bodies_N.xml drops N loose primitives (boxes, spheres and capsules in turn)
onto the floor from a grid 0.5 m apart, each its own tree; humanoids_N.xml
stands N copies of MuJoCo's humanoid on a grid 2 m apart, each with its
tendons, exclusions and motors renamed.

  python application/robotic/reference/make_scenes.py OUTPUT_DIR [N ...]
"""

import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HUMANOID = os.path.join(HERE, '..', '..', '..', '3rd_party', 'mujoco',
                        'humanoid.xml')


def bodies(n):
    side = math.ceil(math.sqrt(n))
    lines = ['<mujoco model="bodies">', '  <size memory="2G"/>',
             '  <worldbody>',
             '    <geom name="floor" type="plane" size="0 0 1"/>']
    shapes = ['type="box" size="0.1 0.08 0.06"', 'type="sphere" size="0.08"',
              'type="capsule" size="0.05 0.1"']
    for k in range(n):
        x = (k % side - side / 2) * 0.5
        y = (k // side - side / 2) * 0.5
        z = 0.2 + 0.1 * (k % 3)
        lines.append(f'    <body pos="{x:.3f} {y:.3f} {z:.3f}" '
                     f'euler="{7 * k % 90} {11 * k % 90} 0">'
                     f'<freejoint/><geom {shapes[k % 3]}/></body>')
    lines += ['  </worldbody>', '</mujoco>']
    return '\n'.join(lines) + '\n'


def humanoids(n):
    text = open(HUMANOID).read()
    text = re.sub(r'<!--.*?-->', '', text, flags=re.S)
    head = text[:text.index('<worldbody>')]
    head = re.sub(r'<visual>.*?</visual>|<asset>.*?</asset>|'
                  r'<statistic[^>]*/>', '', head, flags=re.S)
    head = re.sub(r' material="body"', '', head)
    world = text[text.index('<worldbody>') + len('<worldbody>'):
                 text.index('</worldbody>')]
    world = re.sub(r'<light[^>]*/>|<camera[^>]*/>', '', world)
    floor = re.search(r'<geom name="floor"[^>]*/>', world).group(0)
    torso = world[world.index('<body name="torso"'):]
    contact = re.search(r'<contact>(.*?)</contact>', text, re.S).group(1)
    tendon = re.search(r'<tendon>(.*?)</tendon>', text, re.S).group(1)
    actuator = re.search(r'<actuator>(.*?)</actuator>', text, re.S).group(1)
    names = r'(name|joint|body1|body2)="([^"]+)"'
    side = math.ceil(math.sqrt(n))
    bodies, contacts, tendons, actuators = [], [], [], []
    for k in range(n):
        rename = lambda m: f'{m.group(1)}="{m.group(2)}_{k}"'
        x = (k % side - side / 2) * 2.0
        y = (k // side - side / 2) * 2.0
        body = re.sub(names, rename, torso)
        body = body.replace('pos="0 0 1.282"', f'pos="{x} {y} 1.282"', 1)
        bodies.append(body)
        contacts.append(re.sub(names, rename, contact))
        tendons.append(re.sub(names, rename, tendon))
        actuators.append(re.sub(names, rename, actuator))
    floor = floor.replace(' material="grid"', '')
    head = head.replace('<option', '<size memory="2G"/><option', 1)
    return (head + '<worldbody>' + floor + ''.join(bodies) + '</worldbody>' +
            '<contact>' + ''.join(contacts) + '</contact>' +
            '<tendon>' + ''.join(tendons) + '</tendon>' +
            '<actuator>' + ''.join(actuators) + '</actuator></mujoco>\n')


def main():
    out = sys.argv[1]
    counts = [int(c) for c in sys.argv[2:]] or [1000, 10000]
    os.makedirs(out, exist_ok=True)
    for n in counts:
        with open(os.path.join(out, f'bodies_{n}.xml'), 'w') as f:
            f.write(bodies(n))
        with open(os.path.join(out, f'humanoids_{n // 10}.xml'), 'w') as f:
            f.write(humanoids(n // 10))
        print('wrote', n, 'bodies and', n // 10, 'humanoids')


if __name__ == '__main__':
    main()
