# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Measures pairs of boxes with Shapely, for collision_test.

Shapely (https://shapely.readthedocs.io, BSD-3-Clause) wraps GEOS, the
geometry engine behind PostGIS, and nuPlan checks its collisions with it.
For 2,000 pairs of boxes, a car's to a truck's size at any heading, placed
from overlapping to 10 m apart and a quarter of them a hair from touching,
this script writes shapely_boxes.csv: each pair, whether they intersect,
touching counted, and the distance between them.

  pip install shapely numpy
  python application/automotive/reference/shapely_boxes.py
"""

import math
import os

import numpy as np
import shapely
from shapely.geometry import Polygon


def corners(x, y, heading, length, width):
    c, s = math.cos(heading), math.sin(heading)
    return [(x + c * a - s * b, y + s * a + c * b)
            for a, b in ((length / 2, width / 2), (-length / 2, width / 2),
                         (-length / 2, -width / 2), (length / 2, -width / 2))]


def main():
    random = np.random.default_rng(5)
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, 'shapely_boxes.csv'), 'w') as out:
        out.write('ax,ay,ah,al,aw,bx,by,bh,bl,bw,intersects,distance\n')
        for i in range(2000):
            a = (0.0, 0.0, random.uniform(-math.pi, math.pi),
                 random.uniform(3.5, 12.0), random.uniform(1.6, 2.6))
            heading = random.uniform(-math.pi, math.pi)
            b_size = (random.uniform(3.5, 12.0), random.uniform(1.6, 2.6))
            direction = random.uniform(-math.pi, math.pi)
            reach = random.uniform(0.0, 16.0)
            b = (reach * math.cos(direction), reach * math.sin(direction),
                 heading) + b_size
            if i % 4 == 0:
                # Slide b until it just touches a, then off by a hair.
                pa = Polygon(corners(*a))
                gap = pa.distance(Polygon(corners(*b)))
                if gap > 0.0:
                    scale = (reach - gap - random.choice([-1e-9, 1e-9])) / reach
                    b = (b[0] * scale, b[1] * scale) + b[2:]
            pa, pb = Polygon(corners(*a)), Polygon(corners(*b))
            out.write('%s,%d,%.17g\n' % (','.join('%.17g' % v for v in a + b),
                                         pa.intersects(pb), pa.distance(pb)))
    print('shapely', shapely.__version__, 'GEOS', shapely.geos_version_string)


if __name__ == '__main__':
    main()
