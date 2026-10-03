# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Finds the paramPoly3 test roads' positions by exact arc length, for
opendrive_reference_test.

OpenDRIVE's s is arc length along the reference line, so a parametric cubic's
point at s is where its arc length from the start reaches s. libOpenDRIVE
finds that through a table of chords made to a 1 cm tolerance; this script
finds it by 20-point Gauss-Legendre quadrature over 400 parts and bisection,
to about 1e-12 m, independently of simon's code. It reads the samples from
libopendrive_positions.csv and writes the same rows for paramPoly3.xodr to
exact_positions.csv. The test roads lie flat, without elevation or
superelevation, so a point at t is t along the horizontal normal.

  pip install numpy
  python application/automotive/reference/exact_param_poly3.py
"""

import csv
import math
import os
import xml.etree.ElementTree as ElementTree

import numpy as np

NODES, WEIGHTS = np.polynomial.legendre.leggauss(20)


def integrate(f, a, b, parts=400):
    total = 0.0
    edges = np.linspace(a, b, parts + 1)
    for lo, hi in zip(edges[:-1], edges[1:]):
        mid, half = (lo + hi) / 2, (hi - lo) / 2
        total += half * sum(w * f(mid + half * x) for x, w in zip(NODES, WEIGHTS))
    return total


def road_curves(path):
    """Each road's single paramPoly3 piece, by road id."""
    curves = {}
    for road in ElementTree.parse(path).getroot().iter('road'):
        geometry = road.find('planView/geometry')
        poly = geometry.find('paramPoly3')
        number = lambda key: float(poly.get(key, '0'))
        curves[road.get('id')] = {
            'x': float(geometry.get('x')), 'y': float(geometry.get('y')),
            'hdg': float(geometry.get('hdg')), 'length': float(geometry.get('length')),
            'u': [number(k) for k in ('aU', 'bU', 'cU', 'dU')],
            'v': [number(k) for k in ('aV', 'bV', 'cV', 'dV')],
            'normalized': poly.get('pRange', 'normalized') == 'normalized',
        }
    return curves


def point_at(curve, s):
    """x, y and heading at arc length s."""
    u, v = curve['u'], curve['v']
    slope = lambda c, p: c[1] + 2 * c[2] * p + 3 * c[3] * p * p
    value = lambda c, p: c[0] + c[1] * p + c[2] * p * p + c[3] * p ** 3
    speed = lambda p: math.hypot(slope(u, p), slope(v, p))
    end = 1.0 if curve['normalized'] else curve['length']
    low, high = 0.0, end
    for _ in range(100):  # Bisection on the arc length.
        middle = 0.5 * (low + high)
        if integrate(speed, 0.0, middle) < s:
            low = middle
        else:
            high = middle
    p = 0.5 * (low + high)
    lu, lv = value(u, p), value(v, p)
    h = curve['hdg']
    heading = h + math.atan2(slope(v, p), slope(u, p))
    return (curve['x'] + lu * math.cos(h) - lv * math.sin(h),
            curve['y'] + lu * math.sin(h) + lv * math.cos(h), heading)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    curves = road_curves(os.path.join(here, '..', 'roads', 'paramPoly3.xodr'))
    with open(os.path.join(here, 'libopendrive_positions.csv')) as f:
        rows = [r for r in csv.DictReader(f) if r['file'] == 'paramPoly3.xodr']
    with open(os.path.join(here, 'exact_positions.csv'), 'w') as out:
        out.write('file,road,s,t,h,x,y,z\n')
        for row in rows:
            s, t, h = float(row['s']), float(row['t']), float(row['h'])
            x, y, heading = point_at(curves[row['road']], s)
            x -= t * math.sin(heading)
            y += t * math.cos(heading)
            out.write('paramPoly3.xodr,%s,%s,%s,%s,%.17g,%.17g,%.17g\n'
                      % (row['road'], row['s'], row['t'], row['h'], x, y, h))
    print('wrote', len(rows), 'rows')


if __name__ == '__main__':
    main()
