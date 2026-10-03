# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Writes the test roads in application/automotive/roads.

Each road chains its reference line's pieces, each starting where the last
ends, which this script finds by integrating each piece's tangent with
Gauss-Legendre quadrature, independently of simon's code. A paramPoly3's length
is its arc length, found the same way, since OpenDRIVE's s is arc length.

curves.xodr is a line, a spiral into a left arc, a spiral through straight into
a right arc and a spiral out, with an elevation, a superelevation, a lane
offset, and lane widths that change along the road, over three lane sections,
the second opening a right lane. paramPoly3.xodr has two parametric cubic roads,
one over arcLength, whose arc length over its range is its length, and one
normalized, at map coordinates, 500 km east and 5,400 km north.

  pip install numpy
  python application/automotive/reference/make_test_roads.py
"""

import math
import os

import numpy as np

NODES, WEIGHTS = np.polynomial.legendre.leggauss(20)


def integrate(f, a, b, parts=200):
    """The integral of f over [a, b], by 20-point Gauss-Legendre."""
    total = 0.0
    edges = np.linspace(a, b, parts + 1)
    for lo, hi in zip(edges[:-1], edges[1:]):
        mid, half = (lo + hi) / 2, (hi - lo) / 2
        total += half * sum(w * f(mid + half * x) for x, w in zip(NODES, WEIGHTS))
    return total


def end_of(piece):
    """Where a piece ends: x, y and heading."""
    x0, y0, hdg, length, kind = (piece[k] for k in ('x', 'y', 'hdg', 'length', 'kind'))
    if kind == 'line':
        k0 = k1 = 0.0
    elif kind == 'arc':
        k0 = k1 = piece['curvature']
    elif kind == 'spiral':
        k0, k1 = piece['curvStart'], piece['curvEnd']
    else:
        raise ValueError(kind)
    rate = (k1 - k0) / length
    heading = lambda u: hdg + k0 * u + 0.5 * rate * u * u
    dx = integrate(lambda u: math.cos(heading(u)), 0.0, length)
    dy = integrate(lambda u: math.sin(heading(u)), 0.0, length)
    return x0 + dx, y0 + dy, heading(length)


def chain(start, pieces):
    """The pieces with s, x, y and hdg filled in, each starting where the last
    ends."""
    x, y, hdg = start
    s = 0.0
    out = []
    for piece in pieces:
        piece = dict(piece, s=s, x=x, y=y, hdg=hdg)
        out.append(piece)
        x, y, hdg = end_of(piece)
        s += piece['length']
    return out, s


def geometry_xml(piece):
    head = '<geometry s="%.17g" x="%.17g" y="%.17g" hdg="%.17g" length="%.17g">' % (
        piece['s'], piece['x'], piece['y'], piece['hdg'], piece['length'])
    kind = piece['kind']
    if kind == 'line':
        body = '<line/>'
    elif kind == 'arc':
        body = '<arc curvature="%.17g"/>' % piece['curvature']
    elif kind == 'spiral':
        body = '<spiral curvStart="%.17g" curvEnd="%.17g"/>' % (piece['curvStart'], piece['curvEnd'])
    else:
        body = piece['xml']
    return '        %s\n          %s\n        </geometry>\n' % (head, body)


def lane_xml(lane_id, kind, widths):
    rows = ''.join('<width sOffset="%g" a="%.17g" b="%.17g" c="%.17g" d="%.17g"/>' % w
                   for w in widths)
    return '<lane id="%d" type="%s">%s</lane>' % (lane_id, kind, rows)


def opening(width, length):
    """A cubic that opens a lane from 0 to `width` over `length`, flat at both
    ends."""
    return (0.0, 0.0, 3.0 * width / length ** 2, -2.0 * width / length ** 3)


def curves():
    pieces, length = chain((-20.0, 15.0, 0.3), [
        {'kind': 'line', 'length': 60.0},
        {'kind': 'spiral', 'length': 50.0, 'curvStart': 0.0, 'curvEnd': 1 / 150},
        {'kind': 'arc', 'length': 70.0, 'curvature': 1 / 150},
        {'kind': 'spiral', 'length': 80.0, 'curvStart': 1 / 150, 'curvEnd': -1 / 200},
        {'kind': 'arc', 'length': 60.0, 'curvature': -1 / 200},
        {'kind': 'spiral', 'length': 40.0, 'curvStart': -1 / 200, 'curvEnd': 0.0},
        {'kind': 'line', 'length': 30.0},
    ])
    plan = ''.join(geometry_xml(p) for p in pieces)
    sections = [
        (0.0, [lane_xml(1, 'driving', [(0, 3.5, 0, 0, 0)]),
               lane_xml(2, 'shoulder', [(0, 1.0, 0.004, 0, 0), (100, 1.4, 0, 0, 0)])],
              [lane_xml(-1, 'driving', [(0, 3.5, 0, 0, 0)]),
               lane_xml(-2, 'sidewalk', [(0, 2.0, 0, 0, 0)])]),
        (120.0, [lane_xml(1, 'driving', [(0, 3.5, 0, 0, 0)]),
                 lane_xml(2, 'shoulder', [(0, 1.4, 0, 0, 0)])],
                [lane_xml(-1, 'driving', [(0, 3.5, 0, 0, 0)]),
                 lane_xml(-2, 'driving', [(0,) + opening(3.25, 60.0)[:1] + opening(3.25, 60.0)[1:],
                                          (60, 3.25, 0, 0, 0)]),
                 lane_xml(-3, 'sidewalk', [(0, 2.0, 0, 0, 0)])]),
        (300.0, [lane_xml(1, 'driving', [(0, 3.5, 0, 0, 0)])],
                [lane_xml(-1, 'driving', [(0, 3.5, 0.002, -1e-5, 2e-8)]),
                 lane_xml(-2, 'driving', [(0, 3.25, 0, 0, 0)])]),
    ]
    lanes = ''.join(
        '      <laneSection s="%g">\n        <left>%s</left>\n'
        '        <center><lane id="0" type="none"/></center>\n'
        '        <right>%s</right>\n      </laneSection>\n'
        % (s, ''.join(left), ''.join(right)) for s, left, right in sections)
    return ('  <road name="curves" id="1" length="%.17g" junction="-1">\n'
            '    <planView>\n%s    </planView>\n'
            '    <elevationProfile>\n'
            '      <elevation s="0" a="12" b="0.02" c="0" d="0"/>\n'
            '      <elevation s="150" a="15" b="0.02" c="-1e-4" d="2e-7"/>\n'
            '    </elevationProfile>\n'
            '    <lateralProfile>\n'
            '      <superelevation s="0" a="0" b="0" c="0" d="0"/>\n'
            '      <superelevation s="110" a="0" b="0.0005" c="0" d="0"/>\n'
            '      <superelevation s="250" a="0.07" b="-0.0007" c="0" d="0"/>\n'
            '    </lateralProfile>\n'
            '    <lanes>\n'
            '      <laneOffset s="0" a="0.2" b="0" c="0" d="0"/>\n'
            '      <laneOffset s="200" a="0.2" b="0" c="1e-4" d="-1e-6"/>\n'
            '%s    </lanes>\n  </road>\n') % (length, plan, lanes)


def param_poly3_piece(s, x, y, hdg, u, v, normalized):
    """A paramPoly3 piece and its arc length: u and v are coefficients in p."""
    end = 1.0 if normalized else None
    def speed(p):
        du = u[1] + 2 * u[2] * p + 3 * u[3] * p * p
        dv = v[1] + 2 * v[2] * p + 3 * v[3] * p * p
        return math.hypot(du, dv)
    if normalized:
        length = integrate(speed, 0.0, 1.0)
    else:
        length = integrate(speed, 0.0, ARC_LENGTH_RANGE)
    xml = ('<paramPoly3 aU="%.17g" bU="%.17g" cU="%.17g" dU="%.17g" '
           'aV="%.17g" bV="%.17g" cV="%.17g" dV="%.17g" pRange="%s"/>'
           % (tuple(u) + tuple(v) + ('normalized' if normalized else 'arcLength',)))
    return {'kind': 'paramPoly3', 's': s, 'x': x, 'y': y, 'hdg': hdg,
            'length': length, 'xml': xml}


# The range of p over an arcLength paramPoly3: the test road scales its u so
# that its arc length over [0, 60] is 60 m, as pRange arcLength intends.
ARC_LENGTH_RANGE = 60.0


def arc_length_u(v):
    """The u = b p + d p^3 whose curve with v has arc length 60 m over
    [0, 60], by bisection on b."""
    def length(b):
        u = (0.0, b, 0.0, -2e-6)
        speed = lambda p: math.hypot(u[1] + 3 * u[3] * p * p,
                                     v[1] + 2 * v[2] * p + 3 * v[3] * p * p)
        return integrate(speed, 0.0, ARC_LENGTH_RANGE)
    low, high = 0.5, 1.0
    for _ in range(200):
        middle = 0.5 * (low + high)
        if length(middle) < ARC_LENGTH_RANGE:
            low = middle
        else:
            high = middle
    return (0.0, 0.5 * (low + high), 0.0, -2e-6)


def poly_road(road_id, start, u, v, normalized):
    x, y, hdg = start
    piece = param_poly3_piece(0.0, x, y, hdg, u, v, normalized)
    lanes = ('      <laneSection s="0">\n'
             '        <left>%s</left>\n'
             '        <center><lane id="0" type="none"/></center>\n'
             '        <right>%s</right>\n      </laneSection>\n'
             % (lane_xml(1, 'driving', [(0, 3.5, 0, 0, 0)]),
                lane_xml(-1, 'driving', [(0, 3.5, 0, 0, 0)])))
    return ('  <road name="poly %s" id="%s" length="%.17g" junction="-1">\n'
            '    <planView>\n%s    </planView>\n'
            '    <lanes>\n%s    </lanes>\n  </road>\n'
            % (road_id, road_id, piece['length'], geometry_xml(piece), lanes))


def document(roads):
    return ('<?xml version="1.0" encoding="UTF-8"?>\n<OpenDRIVE>\n'
            '  <header revMajor="1" revMinor="6" name="simon test" version="1"/>\n'
            '%s</OpenDRIVE>\n' % ''.join(roads))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    roads = os.path.join(here, '..', 'roads')
    with open(os.path.join(roads, 'curves.xodr'), 'w') as f:
        f.write(document([curves()]))
    with open(os.path.join(roads, 'paramPoly3.xodr'), 'w') as f:
        f.write(document([
            poly_road('2', (5.0, -3.0, 0.1), arc_length_u((0.0, 0.0, 0.004, -3e-5)),
                      (0.0, 0.0, 0.004, -3e-5), False),
            poly_road('3', (512000.25, 5400000.75, 2.6), (0.0, 80.0, -6.0, 1.5), (0.0, 0.0, 9.0, -4.0), True),
        ]))
    print('wrote', roads)


if __name__ == '__main__':
    main()
