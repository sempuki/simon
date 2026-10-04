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
normalized, at map coordinates, 500 km east and 5,400 km north. ring.xodr is
two half circles leading into each other, two lanes each way, for traffic.
rings.xodr is 100 such rings of 1 km radius, three lanes each way, for the
scale benchmark. signalized.xodr is a four-way junction with a traffic light
and a crosswalk on each approach and two controllers; priority.xodr a T whose
minor road gives way, with junction priorities and signs; crosswalks.xodr a
climbing, leaning curve with a crosswalk in its own frame and one in road
coordinates; light.xodr one lane with a traffic light halfway; crossing.xodr a
one-way major road crossed by a one-way minor road that gives way; and
crossroads.xodr four arms meeting with nothing to say who goes first.

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


def lane_xml(lane_id, kind, widths, predecessor=None, successor=None):
    rows = ''.join('<width sOffset="%g" a="%.17g" b="%.17g" c="%.17g" d="%.17g"/>' % w
                   for w in widths)
    links = ''.join('<%s id="%d"/>' % (name, linked)
                    for name, linked in (('predecessor', predecessor), ('successor', successor))
                    if linked is not None)
    link = '<link>%s</link>' % links if links else ''
    return '<lane id="%d" type="%s">%s%s</lane>' % (lane_id, kind, link, rows)


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
        (0.0, [lane_xml(1, 'driving', [(0, 3.5, 0, 0, 0)], successor=1),
               lane_xml(2, 'shoulder', [(0, 1.0, 0.004, 0, 0), (100, 1.4, 0, 0, 0)], successor=2)],
              [lane_xml(-1, 'driving', [(0, 3.5, 0, 0, 0)], successor=-1),
               lane_xml(-2, 'sidewalk', [(0, 2.0, 0, 0, 0)], successor=-3)]),
        (120.0, [lane_xml(1, 'driving', [(0, 3.5, 0, 0, 0)], 1, 1),
                 lane_xml(2, 'shoulder', [(0, 1.4, 0, 0, 0)], predecessor=2)],
                [lane_xml(-1, 'driving', [(0, 3.5, 0, 0, 0)], -1, -1),
                 lane_xml(-2, 'driving', [(0,) + opening(3.25, 60.0)[:1] + opening(3.25, 60.0)[1:],
                                          (60, 3.25, 0, 0, 0)], successor=-2),
                 lane_xml(-3, 'sidewalk', [(0, 2.0, 0, 0, 0)], predecessor=-2)]),
        (300.0, [lane_xml(1, 'driving', [(0, 3.5, 0, 0, 0)], predecessor=1)],
                [lane_xml(-1, 'driving', [(0, 3.5, 0.002, -1e-5, 2e-8)], predecessor=-1),
                 lane_xml(-2, 'driving', [(0, 3.25, 0, 0, 0)], predecessor=-2)]),
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


def ring(first_id=10, center=(0.0, 0.0), radius=100.0, lanes_each_way=2):
    """Two half circles of `radius` about `center`, each road leading into
    the other at both ends, with lanes each way linked across."""
    def half(road_id, other, start):
        x, y, hdg = start
        piece = {'kind': 'arc', 's': 0.0, 'x': x, 'y': y, 'hdg': hdg,
                 'length': math.pi * radius, 'curvature': 1.0 / radius}
        side = lambda sign: ''.join(
            lane_xml(sign * i, 'driving', [(0, 3.5, 0, 0, 0)], sign * i, sign * i)
            for i in range(1, lanes_each_way + 1))
        lanes = ('      <laneSection s="0">\n'
                 '        <left>%s</left>\n'
                 '        <center><lane id="0" type="none"/></center>\n'
                 '        <right>%s</right>\n      </laneSection>\n'
                 % (side(1), side(-1)))
        return ('  <road name="ring %s" id="%s" length="%.17g" junction="-1">\n'
                '    <link>\n'
                '      <predecessor elementType="road" elementId="%s" contactPoint="end"/>\n'
                '      <successor elementType="road" elementId="%s" contactPoint="start"/>\n'
                '    </link>\n'
                '    <planView>\n%s    </planView>\n'
                '    <lanes>\n%s    </lanes>\n  </road>\n'
                % (road_id, road_id, piece['length'], other, other,
                   geometry_xml(piece), lanes))
    a, b = str(first_id), str(first_id + 1)
    cx, cy = center
    return [half(a, b, (cx, cy - radius, 0.0)),
            half(b, a, (cx, cy + radius, math.pi))]


def rings():
    """100 rings of 1 km radius, three lanes each way, 2.5 km apart on a
    grid: 3,770 km of lanes, room for 100,000 vehicles at highway spacing."""
    roads = []
    for k in range(100):
        roads += ring(1000 + 2 * k, (2500.0 * (k % 10), 2500.0 * (k // 10)),
                      1000.0, 3)
    return roads


LANE = 3.5      # Driving lane width.
SIDEWALK = 2.0  # Sidewalk width.


def arm_lanes():
    """One driving lane each way with a sidewalk beyond it."""
    return ('      <laneSection s="0">\n'
            '        <left>%s%s</left>\n'
            '        <center><lane id="0" type="none"/></center>\n'
            '        <right>%s%s</right>\n      </laneSection>\n'
            % (lane_xml(1, 'driving', [(0, LANE, 0, 0, 0)]),
               lane_xml(2, 'sidewalk', [(0, SIDEWALK, 0, 0, 0)]),
               lane_xml(-1, 'driving', [(0, LANE, 0, 0, 0)]),
               lane_xml(-2, 'sidewalk', [(0, SIDEWALK, 0, 0, 0)])))


def signal_xml(signal_id, s, t, dynamic, country, kind, subtype, validity,
               value=None, unit=None, name=''):
    extra = ''
    if value is not None:
        extra += ' value="%g"' % value
    if unit is not None:
        extra += ' unit="%s"' % unit
    return ('      <signal id="%s" name="%s" s="%.17g" t="%.17g" zOffset="2.5" '
            'dynamic="%s" orientation="+" country="%s" type="%s" subtype="%s"%s '
            'height="0.8" width="0.3"><validity fromLane="%d" toLane="%d"/></signal>\n'
            % (signal_id, name, s, t, 'yes' if dynamic else 'no', country, kind,
               subtype, extra, validity[0], validity[1]))


def crosswalk_xml(object_id, s, t, corners, local, heading=0.0, pitch=0.0,
                  roll=0.0, z_offset=0.0, validity=(-1, 1)):
    """A crosswalk whose outline is `corners`, (s, t, dz) on the road or
    (u, v, z) in its own frame."""
    if local:
        rows = ''.join('<cornerLocal u="%.17g" v="%.17g" z="%.17g" height="0"/>' % c
                       for c in corners)
    else:
        rows = ''.join('<cornerRoad s="%.17g" t="%.17g" dz="%.17g" height="0"/>' % c
                       for c in corners)
    return ('      <object id="%s" name="crosswalk %s" type="crosswalk" s="%.17g" '
            't="%.17g" zOffset="%.17g" hdg="%.17g" pitch="%.17g" roll="%.17g" '
            'orientation="none" length="4" width="7">'
            '<outlines><outline id="0" closed="true">%s</outline></outlines>'
            '<validity fromLane="%d" toLane="%d"/></object>\n'
            % (object_id, object_id, s, t, z_offset, heading, pitch, roll, rows,
               validity[0], validity[1]))


def arm(road_id, outward, half, length, junction, signals='', objects=''):
    """A road from `half` + `length` out along `outward` in to `half` from
    the junction's center, ending in the junction, and starting in a
    turnaround of its own."""
    start = ((half + length) * math.cos(outward), (half + length) * math.sin(outward),
             outward + math.pi)
    piece = dict(kind='line', s=0.0, x=start[0], y=start[1], hdg=start[2], length=length)
    return ('  <road name="arm %s" id="%s" length="%.17g" junction="-1">\n'
            '    <link><predecessor elementType="junction" elementId="turn %s"/>'
            '<successor elementType="junction" elementId="%s"/></link>\n'
            '    <planView>\n%s    </planView>\n'
            '    <lanes>\n%s    </lanes>\n'
            '    <objects>\n%s    </objects>\n'
            '    <signals>\n%s    </signals>\n  </road>\n'
            % (road_id, road_id, length, road_id, junction, geometry_xml(piece),
               arm_lanes(), objects, signals))


def turnaround(arm_id, outward, half, length):
    """A half circle at arm `arm_id`'s far end that turns its outbound lane
    into its inbound one, in a junction of its own, so traffic circulates."""
    far = half + length
    x, y = far * math.cos(outward), far * math.sin(outward)
    # Leave the outbound lane's middle heading out, and turn left into the
    # inbound lane's middle.
    left_in = (-math.sin(outward + math.pi), math.cos(outward + math.pi))
    start = (x + LANE / 2.0 * left_in[0], y + LANE / 2.0 * left_in[1])
    piece = dict(kind='arc', s=0.0, x=start[0], y=start[1], hdg=outward,
                 length=math.pi * LANE / 2.0, curvature=2.0 / LANE)
    road_id = 'u' + arm_id
    lanes = ('      <laneSection s="0">\n'
             '        <center><lane id="0" type="none"/></center>\n'
             '        <right>%s</right>\n      </laneSection>\n'
             % lane_xml(-1, 'driving', [(0, LANE, 0, 0, 0)], 1, -1))
    road = ('  <road name="turn %s" id="%s" length="%.17g" junction="turn %s">\n'
            '    <link>\n'
            '      <predecessor elementType="road" elementId="%s" contactPoint="start"/>\n'
            '      <successor elementType="road" elementId="%s" contactPoint="start"/>\n'
            '    </link>\n'
            '    <planView>\n%s    </planView>\n'
            '    <lanes>\n%s    </lanes>\n  </road>\n'
            % (arm_id, road_id, piece['length'], arm_id, arm_id, arm_id,
               geometry_xml(piece), lanes))
    junction = ('  <junction id="turn %s" name="turn %s">\n'
                '    <connection id="0" incomingRoad="%s" connectingRoad="%s" '
                'contactPoint="start"><laneLink from="1" to="-1"/></connection>\n'
                '  </junction>\n' % (arm_id, arm_id, arm_id, road_id))
    return road, junction


def connecting(road_id, junction, half, a, a_outward, b, b_outward):
    """A road in the junction from arm `a`'s end to arm `b`'s end: straight
    across, or a quarter circle of radius `half` turning left or right."""
    hdg = a_outward + math.pi
    turn = math.remainder(b_outward - hdg, 2.0 * math.pi)
    start = (half * math.cos(a_outward), half * math.sin(a_outward))
    if abs(turn) < 1e-9:
        piece = dict(kind='line', length=2.0 * half)
    else:
        piece = dict(kind='arc', length=math.pi * half / 2.0,
                     curvature=math.copysign(1.0 / half, turn))
    piece.update(s=0.0, x=start[0], y=start[1], hdg=hdg)
    lanes = ('      <laneSection s="0">\n'
             '        <center><lane id="0" type="none"/></center>\n'
             '        <right>%s</right>\n      </laneSection>\n'
             % lane_xml(-1, 'driving', [(0, LANE, 0, 0, 0)], -1, 1))
    return ('  <road name="from %s to %s" id="%s" length="%.17g" junction="%s">\n'
            '    <link>\n'
            '      <predecessor elementType="road" elementId="%s" contactPoint="end"/>\n'
            '      <successor elementType="road" elementId="%s" contactPoint="end"/>\n'
            '    </link>\n'
            '    <planView>\n%s    </planView>\n'
            '    <lanes>\n%s    </lanes>\n  </road>\n'
            % (a, b, road_id, piece['length'], junction, a, b, geometry_xml(piece), lanes))


def junction_xml(junction, connections, priorities=(), controllers=()):
    rows = ''.join('    <connection id="%d" incomingRoad="%s" connectingRoad="%s" '
                   'contactPoint="start"><laneLink from="-1" to="-1"/></connection>\n'
                   % (i, a, c) for i, (a, c) in enumerate(connections))
    rows += ''.join('    <priority high="%s" low="%s"/>\n' % p for p in priorities)
    rows += ''.join('    <controller id="%s" type="0" sequence="%d"/>\n' % c
                    for c in controllers)
    return '  <junction id="%s" name="junction %s">\n%s  </junction>\n' % (
        junction, junction, rows)


def controller_xml(controller_id, name, sequence, signal_ids):
    rows = ''.join('<control signalId="%s" type="0"/>' % s for s in signal_ids)
    return ('  <controller id="%s" name="%s" sequence="%d">%s</controller>\n'
            % (controller_id, name, sequence, rows))


def intersection(arms, half, length, junction, signals, objects):
    """Arms named by their outward directions, each joined to every other
    through the junction."""
    roads = [arm(name, outward, half, length, junction, signals.get(name, ''),
                 objects.get(name, '')) for name, outward in arms]
    turns = [turnaround(name, outward, half, length) for name, outward in arms]
    roads += [road for road, _ in turns]
    connections = []
    for a, a_outward in arms:
        for b, b_outward in arms:
            if a != b:
                road_id = '%s%s' % (a, b)
                roads.append(connecting(road_id, junction, half, a, a_outward,
                                        b, b_outward))
                connections.append((a, road_id))
    return roads, connections, [junction for _, junction in turns]


def signalized():
    """Four arms, 100 m each, one lane each way and sidewalks, meeting in a
    junction 20 m across. Each approach has a traffic light at its stop line,
    8 m before the junction, and a crosswalk 4 m wide between them; two
    controllers group the north-south and east-west lights. Each arm's far end
    turns around into it."""
    half, length = 10.0, 100.0
    arms = [('n', math.pi / 2), ('e', 0.0), ('s', -math.pi / 2), ('w', math.pi)]
    signals = {name: signal_xml('light_' + name, length - 8.0, -(LANE + SIDEWALK + 0.5),
                                True, 'DE', '1000001', '-1', (-1, -1),
                                name='light ' + name)
               for name, _ in arms}
    corners_road = [(length - 6.0, -LANE, 0.0), (length - 2.0, -LANE, 0.0),
                    (length - 2.0, LANE, 0.0), (length - 6.0, LANE, 0.0)]
    corners_local = [(-2.0, -LANE, 0.0), (2.0, -LANE, 0.0),
                     (2.0, LANE, 0.0), (-2.0, LANE, 0.0)]
    objects = {
        'n': crosswalk_xml('crosswalk_n', length - 4.0, 0.0, corners_road, False),
        's': crosswalk_xml('crosswalk_s', length - 4.0, 0.0, corners_road, False),
        'e': crosswalk_xml('crosswalk_e', length - 4.0, 0.0, corners_local, True),
        'w': crosswalk_xml('crosswalk_w', length - 4.0, 0.0, corners_local, True),
    }
    roads, connections, turns = intersection(arms, half, length, '1', signals, objects)
    controllers = [controller_xml('1', 'north-south', 1, ['light_n', 'light_s']),
                   controller_xml('2', 'east-west', 2, ['light_e', 'light_w'])]
    return roads + controllers + [junction_xml('1', connections,
                                               controllers=[('1', 1), ('2', 2)])] + turns


def priority():
    """A T: a main road east-west and a minor road from the south, which gives
    way to it, 100 m arms meeting in a junction 20 m across. The minor road
    has a give-way sign, the main road's west arm a 50 km/h limit. Each arm's
    far end turns around into it."""
    half, length = 10.0, 100.0
    arms = [('w', math.pi), ('e', 0.0), ('s', -math.pi / 2)]
    side = -(LANE + SIDEWALK + 0.5)
    signals = {
        's': signal_xml('give_way', length - 3.0, side, False, 'DE', '205', '-1', (-1, -1),
                        name='give way'),
        'w': signal_xml('limit', 20.0, side, False, 'DE', '274', '55', (-1, -1),
                        value=50, unit='km/h', name='limit 50'),
    }
    roads, connections, turns = intersection(arms, half, length, '2', signals, {})
    priorities = [('we', 'se'), ('we', 'sw'), ('we', 'es'), ('ew', 'sw')]
    return roads + [junction_xml('2', connections, priorities=priorities)] + turns


def crosswalks():
    """A curving road that climbs and leans, with sidewalks, and two
    crosswalks: one in its own frame, turned, pitched, rolled and raised, and
    one in road coordinates a little above the road."""
    piece = dict(kind='arc', s=0.0, x=0.0, y=0.0, hdg=0.3, length=100.0, curvature=0.01)
    objects = (crosswalk_xml('turned', 40.0, 0.5,
                             [(-2.0, -4.0, 0.0), (2.0, -4.0, 0.0), (2.0, 4.0, 0.1),
                              (-2.0, 4.0, 0.1)], True, heading=0.2, pitch=0.01,
                             roll=0.02, z_offset=0.05)
               + crosswalk_xml('raised', 72.0, 0.0,
                               [(70.0, -LANE, 0.02), (74.0, -LANE, 0.02),
                                (74.0, LANE, 0.02), (70.0, LANE, 0.02)], False))
    return ('  <road name="crosswalks" id="1" length="100" junction="-1">\n'
            '    <planView>\n%s    </planView>\n'
            '    <elevationProfile><elevation s="0" a="1" b="0.02" c="0" d="0"/></elevationProfile>\n'
            '    <lateralProfile><superelevation s="0" a="0.03" b="0" c="0" d="0"/></lateralProfile>\n'
            '    <lanes>\n%s    </lanes>\n'
            '    <objects>\n%s    </objects>\n  </road>\n'
            % (geometry_xml(piece), arm_lanes(), objects))


def light():
    """One lane, 1 km, with a traffic light at 500 m that a controller groups,
    for checking a queue at a red light against SUMO."""
    piece = dict(kind='line', s=0.0, x=0.0, y=0.0, hdg=0.0, length=1000.0)
    lanes = ('      <laneSection s="0">\n'
             '        <center><lane id="0" type="none"/></center>\n'
             '        <right>%s</right>\n      </laneSection>\n'
             % lane_xml(-1, 'driving', [(0, LANE, 0, 0, 0)]))
    signal = signal_xml('light', 500.0, -(LANE + 0.5), True, 'DE', '1000001', '-1', (-1, -1),
                        name='light')
    return ['  <road name="light" id="1" length="1000" junction="-1">\n'
            '    <planView>\n%s    </planView>\n'
            '    <lanes>\n%s    </lanes>\n'
            '    <signals>\n%s    </signals>\n  </road>\n'
            % (geometry_xml(piece), lanes, signal),
            controller_xml('1', 'light', 0, ['light'])]


def one_way(road_id, start, heading, length, predecessor=None, successor=None,
            signals=''):
    """A road of one driving lane, with s, linked to junctions at its ends."""
    piece = dict(kind='line', s=0.0, x=start[0], y=start[1], hdg=heading, length=length)
    links = ''
    if predecessor is not None:
        links += '<predecessor elementType="junction" elementId="%s"/>' % predecessor
    if successor is not None:
        links += '<successor elementType="junction" elementId="%s"/>' % successor
    lanes = ('      <laneSection s="0">\n'
             '        <center><lane id="0" type="none"/></center>\n'
             '        <right>%s</right>\n      </laneSection>\n'
             % lane_xml(-1, 'driving', [(0, LANE, 0, 0, 0)]))
    return ('  <road name="%s" id="%s" length="%.17g" junction="-1">\n'
            '    <link>%s</link>\n'
            '    <planView>\n%s    </planView>\n'
            '    <lanes>\n%s    </lanes>\n'
            '    <signals>\n%s    </signals>\n  </road>\n'
            % (road_id, road_id, length, links, geometry_xml(piece), lanes, signals))


def through(road_id, junction, start, heading, length, a, b):
    """A straight connecting road of one lane, from road `a`'s end to road
    `b`'s start."""
    piece = dict(kind='line', s=0.0, x=start[0], y=start[1], hdg=heading, length=length)
    lanes = ('      <laneSection s="0">\n'
             '        <center><lane id="0" type="none"/></center>\n'
             '        <right>%s</right>\n      </laneSection>\n'
             % lane_xml(-1, 'driving', [(0, LANE, 0, 0, 0)], -1, -1))
    return ('  <road name="%s" id="%s" length="%.17g" junction="%s">\n'
            '    <link>\n'
            '      <predecessor elementType="road" elementId="%s" contactPoint="end"/>\n'
            '      <successor elementType="road" elementId="%s" contactPoint="start"/>\n'
            '    </link>\n'
            '    <planView>\n%s    </planView>\n'
            '    <lanes>\n%s    </lanes>\n  </road>\n'
            % (road_id, road_id, length, junction, a, b, geometry_xml(piece), lanes))


def crossing():
    """A one-way major road west to east, crossed by a one-way minor road
    south to north that gives way: a give-way sign and the junction's
    priority. Each road runs 300 m to and from a junction 20 m across."""
    half, length = 10.0, 300.0
    give_way = signal_xml('give_way', length - 3.0, -(LANE + 0.5), False, 'DE', '205', '-1',
                          (-1, -1), name='give way')
    roads = [
        one_way('in_w', (-half - length, 0.0), 0.0, length, successor='9'),
        one_way('out_e', (half, 0.0), 0.0, length, predecessor='9'),
        one_way('in_s', (0.0, -half - length), math.pi / 2, length, successor='9',
                signals=give_way),
        one_way('out_n', (0.0, half), math.pi / 2, length, predecessor='9'),
        through('we', '9', (-half, 0.0), 0.0, 2.0 * half, 'in_w', 'out_e'),
        through('sn', '9', (0.0, -half), math.pi / 2, 2.0 * half, 'in_s', 'out_n'),
    ]
    junction = ('  <junction id="9" name="crossing">\n'
                '    <connection id="0" incomingRoad="in_w" connectingRoad="we" '
                'contactPoint="start"><laneLink from="-1" to="-1"/></connection>\n'
                '    <connection id="1" incomingRoad="in_s" connectingRoad="sn" '
                'contactPoint="start"><laneLink from="-1" to="-1"/></connection>\n'
                '    <priority high="we" low="sn"/>\n'
                '  </junction>\n')
    return roads + [junction]


def crossroads():
    """signalized.xodr's four arms and junction with nothing to say who goes
    first: no lights, priorities or signs, so traffic gives way to the
    right."""
    half, length = 10.0, 100.0
    arms = [('n', math.pi / 2), ('e', 0.0), ('s', -math.pi / 2), ('w', math.pi)]
    roads, connections, turns = intersection(arms, half, length, '1', {}, {})
    return roads + [junction_xml('1', connections)] + turns


def document(roads):
    return ('<?xml version="1.0" encoding="UTF-8"?>\n<OpenDRIVE>\n'
            '  <header revMajor="1" revMinor="6" name="simon test" version="1"/>\n'
            '%s</OpenDRIVE>\n' % ''.join(roads))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    roads = os.path.join(here, '..', 'roads')
    with open(os.path.join(roads, 'curves.xodr'), 'w') as f:
        f.write(document([curves()]))
    with open(os.path.join(roads, 'ring.xodr'), 'w') as f:
        f.write(document(ring()))
    with open(os.path.join(roads, 'rings.xodr'), 'w') as f:
        f.write(document(rings()))
    with open(os.path.join(roads, 'paramPoly3.xodr'), 'w') as f:
        f.write(document([
            poly_road('2', (5.0, -3.0, 0.1), arc_length_u((0.0, 0.0, 0.004, -3e-5)),
                      (0.0, 0.0, 0.004, -3e-5), False),
            poly_road('3', (512000.25, 5400000.75, 2.6), (0.0, 80.0, -6.0, 1.5), (0.0, 0.0, 9.0, -4.0), True),
        ]))
    with open(os.path.join(roads, 'signalized.xodr'), 'w') as f:
        f.write(document(signalized()))
    with open(os.path.join(roads, 'priority.xodr'), 'w') as f:
        f.write(document(priority()))
    with open(os.path.join(roads, 'crosswalks.xodr'), 'w') as f:
        f.write(document([crosswalks()]))
    with open(os.path.join(roads, 'light.xodr'), 'w') as f:
        f.write(document(light()))
    with open(os.path.join(roads, 'crossing.xodr'), 'w') as f:
        f.write(document(crossing()))
    with open(os.path.join(roads, 'crossroads.xodr'), 'w') as f:
        f.write(document(crossroads()))
    print('wrote', roads)


if __name__ == '__main__':
    main()
