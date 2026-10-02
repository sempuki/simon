# Copyright 2022 -- CONTRIBUTORS. See LICENSE.

"""Converts a JSBSim aircraft into simon's aircraft format.

An aircraft's authors, license and notes, and its engines' authors, are
carried into the output's header.

  python tools/jsbsim/convert.py <aircraft.xml> <engine directory> <output>

The output is plain text in SI units, read by model/aircraft_data.hpp. It
holds the aircraft's metrics, mass balance, fuel tanks, engines and
aerodynamics. Locations stay in JSBSim's structural frame (x aft, y right,
z up) but in meters.

Each aerodynamic function becomes a term: a constant, times state variables,
times tables of state variables. Functions that other functions name, such as
ground effect factors, are inlined, and metrics are folded into the constant.
Anything JSBSim can express that this subset cannot, such as a sum inside a
product, stops the conversion with the function's name.
"""

import os
import sys
import xml.etree.ElementTree as ElementTree

FT = 0.3048  # m.
IN = 0.0254  # m.
LBF = 4.4482216152605  # N.
LBM = 0.45359237  # kg.
SLUG = LBF / FT  # kg: a slug is a pound-force second squared per foot.
SLUG_FT2 = SLUG * FT * FT  # kg m^2.
PSF = LBF / (FT * FT)  # Pa.

# JSBSim's state properties, by simon's name for them and the factor from
# JSBSim's units to SI.
VARIABLES = {
    'aero/qbar-psf': ('dynamic_pressure', PSF),
    'aero/alpha-rad': ('alpha', 1.0),
    'aero/beta-rad': ('beta', 1.0),
    'velocities/mach': ('mach', 1.0),
    'aero/bi2vel': ('span_over_twice_speed', 1.0),  # s, either way.
    'aero/ci2vel': ('chord_over_twice_speed', 1.0),
    'velocities/p-aero-rad_sec': ('roll_rate', 1.0),
    'velocities/q-aero-rad_sec': ('pitch_rate', 1.0),
    'velocities/r-aero-rad_sec': ('yaw_rate', 1.0),
    'aero/alphadot-rad_sec': ('alpha_rate', 1.0),
    'aero/cl-squared': ('lift_coefficient_squared', 1.0),
    'aero/h_b-mac-ft': ('height_over_span', 1.0),  # A ratio, despite its name.
    'fcs/elevator-pos-rad': ('elevator', 1.0),
    'fcs/mag-elevator-pos-rad': ('elevator_magnitude', 1.0),
    'fcs/left-aileron-pos-rad': ('left_aileron', 1.0),
    'fcs/right-aileron-pos-rad': ('right_aileron', 1.0),
    'fcs/rudder-pos-rad': ('rudder', 1.0),
    'fcs/flap-pos-norm': ('flaps', 1.0),
    'gear/gear-pos-norm': ('gear', 1.0),
    'fcs/speedbrake-pos-norm': ('speedbrake', 1.0),
    'fcs/spoiler-pos-norm': ('spoilers', 1.0),
    'atmosphere/density-altitude': ('density_altitude', FT),
}

# Metrics that fold into a term's constant, in JSBSim's units.
METRICS = ['metrics/Sw-sqft', 'metrics/bw-ft', 'metrics/cbarw-ft']

# simon's name for each axis, and the factor from JSBSim's force or moment
# units to SI.
AXES = {'DRAG': ('drag', LBF), 'SIDE': ('side', LBF), 'LIFT': ('lift', LBF),
        'ROLL': ('roll', LBF * FT), 'PITCH': ('pitch', LBF * FT),
        'YAW': ('yaw', LBF * FT)}


class ConversionError(Exception):
    pass


def number(element, name, default=None):
    child = element.find(name)
    if child is None:
        if default is None:
            raise ConversionError('missing <%s> in <%s>' % (name, element.tag))
        return default
    return float(child.text)


def length(element, name):
    """A length in meters, from a unit attribute JSBSim allows."""
    child = element.find(name)
    if child is None:
        raise ConversionError('missing <%s>' % name)
    unit = child.get('unit', 'FT')
    return float(child.text) * {'FT': FT, 'IN': IN, 'M': 1.0}[unit]


def area(element, name):
    child = element.find(name)
    unit = child.get('unit', 'FT2')
    return float(child.text) * {'FT2': FT * FT, 'M2': 1.0}[unit]


def mass(element, name):
    child = element.find(name)
    unit = child.get('unit', 'LBS')
    return float(child.text) * {'LBS': LBM, 'KG': 1.0}[unit]


def location(element):
    unit = element.get('unit', 'IN')
    scale = {'IN': IN, 'FT': FT, 'M': 1.0}[unit]
    return [float(element.find(axis).text) * scale for axis in 'xyz']


def variable(name):
    if name not in VARIABLES:
        raise ConversionError('unsupported property %s' % name)
    return VARIABLES[name]


class Table:
    def __init__(self, element):
        variables = element.findall('independentVar')
        data = element.find('tableData').text.split()
        values = [float(x) for x in data]
        if len(variables) == 1:
            self.variables = [variable(variables[0].text.strip())]
            self.rows = values[0::2]
            self.columns = []
            self.values = values[1::2]
        elif len(variables) == 2:
            lookups = [v.get('lookup', 'row') for v in variables]
            row = variables[lookups.index('row')].text.strip()
            column = variables[lookups.index('column')].text.strip()
            self.variables = [variable(row), variable(column)]
            # The first line is the column breakpoints; each row after it
            # starts with its breakpoint.
            lines = [line.split() for line in
                     element.find('tableData').text.strip().splitlines()
                     if line.strip()]
            self.columns = [float(x) for x in lines[0]]
            self.rows = [float(line[0]) for line in lines[1:]]
            self.values = [float(x) for line in lines[1:] for x in line[1:]]
            if len(self.values) != len(self.rows) * len(self.columns):
                raise ConversionError('ragged table')
        else:
            raise ConversionError('tables of three variables are unsupported')
        # Breakpoints into SI.
        self.rows = [x * self.variables[0][1] for x in self.rows]
        if self.columns:
            self.columns = [x * self.variables[1][1] for x in self.columns]

    def write(self, out, indent):
        if not self.columns:
            out.write('%stable %s\n' % (indent, self.variables[0][0]))
            for x, y in zip(self.rows, self.values):
                out.write('%s  %.17g %.17g\n' % (indent, x, y))
        else:
            out.write('%stable %s %s\n' % (indent, self.variables[0][0],
                                           self.variables[1][0]))
            out.write('%s  columns %s\n' % (
                indent, ' '.join('%.17g' % c for c in self.columns)))
            width = len(self.columns)
            for i, r in enumerate(self.rows):
                out.write('%s  %.17g %s\n' % (indent, r, ' '.join(
                    '%.17g' % v for v in self.values[i * width:(i + 1) * width])))
        out.write('%send\n' % indent)


class Term:
    """A constant, times variables, times tables."""

    def __init__(self):
        self.constant = 1.0
        self.factors = []
        self.tables = []

    def multiply(self, element, functions, metrics, where):
        tag = element.tag
        if tag == 'product':
            for child in element:
                if child.tag != 'description':
                    self.multiply(child, functions, metrics, where)
        elif tag == 'value':
            self.constant *= float(element.text)
        elif tag == 'table':
            self.tables.append(Table(element))
        elif tag == 'property':
            name = element.text.strip()
            if name in METRICS:
                self.constant *= metrics[name]
            elif name in functions:
                self.multiply(body(functions[name]), functions, metrics, where)
            else:
                # JSBSim's value is the SI one over `scale`.
                symbol, scale = variable(name)
                self.constant /= scale
                self.factors.append(symbol)
        else:
            raise ConversionError('%s: <%s> is unsupported' % (where, tag))

    def write(self, out, axis, name):
        out.write('term %s %s\n' % (axis, name))
        out.write('  constant %.17g\n' % self.constant)
        for factor in self.factors:
            out.write('  factor %s\n' % factor)
        for table in self.tables:
            table.write(out, '  ')
        out.write('end\n')


def body(function):
    children = [c for c in function if c.tag != 'description']
    if len(children) != 1:
        raise ConversionError('function %s needs one body' % function.get('name'))
    return children[0]


def provenance(root):
    """Who wrote the aircraft, and under what license, from its file header,
    to carry into the conversion."""
    header = root.find('fileheader')
    if header is None:
        return ['The source file names no author or license.']
    lines = []
    authors = [a.text.strip() for a in header.findall('author') if a.text]
    if authors:
        lines.append('Authors: %s.' % ', '.join(authors))
    license = header.find('license')
    if license is not None:
        lines.append('License: %s, %s.' % (license.get('licenseName', '?'),
                                         license.get('licenseURL', '?')))
    else:
        lines.append('The source file names no license.')
    for note in header.findall('note'):
        if note.text:
            words = note.text.split()
            line = 'Note:'
            for word in words:
                if len(line) + len(word) > 74:
                    lines.append(line)
                    line = ' '
                line += ' ' + word
            lines.append(line)
    for tag in ['filecreationdate', 'version']:
        element = header.find(tag)
        if element is not None and element.text:
            lines.append('%s: %s' % (tag, element.text.strip()))
    return lines


def engine_provenance(path):
    """The author an engine file names in its leading comment, if any."""
    with open(path) as source:
        text = source.read(2000)
    for line in text.splitlines():
        if 'Author:' in line:
            return line.split('Author:', 1)[1].strip()
    return 'unnamed'


def convert(path, engine_directory, out):
    root = ElementTree.parse(path).getroot()

    metrics_element = root.find('metrics')
    metrics = {
        'wing_area': area(metrics_element, 'wingarea'),
        'wing_span': length(metrics_element, 'wingspan'),
        'chord': length(metrics_element, 'chord'),
    }
    out.write('simon-aircraft 1\n')
    out.write('# Converted from JSBSim\'s %s by tools/jsbsim/convert.py.\n'
              % os.path.basename(path))
    for line in provenance(root):
        out.write('# %s\n' % line)
    out.write('# SI units. Locations are in JSBSim\'s structural frame, '
              'x aft, y right, z up.\n')
    out.write('name %s\n' % root.get('name'))
    out.write('metrics %.17g %.17g %.17g\n' % (
        metrics['wing_area'], metrics['wing_span'], metrics['chord']))
    # The functions multiply metrics in JSBSim's units.
    jsbsim_metrics = {'metrics/Sw-sqft': metrics['wing_area'] / FT**2,
                      'metrics/bw-ft': metrics['wing_span'] / FT,
                      'metrics/cbarw-ft': metrics['chord'] / FT}
    for point in metrics_element.findall('location'):
        if point.get('name') == 'AERORP':
            out.write('aero_reference %.17g %.17g %.17g\n' % tuple(location(point)))

    balance = root.find('mass_balance')
    negated = balance.get('negated_crossproduct_inertia', 'true') == 'true'
    sign = -1.0 if negated else 1.0
    inertia = [number(balance, axis, 0.0) * SLUG_FT2
               for axis in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']]
    for i in range(3, 6):
        inertia[i] *= sign
    # The tensor's elements, as JSBSim builds them: xx yy zz xy xz yz.
    out.write('empty_mass %.17g\n' % mass(balance, 'emptywt'))
    out.write('empty_inertia %s\n' % ' '.join('%.17g' % x for x in inertia))
    for point in balance.findall('location'):
        if point.get('name') == 'CG':
            out.write('empty_center_of_mass %.17g %.17g %.17g\n'
                      % tuple(location(point)))
    if balance.findall('pointmass'):
        raise ConversionError('point masses are unsupported')

    propulsion = root.find('propulsion')
    for tank in propulsion.findall('tank'):
        if tank.get('type') != 'FUEL':
            raise ConversionError('only fuel tanks are supported')
        out.write('tank %.17g %.17g %.17g %.17g %.17g\n' % (
            *location(tank.find('location')), mass(tank, 'capacity'),
            mass(tank, 'contents')))
    for engine in propulsion.findall('engine'):
        convert_engine(engine, engine_directory, out)

    aerodynamics = root.find('aerodynamics')
    functions = {f.get('name'): f for f in aerodynamics.findall('function')}
    for axis in aerodynamics.findall('axis'):
        name = axis.get('name')
        if name not in AXES:
            raise ConversionError('axis %s is unsupported' % name)
        axis_name, unit = AXES[name]
        for function in axis.findall('function'):
            term = Term()
            where = function.get('name')
            term.multiply(body(function), functions, jsbsim_metrics, where)
            term.constant *= unit
            term.write(out, axis_name, where.split('/')[-1])


def convert_engine(engine, engine_directory, out):
    path = os.path.join(engine_directory, engine.get('file') + '.xml')
    turbine = ElementTree.parse(path).getroot()
    if turbine.tag != 'turbine_engine':
        raise ConversionError('only turbine engines are supported')
    thruster = engine.find('thruster')
    if thruster.get('file') != 'direct':
        raise ConversionError('only direct thrusters are supported')
    orient = thruster.find('orient')
    angles = [number(orient, axis, 0.0) for axis in ['roll', 'pitch', 'yaw']]
    if any(angles):
        raise ConversionError('tilted thrusters are unsupported')
    if int(number(turbine, 'augmented', 0)) or int(number(turbine, 'injected', 0)):
        raise ConversionError('augmented or injected turbines are unsupported')

    out.write('# Engine %s, from JSBSim\'s %s, by %s.\n' % (
        turbine.get('name'), os.path.basename(path), engine_provenance(path)))
    out.write('engine turbine %s\n' % turbine.get('name'))
    out.write('  location %.17g %.17g %.17g\n' % tuple(location(thruster.find('location'))))
    out.write('  feeds %s\n' % ' '.join(f.text.strip() for f in engine.findall('feed')))
    out.write('  military_thrust %.17g\n' % (number(turbine, 'milthrust') * LBF))
    out.write('  bypass_ratio %.17g\n' % number(turbine, 'bypassratio', 0.0))
    # lbm/hr/lbf into kg/s/N.
    out.write('  thrust_specific_fuel_consumption %.17g\n' % (
        number(turbine, 'tsfc') * LBM / 3600.0 / LBF))
    out.write('  bleed %.17g\n' % number(turbine, 'bleed', 0.0))
    for key, name in [('idlen1', 'idle_n1'), ('idlen2', 'idle_n2'),
                      ('maxn1', 'max_n1'), ('maxn2', 'max_n2')]:
        out.write('  %s %.17g\n' % (name, number(turbine, key)))
    for function in turbine.findall('function'):
        name = {'IdleThrust': 'idle_thrust', 'MilThrust': 'military_thrust_factor'}.get(
            function.get('name'))
        if name is None:
            raise ConversionError('turbine function %s is unsupported' % function.get('name'))
        out.write('  %s\n' % name)
        Table(body(function)).write(out, '    ')
    out.write('end\n')


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    try:
        with open(sys.argv[3], 'w') as out:
            convert(sys.argv[1], sys.argv[2], out)
    except ConversionError as error:
        sys.exit('%s: %s' % (sys.argv[1], error))


if __name__ == '__main__':
    main()
