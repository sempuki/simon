# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Converts a JSBSim aircraft into simon's aircraft format.

An aircraft's authors, license and notes, and its engines' authors, are
carried into the output's header.

  python tools/jsbsim/convert.py <aircraft.xml> <engine directory> <output>

The output is plain text in SI units, read by model/aircraft_data.hpp. It
holds the aircraft's metrics and eye point, its mass balance and point
masses, fuel tanks, engines, flight controls and aerodynamics. Locations stay
in JSBSim's structural frame (x aft, y right, z up) but in meters.

Each aerodynamic function becomes a term: a constant, times inputs, times
tables of inputs, where an input is a state variable or a flight control
signal. Functions that other functions name, such as ground effect factors,
are inlined, and metrics are folded into the constant. Each flight control
component becomes a block, in the order JSBSim runs them. Anything JSBSim can
express that this subset cannot, such as a sum inside a product, stops the
conversion and names what stopped it.
"""

import math
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
KT = 1852.0 / 3600.0  # m/s.
DEG = math.pi / 180.0  # rad.

# JSBSim's aerodynamic state properties, by simon's name for them and the
# factor from JSBSim's units to SI.
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
    'atmosphere/density-altitude': ('density_altitude', FT),
}

# Metrics that fold into a term's constant, in JSBSim's units.
METRICS = ['metrics/Sw-sqft', 'metrics/bw-ft', 'metrics/cbarw-ft']

# simon's name for each axis, and the factor from JSBSim's force or moment
# units to SI.
AXES = {'DRAG': ('drag', LBF), 'SIDE': ('side', LBF), 'LIFT': ('lift', LBF),
        'ROLL': ('roll', LBF * FT), 'PITCH': ('pitch', LBF * FT),
        'YAW': ('yaw', LBF * FT)}


# Signals the flight control system reads from the pilot, by simon's name.
COMMANDS = {
    'fcs/elevator-cmd-norm': 'elevator_command',
    'fcs/aileron-cmd-norm': 'aileron_command',
    'fcs/rudder-cmd-norm': 'rudder_command',
    'fcs/flap-cmd-norm': 'flaps_command',
    'gear/gear-cmd-norm': 'gear_command',
    'fcs/speedbrake-cmd-norm': 'speedbrake_command',
    'fcs/spoiler-cmd-norm': 'spoilers_command',
    'fcs/pitch-trim-cmd-norm': 'pitch_trim_command',
    'fcs/roll-trim-cmd-norm': 'roll_trim_command',
    'fcs/yaw-trim-cmd-norm': 'yaw_trim_command',
    'fcs/throttle-cmd-norm': 'throttle_command_0',
}
for engine_index in range(4):
    COMMANDS['fcs/throttle-cmd-norm[%d]' % engine_index] = (
        'throttle_command_%d' % engine_index)

# Signals it reads from the aircraft's state, by simon's name and the factor
# from JSBSim's units to SI. The pilot's accelerations are in g, along body
# axes. The aircraft is always flying, so no wheel carries weight.
STATE = {
    'velocities/mach': ('mach', 1.0),
    'velocities/p-aero-rad_sec': ('roll_rate', 1.0),
    'velocities/q-aero-rad_sec': ('pitch_rate', 1.0),
    'velocities/r-aero-rad_sec': ('yaw_rate', 1.0),
    'aero/alpha-rad': ('alpha', 1.0),
    'aero/alpha-deg': ('alpha', DEG),
    'aero/beta-rad': ('beta', 1.0),
    'velocities/vc-kts': ('calibrated_airspeed', KT),
    'velocities/vg-fps': ('ground_speed', FT),
    'velocities/u-fps': ('body_velocity_x', FT),
    'velocities/v-fps': ('body_velocity_y', FT),
    'attitude/pitch-rad': ('pitch', 1.0),
    'attitude/roll-rad': ('roll', 1.0),
    'accelerations/n-pilot-y-norm': ('pilot_acceleration_y', 1.0),
    'accelerations/n-pilot-z-norm': ('pilot_acceleration_z', 1.0),
}
for gear_index in range(16):
    STATE['gear/unit[%d]/WOW' % gear_index] = ('weight_on_wheels', 1.0)

# The surfaces JSBSim's flight control system has built in (FGFCS), by
# simon's name for each. JSBSim keeps a surface's deflection in radians and
# degrees as one value, `-rad` and `-deg`, and its normalized position,
# `-norm`, as another. simon keeps the deflection in radians, under the
# surface's name, and the position under its name and `_norm`.
SURFACES = {
    'fcs/elevator-pos': 'elevator',
    'fcs/left-aileron-pos': 'left_aileron',
    'fcs/right-aileron-pos': 'right_aileron',
    'fcs/rudder-pos': 'rudder',
    'fcs/flap-pos': 'flaps',
    'fcs/speedbrake-pos': 'speedbrake',
    'fcs/spoiler-pos': 'spoilers',
}


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
    """What the aerodynamics read for property `name`: simon's name for the
    variable or signal, and the factor from JSBSim's units to SI. A
    surface's magnitude is its signal's name between bars."""
    if name in VARIABLES:
        return VARIABLES[name]
    if name.startswith('fcs/mag-') and name.endswith('-rad'):
        signal, scale = flight_signal('fcs/' + name[len('fcs/mag-'):])
        return '|%s|' % signal, scale
    return flight_signal(name)


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
        elif point.get('name') == 'EYEPOINT':
            out.write('eye_point %.17g %.17g %.17g\n' % tuple(location(point)))

    balance = root.find('mass_balance')
    inertia = [number(balance, axis, 0.0) * SLUG_FT2
               for axis in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']]
    # The file gives the tensor in the structural frame: its elements if it
    # says its products of inertia are negated, as it does by default, or
    # the products themselves, which are the elements negated.
    if balance.get('negated_crossproduct_inertia', 'true') != 'true':
        for i in range(3, 6):
            inertia[i] = -inertia[i]
    # Into body axes, whose x and z point opposite the structural frame's:
    # the xy and yz elements change sign, and the xz element does not.
    inertia[3] = -inertia[3]
    inertia[5] = -inertia[5]
    # The tensor's elements in body axes: xx yy zz xy xz yz.
    out.write('empty_mass %.17g\n' % mass(balance, 'emptywt'))
    out.write('empty_inertia %s\n' % ' '.join('%.17g' % x for x in inertia))
    for point in balance.findall('location'):
        if point.get('name') == 'CG':
            out.write('empty_center_of_mass %.17g %.17g %.17g\n'
                      % tuple(location(point)))
    # Each point mass, such as the pilot, as a mass at a location.
    for point in balance.findall('pointmass'):
        if point.find('form') is not None:
            raise ConversionError('point masses with a shape are unsupported')
        out.write('point_mass %.17g %.17g %.17g %.17g\n' % (
            mass(point, 'weight'), *location(point.find('location'))))

    propulsion = root.find('propulsion')
    for tank in propulsion.findall('tank'):
        if tank.get('type') != 'FUEL':
            raise ConversionError('only fuel tanks are supported')
        out.write('tank %.17g %.17g %.17g %.17g %.17g\n' % (
            *location(tank.find('location')), mass(tank, 'capacity'),
            mass(tank, 'contents')))
    for engine in propulsion.findall('engine'):
        convert_engine(engine, engine_directory, out)

    convert_flight_controls(root, out)

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


def component_property(name):
    """The property JSBSim gives a component's output (FGFCSComponent)."""
    if '/' in name:
        return name
    return 'fcs/' + name.strip().lower().replace(' ', '-')


def flight_signal(prop):
    """simon's name for the signal JSBSim calls `prop`, and the factor from
    JSBSim's units to SI."""
    if prop in COMMANDS:
        return COMMANDS[prop], 1.0
    if prop in STATE:
        return STATE[prop]
    for base, name in SURFACES.items():
        if prop == base + '-rad':
            return name, 1.0
        if prop == base + '-deg':
            return name, math.pi / 180.0
        if prop == base + '-norm':
            return name + '_norm', 1.0
    if prop == 'gear/gear-pos-norm':
        return 'gear', 1.0
    if prop.startswith('fcs/throttle-pos-norm'):
        index = prop[len('fcs/throttle-pos-norm'):].strip('[]') or '0'
        return 'throttle_%s' % index, 1.0
    if prop.startswith('fcs/'):
        return prop[len('fcs/'):].replace('/', '-'), 1.0
    raise ConversionError('unsupported property %s' % prop)


def signal_name(prop):
    """simon's name for a signal a component writes as its own, which must be
    in SI units already."""
    name, scale = flight_signal(prop)
    if scale != 1.0:
        raise ConversionError('a component named %s would need its units '
                              'converted' % prop)
    return name


def bounds(element, name):
    child = element.find(name)
    if child is None:
        return None
    return float(child.find('min').text), float(child.find('max').text)


def scaled(name, scale):
    """A signal as a block line reads it: by name, then the factor that
    takes the signal's SI value back to JSBSim's units, if there is one."""
    if scale == 1.0:
        return name
    return '%s %.17g' % (name, scale)


# Comparisons a switch's conditions make, by JSBSim's names for them.
COMPARISONS = {'lt': 'lt', '<': 'lt', 'le': 'le', '<=': 'le',
               'gt': 'gt', '>': 'gt', 'ge': 'ge', '>=': 'ge',
               'eq': 'eq', '==': 'eq', 'ne': 'ne', '!=': 'ne'}

# A function's operations that take a fixed number of arguments.
OPERATIONS = {'sin': 1, 'cos': 1, 'tan': 1, 'abs': 1,
              'difference': 2, 'quotient': 2}


class FlightControls:
    """The blocks of a JSBSim <flight_control>, in the order JSBSim runs
    them, as lines of simon's format."""

    def __init__(self):
        self.written = set()  # The signals some block writes.
        self.read = []  # The signals blocks read, in order.
        self.blocks = []

    def reads(self, prop):
        """A property a block reads in JSBSim's units: simon's name, and the
        factor from SI back to JSBSim's units."""
        name, scale = flight_signal(prop)
        if name not in self.read:
            self.read.append(name)
        return name, 1.0 / scale

    def input(self, text):
        text = text.strip()
        sign = '-' if text.startswith('-') else ''
        name, scale = self.reads(text.lstrip('-'))
        return sign + scaled(name, scale)

    def operand(self, text):
        """A switch's value or a side of a condition: a number, or a signal
        in SI units, which a condition compares with the other side in SI."""
        text = text.strip()
        try:
            return float(text), None
        except ValueError:
            pass
        sign = -1.0 if text.startswith('-') else 1.0
        return sign, self.reads(text.lstrip('-'))

    def value(self, text, where):
        """A switch's value, as a block line: a number, or a signal, which
        must need no change of units."""
        number, signal = self.operand(text)
        if signal is None:
            return '%.17g' % number
        if signal[1] != 1.0:
            raise ConversionError('%s: a value that needs its units converted'
                                  % where)
        return ('-' if number < 0 else '') + signal[0]

    def condition(self, text, where):
        words = text.split()
        if len(words) != 3 or words[1] not in COMPARISONS:
            raise ConversionError('%s: condition "%s" is unsupported'
                                  % (where, text))
        left, right = self.operand(words[0]), self.operand(words[2])
        if left[1] is None:
            raise ConversionError('%s: a condition starts with a property'
                                  % where)
        left_name, left_scale = left[1]
        if right[1] is None:
            # The number in SI, so the comparison is JSBSim's.
            right_text = '%.17g' % (right[0] / left_scale)
        else:
            if right[1][1] != left_scale or right[0] < 0 or left[0] < 0:
                raise ConversionError('%s: condition "%s" is unsupported'
                                      % (where, text))
            right_text = right[1][0]
        return 'condition %s %s %s' % (left_name, COMPARISONS[words[1]],
                                       right_text)

    def function(self, element, lines, where):
        """A function's body as postfix operations."""
        tag = element.tag
        if tag == 'property':
            lines.append('push %s' % self.input(element.text))
        elif tag == 'value':
            lines.append('constant %.17g' % float(element.text))
        elif tag in ('sum', 'product'):
            children = [c for c in element if c.tag != 'description']
            for child in children:
                self.function(child, lines, where)
            lines.append('%s %d' % (tag, len(children)))
        elif tag in OPERATIONS:
            children = [c for c in element if c.tag != 'description']
            if len(children) != OPERATIONS[tag]:
                raise ConversionError('%s: <%s> takes %d arguments'
                                      % (where, tag, OPERATIONS[tag]))
            for child in children:
                self.function(child, lines, where)
            lines.append(tag)
        else:
            raise ConversionError('%s: <%s> is unsupported' % (where, tag))

    def add(self, component):
        kind = component.tag
        name = component.get('name')
        where = '%s %s' % (kind, name)
        own = signal_name(component_property(name))
        lines = []
        for element in component.findall('input'):
            lines.append('input %s' % self.input(element.text))
        clip = component.find('clipto')
        if clip is not None:
            lines.append('clip %.17g %.17g' % (
                float(clip.find('min').text), float(clip.find('max').text)))
        for element in component.findall('output'):
            out, scale = flight_signal(element.text.strip())
            lines.append('output %s' % scaled(out, scale))
            self.written.add(out)

        if kind == 'summer':
            bias = component.find('bias')
            if bias is not None:
                lines.append('bias %.17g' % float(bias.text))
        elif kind in ('pure_gain', 'scheduled_gain', 'aerosurface_scale'):
            gain = component.find('gain')
            if gain is not None:
                lines.append('gain %.17g' % float(gain.text))
            if kind == 'aerosurface_scale':
                domain = bounds(component, 'domain') or (-1.0, 1.0)
                span = bounds(component, 'range')
                if span is None:
                    raise ConversionError('%s has no range' % where)
                lines.append('domain %.17g %.17g' % domain)
                lines.append('range %.17g %.17g' % span)
                zero = component.find('zero_centered')
                centered = zero is None or zero.text.strip() not in ('0', 'false')
                lines.append('zero_centered %d' % centered)
            if kind == 'scheduled_gain':
                table = component.find('table')
                variables = table.findall('independentVar')
                if len(variables) != 1:
                    raise ConversionError('%s: one variable only' % where)
                variable_name, scale = self.reads(variables[0].text.strip())
                data = [float(x) for x in table.find('tableData').text.split()]
                # Breakpoints into SI.
                lines.append('table %s' % variable_name)
                for x, y in zip(data[0::2], data[1::2]):
                    lines.append('%.17g %.17g' % (x / scale, y))
                lines.append('end')
        elif kind == 'kinematic':
            if component.find('noscale') is not None:
                lines.append('scale 0')
            for setting in component.find('traverse').findall('setting'):
                lines.append('setting %.17g %.17g' % (
                    float(setting.find('position').text),
                    float(setting.find('time').text)))
        elif kind == 'switch':
            for test in component:
                if test.tag == 'default':
                    lines.append('default %s' % self.value(test.get('value'), where))
                elif test.tag == 'test':
                    if test.findall('test'):
                        raise ConversionError('%s: nested tests are unsupported'
                                              % where)
                    logic = test.get('logic', 'AND').lower()
                    if logic not in ('and', 'or'):
                        raise ConversionError('%s: logic %s' % (where, logic))
                    lines.append('test %s %s' % (
                        logic, self.value(test.get('value'), where)))
                    for text in test.text.strip().splitlines():
                        if text.strip():
                            lines.append(self.condition(text, where))
                    lines.append('end')
                elif test.tag not in ('output', 'clipto', 'description'):
                    raise ConversionError('%s: <%s> is unsupported'
                                          % (where, test.tag))
            if component.find('delay') is not None:
                raise ConversionError('%s: delays are unsupported' % where)
        elif kind == 'pid':
            if component.get('type', '') == 'standard':
                raise ConversionError('%s: standard PIDs are unsupported' % where)
            if component.find('pvdot') is not None:
                raise ConversionError('%s: <pvdot> is unsupported' % where)
            trigger = component.find('trigger')
            if trigger is not None:
                lines.append('trigger %s' % self.input(trigger.text))
            for gain in ('kp', 'ki', 'kd'):
                element = component.find(gain)
                if element is not None:
                    lines.append('%s %.17g' % (gain, float(element.text)))
            ki = component.find('ki')
            # JSBSim integrates by Adams-Bashforth 2 unless told otherwise.
            integrator = 'none' if ki is None else {
                'rect': 'rect', 'trap': 'trap', 'ab3': 'ab3'}.get(
                    ki.get('type', ''), 'ab2')
            lines.append('integrator %s' % integrator)
        elif kind == 'fcs_function':
            function = component.find('function')
            self.function(body(function), lines, where)
        else:
            raise ConversionError('%s is unsupported' % where)
        kind = {'aerosurface_scale': 'surface_scale',
                'fcs_function': 'function'}.get(kind, kind)
        self.blocks.append((kind, own, lines))
        self.written.add(own)

    def write(self, out, declared):
        out.write('flight_controls\n')
        # Signals blocks read that nothing writes and that the aircraft is
        # not given: they hold zero, as JSBSim's do.
        for name in declared + [n for n in self.read if n not in declared]:
            if name not in self.written and not is_fixed(name):
                out.write('  signal %s\n' % name)
        for kind, name, lines in self.blocks:
            out.write('  block %s %s\n' % (kind, name))
            depth = 4
            for line in lines:
                if line == 'end':
                    depth -= 2
                out.write('%s%s\n' % (' ' * depth, line))
                if line.startswith(('table ', 'test ')):
                    depth += 2
            out.write('  end\n')
        out.write('end\n')


def is_fixed(name):
    """Whether `name` is a signal the aircraft is given."""
    return (name in COMMANDS.values() or
            name in (state for state, _ in STATE.values()))


def convert_flight_controls(root, out):
    """Each component of each channel, in the order JSBSim runs them."""
    controls = root.find('flight_control')
    if controls is None:
        return
    blocks = FlightControls()
    declared = []
    for prop in controls.findall('property'):
        if prop.get('value') not in (None, '0', '0.0'):
            raise ConversionError('declared %s starts at a value' % prop.text)
        declared.append(signal_name(prop.text.strip()))
    for channel in controls.findall('channel'):
        for component in channel:
            blocks.add(component)
    blocks.write(out, declared)


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
    if int(number(turbine, 'injected', 0)):
        raise ConversionError('injected turbines are unsupported')
    augmented = int(number(turbine, 'augmented', 0))
    method = int(number(turbine, 'augmethod', 0))
    if augmented and method != 2:
        raise ConversionError('only reheat by throttle (augmethod 2) is '
                              'supported')

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
    if augmented:
        # Throttle past 1 lights the reheat, which is full at 2.
        out.write('  max_thrust %.17g\n' % (number(turbine, 'maxthrust') * LBF))
        out.write('  reheat_thrust_specific_fuel_consumption %.17g\n' % (
            number(turbine, 'atsfc') * LBM / 3600.0 / LBF))
    for key, name in [('idlen1', 'idle_n1'), ('idlen2', 'idle_n2'),
                      ('maxn1', 'max_n1'), ('maxn2', 'max_n2')]:
        out.write('  %s %.17g\n' % (name, number(turbine, key)))
    # JSBSim estimates what the file leaves out: the idle fuel flow from the
    # military thrust, in pounds per hour, and how fast the spools turn, in
    # percent per second, from the bypass ratio (FGTurbine).
    military = number(turbine, 'milthrust')
    out.write('  idle_fuel_flow %.17g\n' % (military**0.2 * 107.0 * LBM / 3600.0))
    spool = 90.0 / (number(turbine, 'bypassratio', 0.0) + 3.0)
    for name, factor in [('n1_spool_up', 1.0), ('n1_spool_down', 2.4),
                         ('n2_spool_up', 1.0), ('n2_spool_down', 3.0)]:
        out.write('  %s %.17g\n' % (name, factor * spool))
    for function in turbine.findall('function'):
        name = {'IdleThrust': 'idle_thrust', 'MilThrust': 'military_thrust_factor',
                'AugThrust': 'max_thrust_factor'}.get(function.get('name'))
        if name == 'max_thrust_factor' and not augmented:
            continue
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
