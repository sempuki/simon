# Copyright 2026 -- CONTRIBUTORS. See LICENSE.

"""Measures esmini's scenario runs by nuPlan's metrics, for metrics_test.

nuPlan's devkit (https://github.com/motional/nuplan-devkit, Apache-2.0)
measures a planner's runs: how comfortable the ego's ride is, by its
longitudinal and lateral acceleration, its yaw rate and acceleration and its
jerk, each smoothed by Savitzky-Golay filters; and its time to collision with
the vehicles ahead. This script takes the ego's run in each of esmini's
scenarios from esmini_scenarios.csv as the samples nuPlan reads: its rear
axle's pose, its speed, and its acceleration along its heading, the change in
speed over the step, and across it, its speed times its yaw rate. It writes
nuplan_metrics.csv: for each sample, those inputs, and what nuPlan's own code
makes of them, its state extractors for comfort and its time to collision at
the timestamp, the vehicles ahead of the ego its tracks.

For each of the permutations of cut-in_parameter_set.xosc that esmini played
(esmini_permutations.csv), sampled after each step as simon samples them, it
writes nuplan_permutations.csv: at each sample, the time to collision and
the least distance between the ego's box and another's, by Shapely as
nuPlan measures it; and nuplan_verdicts.csv: for each permutation, its
samples, whether nuPlan's comfort metrics all hold within their bounds, the
least time to collision and whether it stays within nuPlan's bound, and the
least distance.

  git clone https://github.com/motional/nuplan-devkit
  pip install numpy scipy shapely geopandas opencv-python-headless pytest \\
      pyquaternion
  PYTHONPATH=nuplan-devkit \\
      python application/automotive/reference/nuplan_metrics.py
"""

import collections
import csv
import math
import os

import numpy as np
from nuplan.common.actor_state.ego_state import EgoState
from nuplan.common.actor_state.oriented_box import OrientedBox
from nuplan.common.actor_state.state_representation import StateSE2, StateVector2D, TimePoint
from nuplan.common.actor_state.vehicle_parameters import VehicleParameters
from nuplan.planning.metrics.evaluation_metrics.base.within_bound_metric_base import (
    WithinBoundMetricBase)
from nuplan.planning.metrics.evaluation_metrics.common.time_to_collision_within_bound import (
    _compute_time_to_collision_at_timestamp)
from nuplan.planning.metrics.metric_result import TimeSeries
from nuplan.planning.metrics.utils.state_extractors import (
    extract_ego_acceleration, extract_ego_jerk, extract_ego_yaw_rate)
from nuplan.planning.simulation.observation.idm.utils import is_agent_ahead

# esmini's cars from its catalog: 5.04 m by 2.0 m, the box's center 1.4 m
# ahead of the rear axle.
LENGTH, WIDTH, CENTER = 5.04, 2.0, 1.4
STEP = 0.05


def vehicle(length=LENGTH, width=WIDTH, center=CENTER):
    return VehicleParameters(vehicle_name='car', vehicle_type='car',
                             width=width, front_length=center + length / 2,
                             rear_length=length / 2 - center, wheel_base=2.98,
                             cog_position_from_rear_axle=center, height=1.5)


# nuPlan's comfort bounds, from its simulation_metric configuration: each
# signal's least and greatest value.
COMFORT = {'lon_acceleration': (-4.05, 2.40), 'lat_acceleration': (-4.89, 4.89),
           'lon_jerk': (-4.13, 4.13), 'jerk': (-8.37, 8.37),
           'yaw_rate': (-0.95, 0.95), 'yaw_acceleration': (-1.93, 1.93)}
LEAST_MIN_TTC = 0.95


def ego_states(times, ego, box):
    """nuPlan's ego states from the ego's rows, each its rear axle's pose and
    speed, and its acceleration from the change in speed and heading."""
    speed = np.array([float(e['speed']) for e in ego])
    heading = np.unwrap([float(e['heading']) for e in ego])
    along = np.concatenate([[0.0], np.diff(speed) / STEP])
    yaw_rate = np.concatenate([[0.0], np.diff(heading) / STEP])
    across = speed * yaw_rate
    states = [EgoState.build_from_rear_axle(
        rear_axle_pose=StateSE2(float(e['x']), float(e['y']), float(e['heading'])),
        rear_axle_velocity_2d=StateVector2D(speed[i], 0.0),
        rear_axle_acceleration_2d=StateVector2D(along[i], across[i]),
        tire_steering_angle=0.0,
        time_point=TimePoint(int(round(times[i] * 1e6))),
        vehicle_parameters=vehicle(*box)) for i, e in enumerate(ego)]
    return states, speed, along, across


def comfort_signals(states):
    return {'lon_acceleration': extract_ego_acceleration(states, 'x'),
            'lat_acceleration': extract_ego_acceleration(states, 'y'),
            'lon_jerk': extract_ego_jerk(states, 'x'),
            'jerk': extract_ego_jerk(states, 'magnitude'),
            'yaw_rate': extract_ego_yaw_rate(states),
            'yaw_acceleration': extract_ego_yaw_rate(states, deriv_order=2,
                                                     poly_order=3)}


def time_to_collision(state, speed, others):
    """nuPlan's time to collision at a sample, `others` each a center pose,
    speed and length and width; the tracks those ahead of the ego."""
    tracks = [(c, v, l, w) for c, v, l, w in others
              if is_agent_ahead(state.rear_axle, c)]
    if not tracks:
        return None
    return _compute_time_to_collision_at_timestamp(
        timestamp=state.time_point.time_us, ego_state=state,
        ego_speed=np.array(speed),
        tracks_poses=np.array([[c.x, c.y, c.heading] for c, _, _, _ in tracks]),
        tracks_speed=np.array([v for _, v, _, _ in tracks]),
        tracks_boxes=np.array([OrientedBox(c, l, w, 1.5) for c, _, l, w in tracks]),
        timestamps_at_fault_collisions=[], time_step_size=0.1,
        time_horizon=3.0, stopped_speed_threshold=5e-3)


def center_of(row):
    h = float(row['heading'])
    c = float(row['center'])
    return StateSE2(float(row['x']) + c * math.cos(h),
                    float(row['y']) + c * math.sin(h), h)


def permutations(here):
    runs = collections.defaultdict(lambda: collections.defaultdict(dict))
    with open(os.path.join(here, 'esmini_permutations.csv')) as f:
        for row in csv.DictReader(f):
            runs[int(row['permutation'])][float(row['time'])][row['entity']] = row
    with open(os.path.join(here, 'nuplan_verdicts.csv'), 'w') as out, \
            open(os.path.join(here, 'nuplan_permutations.csv'), 'w') as series:
        out.write('permutation,samples,comfortable,min_ttc,ttc_within_bound,min_gap\n')
        series.write('permutation,time,ttc,gap\n')
        for permutation, steps in sorted(runs.items()):
            # Sampled after each step: the initial row left out.
            times = sorted(steps)[1:]
            ego = [steps[t]['Ego'] for t in times]
            box = tuple(float(ego[0][k]) for k in ('length', 'width', 'center'))
            states, speed, _, _ = ego_states(times, ego, box)
            signals = comfort_signals(states)
            comfortable = all(
                WithinBoundMetricBase._compute_within_bound(
                    TimeSeries(unit='', time_stamps=[s.time_us for s in states],
                               values=list(signals[name])), least, most)
                for name, (least, most) in COMFORT.items())
            ttcs = []
            gaps = []
            for i, (t, state) in enumerate(zip(times, states)):
                gap = math.inf
                others = []
                for name, row in steps[t].items():
                    if name == 'Ego':
                        continue
                    others.append((center_of(row), float(row['speed']),
                                   float(row['length']), float(row['width'])))
                    track = OrientedBox(center_of(row), float(row['length']),
                                        float(row['width']), 1.5)
                    gap = min(gap, state.car_footprint.oriented_box.geometry.distance(
                        track.geometry))
                ttc = time_to_collision(state, speed[i], others)
                ttcs.append(np.inf if ttc is None else ttc)
                gaps.append(gap)
                series.write('%d,%.2f,%s,%.17g\n' % (
                    permutation, t, '' if ttc is None else '%.17g' % ttc, gap))
            least = float(np.min(ttcs))
            within = bool(np.all(LEAST_MIN_TTC < np.array(ttcs)))
            out.write('%d,%d,%s,%s,%s,%.17g\n' % (
                permutation, len(states), str(comfortable).lower(),
                '' if math.isinf(least) else '%.17g' % least,
                str(within).lower(), min(gaps)))
            print('permutation', permutation, len(states), 'samples')


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    runs = collections.defaultdict(lambda: collections.defaultdict(dict))
    with open(os.path.join(here, 'esmini_scenarios.csv')) as f:
        for row in csv.DictReader(f):
            runs[row['scenario']][float(row['time'])][row['entity']] = row
    with open(os.path.join(here, 'nuplan_metrics.csv'), 'w') as out:
        out.write('scenario,time,x,y,heading,speed,acceleration_x,acceleration_y,'
                  'lon_acceleration,lat_acceleration,lon_jerk,jerk,yaw_rate,'
                  'yaw_acceleration,ttc\n')
        for scenario, steps in runs.items():
            times = sorted(steps)
            ego = [steps[t]['Ego'] for t in times]
            states, speed, along, across = ego_states(
                times, ego, (LENGTH, WIDTH, CENTER))
            signals = comfort_signals(states)
            for i, (t, state) in enumerate(zip(times, states)):
                others = []
                for name, row in steps[t].items():
                    if name == 'Ego':
                        continue
                    h = float(row['heading'])
                    center = StateSE2(float(row['x']) + CENTER * math.cos(h),
                                      float(row['y']) + CENTER * math.sin(h), h)
                    others.append((center, float(row['speed']), LENGTH, WIDTH))
                ttc = time_to_collision(state, speed[i], others)
                e = ego[i]
                out.write('%s,%.2f,%s,%s,%s,%.17g,%.17g,%.17g,%s,%s\n' % (
                    scenario, t, e['x'], e['y'], e['heading'], speed[i], along[i], across[i],
                    ','.join('%.17g' % signals[name][i] for name in COMFORT),
                    '' if ttc is None else '%.17g' % ttc))
            print(scenario, len(states), 'samples')
    permutations(here)


if __name__ == '__main__':
    main()
