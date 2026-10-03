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
from nuplan.planning.metrics.evaluation_metrics.common.time_to_collision_within_bound import (
    _compute_time_to_collision_at_timestamp)
from nuplan.planning.metrics.utils.state_extractors import (
    extract_ego_acceleration, extract_ego_jerk, extract_ego_yaw_rate)
from nuplan.planning.simulation.observation.idm.utils import is_agent_ahead

# esmini's cars from its catalog: 5.04 m by 2.0 m, the box's center 1.4 m
# ahead of the rear axle.
LENGTH, WIDTH, CENTER = 5.04, 2.0, 1.4
STEP = 0.05


def vehicle():
    return VehicleParameters(vehicle_name='car', vehicle_type='car',
                             width=WIDTH, front_length=CENTER + LENGTH / 2,
                             rear_length=LENGTH / 2 - CENTER, wheel_base=2.98,
                             cog_position_from_rear_axle=CENTER, height=1.5)


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
                vehicle_parameters=vehicle()) for i, e in enumerate(ego)]
            lon = extract_ego_acceleration(states, 'x')
            lat = extract_ego_acceleration(states, 'y')
            lon_jerk = extract_ego_jerk(states, 'x')
            jerk = extract_ego_jerk(states, 'magnitude')
            rate = extract_ego_yaw_rate(states)
            yaw_acceleration = extract_ego_yaw_rate(states, deriv_order=2, poly_order=3)
            for i, (t, state) in enumerate(zip(times, states)):
                tracks = []
                for name, row in steps[t].items():
                    if name == 'Ego':
                        continue
                    h = float(row['heading'])
                    center = StateSE2(float(row['x']) + CENTER * math.cos(h),
                                      float(row['y']) + CENTER * math.sin(h), h)
                    if is_agent_ahead(state.rear_axle, center):
                        tracks.append((center, float(row['speed'])))
                ttc = None
                if tracks:
                    ttc = _compute_time_to_collision_at_timestamp(
                        timestamp=state.time_point.time_us, ego_state=state,
                        ego_speed=np.array(speed[i]),
                        tracks_poses=np.array([[c.x, c.y, c.heading] for c, _ in tracks]),
                        tracks_speed=np.array([s for _, s in tracks]),
                        tracks_boxes=np.array([OrientedBox(c, LENGTH, WIDTH, 1.5) for c, _ in tracks]),
                        timestamps_at_fault_collisions=[], time_step_size=0.1,
                        time_horizon=3.0, stopped_speed_threshold=5e-3)
                e = ego[i]
                out.write('%s,%.2f,%s,%s,%s,%.17g,%.17g,%.17g,%s,%s\n' % (
                    scenario, t, e['x'], e['y'], e['heading'], speed[i], along[i], across[i],
                    ','.join('%.17g' % v for v in (lon[i], lat[i], lon_jerk[i], jerk[i],
                                                   rate[i], yaw_acceleration[i])),
                    '' if ttc is None else '%.17g' % ttc))
            print(scenario, len(states), 'samples')


if __name__ == '__main__':
    main()
