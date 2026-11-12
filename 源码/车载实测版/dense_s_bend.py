#!/usr/bin/env python3
"""Dense cone reference, half-width offset, speed PI and steering PD."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
for _sub in ('core', 'planners/dense', 'planners/corridor'):
    _path = str(HERE/_sub)
    if _path not in sys.path:
        sys.path.insert(0, _path)

from geometry import forward_path_points, relative_to_path, swept_clearance, wrap
from dense_reference import AnglePD, DensePlanner, SpeedPI
from progressive import ProgressivePlanner
from run_record import PoseTracker, RunRecorder, stamp_ns
from vehicle_io import VehicleIO, rclpy, SignalHandlerOptions


def stopping_distance(speed):
    return max(.28, .12 + .25*abs(speed) + .25*speed*speed)


def speed_for_distance(distance):
    return (math.sqrt(1 + 16*(max(.28, distance)-.12)) - 1) / 2


# The lowest equivalent coast deceleration in the four recorded stops was
# 0.68 m/s². The reaction interval spans one scan and one control tick.
COAST_DECEL = .65
BRAKE_REACTION = .15
STOP_RESERVE = .06
MIN_OBSTACLE_DRIVE_SPEED = .35


def obstacle_stop_distance(speed):
    return STOP_RESERVE + speed*speed/(2*COAST_DECEL)


def obstacle_speed_limit(distance):
    available = max(0., distance-STOP_RESERVE)
    return math.sqrt((COAST_DECEL*BRAKE_REACTION)**2 + 2*COAST_DECEL*available) - COAST_DECEL*BRAKE_REACTION


def sharp_right_target(route):
    bearing = math.atan2(route.target[1], route.target[0])
    return bearing < -math.radians(30.) and route.heading < -math.radians(25.)


def unsupported_preview(route, reference):
    bearing = math.atan2(route.target[1], route.target[0])
    if abs(bearing) <= math.radians(30.):
        return False
    if (abs(bearing) > math.radians(40.)
            and abs(wrap(bearing-route.heading)) > math.radians(25.)):
        return True
    if reference is None:
        return False
    if route.centers[0, 0] > 1.1 or math.hypot(*route.target) > 1.65:
        residual, _ = relative_to_path(np.asarray([route.target]), reference)
        return abs(float(residual[0])) > .25*route.width
    return False


class LowSpeedThrottle:
    """Limit throttle rise and allow for the measured delay in wheel response."""

    def __init__(self):
        self.encoder = None
        self.acceleration = 0.
        self.reset()

    def reset(self):
        self.pwm = 1500.
        self.started = None
        self.motion_seen = False
        self.integral = 0.

    def observe(self, speed, stamp):
        if stamp is None:
            self.encoder = None
            self.acceleration = 0.
            return
        if self.encoder is not None:
            before, previous = self.encoder
            dt = stamp-before
            if .004 <= dt <= .16:
                rate = max(-4.,min(4.,(speed-previous)/dt))
                self.acceleration += .5*(rate-self.acceleration)
        self.encoder = stamp,speed

    def update(self, speed, target, maximum, now, dt):
        if self.started is None:
            self.started = now
        dt = max(.005,min(.065,dt))
        self.motion_seen = self.motion_seen or speed >= .055
        prediction = speed + .19*max(0.,self.acceleration)
        error = target-prediction
        ceiling = min(1514.,maximum)
        if not self.motion_seen and abs(speed) < .045:
            ceiling = min(maximum,ceiling+min(2,math.floor(max(0.,now-self.started-2.))))
        proposed_integral = max(-.7,min(.7,self.integral+error*dt))
        floor = min(1504.,ceiling)
        demand = 1510.5 + 10.*error + 2.3*proposed_integral
        if floor <= demand <= ceiling:
            self.integral = proposed_integral
        demand = max(floor,min(ceiling,demand))
        self.pwm += max(-26.*dt,min(15.*dt,demand-self.pwm))
        return self.pwm


class DenseSBend(VehicleIO):
    def __init__(self, settings, observing=False):
        self.settings = settings
        self.max_steer = float(settings.get('max_steer_deg', 16.0))
        self.full_steer = float(settings.get('full_steer_deg', 65.0))
        self.follow_offset = float(settings.get('follow_offset_m', .35))
        self.corner_offset = float(settings.get('corner_offset_m', .45))
        self.launch_speed = float(settings.get('launch_speed_mps', .4))
        self.ramp_seconds = float(settings.get('speed_ramp_seconds', 3.0))
        self.minimum_pwm = float(settings.get('minimum_pwm', 1500.0))
        self.a_lat_bend = float(settings.get('a_lat_bend_mps2', 0.45))
        self.bend_curvature = None
        self.speed_ema = None
        self.planner_mode = str(settings.get('planner_mode', 'dense'))
        pairing_mode = settings.get('pairing_mode', 'legacy')
        self.paired = self.planner_mode == 'dense' and pairing_mode == 'nearest_pair'
        self.start_side = int(settings.get('start_side', 0))
        self.path_mode = str(settings.get('path_mode', 'center'))
        if (not 0 < settings['target_speed_mps'] <= 2.0
                or settings['distance_mode'] != 'half_width'
                or self.planner_mode not in ('dense', 'corridor')
                or pairing_mode not in ('legacy', 'nearest_pair')
                or self.path_mode not in ('center', 'hug')
                or self.start_side not in (-1, 0, 1)
                or not 0 < self.launch_speed <= settings['target_speed_mps']
                or self.ramp_seconds < 0
                or not 1500 <= self.minimum_pwm <= settings['maximum_pwm']
                or not .25 <= self.corner_offset <= .60
                or not 1500 <= settings['speed_feedforward_pwm'] <= settings['maximum_pwm'] <= 1600
                or not 1500 < settings['launch_pwm'] <= settings['maximum_pwm']
                or not .20 <= self.follow_offset <= .60
                or not 10.0 <= self.max_steer <= self.full_steer <= 90.0):
            raise ValueError('Invalid dense-reference configuration')
        super().__init__(observing=observing)
        self.first_move_at = None
        self.coast_frames = 0
        self.left_lock_rejected_at = None
        self.hard_rejection = False
        self.last_pwm = 1500.0
        if self.paired:
            self.planner = ProgressivePlanner(self.car, settings)
        elif self.planner_mode == 'corridor':
            from corridor_planner import CorridorPlanner
            self.planner = CorridorPlanner(
                self.car, settings['lookahead_base_m'],
                settings['lookahead_speed_seconds'],
                settings['transition_extension_m'])
        else:
            self.planner = DensePlanner(
                self.car, settings['lookahead_base_m'],
                settings['lookahead_speed_seconds'],
                settings['transition_extension_m'],
                self.follow_offset, self.corner_offset, self.start_side)
        self.speed_pi = SpeedPI(settings['target_speed_mps'], settings['speed_kp'],
                                settings['speed_ki'], settings['speed_feedforward_pwm'],
                                settings['maximum_pwm'], self.minimum_pwm,
                                float(settings.get('speed_pwm_per_mps', 16.0)))
        self.angle_pd = AnglePD(settings['angle_kp'], settings['angle_kd'],
                                settings['angle_derivative_filter_seconds'], self.max_steer)
        self.route = None
        self.pair_missing_at = None
        self.pair_missing_distance = None
        self.pair_missing_frames = 0
        self.reference_pose = None
        self.reference_turning = False
        self.pose_tracker = PoseTracker()
        self.low_speed = LowSpeedThrottle()
        self.scan_pose = None
        self.scan_id = 0
        self.pi_at = self.pd_at = None
        self.preferred_angle = 0.
        self.clearance_state = 'waiting'
        self.steering_speed_limit = math.inf
        self.coast_for_obstacle = False
        self.obstacle_distance = None
        self.obstacle_coast_min = None
        self.revision = self.code_revision()
        self.recorder = None
        try:
            self.recorder = RunRecorder(HERE/'records'/'runs', self.car, settings, observing, self.revision)
            self.get_logger().info(f'Pose and LiDAR journal: {self.recorder.path}')
        except OSError as error:
            self.get_logger().error(f'Could not open run journal: {error}')
        mode = 'bilateral' if self.paired else self.planner_mode
        self.get_logger().info(f'Planner={mode}; speed target={self.speed_pi.target:.2f} m/s; steering=PD')

    def record(self, event):
        if self.recorder is None:
            return
        try:
            self.recorder.append(event)
        except OSError as error:
            self.get_logger().error(f'Run journal write failed: {error}')
            try:
                self.recorder.close()
            except OSError:
                pass
            self.recorder = None

    def on_imu(self, msg):
        super().on_imu(msg)
        if self.imu_at is None:
            self.pose_tracker.imu = None
            self.pose_tracker.invalidate()
        else:
            self.pose_tracker.update_imu(self.imu_at, stamp_ns(msg), self.yaw)

    def on_encoder(self, msg):
        super().on_encoder(msg)
        self.low_speed.observe(self.speed,self.encoder_at)
        if self.encoder_at is None:
            self.pose_tracker.invalidate()
        else:
            self.pose_tracker.update_encoder(self.encoder_at, stamp_ns(msg), self.speed)

    def on_scan(self, msg):
        super().on_scan(msg)
        self.scan_id += 1
        frame = self.frames[-1]
        self.scan_pose = self.pose_tracker.snapshot(frame['at'])
        self.record(dict(frame, kind='scan', scan_id=self.scan_id, pose=self.scan_pose,
                         frame_id=msg.header.frame_id, speed_mps=self.speed,
                         scan_time_s=float(msg.scan_time), time_increment_s=float(msg.time_increment)))

    def paired_reference(self, now):
        pose, prev = self.scan_pose, self.reference_pose
        if (pose is None or prev is None or self.last_dual is None
                or now-self.last_dual > .50 or pose['segment'] != prev['segment']):
            return None
        dx, dy = pose['x_m']-prev['x_m'], pose['y_m']-prev['y_m']
        a = prev['yaw_rad']
        translation = np.array([dx*math.cos(a)+dy*math.sin(a), -dx*math.sin(a)+dy*math.cos(a)])
        angle = wrap(pose['yaw_rad']-a)
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        return (self.reference_path-translation) @ rotation

    def update_paired_target(self, now):
        reference = self.paired_reference(now)
        start = time.monotonic()
        stamp = tuple(self.frames[-1]['stamp'])
        route = self.planner.plan(self.cones, self.speed, now, reference, self.points,
                                  pose=self.scan_pose, stamp=stamp)
        self.plan_ms = 1000*(time.monotonic()-start)
        self.route = route
        self.center_path = None if route is None else route.centers
        self.boundary_counts = (0, 0) if route is None else (len(route.left), len(route.right))
        if route is None:
            self.startup_frames = 0
            self.startup_path = None
            if self.pair_missing_at is None:
                self.pair_missing_at = now
                self.pair_missing_distance = self.distance
            if self.planner.reject != 'duplicate_scan':
                self.pair_missing_frames += 1
            self.route_mode = 'wait_bilateral'
            if not self.pair_gap_supported(now, reference):
                self.target = None
                self.clearance_state = 'no_confirmed_route'
        else:
            self.pair_missing_at = None
            self.pair_missing_frames = 0
            self.target = route.target
            self.lane_width = route.width
            self.route_mode = 'bilateral'
            if self.planner.corner.active is not None:
                self.route_mode = 'corner_'+self.planner.corner.event['direction']
            elif self.planner.corner.event and self.planner.corner.event['state'] == 'exit_reacquired':
                self.angle_pd.reset()
                self.pd_at = None
            if not self.ever_drove:
                previous = reference if reference is not None else self.startup_path
                consistent = False
                if previous is not None:
                    x = route.centers[:, 0]
                    overlap = (x >= previous[0, 0]) & (x <= previous[-1, 0]) & (x <= 1.4)
                    if np.count_nonzero(overlap) >= 3:
                        offset, _ = relative_to_path(route.centers[overlap], previous)
                        consistent = np.max(np.abs(offset)) <= .20
                self.startup_frames = min(3, self.startup_frames+1) if consistent else 1
                self.startup_path = route.centers
            self.last_dual = now
            self.reference_path = route.centers
            self.reference_pose = self.scan_pose
            self.reference_turning = self.planner.corner.active is not None
        self.record_plan(now, route, self.planner.last_row_stats, pair=self.planner.last_pair,
                         reject=self.planner.reject, preview=self.planner.preview,
                         revalidation=self.planner.revalidation, corner=self.planner.corner.event)

    def pair_gap_supported(self, now, reference):
        if (not self.ever_drove or self.target is None or reference is None
                or self.planner.corner.fault is not None
                or now-self.pair_missing_at >= .35+.15*abs(self.speed)
                or self.distance-self.pair_missing_distance >= .20+.20*abs(self.speed)):
            return False
        # Continue only along the pose-aligned part of the last verified fit.
        return bool(np.any(reference[:, 0] >= .28))

    def update_target(self, now):
        if self.paired:
            self.update_paired_target(now)
            return
        reference = None
        if self.ever_drove and self.last_dual is not None and now-self.last_dual <= .55:
            angle = wrap(self.yaw-self.reference_yaw)
            travel = self.distance-self.reference_distance
            translation = np.array([travel, 0.])
            if abs(angle) > 1e-6:
                translation = travel*np.array([math.sin(angle), 1-math.cos(angle)])/angle
            rotation = np.array([[math.cos(angle), -math.sin(angle)],
                                 [math.sin(angle), math.cos(angle)]])
            reference = (self.reference_path-translation) @ rotation
        started = time.monotonic()
        left_locked = getattr(self.planner, 'side', None) == 1
        planner_state = vars(self.planner).copy()
        route = self.planner.plan(self.cones, self.speed, now, reference)
        if (route is None and self.ever_drove and self.coast_frames < 8
                and self.left_lock_rejected_at is None):
            # A single lost frame at the next corner entrance used to stop the
            # car immediately. Coast along the last heading for up to 0.8 s so
            # it can reach the corner, then stop if the fit does not recover.
            route = self.planner.hold(self.speed)
            if route is not None:
                self.coast_frames += 1
        elif route is not None:
            self.coast_frames = 0
        self.plan_ms = 1000*(time.monotonic()-started)
        sharp_right = left_locked and route is not None and sharp_right_target(route)
        off_path = route is not None and unsupported_preview(route, reference)
        # DensePlanner's straight hold uses its synthetic centerline as its baseline.
        lost_fit = route is None or route.baseline is route.centers
        stale_fit = lost_fit and self.ever_drove and (
            self.last_dual is None or now-self.last_dual >= .25
            or self.distance-self.reference_distance >= .15)
        if sharp_right or off_path or stale_fit or (self.left_lock_rejected_at is not None and lost_fit):
            row_stats = self.planner.last_row_stats
            vars(self.planner).update(planner_state)
            reason = 'unsupported_preview' if off_path else 'left_lock' if sharp_right else 'lost_fit'
            self.reject_left_lock_frame(now, route, reason, row_stats)
            return
        self.left_lock_rejected_at = None
        self.hard_rejection = False
        self.route = route
        route = self.route
        self.center_path = None if route is None else route.centers
        self.target = None if route is None else route.target
        self.boundary_counts = (0, 0) if route is None else (len(route.left), len(route.right))
        self.route_mode = 'none' if route is None else ('dense_left' if route.side > 0 else 'dense_right')
        if not self.ever_drove:
            consistent = route is not None and self.startup_path is not None
            if consistent:
                residual, _ = relative_to_path(route.centers, self.startup_path)
                consistent = np.max(np.abs(residual)) <= .2*route.width
            self.startup_frames = (min(3, self.startup_frames+1) if consistent
                                   else (1 if route is not None else 0))
            self.startup_path = None if route is None else route.centers.copy()
        if route is not None:
            self.lane_width = route.width
        if not lost_fit:
            self.last_dual = now
            self.reference_path = route.centers.copy()
            self.reference_distance, self.reference_yaw = self.distance, self.yaw
        self.record_plan(now, route, self.planner.last_row_stats)

    def record_plan(self, now, route, row_stats, pair=None, reject=None,
                    preview=None, revalidation=None, corner=None):
        event = dict(kind='dense_plan', at=now, scan_id=self.scan_id, plan_ms=self.plan_ms,
                     side=None if route is None else route.side,
                     width=None if route is None else route.width,
                     lookahead=None if route is None else route.lookahead,
                     heading=None if route is None else route.heading,
                     target=None if route is None else route.target, cones=self.cones.tolist(),
                     baseline=None if route is None else route.baseline.tolist(),
                     centers=None if route is None else route.centers.tolist(),
                     left=None if route is None else route.left.tolist(),
                     right=None if route is None else route.right.tolist(),
                     row_stats=row_stats, pair=pair, reject=reject,
                     far_preview=preview, revalidation=revalidation,
                     corner=corner,
                     distance=self.distance, yaw=self.yaw, speed=self.speed,
                     inputs=self.input_state(time.monotonic()))
        self.events.append(event)
        self.record(event)

    def reject_left_lock_frame(self, now, route, reason, row_stats):
        if self.left_lock_rejected_at is None:
            self.left_lock_rejected_at = now
            self.angle_pd.reset()
            self.pd_at = self.first_move_at = None
        self.hard_rejection = self.hard_rejection or reason != 'lost_fit'
        self.route_mode = 'left_lock_rejected' if self.hard_rejection else 'wait_near_fit'
        self.events.append(dict(kind='left_lock_rejected', at=now,
                                reason=reason,
                                target=None if route is None else route.target,
                                heading=None if route is None else route.heading,
                                distance=self.distance, yaw=self.yaw, speed=self.speed,
                                row_stats=row_stats, plan_ms=self.plan_ms))
        # choose_steering keeps the throttle neutral while new scans are checked.
        # Clearing the target ends the run once the recovery window expires.
        elapsed = now-self.left_lock_rejected_at
        expired = elapsed >= .75
        if self.hard_rejection:
            expired = elapsed >= .25 or self.distance-self.reference_distance >= .15
        if expired or self.last_dual is None:
            self.route = self.target = self.center_path = None
            self.boundary_counts = (0, 0)

    def choose_steering(self, step):
        self.steering_speed_limit = math.inf
        self.clearance_state = 'waiting'
        self.coast_for_obstacle = False
        self.obstacle_distance = None
        if self.pair_missing_at is not None:
            now = time.monotonic()
            if not self.pair_gap_supported(now, self.paired_reference(now)):
                self.target = None
                self.clearance_state = 'no_confirmed_route'
                return self.servo, None, False
            offset = self.servo-self.car.servo_center
            horizon = self.obstacle_horizon()
            raw = self.raw_arc_clearance(offset, horizon, self.reference_turning)
            state, limit, coast = self.assess_obstacle(raw, horizon)
            self.set_obstacle_decision(raw, horizon, coast)
            if limit <= 0.:
                self.clearance_state = 'obstacle_during_pair_gap'
                return self.servo, None, False
            if self.pair_missing_frames <= 1 and now-self.pair_missing_at < .14:
                self.clearance_state = 'single_pair_gap' if state == 'clear' else state
            else:
                self.clearance_state = 'brief_pair_gap' if state == 'clear' else state
                limit = min(limit, max(.65, abs(self.speed)-.30))
            self.steering_speed_limit = limit
            return self.servo, None, True
        if self.left_lock_rejected_at is not None:
            return self.servo, None, False
        if self.route is None:
            if not self.observing and self.cones is not None and len(self.cones):
                left = self.cones[self.cones[:, 1] > .2]
                right = self.cones[self.cones[:, 1] < -.2]
                if len(left) and len(right):
                    d_left = float(np.max(np.hypot(left[:, 0], left[:, 1])))
                    d_right = float(np.max(np.hypot(right[:, 0], right[:, 1])))
                    error = d_left-d_right
                    preferred = max(-self.max_steer, min(self.max_steer, 25.0*error))
                    previous = self.servo-self.car.servo_center
                    command = previous+max(-step, min(step, preferred-previous))
                    return self.car.servo_center+command, 0., True
            return None, None, False
        now = time.monotonic()
        dt = .05 if self.pd_at is None else now-self.pd_at
        self.pd_at = now
        corner = getattr(self.planner, 'corner_target', None)
        if getattr(self.planner, 'right_turn', False) and corner is not None:
            self.target = np.asarray(corner, dtype=float)
        error = wrap(math.atan2(self.target[1], self.target[0])
                     + self.settings['preview_heading_weight']*self.route.heading)
        turning = self.paired and self.planner.corner.active is not None
        if turning:
            x,y = self.target
            curvature = 2*y/(x*x+y*y)
            preferred = float(np.clip(curvature*self.car.radius*self.full_steer,
                                      -self.max_steer,self.max_steer))
            # 65 deg/s needs 0.55 s to reach the 35.5 deg the arc wants -- at
            # 1.33 m/s that is 0.73 m of travel, which is exactly how far the
            # car drifted (0.316 m) before corner_tracking_lost tripped. The
            # corner target is a pre-computed arc, so slew the wheel faster
            # here than in normal tracking where hunting matters.
            step = 200.*max(.001,min(.06,dt))
        else:
            if self.paired:
                offset, _ = relative_to_path(np.zeros((1, 2)), self.route.centers)
                off_center = float(offset[0])
                excess = max(0., abs(off_center) - .15)
                correction = math.copysign(math.atan2(.75 * excess, .65 + abs(self.speed)), off_center)
                error -= correction
            preferred = self.angle_pd.update(error, dt)
        if getattr(self.planner, 'right_turn', False):
            if corner is None or float(np.linalg.norm(self.target)) < 1.0:
                preferred = -self.max_steer
        self.preferred_angle = preferred
        if self.paired:
            return self.choose_paired_arc(preferred, step, turning)
        car = self.car
        previous = self.servo-car.servo_center
        # Free distance the swept footprint wants ahead of the car. The old
        # v^2/0.9 braking model asked for 2.14 m at 1.2 m/s, more than the S
        # corridor itself, so the car held neutral through every bend. This is
        # deliberately less conservative: 0.90 m at 1.2 m/s, 0.37 m at 0.6.
        required = stopping_distance(self.speed)
        horizon = min(5.-car.front-car.margin+car.laser_x, max(.55, required+.08))

        def clearance(offset):
            curvature = (offset/self.full_steer)/car.radius
            points = forward_path_points(self.points, offset, car)
            return swept_clearance(points, curvature, car, horizon)

        corrections = [0., -4., 4., -8., 8., -12., 12.]
        if self.max_steer > 20.:
            corrections += [-16., 16., -20., 20.]
        presteer = None
        for correction in corrections:
            goal = preferred+correction
            if abs(goal) > self.max_steer or clearance(goal) < required:
                continue
            command = previous+max(-step, min(step, goal-previous))
            result = (car.servo_center+command, correction, True)
            if goal == command or clearance(command) >= required:
                return result
            if presteer is None:
                presteer = (result[0], correction, False)
        return presteer or (None, None, False)

    def raw_arc_clearance(self, offset, horizon, turning=False):
        car = self.car
        curvature = offset/(self.full_steer*car.radius)
        if not turning:
            points = forward_path_points(self.points, offset, car)
            return swept_clearance(points, curvature, car, horizon, fine_resolution=True)
        reach = horizon + car.front + car.margin
        points = self.points[(self.points[:, 0] >= -car.rear-car.margin)
                             & (self.points[:, 0] <= reach)
                             & (np.abs(self.points[:, 1]) <= reach)]
        return swept_clearance(points, curvature, car, horizon, rear_wheel_padding=True, fine_resolution=True)

    def obstacle_horizon(self):
        coast = obstacle_stop_distance(self.speed)
        return min(5.-self.car.front-self.car.margin+self.car.laser_x,
                   max(1.20, coast + BRAKE_REACTION*abs(self.speed) + .55))

    def assess_obstacle(self, distance, horizon):
        if distance >= horizon:
            return 'clear', math.inf, False
        if distance <= obstacle_stop_distance(self.speed):
            return 'obstacle_in_stopping_distance', 0., True
        limit = obstacle_speed_limit(distance)
        # The drivetrain cannot reliably creep below this speed under PWM.
        closing = self.obstacle_coast_min is not None and distance < self.obstacle_coast_min+.15
        coast = closing or abs(self.speed) >= limit or limit < MIN_OBSTACLE_DRIVE_SPEED
        return ('coasting_for_obstacle' if coast else 'slowing_for_obstacle'), limit, coast

    def set_obstacle_decision(self, distance, horizon, coast):
        self.obstacle_distance = distance if distance < horizon else None
        self.coast_for_obstacle = coast
        if coast:
            self.obstacle_coast_min = (distance if self.obstacle_coast_min is None
                                       else min(distance,self.obstacle_coast_min))
        else:
            self.obstacle_coast_min = None

    def choose_paired_arc(self, preferred, step, turning):
        car = self.car
        previous = self.servo - car.servo_center
        required = stopping_distance(self.speed)
        lidar_horizon = self.obstacle_horizon()
        near_horizon = min(.65, required)
        assessments = {}

        def assess(offset):
            if offset in assessments:
                return assessments[offset]
            curvature = offset/(self.full_steer*car.radius)
            raw = self.raw_arc_clearance(offset, lidar_horizon, turning)
            state, limit, coast = self.assess_obstacle(raw, lidar_horizon)
            if limit > 0.:
                if turning:
                    physical, center = near_horizon, near_horizon
                    if not self.planner.corner.contains_arc(curvature, near_horizon):
                        limit = min(limit, .85)
                else:
                    physical, center = self.corridor_reach(curvature, near_horizon)
                if physical < .28 - 1e-6:
                    state, limit = 'footprint_outside_corridor', 0.
                else:
                    if min(physical, center) < near_horizon:
                        limit = min(limit, max(.65, speed_for_distance(min(physical, center))))
                    if state == 'clear' and math.isfinite(limit):
                        state = 'slowing_for_clearance'
            result = state, limit, raw, coast
            assessments[offset] = result
            return result

        corrections = (0., -4., 4., -8., 8.) if turning else (0., -4., 4., -8., 8., -12., 12., -16., 16., -20., 20.)
        presteer = None
        for correction in corrections:
            goal = preferred + correction
            if abs(goal) > self.max_steer or assess(goal)[1] <= 0.:
                continue
            command = previous + float(np.clip(goal-previous, -step, step))
            state, limit, raw, coast = assess(command)
            if limit > 0.:
                self.clearance_state = state
                self.steering_speed_limit = limit
                self.set_obstacle_decision(raw, lidar_horizon, coast)
                return car.servo_center+command, correction, True
            if presteer is None:
                presteer = car.servo_center+command, correction, False
        state, _, raw, coast = assess(preferred)
        self.clearance_state = 'presteer_required' if presteer else state
        self.set_obstacle_decision(raw, lidar_horizon, coast)
        return presteer or (None, None, False)

    def corridor_reach(self, curvature, horizon):
        centers = self.route.centers
        distance = np.linspace(0., horizon, 16)
        yaw = distance*curvature
        xy = np.column_stack((distance*np.sinc(yaw/np.pi),
                              .5*distance*yaw*np.sinc(yaw/(2*np.pi))**2))
        valid = (xy[:, 0] >= centers[0, 0]-.90) & (xy[:, 0] <= centers[-1, 0])
        offset, tangent = relative_to_path(xy, centers)
        pair = self.planner.last_pair
        relative_yaw = yaw-np.arctan2(tangent[:, 1], tangent[:, 0])
        sine = np.sin(relative_yaw)
        half_body = self.car.half_width * np.abs(np.cos(relative_yaw))
        left_reach = half_body + np.maximum(self.car.front*sine, -self.car.rear*sine)
        right_reach = half_body + np.maximum(self.car.rear*sine, -self.car.front*sine)
        half_lane = pair['width_min']/2 - self.car.margin
        recovery_limit = max(.28, abs(float(offset[0]))+.035)
        physical = valid & (offset+left_reach <= half_lane) & (right_reach-offset <= half_lane)
        centered = np.abs(offset) <= recovery_limit

        def reach(mask):
            first = np.flatnonzero(~mask)
            return horizon if not len(first) else float(distance[max(0, first[0]-1)])

        return reach(physical), reach(physical & centered)

    def _speed_target(self, now):
        if self.first_move_at is None:
            return self.launch_speed
        if self.ramp_seconds <= 0:
            return self.settings['target_speed_mps']
        fraction = min(1.0, (now-self.first_move_at)/self.ramp_seconds)
        return self.launch_speed + (self.settings['target_speed_mps']-self.launch_speed)*fraction

    def publish(self, pwm, servo):
        # The inherited lifecycle supplies drive/neutral intent.
        preserve = (self.clearance_state == 'single_pair_gap' and self.last_pwm > 1500.)
        if pwm > 1500 and not self.observing and preserve:
            pwm = self.last_pwm
            self.pi_at = time.monotonic()
            self.pwm = pwm
        elif pwm > 1500 and not self.observing:
            now = time.monotonic()
            if self.first_move_at is None:
                self.first_move_at = now
            target = min(self._speed_target(now), self.steering_speed_limit)
            if getattr(self.planner, 'tight_latch', False):
                target = min(target, 1.1)
            if getattr(self.planner, 'right_turn', False):
                target = min(target, .9)
            # A dropped-pair frame now keeps driving with route=None, so the
            # support cap must not assume last_pair/route exist (it crashed
            # with TypeError on the 23:31 run).
            if (self.paired and self.route is not None
                    and self.planner.last_pair is not None):
                support = self.planner.last_pair['supported_ahead_m']
                preview = self.planner.preview
                if (preview is not None and abs(servo-self.car.servo_center) < 11.
                        and abs(self.route.heading) < math.radians(12.)):
                    # A bilateral preview may inform longitudinal speed.
                    # Only re-observed near points supply the steering target.
                    support = max(support, preview['supported_x_m'])
                    curvature = (servo-self.car.servo_center)/(self.full_steer*self.car.radius)
                    ahead = self.points[(self.points[:,0] >= .40) & (self.points[:,0] <= 2.40)
                                        & (np.abs(self.points[:,1]) <= 1.35)]
                    support = min(support, swept_clearance(ahead, curvature, self.car, 2.0))
                available = max(.28, support - .12)
                # Invert the existing swept-clearance distance model.
                supported_speed = speed_for_distance(available)
                half = self.route.curvature[:len(self.route.curvature)//2]
                # max() turned one wiggle at the far end of a fitted row into a
                # 1.5-1.9 m radius and slammed the speed cap: that is the stutter
                # felt in the second half of the 00:50 run. A percentile keeps a
                # genuine sustained bend while ignoring the spike.
                curvature = float(np.percentile(np.abs(half), 75))
                # The fitted rows swap as cones enter and leave view, so the
                # curvature estimate itself jumps (0.42 -> 0.17 -> 0.45 in
                # 0.2 m on the 00:50 run). Smooth it before using it as a cap.
                self.bend_curvature = (curvature if self.bend_curvature is None
                                       else .35*curvature+.65*self.bend_curvature)
                # 0.65 m/s^2 was optimistic: the car measures ~0.57, so it slid
                # 0.49 m wide through a 1.9 m bend. 0.45 leaves margin.
                target = min(target, supported_speed,
                             math.sqrt(self.a_lat_bend/max(.1, self.bend_curvature)))
                # Start slowing as soon as a corner is being confirmed, not
                # only once it is active: the car otherwise enters the arc at
                # straight-line speed (1.33 m/s) and cannot hold radius.
                turn = self.planner.corner.active or self.planner.corner.candidate
                if turn is not None:
                    target = min(target, self.planner.corner.speed,
                                 math.sqrt(self.a_lat_bend*turn.radius))
                elif self.bend_curvature >= .12:
                    target = min(target, self.planner.corner.speed)
            # supported_ahead swings with the end of the fitted row (0.70 to
            # 1.56 m inside two seconds on the 23:55 run), so the raw cap made
            # the throttle hunt and read as surging. Tighten instantly, release
            # slowly: safety keeps the fast path down, comfort keeps the slow
            # path up.
            # Slew the reference in physical time. Slow down faster than we
            # accelerate; the swept-footprint gate handles immediate stops.
            if self.speed_ema is None:
                self.speed_ema = target
            dt = .05 if self.pi_at is None else min(.10, now-self.pi_at)
            self.speed_ema += float(np.clip(target-self.speed_ema, -3.*dt, 1.5*dt))
            target = self.speed_ema
            self.speed_pi.target = target
            self.pi_at = now
            if self.coast_for_obstacle:
                pwm = 1500.
            elif self.paired and target <= .55:
                self.speed_pi.reset()
                pwm = self.low_speed.update(self.speed,target,self.settings['maximum_pwm'],now,dt)
            else:
                self.low_speed.reset()
                if self.paired:
                    predicted = self.speed + .10*self.low_speed.acceleration
                    target_pwm = self.speed_pi.update(predicted, dt)
                    if abs(self.speed) < .05:
                        target_pwm = min(target_pwm, self.settings['launch_pwm'], 1518.)
                    pwm = self.last_pwm + float(np.clip(target_pwm-self.last_pwm, -90.*dt, 42.*dt))
                else:
                    pi_pwm = self.speed_pi.update(self.speed, dt)
                    target_pwm = self.settings['launch_pwm'] if abs(self.speed) < .05 else pi_pwm
                    pwm = (self.last_pwm+max(-6.,min(6.,target_pwm-self.last_pwm))
                           if target_pwm > 1500. else 1500.)
            self.last_pwm = pwm
            self.pwm = pwm
        elif hasattr(self, 'speed_pi'):
            self.speed_pi.reset()
            self.low_speed.reset()
            self.last_pwm = 1500.0
            self.pi_at = None
        super().publish(pwm, servo)
        self.events[-1].update(speed_mps=self.speed, target_mps=self.speed_pi.target if pwm > 1500 else 0.,
                               clearance_state=self.clearance_state, pair_missing_frames=self.pair_missing_frames,
                               obstacle_distance_m=self.obstacle_distance,
                               obstacle_stop_distance_m=obstacle_stop_distance(self.speed) if self.paired else None,
                               obstacle_reaction_m=BRAKE_REACTION*abs(self.speed) if self.paired else None)
        self.record(self.events[-1])

    def status(self, now, reason):
        due = reason != self.last_status or now-self.last_log >= .5
        super().status(now, reason)
        if due and self.route is not None:
            self.get_logger().info(f'baseline={self.route_mode} width={self.route.width:.3f}m '
                                   f'distance={self.route.width/2:.3f}m preview={self.route.lookahead:.3f}m '
                                   f'v_target={self.speed_pi.target:.2f}m/s PD={self.preferred_angle:+.2f}deg')

    @staticmethod
    def code_revision():
        files = {
            'dense_reference.py': HERE/'planners/dense/dense_reference.py',
            'pairing.py': HERE/'planners/dense/pairing.py',
            'progressive.py': HERE/'planners/dense/progressive.py',
            'corner_transition.py': HERE/'planners/dense/corner_transition.py',
            'dense_s_bend.py': HERE/'dense_s_bend.py',
            'dense_settings.json': HERE/'dense_settings.json',
            'run_dense.sh': HERE/'run_dense.sh',
            'vehicle_io.py': HERE/'core/vehicle_io.py',
            'geometry.py': HERE/'core/geometry.py',
            'run_record.py': HERE/'core/run_record.py',
        }
        return {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in files.items()}

    def save_record(self):
        journal = None
        if self.recorder is not None:
            journal = str(Path(self.recorder.path).relative_to(HERE))
            self.record(dict(kind='end', at=time.monotonic(), stop=self.stop_reason or 'manual'))
            if self.recorder is not None:
                self.recorder.close()
                self.recorder = None
        record = dict(mode='observe' if self.observing else 'drive',
                      stop=self.stop_reason or 'manual', settings=self.settings,
                      revision=self.revision, pose_journal=journal,
                      final_inputs=self.input_state(time.monotonic()),
                      frames=list(self.frames), events=list(self.events))
        import tempfile
        with tempfile.NamedTemporaryFile('w', dir=HERE, prefix='dense_last_run.', suffix='.tmp',
                                         encoding='utf-8', delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(record, stream, ensure_ascii=False, allow_nan=False)
        temporary.replace(HERE/'dense_last_run.json')
        if journal is not None:
            print(f'Recorded {journal}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observe', action='store_true')
    args = parser.parse_args()
    settings = json.loads((Path(__file__).resolve().parent/'dense_settings.json').read_text(encoding='utf-8'))
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = DenseSBend(settings, observing=args.observe)
    interrupted = False

    def request_stop(signum, frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not interrupted and node.stop_reason is None:
            rclpy.spin_once(node, timeout_sec=.05)
        print(f'STOP {node.stop_reason or "manual"} distance={node.distance:.2f}m', flush=True)
    finally:
        if not node.observing and rclpy.ok() and (node.ever_drove or node.count_publishers('/car_cmd_vel') <= 1):
            for _ in range(6):
                node.neutral()
                time.sleep(.05)
        try:
            node.save_record()
            print('Recorded dense_last_run.json', flush=True)
        except Exception as error:
            print(f'Recording failed: {error}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
