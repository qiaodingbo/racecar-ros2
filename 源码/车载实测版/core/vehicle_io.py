"""Shared vehicle I/O and control lifecycle; planning is provided by the module."""
from dataclasses import dataclass
from collections import deque
import json
import math
import time
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, LaserScan
from geometry import ObstacleHold, scan_geometry

@dataclass(frozen=True)
class Car:
    width: float = 1.50
    # Reverted to 0.83: a 1.02 calibration plans a 1.845 m arc that the L-lane
    # envelope rejects every time (A/B on recorded frames: blocked 7/7 vs 0/4),
    # so the corner was never detected. The real understeer at speed has to be
    # handled by the corner speed cap, not by widening the planned arc.
    radius: float = 0.83
    front: float = 0.40
    rear: float = 0.16
    half_width: float = 0.175
    margin: float = 0.055
    servo_center: float = 84.5
    encoder_scale: float = 1.009
    laser_x: float = 0.23


class VehicleIO(Node):
    def __init__(self, observing=False):
        super().__init__('s_bend')
        self.car = Car()
        self.observing = observing
        self.commands = None if observing else self.create_publisher(Twist, '/car_cmd_vel', 1)
        self.frames = deque(maxlen=80)
        self.events = deque(maxlen=500)
        self.startup_path = None
        self.startup_width = None
        self.startup_frames = 0
        self.plan_ms = 0.0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        self.create_subscription(Imu, '/IMU_data', self.on_imu, qos)
        self.create_subscription(Odometry, '/encoder', self.on_encoder, qos)
        self.scan_at = self.imu_at = self.encoder_at = None
        self.plan_at = self.last_dual = self.target = None
        self.center_path = None
        self.lane_width = None
        self.boundary_counts = (0, 0)
        self.route_mode = 'none'
        self.reference_path = None
        self.reference_distance = self.reference_yaw = self.yaw = 0.0
        self.cones = self.points = None
        self.obstacle = ObstacleHold()
        self.speed = self.distance = 0.0
        self.servo, self.pwm = self.car.servo_center, 1500.0
        self.moving_command = self.ever_drove = False
        self.started_at = self.last_tick = time.monotonic()
        self.last_log = 0.0
        self.last_status = self.stop_reason = None
        self.create_timer(0.05, self.control)


    def on_scan(self, msg):
        self.cones, self.points = scan_geometry(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
            (self.car.laser_x, 0.0, 0.0), mask_observed_body=True)
        self.scan_at = time.monotonic() if len(self.points) >= 8 else None
        self.frames.append(dict(at=time.monotonic(), stamp=[msg.header.stamp.sec, msg.header.stamp.nanosec],
                                angle_min=float(msg.angle_min), angle_increment=float(msg.angle_increment),
                                range_min=float(msg.range_min), range_max=float(msg.range_max),
                                ranges=[float(r) if math.isfinite(r) else None for r in msg.ranges]))


    def on_imu(self, msg):
        q = msg.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        valid = 0.8 <= norm <= 1.2 and msg.orientation_covariance[0] != -1
        self.imu_at = time.monotonic() if valid else None
        if valid:
            self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


    def on_encoder(self, msg):
        speed = msg.twist.twist.linear.x * self.car.encoder_scale
        if not math.isfinite(speed) or abs(speed) > 3.0:
            self.encoder_at = None
            return
        now = time.monotonic()
        if self.encoder_at is not None and now - self.encoder_at < 0.4:
            self.distance += max(0.0, (self.speed + speed)/2) * (now - self.encoder_at)
        self.speed, self.encoder_at = speed, now


    def publish(self, pwm, servo):
        if self.observing:
            raise RuntimeError('Observation mode must never publish driving commands')
        if hasattr(self, 'events'):
            self.events.append(dict(at=time.monotonic(), kind='command', pwm=float(pwm), servo=float(servo)))
        msg = Twist()
        msg.linear.x, msg.angular.z = float(pwm), float(servo)
        self.commands.publish(msg)


    def neutral(self, servo=None):
        self.pwm = 1500.0
        self.servo = self.car.servo_center if servo is None else servo
        self.moving_command = False
        self.publish(self.pwm, self.servo)


    def stop(self, reason):
        self.stop_reason = reason
        if not getattr(self, 'observing', False):
            self.neutral()


    def input_state(self, now):
        return dict(ages_s={name: None if stamp is None else round(now-stamp, 4)
                            for name, stamp in [('scan', self.scan_at), ('imu', self.imu_at),
                                                ('encoder', self.encoder_at)]},
                    publishers=self.count_publishers('/car_cmd_vel'),
                    subscribers=None if self.commands is None else self.commands.get_subscription_count())


    def save_record(self):
        raise NotImplementedError

    def status(self, now, reason):
        if reason != self.last_status or now - self.last_log >= 0.5:
            detail = ''
            if reason == 'WAIT_ROUTE':
                car = self.car
                x, y = self.points.T
                near_body = self.points[(x >= -car.rear - car.margin)
                                        & (x <= car.front + car.margin)
                                        & (np.abs(y) <= car.half_width + car.margin)]
                if len(near_body):
                    points = ', '.join(f'({x:+.3f},{y:+.3f})' for x, y in near_body[:4])
                    detail = f' near_body_xy_m=[{points}]'
            self.get_logger().info(f'{reason} pwm={self.pwm:.1f} servo={self.servo:.1f} '
                                   f'v={self.speed:+.3f} m/s distance={self.distance:.2f} m'
                                   f' route={self.route_mode} L/R={self.boundary_counts}'
                                   f' centers={None if self.center_path is None else np.round(self.center_path[[0, -1]], 2).tolist()}'
                                   f' confirm={self.startup_frames}/3 plan_ms={self.plan_ms:.1f}'
                                   f'{detail}')
            self.last_status, self.last_log = reason, now


    def update_target(self, now):
        raise NotImplementedError

    def observe(self, now):
        if self.scan_at is not None and self.scan_at != self.plan_at:
            self.plan_at = self.scan_at
            self.update_target(now)
            if self.target is not None:
                servo, correction, clear = self.choose_steering(32.0)
                self.servo = self.car.servo_center if servo is None else servo
                self.events.append(dict(at=now, kind='observation', servo=self.servo,
                                        correction=correction, clear=clear, inputs=self.input_state(now)))
            self.status(now, 'OBSERVE_NO_COMMANDS')
        if now-self.started_at >= 8.0:
            self.stop('observation_complete')


    def choose_steering(self, step):
        raise NotImplementedError

    def control(self):
        if self.stop_reason is not None:
            return
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        if getattr(self, 'observing', False):
            self.observe(now)
            return
        publishers = self.count_publishers('/car_cmd_vel')
        if publishers > 1:
            if self.ever_drove:
                self.stop('other_command_publisher')
            else:
                self.stop_reason = 'other_command_publisher'
            return
        inputs = ((self.scan_at, 0.28), (self.imu_at, 0.28), (self.encoder_at, 0.40))
        ready = publishers == 1 and self.commands.get_subscription_count() >= 1 and all(
            stamp is not None and now - stamp < age for stamp, age in inputs)
        if not ready:
            if self.ever_drove or now - self.started_at > 8.0:
                self.stop('sensor_or_driver_unavailable '+json.dumps(self.input_state(now)))
            else:
                self.neutral()
                self.status(now, 'WAIT_SENSORS')
            return
        if dt > 0.25:
            self.stop(f'control_delay dt={dt:.3f}s v={self.speed:+.3f}m/s')
            return
        if self.speed < -0.09:
            self.stop(f'reverse_motion v={self.speed:+.3f}m/s')
            return
        if self.plan_at != self.scan_at:
            self.plan_at = self.scan_at
            self.update_target(now)
        if self.target is None or self.target[0] <= 0.12:
            if self.ever_drove:
                self.stop('cone_corridor_lost')
            else:
                self.neutral()
                self.status(now, 'WAIT_CONE_CORRIDOR')
            return
        if not self.ever_drove and self.startup_frames < 3:
            self.neutral()
            self.status(now, 'WAIT_ROUTE_CONFIRM')
            return
        # 85 deg/s needed 0.24 s to reach a 20 deg bend command -- 0.31 m of
        # travel at 1.3 m/s, which reads as a lazy turn-in. The servo is
        # mechanically fine at 160 (corner mode already uses 200).
        servo, correction, clear = self.choose_steering(160.0*min(dt, 0.06))
        if not clear:
            self.obstacle.clear_frames = 0
        hold = self.obstacle.update(self.scan_at, now, not clear)
        if not clear or hold != 'clear' or self.obstacle.clear_frames < 3:
            settled_servo = servo if servo is not None and abs(self.speed) <= 0.03 else self.servo
            self.neutral(settled_servo)
            self.status(now, 'ALIGN_ROUTE' if servo is not None and not clear else 'WAIT_ROUTE')
            return
        # Only the very first move waits for a standstill. After a brief gate
        # block the car is still rolling; insisting on a full stop turned
        # every recovery into a stop-and-go (4 in the 00:29 run, each ~1 s).
        if not self.ever_drove and abs(self.speed) > 0.03:
            self.neutral(self.servo)
            self.status(now, 'WAIT_STATIONARY')
            return
        self.servo = servo
        self.publish(1548.0, self.servo)
        self.moving_command = self.ever_drove = True
        self.status(now, 'S_BEND_AVOID' if correction else 'S_BEND')


