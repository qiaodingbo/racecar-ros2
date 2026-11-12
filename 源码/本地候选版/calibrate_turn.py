#!/usr/bin/env python3
"""Measure the real turning radius at fixed servo angles.

Drives short constant-steering arcs and reports kappa = d(yaw)/d(distance), so
the planner can use measured left/right steering strength instead of a guess.

Safety
  * put the car in an open area with at least a 4 m circle of free floor;
  * speed is fixed at a crawl (pwm 1546, barely above neutral);
  * every run ends with neutral throttle and centred steering;
  * Ctrl-C aborts immediately and still sends neutral.

    ./calibrate_turn.py                 # 110 and 60 deg, 3 s each
    ./calibrate_turn.py --angles 105 100 65 70
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def yaw_of(msg):
    q = msg.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Calibrator(rclpy.node.Node):
    def __init__(self):
        super().__init__("turn_calibrator")
        qos = rclpy.qos.qos_profile_sensor_data
        self.pub = self.create_publisher(Twist, "/car_cmd_vel", 1)
        self.create_subscription(Imu, "/IMU_data", self.on_imu, qos)
        self.create_subscription(Odometry, "/encoder", self.on_enc, qos)
        self.yaw = None
        self.distance = None
        self.speed = 0.0
        self.last_enc = None

    def on_imu(self, msg):
        self.yaw = yaw_of(msg)

    def on_enc(self, msg):
        now = time.monotonic()
        speed = msg.twist.twist.linear.x * 1.009
        if self.last_enc is not None and now - self.last_enc < 0.4 and self.distance is not None:
            self.distance += max(0.0, 0.5 * (self.speed + speed) * (now - self.last_enc))
        else:
            self.distance = 0.0 if self.distance is None else self.distance
        self.speed = speed
        self.last_enc = now

    def ready(self):
        return self.yaw is not None and self.distance is not None

    def drive(self, pwm, servo, seconds):
        msg = Twist()
        msg.linear.x = float(pwm)
        msg.angular.z = float(servo)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pub.publish(msg)
            time.sleep(0.02)

    def neutral(self):
        msg = Twist()
        msg.linear.x = 1500.0
        msg.angular.z = 84.5
        for _ in range(6):
            self.pub.publish(msg)
            time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--angles", nargs="*", type=float, default=[110.0, 60.0])
    parser.add_argument("--hold", type=float, default=3.0)
    parser.add_argument("--pwm", type=float, default=1546.0)
    parser.add_argument("--settle", type=float, default=0.5)
    parser.add_argument("--yes", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = Calibrator()
    print("OPEN AREA NEEDED: the car will drive a short arc at each angle.", flush=True)
    if not args.yes:
        for left in range(5, 0, -1):
            print("  starting in %d s (Ctrl-C aborts)" % left, flush=True)
            time.sleep(1.0)
    try:
        while not node.ready():
            rclpy.spin_once(node, timeout_sec=0.1)
        print("\n servo  offset |   arc     dyaw     kappa     R     |  deg/kappa", flush=True)
        for angle in args.angles:
            offset = angle - 84.5
            node.drive(args.pwm, angle, args.settle)
            y0, s0 = node.yaw, node.distance
            node.drive(args.pwm, angle, args.hold)
            dyaw = math.atan2(math.sin(node.yaw - y0), math.cos(node.yaw - y0))
            ds = node.distance - s0
            node.neutral()
            time.sleep(0.8)
            if abs(ds) < 0.15:
                print(" %5.1f  %+6.1f | arc too short (%.2f m)" % (angle, offset, ds), flush=True)
                continue
            kappa = dyaw / ds
            radius = 1.0 / abs(kappa) if abs(kappa) > 1e-6 else float("inf")
            print(" %5.1f  %+6.1f | %5.2f m  %+6.1f deg  %+0.3f  %5.2f m | %7.1f" % (
                angle, offset, ds, math.degrees(dyaw), kappa, radius,
                offset / kappa if abs(kappa) > 1e-6 else float("nan")), flush=True)
    except KeyboardInterrupt:
        print("\naborted", flush=True)
    finally:
        node.neutral()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
