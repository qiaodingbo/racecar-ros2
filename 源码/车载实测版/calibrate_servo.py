#!/usr/bin/env python3
"""Hold the steering servo at fixed angles so the wheel angle can be measured.

Safety: the car must be LIFTED (wheels off the ground) and secured. Throttle is
published as 1500 us (neutral) for every step, so the wheels only steer.

    ./calibrate_servo.py              # 84.5 -> 122.5 -> 50 -> 84.5, 3 s each
    ./calibrate_servo.py --hold 5     # slower
    ./calibrate_servo.py --angles 100 110 120

The driver (racecar_driver) must be running and be the only subscriber.
"""
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist

ANGLES = [84.5, 95.0, 100.0, 105.0, 110.0, 115.0, 120.0, 122.5,
          84.5, 75.0, 70.0, 65.0, 60.0, 55.0, 50.0, 84.5]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hold", type=float, default=3.0, help="seconds per angle")
    parser.add_argument("--angles", nargs="*", type=float, default=None)
    parser.add_argument("--yes", action="store_true", help="skip the 5 s warning")
    args = parser.parse_args()
    angles = args.angles or ANGLES

    rclpy.init()
    node = rclpy.create_node("servo_calib")
    publisher = node.create_publisher(Twist, "/car_cmd_vel", 1)
    msg = Twist()

    print("WHEELS OFF THE GROUND. Throttle stays at 1500 us (neutral).", flush=True)
    if not args.yes:
        for remaining in range(5, 0, -1):
            print("  starting in %d s (Ctrl-C aborts)" % remaining, flush=True)
            time.sleep(1.0)
    try:
        for angle in angles:
            msg.linear.x = 1500.0
            msg.angular.z = float(angle)
            publisher.publish(msg)
            print("ANGLE %5.1f   servo pwm %4.0f us" % (angle, 2500.0 - angle * 2000.0 / 180.0),
                  flush=True)
            deadline = time.monotonic() + args.hold
            while time.monotonic() < deadline:
                publisher.publish(msg)
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("aborted", flush=True)
    finally:
        msg.linear.x = 1500.0
        msg.angular.z = 84.5
        for _ in range(5):
            publisher.publish(msg)
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
