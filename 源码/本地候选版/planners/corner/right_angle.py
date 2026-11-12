"""One-shot blind-turn fallback for a confirmed right-angle entrance.

The fallback is subordinate to normal cone planning. It arms only after the
car has followed a valid corridor, the forward sector has remained free of
cone clusters for several lidar frames, and the recent corridor is still
fresh. The turn is closed on IMU yaw and exits back to normal planning only
after the heading and corridor agree.
"""
import math

import numpy as np

from dense_reference import DenseRoute
from geometry import wrap


class RightAngleFallback:
    def __init__(self, car, settings):
        self.car = car
        self.enabled = bool(settings.get("right_angle_fallback_enabled", False))
        self.direction = str(settings.get("right_angle_turn_direction", "left")).lower()
        self.front_half_angle = math.radians(float(
            settings.get("right_angle_front_half_angle_deg", 45.0)))
        self.front_max_range = float(settings.get("right_angle_max_range_m", 2.5))
        self.trigger_frames = int(settings.get("right_angle_trigger_frames", 4))
        self.recent_route_s = float(settings.get("right_angle_recent_route_s", 2.0))
        self.speed = float(settings.get("right_angle_speed_mps", 0.25))
        self.radius = float(settings.get("right_angle_turn_radius_m", 0.95))
        self.lookahead = float(settings.get("right_angle_lookahead_m", 0.70))
        self.target_yaw = math.radians(float(
            settings.get("right_angle_target_yaw_deg", 90.0)))
        self.release_yaw = math.radians(float(
            settings.get("right_angle_release_yaw_deg", 75.0)))
        self.abort_yaw = math.radians(float(
            settings.get("right_angle_abort_yaw_deg", 105.0)))
        self.timeout = float(settings.get("right_angle_timeout_s", 10.0))
        self.once = bool(settings.get("right_angle_once", True))

        if self.direction not in ("left", "right"):
            raise ValueError("right_angle_turn_direction must be left or right")
        if not 20.0 <= math.degrees(self.front_half_angle) <= 60.0:
            raise ValueError("right_angle_front_half_angle_deg must be 20..60")
        if not 1.0 <= self.front_max_range <= 4.0:
            raise ValueError("right_angle_max_range_m must be 1..4")
        if not 2 <= self.trigger_frames <= 10:
            raise ValueError("right_angle_trigger_frames must be 2..10")
        if not 0.05 <= self.recent_route_s <= 5.0:
            raise ValueError("right_angle_recent_route_s must be 0.05..5")
        if not 0.10 <= self.speed <= 0.60:
            raise ValueError("right_angle_speed_mps must be 0.10..0.60")
        if not self.car.radius * 0.95 <= self.radius <= 2.0:
            raise ValueError("right_angle_turn_radius_m is outside the safe range")
        if not 0.40 <= self.lookahead <= 1.20:
            raise ValueError("right_angle_lookahead_m must be 0.40..1.20")
        if not math.radians(60.0) <= self.target_yaw <= math.radians(120.0):
            raise ValueError("right_angle_target_yaw_deg must be 60..120")
        if not 0.0 < self.release_yaw < self.target_yaw < self.abort_yaw <= math.radians(130.0):
            raise ValueError("right-angle yaw windows are inconsistent")
        if not 0.5 <= self.timeout <= 12.0:
            raise ValueError("right_angle_timeout_s must be 0.5..12")

        self.sign = 1.0 if self.direction == "left" else -1.0
        self.active = False
        self.used = False
        self.aborted = False
        self.empty_frames = 0
        self.front_count = 0
        self.last_route_at = None
        self.last_width = None
        self.entry_yaw = None
        self.started_at = None
        self.last_reason = "idle"
        self.max_progress = 0.0

    def _front_cone_count(self, cones):
        points = np.asarray(cones, dtype=float).reshape(-1, 2)
        if not len(points):
            return 0
        dx = points[:, 0] - self.car.laser_x
        distance = np.hypot(dx, points[:, 1])
        bearing = np.arctan2(points[:, 1], dx)
        forward = ((distance >= 0.25) & (distance <= self.front_max_range)
                   & (np.abs(bearing) <= self.front_half_angle))
        return int(np.count_nonzero(forward))

    def _progress(self, yaw):
        if self.entry_yaw is None or not math.isfinite(yaw):
            return 0.0
        progress = self.sign * wrap(yaw - self.entry_yaw)
        self.max_progress = max(self.max_progress, progress)
        return progress

    def update(self, cones, route, yaw, now, ever_drove, width=None):
        if width is not None and math.isfinite(width) and width > 0.0:
            self.last_width = float(width)
        if not self.enabled:
            self.last_reason = "disabled"
            return
        self.front_count = self._front_cone_count(cones)

        if not ever_drove:
            self.empty_frames = 0
            self.last_reason = "waiting_for_drive"
            return

        if self.active:
            progress = self._progress(yaw)
            if route is not None and progress >= self.release_yaw:
                self.active = False
                self.last_reason = "released_to_planner"
            elif progress >= self.abort_yaw:
                self.active = False
                self.aborted = True
                self.last_reason = "yaw_overrun"
            elif self.started_at is not None and now - self.started_at >= self.timeout:
                self.active = False
                self.aborted = True
                self.last_reason = "timeout"
            return

        if self.aborted or (self.used and self.once):
            return

        recent_route = (self.last_route_at is not None
                        and now - self.last_route_at <= self.recent_route_s)
        if self.front_count == 0 and recent_route:
            self.empty_frames = min(self.trigger_frames, self.empty_frames + 1)
        else:
            self.empty_frames = 0

        if route is not None:
            self.last_route_at = now
            if width is not None and math.isfinite(width) and width > 0.0:
                self.last_width = float(width)
        elif not recent_route:
            self.last_reason = "no_recent_corridor"

        if self.empty_frames >= self.trigger_frames and math.isfinite(yaw):
            self.active = True
            self.used = True
            self.entry_yaw = float(yaw)
            self.started_at = now
            self.max_progress = 0.0
            self.empty_frames = 0
            self.last_reason = "blind_" + self.direction

    def limit_speed(self, target):
        if self.active and math.isfinite(target):
            return min(float(target), self.speed)
        return target

    def route(self, speed=0.0, width=None):
        width = width if width is not None else self.last_width
        if width is None or not math.isfinite(width) or not 0.6 <= width <= 2.4:
            width = 1.42
        max_angle = max(self.abort_yaw, self.target_yaw + math.radians(10.0))
        arc_length = self.radius * max_angle
        distance = np.linspace(0.02, arc_length, 80)
        angle = distance / self.radius
        centers = np.column_stack((
            self.radius * np.sin(angle),
            self.sign * self.radius * (1.0 - np.cos(angle)),
        ))
        tangent = np.column_stack((np.cos(angle), self.sign * np.sin(angle)))
        curvature = np.full(len(distance), self.sign / self.radius)
        preview = max(0.40, min(self.lookahead, arc_length - 0.05))
        target_x = self.radius * math.sin(preview / self.radius)
        target_y = self.sign * self.radius * (1.0 - math.cos(preview / self.radius))
        heading = self.sign * (preview / self.radius)
        empty = np.empty((0, 2))
        return DenseRoute(
            centers, empty, empty, centers, int(self.sign), float(width),
            curvature, preview, (float(target_x), float(target_y)),
            float(heading), True,
        )

    def event(self):
        return dict(
            enabled=self.enabled, active=self.active, used=self.used,
            aborted=self.aborted, direction=self.direction,
            front_count=self.front_count, empty_frames=self.empty_frames,
            reason=self.last_reason,
            progress_deg=round(math.degrees(self.max_progress), 2),
        )
