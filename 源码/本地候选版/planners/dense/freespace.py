"""Ridge-following free-space planner.

Rows and midpoints are not used: the drivable area is described by a clearance
field (distance from every cell to the nearest cone), and the path follows the
ridge of that field. That is exactly "drive down the middle of the empty
space", which needs no track width, no row fitting, and no island detection -
an island simply splits the ridge and the walk goes round it.

Only the car's own size appears in the cost, so the same code works on any
layout.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class FreeRoute:
    centers: np.ndarray
    target: np.ndarray
    heading: float
    curvature_target: float
    width: float
    left: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    right: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    preview: float = 1.0
    parts: dict = field(default_factory=dict)


class FreeSpacePlanner:
    def __init__(self, car, settings):
        self.car = car
        self.settings = settings
        self.window_s = float(settings.get("fs_window_s", 2.0))
        self.horizon = float(settings.get("fs_horizon_m", 3.0))
        self.step = float(settings.get("fs_step_m", 0.10))
        self.look = float(settings.get("fs_look_m", 0.35))
        self.turn_step = math.radians(float(settings.get("fs_turn_deg", 14.0)))
        self.kappa_max = float(settings.get("fs_kappa_max", 0.55))
        self.n_curv = int(settings.get("fs_candidates", 21))
        self.offset_penalty = float(settings.get("fs_offset_penalty", 0.9))
        self.safe_radius = float(car.half_width) + float(car.margin) + 0.05
        self.map = []
        self.last_pair = None
        self.last_row_stats = {1: {"candidates": 0}, -1: {"candidates": 0}}
        self.prev_path = None
        self.side = 0
        self.last_kappa = 0.0
        self.reject = None

    # -- map ---------------------------------------------------------------
    def _update_map(self, points, now, ego):
        if ego is not None:
            angle, travel = float(ego[0]), float(ego[1])
            if abs(angle) < 1e-6:
                translation = np.array([travel, 0.0])
            else:
                translation = travel * np.array([math.sin(angle),
                                                 1.0 - math.cos(angle)]) / angle
            rotation = np.array([[math.cos(angle), -math.sin(angle)],
                                 [math.sin(angle), math.cos(angle)]])
            if self.prev_path is not None:
                self.prev_path = (self.prev_path - translation) @ rotation
            kept = []
            for (x, y, ts) in self.map:
                if now - ts > self.window_s:
                    continue
                q = (np.array([x, y]) - translation) @ rotation
                if -0.5 < q[0] < 7.0 and abs(q[1]) < 6.0:
                    kept.append((float(q[0]), float(q[1]), ts))
            self.map = kept
        for (x, y) in points:
            if not (0.05 <= x <= 6.5) or abs(y) > 6.0:
                continue
            best, best_d = -1, 1e9
            for k, (mx, my, _ts) in enumerate(self.map):
                dd = (mx - x) ** 2 + (my - y) ** 2
                if dd < best_d:
                    best_d, best = dd, k
            if best >= 0 and best_d < 0.09:
                mx, my, _ = self.map[best]
                self.map[best] = (0.5 * (mx + x), 0.5 * (my + y), now)
            else:
                self.map.append((float(x), float(y), now))
        if len(self.map) > 240:
            del self.map[:len(self.map) - 240]
        return np.array([[x, y] for (x, y, _t) in self.map], dtype=float)

    # -- clearance ---------------------------------------------------------
    def _clearance(self, pts, xy):
        if len(pts) == 0:
            return 9.0
        d = pts - xy
        return float(np.min(np.hypot(d[:, 0], d[:, 1])))

    def _choose_side(self, pts):
        """Pick the side of an island by the room it leaves ~3 m out."""
        probe = max(1.5, 0.85 * self.horizon)
        ang = 0.35  # ~20 deg off the nose
        c_left = self._clearance(pts, np.array([probe * math.cos(ang), probe * math.sin(ang)]))
        c_right = self._clearance(pts, np.array([probe * math.cos(ang), -probe * math.sin(ang)]))
        pick = 1 if c_left >= c_right else -1
        if self.side == 0:
            self.side = pick
        elif pick != self.side:
            # only swap when the other side is clearly roomier
            better = (c_left if pick > 0 else c_right)
            current = (c_left if self.side > 0 else c_right)
            if better > 1.25 * current:
                self.side = pick
        self.side_room = (round(c_left, 2), round(c_right, 2))
        return self.side

    def _rollout(self, kappa):
        n = max(6, int(round(self.horizon / self.step)))
        s = np.arange(n) * self.step
        if abs(kappa) < 1e-4:
            path = np.column_stack((s, np.zeros_like(s)))
            tang = np.zeros_like(path)
            tang[:, 0] = 1.0
            return path, tang
        theta = kappa * s
        path = np.column_stack((np.sin(theta) / kappa, (1.0 - np.cos(theta)) / kappa))
        tang = np.column_stack((np.cos(theta), np.sin(theta)))
        return path, tang

    def _best_arc(self, pts):
        """Score curvature candidates by the room they keep, with a side bias."""
        side = self._choose_side(pts)
        best = None
        for kappa in np.linspace(-self.kappa_max, self.kappa_max, self.n_curv):
            path, tang = self._rollout(float(kappa))
            clr = np.array([self._clearance(pts, q) for q in path])
            clr[:3] = clr[3] if len(clr) > 3 else clr[:3]
            room = float(np.min(clr))
            if room < self.safe_radius:
                continue
            # room far ahead matters more than the immediate 0.5 m
            far = float(np.min(clr[len(clr) // 2:]))
            score = far + 0.35 * room - 0.9 * abs(float(kappa)) - 0.35 * abs(float(kappa) - self.last_kappa)
            if side and np.sign(kappa) != np.sign(side) and abs(kappa) > 0.12:
                score -= 0.12
            if best is None or score > best[0]:
                best = (score, float(kappa), path, tang, room, far)
        return best

    def hold(self, speed=None):
        """No route available: the node falls back to coasting/stopping."""
        return None

    # -- public ------------------------------------------------------------
    def plan(self, cones, speed, now, raw_points=None, ego=None, reference=None):
        pts = np.asarray(raw_points if raw_points is not None else cones,
                         dtype=float).reshape(-1, 2)
        mapped = self._update_map(pts, now, ego)
        if len(mapped) < 3:
            self.reject = "empty"
            return None
        arc = self._best_arc(mapped)
        if arc is None:
            self.reject = "blocked"
            return None
        _score, kappa0, path, tang, room, far = arc
        headings = np.arctan2(tang[:, 1], tang[:, 0])
        clr = np.full(len(path), room)

        preview = max(0.8, min(1.7, 0.9 + 0.35 * abs(speed)))
        idx = int(np.argmin(np.abs(path[:, 0] - preview)))
        idx = min(max(idx, 2), len(path) - 1)
        target = path[idx].copy()
        heading = float(headings[idx])

        # curvature from the path itself
        if idx >= 3:
            a, b, c = path[idx - 3], path[idx - 1], path[idx]
            v1 = b - a
            v2 = c - b
            denom = (np.linalg.norm(v1) * np.linalg.norm(v2) *
                     np.linalg.norm(c - a)) if np.linalg.norm(c - a) > 1e-6 else 1.0
            cross = v1[0] * v2[1] - v1[1] * v2[0]
            kappa = float(2.0 * cross / max(1e-6, denom))
        else:
            kappa = 0.0
        kappa = max(-1.0, min(1.0, kappa0))
        self.last_kappa = kappa

        self.prev_path = path.copy()
        width = 2.0 * float(np.min(clr))
        return FreeRoute(centers=path, target=target, heading=heading,
                         curvature_target=kappa, width=width, preview=preview,
                         parts={"clearance": round(float(np.min(clr)), 2),
                                "kappa": round(kappa, 3),
                                "side": getattr(self, "side", 0)})
