"""Two-edge corridor planner.

Fits the left and right cone rows as edges, pairs them across the corridor,
and derives a continuous midline plus a per-sample width. The turn angle is a
property of that midline, so an S, a 90-degree corner and a U-turn all use the
same code; there is no row switch or fixed hug offset.
"""
import math

import numpy as np

from dense_reference import (DenseRoute, boundary_density, entrance_chain,
                             fit_curve, nearest_width)


class CorridorPlanner:
    def __init__(self, car, base_lookahead=.7, speed_lookahead=.25, extension=.3):
        self.car = car
        self.base_lookahead = base_lookahead
        self.speed_lookahead = speed_lookahead
        self.extension = extension
        self.width = None
        self.last_heading = None
        self.last_row_stats = {}
        self.smooth_target = None
        self.smooth_heading = None
        self.last_reason = None

    @staticmethod
    def _normal(tangent):
        return np.column_stack((-tangent[:, 1], tangent[:, 0]))

    @staticmethod
    def _dedup(left, right):
        if len(left) and len(right):
            shared = np.linalg.norm(left[:, None, :]-right[None, :, :], axis=2) < .12
            if np.any(shared):
                left = np.delete(left, np.flatnonzero(np.any(shared, axis=1)), axis=0)
                right = np.delete(right, np.flatnonzero(np.any(shared, axis=0)), axis=0)
        return left, right

    @staticmethod
    def _pair_midpoints(curve_left, curve_right):
        pts_l, tan_l, _ = curve_left
        pts_r, _, _ = curve_right
        mids, widths = [], []
        for point, tangent in zip(pts_l, tan_l):
            normal = np.array([-tangent[1], tangent[0]])
            distance = np.linalg.norm(pts_r-point, axis=1)
            other = pts_r[int(np.argmin(distance))]
            gap = other-point
            length = float(np.linalg.norm(gap))
            if length < .4 or length > 2.4:
                continue
            if abs(float(np.dot(gap, normal))) < .6*length:
                continue
            width = -float(np.dot(gap, normal))
            if width <= .2:
                continue
            mids.append(.5*(point+other))
            widths.append(width)
        if len(mids) < 3:
            return None, None
        return np.asarray(mids), np.asarray(widths)

    @staticmethod
    def _curve(points):
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(pts) < 2 or not np.all(np.isfinite(pts)):
            return None
        keep = [0]
        for i in range(1, len(pts)):
            if np.linalg.norm(pts[i]-pts[keep[-1]]) > .005:
                keep.append(i)
        pts = pts[keep]
        if len(pts) < 2:
            return None
        tangent = np.gradient(pts, axis=0)
        norm = np.linalg.norm(tangent, axis=1)
        norm[norm < 1e-6] = 1.
        tangent = tangent/norm[:, None]
        dx = np.gradient(pts[:, 0])
        dy = np.gradient(pts[:, 1])
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        curvature = (dx*ddy-dy*ddx)/np.maximum(np.hypot(dx, dy)**3, 1e-6)
        return pts, tangent, curvature

    @staticmethod
    def _offset_edge(edge, side, width):
        pts, tangent, _ = edge
        return pts-side*(width/2.)*CorridorPlanner._normal(tangent)

    def plan(self, cones, speed, now, reference=None):
        left = entrance_chain(cones, 1, self.car, reference, self.width)
        right = entrance_chain(cones, -1, self.car, reference, self.width)
        left, right = self._dedup(left, right)
        curve_left = fit_curve(left)
        curve_right = fit_curve(right)
        self.last_row_stats = {
            1: {'points': len(left), 'density': round(float(boundary_density(left)), 3),
                'usable': curve_left is not None},
            -1: {'points': len(right), 'density': round(float(boundary_density(right)), 3),
                 'usable': curve_right is not None},
        }
        if curve_left is None and curve_right is None:
            self.last_reason = 'no_rows'
            return None

        mids, widths = (None, None)
        if curve_left is not None and curve_right is not None:
            mids, widths = self._pair_midpoints(curve_left, curve_right)

        if mids is not None:
            width = float(np.median(widths))
            fitted = self._curve(mids)
            if fitted is None:
                mids = None
        if mids is None:
            width = nearest_width(left, right)
            if width is None:
                width = self.width if self.width is not None else 1.42
            if curve_left is not None and len(left) >= len(right):
                centres = self._offset_edge(curve_left, 1, width)
                side = 1
            else:
                centres = self._offset_edge(curve_right, -1, width)
                side = -1
            fitted = self._curve(centres)
            if fitted is None:
                self.last_reason = 'single_fit'
                return None
            mids = centres
            self.last_row_stats[side]['usable'] = True

        centres, tangent, curvature = fitted
        if centres[0, 0] > centres[-1, 0]:
            centres, tangent, curvature = centres[::-1], tangent[::-1], curvature[::-1]
        # Extend the fitted centreline back to the car. The first cone can be
        # 1 m ahead; measuring the preview from the first cone made every
        # short fit exceed `extension` and return None.
        if centres[0, 0] > .2:
            first_tangent = tangent[0].copy()
            centres = np.vstack(([.05, 0.], centres))
            tangent = np.vstack((first_tangent, tangent))
            curvature = np.r_[curvature[0], curvature]
        self.width = width

        length = np.r_[0., np.cumsum(np.linalg.norm(np.diff(centres, axis=0), axis=1))]
        preview = self.base_lookahead+self.speed_lookahead*abs(speed)
        preview /= 1+.3*float(np.max(np.abs(curvature[:max(1, len(curvature)//2)])))
        preview = max(.45, min(1.2, preview))
        remaining = preview
        predicted = remaining > length[-1]
        if predicted:
            extra = remaining-length[-1]
            if extra > self.extension:
                self.last_reason = 'extension %.2f' % extra
                return None
            target = centres[-1]+extra*tangent[-1]
        else:
            target = np.array([np.interp(remaining, length, centres[:, i]) for i in (0, 1)])
        direction = np.array([np.interp(min(remaining, length[-1]), length, tangent[:, i])
                              for i in (0, 1)])
        heading = math.atan2(direction[1], direction[0])

        if self.smooth_target is None:
            self.smooth_target = np.asarray(target, dtype=float).copy()
            self.smooth_heading = heading
        else:
            alpha = .5
            self.smooth_target = (alpha*np.asarray(target, dtype=float)
                                  +(1.-alpha)*self.smooth_target)
            delta = math.atan2(math.sin(heading-self.smooth_heading),
                               math.cos(heading-self.smooth_heading))
            self.smooth_heading = math.atan2(math.sin(self.smooth_heading+alpha*delta),
                                             math.cos(self.smooth_heading+alpha*delta))
        target = tuple(float(v) for v in self.smooth_target)
        heading = self.smooth_heading
        self.last_heading = heading
        self.last_reason = 'ok'
        side = 1 if curve_left is not None else -1
        return DenseRoute(centres, left, right, centres, side, width,
                          curvature, preview, target, heading, predicted)

    def hold(self, speed):
        if self.last_heading is None:
            return None
        heading = self.last_heading
        width = self.width if self.width is not None else 1.42
        look = max(.45, min(1.2, .7+.25*abs(speed)))
        distance = np.linspace(.05, max(1.2, look+.6), 30)
        centres = np.column_stack((distance*math.cos(heading), distance*math.sin(heading)))
        tangent = np.tile([math.cos(heading), math.sin(heading)], (len(distance), 1))
        curvature = np.zeros(len(distance))
        empty = np.empty((0, 2))
        return DenseRoute(centres, empty, empty, centres, 1, width, curvature,
                          look, (look*math.cos(heading), look*math.sin(heading)),
                          heading, True)
