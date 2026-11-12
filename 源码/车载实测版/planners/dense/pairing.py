"""Fit both observed LiDAR boundaries and plan in their common corridor."""
from dataclasses import dataclass
import math

import numpy as np

from dense_reference import DenseRoute, fit_curve
from geometry import relative_to_path, wrap


@dataclass
class Boundary:
    points: np.ndarray
    edge: np.ndarray
    center: np.ndarray
    heading: np.ndarray
    curvature: np.ndarray
    near: float
    density: float
    continuation: float
    samples: tuple = None
    line: tuple = None


def _near_support(row, laser_x):
    first, second = row[:2]
    near = (math.hypot(first[0]-laser_x, first[1])
            + math.hypot(second[0]-laser_x, second[1]))/2
    gap = math.hypot(*(second-first))
    if len(row) >= 3:
        gap = (gap + math.hypot(*(row[2]-second)))/2
    return near, 1/gap


def _rows(points, max_points, laser_x, max_gap):
    """Generate near-seeded, connected prefixes. Keep alternative second points."""
    if len(points) < 2:
        return []
    distance = np.hypot(points[:, 0]-laser_x, points[:, 1])
    nearest = np.argsort(distance, kind='stable')[:max_points]
    points = points[nearest]
    delta = points[None, :, :] - points[:, None, :]
    gap = np.linalg.norm(delta, axis=2)
    angle = np.arctan2(delta[:, :, 1], delta[:, :, 0])
    connected = (delta[:, :, 0] > .16) & (gap >= .28) & (gap <= max_gap)
    rows = {}
    for i in range(min(5, len(points))):
        choices = np.flatnonzero(connected[i] & (np.abs(angle[i]) < math.radians(60)))
        order = np.argsort(gap[i, choices] * (1 + .35 * np.abs(angle[i, choices])))
        for j in choices[order[:3]]:
            chain = [i, j]
            while len(chain) <= min(6, max_points):
                key = tuple(chain)
                if key not in rows:
                    rows[key] = points[chain]
                a, b = chain[-2:]
                turn = np.abs(angle[b] - angle[a, b])
                options = np.flatnonzero(connected[b] & (turn <= math.radians(45)))
                if not len(options):
                    break
                cost = gap[b, options] * (1 + .8 * turn[options])
                chain.append(int(options[np.argmin(cost)]))
    return list(rows.values())


def _row_priority(row, laser_x):
    near, density = _near_support(row, laser_x)
    span = float(row[-1, 0] - row[0, 0])
    return .48 / (1 + near) + .23 * min(density / 2, 1.) + .29 * span / (1 + span)


def _continuation(row, edge, tangent, curvature, points):
    """Score the next available point on this side using the end tangent/turn."""
    forward = (points - edge[-1]) @ tangent[-1]
    normal = np.array([-tangent[-1, 1], tangent[-1, 0]])
    lateral = (points - edge[-1]) @ normal
    near_tail = (forward >= .22) & (forward <= 1.35) & (np.abs(lateral) < .85)
    if not np.any(near_tail):
        return .50
    s = forward[near_tail]
    error = np.abs(lateral[near_tail] - .5 * curvature[-1] * s * s)
    index = int(np.argmin(s + .45 * error))
    return float(np.clip(1. - error[index] / .55, 0., 1.))


def _boundaries(points, side, width, max_points, max_curvature, laser_x, max_gap=1.65):
    rows = _rows(points, max_points, laser_x, max_gap)
    # Keep short, near prefixes as well as rows with farther support.
    priority = lambda row: _row_priority(row, laser_x)
    short = sorted((row for row in rows if len(row) == 2), key=priority, reverse=True)
    long = sorted((row for row in rows if len(row) > 2), key=priority, reverse=True)
    candidates = []
    for row in short[:5] + long[:8]:
        line = None
        delta = row[-1]-row[0]
        perpendicular = (row-row[0]) @ np.array([-delta[1], delta[0]])/math.hypot(*delta)
        if np.abs(perpendicular).max() <= .035:
            # A straight model avoids turning centimetre-scale ranging noise
            # into curvature, and avoids repeated circle/spline solves.
            mean = row.mean(axis=0)
            offset = row-mean
            slope = float((offset[:, 0] @ offset[:, 1])/(offset[:, 0] @ offset[:, 0]))
            line = (float(mean[1]-slope*mean[0]), slope)
            xs = np.linspace(row[0, 0], row[-1, 0], 36)
            edge = np.column_stack((xs, mean[1]+slope*(xs-mean[0])))
            tangent = np.tile(np.array([1., slope])/math.hypot(1., slope), (36, 1))
            fitted = edge, tangent, np.zeros(36)
        else:
            fitted = fit_curve(row, samples=36)
        if fitted is None:
            continue
        edge, tangent, curvature = fitted
        if np.any(np.diff(edge[:, 0]) <= 1e-4) or np.max(np.abs(curvature)) > max_curvature:
            continue
        denom = 1 + side * width / 2 * curvature
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        center = edge - side * width / 2 * normal
        if np.min(denom) <= .18 or np.any(np.diff(center[:, 0]) <= 1e-4):
            continue
        near, density = _near_support(row, laser_x)
        heading = np.arctan2(tangent[:, 1], tangent[:, 0])
        boundary = Boundary(row, edge, center, heading, curvature / denom,
                            near, density, .5, line=line)
        if line is not None:
            redundant = []
            dominated = False
            span = row[-1, 0]-row[0, 0]
            for existing in candidates:
                if (existing.line is None or abs(line[0]-existing.line[0]) > .02
                        or abs(line[1]-existing.line[1]) > .02):
                    continue
                existing_span = existing.points[-1, 0]-existing.points[0, 0]
                if (existing.near <= near+.02 and existing.density >= density-.05
                        and existing_span >= span-.03):
                    dominated = True
                    break
                if (near <= existing.near+.02 and density >= existing.density-.05
                        and span >= existing_span-.03):
                    redundant.append(existing)
            if dominated:
                continue
            # Remove geometrically equivalent straight prefixes only when
            # a longer row preserves their near and density support.
            candidates = [existing for existing in candidates
                          if all(existing is not old for old in redundant)]
        boundary.continuation = _continuation(row, edge, tangent, curvature, points)
        candidates.append(boundary)
    return candidates


def _previous_y(reference, x):
    if reference is None or len(reference) < 3 or np.any(np.diff(reference[:, 0]) <= 0):
        return None
    return np.interp(x, reference[:, 0], reference[:, 1])


def _opposite_points(points, row, side, width):
    """Find observed cones near the outward normal offset of a fitted row."""
    tangent = np.column_stack((np.cos(row.heading), np.sin(row.heading)))
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    opposite = row.edge - side * width * normal
    opposite = np.vstack((opposite[0] - .35 * tangent[0], opposite,
                          opposite[-1] + .35 * tangent[-1]))
    lateral, _ = relative_to_path(points, opposite)
    along_start = (points - opposite[0]) @ tangent[0]
    along_end = (points - opposite[-1]) @ tangent[-1]
    return points[(np.abs(lateral) <= .30) & (along_start >= -.05) & (along_end <= .05)]


class PairPlanner:
    """Choose a fresh bilateral fit using near support and common geometry."""

    def __init__(self, car, settings):
        self.car = car
        self.track = float(settings.get('track_width_m', car.width))
        self.near_range = float(settings.get('pair_near_range_m', 3.6))
        self.window = float(settings.get('pair_window_m', 2.4))
        self.max_points = int(settings.get('pair_max_points', 8))
        self.min_overlap = float(settings.get('pair_min_overlap_m', .6))
        self.extension = float(settings.get('pair_max_extension_m', settings['transition_extension_m']))
        self.max_curvature = float(settings.get('pair_max_curvature', 1.65))
        self.cut_in = float(settings.get('cut_in_gain', .30))
        self.cut_curv = None
        self.lookahead = float(settings['lookahead_base_m'])
        self.speed_lookahead = float(settings['lookahead_speed_seconds'])
        self.weights = tuple(float(settings.get('pair_weight_' + name, default))
                             for name, default in (('near', .30), ('density', .25),
                                                   ('similarity', .25), ('length', .12),
                                                   ('continuity', .08)))
        values = (self.track, self.near_range, self.window, self.min_overlap,
                  self.extension, self.max_curvature, self.lookahead, self.speed_lookahead, *self.weights)
        if (not all(math.isfinite(value) for value in values)
                or not 1.2 <= self.track <= 2.0 or not 3 <= self.max_points <= 12
                or not .8 <= self.near_range <= 5.0 or not .4 <= self.min_overlap <= self.window
                or not 1.2 <= self.window <= 4.0 or not 0 <= self.extension <= 2.0
                or not 0 < self.max_curvature <= 3.0 or not self.lookahead > 0
                or self.speed_lookahead < 0 or min(self.weights) < 0
                or abs(sum(self.weights)-1.) > 1e-6):
            raise ValueError('Invalid bilateral fit settings')
        self.last_pair = None
        self.last_rows = None
        self.last_row_stats = {}
        self.reject = None

    def _score(self, left, right, grid, previous, reference):
        lo = max(left.center[0, 0], right.center[0, 0])
        hi = min(left.center[-1, 0], right.center[-1, 0], lo + self.window)
        if hi - lo < self.min_overlap or lo > .90 or hi < .48:
            return None
        first = int(np.searchsorted(grid, lo))
        last = int(np.searchsorted(grid, hi, side='right'))
        xs = grid[first:last]
        ly, lh, lk = (sample[first:last] for sample in left.samples)
        ry, rh, rk = (sample[first:last] for sample in right.samples)
        heading = (lh + rh) / 2
        heading_error = np.abs(lh - rh)
        widths = self.track + (ly - ry) * np.cos(heading)
        ordered_widths = sorted(widths.tolist())
        count = len(ordered_widths)
        width = (ordered_widths[(count-1)//2]+ordered_widths[count//2])/2
        variation = ordered_widths[-1]-ordered_widths[0]
        mismatch = float(heading_error.mean())
        if (max(abs(ordered_widths[0]-self.track), abs(ordered_widths[-1]-self.track)) > .40
                or abs(width - self.track) > .28
                or variation > .43 or mismatch > math.radians(23)
                or heading_error.max() > math.radians(43)):
            return None
        y = (ly + ry) / 2
        at_car = float(y[0] - xs[0] * math.tan(heading[0]))
        at_car *= math.cos(heading[0])
        if abs(at_car) > self.track / 2 - self.car.half_width - self.car.margin + .04:
            return None
        curvature_error = float(np.abs(lk-rk).mean())
        if curvature_error > .85:
            return None
        similarity = 1 - (.35 * abs(width - self.track) / .28 + .25 * variation / .43
                          + .25 * mismatch / math.radians(23) + .15 * curvature_error / .85)
        length = float(np.hypot(np.diff(xs), np.diff(y)).sum())
        nearest = (left.near + right.near) / 2
        density = 2 / (1 / left.density + 1 / right.density)
        continuation = (left.continuation + right.continuation) / 2
        terms = (1 / (1 + nearest), min(1., density / 2.4), similarity,
                 length / (length + .85), continuation)
        score = sum(weight * term for weight, term in zip(self.weights, terms))
        score -= .10 * abs(at_car) / .55
        if previous is not None:
            common = (xs >= reference[0, 0]) & (xs <= reference[-1, 0]) & (xs <= 1.35)
            if np.count_nonzero(common) >= 3:
                offset = np.abs(previous[first:last][common] - y[common]) * np.cos(heading[common])
                if offset.max() > .32:
                    return None
                score -= .10 * float(offset.mean()) / .32
        parts = dict(width=width, width_min=ordered_widths[0], width_max=ordered_widths[-1],
                     heading_diff_deg=math.degrees(mismatch), similarity=similarity,
                     overlap_m=length, near_m=nearest, density_per_m=density,
                     continuation=continuation, at_car=at_car, score=score)
        return score, xs, y, heading, (lk + rk) / 2, parts

    def plan(self, cones, speed, now, reference=None, raw_points=None, side_reference=None):
        points = np.asarray(cones, dtype=float).reshape(-1, 2)
        x, y = points.T
        valid = (np.isfinite(points).all(axis=1) & (x >= -.75) & (x <= self.near_range)
                 & (np.abs(y) <= self.track + .9))
        points = points[valid]
        # Use the carried centre only in its observed interval. Beyond it,
        # continue the end tangent for classification, with a bounded shift.
        axis = reference if side_reference is None else side_reference
        lateral = points[:, 1]
        if axis is not None and len(axis) >= 3:
            ref_y = _previous_y(axis, points[:, 0])
            if ref_y is not None:
                for i, j, mask in ((0, 1, points[:, 0] < axis[0, 0]),
                                   (-1, -2, points[:, 0] > axis[-1, 0])):
                    slope = (axis[i, 1] - axis[j, 1]) / (axis[i, 0] - axis[j, 0])
                    dx = np.clip(points[mask, 0] - axis[i, 0], -.8, .8)
                    ref_y[mask] += dx * np.clip(slope, -1.6, 1.6)
                lateral = lateral - ref_y
        left = _boundaries(points[lateral > .32], 1, self.track, self.max_points,
                           self.max_curvature, self.car.laser_x)
        right = _boundaries(points[lateral < -.32], -1, self.track, self.max_points,
                            self.max_curvature, self.car.laser_x)
        self.last_row_stats = {1: {'candidates': len(left)}, -1: {'candidates': len(right)}}
        self.last_pair = None
        self.last_rows = None
        self.reject = 'missing_side' if not left or not right else 'inconsistent_pair'
        grid = np.arange(-.90, self.near_range+.91, .06)
        previous = _previous_y(reference, grid)
        ordinary = [(lrow, rrow) for lrow in left for rrow in right]
        route = self._select(ordinary, grid, previous, reference, speed, raw_points)
        if route is not None:
            return route

        recovered = []
        for side, anchors in ((1, left), (-1, right)):
            for row in sorted(anchors, key=lambda item: _row_priority(item.points, self.car.laser_x),
                              reverse=True)[:5]:
                candidates = _opposite_points(points, row, side, self.track)
                # A cone can be occluded on the outside of a bend. A longer
                # gap is eligible only when backed by the other observed row.
                opposite = _boundaries(candidates, -side, self.track, self.max_points,
                                       self.max_curvature, self.car.laser_x, max_gap=2.20)
                for other in opposite:
                    if set(map(tuple, row.points)).isdisjoint(map(tuple, other.points)):
                        recovered.append((row, other) if side == 1 else (other, row))
        return self._select(recovered, grid, previous, reference, speed, raw_points, recovered_side=True)

    def _select(self, candidates, grid, previous, reference, speed, raw_points, recovered_side=False):
        scored = []
        for lrow, rrow in candidates:
            for row in (lrow, rrow):
                if row.samples is None:
                    row.samples = tuple(np.interp(grid, row.center[:, 0], values)
                                        for values in (row.center[:, 1], row.heading, row.curvature))
            pair = self._score(lrow, rrow, grid, previous, reference)
            if pair is not None:
                scored.append((pair, lrow, rrow))
        for pair, lrow, rrow in sorted(scored, key=lambda p: p[0][0], reverse=True):
            old_cut_curv = self.cut_curv
            route = self._route(pair, lrow, rrow, speed, reference, raw_points)
            if route is not None:
                self.last_pair = pair[-1]
                if recovered_side:
                    self.last_pair['opposite_side_recovery'] = True
                self.last_rows = lrow, rrow
                self.reject = None
                for side, row in ((1, lrow), (-1, rrow)):
                    self.last_row_stats[side].update(points=len(row.points), usable=True,
                                                     density=row.density, near_m=row.near)
                return route
            self.cut_curv = old_cut_curv
        return None

    def _route(self, pair, left, right, speed, reference, raw_points):
        _, x, y, heading, curvature, parts = pair
        old = _previous_y(reference, x)
        if old is not None:
            taper = np.clip(np.minimum(x - reference[0, 0], reference[-1, 0] - x) / .55, 0., 1.)
            # Smoothly taper the carried route at both ends of its overlap.
            weight = .30 * taper**2 * (3 - 2 * taper)
            y = y + weight * np.clip(old - y, -.18, .18)
        # Racing line: the midline of the two offset rows is always the lane
        # centre, so the car never hugs the inside of a bend. Bias it toward
        # the concave side, scaled by curvature and ramped in so the near
        # field stays centred.
        d0 = np.gradient(y, x, edge_order=2)
        bend = np.gradient(np.arctan(d0), x, edge_order=2)/(1+d0**2)**1.5
        if self.cut_in > 0.:
            # Only the smoothed bend scalar is computed here; the shift is
            # applied to the AIM POINT further down. Shifting the fitted
            # centreline made stays_in_corridor() compare the predicted arc
            # against a line the car was not on, blocking every candidate
            # and parking the car at 16.4 m on the 23:42 run.
            mean_k = float(np.mean(bend))
            self.cut_curv = (mean_k if self.cut_curv is None
                             else .30*mean_k+.70*self.cut_curv)
        centers = np.column_stack((x, y))
        # The same differentiable curve supplies the target and its heading.
        derivative = np.gradient(y, x, edge_order=2)
        direction = np.arctan(derivative)
        curvature = np.gradient(derivative, x, edge_order=2)/(1 + derivative**2)**1.5
        preview = float(np.clip(self.lookahead + self.speed_lookahead * abs(speed), .55, 1.35))
        length = np.r_[0., np.cumsum(np.linalg.norm(np.diff(centers, axis=0), axis=1))]
        at_axle = float(np.interp(0., x, length))
        front_gap = max(0., float(x[0])) / max(.5, math.cos(direction[0]))
        aim = at_axle + max(.15, preview - front_gap)
        supported = float(length[-1] - at_axle)
        missing = max(0., aim - length[-1])
        extension = min(missing, self.extension, .30)
        if extension > .01:
            step = np.linspace(extension / 4, extension, 4)
            turn = float(np.clip(curvature[-1], -.75, .75)) * step
            angle = direction[-1] + turn / 2
            tail = centers[-1] + step[:, None] * np.column_stack((np.cos(angle), np.sin(angle)))
            if np.all(np.diff(np.r_[centers[-1, 0], tail[:, 0]]) > 0):
                centers = np.vstack((centers, tail))
                direction = np.r_[direction, direction[-1] + turn]
                curvature = np.r_[curvature, np.full(4, curvature[-1])]
                length = np.r_[length, length[-1] + step]
            else:
                extension = 0.
        if supported < .45:
            self.reject = 'short_support'
            return None
        if raw_points is not None:
            raw = raw_points[(raw_points[:, 0] >= .22)
                             & (raw_points[:, 0] <= min(1.35, centers[-1, 0]))
                             & (np.abs(raw_points[:, 1]) <= 1.9)]
            if len(raw):
                lateral, _ = relative_to_path(raw, centers)
                if np.count_nonzero(np.abs(lateral) < self.car.half_width + self.car.margin) >= 2:
                    self.reject = 'center_blocked'
                    return None
        aim = min(aim, float(length[-1]))
        target = tuple(float(np.interp(aim, length, centers[:, i])) for i in (0, 1))
        if self.cut_in > 0. and self.cut_curv is not None and target[0] > .45:
            ramp = min(1., (target[0]-.45)/.80)
            centered = 1. - np.clip((abs(parts['at_car'])-.12)/.16, 0., 1.)
            cut = self.cut_in*self.track*.5*float(np.clip(self.cut_curv/.45, -1., 1.))*ramp*centered
            target = (target[0], target[1]+cut)
        target_heading = float(np.interp(aim, length, direction))
        if target[0] <= .2 or abs(wrap(math.atan2(target[1], target[0]) - target_heading)) > math.radians(48):
            self.reject = 'unsupported_target'
            return None
        parts.update(extension_m=extension, supported_ahead_m=supported, target_heading=target_heading,
                     curvature_peak_per_m=float(np.max(np.abs(curvature))))
        return DenseRoute(centers, left.points, right.points, centers, 0, parts['width'],
                          curvature, preview, target, target_heading, extension > .01)
