"""Local dense-boundary fitting and PI/PD control; NumPy only."""
from dataclasses import dataclass
import math

import numpy as np

from geometry import relative_to_path, wrap


def detect_right_turn(points, reference=None):
    """Three ~1.5 m edges with a right-angle: A-B along +x, B-C across to
    -y, C-D back along -x. Returns the corner point B, else None. Every left
    point is tried as B, so far/background cones simply fail the length and
    direction checks instead of needing an arbitrary cutoff."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) < 6:
        return None
    left = pts[pts[:, 1] > .2]
    right = pts[pts[:, 1] < -.2]
    if len(left) < 4 or len(right) < 3:
        return None
    for b in left[np.argsort(-left[:, 0])]:
        if float(b[0]) < 1.2:
            continue
        a = left[int(np.argmin(np.linalg.norm(left-(b-np.array([1.5, 0.])), axis=1)))]
        c = right[int(np.argmin(np.linalg.norm(right-b, axis=1)))]
        d = right[int(np.argmin(np.linalg.norm(right-(c-np.array([1.4, 0.])), axis=1)))]
        if not 1.35 <= float(np.linalg.norm(a-b)) <= 1.70:
            continue
        if abs(float(a[1]-b[1])) > .5:
            continue
        if not 1.25 <= float(np.linalg.norm(b-c)) <= 1.60:
            continue
        if float(c[1]) > float(b[1])-1.0:
            continue
        if not 1.25 <= float(np.linalg.norm(c-d)) <= 1.60:
            continue
        if float(d[0]) > float(c[0])-.8:
            continue
        return (float(b[0]), float(b[1]))
    return None


def entrance_chain(cones, side, car, reference=None, expected_width=None):
    """Follow nearby connected cones, anchored at the actual entrance."""
    points = np.asarray(cones, dtype=float).reshape(-1, 2)
    points = points[np.all(np.isfinite(points), axis=1)]
    lateral, tangents = relative_to_path(points, reference)
    valid = ((points[:, 0] > car.laser_x) & (points[:, 0] < 3.5)
             & (np.abs(lateral) < 1.4*car.width))
    points, lateral, tangents = points[valid], lateral[valid], tangents[valid]
    seeds = np.flatnonzero((side*lateral > 0) & (points[:, 0] <= 1.5))
    if len(seeds) == 0:
        return np.empty((0, 2))
    if reference is None or expected_width is None:
        seed = min(seeds, key=lambda i: float(np.linalg.norm(points[i])))
    else:
        seed = min(seeds, key=lambda i: abs(lateral[i]-side*expected_width/2)
                   + .2*float(np.linalg.norm(points[i])))
    indices = [int(seed)]
    heading = math.atan2(tangents[seed, 1], tangents[seed, 0])
    for _ in range(8):
        delta = points-points[indices[-1]]
        distance = np.linalg.norm(delta, axis=1)
        angle = np.arctan2(delta[:, 1], delta[:, 0])
        turn = np.arctan2(np.sin(angle-heading), np.cos(angle-heading))
        candidates = np.flatnonzero((delta[:, 0] > .12) & (distance <= 2*car.width)
                                   & (np.abs(turn) <= .82))
        if len(candidates) == 0:
            break
        j = min(candidates, key=lambda i: float(distance[i])*(1+.25*abs(float(turn[i]))))
        indices.append(int(j))
        heading = float(angle[j])
    return points[indices]


def fit_circle(points):
    """Kasa circle through a row. Returns (cx, cy, R, rms) or None."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(points) < 3:
        return None
    x, y = points[:, 0], points[:, 1]
    matrix = np.column_stack((x, y, np.ones_like(x)))
    rhs = x*x+y*y
    try:
        solution, *_ = np.linalg.lstsq(matrix, rhs, rcond=None)
    except np.linalg.LinAlgError:
        return None
    cx, cy = solution[0]/2.0, solution[1]/2.0
    r2 = solution[2]+cx*cx+cy*cy
    if not np.isfinite(r2) or r2 <= 0.0:
        return None
    radius = math.sqrt(r2)
    rms = math.sqrt(np.mean((np.hypot(x-cx, y-cy)-radius)**2))
    return cx, cy, radius, rms


def fit_arc(points, samples=60):
    """Arc parameterised by angle: handles the closing ")" and hairpins."""
    result = fit_circle(points)
    if result is None:
        return None
    cx, cy, radius, rms = result
    if not (.30 <= radius <= 3.5) or rms > .18:
        return None
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    angle = np.unwrap(np.arctan2(points[:, 1]-cy, points[:, 0]-cx))
    span = angle[-1]-angle[0]
    # The sign of the unwrapped angle span is not the turn direction: a
    # right-turning row can unwrap with a positive span when it crosses the
    # -pi/+pi branch. Decide from the polyline cross product instead --
    # getting this wrong returned +0.29 1/m on a right-hand U and the car
    # drove straight until the fit flipped to -106 deg.
    arm_a = points[1:-1]-points[:-2]
    arm_b = points[2:]-points[1:-1]
    turn = float(np.sum(arm_a[:, 0]*arm_b[:, 1]-arm_a[:, 1]*arm_b[:, 0]))
    if abs(turn) < 1e-9:
        return None
    if (turn > 0) != (span > 0):
        angle = angle.copy()
        angle[1:] -= 2*math.pi*math.copysign(1.0, span)
        span = angle[-1]-angle[0]
    if abs(span) < .8 or abs(span) > 2.2*math.pi:
        return None
    sample = np.linspace(angle[0], angle[-1], samples)
    curve = np.column_stack((cx+radius*np.cos(sample), cy+radius*np.sin(sample)))
    sign = 1.0 if turn > 0 else -1.0
    tangent = np.column_stack((-np.sin(sample)*sign, np.cos(sample)*sign))
    curvature = np.full(samples, sign/radius)
    return curve, tangent, curvature


def fit_quadratic(points, samples=60):
    """Least-squares parabola for short rows.

    A natural cubic spline through 3-5 cones puts most of its curvature at
    the two ends: at the S exit a 4-point left row reached +/-1.5 1/m and the
    geometric guard dropped the correct inside row, so the car followed the
    outside row around the cones. A quadratic has no end conditions to
    overshoot and is stable for exactly this many points.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    order = np.argsort(points[:, 0])
    points = points[order]
    keep = np.r_[True, np.diff(points[:, 0]) > 1e-3]
    points = points[keep]
    if len(points) < 2:
        return None
    x, y = points[:, 0], points[:, 1]
    span = float(np.ptp(x))
    if span < .10:
        return None
    xs = np.linspace(x[0], x[-1], samples)
    if len(points) >= 3:
        a, b, c = np.polyfit(x, y, 2)
        ys = a*xs*xs+b*xs+c
        dy = 2*a*xs+b
        ddy = np.full_like(xs, 2*a)
    else:
        b, c = np.polyfit(x, y, 1)
        ys = b*xs+c
        dy = np.full_like(xs, b)
        ddy = np.zeros_like(xs)
    norm = np.hypot(1.0, dy)
    curve = np.column_stack((xs, ys))
    tangent = np.column_stack((1.0/norm, dy/norm))
    curvature = ddy/norm**3
    return curve, tangent, curvature


def fit_curve(points, samples=60):
    """Natural parametric cubic spline: position, tangent and curvature."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(points) < 2:
        return None
    # Tight, clearly-curved rows (the U-turn) are arcs; a y=f(x)
    # quadratic flattens "approach + turn" into a wrong heading. Gentle
    # bends fall through to the quadratic/spline below.
    arc = fit_arc(points, samples)
    if arc is not None:
        return arc
    if len(points) <= 5:
        return fit_quadratic(points, samples)
    length = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if np.any(length < .05):
        return None
    knots = np.r_[0., np.cumsum(length)]
    second = np.zeros_like(points)
    if len(points) > 2:
        matrix = np.zeros((len(points)-2, len(points)-2))
        rhs = np.empty((len(points)-2, 2))
        for j in range(1, len(points)-1):
            i = j-1
            matrix[i, i] = 2*(length[j-1]+length[j])
            if i > 0:
                matrix[i, i-1] = length[j-1]
            if i+1 < len(matrix):
                matrix[i, i+1] = length[j]
            rhs[i] = 6*((points[j+1]-points[j])/length[j]
                        - (points[j]-points[j-1])/length[j-1])
        second[1:-1] = np.linalg.solve(matrix, rhs)
    s = np.linspace(0., knots[-1], samples)
    idx = np.minimum(np.searchsorted(knots, s, side='right')-1, len(length)-1)
    h = length[idx, None]
    a, b = (knots[idx+1]-s)[:, None]/h, (s-knots[idx])[:, None]/h
    curve = (a*points[idx]+b*points[idx+1]
             + ((a**3-a)*second[idx]+(b**3-b)*second[idx+1])*h*h/6)
    derivative = ((points[idx+1]-points[idx])/h
                  + ((-3*a*a+1)*second[idx]+(3*b*b-1)*second[idx+1])*h/6)
    acceleration = a*second[idx]+b*second[idx+1]
    norm = np.linalg.norm(derivative, axis=1)
    if np.min(norm) < 1e-5:
        return None
    tangent = derivative/norm[:, None]
    curvature = (derivative[:, 0]*acceleration[:, 1]
                 - derivative[:, 1]*acceleration[:, 0])/norm**3
    return curve, tangent, curvature


def boundary_density(row):
    if len(row) < 2:
        return 0.
    return (len(row)-1)/float(np.sum(np.linalg.norm(np.diff(row, axis=0), axis=1)))


def measure_width(opposite, curve, side, car):
    """Measure perpendicular cross-sections, not rank-by-rank cone pairs."""
    if len(opposite) == 0:
        return None
    lateral, _ = relative_to_path(opposite, curve)
    widths = -side*lateral
    usable = ((opposite[:, 0] >= curve[0, 0]-.40)
              & (opposite[:, 0] <= curve[-1, 0]+.40)
              & (widths > 2*(car.half_width+car.margin)+.20)
              & (widths < 1.4*car.width))
    widths = widths[usable]
    if len(widths) < 1:
        return None
    median = float(np.median(widths))
    widths = widths[np.abs(widths-median) <= .2*median]
    return float(np.median(widths)) if len(widths) >= 1 else None


def nearest_width(left, right):
    """Closest left-right cone pair, used only to initialise the width.

    At a bend the two rows are not parallel in view, so the perpendicular
    cross-sections disagree and the median filter can reject all of them;
    without a width the planner never starts. The closest pair is still the
    lane width at the car's cross-section.
    """
    if len(left) == 0 or len(right) == 0:
        return None
    distance = np.linalg.norm(left[:, None, :]-right[None, :, :], axis=2)
    width = float(np.min(distance))
    return width if .80 <= width <= 1.80 else None


@dataclass
class DenseRoute:
    centers: np.ndarray
    left: np.ndarray
    right: np.ndarray
    baseline: np.ndarray
    side: int
    width: float
    curvature: np.ndarray
    lookahead: float
    target: tuple
    heading: float
    predicted: bool


def row_straight(points):
    """True when consecutive cone gaps are uniform and the row is straight."""
    if len(points) < 3:
        return False
    delta = np.diff(points, axis=0)
    gaps = np.linalg.norm(delta, axis=1)
    if len(gaps) < 2 or float(np.mean(gaps)) <= .05:
        return False
    angles = np.arctan2(delta[:, 1], delta[:, 0])
    turns = np.abs(np.arctan2(np.sin(np.diff(angles)), np.cos(np.diff(angles))))
    uniform = float(np.std(gaps))/float(np.mean(gaps)) < .35
    return (uniform and float(np.mean(turns)) < .12
            and float(np.median(gaps)) > .35)


def paired_inner_side(curves, usable, width):
    """Return the paired bend and whether its approach is in the local preview."""
    if len(usable) != 2:
        return None, False
    profiles = []
    for side in (1, -1):
        edge, tangent, curvature = curves[side]
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        center = edge-side*(width/2)*normal
        if np.any(np.diff(center[:, 0]) <= 0) or np.min(1+side*(width/2)*curvature) <= .15:
            return None, False
        heading = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
        profiles.append((center, heading))
    start = max(.35, *(center[0, 0] for center, _ in profiles))
    end = min(2.1, *(center[-1, 0] for center, _ in profiles))
    if end-start < .45:
        return None, False
    x = np.linspace(start, end, 9)
    (left, left_heading), (right, right_heading) = profiles
    center_gap = np.interp(x, left[:, 0], left[:, 1])-np.interp(x, right[:, 0], right[:, 1])
    lh = np.interp(x, left[:, 0], left_heading)
    rh = np.interp(x, right[:, 0], right_heading)
    heading_gap = np.arctan2(np.sin(lh-rh), np.cos(lh-rh))
    if np.max(np.abs(center_gap)) > .20*width or np.max(np.abs(heading_gap)) > math.radians(25.):
        return None, False
    left_turn, right_turn = lh[-1]-lh[0], rh[-1]-rh[0]
    if min(left_turn, right_turn) >= math.radians(8.):
        inner = 1
    elif max(left_turn, right_turn) <= -math.radians(8.):
        inner = -1
    else:
        return None, False
    # A visible reverse arc can still start beyond a left/right approach.
    probe = min(end, max(1.15, start+.25))
    near = start <= 1.25 and min(inner*np.interp(probe, x, h) for h in (lh, rh)) >= -math.radians(18.)
    return inner, bool(near)


class DensePlanner:
    def __init__(self, car, base_lookahead=.65, speed_lookahead=.8, extension=.25,
                 follow_offset=.35, corner_offset=.45, start_side=0):
        self.car = car
        self.base_lookahead = base_lookahead
        self.speed_lookahead = speed_lookahead
        self.extension = extension
        self.follow_offset = follow_offset
        self.corner_offset = corner_offset
        self.start_side = start_side
        self.tight_latch = False
        self.tight_at = None
        self.tight_dir0 = None
        self.side = None
        self.pending_side = None
        self.pending_frames = 0
        self.pending_paired_since = None
        self.upcoming_side = None
        self.upcoming_at = None
        self.inner_reference_side = None
        self.width = None
        self.width_at = None
        self.switch_at = None
        self.left_since = None
        self.left_lost_frames = 0
        self.hold_frames = 0
        self.last_heading = None
        self.left_heading = None
        self.exit_reason = None
        self.last_row_stats = None
        self.smooth_target = None
        self.smooth_heading = None
        self.smooth_at = None
        self.coast_frames = 0

    def _hold_route(self, speed, left, right, heading=None):
        """Straight continuation along the last accepted heading.

        When the left row thins out at the S exit, neither switching to the
        right row (it turns away) nor stopping immediately is right: hold the
        last heading for up to 1 s, then let the caller declare the S done.
        """
        if heading is None:
            heading = 0.0 if self.last_heading is None else self.last_heading
        width = self.width if self.width is not None else 1.42
        look = max(.45, min(1.2, .7+.25*abs(speed)))
        distance = np.linspace(.05, max(1.2, look+.6), 30)
        centers = np.column_stack((distance*math.cos(heading),
                                   distance*math.sin(heading)))
        tangent = np.tile([math.cos(heading), math.sin(heading)], (len(distance), 1))
        curvature = np.zeros(len(distance))
        target = (look*math.cos(heading), look*math.sin(heading))
        return DenseRoute(centers, left, right, centers, 1, width,
                          curvature, look, target, heading, True)

    def hold(self, speed):
        """Public straight hold along the last accepted heading."""
        if self.last_heading is None:
            return None
        empty = np.empty((0, 2))
        return self._hold_route(speed, empty, empty)

    def plan(self, cones, speed, now, reference=None):
        corner = detect_right_turn(cones, reference)
        self.right_turn = corner is not None
        self.corner_target = corner
        left = entrance_chain(cones, 1, self.car, reference, self.width)
        right = entrance_chain(cones, -1, self.car, reference, self.width)
        # A cone cannot be part of both boundaries. At the S transition the two
        # rows converge and one cone can land in both; 0.03 m is smaller than
        # the cone radius, so a shared cone used to survive as two points.
        # 0.12 m and dropping it from both rows keeps the width honest.
        if len(left) and len(right):
            shared = np.linalg.norm(left[:, None, :]-right[None, :, :], axis=2) < .12
            if np.any(shared):
                left = np.delete(left, np.flatnonzero(np.any(shared, axis=1)), axis=0)
                right = np.delete(right, np.flatnonzero(np.any(shared, axis=0)), axis=0)
        rows = {1: left, -1: right}
        curves = {s: fit_curve(row) for s, row in rows.items()}
        usable = [s for s in (1, -1) if curves[s] is not None]
        # Record both rows' stats every frame, even when neither row fuses:
        # point count, density, end heading and curvature range.
        self.last_row_stats = {}
        for s in (1, -1):
            entry = {'points': int(len(rows[s])),
                     'density': round(float(boundary_density(rows[s])), 3),
                     'usable': bool(curves[s] is not None)}
            curve_s = curves[s]
            if curve_s is not None:
                _, tangent_s, curvature_s = curve_s
                entry['heading_end_deg'] = round(
                    math.degrees(math.atan2(tangent_s[-1, 1], tangent_s[-1, 0])), 1)
                entry['curv_min'] = round(float(curvature_s.min()), 3)
                entry['curv_max'] = round(float(curvature_s.max()), 3)
            self.last_row_stats[s] = entry
        if not usable:
            return None
        # A 4-point row can fit a spurious 1.4 1/m hook at the second S bend,
        # making the offset centreline tighter than the car's minimum radius.
        # Drop such a side before the density vote, so a valid row is used
        # instead of stopping the car at the bend entrance.
        geometric = {}
        for s in usable:
            curve_s, tangent_s, curvature_s = curves[s]
            measured = measure_width(rows[-s], curve_s, s, self.car)
            width_s = self.width if measured is None else measured
            if width_s is None:
                geometric[s] = True
                continue
            d = (self.width if self.width is not None else 1.42)/2
            den = 1+s*d*curvature_s
            geometric[s] = bool(np.min(den) > .15)
        valid = [s for s in usable if geometric[s]]
        if valid:
            usable = valid
        for s in (1, -1):
            if self.last_row_stats is not None:
                self.last_row_stats[s]['usable'] = s in usable
        if self.side is None and self.start_side in (1, -1) and self.start_side in usable:
            # The S starts against the right row; without this the density
            # vote sometimes picked the left row at the entrance and the car
            # drove straight off to the left.
            self.side = self.start_side
        if self.side == 1 and (1 not in usable or len(rows[1]) < 2):
            # Left row thinned out at the S exit. Keep rolling on the last
            # heading instead of stopping or switching to the right row.
            return self._hold_route(speed, left, right)
        strong = [s for s in usable if len(rows[s]) >= 3]
        pool = strong if strong else usable
        inner, near_bend = paired_inner_side(curves, usable, self.width or self.car.width)
        if inner is not None:
            self.upcoming_side = inner if not near_bend and inner != self.side else None
            self.upcoming_at = now if self.upcoming_side is not None else None
        elif self.upcoming_at is not None and now-self.upcoming_at > .45:
            self.upcoming_side, self.upcoming_at = None, None
        aim_side = None
        if (self.smooth_at is not None and 0 <= now-self.smooth_at <= .25
                and self.smooth_target[0] > .12):
            bearing = math.atan2(self.smooth_target[1], self.smooth_target[0])
            if min(bearing, self.smooth_heading) > math.radians(5.):
                aim_side = 1
            elif max(bearing, self.smooth_heading) < -math.radians(5.):
                aim_side = -1
        scores = {s: boundary_density(rows[s])*(1.5 if s == inner else 1.)
                  *(1.2 if inner is None and s == aim_side else 1.) for s in usable}
        for s in usable:
            self.last_row_stats[s]['inner'] = s == inner
            self.last_row_stats[s]['near_bend'] = s == inner and near_bend
            self.last_row_stats[s]['upcoming'] = s == self.upcoming_side
            self.last_row_stats[s]['selection_score'] = round(scores[s], 3)
        candidate = max(pool, key=scores.get)
        if near_bend:
            candidate = inner
        elif self.upcoming_side is not None and self.side in usable:
            candidate = self.side
        side = self.side if self.side in usable else candidate
        recent_paired_vote = (self.pending_paired_since is not None
                             and now-self.pending_paired_since <= .55)
        if recent_paired_vote and inner is None and self.side in usable:
            # An ambiguous scan neither confirms nor contradicts a nearby bend.
            pass
        elif candidate == self.side or self.side is None:
            self.pending_side, self.pending_frames = None, 0
            self.pending_paired_since = None
        elif self.side == 1 and candidate == -1 and not near_bend:
            # Preserve the exit's left-row lock against an uncorroborated right row.
            side = 1
            self.pending_side, self.pending_frames = None, 0
            self.pending_paired_since = None
        else:
            confirmed_geometry = near_bend
            stronger = (self.side not in usable or confirmed_geometry
                        or scores[candidate] > 1.25*scores[self.side])
            cooling_down = self.switch_at is not None and now-self.switch_at < 3.0
            can_vote = stronger and (not cooling_down or confirmed_geometry or self.side not in usable)
            if can_vote:
                same_vote = self.pending_side == candidate and (
                    recent_paired_vote if confirmed_geometry else self.pending_paired_since is None)
                self.pending_frames = self.pending_frames+1 if same_vote else 1
                self.pending_side = candidate
                if not same_vote:
                    self.pending_paired_since = now if confirmed_geometry else None
            else:
                self.pending_side, self.pending_frames = None, 0
                self.pending_paired_since = None
            if self.pending_frames >= 3:
                accept = True
                if reference is not None and self.width is not None:
                    edge, tangent_c, _ = curves[candidate]
                    normal_c = np.column_stack((-tangent_c[:, 1], tangent_c[:, 0]))
                    centers_c = edge-candidate*(self.width/2)*normal_c
                    lo, hi = reference[0, 0], reference[-1, 0]
                    if confirmed_geometry:
                        lo, hi = max(.35, lo), min(1.8, hi)
                    overlap = centers_c[(centers_c[:, 0] >= lo) & (centers_c[:, 0] <= hi)]
                    accept = len(overlap) >= 2
                    if accept:
                        residual, _ = relative_to_path(overlap, reference)
                        accept = np.max(np.abs(residual)) <= .2*self.width
                if accept:
                    side = candidate
                self.pending_side, self.pending_frames = None, 0
                self.pending_paired_since = None
        for s in usable:
            self.last_row_stats[s]['switch_votes'] = self.pending_frames if self.pending_side == s else 0
        curve, tangent, curvature = curves[side]
        observed_width = measure_width(rows[-side], curve, side, self.car)
        if observed_width is None and self.width is None:
            observed_width = nearest_width(rows[1], rows[-1])
        if observed_width is not None:
            if self.width is None:
                self.width = observed_width
            else:
                change = observed_width-self.width
                # A >0.20 m jump is a row mix-up at the S transition, not a
                # narrower lane: keep the last trusted width. The measurement
                # still counts as fresh, or one bad patch would time the route
                # out after the 1.0 s stale timeout.
                if abs(change) <= .20:
                    self.width += .15*change
            self.width_at = now
        elif self.width is None:
            # Width is now only used for logging: the centreline offset is the
            # fixed follow_offset. Without this fallback a patch where the
            # opposite row is out of view stopped the route after 1 s.
            self.width = nearest_width(rows[1], rows[-1]) or 1.42
        if side is not None:
            side_k = float(np.max(np.abs(curves[side][2])))
            if side_k > .45 and not self.tight_latch:
                self.tight_latch = True
                self.tight_at = now
            elif (side_k < .25 and self.tight_at is not None
                  and now-self.tight_at > 1.0):
                self.tight_latch = False
        tight = self.tight_latch
        # Keep the offset policy tied to the accepted baseline. A single missed
        # inner-edge vote should not change the lateral position of the path.
        follow_inner = side == inner or side == self.inner_reference_side
        half_width = (self.width if self.width is not None else 1.42)/2
        distance = .62 if tight and not follow_inner else half_width
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        centers = curve-side*distance*normal
        denominator = 1+side*distance*curvature
        if np.min(denominator) <= .15:
            return None
        # A tight bend can make the offset centreline turn back in x near the
        # far end of the fit. That tail is outside the lookahead, so cut it
        # instead of throwing the whole route away -- this is what stopped the
        # car right after the first S bend (dx = -0.9 mm at 3.8 m, lookahead
        # target at 0.7 m).
        reversals = np.flatnonzero(np.diff(centers[:, 0]) <= 0)
        if len(reversals):
            cut = int(reversals[0])+1
            if cut < 8:
                return None
            centers = centers[:cut]
            tangent = tangent[:cut]
            curvature = curvature[:cut]
            denominator = denominator[:cut]
        center_curvature = curvature/denominator
        length = np.r_[0., np.cumsum(np.linalg.norm(np.diff(centers, axis=0), axis=1))]
        preview = self.base_lookahead+self.speed_lookahead*abs(speed)
        preview /= 1+.3*float(np.max(np.abs(center_curvature[:len(curve)//2])))
        preview = max(.45, min(1.1, preview))
        remaining = max(0., preview-max(0., centers[0, 0]))
        # When the visible centreline starts ahead of the car, the lookahead
        # point can land almost beside it: at the last curve that made the
        # bearing ~66 deg and saturated the steering into the inside cone.
        # Give a nearby fit up to 0.55 m of forward support without extending
        # a fit that already starts beyond the nominal preview.
        remaining = max(remaining, min(.55, length[-1], max(0., 1.65-centers[0, 0])))
        predicted = remaining > length[-1]
        if predicted:
            extra = remaining-length[-1]
            if extra > self.extension:
                return None
            target = centers[-1]+extra*tangent[-1]
        else:
            target = np.array([np.interp(remaining, length, centers[:, i]) for i in (0, 1)])
        direction = np.array([np.interp(min(remaining, length[-1]), length, tangent[:, i])
                              for i in (0, 1)])
        heading = math.atan2(direction[1], direction[0])
        # Both rows visible: aim at the midpoint of the two offset
        # centrelines. This is the corridor centre from LiDAR alone, not a
        # hard-coded "hug left/right" rule, so S/90/hairpin need no track map.
        if -side in usable and not tight and not follow_inner:
            curve_o, tangent_o, _ = curves[-side]
            normal_o = np.column_stack((-tangent_o[:, 1], tangent_o[:, 0]))
            centers_o = curve_o-(-side)*distance*normal_o
            if centers_o[0, 0] <= target[0] <= centers_o[-1, 0]:
                y_o = float(np.interp(target[0], centers_o[:, 0], centers_o[:, 1]))
                index = int(np.searchsorted(centers_o[:, 0], target[0]))
                index = min(max(index, 0), len(centers_o)-1)
                dir_o = tangent_o[index]
                heading_o = math.atan2(dir_o[1], dir_o[0])
                delta_h = math.atan2(math.sin(heading_o-heading),
                                     math.cos(heading_o-heading))
                # Only blend when both rows agree on the direction. At the
                # U exit the left row can be down to 2 points while the right
                # row is 3; the old point-count weight then pulled the aim
                # point across to the right row and the car turned away from
                # the inner red cones.
                if abs(delta_h) <= math.radians(25):
                    gap = abs(float(target[1])-y_o)
                    if gap <= .20:
                        weight = .5
                    else:
                        weight = 1.0
                    target = np.array([target[0],
                                       weight*target[1]+(1.0-weight)*y_o])
                    hx = weight*math.cos(heading)+(1.0-weight)*dir_o[0]
                    hy = weight*math.sin(heading)+(1.0-weight)*dir_o[1]
                    heading = math.atan2(hy, hx)
        # A real steering change is bounded by the servo rate; a >40 deg jump
        # between two plans is a fit glitch (at the last curve the fitted
        # heading flipped +59 -> -56 and the target jumped across the track).
        # Hold the previous heading until the fit is continuous again.
        if self.last_heading is not None:
            jump = math.atan2(math.sin(heading-self.last_heading),
                              math.cos(heading-self.last_heading))
            if abs(jump) > math.radians(40):
                return self._hold_route(speed, left, right)
        # The final ")" hooks right while we are on the left row. Once the
        # route has turned more than 40 deg away from the heading we entered
        # the left segment with, go straight along that heading and keep
        # looking for the left group instead of following the right hook.
        if (self.side == 1 and side == 1 and self.left_heading is not None
                and not (near_bend and inner == -1)
                and heading-self.left_heading < -math.radians(40)):
            return self._hold_route(speed, left, right)
        if (self.side is not None and self.side in usable
                and side != self.side and reference is not None
                and not (near_bend and side == inner)):
            residual, _ = relative_to_path(centers, reference)
            overlap = (centers[:, 0] >= reference[0, 0]) & (centers[:, 0] <= reference[-1, 0])
            if not np.any(overlap) or np.max(np.abs(residual[overlap])) > .2*self.width:
                return None
        if side == 1 and heading < -.1 and not follow_inner:
            # Coming out of the U on the left row, the exit should keep
            # hugging the left (red) group, not drift to the right-front.
            # Shift only the aim point; row selection is untouched.
            shift = max(0., distance-self.corner_offset)
            target = np.array([target[0], target[1]+shift])
        # The U exit used to jump the aim point across the corridor in one
        # frame (y -0.19 -> +0.10), swinging the servo left-right-left into
        # the right cones. Smooth the aim point, not the route.
        if self.smooth_target is None:
            self.smooth_target = np.asarray(target, dtype=float).copy()
            self.smooth_heading = heading
        else:
            alpha = .5
            if follow_inner:
                self.smooth_target = np.asarray(target, dtype=float).copy()
            else:
                self.smooth_target = alpha*np.asarray(target, dtype=float)+(1.-alpha)*self.smooth_target
            delta = math.atan2(math.sin(heading-self.smooth_heading),
                               math.cos(heading-self.smooth_heading))
            self.smooth_heading = math.atan2(
                math.sin(self.smooth_heading+alpha*delta),
                math.cos(self.smooth_heading+alpha*delta))
        target = tuple(float(v) for v in self.smooth_target)
        heading = self.smooth_heading
        self.last_heading = heading
        self.smooth_at = now
        if self.side != side:
            self.switch_at = now
            self.left_since = now if side == 1 else None
            self.left_heading = heading if side == 1 else None
        self.side = side
        self.inner_reference_side = side if follow_inner else None
        return DenseRoute(centers, left, right, curve, side, self.width,
                          center_curvature, preview, tuple(float(v) for v in target), heading, predicted)


class SpeedPI:
    def __init__(self, target=.30, kp=60., ki=20., feedforward=1535., maximum=1558.,
                 minimum=1500., slope=0.):
        self.target, self.kp, self.ki = target, kp, ki
        self.feedforward, self.maximum = feedforward, maximum
        self.minimum = minimum
        self.slope = slope
        self.integral = 0.

    def reset(self):
        self.integral = 0.

    def update(self, speed, dt):
        error = self.target-speed
        candidate = self.integral+error*max(0., min(.1, dt))
        # feedforward is the zero-speed intercept; slope is pwm per m/s.
        # Measured on the car pwm ~ 1497.5 + 18.2*v, so a fixed intercept
        # alone commanded ~1.3 m/s at every target and could never brake:
        # asking 0.8 while doing 1.33 still yielded 1514 (throttle).
        raw = (self.feedforward+self.slope*self.target
               +self.kp*error+self.ki*candidate)
        output = max(self.minimum, min(self.maximum, raw))
        if output == raw or (output >= self.maximum and error < 0) or (output <= self.minimum and error > 0):
            self.integral = candidate
        return output


class AnglePD:
    def __init__(self, kp=1.2, kd=.15, filter_seconds=.15, max_steer=16.):
        self.kp, self.kd, self.filter_seconds = kp, kd, filter_seconds
        self.max_steer = max_steer
        self.previous = None
        self.rate = 0.

    def reset(self):
        self.previous, self.rate = None, 0.

    def update(self, error, dt):
        dt = max(.001, min(.1, dt))
        raw_rate = 0. if self.previous is None else wrap(error-self.previous)/dt
        self.rate += dt/(self.filter_seconds+dt)*(raw_rate-self.rate)
        self.previous = error
        return max(-self.max_steer, min(self.max_steer,
                                        math.degrees(self.kp*error+self.kd*self.rate)))
