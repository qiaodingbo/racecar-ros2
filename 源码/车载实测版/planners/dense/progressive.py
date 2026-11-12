"""Plan from re-observed near cones and preview farther bilateral continuations."""
from dataclasses import dataclass
import math

import numpy as np

from geometry import relative_to_path, wrap
from pairing import PairPlanner
from corner_transition import CornerTransition


@dataclass
class ConeTrack:
    world: np.ndarray
    seen: float
    frame: int
    near_hits: int
    pending_far: bool


class ConeHistory:
    def __init__(self, near_range):
        self.near_range = near_range
        self.segment = None
        self.tracks = []
        self.frame = 0

    def reset(self, segment=None):
        self.segment = segment
        self.tracks = []

    def update(self, points, pose, now):
        """Associate actual detections in the run's odometry frame."""
        if self.segment != pose['segment']:
            self.reset(pose['segment'])
        self.frame += 1
        a = pose['yaw_rad']
        c, s = math.cos(a), math.sin(a)
        world = points @ np.array([[c,s],[-s,c]]) + [pose['x_m'],pose['y_m']]
        old = [track for track in self.tracks if now-track.seen <= .60]
        matched = {}
        used = set()
        if old and len(world):
            errors = np.linalg.norm(world[:,None,:]-np.asarray([p.world for p in old]),axis=2)
            indices = np.argwhere(errors <= .16)
            for i,j in sorted(indices,key=lambda ij:errors[tuple(ij)]):
                if i not in matched and j not in used:
                    matched[i] = old[j]
                    used.add(j)
        current = []
        for i,(point,xy) in enumerate(zip(points,world)):
            previous = matched.get(i)
            near = point[0] <= self.near_range
            hits = 0
            if near:
                hits = 1
                if (previous is not None and previous.frame == self.frame-1
                        and now-previous.seen <= .25):
                    hits += previous.near_hits
            pending = not near or (previous is not None and previous.pending_far)
            current.append(ConeTrack(xy,now,self.frame,min(hits,12),pending))
        # Keep briefly occluded identities, without giving them confirmation credit.
        self.tracks = (current+[track for j,track in enumerate(old) if j not in used])[:96]
        return current


def startup_axis(points, near_range):
    """Estimate a side-classification axis from the two near rows."""
    x,y = points.T
    close = points[(x >= -.65) & (x <= .85) & (np.abs(y) >= .40) & (np.abs(y) <= 1.18)]
    lines = []
    for side in (1,-1):
        row = close[side*close[:,1] > 0]
        if len(row) < 2 or np.ptp(row[:,0]) < .35:
            return None
        mean = row.mean(axis=0)
        dx,dy = (row-mean).T
        slope = float((dx @ dy)/(dx @ dx))
        intercept = float(mean[1]-slope*mean[0])
        lines.append((intercept,slope,row[:,0].min(),row[:,0].max()))
    left,right = lines
    intercept,slope = (np.asarray(left[:2])+right[:2])/2
    shared = min(left[3],right[3])-max(left[2],right[2])
    width = (left[0]-right[0])/math.hypot(1.,slope)
    if (shared < .20 or not 1.05 <= width <= 1.85 or abs(intercept) > .32
            or max(abs(left[1]),abs(right[1])) > .95
            or abs(math.atan(left[1])-math.atan(right[1])) > .45):
        return None
    x = np.array([-.75,0.,near_range])
    return np.column_stack((x,intercept+slope*x))


def tail_choices(row, points):
    """Search the next few observed cones with bounded gaps and turn angles."""
    delta = row.edge[-1]-row.edge[-2]
    heading = math.atan2(delta[1],delta[0])
    frontier = [(0.,row.edge[-1],heading,[])]
    choices = [[]]
    for _ in range(3):
        next_frontier = []
        for cost,tip,direction,chain in frontier:
            delta = points-tip
            gap = np.hypot(delta[:,0],delta[:,1])
            angle = np.arctan2(delta[:,1],delta[:,0])
            turn = np.abs(angle-direction)
            for j in np.flatnonzero((delta[:,0] >= .22) & (gap >= .30) & (gap <= 1.55)
                                    & (turn <= math.radians(55))):
                next_frontier.append((cost+float(gap[j]+.55*turn[j]),
                                      points[j],angle[j],chain+[points[j]]))
        frontier = sorted(next_frontier,key=lambda item:item[0])[:3]
        choices.extend(item[-1] for item in frontier)
    return choices


def attach_tail(edge, points):
    """Interpolate a continuation with the incumbent edge tangent at the join."""
    slope = np.gradient(edge[:,1],edge[:,0],edge_order=2)
    if not points:
        return edge,slope
    knots = np.vstack((edge[-1],points))
    dx,dy = np.diff(knots,axis=0).T
    chord = dy/dx
    tangent = np.r_[slope[-1],.5*(chord[:-1]+chord[1:]),chord[-1]]
    segments, slopes = [edge], [slope]
    t = np.linspace(0.,1.,14)[1:]
    for i,length in enumerate(dx):
        y0,y1 = knots[i:i+2,1]
        d0,d1 = tangent[i:i+2]
        y = (2*t**3-3*t**2+1)*y0+(t**3-2*t**2+t)*length*d0
        y += (-2*t**3+3*t**2)*y1+(t**3-t**2)*length*d1
        derivative = ((6*t*t-6*t)*y0+(-6*t*t+6*t)*y1)/length
        derivative += (3*t*t-4*t+1)*d0+(3*t*t-2*t)*d1
        segments.append(np.column_stack((knots[i,0]+length*t,y)))
        slopes.append(derivative)
    return np.vstack(segments),np.concatenate(slopes)


def preview_corridor(rows, points, active, track, near_range):
    """Evaluate farther continuations without feeding them into steering."""
    proposals = [[(chain,*attach_tail(row.edge,chain)) for chain in tail_choices(row,points)]
                 for row in rows]
    best = None
    for left,le,ls in proposals[0]:
        for right,re,rs in proposals[1]:
            start = max(0.,active.centers[-1,0]-.32,le[0,0],re[0,0])
            end = min(le[-1,0],re[-1,0])
            if (start > active.centers[-1,0] or end <= max(near_range,active.centers[-1,0]+.22)
                    or end-start < .35):
                continue
            if set(map(tuple,left)).intersection(map(tuple,right)):
                continue
            x = np.linspace(start,end,18)
            ly = np.interp(x,le[:,0],le[:,1])
            ry = np.interp(x,re[:,0],re[:,1])
            lh = np.arctan(np.interp(x,le[:,0],ls))
            rh = np.arctan(np.interp(x,re[:,0],rs))
            heading_error = np.abs(lh-rh)
            width = (ly-ry)*np.cos((lh+rh)/2)
            middle = (ly+ry)/2
            near = x <= active.centers[-1,0]
            if (np.abs(width-track).max() > .40 or abs(float(np.median(width))-track) > .28
                    or np.ptp(width) > .43 or heading_error.max() > math.radians(43)
                    or heading_error.mean() > math.radians(23)
                    or np.abs(middle[near]-np.interp(x[near],*active.centers.T)).max() > .20):
                continue
            quality = end-.65*np.abs(width-track).mean()-.5*heading_error.mean()
            if best is None or quality > best[0]:
                best = (quality,dict(centers=np.column_stack((x,middle)).tolist(),
                                    left_edge=le.tolist(),right_edge=re.tolist(),
                                    left_points=np.asarray(left).tolist(),
                                    right_points=np.asarray(right).tolist(),
                                    supported_x_m=float(end),width_m=float(np.median(width)),
                                    heading_difference_deg=math.degrees(float(heading_error.mean()))))
    return None if best is None else best[1]


class ProgressivePlanner(PairPlanner):
    """Let farther data become eligible only after fresh near re-observation."""

    def __init__(self, car, settings):
        super().__init__(car,settings)
        self.activation = float(settings.get('pair_activation_m',2.0))
        self.preview_range = float(settings.get('pair_preview_range_m',4.8))
        frames = settings.get('pair_confirm_frames',3)
        if (not 1.5 <= self.activation <= self.near_range
                or not self.activation+.4 <= self.preview_range <= 5.0
                or frames not in (2,3,4,5)):
            raise ValueError('Invalid progressive pairing settings')
        self.confirm_frames = int(frames)
        self.history = ConeHistory(self.activation)
        self.preview = None
        self.last_stamp = None
        self.revalidation = {}
        self.pending = set()
        self.observed = {}
        self.corner = CornerTransition(car,settings)

    def plan(self, cones, speed, now, reference=None, raw_points=None, pose=None, stamp=None):
        normal_reference = reference if self.corner.active is None else None
        normal = self._plan_near(cones,speed,now,normal_reference,raw_points,pose,stamp)
        if self.reject == 'duplicate_scan':
            return None
        points = np.asarray(cones,dtype=float).reshape(-1,2)
        points = points[np.isfinite(points).all(axis=1)]
        turn = self.corner.update(normal,points,raw_points,pose,now,self.activation,speed)
        if self.corner.event is not None:
            self.preview = None
        if turn is not None:
            self.last_pair = dict(width=turn.width,width_min=turn.width,at_car=0.,
                                  supported_ahead_m=self.corner.event['remaining_m'])
            self.reject = None
            return turn
        if self.corner.active is not None or self.corner.fault is not None:
            self.reject = self.corner.event['state']
            self.last_pair = None
            return None
        if normal is not None:
            for side,selected in (('left',normal.left),('right',normal.right)):
                for point in selected:
                    track = self.observed[tuple(point)]
                    if track.pending_far:
                        self.revalidation['promoted_'+side].append(point.tolist())
                        track.pending_far = False
        return normal

    def _plan_near(self, cones, speed, now, reference, raw_points, pose, stamp):
        self.preview = self.last_pair = self.last_rows = None
        self.last_row_stats = {}
        self.revalidation = {}
        if pose is None:
            self.history.reset()
            self.last_stamp = None
            self.reject = 'pose_unavailable'
            return None
        if stamp and any(stamp) and stamp == self.last_stamp and self.history.segment == pose['segment']:
            self.reject = 'duplicate_scan'
            return None
        self.last_stamp = stamp
        points = np.asarray(cones,dtype=float).reshape(-1,2)
        x,y = points.T
        points = points[np.isfinite(points).all(axis=1) & (x >= -.75) & (x <= self.preview_range)
                        & (np.abs(y) <= self.track+.9)]
        tracks = self.history.update(points,pose,now)
        self.observed = {tuple(point): track for point,track in zip(points,tracks)}
        eligible = np.array([p.near_hits >= self.confirm_frames for p in tracks],dtype=bool)
        self.pending = {tuple(point) for point,track in zip(points,tracks)
                        if track.pending_far and track.near_hits >= self.confirm_frames}
        self.revalidation = dict(near_confirmed=int(np.count_nonzero(eligible)),
                                 near_waiting=int(sum(0 < p.near_hits < self.confirm_frames for p in tracks)),
                                 far_visible=int(sum(p.near_hits == 0 for p in tracks)),
                                 promoted_left=[],promoted_right=[])
        trusted = points[eligible]
        if len(trusted) < 4:
            self.reject = 'waiting_near_confirmation'
            return None
        corner = self.corner.active
        if corner is not None and corner.sign*wrap(pose['yaw_rad']-corner.yaw) < math.radians(60):
            self.reject = 'corner_in_progress'
            return None
        seed = startup_axis(trusted,self.activation) if reference is None else None
        route = super().plan(trusted,speed,now,reference,raw_points,side_reference=seed)
        if route is None:
            return None
        if corner is None:
            self.preview = preview_corridor(self.last_rows,points,route,self.track,self.activation)
        return route

    def _route(self, pair, left, right, speed, reference, raw_points):
        route = super()._route(pair,left,right,speed,reference,raw_points)
        if route is None or reference is None:
            return route
        if not self.pending.intersection(map(tuple,np.vstack((left.points,right.points)))):
            return route
        x = route.centers[:,0]
        common = (x >= reference[0,0]) & (x <= reference[-1,0]) & (x <= 1.40)
        # A verified continuation can begin bending near the end of a short
        # incumbent fit. Keep the strict join check on its supported interior.
        if x[-1] >= reference[-1,0] + .12:
            interior = common & (x <= reference[-1,0] - .18)
            if np.count_nonzero(interior) >= 3:
                common = interior
        if np.count_nonzero(common) >= 3:
            overlap = route.centers[common]
            lateral,tangent = relative_to_path(overlap,reference)
            delta = np.diff(overlap,axis=0)
            angle = np.arctan2(delta[:,1],delta[:,0])
            old_angle = np.arctan2(tangent[:-1,1],tangent[:-1,0])
            if np.abs(lateral).max() <= .22 and np.abs(angle-old_angle).max() <= math.radians(30):
                return route
        self.reject = 'handoff_discontinuity'
        return None
