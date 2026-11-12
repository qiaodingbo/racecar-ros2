"""Identify a measured perpendicular exit and track a footprint-checked turn."""
from dataclasses import dataclass
import math

import numpy as np

from dense_reference import DenseRoute
from geometry import wrap


def rotation(yaw):
    return np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])


def pose_xy(pose):
    return np.array([pose['x_m'], pose['y_m']])


@dataclass
class Corner:
    origin: np.ndarray
    yaw: float
    sign: int
    inner: float
    outer: float
    entry_low: float
    entry_high: float
    exit_end: float
    path: np.ndarray
    headings: np.ndarray
    curvature: np.ndarray
    radius: float
    left: np.ndarray
    right: np.ndarray
    blocked: bool = False

    @property
    def key(self):
        point = np.array([(self.inner+self.outer)/2,0.])
        return point @ rotation(self.yaw).T + self.origin

    def world(self, path):
        return (path*np.array([1.,self.sign])) @ rotation(self.yaw).T + self.origin

    def local(self, world):
        return ((world-self.origin) @ rotation(self.yaw))*[1.,self.sign]

    def contains(self, xy):
        x,y = np.asarray(xy).T
        entry = (y >= self.entry_low) & (y <= self.entry_high) & (x <= self.outer)
        exit_lane = (x >= self.inner) & (x <= self.outer) & (y >= self.entry_low)
        return entry | exit_lane

    def envelope_ok(self, corners):
        if not np.all(self.contains(corners.reshape(-1,2))):
            return False
        # The inside of an L is concave. A body edge can clip the inside
        # corner even when all four vertices are within one of the lanes.
        next_corner = np.roll(corners,-1,axis=1)
        x0,y0 = corners[:,:,0],corners[:,:,1]
        x1,y1 = next_corner[:,:,0],next_corner[:,:,1]
        cross = (x0 < self.inner) != (x1 < self.inner)
        if np.any(cross):
            y_cross = y0[cross]+(self.inner-x0[cross])*(y1[cross]-y0[cross])/(x1[cross]-x0[cross])
            return bool(np.all(y_cross <= self.entry_high))
        return True


def footprint(xy, angle, car):
    bounds = np.array([[-car.rear-car.margin,-car.half_width-car.margin],
                       [car.front+car.margin,-car.half_width-car.margin],
                       [car.front+car.margin,car.half_width+car.margin],
                       [-car.rear-car.margin,car.half_width+car.margin]])
    c,s = np.cos(angle),np.sin(angle)
    x = xy[:,0,None]+c[:,None]*bounds[:,0]-s[:,None]*bounds[:,1]
    y = xy[:,1,None]+s[:,None]*bounds[:,0]+c[:,None]*bounds[:,1]
    return np.stack((x,y),axis=-1)


def exit_rows(points, anchor, width):
    x,y = points.T
    elevated = (y >= anchor[1]+.16) & (y <= anchor[1]+1.55)
    inner = np.vstack((anchor,points[elevated & (np.abs(x-anchor[0]) <= .19)]))
    inner_span = np.ptp(inner[:,1])
    if inner_span < .35 or np.ptp(inner[:,0]) > .32*inner_span:
        return None
    for seed in points[elevated & (np.abs(x-anchor[0]-width) <= .26)]:
        outer = points[elevated & (np.abs(x-seed[0]) <= .17)]
        outer_span = np.ptp(outer[:,1])
        if outer_span < .35 or np.ptp(outer[:,0]) > .32*outer_span:
            continue
        inner_x = float(np.median(inner[:,0]))
        outer_x = float(np.median(outer[:,0]))
        common = min(inner[:,1].max(),outer[:,1].max())-max(inner[:,1].min(),outer[:,1].min())
        if abs(outer_x-inner_x-width) <= .24 and common >= .14:
            return inner,outer
    return None


def detect_corner(route, points, pose, car, full_steer, max_steer, raw_points, activation):
    if route is None or pose is None or len(route.centers) < 4:
        return None
    delta = route.centers[-1]-route.centers[0]
    entry_yaw = math.atan2(delta[1],delta[0])
    if abs(entry_yaw) > math.radians(24) or np.max(np.abs(route.curvature)) > .32:
        return None
    y0 = float(route.centers[0,1]-route.centers[0,0]*math.tan(entry_yaw))
    rot = rotation(entry_yaw)
    offset = np.array([0.,y0])
    q = (points-offset) @ rot
    origin = offset @ rotation(pose['yaw_rad']).T + pose_xy(pose)
    radius = car.radius*full_steer/max_steer + .10
    blocked = None
    for sign, inner_row, outer_row in ((1,route.left,route.right),(-1,route.right,route.left)):
        signed = np.array([1.,sign])
        candidates = q*signed
        inside = ((inner_row-offset)@rot)*signed
        outside = ((outer_row-offset)@rot)*signed
        low,high = float(np.median(outside[:,1])),float(np.median(inside[:,1]))
        for anchor in inside:
            if not .36 <= anchor[0] <= activation or abs(anchor[1]-high) > .20:
                continue
            rows = exit_rows(candidates,anchor,route.width)
            if rows is None:
                continue
            inner,outer = rows
            ix,ox = float(np.max(inner[:,0])),float(np.min(outer[:,0]))
            if not np.any((np.abs(candidates[:,0]-ox) <= .26) & (np.abs(candidates[:,1]-low) <= .24)):
                continue
            center = (ix+ox)/2
            start = center-radius
            if start < -.18 or start > .95:
                continue
            angle = np.linspace(0.,math.pi/2,65)
            arc = np.column_stack((start+radius*np.sin(angle),radius*(1-np.cos(angle))))
            entry_x = np.linspace(min(-.50,start-.18),start,12)[:-1]
            exit_end = min(float(inner[:,1].max()),float(outer[:,1].max()))
            # A short exit tangent gives a lookahead while the outgoing pair
            # becomes observable in the car's new forward frame.
            exit_y = np.linspace(radius,max(radius+.48,exit_end+.42),16)[1:]
            path = np.vstack((np.column_stack((entry_x,np.zeros(len(entry_x)))),
                              arc,np.column_stack((np.full(len(exit_y),center),exit_y))))
            heading = np.r_[np.zeros(len(entry_x)),angle,np.full(len(exit_y),math.pi/2)]
            curvature = sign*np.r_[np.zeros(len(entry_x)),np.full(len(arc),1/radius),np.zeros(len(exit_y))]
            corner = Corner(origin,pose['yaw_rad']+entry_yaw,sign,ix,ox,low,high,
                            exit_end,path,heading,curvature,radius,
                            np.vstack((inside,inner[1:])),np.vstack((outside,outer)))
            envelopes = footprint(path,heading,car)
            if not corner.envelope_ok(envelopes):
                corner.blocked = True
                blocked = corner
                continue
            if raw_points is not None and len(raw_points):
                raw = ((raw_points-offset) @ rot)*signed
                # Exact swept rectangular footprint against all nearby returns.
                delta = raw[None,:,:]-path[:,None,:]
                c,s = np.cos(heading)[:,None],np.sin(heading)[:,None]
                along = delta[:,:,0]*c+delta[:,:,1]*s
                across = -delta[:,:,0]*s+delta[:,:,1]*c
                if np.any((along >= -car.rear-car.margin) & (along <= car.front+car.margin)
                          & (np.abs(across) <= car.half_width+car.margin)):
                    corner.blocked = True
                    blocked = corner
                    continue
            return corner
    return blocked


class CornerTransition:
    def __init__(self, car, settings):
        self.car = car
        self.full = float(settings.get('full_steer_deg',65.))
        self.limit = float(settings.get('max_steer_deg',16.))
        self.speed = float(settings.get('corner_speed_mps',.35))
        if not .15 <= self.speed <= 2.00:
            raise ValueError('corner_speed_mps must be 0.15..2.00')
        self.candidate = self.active = self.pose = None
        self.votes = 0
        self.seen = None
        self.finished = None
        self.segment = None
        self.started = None
        self.event = None
        self.fault = None

    def update(self, route, points, raw, pose, now, near_range, speed=0.):
        self.event = None
        if self.fault is not None:
            self.event = {'state':self.fault}
            return None
        if pose is None or (self.active is not None and pose['segment'] != self.segment):
            # A pose gap is an infrastructure hiccup (sensor stall), not a
            # verdict about the corner. Latching it made every later plan()
            # return early: 140 of 145 frames on the 23:46 run.
            if self.active is not None:
                self.event = {'state':'corner_pose_lost'}
            self.active = self.candidate = None
            self.votes = 0
            return None
        self.pose = pose
        if self.active is None:
            candidate = detect_corner(route,points,pose,self.car,self.full,self.limit,raw,near_range)
            if candidate is None or (self.finished is not None
                                      and np.linalg.norm(candidate.key-self.finished) < .9):
                self.votes = 0
                self.candidate = None
                return None
            # Fixed tolerances were tuned at ~0.5 m/s. At 1.34 m/s the car
            # moves 0.137 m per scan, so the same corner key drifts past
            # 0.16 m and the vote resets every frame. Scale with speed.
            key_tol = .16 + .12*abs(speed)
            yaw_tol = math.radians(9 + 6*abs(speed))
            previous_blocked = self.candidate is None or self.candidate.blocked
            if (self.candidate is not None and now-self.seen <= .25
                    and candidate.sign == self.candidate.sign
                    and np.linalg.norm(candidate.key-self.candidate.key) < key_tol
                    and abs(wrap(candidate.yaw-self.candidate.yaw)) < yaw_tol):
                self.votes += 1
            else:
                self.votes = 1
            self.candidate,self.seen = candidate,now
            self.event = dict(state='confirming',direction='left' if candidate.sign>0 else 'right',
                              votes=self.votes)
            # Each unblocked proposal already validates both rows and the
            # swept envelope. Two agreeing scans leave time to presteer.
            required_votes = 2 if not candidate.blocked and not previous_blocked else 3
            if self.votes < required_votes:
                return None
            if candidate.blocked:
                # Do not latch: one blocked frame used to set self.fault for
                # the rest of the run, so every later plan() returned early
                # and the car could never recover. Reject this frame, drop the
                # candidate and re-confirm from scratch next frame.
                self.event = dict(state='corner_blocked',
                                  direction='left' if candidate.sign > 0 else 'right',
                                  votes=self.votes)
                self.candidate = None
                self.votes = 0
                return None
            self.active,self.segment,self.started = candidate,pose['segment'],now

        corner = self.active
        position = corner.local(pose_xy(pose)[None,:])[0]
        yaw = corner.sign*wrap(pose['yaw_rad']-corner.yaw)
        x_exit = (corner.inner+corner.outer)/2
        if (math.radians(73) <= yaw <= math.radians(105)
                and position[1] >= corner.radius-.18 and route is not None):
            world = route.centers @ rotation(pose['yaw_rad']).T + pose_xy(pose)
            exit_path = corner.local(world)
            direction = exit_path[-1]-exit_path[0]
            if (direction[1] > .4 and abs(math.atan2(direction[0],direction[1])) < math.radians(22)
                    and np.max(np.abs(exit_path[:,0]-x_exit)) <= .25):
                self.finished = corner.key
                self.active = self.candidate = None
                self.votes = 0
                self.event = dict(state='exit_reacquired',yaw_deg=math.degrees(yaw))
                return None
        path = corner.world(corner.path)
        local = (path-pose_xy(pose)) @ rotation(pose['yaw_rad'])
        ds = np.linalg.norm(np.diff(local,axis=0),axis=1)
        station = np.r_[0.,np.cumsum(ds)]
        nearest = int(np.argmin(np.linalg.norm(local,axis=1)))
        support = float(station[-1]-station[nearest])
        self.event = dict(state='turning',direction='left' if corner.sign>0 else 'right',
                          radius_m=corner.radius,yaw_deg=math.degrees(yaw),remaining_m=support)
        if position[1] > corner.exit_end+.18 or support < .29:
            self.fault = self.event['state'] = 'corner_exit_unconfirmed'
            return None
        if now-self.started > 15. or yaw > math.radians(108) or np.linalg.norm(local[nearest]) > .34:
            # Abort this attempt and fall back to bilateral planning instead of
            # ending the run: measured deviation varies 0.10-0.32 m between runs
            # at the same speed, so one bad attempt must not be fatal.
            self.event = {'state':'corner_tracking_lost'}
            self.active = self.candidate = None
            self.votes = 0
            return None
        aim = min(station[-1],station[nearest]+.48)
        target = np.array([np.interp(aim,station,local[:,i]) for i in (0,1)])
        arc_start = int(np.flatnonzero(np.abs(corner.curvature)>0.)[0])
        before = corner.path[arc_start,0]-position[0]
        if before > 0.:
            lead = .14+.30*abs(speed)
            blend = np.clip((lead+.06-before)/.17,0.,1.)
            blend = blend*blend*(3.-2.*blend)
            straight = corner.world(np.array([position[0]+.64,0.]))
            straight = (straight-pose_xy(pose)) @ rotation(pose['yaw_rad'])
            target = blend*target+(1.-blend)*straight
        target = tuple(float(value) for value in target)
        heading = wrap(corner.yaw+corner.sign*np.interp(aim,station,corner.headings)-pose['yaw_rad'])
        if target[0] <= .12:
            self.fault = self.event['state'] = 'corner_target_behind'
            return None
        left,right = [((corner.world(row)-pose_xy(pose)) @ rotation(pose['yaw_rad']))
                      for row in (corner.left,corner.right)]
        if corner.sign<0:
            left,right = right,left
        width = corner.outer-corner.inner
        return DenseRoute(local,left,right,local,0,width,corner.curvature,.48,target,heading,False)

    def contains_arc(self, curvature, horizon):
        corner = self.active
        pose = self.pose
        distance = np.linspace(0.,horizon,14)
        yaw = curvature*distance
        xy = np.column_stack((distance*np.sinc(yaw/np.pi),
                              .5*distance*yaw*np.sinc(yaw/(2*np.pi))**2))
        envelope = footprint(xy,yaw,self.car)
        world = envelope @ rotation(pose['yaw_rad']).T + pose_xy(pose)
        return corner.envelope_ok(corner.local(world))
