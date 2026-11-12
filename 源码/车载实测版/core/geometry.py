"""Shared LiDAR geometry and path clearance for the dense S-bend module."""
from math import atan2, ceil, cos, hypot, pi, sin
import numpy as np

def wrap(angle):
    return atan2(sin(angle), cos(angle))


def scan_geometry(ranges, angle_min, increment, minimum, maximum, laser_pose,
                  mask_observed_body=False):
    """Return narrow clusters for tracking and raw returns for clearance."""
    ranges = np.asarray(ranges, dtype=float)
    if ranges.size < 90 or not np.isfinite(increment) or increment <= 0:
        return np.empty((0, 2)), np.empty((0, 2))
    x, y, yaw = laser_pose
    angle = angle_min + np.arange(ranges.size) * increment + yaw
    indices = np.flatnonzero(np.isfinite(ranges) & (ranges >= max(0.12, minimum))
                             & (ranges <= min(5.0, maximum)))
    r = ranges[indices]
    beam_angles = angle[indices]
    dx, dy = r * np.cos(beam_angles), r * np.sin(beam_angles)
    if mask_observed_body:
        # Any return inside the car rectangle (relative to the laser) is a
        # self-return. The old mask only covered a small rear patch, so a
        # return from the front/side wheel stayed in the clearance map and
        # latched WAIT_ROUTE forever. Body spans x=-0.39..+0.17, y=+-0.18
        # in the laser frame.
        self_echo = ((dx >= -0.39) & (dx <= 0.17) & (np.abs(dy) <= 0.19))
        # The right rear wheel produces a narrow continuation of the body
        # return when steered. Mask it only when adjacent beams also see the
        # calibrated body; isolated returns remain in obstacle clearance.
        wheel = np.flatnonzero((dx >= -.305) & (dx <= -.175)
                               & (dy >= -.248) & (dy < -.19))
        body = np.flatnonzero(self_echo)
        if len(wheel) and len(body):
            rank = np.searchsorted(body, wheel)
            neighbors = body[np.clip(np.column_stack((rank-1,rank)),0,len(body)-1)]
            adjacent = np.abs(indices[wheel,None]-indices[neighbors]) <= 7
            close = np.hypot(dx[wheel,None]-dx[neighbors],dy[wheel,None]-dy[neighbors]) < .12
            self_echo[wheel] = np.any(adjacent & close,axis=1)
        indices, dx, dy = indices[~self_echo], dx[~self_echo], dy[~self_echo]
    points = np.column_stack((x + dx, y + dy))
    if len(points) < 2:
        return np.empty((0, 2)), points
    breaks = (np.diff(indices) > 3) | (np.linalg.norm(np.diff(points, axis=0), axis=1) > 0.16)
    groups = np.split(points, np.flatnonzero(breaks) + 1)
    full_circle = ranges.size * increment > 2*pi - 3*increment
    seam_gap = ranges.size - indices[-1] + indices[0]
    if len(groups) > 1 and full_circle and seam_gap <= 3 and np.linalg.norm(points[-1] - points[0]) <= 0.16:
        groups[0] = np.concatenate((groups.pop(), groups[0]))
    cones = [g.mean(axis=0) for g in groups
             if len(g) >= 2 and hypot(*np.ptp(g, axis=0)) <= 0.36]
    # A lone beam (multipath off the body, wheel edge) that lands inside the
    # swept rectangle pins swept_clearance() at 0.00 m for every curvature,
    # so the car sits neutral with no timeout. A real obstacle at close range
    # always returns adjacent beams, so the clearance map keeps only groups
    # of two or more -- the same adjacent-beam rule forward_path_points uses.
    multi = [g for g in groups if len(g) >= 2]
    strong = np.vstack(multi) if multi else np.empty((0, 2))
    return np.asarray(cones).reshape(-1, 2), strong


def relative_to_path(points, path):
    """Signed normal distance and tangent at the closest polyline segment."""
    if path is None or len(path) < 2:
        path = np.array([[0.0, 0.0], [4.0, 0.0]])
    segments = np.diff(path, axis=0)
    length_squared = np.sum(segments*segments, axis=1)
    valid = length_squared > 1e-8
    if not np.any(valid):
        return points[:, 1].copy(), np.tile([1.0, 0.0], (len(points), 1))
    starts, segments, length_squared = path[:-1][valid], segments[valid], length_squared[valid]
    delta = points[:, None, :] - starts[None, :, :]
    fraction = np.clip(np.sum(delta*segments, axis=2)/length_squared, 0.0, 1.0)
    offset = delta - fraction[:, :, None]*segments
    index = np.argmin(np.sum(offset*offset, axis=2), axis=1)
    tangent = segments[index]/np.sqrt(length_squared[index, None])
    nearest = offset[np.arange(len(points)), index]
    lateral = tangent[:, 0]*nearest[:, 1] - tangent[:, 1]*nearest[:, 0]
    return lateral, tangent


class ObstacleHold:
    """Brake immediately, but confirm on distinct lidar frames before latching."""
    def __init__(self):
        self.last_scan = None
        self.blocked_frames = 0
        self.clear_frames = 0
        self.holding_since = None

    def update(self, scan_at, now, blocked):
        if blocked and self.holding_since is None:
            self.holding_since = now
            self.blocked_frames = 1
            self.clear_frames = 0
            self.last_scan = scan_at
        elif scan_at != self.last_scan:
            self.last_scan = scan_at
            if blocked:
                self.blocked_frames += 1
                self.clear_frames = 0
            else:
                self.blocked_frames = 0
                self.clear_frames = min(3, self.clear_frames + 1)
                if self.clear_frames >= 3:
                    self.holding_since = None
        if self.holding_since is None:
            return 'clear'
        if self.blocked_frames >= 2 or now - self.holding_since >= 0.9:
            return 'fault'
        return 'hold'


def forward_path_points(points, servo_offset, cfg):
    """Reuse the hairpin's steering-led forward window and adjacent-beam rule."""
    relative = points - np.array([cfg.laser_x, 0.0])
    angles = np.arctan2(relative[:, 1], relative[:, 0])
    distance = np.linalg.norm(relative, axis=1)
    turn = servo_offset * 0.5 * pi/180
    window = 22*pi/180
    hits = np.flatnonzero((distance >= 0.15)
                         & (angles >= turn - window) & (angles <= turn + window)
                         & (np.abs(relative[:, 1]) < 0.28))
    hits = hits[np.argsort(angles[hits])]
    groups = np.split(hits, np.flatnonzero(np.diff(angles[hits]) >= 3*pi/180) + 1)
    confirmed = [group for group in groups if len(group) >= 3]
    indices = np.concatenate(confirmed) if confirmed else np.empty(0, dtype=int)
    return points[indices]


def swept_clearance(points, curvature, cfg, horizon=1.1, rear_wheel_padding=False,
                    fine_resolution=False):
    """Check the rectangular footprint against raw returns along the arc."""
    if len(points) == 0:
        return horizon
    margin = cfg.margin
    if rear_wheel_padding:
        # Returns at the measured rear wheel edge can overlap the padded
        # footprint at t=0. Still test them against the physical rectangle
        # with a guard band at every station, including the rear swing.
        wheel_edge = ((points[:, 0] >= -cfg.rear) & (points[:, 0] <= -.075)
                      & (points[:, 1] < -.195) & (points[:, 1] >= -.248))
        margin = np.where(wheel_edge, .018, cfg.margin)
    if fine_resolution and abs(curvature) < 1e-7:
        ahead = ((points[:, 0] >= -cfg.rear-margin)
                 & (np.abs(points[:, 1]) <= cfg.half_width+margin))
        contact = points[:, 0]-cfg.front-margin
        return max(0., min(horizon, float(np.min(contact[ahead])))) if np.any(ahead) else horizon
    count = max(26, ceil(horizon/.025)+1) if fine_resolution else 26
    travel = np.linspace(0.0, horizon, count)
    angle = travel * curvature
    x = travel if abs(curvature) < 1e-7 else np.sin(angle) / curvature
    y = np.zeros_like(travel) if abs(curvature) < 1e-7 else (1.0 - np.cos(angle)) / curvature
    dx = points[None, :, 0] - x[:, None]
    dy = points[None, :, 1] - y[:, None]
    longitudinal = dx*np.cos(angle[:, None]) + dy*np.sin(angle[:, None])
    lateral = -dx*np.sin(angle[:, None]) + dy*np.cos(angle[:, None])
    hit = np.any((longitudinal >= -cfg.rear - margin)
                 & (longitudinal <= cfg.front + margin)
                 & (np.abs(lateral) <= cfg.half_width + margin), axis=1)
    blocked = np.flatnonzero(hit)
    return float(travel[max(0, blocked[0] - 1)]) if len(blocked) else horizon


