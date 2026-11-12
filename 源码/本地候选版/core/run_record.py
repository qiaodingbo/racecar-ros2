"""Full-run LiDAR and wheel/IMU dead-reckoning journal."""
import json
import math
import tempfile
import time

from geometry import wrap


def stamp_ns(msg):
    stamp = msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class PoseTracker:
    """Planar pose in the first valid axle frame, with explicit gaps."""

    def __init__(self):
        self.x = self.y = 0.
        self.yaw_zero = None
        self.imu = None
        self.encoder = None
        self.segment = 0
        self.gap = False

    def update_imu(self, now, stamp, yaw):
        self.imu = (now, stamp, yaw)

    def update_encoder(self, now, stamp, speed):
        if self.imu is None or now - self.imu[0] > .18:
            self.invalidate()
            return
        imu_at, imu_stamp, yaw = self.imu
        if self.yaw_zero is None:
            self.yaw_zero = yaw
        yaw = wrap(yaw - self.yaw_zero)
        if self.encoder is not None:
            old_at, old_stamp, old_speed, old_yaw, _, _ = self.encoder
            dt = now - old_at
            if stamp > 0 and old_stamp > 0:
                dt = (stamp - old_stamp) * 1e-9
            if 0 < dt <= .20 and now - old_at <= .25:
                middle = old_yaw + .5 * wrap(yaw - old_yaw)
                travel = .5 * (old_speed + speed) * dt
                self.x += travel * math.cos(middle)
                self.y += travel * math.sin(middle)
            else:
                self.gap = True
        if self.gap:
            self.segment += 1
            self.gap = False
        self.encoder = (now, stamp, speed, yaw, imu_at, imu_stamp)

    def invalidate(self):
        if self.encoder is not None:
            self.gap = True
            self.encoder = None

    def snapshot(self, now):
        if self.encoder is None or self.imu is None:
            return None
        enc_at, enc_stamp, speed, old_yaw, _, _ = self.encoder
        imu_at, imu_stamp, imu_yaw = self.imu
        if now - enc_at > .12 or now - imu_at > .18:
            return None
        yaw = wrap(imu_yaw - self.yaw_zero)
        dt = now - enc_at
        middle = old_yaw + .5 * wrap(yaw - old_yaw)
        return dict(x_m=self.x + speed * dt * math.cos(middle),
                    y_m=self.y + speed * dt * math.sin(middle),
                    yaw_rad=yaw, segment=self.segment, encoder_stamp_ns=enc_stamp,
                    imu_stamp_ns=imu_stamp, encoder_age_s=dt, imu_age_s=now-imu_at)


class RunRecorder:
    def __init__(self, folder, car, settings, observing, revision):
        folder.mkdir(parents=True, exist_ok=True)
        mode = 'observe' if observing else 'drive'
        prefix = time.strftime('%Y%m%d_%H%M%S') + '_' + mode + '_'
        self.file = tempfile.NamedTemporaryFile('w', dir=folder, prefix=prefix,
                                                suffix='.jsonl', encoding='utf-8', delete=False)
        self.path = self.file.name
        self.flushed_at = time.monotonic()
        self.append(dict(kind='meta', schema=1, mode=mode, settings=settings, revision=revision,
                         pose_frame='initial_rear_axle', pose_source='wheel_velocity_imu_yaw',
                         pose_time_basis='monotonic_receive',
                         lidar_in_axle=[car.laser_x, 0., 0.],
                         wall_time_ns=time.time_ns(), monotonic_s=time.monotonic()))

    def append(self, record):
        self.file.write(json.dumps(record, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n')
        now = time.monotonic()
        if now - self.flushed_at >= .75:
            self.file.flush()
            self.flushed_at = now

    def close(self):
        self.file.close()
