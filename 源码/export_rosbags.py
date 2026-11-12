import argparse
from collections import Counter
from contextlib import closing
import csv
from hashlib import sha256
import json
from math import cos, sin
from pathlib import Path, PurePosixPath
import shutil
import sqlite3
import tarfile
import zipfile

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from ruamel.yaml import YAML


NANOSECONDS = 1_000_000_000
TOPICS = {
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/replay/odom': 'nav_msgs/msg/Odometry',
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/replay/control_pwm': 'geometry_msgs/msg/Twist',
    '/replay/scan_info': 'std_msgs/msg/String',
    '/replay/plan': 'std_msgs/msg/String',
    '/replay/events': 'std_msgs/msg/String',
}
RUN_PREFIX = 'tools/s_bend/records/runs/'
LATEST = '20261112_212001_drive_op9zcbzp'
STORE = get_typestore(Stores.ROS2_HUMBLE)
TYPES = STORE.types
ZERO = TYPES['geometry_msgs/msg/Vector3'](0., 0., 0.)
EMPTY_INTENSITIES = np.array([], dtype=np.float32)
UNRECORDED_COVARIANCE = np.zeros(36, dtype=np.float64)


def header_ns(event):
    sec, nanos = event.get('stamp', (0, 0))
    return sec*NANOSECONDS + nanos


def header(timestamp, frame):
    seconds, nanos = divmod(timestamp, NANOSECONDS)
    return TYPES['std_msgs/msg/Header'](TYPES['builtin_interfaces/msg/Time'](seconds, nanos), frame)


def json_message(event):
    return TYPES['std_msgs/msg/String'](json.dumps(event, ensure_ascii=False, separators=(',', ':')))


def rotation(yaw):
    return TYPES['geometry_msgs/msg/Quaternion'](0., 0., sin(yaw/2), cos(yaw/2))


def twist(x, yaw):
    vector = TYPES['geometry_msgs/msg/Vector3']
    return TYPES['geometry_msgs/msg/Twist'](vector(x, 0., 0.), vector(0., 0., yaw))


def tf_message(stamp, root_frame, scan_frame, x, y, yaw, extrinsic):
    transform = TYPES['geometry_msgs/msg/Transform']
    stamped = TYPES['geometry_msgs/msg/TransformStamped']
    vector = TYPES['geometry_msgs/msg/Vector3']
    dx, dy, dyaw = extrinsic
    base = stamped(header(stamp, root_frame), 'rear_axle', transform(vector(x, y, 0.), rotation(yaw)))
    laser = stamped(header(stamp, 'rear_axle'), scan_frame, transform(vector(dx, dy, 0.), rotation(dyaw)))
    return TYPES['tf2_msgs/msg/TFMessage']([base, laser])


def scan_messages(event, meta, timestamp):
    stamp = header_ns(event) or timestamp
    frame = event.get('frame_id') or 'laser_link'
    ranges = np.asarray(event['ranges'], dtype=np.float32)
    pose = event.get('pose')
    if pose is not None:
        root = meta.get('pose_frame', 'initial_rear_axle')
        x, y, yaw = (pose[k] for k in ('x_m', 'y_m', 'yaw_rad'))
        if 'lidar_in_axle' in meta:
            yield '/tf', tf_message(stamp, root, frame, x, y, yaw, meta['lidar_in_axle'])
        odom_pose = TYPES['geometry_msgs/msg/Pose'](TYPES['geometry_msgs/msg/Point'](x, y, 0.), rotation(yaw))
        pose_cov = TYPES['geometry_msgs/msg/PoseWithCovariance'](odom_pose, UNRECORDED_COVARIANCE)
        velocity = twist(event.get('speed_mps', 0.), 0.)
        twist_cov = TYPES['geometry_msgs/msg/TwistWithCovariance'](velocity, UNRECORDED_COVARIANCE)
        yield '/replay/odom', TYPES['nav_msgs/msg/Odometry'](header(stamp, root), 'rear_axle', pose_cov, twist_cov)
    angle_min, increment = event['angle_min'], event['angle_increment']
    yield '/scan', TYPES[TOPICS['/scan']](
        header(stamp, frame), angle_min, angle_min+max(0, len(ranges)-1)*increment, increment,
        event.get('time_increment_s', 0.), event.get('scan_time_s', 0.), event['range_min'],
        event['range_max'], ranges, EMPTY_INTENSITIES)
    yield '/replay/scan_info', json_message({k: v for k, v in event.items() if k != 'ranges'})


def read_run(raw, suffix):
    if suffix == '.jsonl':
        events = [json.loads(line) for line in raw.splitlines() if line.strip()]
        meta = events[0]
        return meta, events, meta['wall_time_ns'], meta['monotonic_s']
    legacy = json.loads(raw)
    frames = legacy['frames']
    stamped = next(frame for frame in frames if header_ns(frame)>0)
    meta = {k: v for k, v in legacy.items() if k not in ('frames','events')}
    meta.update(kind='meta', source_format='legacy_json', at=min(frame['at'] for frame in frames))
    events = [meta, *legacy['events'], *(dict(frame, kind='scan', scan_id=i) for i, frame in enumerate(frames, 1))]
    return meta, events, header_ns(stamped), stamped['at']


def messages(events, meta, epoch_ns, epoch_monotonic):
    for event in events:
        timestamp = epoch_ns + round((event.get('at', epoch_monotonic)-epoch_monotonic)*NANOSECONDS)
        kind = event['kind']
        if kind == 'scan':
            for topic, message in scan_messages(event, meta, timestamp):
                yield topic, timestamp, message
            continue
        if kind == 'command':
            yield '/replay/control_pwm', timestamp, twist(event['pwm'], event['servo'])
        topic = '/replay/plan' if kind == 'dense_plan' else '/replay/events'
        yield topic, timestamp, json_message(event)


SCHEMA = '''
CREATE TABLE schema (schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL);
CREATE TABLE metadata (id INTEGER PRIMARY KEY, metadata_version INTEGER NOT NULL, metadata TEXT NOT NULL);
CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL,
                     serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);
CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL,
                       data BLOB NOT NULL);
INSERT INTO schema VALUES (3, 'humble');
'''


def write_bag(folder, meta, events, epoch_ns, epoch_monotonic):
    folder.mkdir(parents=True)
    db_path = folder/(folder.name+'_0.db3')
    counts = Counter()
    topic_ids = {}
    with closing(sqlite3.connect(db_path)) as db, db:
        db.executescript(SCHEMA)

        def rows():
            for topic, timestamp, message in messages(events, meta, epoch_ns, epoch_monotonic):
                if topic not in topic_ids:
                    topic_ids[topic] = db.execute(
                        'INSERT INTO topics(name,type,serialization_format,offered_qos_profiles) VALUES (?, ?, ?, ?)',
                        (topic, TOPICS[topic], 'cdr', '')).lastrowid
                counts[topic] += 1
                yield topic_ids[topic], timestamp, STORE.serialize_cdr(message, TOPICS[topic])

        db.executemany('INSERT INTO messages(topic_id,timestamp,data) VALUES (?, ?, ?)', rows())
        db.execute('CREATE INDEX timestamp_idx ON messages(timestamp ASC)')
        start, end = db.execute('SELECT MIN(timestamp), MAX(timestamp) FROM messages').fetchone()
        summary = dict(starting_time={'nanoseconds_since_epoch': start}, duration={'nanoseconds': end-start},
                       message_count=sum(counts.values()))
        info = dict(version=5, storage_identifier='sqlite3', **summary, topics_with_message_count=[
            dict(topic_metadata=dict(name=topic, type=TOPICS[topic], serialization_format='cdr',
                                     offered_qos_profiles=''), message_count=count)
            for topic, count in counts.items()], compression_format='', compression_mode='',
            relative_file_paths=[db_path.name], files=[dict(path=db_path.name, **summary)])
        db.execute('INSERT INTO metadata(metadata_version,metadata) VALUES (?, ?)', (5, json.dumps(info)))
        if db.execute('PRAGMA quick_check').fetchone() != ('ok',):
            raise RuntimeError(f'SQLite verification failed for {folder}')
    with (folder/'metadata.yaml').open('w', encoding='utf8') as out:
        YAML().dump({'rosbag2_bagfile_information': info}, out)
    return counts, info


def verify_bag(folder, events, expected):
    counts = Counter()
    scans = iter(sorted((event for event in events if event['kind']=='scan'), key=lambda event: event['at']))
    with Reader(folder) as bag:
        for connection, _, data in bag.messages():
            topic = connection.topic
            msg = STORE.deserialize_cdr(data, connection.msgtype)
            counts[topic] += 1
            if topic == '/scan':
                original = next(scans)
                if not np.array_equal(msg.ranges, np.asarray(original['ranges'],dtype=np.float32), equal_nan=True):
                    raise ValueError(f'Scan ranges differ in {folder}')
                actual = msg.header.stamp.sec*NANOSECONDS+msg.header.stamp.nanosec
                if header_ns(original) and actual != header_ns(original):
                    raise ValueError(f'Scan timestamp differs in {folder}')
    if counts != expected or next(scans, None) is not None:
        raise ValueError(f'Message counts differ in {folder}')


def extract_source(archive, offline_kit, desktop):
    root = desktop/'源码'
    files = []
    with tarfile.open(archive) as bag:
        for member in bag:
            path = PurePosixPath(member.name)
            if not member.isfile() or '..' in path.parts or path.is_absolute():
                continue
            if path.parts[:2] == ('racecar', 'src'):
                destination = root/'ROS工作区'/Path(*path.parts[1:])
            elif (path.parts[:2] == ('tools', 's_bend') and path.parts[2] != 'records'
                  and path.name != 'dense_last_run.json'):
                destination = root/'车载实测版'/Path(*path.parts[2:])
            else:
                continue
            destination.parent.mkdir(parents=True, exist_ok=True)
            with bag.extractfile(member) as incoming, destination.open('wb') as output:
                shutil.copyfileobj(incoming, output)
            files.append(destination)
    with zipfile.ZipFile(offline_kit) as bundle:
        for name in bundle.namelist():
            path = PurePosixPath(name)
            if path.parts[0] == 'candidate':
                destination = root/'本地候选版'/Path(*path.parts[1:])
            elif name in ('run_user_course.py','manifest.json'):
                destination = root/name
            else:
                continue
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(bundle.read(name))
            files.append(destination)
    manifest = json.loads((root/'manifest.json').read_text(encoding='utf8'))
    for path, expected in manifest['vehicle_revisions_from_journal'].items():
        if sha256((root/'车载实测版'/path).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Car source hash differs: {path}')
    for path, expected in manifest['candidate_changed_files'].items():
        if sha256((root/'本地候选版'/path).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Candidate source hash differs: {path}')
    shutil.copy2(__file__, root/'export_rosbags.py')
    print(f'Source files copied and checked: {len(files)}', flush=True)


def convert(archive, desktop):
    results = []
    with tarfile.open(archive) as source:
        for member in source:
            if not member.isfile() or not member.name.startswith(RUN_PREFIX):
                continue
            path = PurePosixPath(member.name)
            if path.suffix not in ('.jsonl','.json'):
                continue
            with source.extractfile(member) as stream:
                raw = stream.read()
            meta, events, epoch_ns, epoch_monotonic = read_run(raw, path.suffix)
            folder = desktop/'rosbags'/path.stem
            counts, info = write_bag(folder, meta, events, epoch_ns, epoch_monotonic)
            verify_bag(folder, events, counts)
            results.append(dict(bag=folder.relative_to(desktop).as_posix(), source=member.name,
                                source_sha256=sha256(raw).hexdigest(), mode=meta.get('mode'),
                                messages=info['message_count'], scan=counts['/scan'], odom=counts['/replay/odom'],
                                control=counts['/replay/control_pwm'], duration_s=info['duration']['nanoseconds']/NANOSECONDS,
                                topics=dict(counts)))
            if len(results)%25 == 0:
                print(f'Bags created and read back: {len(results)}', flush=True)
    results.sort(key=lambda item: item['bag'])
    (desktop/'manifest.json').write_text(json.dumps(results, indent=2, ensure_ascii=False)+'\n', encoding='utf8')
    with (desktop/'bag_index.csv').open('w', encoding='utf-8-sig', newline='') as stream:
        columns = ('bag','mode','scan','odom','control','messages','duration_s','source')
        writer = csv.DictWriter(stream, columns, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(results)
    assert len(results)==210
    print('Verified bags, scans, odometry and control:',len(results),
          *(sum(item[name] for item in results) for name in ('scan','odom','control')),flush=True)
    print(next(item for item in results if item['bag'].endswith(LATEST)), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('offline_kit', type=Path)
    parser.add_argument('desktop', type=Path)
    args = parser.parse_args()
    extract_source(args.archive, args.offline_kit, args.desktop)
    convert(args.archive, args.desktop)
