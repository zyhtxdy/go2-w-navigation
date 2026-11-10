"""Bounded, opt-in replay evidence; never classifies a return's physical source.

Subscriptions retain original messages. Compression and neighborhood queries run
only when the replay has ended, so they do not stall planning action callbacks.
"""
from collections import deque
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from rclpy.serialization import serialize_message
from rclpy.time import Time


_NANOSECOND = 1000000000


def _stamp_ns(stamp):
    return int(stamp.sec) * _NANOSECOND + int(stamp.nanosec)


def xyz_points(message, include_observations=False):
    """Read XYZ respecting row padding, offsets and endianness; ignore NaN/Inf."""
    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ('x', 'y', 'z')):
        raise ValueError('PointCloud2 does not contain x/y/z fields')
    count, width, height = int(message.point_step), int(message.width), int(message.height)
    if count <= 0 or width < 0 or height < 0 or int(message.row_step) < width * count:
        raise ValueError('Invalid PointCloud2 dimensions or point step')
    if len(message.data) < int(message.row_step) * height:
        raise ValueError('Truncated PointCloud2 data')
    formats, offsets = [], []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.count != 1 or field.datatype not in (7, 8):
            raise ValueError('XYZ fields must be scalar FLOAT32 or FLOAT64')
        dtype = ('>' if message.is_bigendian else '<') + ('f4' if field.datatype == 7 else 'f8')
        if field.offset < 0 or field.offset + np.dtype(dtype).itemsize > count:
            raise ValueError('XYZ field outside point step')
        formats.append(dtype)
        offsets.append(field.offset)
    names = ['x', 'y', 'z']
    has_observations = all(name in fields for name in ('observation_sec', 'observation_nanosec'))
    if include_observations and has_observations:
        for name, expected, kind in (('observation_sec', 5, 'i4'), ('observation_nanosec', 6, 'u4')):
            field = fields[name]
            if field.count != 1 or field.datatype != expected or field.offset < 0 or field.offset+4 > count:
                raise ValueError('Invalid observation timestamp PointCloud2 fields')
            names.append(name)
            formats.append(('>' if message.is_bigendian else '<') + kind)
            offsets.append(field.offset)
    dtype = np.dtype(dict(names=names, formats=formats, offsets=offsets, itemsize=count))
    array = np.ndarray((height, width), dtype=dtype, buffer=message.data,
                       strides=(int(message.row_step), count))
    points = np.column_stack([array[name].reshape(-1) for name in ('x', 'y', 'z')]).astype(float)
    valid = np.isfinite(points).all(axis=1)
    if include_observations:
        observations = None
        if has_observations:
            seconds = array['observation_sec'].reshape(-1).astype(np.int64)
            nanoseconds = array['observation_nanosec'].reshape(-1).astype(np.int64)
            observations = seconds*_NANOSECOND + nanoseconds
            observations[(seconds < 0) | (nanoseconds >= _NANOSECOND)] = 0
            observations = observations[valid]
        return points[valid], observations
    return points[valid]


def transform_points(points, transform):
    """Apply a saved ROS parent<-child transform to a row array of XYZ points."""
    x, y, z, w = transform['quaternion']
    norm = x*x + y*y + z*z + w*w
    if not math.isfinite(norm) or abs(norm - 1.) > .01:
        raise ValueError('Invalid transform quaternion')
    matrix = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                       [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                       [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return points @ matrix.T + np.asarray(transform['translation'])


def collision_target(diagnostic):
    """Use a recorded source return, never a footprint test sample as a return."""
    obstacle = diagnostic.get('obstacle') or {}
    point = obstacle.get('point_current_datum')
    if diagnostic.get('frame_id') != 'map' or not isinstance(point, list) or len(point) != 3:
        return None
    if not all(type(value) in (int, float) and math.isfinite(value) for value in point):
        return None
    # The live cloud's original map-height observation is recorded separately
    # from the planner's internally adjusted datum. Prefer that direct evidence.
    result = [float(value) for value in point]
    observed_z = obstacle.get('point_observed_z', result[2])
    if type(observed_z) not in (int, float) or not math.isfinite(observed_z):
        return None
    result[2] = float(observed_z)
    return result


class ObstacleEvidence:
    """Retain bounded raw/actual-obstacle windows and independent source stamps."""
    def __init__(self, output, tf_buffer, *, raw_topic='/hesai/points',
                 obstacle_topic='/terrain/obstacles_measured_global',
                 registered_topic='/cloud_registered_body',
                 pre_s=3., post_s=3., radius_m=.15, memory_mib=256., disk_mib=128., max_events=16):
        for name, value in dict(pre_s=pre_s, post_s=post_s, radius_m=radius_m,
                                memory_mib=memory_mib, disk_mib=disk_mib).items():
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be positive and finite')
        if type(max_events) is not int or max_events <= 0:
            raise ValueError('max_events must be a positive integer')
        self.output, self.tf_buffer = Path(output), tf_buffer
        self.raw_topic, self.obstacle_topic = raw_topic, obstacle_topic
        self.registered_topic = registered_topic
        self.pre_ns, self.post_ns = round(pre_s*_NANOSECOND), round(post_s*_NANOSECOND)
        self.radius_m, self.max_events = radius_m, max_events
        self.memory_limit, self.disk_limit = int(memory_mib*1048576), int(disk_mib*1048576)
        self.records, self.order, self.ring = {}, deque(), deque()
        self.events, self.dropped = [], []
        self.sequence, self.retained_bytes = 0, 0
        self.odometry = deque()
        self.duplicates, self.invalid_stamps, self.memory_dropped = 0, 0, 0
        self.closed = False

    def observe(self, topic, message, clock_ns):
        if self.closed:
            return
        stamp = _stamp_ns(message.header.stamp)
        if stamp <= 0:
            self.invalid_stamps += 1
            return
        # A repeated publication of the same original header is one source
        # frame. A new terrain publication stamp is NOT an independent raw hit.
        key = (topic, stamp, message.header.frame_id)
        if key in self.records:
            self.duplicates += 1
            return
        self.sequence += 1
        record = dict(id=self.sequence, key=key, topic=topic, stamp_ns=stamp,
                      frame_id=message.header.frame_id, received_clock_ns=int(clock_ns),
                      message=message, bytes=len(message.data) + 2048, transforms={},
                      events=set())
        self.records[key] = record
        self.order.append(key)
        self.ring.append(key)
        self.retained_bytes += record['bytes']
        for event in self.events:
            if event['window_start_ns'] <= clock_ns <= event['window_end_ns']:
                record['events'].add(event['id'])
        self._resolve_transforms(record, clock_ns)
        self._prune(clock_ns)

    def observe_odometry(self, message, clock_ns):
        self.odometry.append((int(clock_ns), message))
        # Pose messages are small; retain a bounded prehistory / 5000 samples,
        # for every active six-second capture window. Selected evidence is copied
        # to events immediately so subsequent pruning cannot remove it.
        for event in self.events:
            if event['window_start_ns'] <= clock_ns <= event['window_end_ns']:
                if len(event['odometry']) < 1000:
                    event['odometry'].append(message)
                else:
                    event['odometry_omitted_samples'] = event.get('odometry_omitted_samples', 0) + 1
        cutoff = int(clock_ns) - self.pre_ns - _NANOSECOND
        while self.odometry and (self.odometry[0][0] < cutoff or len(self.odometry) > 5000):
            self.odometry.popleft()

    def _resolve_transforms(self, record, clock_ns):
        attempts = record.get('tf_attempts', 0)
        record['tf_attempts'] = attempts+1
        record['next_tf_retry_ns'] = int(clock_ns) + round(min(.1*2**min(attempts, 4), 1.)*_NANOSECOND)
        for parent in ('map', 'base_link'):
            if parent in record['transforms']:
                continue
            if record['frame_id'] == parent:
                record['transforms'][parent] = dict(parent=parent, child=parent,
                    stamp_ns=record['stamp_ns'], translation=[0., 0., 0.], quaternion=[0., 0., 0., 1.])
                continue
            try:
                value = self.tf_buffer.lookup_transform(parent, record['frame_id'],
                                                       Time(nanoseconds=record['stamp_ns']))
                t, q = value.transform.translation, value.transform.rotation
                values = [t.x, t.y, t.z, q.x, q.y, q.z, q.w]
                if not all(math.isfinite(item) for item in values):
                    raise ValueError('Nonfinite transform')
                record['transforms'][parent] = dict(parent=parent, child=record['frame_id'],
                    stamp_ns=_stamp_ns(value.header.stamp), requested_stamp_ns=record['stamp_ns'],
                    translation=values[:3], quaternion=values[3:])
            except Exception as error:
                record.setdefault('tf_errors', {})[parent] = str(error)

    def _prune(self, clock_ns):
        # Uncaptured prehistory is bounded in both source-clock duration and
        # bytes. Captures share these records; memory pressure is explicit.
        cutoff = int(clock_ns) - self.pre_ns - _NANOSECOND
        while self.ring:
            record = self.records.get(self.ring[0])
            if record is not None and record['received_clock_ns'] >= cutoff:
                break
            key = self.ring.popleft()
            if record is not None and not record['events']:
                self.retained_bytes -= record['bytes']
                del self.records[key]
        while self.retained_bytes > self.memory_limit and self.order:
            # A captured event must survive subsequent unrelated rolling data.
            # Prefer sacrificing uncaptured prehistory; only when the captures
            # alone exceed the budget may their oldest frame be evicted.
            disposable = next((key for key in self.order
                if key in self.records and not self.records[key]['events']), None)
            if disposable is None:
                key = self.order.popleft()
            else:
                key = disposable
                self.order.remove(key)
            record = self.records.pop(key, None)
            if record is None:
                continue
            self.memory_dropped += 1
            self.retained_bytes -= record['bytes']
            self.dropped.append(dict(record_id=record['id'], topic=record['topic'],
                stamp_ns=record['stamp_ns'], event_ids=sorted(record['events']), reason='memory_limit'))
        # Keep bookkeeping itself bounded, including keys already pruned.
        self.order = deque(key for key in self.order if key in self.records)
        self.ring = deque(key for key in self.ring if key in self.records)

    def tick(self, clock_ns):
        if self.closed:
            return
        # Only retry recent unresolved transforms, while their exact timestamp
        # is still in the TF cache. Historical captured messages are not queried
        # against a newer latest transform.
        cutoff = int(clock_ns) - self.pre_ns - _NANOSECOND
        for key in self.ring:
            record = self.records[key]
            if record['received_clock_ns'] >= cutoff and len(record['transforms']) < 2 and \
                    clock_ns >= record.get('next_tf_retry_ns', 0):
                self._resolve_transforms(record, clock_ns)
        self._prune(clock_ns)

    def capture(self, diagnostic, clock_ns, context=None):
        target = collision_target(diagnostic)
        if target is None:
            return None
        if len(self.events) >= self.max_events:
            self.dropped.append(dict(reason='event_limit', request_stamp_ns=diagnostic.get('request_stamp_ns')))
            return None
        request_ns = diagnostic.get('request_stamp_ns')
        center = request_ns if type(request_ns) is int and request_ns > 0 else int(clock_ns)
        event = dict(id=len(self.events)+1, received_clock_ns=int(clock_ns),
            window_start_ns=center-self.pre_ns, window_end_ns=center+self.post_ns,
            target_map_xyz=target, diagnostic=diagnostic, context=context or {},
            odometry=[message for clock, message in self.odometry
                      if center-self.pre_ns <= clock <= center+self.post_ns][-1000:])
        self.events.append(event)
        for record in self.records.values():
            if event['window_start_ns'] <= record['received_clock_ns'] <= event['window_end_ns']:
                record['events'].add(event['id'])
        return event['id']

    def write(self, final_clock_ns):
        """Write original CDR + saved TF and separate per-event geometric facts."""
        self.closed = True
        self.output.mkdir(parents=True, exist_ok=True)
        saved, used_bytes = [], 0
        event_frames = {event['id']: [] for event in self.events}
        for record in sorted(self.records.values(), key=lambda item: item['id']):
            if not record['events']:
                continue
            metadata = {key: record[key] for key in ('id', 'topic', 'stamp_ns', 'frame_id',
                                                      'received_clock_ns', 'transforms')}
            metadata['event_ids'] = sorted(record['events'])
            metadata['missing_tf'] = {parent: record.get('tf_errors', {}).get(parent, 'unavailable')
                                      for parent in ('map', 'base_link') if parent not in record['transforms']}
            payload = gzip.compress(serialize_message(record['message']), compresslevel=1, mtime=0)
            if used_bytes + len(payload) <= self.disk_limit:
                name = 'cloud_%06d.cdr.gz' % record['id']
                (self.output/name).write_bytes(payload)
                metadata.update(file=name, sha256=hashlib.sha256(payload).hexdigest(), bytes=len(payload),
                                ros_type='sensor_msgs/msg/PointCloud2', serialization_format='cdr+gzip')
                used_bytes += len(payload)
            else:
                metadata['file_omitted_reason'] = 'disk_limit'
            try:
                points, observations = xyz_points(record['message'], include_observations=True)
                metadata['finite_points'] = len(points)
                if 'map' not in record['transforms']:
                    raise ValueError('Exact source-stamp map transform unavailable')
                map_points = transform_points(points, record['transforms']['map'])
                for event_id in record['events']:
                    event = self.events[event_id-1]
                    target = np.asarray(event['target_map_xyz'])
                    distances = np.linalg.norm(map_points-target, axis=1)
                    nearby = np.flatnonzero(distances <= self.radius_m)
                    value = dict(record_id=record['id'], topic=record['topic'], stamp_ns=record['stamp_ns'],
                                 received_clock_ns=record['received_clock_ns'], near_point_count=int(len(nearby)),
                                 minimum_distance_m=float(distances.min()) if len(distances) else None)
                    if observations is not None:
                        observed = sorted({int(stamp) for stamp in observations[nearby] if stamp > 0})
                        value['nearby_observation_stamps_ns'] = observed
                        value['nearby_observation_unknown_points'] = int(sum(observations[nearby] <= 0))
                        value['oldest_nearby_observation_age_s'] = (
                            (record['stamp_ns']-observed[0])/_NANOSECOND if observed else None)
                    if len(nearby):
                        nearest = int(nearby[np.argmin(distances[nearby])])
                        value['nearest_raw_xyz'] = points[nearest].tolist()
                        value['nearest_map_xyz'] = map_points[nearest].tolist()
                        if 'base_link' in record['transforms']:
                            value['nearest_base_link_xyz'] = transform_points(
                                points[nearest:nearest+1], record['transforms']['base_link'])[0].tolist()
                    event_frames[event_id].append(value)
            except (TypeError, ValueError, IndexError) as error:
                metadata['neighborhood_error'] = str(error)
            saved.append(metadata)
        events = []
        for event in self.events:
            value = {key: item for key, item in event.items() if key != 'odometry'}
            # Odometry stores the original stamped pose/velocity that generated
            # body TF; exact CDR is saved as a separate bounded gzip payload.
            odometry = event['odometry']
            value['odometry_samples'] = len(odometry)
            if odometry:
                payload = gzip.compress(b''.join(len(data).to_bytes(4, 'little') + data
                    for data in (serialize_message(message) for message in odometry)),
                    compresslevel=1, mtime=0)
                if used_bytes + len(payload) <= self.disk_limit:
                    name = 'event_%03d_odometry.cdrseq.gz' % event['id']
                    (self.output/name).write_bytes(payload)
                    value['odometry_file'] = dict(file=name, bytes=len(payload),
                        sha256=hashlib.sha256(payload).hexdigest(), ros_type='nav_msgs/msg/Odometry',
                        serialization_format='gzip of repeated uint32_le length + original CDR')
                    used_bytes += len(payload)
                else:
                    value['odometry_file_omitted_reason'] = 'disk_limit'
            frames = sorted(event_frames[event['id']], key=lambda item: (item['stamp_ns'], item['record_id']))
            raw = [item for item in frames if item['topic'] == self.raw_topic]
            raw_hits = [item for item in raw if item['near_point_count'] > 0]
            registered_hits = sorted({item['stamp_ns'] for item in frames
                if item['topic'] == self.registered_topic and item['near_point_count'] > 0})
            hits = sorted({item['stamp_ns'] for item in raw_hits})
            obstacle_hits = [item for item in frames if item['topic'] == self.obstacle_topic and
                             item['near_point_count'] > 0]
            last_obstacle_hit = max((item['stamp_ns'] for item in obstacle_hits), default=None)
            obstacle_absences = [item['stamp_ns'] for item in frames
                if item['topic'] == self.obstacle_topic and item['near_point_count'] == 0
                and (last_obstacle_hit is None or item['stamp_ns'] > last_obstacle_hit)]
            observed_sources = sorted({stamp for item in obstacle_hits
                for stamp in item.get('nearby_observation_stamps_ns', [])})
            last_hit = max(hits) if hits else None
            raw_absences = [item['stamp_ns'] for item in raw if item['near_point_count'] == 0 and
                            (last_hit is None or item['stamp_ns'] > last_hit)]
            retained_raw_stamps = {record['stamp_ns'] for record in self.records.values()
                if event['id'] in record['events'] and record['topic'] == self.raw_topic}
            value.update(frames=frames, independent_raw_frames=len(retained_raw_stamps),
                raw_frames_with_map_tf=len({item['stamp_ns'] for item in raw}),
                independent_raw_hit_frames=len(hits), first_raw_hit_stamp_ns=hits[0] if hits else None,
                last_raw_hit_stamp_ns=last_hit,
                first_raw_absence_after_last_hit_stamp_ns=min(raw_absences) if raw_absences else None,
                registered_hit_frames=len(registered_hits),
                first_registered_hit_stamp_ns=registered_hits[0] if registered_hits else None,
                last_registered_hit_stamp_ns=registered_hits[-1] if registered_hits else None,
                obstacle_hit_publications=len(obstacle_hits),
                distinct_obstacle_observation_frames=len(observed_sources),
                first_obstacle_observation_stamp_ns=observed_sources[0] if observed_sources else None,
                last_obstacle_observation_stamp_ns=observed_sources[-1] if observed_sources else None,
                first_obstacle_hit_stamp_ns=min((item['stamp_ns'] for item in obstacle_hits), default=None),
                last_obstacle_hit_stamp_ns=last_obstacle_hit,
                first_obstacle_absence_after_last_hit_stamp_ns=min(obstacle_absences) if obstacle_absences else None,
                post_window_complete=final_clock_ns >= event['window_end_ns'])
            events.append(value)
        report = dict(schema_version=1, raw_topic=self.raw_topic, obstacle_topic=self.obstacle_topic,
            registered_topic=self.registered_topic,
            pre_s=self.pre_ns/_NANOSECOND, post_s=self.post_ns/_NANOSECOND, radius_m=self.radius_m,
            memory_limit_bytes=self.memory_limit, disk_limit_bytes=self.disk_limit,
            retained_payload_estimate_bytes=self.retained_bytes, saved_cdr_bytes=used_bytes,
            duplicate_source_headers_ignored=self.duplicates, invalid_source_stamps=self.invalid_stamps,
            memory_dropped_records=self.memory_dropped, dropped=self.dropped,
            final_clock_ns=int(final_clock_ns), records=saved, events=events,
            limits=[
                'Raw-frame independence means distinct positive original LiDAR header stamps; terrain publication stamps are not raw-frame evidence.',
                'Neighborhood evidence uses source-stamp rigid transforms; it is not point-wise deskewed and does not identify people, self returns or reflections.',
                'First/last hits apply only to this saved bounded window and geometric radius, not the complete bag or physical obstacle lifetime.',
                'An absent nearby raw hit is not proof of free space; visibility and beam coverage are not established.',
                'PointCloud2 files preserve original message fields, header and payload. Saved transforms use the exact cloud stamp, never latest TF.'])
        (self.output/'evidence.json').write_text(json.dumps(report, ensure_ascii=False, allow_nan=False,
                                                         indent=2)+'\n', encoding='utf-8')
        return dict(path=str(self.output/'evidence.json'), events=len(events), cloud_records=len(saved),
                    saved_cdr_bytes=used_bytes, dropped_records=len(self.dropped))
