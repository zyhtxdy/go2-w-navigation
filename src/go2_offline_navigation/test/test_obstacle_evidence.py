"""Original-message and temporal evidence contracts, no rclpy initialization/DDS."""
import gzip
import json
import math
import struct

from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import pytest
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from go2_offline_navigation.obstacle_evidence import (
    ObstacleEvidence, collision_target, transform_points, xyz_points)


BASE = 1700000000000000000


def cloud(stamp=BASE, points=((0., 0., 0.),), frame='lidar', *, padded=False,
          bigendian=False, observations=None):
    field_names = [('x', 7), ('y', 7), ('z', 7)]
    if observations is not None:
        field_names += [('observation_sec', 5), ('observation_nanosec', 6)]
    point_step = 4*len(field_names)
    # Organize as one point per row to exercise nontrivial row padding.
    row_padding = b'\xa5'*8 if padded else b''
    payload = b''
    for index, point in enumerate(points):
        payload += struct.pack(('>' if bigendian else '<')+'fff', *point)
        if observations is not None:
            payload += struct.pack(('>' if bigendian else '<')+'iI', *divmod(observations[index], 10**9))
        payload += row_padding
    result = PointCloud2(header=Header(stamp=TimeMessage(sec=stamp//10**9, nanosec=stamp%10**9),
                                      frame_id=frame), width=1, height=len(points),
        fields=[PointField(name=name, offset=4*index, datatype=kind, count=1)
                for index, (name, kind) in enumerate(field_names)],
        is_bigendian=bigendian, point_step=point_step, row_step=point_step+len(row_padding),
        data=payload, is_dense=False)
    return result


class Buffer:
    def __init__(self, fail=False):
        self.fail, self.requests = fail, []

    def lookup_transform(self, parent, child, stamp):
        self.requests.append((parent, child, stamp.nanoseconds))
        if self.fail:
            raise RuntimeError('No exact source-stamp TF')
        result = TransformStamped(header=Header(frame_id=parent,
            stamp=TimeMessage(sec=stamp.nanoseconds//10**9, nanosec=stamp.nanoseconds%10**9)),
            child_frame_id=child)
        result.transform.rotation.w = 1.
        if parent == 'map':
            result.transform.translation.x = 1.
        return result


def diagnostic(stamp=BASE, point=(1., 0., 0.)):
    return dict(frame_id='map', request_stamp_ns=stamp,
        obstacle=dict(point_current_datum=list(point), point_observed_z=point[2],
                      source_topic='/terrain/obstacles_measured_global'))


@pytest.mark.parametrize('bigendian', [False, True])
def test_xyz_preserves_rows_padding_endianness_and_observation_alignment(bigendian):
    value = cloud(points=((1., 2., 3.), (math.nan, 0., 0.), (-1., 4., 5.)),
                  observations=[BASE, BASE+1, BASE+2], padded=True, bigendian=bigendian)
    points, observations = xyz_points(value, include_observations=True)
    assert points.tolist() == [[1., 2., 3.], [-1., 4., 5.]]
    assert observations.tolist() == [BASE, BASE+2]


@pytest.mark.parametrize('change', [
    lambda value: value.fields.pop(),
    lambda value: setattr(value, 'point_step', 2),
    lambda value: setattr(value, 'row_step', 1),
    lambda value: setattr(value, 'data', b'\x00'),
    lambda value: setattr(value.fields[0], 'datatype', 3),
    lambda value: setattr(value.fields[0], 'offset', 100),
])
def test_malformed_cloud_is_rejected(change):
    value = cloud()
    change(value)
    with pytest.raises(ValueError):
        xyz_points(value)


def test_rotation_translation_and_source_observed_height():
    points = transform_points(np.array([[1., 0., 0.]]),
        dict(quaternion=[0., 0., math.sqrt(.5), math.sqrt(.5)], translation=[2., 3., 4.]))
    np.testing.assert_allclose(points, [[2., 4., 4.]])
    value = diagnostic()
    value['height_datum_offset'] = .4
    value['obstacle']['point_current_datum'][2] = .8
    value['obstacle']['point_observed_z'] = .3
    assert collision_target(value) == [1., 0., .3]
    assert collision_target(dict(frame_id='map', collision=dict(tested_sample=[1., 2., 3.]))) is None
    value['frame_id'] = 'odom'
    assert collision_target(value) is None


def test_capture_deduplicates_raw_frames_but_keeps_republication_and_original_cdr(tmp_path):
    buffer = Buffer()
    recorder = ObstacleEvidence(tmp_path, buffer)
    first = cloud()
    recorder.observe('/hesai/points', first, BASE)
    recorder.observe('/hesai/points', first, BASE+1)
    # Two obstacle snapshots share exactly one real source observation.
    recorder.observe(recorder.obstacle_topic, cloud(BASE, observations=[BASE]), BASE)
    recorder.observe_odometry(Odometry(header=first.header), BASE)
    assert recorder.capture(diagnostic(), BASE, context={'row_index': 1}) == 1
    recorder.observe(recorder.obstacle_topic, cloud(BASE+10**9, observations=[BASE]), BASE+10**9)
    recorder.observe('/hesai/points', cloud(BASE+10**9, points=((2., 0., 0.),)), BASE+10**9)
    recorder.observe(recorder.registered_topic, cloud(BASE+10**9), BASE+10**9)
    summary = recorder.write(BASE+3*10**9)
    report = json.loads((tmp_path/'evidence.json').read_text())
    event = report['events'][0]
    assert event['independent_raw_frames'] == 2
    assert event['independent_raw_hit_frames'] == 1
    assert event['first_raw_hit_stamp_ns'] == BASE == event['last_raw_hit_stamp_ns']
    assert event['first_raw_absence_after_last_hit_stamp_ns'] == BASE+10**9
    assert event['obstacle_hit_publications'] == 2
    assert event['distinct_obstacle_observation_frames'] == 1
    assert event['last_obstacle_observation_stamp_ns'] == BASE
    assert event['registered_hit_frames'] == 1
    assert event['post_window_complete'] is True
    assert event['odometry_samples'] == 1
    assert report['duplicate_source_headers_ignored'] == 1
    raw = next(item for item in report['records'] if item['topic'] == '/hesai/points')
    recovered = deserialize_message(gzip.decompress((tmp_path/raw['file']).read_bytes()), PointCloud2)
    assert recovered == first
    assert raw['transforms']['map']['requested_stamp_ns'] == BASE
    assert all(request[2] > 0 for request in buffer.requests)
    assert summary['cloud_records'] == 5


def test_missing_tf_retries_exact_stamp_without_inventing_raw_hit(tmp_path):
    buffer = Buffer(fail=True)
    recorder = ObstacleEvidence(tmp_path, buffer)
    recorder.observe('/hesai/points', cloud(), BASE)
    recorder.capture(diagnostic(), BASE)
    recorder.tick(BASE+10**9)
    recorder.write(BASE+2*10**9)
    report = json.loads((tmp_path/'evidence.json').read_text())
    assert report['events'][0]['independent_raw_frames'] == 1
    assert report['events'][0]['raw_frames_with_map_tf'] == 0
    assert report['events'][0]['independent_raw_hit_frames'] == 0
    assert report['events'][0]['post_window_complete'] is False
    assert report['records'][0]['missing_tf']['map'] == 'No exact source-stamp TF'
    assert report['records'][0]['neighborhood_error'] == 'Exact source-stamp map transform unavailable'
    assert all(request[2] == BASE for request in buffer.requests)


def test_tf_that_arrives_later_is_saved_at_original_stamp(tmp_path):
    buffer = Buffer(fail=True)
    recorder = ObstacleEvidence(tmp_path, buffer)
    recorder.observe('/hesai/points', cloud(), BASE)
    recorder.capture(diagnostic(), BASE)
    buffer.fail = False
    recorder.tick(BASE+10**9)
    recorder.write(BASE+3*10**9)
    report = json.loads((tmp_path/'evidence.json').read_text())
    assert report['records'][0]['missing_tf'] == {}
    assert report['events'][0]['independent_raw_hit_frames'] == 1


def test_memory_and_disk_caps_are_explicit_and_uncaptured_prehistory_expires(tmp_path):
    recorder = ObstacleEvidence(tmp_path, Buffer(), memory_mib=.0021, disk_mib=.00001)
    recorder.observe('/hesai/points', cloud(), BASE)
    recorder.capture(diagnostic(), BASE)
    recorder.observe('/hesai/points', cloud(BASE+10**9), BASE+10**9)
    recorder.write(BASE+10**9)
    report = json.loads((tmp_path/'evidence.json').read_text())
    assert report['memory_dropped_records'] == 1
    assert report['dropped'][0]['reason'] == 'memory_limit'
    assert report['records'][0]['file_omitted_reason'] == 'disk_limit'
    assert not list(tmp_path.glob('*.gz'))
    recorder = ObstacleEvidence(tmp_path/'prehistory', Buffer())
    recorder.observe('/hesai/points', cloud(), BASE)
    recorder.tick(BASE+5*10**9)
    assert not recorder.records
    assert not recorder.ring
    assert not recorder.order


def test_invalid_stamps_and_event_limit_remain_visible(tmp_path):
    recorder = ObstacleEvidence(tmp_path, Buffer(), max_events=1)
    recorder.observe('/hesai/points', cloud(stamp=0), BASE)
    assert recorder.capture(diagnostic(), BASE) == 1
    assert recorder.capture(diagnostic(), BASE) is None
    recorder.write(BASE)
    report = json.loads((tmp_path/'evidence.json').read_text())
    assert report['invalid_source_stamps'] == 1
    assert report['dropped'] == [dict(reason='event_limit', request_stamp_ns=BASE)]


@pytest.mark.parametrize('option', ['nan', 'inf', '0', '-1'])
def test_replay_rejects_invalid_retention_before_reading_bag(monkeypatch, option, capsys):
    from go2_offline_navigation.replay_probe import main
    monkeypatch.setattr('sys.argv', ['validate_replay', 'no-bag', 'no-database', 'no-map', 'output',
                                     '--measured-obstacle-decay-time', option])
    with pytest.raises(SystemExit) as error:
        main()
    assert error.value.code == 2
    assert 'Measured obstacle decay time must be positive and finite' in capsys.readouterr().err


def test_evidence_requires_planning_cases_before_reading_bag(monkeypatch, capsys):
    from go2_offline_navigation.replay_probe import main
    monkeypatch.setattr('sys.argv', ['validate_replay', 'no-bag', 'no-database', 'no-map', 'output',
                                     '--obstacle-evidence'])
    with pytest.raises(SystemExit) as error:
        main()
    assert error.value.code == 2
    assert '--obstacle-evidence requires --planning-cases' in capsys.readouterr().err


def test_later_rolling_clouds_cannot_displace_capture_when_uncaptured_frames_can_be_dropped(tmp_path):
    # Two retained messages fit; all subsequent uncaptured rolling messages
    # must give way before the earlier event's original source frame does.
    recorder = ObstacleEvidence(tmp_path, Buffer(), memory_mib=.0042)
    original = cloud()
    recorder.observe('/hesai/points', original, BASE)
    recorder.capture(diagnostic(), BASE)
    for offset in (4.1, 4.2, 4.3, 4.4):
        stamp = BASE + round(offset*10**9)
        recorder.observe('/hesai/points', cloud(stamp), stamp)
    assert any(record['stamp_ns'] == BASE for record in recorder.records.values())
    assert recorder.retained_bytes <= recorder.memory_limit
    assert recorder.memory_dropped > 0
    assert all(item['event_ids'] == [] for item in recorder.dropped)
    recorder.write(BASE+5*10**9)
    report = json.loads((tmp_path/'evidence.json').read_text())
    assert report['events'][0]['independent_raw_hit_frames'] == 1
    assert report['events'][0]['first_raw_hit_stamp_ns'] == BASE
    assert len(report['records']) == 1
    original_record = report['records'][0]
    recovered = deserialize_message(gzip.decompress((tmp_path/original_record['file']).read_bytes()),
                                    PointCloud2)
    assert recovered == original
    assert all(item['event_ids'] == [] for item in report['dropped'])
