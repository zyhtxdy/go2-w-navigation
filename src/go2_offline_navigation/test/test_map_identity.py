"""Static map identity, using only a private localhost ROS domain."""
import hashlib
import json
import os
import struct
import time

os.environ['ROS_DOMAIN_ID'] = '152'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
for key in ('CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
            'ROS_DISCOVERY_SERVER', 'ROS_STATIC_PEERS', 'ROS_AUTOMATIC_DISCOVERY_RANGE'):
    os.environ.pop(key, None)

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from go2_offline_navigation.map_io import (
    cloud_message, database_signature, load_map, map_identity, obstacle_sources,
    sha256, xyz_payload)
from go2_offline_navigation.map_server import OfflineMapServer


def sample_map():
    metadata = dict(frame_id='map', database_signature={
        'map.pcd': 'a' * 64, 'manifest.csv': 'b' * 64})
    support = np.array([[0.2, 0.4, -0.5], [1.2, 0.4, -0.5]], dtype=np.float64)
    obstacles = np.array([[1.0, 0.0, 0.1]], dtype=np.float32)
    return metadata, support, obstacles


def write_map(tmp_path, obstacles=None, source_rows=None, signed=True):
    """A real checksummed map fixture, including an optional source sidecar."""
    database, directory = tmp_path / 'database', tmp_path / 'navigation'
    database.mkdir()
    directory.mkdir()
    (database / 'map.pcd').write_text('synthetic signature fixture\n')
    (database / 'manifest.csv').write_text('synthetic manifest fixture\n')
    _, support, default_obstacles = sample_map()
    if obstacles is None:
        obstacles = default_obstacles
    for name, points in [('support.csv', support), ('obstacles.csv', obstacles)]:
        np.savetxt(directory / name, points, delimiter=',', header='x,y,z', comments='')
    metadata = dict(format_version=1, frame_id='map', database_signature=database_signature(database),
                    files={name: dict(sha256=sha256(directory / name), points=len(points))
                           for name, points in [('support.csv', support), ('obstacles.csv', obstacles)]})
    if source_rows is not None:
        sidecar = directory / 'hazard_sources.csv'
        sidecar.write_text('x,y,z,source\n' + ''.join(
            ','.join(map(str, row)) + '\n' for row in source_rows), encoding='ascii')
        if signed:
            metadata['diagnostic_files'] = {
                'hazard_sources.csv': dict(sha256=sha256(sidecar))}
    (directory / 'navigation_map.json').write_text(json.dumps(metadata))
    return database, directory, metadata


def legacy_geometry_id(metadata, support, obstacles):
    # Independently reproduce the old public schema, using struct rather than
    # the new obstacle serializer. Old saved goal lists contain this exact ID.
    clouds = {}
    for name, points in [('support', support), ('obstacles', obstacles)]:
        payload = b''.join(struct.pack('<fff', *point) for point in points)
        clouds[name] = dict(topic='/offline_map/' + name, points=len(points),
                            data_sha256=hashlib.sha256(payload).hexdigest())
    body = dict(schema_version=1, frame_id='map',
                database_signature=metadata['database_signature'], clouds=clouds)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(',', ':')).encode('utf-8')).hexdigest()


def decoded_obstacles(message):
    return list(struct.iter_unpack('<fffB3x', bytes(message.data)))


def test_identity_hashes_the_actual_cloud_bytes_and_complete_stable_schema():
    metadata, support, obstacles = sample_map()
    identity = map_identity(metadata, support, obstacles)
    assert identity['schema_version'] == 2
    assert identity['frame_id'] == 'map'
    assert identity['database_signature'] == metadata['database_signature']
    assert identity['geometry_id'] == legacy_geometry_id(metadata, support, obstacles)
    for name, points in (('support', support), ('obstacles', obstacles)):
        sources = obstacle_sources(metadata, points) if name == 'obstacles' else None
        message = cloud_message(points, Time(sec=123), sources=sources)
        assert message.header.frame_id == 'map'
        assert message.height == 1 and message.width == len(points)
        assert not message.is_bigendian
        step = 16 if name == 'obstacles' else 12
        assert message.point_step == step and message.row_step == step * len(points)
        fields = [
            (name, offset, PointField.FLOAT32, 1) for name, offset in
            [('x', 0), ('y', 4), ('z', 8)]]
        if name == 'obstacles':
            fields.append(('obstacle_source', 12, PointField.UINT8, 1))
            assert [row[3] for row in decoded_obstacles(message)] == [4]
            assert bytes(message.data)[13:16] == b'\x00\x00\x00'
        assert [(f.name, f.offset, f.datatype, f.count) for f in message.fields] == fields
        assert identity['clouds'][name] == dict(
            topic='/offline_map/' + name, points=len(points),
            data_sha256=hashlib.sha256(bytes(message.data)).hexdigest())
    body = {key: value for key, value in identity.items() if key != 'map_id'}
    assert identity['map_id'] == hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(',', ':')).encode('utf-8')).hexdigest()


def test_obstacle_provenance_changes_map_identity_but_preserves_legacy_geometry_id():
    metadata, support, obstacles = sample_map()
    identities = [map_identity(dict(metadata, _obstacle_sources=[source]), support, obstacles)
                  for source in (1, 2, 4)]
    assert len({identity['map_id'] for identity in identities}) == 3
    assert len({identity['clouds']['obstacles']['data_sha256'] for identity in identities}) == 3
    assert {identity['geometry_id'] for identity in identities} == {
        legacy_geometry_id(metadata, support, obstacles)}
    assert len({identity['clouds']['support']['data_sha256'] for identity in identities}) == 1


def test_identity_ignores_paths_metadata_order_and_float_storage_endianness():
    metadata, support, obstacles = sample_map()
    changed_metadata = dict(database_signature=dict(reversed(
        list(metadata['database_signature'].items()))), frame_id='map',
        map_directory='/some/other/path', publication_stamp=dict(sec=999, nanosec=3))
    assert map_identity(metadata, support, obstacles) == map_identity(
        changed_metadata, support.astype('>f4'), obstacles.astype('>f4'))


@pytest.mark.parametrize('change', ['support', 'obstacles', 'database'])
def test_identity_changes_when_map_or_database_changes(change):
    metadata, support, obstacles = sample_map()
    before = map_identity(metadata, support, obstacles)['map_id']
    if change == 'database':
        metadata['database_signature']['manifest.csv'] = 'c' * 64
    elif change == 'support':
        support[0, 2] += 0.1
    else:
        obstacles[0, 2] += 0.1
    assert map_identity(metadata, support, obstacles)['map_id'] != before


def test_empty_obstacle_cloud_has_valid_unambiguous_identity():
    metadata, support, _ = sample_map()
    empty = np.empty((0, 3), dtype=np.float32)
    identity = map_identity(metadata, support, empty)
    assert identity['clouds']['obstacles']['points'] == 0
    assert identity['clouds']['obstacles']['data_sha256'] == hashlib.sha256(b'').hexdigest()
    message = cloud_message(empty, Time(sec=1), sources=[])
    assert bytes(message.data) == b''
    assert message.point_step == 16 and message.row_step == 0
    assert message.fields[-1].name == 'obstacle_source'


@pytest.mark.parametrize('bad', [np.array([1, 2, 3]), np.zeros((2, 4)),
                                  np.array([[np.nan, 0, 0]]), np.array([[0, np.inf, 0]])])
def test_noncanonical_coordinates_are_rejected(bad):
    with pytest.raises(ValueError, match='finite Nx3'):
        xyz_payload(bad)


def test_invalid_frame_and_database_signature_are_rejected():
    metadata, support, obstacles = sample_map()
    for invalid in [dict(metadata, frame_id='odom'),
                    dict(metadata, database_signature={}),
                    dict(metadata, database_signature={'map.pcd': '0' * 63, 'manifest.csv': '1' * 64})]:
        with pytest.raises(ValueError):
            map_identity(invalid, support, obstacles)


@pytest.mark.parametrize('sources', [[0], [3], [7], [-1], [True], [1.0], [], [1, 2]])
def test_invalid_provenance_cannot_acquire_a_map_identity(sources):
    metadata, support, obstacles = sample_map()
    metadata['_obstacle_sources'] = sources
    with pytest.raises(ValueError, match='source labels'):
        map_identity(metadata, support, obstacles)


def test_signed_sidecar_matches_float32_coordinates_and_preserves_mixed_sources(tmp_path):
    points = np.array([[0.1, 0.2, -0.3], [0.4, 0.5, 0.6],
                       [0.7, 0.8, 0.9], [1.0, 1.1, 1.2]], dtype=np.float32)
    # Sidecar decimals differ in float64, but represent the same ROS float32
    # point. Unrecognized labels and points with no sidecar row stay unknown.
    rows = [(0.1000000001, 0.2000000001, -0.3000000001, 'measured'),
            (0.4, 0.5, 0.6, 'synthetic'), (0.7, 0.8, 0.9, 'unrecognized')]
    database, directory, _ = write_map(tmp_path, points, rows)
    metadata, _, loaded = load_map(directory, database)
    np.testing.assert_array_equal(loaded, points)
    assert metadata['_obstacle_sources'] == [1, 2, 4, 4]


def test_conflicting_source_rows_stay_unknown_but_repeated_identical_rows_are_valid(tmp_path):
    points = np.array([[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]], dtype=np.float32)
    rows = [(0.1, 0.2, 0.3, 'measured'), (0.1, 0.2, 0.3, 'synthetic'),
            (0.1, 0.2, 0.3, 'measured'), (0.4, 0.5, 0.6, 'measured'),
            (0.4, 0.5, 0.6, 'measured')]
    database, directory, _ = write_map(tmp_path, points, rows)
    metadata, _, _ = load_map(directory, database)
    assert metadata['_obstacle_sources'] == [4, 1]


@pytest.mark.parametrize('sidecar_present', [False, True])
def test_missing_or_unsigned_sidecar_does_not_refine_obstacle_geometry(tmp_path, sidecar_present):
    rows = [(1.0, 0.0, 0.1, 'measured')] if sidecar_present else None
    database, directory, metadata = write_map(tmp_path, source_rows=rows, signed=False)
    # Serialized metadata must not inject the loader's private trusted field.
    metadata['_obstacle_sources'] = [1]
    (directory / 'navigation_map.json').write_text(json.dumps(metadata))
    loaded_metadata, support, obstacles = load_map(directory, database)
    assert loaded_metadata['_obstacle_sources'] == [4]
    legacy_metadata = dict(loaded_metadata)
    legacy_metadata.pop('_obstacle_sources')
    assert map_identity(loaded_metadata, support, obstacles) == map_identity(
        legacy_metadata, support, obstacles)


def test_source_checksum_mismatch_is_rejected_before_labels_are_used(tmp_path):
    database, directory, _ = write_map(tmp_path, source_rows=[(1.0, 0.0, 0.1, 'measured')])
    sidecar = directory / 'hazard_sources.csv'
    sidecar.write_text(sidecar.read_text().replace('measured', 'synthetic'))
    with pytest.raises(ValueError, match='diagnostics checksum mismatch: hazard_sources.csv'):
        load_map(directory, database)


@pytest.mark.parametrize('contents,reason', [
    ('x,y,z,label\n1,0,0.1,measured\n', 'source CSV header'),
    ('x,y,z,source\nnan,0,0.1,measured\n', 'Nonfinite obstacle source coordinates'),
])
def test_signed_malformed_source_sidecar_is_rejected(tmp_path, contents, reason):
    database, directory, metadata = write_map(tmp_path, source_rows=[])
    sidecar = directory / 'hazard_sources.csv'
    sidecar.write_text(contents)
    metadata['diagnostic_files']['hazard_sources.csv']['sha256'] = sha256(sidecar)
    (directory / 'navigation_map.json').write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match=reason):
        load_map(directory, database)


def test_server_latches_identity_and_exact_map_for_late_joiners(tmp_path):
    obstacles = np.array([[0.1, 0.2, 0.3], [0.4, 0.5, 0.6],
                          [0.7, 0.8, 0.9], [1.0, 1.1, 1.2]], dtype=np.float32)
    rows = [(0.1, 0.2, 0.3, 'measured'), (0.4, 0.5, 0.6, 'synthetic'),
            (0.7, 0.8, 0.9, 'unknown')]
    database, directory, _ = write_map(tmp_path, obstacles, rows)
    rclpy.init(args=['--ros-args', '-p', 'map_directory:=' + str(directory),
                    '-p', 'database_directory:=' + str(database)])
    server, observer, late = None, None, None
    executor = SingleThreadedExecutor()
    try:
        server = OfflineMapServer()
        observer = Node('map_identity_observer')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        captured = {name: [] for name in ('identity', 'id', 'support', 'obstacles')}
        subscriptions = [observer.create_subscription(
            String if name in ('identity', 'id') else PointCloud2,
            '/offline_map/' + name, messages.append, qos)
            for name, messages in captured.items()]
        executor.add_node(server)
        executor.add_node(observer)

        def spin_until(predicate, seconds=5):
            until = time.monotonic() + seconds
            while time.monotonic() < until and not predicate():
                executor.spin_once(timeout_sec=0.02)
            assert predicate(), 'Static map identity or cloud did not arrive'

        spin_until(lambda: all(captured.values()))
        first = json.loads(captured['identity'][0].data)
        assert captured['id'][0].data == first['map_id']
        assert first['publisher_node'] == '/offline_map_server'
        assert first['map_directory'] == str(directory.resolve())
        assert first['database_directory'] == str(database.resolve())
        assert len(first['session_id']) == 32
        assert first['database_signature'] == database_signature(database)
        assert first['schema_version'] == 2
        assert len(first['geometry_id']) == 64
        for name in ('support', 'obstacles'):
            message = captured[name][0]
            assert first['publication_stamp'] == dict(
                sec=message.header.stamp.sec, nanosec=message.header.stamp.nanosec)
            assert first['clouds'][name]['data_sha256'] == hashlib.sha256(bytes(message.data)).hexdigest()
            assert message.point_step == (16 if name == 'obstacles' else 12)
        obstacle_message = captured['obstacles'][0]
        assert [(field.name, field.offset, field.datatype) for field in obstacle_message.fields] == [
            ('x', 0, PointField.FLOAT32), ('y', 4, PointField.FLOAT32),
            ('z', 8, PointField.FLOAT32), ('obstacle_source', 12, PointField.UINT8)]
        decoded = decoded_obstacles(obstacle_message)
        np.testing.assert_array_equal(np.asarray([row[:3] for row in decoded], dtype=np.float32),
                                      obstacles)
        assert [row[3] for row in decoded] == [1, 2, 4, 4]
        assert all(bytes(obstacle_message.data)[offset:offset + 3] == b'\x00\x00\x00'
                   for offset in range(13, obstacle_message.row_step, 16))

        late = Node('map_identity_late_observer')
        retained = {name: [] for name in captured}
        subscriptions.extend(late.create_subscription(
            String if name in ('identity', 'id') else PointCloud2,
            '/offline_map/' + name, messages.append, qos)
            for name, messages in retained.items())
        executor.add_node(late)
        spin_until(lambda: all(retained.values()))
        assert retained['identity'][0].data == captured['identity'][0].data
        assert retained['id'][0].data == captured['id'][0].data
        for name in ('support', 'obstacles'):
            assert bytes(retained[name][0].data) == bytes(captured[name][0].data)
            assert retained[name][0].header == captured[name][0].header
        # The ready timer must not refresh/rewrite the static publication.
        until = time.monotonic() + 1.1
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=0.02)
        assert all(len(messages) == 1 for messages in captured.values())
        assert all(len(messages) == 1 for messages in retained.values())
    finally:
        executor.shutdown()
        for node in (late, observer, server):
            if node is not None:
                node.destroy_node()
        rclpy.shutdown()
