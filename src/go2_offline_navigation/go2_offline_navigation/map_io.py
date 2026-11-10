"""Portable navigation-map format tied to a specific localization database."""
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def offline_connection_radius(metadata, planner):
    """Connect observed diagonal neighbors of the exported terrain lattice.

    Continuous ground, slope, step, obstacle and footprint checks still decide
    whether an edge is traversable. This does not add support or fill holes.
    Legacy maps without recorded spacing retain the configured radius.
    """
    configured = float(planner['surface_connection_radius'])
    resolution = float(planner['resolution'])
    spacing = metadata.get('terrain_parameters', {}).get('grid_resolution')
    values = [configured, resolution] + ([] if spacing is None else [float(spacing)])
    if not all(math.isfinite(value) and value > 0 for value in values):
        raise ValueError('Positive finite map spacing and planner geometry required')
    if spacing is None:
        return configured
    diagonal_cells = math.ceil(math.sqrt(2) * float(spacing) / resolution - 1e-9)
    return max(configured, diagonal_cells * resolution)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def database_signature(directory):
    directory = Path(directory)
    return {name: sha256(directory / name) for name in ('map.pcd', 'manifest.csv')}


def read_points(path, allow_empty=False):
    path = Path(path)
    if path.stat().st_size > 256 * 1024 * 1024:
        raise ValueError('Navigation point file exceeds 256 MiB; split the map')
    with path.open(encoding='ascii') as stream:
        if stream.readline().strip() != 'x,y,z':
            raise ValueError('Expected x,y,z CSV header')
        # Avoid numpy warnings for an intentionally empty obstacle map.
        if not stream.read(1):
            if allow_empty:
                return np.empty((0, 3), dtype=np.float32)
            raise ValueError('Support map is empty')
    points = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2, dtype=np.float32)
    if points.shape[1] != 3 or len(points) > 5000000 or not np.isfinite(points).all():
        raise ValueError('Invalid XYZ data or excessive point count')
    if np.abs(points).max(initial=0) > 100000:
        raise ValueError('Unreasonable point coordinates')
    return np.ascontiguousarray(points, dtype='<f4')


def load_map(directory, database=None):
    directory = Path(directory)
    metadata = json.loads((directory / 'navigation_map.json').read_text())
    if metadata.get('format_version') != 1 or metadata.get('frame_id') != 'map':
        raise ValueError('Unsupported navigation map format or frame')
    if database is not None and database_signature(database) != metadata.get('database_signature'):
        raise ValueError('Navigation map and relocalization database do not match')
    for name, expected in metadata.get('diagnostic_files', {}).items():
        if name not in ('hazard_sources.csv', 'terrain_tiles.csv', 'terrain_cells.csv'):
            raise ValueError('Unsupported navigation map diagnostics file')
        if sha256(directory / name) != expected['sha256']:
            raise ValueError('Navigation map diagnostics checksum mismatch: ' + name)
    result = []
    for name, allow_empty in (('support.csv', False), ('obstacles.csv', True)):
        expected = metadata['files'][name]
        if sha256(directory / name) != expected['sha256']:
            raise ValueError('Navigation map file checksum mismatch: ' + name)
        points = read_points(directory / name, allow_empty)
        if len(points) != expected['points']:
            raise ValueError('Navigation map point count mismatch: ' + name)
        result.append(points)
    # Only a checksummed sidecar may refine obstacle extent semantics. Match
    # float32 coordinates, exactly as sent to ROS; unmatched/conflicting labels
    # remain unknown and keep the full occupied voxel.
    sources = np.full(len(result[1]), 4, dtype=np.uint8)
    if 'hazard_sources.csv' in metadata.get('diagnostic_files', {}):
        labels = {}
        tags = {'measured': 1, 'synthetic': 2, 'unknown': 4}
        with (directory / 'hazard_sources.csv').open(encoding='ascii') as stream:
            rows = csv.DictReader(stream)
            if rows.fieldnames != ['x', 'y', 'z', 'source']:
                raise ValueError('Invalid obstacle source CSV header')
            for row in rows:
                key = tuple(np.float32(row[axis]) for axis in ('x', 'y', 'z'))
                if not all(math.isfinite(value) for value in key):
                    raise ValueError('Nonfinite obstacle source coordinates')
                tag = tags.get(row['source'], 4)
                labels[key] = tag if key not in labels else (
                    tag if labels[key] == tag else 4)
        sources = np.array([labels.get(tuple(point), 4) for point in result[1]], dtype=np.uint8)
    metadata['_obstacle_sources'] = sources.tolist()
    return metadata, result[0], result[1]


def xyz_payload(points):
    """Canonical packed XYZ float32 bytes used by map messages and identity.

    Hash the actual message payload rather than the CSV text: equivalent maps
    with different CSV formatting must have the same identity.
    """
    values = np.ascontiguousarray(points, dtype='<f4')
    if values.ndim != 2 or values.shape[1] != 3 or not np.isfinite(values).all():
        raise ValueError('Map identity requires finite Nx3 XYZ points')
    return values.tobytes()


def obstacle_sources(metadata, obstacles):
    """Validated provenance; legacy maps without a signed sidecar stay unknown."""
    values = metadata.get('_obstacle_sources', [4] * len(obstacles))
    if len(values) != len(obstacles) or any(
            isinstance(value, bool) or not isinstance(value, (int, np.integer)) or
            value not in (1, 2, 4) for value in values):
        raise ValueError('Invalid per-point obstacle source labels')
    return np.asarray(values, dtype=np.uint8)


def obstacle_payload(points, sources):
    """Canonical XYZ + source uint8 with three zero padding bytes (16 bytes)."""
    xyz_payload(points)  # Validate coordinates before structured assignment.
    sources = obstacle_sources({'_obstacle_sources': list(sources)}, points)
    values = np.zeros(len(points), dtype=[('xyz', '<f4', (3,)), ('source', 'u1'),
                                         ('padding', 'u1', (3,))])
    values['xyz'] = points
    values['source'] = sources
    return values.tobytes()


def map_identity(metadata, support, obstacles):
    """Bind both geometry and provenance to the exact published cloud bytes.

    Schema 2 adds source semantics to the obstacle payload hash. ``geometry_id``
    is the former schema-1 identity for old goal coordinates only; runtime map
    guards always require the complete schema-2 ``map_id`` and payload hashes.
    """
    if metadata.get('frame_id') != 'map':
        raise ValueError('Map identity requires frame map')
    signature = metadata.get('database_signature', {})
    if set(signature) != {'map.pcd', 'manifest.csv'} or any(
            not isinstance(value, str) or len(value) != 64 or
            any(character not in '0123456789abcdef' for character in value)
            for value in signature.values()):
        raise ValueError('Map identity requires a valid database signature')
    clouds = {}
    for name, points in (('support', support), ('obstacles', obstacles)):
        clouds[name] = dict(topic='/offline_map/' + name, points=len(points),
                            data_sha256=hashlib.sha256(xyz_payload(points)).hexdigest())
    legacy = dict(schema_version=1, frame_id='map',
                  database_signature=dict(signature), clouds=clouds)
    encoded = json.dumps(legacy, sort_keys=True, separators=(',', ':')).encode('utf-8')
    geometry_id = hashlib.sha256(encoded).hexdigest()
    clouds['obstacles']['data_sha256'] = hashlib.sha256(
        obstacle_payload(obstacles, obstacle_sources(metadata, obstacles))).hexdigest()
    result = dict(schema_version=2, frame_id='map', geometry_id=geometry_id,
                  database_signature=dict(signature), clouds=clouds)
    encoded = json.dumps(result, sort_keys=True, separators=(',', ':')).encode('utf-8')
    result['map_id'] = hashlib.sha256(encoded).hexdigest()
    return result


def offline_planner_settings(metadata, support, obstacles, planner,
                             live_obstacle_topic='/terrain/obstacles_measured_global',
                             measured_obstacle_decay_time=0.6,
                             live_obstacle_freshness_timeout=None):
    """One native-typed parameter overlay for known-map launch and evaluation.

    Inputs are left unchanged. Callers may pass this overlay directly to ROS
    parameter dictionaries, or stringify values for ``MapRewrittenYaml``. Map
    digests always remain strings; geometry comes from the existing validated
    diagonal-connection rule rather than changing support or collision gates.
    """
    identity = map_identity(metadata, support, obstacles)
    if not isinstance(live_obstacle_topic, str) or not live_obstacle_topic.startswith('/') or \
            any(character.isspace() for character in live_obstacle_topic):
        raise ValueError('An absolute live obstacle topic without whitespace is required')
    retention = float(measured_obstacle_decay_time)
    if not math.isfinite(retention) or not 0 < retention <= 3600:
        raise ValueError('Positive finite measured obstacle retention required')
    freshness = float(planner.get('live_obstacle_freshness_timeout', 2.0)
                      if live_obstacle_freshness_timeout is None else live_obstacle_freshness_timeout)
    if not math.isfinite(freshness) or not 0 < freshness <= 3600:
        raise ValueError('Positive finite live obstacle freshness timeout required')
    return {
        'support_topic': '/offline_map/support',
        # Fixed terrain retains mapped slope/step hazards; live measured returns
        # add people/objects. Local terrain still publishes/checks all hazards.
        'obstacle_topic': live_obstacle_topic,
        'live_obstacle_max_observation_age': retention,
        'live_obstacle_freshness_timeout': freshness,
        'use_static_map': True,
        'localization_ready_topic': '/offline_localization/planning_ready',
        'verify_static_map_identity': True,
        'static_map_id_topic': '/offline_map/id',
        'expected_map_id': identity['map_id'],
        'expected_support_sha256': identity['clouds']['support']['data_sha256'],
        'expected_static_obstacles_sha256': identity['clouds']['obstacles']['data_sha256'],
        'static_obstacle_topic': '/offline_map/obstacles',
        'compensate_height_drift': False,
        'reuse_previous_path': False,
        'cached_goal_obstacle_grace_period': 0.0,
        'surface_connection_radius': offline_connection_radius(metadata, planner),
        'retain_start_projection_bridge': True,
    }


def cloud_message(points, stamp, frame='map', sources=None):
    from sensor_msgs.msg import PointCloud2, PointField
    message = PointCloud2()
    message.header.frame_id = frame
    message.header.stamp = stamp
    message.height, message.width = 1, len(points)
    message.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                      for name, offset in (('x', 0), ('y', 4), ('z', 8))]
    if sources is not None:
        message.fields.append(PointField(name='obstacle_source', offset=12,
                                         datatype=PointField.UINT8, count=1))
    message.point_step = 12 if sources is None else 16
    message.row_step = message.point_step * len(points)
    message.is_bigendian = False
    message.is_dense = True
    message.data = xyz_payload(points) if sources is None else obstacle_payload(points, sources)
    return message
