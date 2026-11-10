"""Pure data tests for the shared known-map planner parameter overlay."""
from copy import deepcopy
import hashlib
import struct

import numpy as np
import pytest

from go2_offline_navigation.map_io import map_identity, offline_planner_settings


def sample_inputs():
    metadata = dict(frame_id='map', database_signature={
        'map.pcd': 'a' * 64, 'manifest.csv': 'b' * 64},
        terrain_parameters=dict(grid_resolution=0.2))
    support = np.array([[0.2, 0.4, -0.5], [1.2, 0.4, -0.5]], dtype=np.float64)
    obstacles = np.array([[1.0, 0.0, 0.1]], dtype=np.float32)
    planner = dict(surface_connection_radius=0.25, resolution=0.1,
                   use_static_map=False, support_topic='/terrain/support',
                   inflate_radius=0.24, nested=dict(unchanged=True))
    return metadata, support, obstacles, planner


def test_overlay_matches_known_map_launch_settings_with_native_types():
    metadata, support, obstacles, planner = sample_inputs()
    identity = map_identity(metadata, support, obstacles)
    result = offline_planner_settings(metadata, support, obstacles, planner)
    expected = {
        'support_topic': '/offline_map/support',
        'obstacle_topic': '/terrain/obstacles_measured_global',
        'live_obstacle_max_observation_age': 0.6,
        'live_obstacle_freshness_timeout': 2.0,
        'use_static_map': True,
        'localization_ready_topic': '/offline_localization/planning_ready',
        'verify_static_map_identity': True,
        'static_map_id_topic': '/offline_map/id',
        'expected_map_id': identity['map_id'],
        'expected_support_sha256': hashlib.sha256(
            np.ascontiguousarray(support, dtype='<f4').tobytes()).hexdigest(),
        'expected_static_obstacles_sha256': hashlib.sha256(
            b''.join(struct.pack('<fffB3x', *point, 4) for point in obstacles)).hexdigest(),
        'static_obstacle_topic': '/offline_map/obstacles',
        'compensate_height_drift': False,
        'reuse_previous_path': False,
        'cached_goal_obstacle_grace_period': 0.0,
        'surface_connection_radius': pytest.approx(0.3),
        'retain_start_projection_bridge': True,
    }
    assert result == expected
    for key in ('use_static_map', 'verify_static_map_identity', 'compensate_height_drift',
                'reuse_previous_path', 'retain_start_projection_bridge'):
        assert type(result[key]) is bool
    for key in ('cached_goal_obstacle_grace_period', 'surface_connection_radius'):
        assert type(result[key]) is float
    for key in ('expected_map_id', 'expected_support_sha256',
                'expected_static_obstacles_sha256'):
        assert type(result[key]) is str and len(result[key]) == 64


def test_overlay_does_not_mutate_map_or_online_planner_inputs():
    metadata, support, obstacles, planner = sample_inputs()
    metadata_before, planner_before = deepcopy(metadata), deepcopy(planner)
    support_before, obstacles_before = support.copy(), obstacles.copy()
    result = offline_planner_settings(metadata, support, obstacles, planner)
    result['use_static_map'] = False
    assert metadata == metadata_before
    assert planner == planner_before
    np.testing.assert_array_equal(support, support_before)
    np.testing.assert_array_equal(obstacles, obstacles_before)


@pytest.mark.parametrize('retention', [0.2, 0.6, 1.2])
def test_observation_retention_matches_terrain_input(retention):
    result = offline_planner_settings(*sample_inputs(), measured_obstacle_decay_time=retention)
    assert result['live_obstacle_max_observation_age'] == retention


@pytest.mark.parametrize('retention', [0, -1, float('nan'), float('inf')])
def test_invalid_observation_retention_is_rejected(retention):
    with pytest.raises(ValueError):
        offline_planner_settings(*sample_inputs(), measured_obstacle_decay_time=retention)


@pytest.mark.parametrize('changed', ['support', 'obstacles', 'database'])
def test_overlay_identity_follows_the_actual_payload_and_database(changed):
    metadata, support, obstacles, planner = sample_inputs()
    before = offline_planner_settings(metadata, support, obstacles, planner)
    if changed == 'database':
        metadata['database_signature']['manifest.csv'] = 'c' * 64
    elif changed == 'support':
        support[0, 0] += 0.1
    else:
        obstacles[0, 0] += 0.1
    after = offline_planner_settings(metadata, support, obstacles, planner)
    assert after['expected_map_id'] != before['expected_map_id']
    for layer, key in [('support', 'expected_support_sha256'),
                       ('obstacles', 'expected_static_obstacles_sha256')]:
        assert (after[key] != before[key]) == (changed == layer)
    assert after['surface_connection_radius'] == before['surface_connection_radius']


def test_overlay_preserves_explicit_larger_connection_radius_and_legacy_spacing():
    metadata, support, obstacles, planner = sample_inputs()
    planner['surface_connection_radius'] = 0.4
    assert offline_planner_settings(metadata, support, obstacles, planner)[
        'surface_connection_radius'] == 0.4
    metadata.pop('terrain_parameters')
    planner['surface_connection_radius'] = 0.25
    assert offline_planner_settings(metadata, support, obstacles, planner)[
        'surface_connection_radius'] == 0.25


def test_overlay_accepts_empty_fixed_obstacles_without_changing_its_identity():
    metadata, support, _, planner = sample_inputs()
    empty = np.empty((0, 3), dtype=np.float32)
    result = offline_planner_settings(metadata, support, empty, planner)
    assert result['expected_static_obstacles_sha256'] == hashlib.sha256(b'').hexdigest()


def test_live_obstacle_input_is_explicit_and_does_not_remove_static_hazards():
    metadata, support, obstacles, planner = sample_inputs()
    result = offline_planner_settings(metadata, support, obstacles, planner,
                                     live_obstacle_topic='/terrain/obstacles_global')
    assert result['obstacle_topic'] == '/terrain/obstacles_global'
    assert result['static_obstacle_topic'] == '/offline_map/obstacles'
    assert result['expected_static_obstacles_sha256'] == map_identity(
        metadata, support, obstacles)['clouds']['obstacles']['data_sha256']
    for bad in ('relative', '', '/bad topic'):
        with pytest.raises(ValueError):
            offline_planner_settings(metadata, support, obstacles, planner, bad)


@pytest.mark.parametrize('freshness', [1.0, 2.0, 2.5])
def test_global_input_age_is_independent_of_measured_point_retention(freshness):
    result = offline_planner_settings(*sample_inputs(), live_obstacle_freshness_timeout=freshness)
    assert result['live_obstacle_freshness_timeout'] == freshness
    assert result['live_obstacle_max_observation_age'] == .6


@pytest.mark.parametrize('freshness', [0, -1, float('nan'), float('inf')])
def test_invalid_global_input_age_is_rejected(freshness):
    with pytest.raises(ValueError):
        offline_planner_settings(*sample_inputs(), live_obstacle_freshness_timeout=freshness)
