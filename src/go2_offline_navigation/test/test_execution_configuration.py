"""Construct launch actions and check control parameters without starting ROS."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

from go2_offline_navigation import map_io


WORKSPACE = Path(__file__).resolve().parents[3]


def load_launch(monkeypatch):
    path = WORKSPACE / 'src/go2_offline_navigation/launch/offline_nav.launch.py'
    spec = importlib.util.spec_from_file_location('offline_execution_configuration', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    shares = dict(go2_offline_navigation='src/go2_offline_navigation',
                  go2_relocalization='src/go2_relocalization',
                  navigation_bringup='src/nav2/navigation_bringup',
                  go2_cmd_vel_bridge='src/driver/go2_cmd_vel_bridge')
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda package: str(WORKSPACE / shares[package]))
    return module


@pytest.mark.parametrize('forward', ['true', 'false'])
@pytest.mark.parametrize('angular', ['true', 'false'])
def test_offline_bridge_receives_both_compensation_switches(monkeypatch, tmp_path, forward, angular):
    module = load_launch(monkeypatch)
    # Map verification is tested separately. This test checks the actual launch
    # parameter overlay, including false overriding the bridge YAML's true.
    monkeypatch.setattr(map_io, 'load_map', lambda *_: ({}, [], []))
    monkeypatch.setattr(map_io, 'offline_planner_settings', lambda *_args, **_kwargs: {})
    context = LaunchContext()
    overrides = dict(database=str(tmp_path), map_directory=str(tmp_path),
                     start_odometry='false', start_navigation='true', start_cmd_vel_bridge='true',
                     forward_compensation=forward, angular_compensation=angular,
                     params_file=str(WORKSPACE / 'src/nav2/navigation_bringup/config/nav2_mppi_params.yaml'))
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = overrides.get(
                action.name, perform_substitutions(context, action.default_value or []))
    actions = module.start(context)
    bridge_parameters = []
    for action in actions:
        if isinstance(action, Node):
            parameters = evaluate_parameters(context, action._Node__parameters)
            bridge_parameters.extend(value for value in parameters if isinstance(value, dict)
                                     and 'forward_compensation_enabled' in value)
    assert len(bridge_parameters) == 1
    bridge = bridge_parameters[0]
    assert bridge['forward_compensation_enabled'] is (forward == 'true')
    assert bridge['angular_compensation_enabled'] is (angular == 'true')
    assert bridge['input_topic'] == '/go2/offline/cmd_vel'
    assert bridge['use_sim_time'] is False


def test_offline_compensation_defaults_match_existing_enabled_behavior(monkeypatch):
    defaults = {action.name: perform_substitutions(LaunchContext(), action.default_value or [])
                for action in load_launch(monkeypatch).generate_launch_description().entities
                if isinstance(action, DeclareLaunchArgument)}
    assert defaults['forward_compensation'] == defaults['angular_compensation'] == 'true'


def test_local_costmap_requires_fresh_marking_and_clearing_observations():
    navigation = yaml.safe_load((WORKSPACE /
        'src/nav2/navigation_bringup/config/nav2_mppi_params.yaml').read_text())
    terrain = yaml.safe_load((WORKSPACE /
        'src/nav2/terrain_costmap/config/terrain_costmap.yaml').read_text())
    processing_interval = terrain['terrain_costmap']['ros__parameters']['processing_interval']
    obstacle_layer = navigation['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']
    for name in ('terrain_obstacles', 'terrain_clear'):
        # Humble treats expected_update_rate as a maximum interval in seconds;
        # zero disables freshness checks even when the perception source dies.
        deadline = obstacle_layer[name]['expected_update_rate']
        assert 2 * processing_interval <= deadline <= 1.0


@pytest.mark.parametrize('timeout', ['2.0', '2.5'])
def test_launch_passes_one_global_freshness_value_to_planner_and_adapter(monkeypatch, tmp_path, timeout):
    module = load_launch(monkeypatch)
    monkeypatch.setattr(map_io, 'load_map', lambda *_: ({}, [], []))
    received = []
    def settings(*_args, **kwargs):
        value = float(kwargs['live_obstacle_freshness_timeout'])
        received.append(value)
        return dict(live_obstacle_freshness_timeout=value)
    monkeypatch.setattr(map_io, 'offline_planner_settings', settings)
    context = LaunchContext()
    overrides = dict(database=str(tmp_path), map_directory=str(tmp_path),
                     start_odometry='false', start_navigation='true', start_cmd_vel_bridge='false',
                     live_obstacle_freshness_timeout=timeout,
                     params_file=str(WORKSPACE / 'src/nav2/navigation_bringup/config/nav2_mppi_params.yaml'))
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = overrides.get(
                action.name, perform_substitutions(context, action.default_value or []))
    actions = module.start(context)
    values = [p['live_obstacle_freshness_timeout']
              for node in actions if isinstance(node, Node)
              for p in evaluate_parameters(context, node._Node__parameters)
              if isinstance(p, dict) and 'live_obstacle_freshness_timeout' in p]
    assert received == values == [float(timeout)]


def test_longer_global_verification_does_not_relax_current_tracking_or_local_costmap():
    values = yaml.safe_load((WORKSPACE / 'src/go2_relocalization/config/relocalization.yaml').read_text())[
        'relocalization']['ros__parameters']
    assert values['max_global_result_age'] == 6.
    assert values['max_global_verification_age'] == 12.
    assert values['max_input_age'] == values['tracking_timeout'] == 3.
    nav = yaml.safe_load((WORKSPACE / 'src/nav2/navigation_bringup/config/nav2_mppi_params.yaml').read_text())
    layers = nav['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']
    assert layers['terrain_obstacles']['expected_update_rate'] == .8
    assert layers['terrain_clear']['expected_update_rate'] == .8


def test_behavior_tree_input_wait_outlasts_adapter_recovery_budget():
    # Check both installed production tree sources, not a mirrored test tree.
    import ast
    source = ast.parse((WORKSPACE / 'src/go2_offline_navigation/go2_offline_navigation/'
                        'localization_adapter.py').read_text())
    budget = next(ast.literal_eval(keyword.value)
                  for node in ast.walk(source) if isinstance(node, ast.Call)
                  and isinstance(node.func, ast.Name) and node.func.id == 'dict'
                  for keyword in node.keywords if keyword.arg == 'input_recovery_timeout')
    assert budget == 10.
    for name in ('navigate_to_pose', 'navigate_through_poses'):
        tree = ET.parse(WORKSPACE / 'src/nav2/navigation_bringup/behavior_trees' /
                        (name + '_w_replanning_no_spin.xml'))
        guards = tree.findall('.//TerrainValidationGuard')
        assert guards
        assert all(int(guard.attrib['wait_timeout_ms']) >= (budget + 2.) * 1000
                   for guard in guards)
