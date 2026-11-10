"""ROS wiring checks using localhost, a separate domain and fake API topics.

No driver, Nav2 controller or physical Sport request topic is started.
"""

import json
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('unitree_api.msg')

from geometry_msgs.msg import Twist, TwistStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from unitree_api.msg import Request

from go2_cmd_vel_bridge.command import MOVE_API_ID, STOP_MOVE_API_ID
from go2_cmd_vel_bridge.node import CmdVelBridge


@pytest.fixture
def rig(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '124')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    for key in ('CYCLONEDDS_URI', 'ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                'FASTDDS_DEFAULT_PROFILES_FILE'):
        monkeypatch.delenv(key, raising=False)
    context = Context()
    rclpy.init(context=context)
    bridge = None
    probe = None
    executor = SingleThreadedExecutor(context=context)
    try:
        bridge = CmdVelBridge(context=context, parameter_overrides=[
            Parameter('input_topic', value='/test_forward_compensation/input'),
            Parameter('request_topic', value='/test_forward_compensation/fake_sport_request'),
            Parameter('estop_topic', value='/test_forward_compensation/estop'),
        ])
        probe = rclpy.create_node('forward_compensation_probe', context=context)
        requests, targets, outputs = [], [], []
        publisher = probe.create_publisher(Twist, '/test_forward_compensation/input', 10)
        subscriptions = [
            probe.create_subscription(Request, '/test_forward_compensation/fake_sport_request',
                                      requests.append, 10),
            probe.create_subscription(TwistStamped, '/go2_cmd_vel_bridge/cmd_vel_requested',
                                      targets.append, 10),
            probe.create_subscription(TwistStamped, '/go2_cmd_vel_bridge/cmd_vel_sent',
                                      outputs.append, 10),
        ]
        executor.add_node(bridge)
        executor.add_node(probe)

        def wait_until(predicate, timeout=3.0):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
            assert predicate(), 'Timed out waiting for the isolated test condition'

        wait_until(lambda: publisher.get_subscription_count() == 1 and
                   bridge.publisher.get_subscription_count() == 1 and
                   bridge.requested_publisher.get_subscription_count() == 1 and
                   bridge.sent_publisher.get_subscription_count() == 1)
        assert not probe.get_publishers_info_by_topic('/api/sport/request')
        assert not requests
        yield bridge, probe, publisher, requests, targets, outputs, wait_until, subscriptions
    finally:
        if bridge is not None:
            bridge.destroy_node()
        if probe is not None:
            probe.destroy_node()
        executor.shutdown()
        context.shutdown()


def test_default_compensation_debug_and_watchdog(rig):
    bridge, probe, publisher, requests, targets, outputs, wait_until, _ = rig
    command = Twist()
    command.linear.x = 0.3
    command.angular.z = -0.2
    publisher.publish(command)
    wait_until(lambda: requests and targets and outputs)
    move = next(message for message in requests if message.header.identity.api_id == MOVE_API_ID)
    assert json.loads(move.parameter) == pytest.approx({'x': 0.3926349, 'y': 0, 'z': -0.202374})
    assert targets[0].twist.linear.x == pytest.approx(0.3)
    assert outputs[0].twist.linear.x == pytest.approx(0.3926349)
    assert outputs[0].twist.angular.z == pytest.approx(-0.202374)
    assert targets[0].twist.angular.z == pytest.approx(-0.2)
    assert targets[0].header.stamp == outputs[0].header.stamp
    wait_until(lambda: sum(message.header.identity.api_id == STOP_MOVE_API_ID
                           for message in requests) == 3)
    wait_until(lambda: outputs[-1].twist.linear.x == 0.0)
    assert all(message.parameter == '' for message in requests
               if message.header.identity.api_id == STOP_MOVE_API_ID)
    assert not probe.get_publishers_info_by_topic('/api/sport/request')


def test_calibration_cannot_change_silently_while_node_is_running(rig):
    bridge, _, _, requests, _, _, _, _ = rig
    result = bridge.set_parameters([
        Parameter('forward_compensation_enabled', value=False)])[0]
    assert not result.successful
    assert bridge.get_parameter('forward_compensation_enabled').value is True
    assert bridge.gate.compensation.enabled
    assert not requests


def test_angular_parameters_are_read_only(rig):
    bridge, _, _, requests, _, _, _, _ = rig
    result = bridge.set_parameters([Parameter('angular_compensation_enabled', value=False)])[0]
    assert not result.successful
    assert bridge.gate.angular_compensation.enabled
    assert not requests


def test_positive_angular_compensation_can_reduce_command(rig):
    bridge, probe, publisher, requests, targets, outputs, wait_until, _ = rig
    command = Twist(); command.angular.z = 0.4
    publisher.publish(command)
    wait_until(lambda: requests and targets and outputs)
    move = next(message for message in requests if message.header.identity.api_id == MOVE_API_ID)
    assert json.loads(move.parameter) == pytest.approx({'x': 0, 'y': 0, 'z': 0.378850})
    assert targets[0].twist.angular.z == pytest.approx(0.4)
    assert outputs[0].twist.angular.z == pytest.approx(0.378850)
    assert not probe.get_publishers_info_by_topic('/api/sport/request')


def test_bridge_config_applies_to_renamed_offline_node(monkeypatch, tmp_path):
    from pathlib import Path
    monkeypatch.setenv('ROS_DOMAIN_ID', '125')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    for key in ('CYCLONEDDS_URI', 'ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                'FASTDDS_DEFAULT_PROFILES_FILE'):
        monkeypatch.delenv(key, raising=False)
    context = Context(); rclpy.init(context=context)
    bridge = None
    try:
        source = Path(__file__).resolve().parents[1] / 'config/go2_cmd_vel.yaml'
        params = tmp_path / 'renamed_bridge.yaml'
        params.write_text(source.read_text().replace('angular_compensation_gain: 0.882380',
            'angular_compensation_gain: 0.8').replace('publish_rate: 20.0', 'publish_rate: 13.0'))
        bridge = CmdVelBridge(context=context,
            cli_args=['--ros-args', '-r', '__node:=offline_cmd_vel_bridge', '--params-file', str(params)],
            parameter_overrides=[
                Parameter('input_topic', value='/test_angular_compensation/input'),
                Parameter('request_topic', value='/test_angular_compensation/fake_sport_request')])
        assert bridge.get_name() == 'offline_cmd_vel_bridge'
        assert bridge.get_parameter('angular_compensation_gain').value == pytest.approx(0.8)
        assert bridge.gate.angular_compensation.apply(-0.3) == pytest.approx(-0.265898)
        # This non-default transport field proves the wildcard YAML really applied.
        assert bridge.get_parameter('publish_rate').value == 13.0
        assert bridge.publisher.topic_name == '/test_angular_compensation/fake_sport_request'
        assert not bridge.get_publishers_info_by_topic('/api/sport/request')
    finally:
        if bridge is not None: bridge.destroy_node()
        context.shutdown()
