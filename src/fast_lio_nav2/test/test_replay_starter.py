"""Check end-of-bag clock ownership and monotonic completion in an isolated ROS domain."""

import importlib.util
import os
from pathlib import Path
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosbag2_interfaces.srv import PlayNext
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger

spec = importlib.util.spec_from_file_location(
    'replay_starter', Path(__file__).resolve().parents[1] / 'scripts/replay_starter.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def wait(executor, condition, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.02)
        if condition():
            return
    raise AssertionError('Timed out waiting for local ROS discovery or callback')


@pytest.fixture
def runtime(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(180 + os.getpid() % 40))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    for name in ['CYCLONEDDS_URI', 'ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE']:
        monkeypatch.delenv(name, raising=False)
    (tmp_path / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n'
        '  starting_time: {nanoseconds_since_epoch: 1000000000}\n'
        '  duration: {nanoseconds: 100000000}\n')
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', f'bag_path:={tmp_path}'])
    starter = module.ReplayStarter()
    # The full replay checks the priming/resume sequence. These component
    # checks begin after that stage and isolate process-exit clock handling.
    starter.stage = 'started'; starter.timer.cancel()
    observer = Node('replay_completion_observer')
    executor = SingleThreadedExecutor()
    executor.add_node(starter); executor.add_node(observer)
    try:
        yield starter, observer, executor
    finally:
        executor.shutdown()
        starter.destroy_node(); observer.destroy_node()
        rclpy.shutdown()


def test_running_player_cannot_transfer_clock_ownership(runtime):
    starter, observer, executor = runtime
    service = observer.create_service(
        PlayNext, '/rosbag2_player/play_next', lambda request, response: response)
    wait(executor, starter.step.service_is_ready)
    response = starter.finish(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert starter.final_clock_publisher is None
    observer.destroy_service(service)


@pytest.mark.parametrize('last_ns', [1080000000, 1120000000])
def test_completion_uses_original_end_and_never_rewinds_observed_time(runtime, last_ns):
    starter, observer, executor = runtime
    received = []
    observer.create_subscription(
        Clock, '/clock', lambda m: received.append(
            m.clock.sec * 1000000000 + m.clock.nanosec),
        qos_profile_sensor_data)
    previous = observer.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    value = Clock(); value.clock.sec, value.clock.nanosec = divmod(last_ns, 1000000000)
    deadline = time.monotonic() + 3
    while starter.get_clock().now().nanoseconds != last_ns and time.monotonic() < deadline:
        previous.publish(value); executor.spin_once(timeout_sec=.02)
    assert starter.get_clock().now().nanoseconds == last_ns
    observer.destroy_publisher(previous)
    # Process queued samples from the former writer before handing off, as
    # the real launch also waits after process exit. Keep them in the record.
    drain_until = time.monotonic() + .25
    while time.monotonic() < drain_until:
        executor.spin_once(timeout_sec=.02)
    prior = list(received)
    received.clear()
    response = starter.finish(Trigger.Request(), Trigger.Response())
    assert response.success
    wait(executor, lambda: len(received) >= 2)
    assert set(received) == {max(starter.end_ns, last_ns)}
    stamps = prior + received
    assert all(a <= b for a, b in zip(stamps, stamps[1:]))
    assert observer.count_publishers('/go2/odom_raw') == 0
