"""Exercise actual terrain-node timing with delayed and stale odometry."""

import os
import signal
import struct
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster


@pytest.mark.parametrize('wait_seconds,sim_time', [(0.10, False), (0.10, True), (0.0, False)])
def test_delayed_pose_and_expiration(tmp_path, monkeypatch, wait_seconds, sim_time):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(160 + os.getpid() % 20))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros_logs'))
    for name in ('CYCLONEDDS_URI', 'ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE'):
        monkeypatch.delenv(name, raising=False)
    rclpy.init()
    node = rclpy.create_node('pose_waiting_test')
    received = []
    node.create_subscription(
        PointCloud2, '/pose_wait_test/support', received.append, qos_profile_sensor_data)
    clouds = node.create_publisher(PointCloud2, '/pose_wait_test/cloud', qos_profile_sensor_data)
    odometry = node.create_publisher(Odometry, '/pose_wait_test/odom', 20)
    clock = node.create_publisher(Clock, '/clock', 10)
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'body'
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)
    executable = os.environ['TERRAIN_NODE_EXECUTABLE']
    parameters = {
        'cloud_topic': '/pose_wait_test/cloud', 'odometry_topic': '/pose_wait_test/odom',
        'support_topic': '/pose_wait_test/support', 'use_sim_time': str(sim_time).lower(),
        'pose_wait_timeout': str(wait_seconds), 'maximum_pose_age': '0.20',
        'maximum_slope_degrees': '10.0', 'maximum_step_height': '0.08',
    }
    command = [executable, '--ros-args']
    for key, value in parameters.items():
        command.extend(['-p', f'{key}:={value}'])
    output = (tmp_path/'node.log').open('w')
    process = None

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.005)

    def wait_for(predicate, seconds=5):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.005)
            if predicate():
                return
        raise AssertionError((tmp_path/'node.log').read_text())

    def set_stamp(stamp, value):
        stamp.sec = int(value)
        stamp.nanosec = round((value - int(value)) * 1e9)

    def publish_odom(value):
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'body'
        set_stamp(msg.header.stamp, value)
        msg.pose.pose.position.z = 0.5
        msg.pose.pose.orientation.w = 1.0
        odometry.publish(msg)

    def publish_cloud(value):
        msg = PointCloud2()
        msg.header.frame_id = 'body'
        set_stamp(msg.header.stamp, value)
        points = [(x*.20, y*.20, -.5) for x in range(5, 11) for y in range(-3, 4)]
        msg.height, msg.width = 1, len(points)
        msg.point_step, msg.row_step = 12, len(points)*12
        msg.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z'))]
        msg.data = b''.join(struct.pack('<fff', *p) for p in points)
        msg.is_dense = True
        clouds.publish(msg)

    try:
        process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        wait_for(lambda: clouds.get_subscription_count() and odometry.get_subscription_count())
        spin(.3)
        # /clock is published once then remains stopped, as at bag end.
        tick = Clock()
        tick.clock.sec = 100
        clock.publish(tick)
        spin(.1)
        publish_odom(100.0 - .201)
        spin(.05)
        publish_cloud(100.0)
        spin(.04)
        assert not received
        publish_odom(100.0)
        if wait_seconds:
            wait_for(lambda: len(received) == 1)
            assert received[0].header.stamp.sec == 100
            assert received[0].width > 0
        else:
            spin(.15)
            assert not received

        # Waiting must not accept a pose outside the original 0.20 s limit.
        before = len(received)
        publish_odom(110.0 - .201)
        spin(.05)
        publish_cloud(110.0)
        spin(.20)
        assert len(received) == before
        assert 'Skipping terrain cloud' in (tmp_path/'node.log').read_text()

        # A timed-out cloud must not prevent processing a later valid frame.
        publish_odom(111.0)
        spin(.05)
        publish_cloud(111.0)
        wait_for(lambda: len(received) == before + 1)
        assert received[-1].header.stamp.sec == 111
        assert received[-1].width > 0
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        output.close()
        node.destroy_node()
        rclpy.shutdown()
