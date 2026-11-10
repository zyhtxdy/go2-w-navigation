"""Verify actual node output separates ground history from obstacle freshness."""
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
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster


@pytest.mark.parametrize('terrain_publish_interval', [.01, 10.0])
def test_measured_freshness_and_local_history(tmp_path, monkeypatch, terrain_publish_interval):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(190 + os.getpid() % 20))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros_logs'))
    for name in ('CYCLONEDDS_URI', 'ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE'):
        monkeypatch.delenv(name, raising=False)
    rclpy.init()
    node = rclpy.create_node('obstacle_time_node_test')
    received = {name: [] for name in ('measured', 'local', 'support')}
    for name in received:
        node.create_subscription(PointCloud2, '/obstacle_time/'+name,
                                 received[name].append, qos_profile_sensor_data)
    clouds = node.create_publisher(PointCloud2, '/obstacle_time/cloud', qos_profile_sensor_data)
    odom = node.create_publisher(Odometry, '/obstacle_time/odom', 20)
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'body'
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)
    params = {
        'cloud_topic': '/obstacle_time/cloud', 'odometry_topic': '/obstacle_time/odom',
        'measured_global_obstacle_topic': '/obstacle_time/measured',
        'obstacle_topic': '/obstacle_time/local', 'support_topic': '/obstacle_time/support',
        'history_decay_time': '2.0', 'measured_obstacle_decay_time': '0.6',
        'processing_interval': '0.01', 'terrain_publish_interval': str(terrain_publish_interval),
        'pose_wait_timeout': '0.1',
    }
    command = [os.environ['TERRAIN_NODE_EXECUTABLE'], '--ros-args']
    for key, value in params.items():
        command.extend(['-p', key+':='+value])
    log = (tmp_path/'node.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)

    def wait(predicate, seconds=5):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate():
                return
        raise AssertionError((tmp_path/'node.log').read_text())

    def send(sec, nsec=0, obstacle=False, sparse=False):
        pose = Odometry()
        pose.header.frame_id, pose.child_frame_id = 'odom', 'body'
        pose.header.stamp.sec, pose.header.stamp.nanosec = sec, nsec
        pose.pose.pose.orientation.w, pose.pose.pose.position.z = 1.0, .5
        odom.publish(pose)
        msg = PointCloud2()
        msg.header.frame_id = 'body'
        msg.header.stamp = pose.header.stamp
        points = [(x*.2, y*.2, -.5) for x in range(4, 13) for y in range(-4, 5)]
        if sparse:
            points = [points[0]]
        if obstacle:
            points += [(1.23, y, .2) for y in (-.23, 0., .23)]
        msg.height, msg.width = 1, len(points)
        msg.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z'))]
        msg.point_step, msg.row_step = 12, 12*len(points)
        msg.data = b''.join(struct.pack('<fff', *p) for p in points)
        clouds.publish(msg)
        wait(lambda: received['measured'] and
             received['measured'][-1].header.stamp == msg.header.stamp)
        wait(lambda: received['local'] and received['local'][-1].header.stamp == msg.header.stamp)

    try:
        wait(lambda: clouds.get_subscription_count() and odom.get_subscription_count())
        # Let the transient-local static transform reach the terrain process.
        end = time.monotonic()+.3
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
        send(100, obstacle=True)
        first = received['measured'][-1]
        assert first.width >= 1  # A new obstacle enters immediately, without a two-frame delay.
        fields = {f.name: f for f in first.fields}
        assert fields['observation_sec'].datatype == PointField.INT32
        assert fields['observation_nanosec'].datatype == PointField.UINT32
        for offset in range(0, first.row_step, first.point_step):
            assert struct.unpack_from('<i', first.data, offset+fields['observation_sec'].offset)[0] == 100
            assert struct.unpack_from('<I', first.data, offset+fields['observation_nanosec'].offset)[0] == 0
        send(100, 300000000)
        assert received['measured'][-1].width >= 1
        # Analysis time advances, but retained old points still carry their original time.
        msg = received['measured'][-1]
        assert struct.unpack_from('<i', msg.data, fields['observation_sec'].offset)[0] == 100
        assert struct.unpack_from('<I', msg.data, fields['observation_nanosec'].offset)[0] == 0
        send(100, 700000000, sparse=True)
        assert received['measured'][-1].width == 0
        assert received['local'][-1].width >= 1  # Local hazards keep the full ground-fit history.
        assert received['support'][-1].width > 20  # Fit still uses the older floor, not just one new point.
        if terrain_publish_interval == 10.0:
            # Measured replacement must not wait behind throttled support publication.
            assert received['support'][-1].header.stamp.sec == 100
            assert received['support'][-1].header.stamp.nanosec == 0
        send(90)
        assert received['measured'][-1].width == 0
        assert received['local'][-1].width == 0  # No previous epoch's future obstacles survive.
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()
