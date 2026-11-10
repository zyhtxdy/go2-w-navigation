"""Exercise near-wall detection and evidence-limited clearing in the real node."""

import math
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
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster


@pytest.mark.parametrize('offset_frames', [False, True])
def test_near_wall_without_ground_and_current_frame_clearing(tmp_path, monkeypatch, offset_frames):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(210 + os.getpid() % 20))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros_logs'))
    for name in ('CYCLONEDDS_URI', 'ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE'):
        monkeypatch.delenv(name, raising=False)

    rclpy.init()
    node = rclpy.create_node('near_obstacle_node_test')
    received = {name: [] for name in ('local', 'measured', 'stable', 'support', 'scan')}
    subscriptions = [
        node.create_subscription(
            LaserScan if name == 'scan' else PointCloud2,
            '/near_obstacle_test/'+name, received[name].append, qos_profile_sensor_data)
        for name in received
    ]
    clouds = node.create_publisher(
        PointCloud2, '/near_obstacle_test/cloud', qos_profile_sensor_data)
    odometry = node.create_publisher(Odometry, '/near_obstacle_test/odom', 20)
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'body'
    base_yaw = math.pi / 2 if offset_frames else 0.
    base_offset = (.10, -.04, .04232) if offset_frames else (0., 0., 0.)
    body_yaw = -.3 if offset_frames else 0.
    body_position = (1., -2., .7) if offset_frames else (0., 0., .5)
    transform.transform.translation.x = base_offset[0]
    transform.transform.translation.y = base_offset[1]
    transform.transform.translation.z = base_offset[2]
    transform.transform.rotation.z = math.sin(base_yaw / 2)
    transform.transform.rotation.w = math.cos(base_yaw / 2)
    broadcaster.sendTransform(transform)

    parameters = {
        'cloud_topic': '/near_obstacle_test/cloud',
        'odometry_topic': '/near_obstacle_test/odom',
        'obstacle_topic': '/near_obstacle_test/local',
        'measured_global_obstacle_topic': '/near_obstacle_test/measured',
        'global_obstacle_topic': '/near_obstacle_test/stable',
        'support_topic': '/near_obstacle_test/support',
        'clearing_scan_topic': '/near_obstacle_test/scan',
        'near_obstacle_detection_enabled': 'true',
        'maximum_slope_degrees': '10.0',
        'history_decay_time': '2.0',
        'processing_interval': '0.01',
        'terrain_publish_interval': '0.01',
        'pose_wait_timeout': '0.1',
        'clearing_endpoint_margin': '0.10',
    }
    command = [os.environ['TERRAIN_NODE_EXECUTABLE'], '--ros-args']
    for key, value in parameters.items():
        command.extend(['-p', f'{key}:={value}'])
    log_path = tmp_path/'node.log'
    log = log_path.open('w')
    process = None

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)

    def wait_for(predicate, description, seconds=6):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate():
                return
            if process.poll() is not None:
                break
        raise AssertionError(f'{description}\n{log_path.read_text()}')

    def body_point(point):
        x, y, z = (point[i] - base_offset[i] for i in range(3))
        return (math.cos(base_yaw)*x + math.sin(base_yaw)*y,
                -math.sin(base_yaw)*x + math.cos(base_yaw)*y, z)

    def odom_point(point):
        x, y, z = body_point(point)
        return (body_position[0] + math.cos(body_yaw)*x - math.sin(body_yaw)*y,
                body_position[1] + math.sin(body_yaw)*x + math.cos(body_yaw)*y,
                body_position[2] + z)

    def send(frame, points, epoch=100, moved=False):
        pose = Odometry()
        pose.header.frame_id, pose.child_frame_id = 'odom', 'body'
        pose.header.stamp.sec, pose.header.stamp.nanosec = epoch, frame * 100000000
        pose.pose.pose.orientation.z = math.sin(body_yaw / 2)
        pose.pose.pose.orientation.w = math.cos(body_yaw / 2)
        pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z = body_position
        if moved:
            pose.pose.pose.position.x += .30
            pose.pose.pose.position.y -= .20
        odometry.publish(pose)
        spin(.03)

        cloud = PointCloud2()
        cloud.header.frame_id, cloud.header.stamp = 'body', pose.header.stamp
        cloud.height, cloud.width = 1, len(points)
        cloud.fields = [
            PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
            for index, name in enumerate(('x', 'y', 'z'))
        ]
        cloud.point_step, cloud.row_step = 12, 12 * len(points)
        cloud.data = b''.join(struct.pack('<fff', *body_point(point)) for point in points)
        cloud.is_dense = True
        clouds.publish(cloud)
        wait_for(lambda: all(messages and messages[-1].header.stamp == cloud.header.stamp
                             for messages in received.values()), f'Waiting for frame {frame}')
        return {name: messages[-1] for name, messages in received.items()}

    def scan_bin(scan, x, y):
        return math.floor((math.atan2(y, x) - scan.angle_min) / scan.angle_increment)

    def assert_no_ground_fit(messages):
        # Real near-wall evidence must not become invented ground/support.
        assert messages['support'].width == 0

    def assert_empty_frame(messages, retained_count):
        assert messages['local'].width == 0
        assert messages['measured'].width == retained_count
        assert_no_ground_fit(messages)
        assert messages['scan'].ranges
        assert all(math.isnan(value) for value in messages['scan'].ranges)

    try:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        wait_for(lambda: clouds.get_subscription_count() and odometry.get_subscription_count()
                 and all(node.count_publishers('/near_obstacle_test/'+name) for name in received),
                 'Waiting for node discovery')
        spin(.3)  # Allow the transient-local base/body transform to reach the process.

        # Two collinear XY columns cannot supply a ground-plane fit. Each has
        # actual lower returns and upper returns outside the configured body box.
        wall = [(x, .55, z) for x in (.2, .3) for z in (.01, .09, .21, .31)]
        first = send(0, wall)
        assert_no_ground_fit(first)
        assert first['local'].width >= 3
        global_count = first['measured'].width
        assert global_count == first['local'].width
        assert first['measured'].header.frame_id == 'odom'
        assert first['stable'].width == 0  # Online mixed stream still needs two real frames.
        assert first['local'].header.frame_id == 'base_link'
        fields = {field.name: field.offset for field in first['local'].fields}
        for offset in range(0, first['local'].row_step, first['local'].point_step):
            x = struct.unpack_from('<f', first['local'].data, offset + fields['x'])[0]
            y = struct.unpack_from('<f', first['local'].data, offset + fields['y'])[0]
            assert min(abs(x - candidate) for candidate in (.2, .3)) < 1e-6
            assert abs(y - .55) < 1e-6

        def assert_source_evidence(message, nanosecond):
            fields = {field.name: field.offset for field in message.fields}
            expected = [odom_point(p) for p in wall if p[2] >= .2]
            for offset in range(0, message.row_step, message.point_step):
                xyz = [struct.unpack_from('<f', message.data, offset+fields[key])[0]
                       for key in ('x', 'y', 'z')]
                # Global XYZ is a transformed actual return, never local relative-height Z.
                assert any(math.dist(xyz, point) < 1e-6 for point in expected)
                assert message.data[offset+fields['obstacle_source']] == 1
                assert struct.unpack_from('<i', message.data,
                                          offset+fields['observation_sec'])[0] == 100
                assert struct.unpack_from('<I', message.data,
                                          offset+fields['observation_nanosec'])[0] == nanosecond

        assert_source_evidence(first['measured'], 0)

        scan = first['scan']
        wall_bins = {scan_bin(scan, x, .55) for x in (.2, .3)}
        assert {index for index, value in enumerate(scan.ranges) if math.isfinite(value)} == wall_bins
        for x in (.2, .3):
            value = scan.ranges[scan_bin(scan, x, .55)]
            assert scan.range_min <= value < math.hypot(x, .55)
            assert abs(value - (math.hypot(x, .55) - .10)) < 1e-5
        assert all(math.isnan(value) for index, value in enumerate(scan.ranges)
                   if index not in wall_bins)

        # Global evidence gets the established 0.6s retention using its source
        # time. Local near marking and clearing still require this actual scan.
        retained = send(1, [], moved=True)
        assert_empty_frame(retained, global_count)
        # Old detections stay at their observed world position when the body
        # moves; neither current pose nor current cloud header may reattach them.
        assert_source_evidence(retained['measured'], 0)
        assert retained['stable'].width == 0  # Replaying old evidence cannot pay the entry gate.

        # An actual farther ground return proves this ray clear beyond the old
        # wall. It lies outside the 5m fit grid, so cannot complete a ground fit.
        far = (2.0, 5.5, -.5)
        cleared = send(2, [far])
        assert_no_ground_fit(cleared)
        assert cleared['local'].width == 0
        assert_source_evidence(cleared['measured'], 0)
        scan = cleared['scan']
        index = scan_bin(scan, far[0], far[1])
        assert index in wall_bins
        assert scan.ranges[index] > math.hypot(.2, .55)
        assert abs(scan.ranges[index] - (math.hypot(*far[:2]) - .10)) < 1e-5
        assert all(math.isnan(value) for other, value in enumerate(scan.ranges) if other != index)
        assert_empty_frame(send(6, []), global_count)  # Exact 0.6s boundary is inclusive.
        assert_empty_frame(send(7, []), 0)  # The remaining 2s history cannot revive it.
        renewed = send(8, wall)
        assert_source_evidence(renewed['measured'], 800000000)
        confirmed = send(9, wall)
        assert confirmed['stable'].width > 0
        rewound = send(0, [], epoch=90)
        assert_empty_frame(rewound, 0)
        assert rewound['stable'].width == 0
    finally:
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
