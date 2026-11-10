"""Actual saved lab map + terrain node + planner; no motion/controller nodes."""

import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import numpy as np
import pytest
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String
from terrain_navigation_interfaces.srv import ValidateTerrainPath
from tf2_ros import StaticTransformBroadcaster

from go2_offline_navigation.map_io import load_map, offline_planner_settings


def test_saved_map_first_plan_avoids_new_near_wall(tmp_path, monkeypatch):
    root = Path(__file__).resolve().parents[3]
    database = root/'maps/lab_v1_database'
    directory = root/'maps/lab_v1_navigation_guarded_20261026'
    if not directory.exists() or not database.exists():
        pytest.skip('Optional recorded lab map/database are unavailable')
    monkeypatch.setenv('ROS_DOMAIN_ID', str(120 + os.getpid() % 10))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros_logs'))
    for key in ('CYCLONEDDS_URI', 'ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                'FASTDDS_DEFAULT_PROFILES_FILE', 'ROS_STATIC_PEERS'):
        monkeypatch.delenv(key, raising=False)
    metadata, support, obstacles = load_map(directory, database)
    params = yaml.safe_load((root/'src/nav2/navigation_bringup/config/nav2_mppi_params.yaml').read_text())
    planner = params['planner_server']['ros__parameters']['GridBased']
    planner.update(offline_planner_settings(metadata, support, obstacles, planner,
                   live_obstacle_topic='/near_planning/measured'))
    planner.update(lio_odom_topic='/near_planning/odom')
    (tmp_path/'planner.yaml').write_text(yaml.safe_dump(params))
    terrain_params = yaml.safe_load((root/'src/nav2/terrain_costmap/config/terrain_costmap.yaml').read_text())
    terrain_params['terrain_costmap']['ros__parameters'].update(
        cloud_topic='/near_planning/cloud', odometry_topic='/near_planning/odom',
        measured_global_obstacle_topic='/near_planning/measured',
        support_topic='/near_planning/support', obstacle_topic='/near_planning/local',
        processing_interval=.05, terrain_publish_interval=.05)
    (tmp_path/'terrain.yaml').write_text(yaml.safe_dump(terrain_params))

    rclpy.init()
    node = rclpy.create_node('near_saved_map_planning_test')
    processes, logfiles = [], []
    retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    ready = node.create_publisher(Bool, '/offline_localization/planning_ready', retained)
    alignment = node.create_publisher(Bool, '/offline_localization/localization_available', retained)
    clouds = node.create_publisher(PointCloud2, '/near_planning/cloud', qos_profile_sensor_data)
    odometry = node.create_publisher(Odometry, '/near_planning/odom', 20)
    health, measured, ground = [], [], []
    node.create_subscription(String, '/planner/live_obstacle_status',
                             lambda msg: health.append(json.loads(msg.data)), retained)
    node.create_subscription(PointCloud2, '/near_planning/measured', measured.append,
                             qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/near_planning/support', ground.append,
                             qos_profile_sensor_data)
    broadcaster = StaticTransformBroadcaster(node)
    base_x, base_y, body_z = 2., -.4, .12
    transforms = []
    for parent, child in (('map', 'odom'), ('odom', 'base_link'), ('base_link', 'body')):
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = parent, child
        transform.transform.rotation.w = 1.
        if child == 'base_link':
            transform.transform.translation.x = base_x
            transform.transform.translation.y = base_y
            transform.transform.translation.z = body_z
        transforms.append(transform)
    broadcaster.sendTransform(transforms)
    # Two columns, no ground returns: this obstacle must use the near branch.
    wall = [(x, 0., z) for x in (.67, .77) for z in (-.2, -.12, .02, .12)]
    active_wall = False

    def launch(command, name):
        logfile = (tmp_path/(name+'.log')).open('w')
        logfiles.append(logfile)
        processes.append(subprocess.Popen(command, stdout=logfile, stderr=subprocess.STDOUT,
                                          start_new_session=True))

    def evidence():
        return '\n'.join(p.read_text(errors='replace')[-12000:] for p in tmp_path.glob('*.log'))

    def pump(seconds=.05):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            assert all(p.poll() is None for p in processes), evidence()
            stamp = node.get_clock().now().to_msg()
            ready.publish(Bool(data=True))
            alignment.publish(Bool(data=True))
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'body'
            odom.header.stamp = stamp
            odom.pose.pose.position.x, odom.pose.pose.position.y = base_x, base_y
            odom.pose.pose.position.z, odom.pose.pose.orientation.w = body_z, 1.
            odometry.publish(odom)
            cloud = PointCloud2()
            cloud.header.frame_id, cloud.header.stamp = 'body', stamp
            points = wall if active_wall else []
            cloud.height, cloud.width = 1, len(points)
            cloud.fields = [PointField(name=key, offset=i*4, datatype=PointField.FLOAT32, count=1)
                            for i, key in enumerate(('x', 'y', 'z'))]
            cloud.point_step, cloud.row_step = 12, len(points)*12
            cloud.data = b''.join(struct.pack('<fff', *p) for p in points)
            clouds.publish(cloud)
            rclpy.spin_once(node, timeout_sec=.02)

    def wait_for(predicate, seconds=10):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            pump()
            if predicate():
                return
        raise AssertionError(evidence())

    def wait_future(future):
        wait_for(future.done)
        return future.result()

    def plan(action):
        required = node.get_clock().now().nanoseconds-100000000
        wait_for(lambda: health and health[-1]['ready'] and
                 health[-1]['consumed_stamp_ns'] >= required)
        request = ComputePathToPose.Goal()
        request.planner_id, request.use_start = 'GridBased', True
        for pose, x in ((request.start, base_x), (request.goal, 4.5)):
            pose.header.frame_id = 'map'
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = x, base_y
            pose.pose.orientation.w = 1.
        handle = wait_future(action.send_goal_async(request))
        assert handle.accepted
        result = wait_future(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_SUCCEEDED, evidence()
        return result.result.path

    lifecycle = None
    activated = False
    try:
        terrain_executable = os.environ.get('TERRAIN_NODE_EXECUTABLE', str(
            Path(get_package_prefix('terrain_costmap'))/'lib/terrain_costmap/terrain_costmap_node'))
        launch([terrain_executable, '--ros-args', '--params-file', str(tmp_path/'terrain.yaml')],
               'terrain')
        launch([str(Path(get_package_prefix('go2_offline_navigation'))/
                    'lib/go2_offline_navigation/offline_map_server'), '--ros-args',
                '-p', 'map_directory:='+str(directory), '-p', 'database_directory:='+str(database)],
               'fixed_map')
        launch(['/opt/ros/humble/lib/nav2_planner/planner_server', '--ros-args',
                '--params-file', str(tmp_path/'planner.yaml')], 'planner')
        lifecycle = node.create_client(ChangeState, '/planner_server/change_state')
        wait_for(lifecycle.service_is_ready)
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            assert wait_future(lifecycle.call_async(request)).success, evidence()
        activated = True
        action = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        wait_for(action.server_is_ready)
        baseline = plan(action)
        assert len(baseline.poses) > 2
        assert all(abs(p.pose.position.y-base_y) < .05 for p in baseline.poses), evidence()

        # It is placed BEFORE this new planning request, absent from the fixed map.
        active_wall = True
        wait_for(lambda: measured and measured[-1].width > 0)
        pump(.2)
        strict = node.create_client(ValidateTerrainPath, '/terrain/is_path_valid_detailed')
        reference = node.create_client(ValidateTerrainPath, '/terrain/is_path_valid_reference')
        wait_for(lambda: strict.service_is_ready() and reference.service_is_ready())
        validation = ValidateTerrainPath.Request(path=baseline)
        strict_result = wait_future(strict.call_async(validation))
        assert not strict_result.is_valid and strict_result.status == strict_result.COLLISION, evidence()
        # Preserve the established policy for an ALREADY published reference:
        # MPPI may temporarily detour while the current robot body is clear.
        reference_result = wait_future(reference.call_async(validation))
        assert reference_result.is_valid, (reference_result.reason, evidence())
        avoided = plan(action)
        assert ground and all(msg.width == 0 for msg in ground), 'Near wall invented support'
        assert any(abs(p.pose.position.y-base_y) > .25 for p in avoided.poses), evidence()
        # Independent planar rectangle check along each translation, using the
        # global padded half-extents. Do not infer safety from centre distance.
        obstacle_xy = np.array([(base_x+x, base_y) for x in (.67, .77)])
        for first, last in zip(avoided.poses, avoided.poses[1:]):
            a, b = first.pose.position, last.pose.position
            length = math.hypot(b.x-a.x, b.y-a.y)
            if length < 1e-8:
                continue
            yaw = math.atan2(b.y-a.y, b.x-a.x)
            for fraction in np.linspace(0., 1., max(2, math.ceil(length/.01)+1)):
                centre = np.array([a.x+(b.x-a.x)*fraction, a.y+(b.y-a.y)*fraction])
                delta = obstacle_xy-centre
                local_x = math.cos(yaw)*delta[:, 0]+math.sin(yaw)*delta[:, 1]
                local_y = -math.sin(yaw)*delta[:, 0]+math.cos(yaw)*delta[:, 1]
                assert not np.any((abs(local_x) < .41-1e-5) & (abs(local_y) < .26-1e-5)), evidence()

        active_wall = False
        wait_for(lambda: measured and measured[-1].width == 0)
        restored = plan(action)
        assert all(abs(p.pose.position.y-base_y) < .05 for p in restored.poses), evidence()
        (tmp_path/'paths.json').write_text(json.dumps({name: [
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z] for pose in path.poses]
            for name, path in (('baseline', baseline), ('avoided', avoided), ('restored', restored))}))
    finally:
        if activated and lifecycle is not None:
            for transition in (4, 2):
                request = ChangeState.Request()
                request.transition.id = transition
                try:
                    wait_future(lifecycle.call_async(request))
                except Exception:
                    break
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        for logfile in logfiles:
            logfile.close()
        node.destroy_node()
        rclpy.shutdown()
