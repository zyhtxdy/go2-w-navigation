"""Filesystem and actual ROS tests. Private localhost domain; no robot APIs."""
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = '109'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
for key in ('CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
            'ROS_DISCOVERY_SERVER', 'ROS_STATIC_PEERS', 'ROS_AUTOMATIC_DISCOVERY_RANGE'):
    os.environ.pop(key, None)

import numpy as np
import pytest
import rclpy
from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from nav_msgs.msg import Odometry, Path as NavigationPath
from terrain_navigation_interfaces.srv import ValidateTerrainPath
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

from go2_offline_navigation.alignment import AlignmentGate
from go2_offline_navigation.map_io import load_map, map_identity, sha256, offline_connection_radius


def test_offline_connection_matches_recorded_diagonal_spacing_without_filling_map():
    planner = dict(surface_connection_radius=0.25, resolution=0.1)
    metadata = dict(terrain_parameters=dict(grid_resolution=0.2))
    assert offline_connection_radius(metadata, planner) == pytest.approx(0.3)
    assert planner['surface_connection_radius'] == 0.25
    assert offline_connection_radius({}, planner) == 0.25
    assert offline_connection_radius(metadata, dict(planner, surface_connection_radius=0.4)) == 0.4
    for bad in (0, -1, math.nan, math.inf):
        with pytest.raises(ValueError):
            offline_connection_radius(dict(terrain_parameters=dict(grid_resolution=bad)), planner)


@pytest.fixture(scope='module')
def maps(tmp_path_factory):
    directory = tmp_path_factory.mktemp('offline_navigation')
    database = directory / 'database'
    (database / 'keyframes').mkdir(parents=True)
    # 14 m includes a tile boundary. This is a software fixture, not a real map.
    points = np.array([(x, y, -.5) for x in np.arange(-1, 14, .1)
                       for y in np.arange(-1.5, 1.6, .1)] +
                      [(x, y, z) for x in np.arange(5.8, 6.3, .1)
                       for y in np.arange(.8, 1.3, .1) for z in np.arange(-.3, .7, .1)])
    def pcd(path):
        with path.open('w') as out:
            out.write('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                      f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n')
            np.savetxt(out, points, fmt='%.9g')
    pcd(database / 'map.pcd')
    pcd(database / 'keyframes/k0.pcd')
    (database / 'manifest.csv').write_text('id,x,y,z,qx,qy,qz,qw\nk0,0,0,0,0,0,0,1\n')
    output = directory / 'navigation'
    executable = Path(get_package_prefix('go2_offline_navigation')) / 'lib/go2_offline_navigation/build_terrain_map'
    subprocess.run([sys.executable, str(executable), str(database), str(output)], check=True,
                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=45)
    return database, output, executable


@pytest.mark.parametrize('legacy_map_parameters', [False, True])
def test_offline_launch_enables_selected_start_connector_without_changing_online_params(
        maps, tmp_path, legacy_map_parameters):
    """Materialize the real launch's private YAML; do not start any nodes."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    import yaml
    database, output, _ = maps
    share = Path(get_package_share_directory('go2_offline_navigation'))
    spec = importlib.util.spec_from_file_location('offline_launch_geometry',
                                                 share / 'launch/offline_nav.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    overrides = dict(database=str(database), map_directory=str(output),
                     start_odometry='false', start_cmd_vel_bridge='false')
    for declaration in module.generate_launch_description().entities:
        if isinstance(declaration, DeclareLaunchArgument):
            context.launch_configurations[declaration.name] = overrides.get(
                declaration.name, perform_substitutions(context, declaration.default_value or []))
    canonical = Path(context.launch_configurations['params_file'])
    if legacy_map_parameters:
        legacy = yaml.safe_load(canonical.read_text())
        plugin = legacy['planner_server']['ros__parameters']['GridBased']
        for key in ('verify_static_map_identity', 'static_map_id_topic', 'expected_map_id',
                    'expected_support_sha256', 'expected_static_obstacles_sha256',
                    'retain_start_projection_bridge', 'reuse_previous_path',
                    'cached_goal_obstacle_grace_period', 'compensate_height_drift'):
            plugin.pop(key)
        canonical = tmp_path / 'legacy_params.yaml'
        canonical.write_text(yaml.safe_dump(legacy))
        context.launch_configurations['params_file'] = str(canonical)
    before = canonical.read_bytes()
    original_rewriter = module.MapRewrittenYaml
    rewritten = []
    def capture(**kwargs):
        value = original_rewriter(**kwargs)
        rewritten.append(value)
        return value
    module.MapRewrittenYaml = capture
    module.start(context)
    assert len(rewritten) == 1
    private_file = Path(rewritten[0].perform(context))
    try:
        values = yaml.safe_load(private_file.read_text())['planner_server']['ros__parameters']['GridBased']
        assert values['retain_start_projection_bridge'] is True
        assert values['use_static_map'] is True
        assert values['verify_static_map_identity'] is True
        identity = map_identity(*load_map(output, database))
        assert values['static_map_id_topic'] == '/offline_map/id'
        assert values['expected_map_id'] == identity['map_id']
        assert values['expected_support_sha256'] == identity['clouds']['support']['data_sha256']
        assert values['expected_static_obstacles_sha256'] == identity['clouds']['obstacles']['data_sha256']
        assert values['surface_connection_radius'] == pytest.approx(0.3)
        assert values['obstacle_topic'] == '/terrain/obstacles_measured_global'
        assert values['support_radius'] == 0.15
        assert canonical.read_bytes() == before
        online = yaml.safe_load(before)['planner_server']['ros__parameters']['GridBased']
        assert online.get('retain_start_projection_bridge', False) is False
        assert online.get('verify_static_map_identity', False) is False
        assert online.get('expected_map_id', '') == ''
        assert online['surface_connection_radius'] == 0.25
        assert online['obstacle_topic'] == '/terrain/obstacles_global'
        local = yaml.safe_load(private_file.read_text())['local_costmap']['local_costmap']['ros__parameters']
        original_local = yaml.safe_load(before)['local_costmap']['local_costmap']['ros__parameters']
        assert local == original_local, 'Offline global input rewrite must preserve local hazard checks'
    finally:
        private_file.unlink(missing_ok=True)


def test_numeric_sha256_values_remain_strings_in_actual_rewritten_yaml(tmp_path):
    from launch import LaunchContext
    import yaml
    share = Path(get_package_share_directory('go2_offline_navigation'))
    spec = importlib.util.spec_from_file_location('offline_launch_numeric_sha',
                                                 share / 'launch/offline_nav.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    source = tmp_path / 'params.yaml'
    hashes = dict(expected_map_id='1' * 64, expected_support_sha256='0' * 64,
                  expected_static_obstacles_sha256='9' * 64)
    source.write_text(yaml.safe_dump(dict(planner=dict(
        **{key: '' for key in hashes}, use_static_map=False))))
    rewritten = Path(module.MapRewrittenYaml(source_file=str(source),
        param_rewrites=dict(hashes, use_static_map='true'), convert_types=True).perform(LaunchContext()))
    try:
        actual = yaml.safe_load(rewritten.read_text())['planner']
        assert actual['use_static_map'] is True
        for key, value in hashes.items():
            assert actual[key] == value
            assert isinstance(actual[key], str)
    finally:
        rewritten.unlink(missing_ok=True)


def test_builder_preserves_frame_coverage_and_observed_heights(maps):
    database, output, _ = maps
    metadata, support, obstacles = load_map(output, database)
    assert metadata['frame_id'] == 'map'
    assert support[:, 0].min() < 0 and support[:, 0].max() > 13
    assert len(support[(support[:, 0] > 8.5) & (support[:, 0] < 9.1)]) > 10
    clear_floor = support[(support[:, 0] < 5) | (support[:, 0] > 7)]
    assert np.max(np.abs(clear_floor[:, 2] + .5)) < .001
    assert np.max(np.abs(support[:, 2] + .5)) < .02
    assert len(obstacles) > 0
    assert metadata['synthetic_obstacle_clearance_radius_used'] == 0


def test_builder_refuses_overwrite(maps):
    database, output, executable = maps
    before = sha256(output / 'navigation_map.json')
    completed = subprocess.run([sys.executable, str(executable), str(database), str(output)],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
    assert completed.returncode != 0
    assert sha256(output / 'navigation_map.json') == before


def test_loader_rejects_changed_database_and_map(maps, tmp_path):
    database, output, _ = maps
    signature = database / 'manifest.csv'
    original = signature.read_bytes()
    try:
        signature.write_bytes(original + b'\n')
        with pytest.raises(ValueError, match='do not match'):
            load_map(output, database)
    finally:
        signature.write_bytes(original)
    import shutil
    shutil.copytree(output, tmp_path / 'bad_map')
    with (tmp_path / 'bad_map/support.csv').open('a') as file:
        file.write('100,100,100\n')
    with pytest.raises(ValueError, match='checksum'):
        load_map(tmp_path / 'bad_map', database)


def test_alignment_requires_new_frames_and_reconfirms_large_jump():
    gate = AlignmentGate()
    value = np.eye(4)
    assert not gate.observe(1, value)
    assert not gate.observe(1, value)
    assert not gate.observe(2, value)
    assert gate.observe(3, value)
    moved = value.copy()
    moved[0, 3] = 3
    assert not gate.observe(4, moved)
    assert gate.accepted is None
    assert not gate.observe(5, moved)
    assert gate.observe(6, moved)
    assert not gate.observe(5, value)
    gate.reset()
    assert gate.accepted is None


def test_launch_preflight_preserves_online_defaults_and_blocks_physical_replay(maps):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    def module(package, filename):
        spec = importlib.util.spec_from_file_location(filename, str(Path(
            get_package_share_directory(package)) / 'launch' / filename))
        result = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(result)
        return result
    online = module('fast_lio_nav2', 'odom_fusion.launch.py').generate_launch_description()
    defaults = {action.name: ''.join(item.text for item in action.default_value)
                for action in online.entities if isinstance(action, DeclareLaunchArgument)}
    assert defaults['publish_map_odom_tf'] == 'true'
    offline = module('go2_offline_navigation', 'offline_nav.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(database=str(maps[0]), map_directory=str(maps[1]),
        use_sim_time='true', start_cmd_vel_bridge='true', start_navigation='true')
    with pytest.raises(ValueError, match='forbidden'):
        offline.start(context)
    context.launch_configurations.update(use_sim_time='false', start_cmd_vel_bridge='false',
        start_navigation='false', start_odometry='false', publish_known_map='false',
        relocalization_params_file=str(Path(get_package_share_directory('go2_relocalization')) /
                                       'config/relocalization.yaml'),
        localization_candidate_threads='2',
        live_obstacle_topic='/terrain/obstacles_measured_global',
        measured_obstacle_decay_time='0.6',
        live_obstacle_freshness_timeout='2.0',
        params_file=str(Path(get_package_share_directory('go2_offline_navigation')) /
                        'config/nav2_mppi_params.yaml'))
    actions = offline.start(context)
    assert len(actions) == 3


def test_ros_latched_maps_alignment_loss_correction_and_velocity_gate(maps):
    database, output, _ = maps
    rclpy.init()
    node = rclpy.create_node('offline_navigation_test_client')
    processes = []
    try:
        prefix = Path(get_package_prefix('go2_offline_navigation')) / 'lib/go2_offline_navigation'
        logfiles = []
        for executable, args in (
                ('offline_map_server', ['--ros-args', '-p', 'map_directory:=' + str(output),
                                        '-p', 'database_directory:=' + str(database)]),
                ('localization_adapter', [])):
            logfile = (output.parent / (executable + '.log')).open('w')
            logfiles.append(logfile)
            processes.append(subprocess.Popen([sys.executable, str(prefix / executable), *args],
                                               stdout=logfile, stderr=subprocess.STDOUT))
        state = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        ready, commands, transforms, reasons = [], [], [], []
        subs = [node.create_subscription(Bool, '/offline_localization/ready', lambda m: ready.append(m.data), state),
                node.create_subscription(String, '/offline_localization/status', lambda m: reasons.append(m.data), state),
                node.create_subscription(Twist, '/go2/offline/cmd_vel', lambda m: commands.append(m), 10),
                node.create_subscription(TFMessage, '/tf', lambda m: transforms.extend(
                    t for t in m.transforms if t.header.frame_id == 'map' and t.child_frame_id == 'odom'), 100)]
        diag = node.create_publisher(DiagnosticArray, '/relocalization/status', state)
        results = node.create_publisher(TransformStamped, '/relocalization/map_to_odom', 10)
        odom_pub = node.create_publisher(Odometry, '/odom', 10)
        lio_pub = node.create_publisher(Odometry, '/lio/odom_base', 10)
        terrain_pub = node.create_publisher(PointCloud2, '/terrain/obstacles', 10)
        planner_health_pub = node.create_publisher(String, '/planner/live_obstacle_status', state)
        cmd_pub = node.create_publisher(Twist, '/go2/offline/cmd_vel_unchecked', 10)
        plan_pub = node.create_publisher(NavigationPath, '/plan', 1)
        validations = []
        def validate(request, response):
            validations.append(request.path)
            response.is_valid = True
            response.status = response.VALID
            return response
        validation_service = node.create_service(ValidateTerrainPath,
            '/terrain/is_path_valid_detailed', validate)
        action_status = node.create_publisher(GoalStatusArray, '/follow_path/_action/status', state)
        broadcaster = TransformBroadcaster(node)
        cancellations = []
        old_goal, old_goal_terminal = None, False
        def cancel(name, request, response):
            cancellations.append(request)
            response.return_code = CancelGoal.Response.ERROR_NONE
            if name == 'follow_path' and old_goal is not None and not old_goal_terminal:
                response.goals_canceling = [old_goal]
            return response
        services = [node.create_service(CancelGoal, '/' + name + '/_action/cancel_goal',
                    lambda request, response, name=name: cancel(name, request, response))
                    for name in ('navigate_to_pose', 'navigate_through_poses', 'follow_path')]
        def spin(duration):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
                assert all(p.poll() is None for p in processes), 'ROS node exited; inspect test logs'
        def wait(predicate, timeout=5):
            end = time.monotonic() + timeout
            while not predicate() and time.monotonic() < end:
                spin(.05)
            assert predicate(), 'Timed out; status=' + str(reasons[-10:])
        terrain_enabled = True
        def inputs():
            stamp = node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
            odom.header.stamp = stamp
            odom.pose.pose.position.x = 1.0
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            lio_pub.publish(odom)
            ekf = TransformStamped()
            ekf.header = odom.header
            ekf.child_frame_id = 'base_link'
            ekf.transform.translation.x = 1.0
            ekf.transform.rotation.w = 1.0
            broadcaster.sendTransform(ekf)
            planner_health_pub.publish(String(data=json.dumps(dict(ready=True,
                consumed_stamp_ns=stamp.sec*1000000000+stamp.nanosec, processed_age_s=0.0))))
            terrain = PointCloud2()
            terrain.header.frame_id, terrain.header.stamp = 'base_link', stamp
            if terrain_enabled:
                terrain_pub.publish(terrain)
            return stamp
        def alignment(x=4, valid=True):
            stamp = inputs()
            message = TransformStamped()
            message.header.frame_id, message.child_frame_id = 'map', 'odom'
            message.header.stamp = stamp
            message.transform.translation.x = float(x)
            message.transform.rotation.w = 1.0
            results.publish(message)
            status = DiagnosticStatus(name='relocalization')
            status.values = [KeyValue(key='valid', value=str(valid).lower()),
                KeyValue(key='last_verified_stamp', value=f'{stamp.sec + stamp.nanosec * 1e-9:.9f}'),
                KeyValue(key='reason', value='tracking' if valid else 'ambiguous_match')]
            diag.publish(DiagnosticArray(status=[status]))
            spin(.12)
        wait(lambda: ready and not ready[-1])
        cmd = Twist()
        cmd.linear.x = .2
        cmd_pub.publish(cmd)
        spin(.2)
        assert not commands and not transforms
        # Map was already published before this subscriber joins.
        spin(.5)
        static_support, static_obstacles = [], []
        subs.extend([node.create_subscription(PointCloud2, '/offline_map/support', static_support.append, state),
                     node.create_subscription(PointCloud2, '/offline_map/obstacles', static_obstacles.append, state)])
        wait(lambda: static_support and static_obstacles)
        assert static_support[-1].width > 0
        for _ in range(5):
            alignment()
        wait(lambda: ready[-1])
        assert transforms and abs(transforms[-1].transform.translation.x - 4) < 1e-9
        inputs(); cmd_pub.publish(cmd); spin(.08)
        assert any(m.linear.x == .2 for m in commands)
        assert not node.get_publishers_info_by_topic('/api/sport/request')
        plan = NavigationPath()
        plan.header.frame_id = 'map'
        plan.poses = [PoseStamped(), PoseStamped()]
        plan.poses[-1].pose.position.x = 2.
        plan_pub.publish(plan)
        spin(.1)
        before_cancel = len(cancellations)
        planner_health_pub.publish(String(data=json.dumps(dict(ready=False,
            consumed_stamp_ns=node.get_clock().now().nanoseconds,
            processed_age_s=0., reason='live_obstacle_processing_stale'))))
        spin(.1)
        assert not ready[-1] and commands[-1].linear.x == 0
        assert len(cancellations) == before_cancel
        for _ in range(3):
            alignment()
        wait(lambda: ready[-1])
        assert validations and validations[-1].poses[-1].pose.position.x == 2.
        assert len(cancellations) == before_cancel
        old_goal = GoalInfo(stamp=node.get_clock().now().to_msg())
        old_goal.goal_id.uuid = [1] * 16
        action_status.publish(GoalStatusArray(status_list=[GoalStatus(
            goal_info=old_goal, status=GoalStatus.STATUS_EXECUTING)]))
        spin(.1)
        alignment(valid=False)
        wait(lambda: not ready[-1] and cancellations)
        assert commands[-1].linear.x == 0
        before = len(commands)
        for _ in range(3):
            inputs(); cmd_pub.publish(cmd); spin(.08)
        assert not any(m.linear.x != 0 for m in commands[before:])
        for _ in range(5):
            alignment()
        assert not ready[-1], 'An ACK alone cannot reopen while the old goal is still active'
        assert any('navigation_waiting_for_goal_terminal' in reason for reason in reasons)
        old_goal_terminal = True
        action_status.publish(GoalStatusArray(status_list=[GoalStatus(
            goal_info=old_goal, status=GoalStatus.STATUS_CANCELED)]))
        wait(lambda: ready[-1])
        inputs(); cmd_pub.publish(cmd); spin(.08)
        alignment(x=7)
        wait(lambda: not ready[-1])
        assert commands[-1].linear.x == 0
        alignment(x=7)
        assert not ready[-1]
        alignment(x=7)
        wait(lambda: ready[-1])
        assert abs(transforms[-1].transform.translation.x - 7) < 1e-9
        # Only the live terrain stops; healthy localization/EKF must remain
        # visible while velocity stays blocked.
        terrain_enabled = False
        transform_count = len(transforms)
        deadline = time.monotonic() + 1.3
        while time.monotonic() < deadline:
            alignment(x=7)
            cmd_pub.publish(cmd)
        wait(lambda: not ready[-1])
        assert len(transforms) > transform_count + 10
        assert commands[-1].linear.x == 0
        blocked_count = len(commands)
        alignment(x=7); cmd_pub.publish(cmd); spin(.08)
        assert not any(m.linear.x != 0 for m in commands[blocked_count:])
        terrain_enabled = True
        # A short terrain outage retains the task, but recovery requires a
        # new successful current-path RPC before subsequent commands may pass.
        validation_count = len(validations)
        alignment(x=7)
        wait(lambda: ready[-1])
        assert len(validations) > validation_count
        inputs(); cmd_pub.publish(cmd); spin(.08)
        spin(.7)
        assert not ready[-1] and commands[-1].linear.x == 0
        assert cancellations and all(not any(request.goal_info.goal_id.uuid) for request in cancellations)
        assert all(request.goal_info.stamp.sec > 0 for request in cancellations)
    finally:
        for process in processes:
            process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        for logfile in locals().get('logfiles', []):
            logfile.close()
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('clearance', [.48, .24])
def test_actual_planner_loads_fixed_map_and_overlays_live_obstacles(maps, clearance):
    """Actual Nav2 planner action server; no controllers or robot command nodes."""
    import yaml
    from action_msgs.msg import GoalStatus
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.action import ComputePathToPose
    from nav2_msgs.srv import IsPathValid
    from rclpy.action import ActionClient
    from tf2_ros import StaticTransformBroadcaster
    from go2_offline_navigation.map_io import cloud_message
    database, output, _ = maps
    parameters = yaml.safe_load((Path(get_package_share_directory('go2_offline_navigation')) /
                                  'config/nav2_mppi_params.yaml').read_text())
    plugin = parameters['planner_server']['ros__parameters']['GridBased']
    plugin.update(use_static_map=True, support_topic='/offline_map/support',
                  static_obstacle_topic='/offline_map/obstacles',
                  compensate_height_drift=False, reuse_previous_path=(clearance == .24),
                  # Pin this fixture's clearance independently from live tuning.
                  inflate_radius=clearance)
    parameter_file = output.parent / 'planner.yaml'
    parameter_file.write_text(yaml.safe_dump(parameters))
    rclpy.init()
    node = rclpy.create_node('static_map_planning_client')
    processes, logfiles = [], []
    try:
        for executable, args in (
                (str(Path(get_package_prefix('go2_offline_navigation')) /
                     'lib/go2_offline_navigation/offline_map_server'),
                 ['--ros-args', '-p', 'map_directory:=' + str(output),
                  '-p', 'database_directory:=' + str(database)]),
                ('/opt/ros/humble/lib/nav2_planner/planner_server',
                 ['--ros-args', '--params-file', str(parameter_file)])):
            logfile = (output.parent / (Path(executable).name + '_planning.log')).open('w')
            logfiles.append(logfile)
            processes.append(subprocess.Popen([executable, *args], stdout=logfile, stderr=subprocess.STDOUT))
        state = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        ready_pub = node.create_publisher(Bool, '/offline_localization/ready', state)
        alignment_pub = node.create_publisher(Bool, '/offline_localization/localization_available', state)
        live_health = []
        health_subscription = node.create_subscription(String, '/planner/live_obstacle_status',
            lambda message: live_health.append(json.loads(message.data)), state)
        map_ready = []
        map_ready_subscription = node.create_subscription(
            Bool, '/offline_map/ready', lambda message: map_ready.append(message.data), state)
        odom_pub = node.create_publisher(Odometry, '/Odometry', 10)
        obstacle_pub = node.create_publisher(PointCloud2, '/terrain/obstacles_global', 10)
        failures = []
        failure_subscription = node.create_subscription(
            String, '/planner/start_failure', lambda message: failures.append(message.data), state)
        broadcaster = StaticTransformBroadcaster(node)
        links = []
        for parent, child in (('map', 'odom'), ('odom', 'base_link'), ('base_link', 'body')):
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = parent, child
            transform.header.stamp = node.get_clock().now().to_msg()
            transform.transform.rotation.w = 1.0
            if child == 'base_link':
                transform.transform.translation.x = 2.0
            links.append(transform)
        broadcaster.sendTransform(links)
        ready = True
        hazards = np.empty((0, 3), dtype=np.float32)
        def pump(duration=.05):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                stamp = node.get_clock().now().to_msg()
                ready_pub.publish(Bool(data=ready))
                alignment_pub.publish(Bool(data=ready))
                odom = Odometry()
                odom.header.frame_id, odom.child_frame_id = 'odom', 'body'
                odom.header.stamp = stamp
                odom.pose.pose.position.x = 2.0
                odom.pose.pose.orientation.w = 1.0
                odom_pub.publish(odom)
                obstacle_pub.publish(cloud_message(hazards, stamp, 'odom'))
                rclpy.spin_once(node, timeout_sec=.02)
                assert all(p.poll() is None for p in processes), 'Planner node exited; inspect logs'
        def await_future(future, timeout=10):
            end = time.monotonic() + timeout
            while not future.done() and time.monotonic() < end:
                pump()
            assert future.done(), 'ROS request timed out'
            return future.result()
        lifecycle = node.create_client(ChangeState, '/planner_server/change_state')
        end = time.monotonic() + 10
        while not lifecycle.service_is_ready() and time.monotonic() < end:
            pump()
        assert lifecycle.service_is_ready()
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            assert await_future(lifecycle.call_async(request)).success
        end = time.monotonic() + 5
        while not map_ready and time.monotonic() < end:
            pump()
        assert map_ready[-1]
        pump(.2)
        action = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        assert action.wait_for_server(timeout_sec=3)
        def plan():
            if ready:
                # The old fixed sleep did not prove successful map consumption.
                # Query only after this actual planner acknowledges a recent input.
                required = node.get_clock().now().nanoseconds - 100000000
                deadline = time.monotonic() + 5
                while not (live_health and live_health[-1]['ready'] and
                           live_health[-1]['consumed_stamp_ns'] >= required) and time.monotonic() < deadline:
                    pump()
                assert live_health and live_health[-1]['ready'] and live_health[-1]['consumed_stamp_ns'] >= required, (
                    live_health[-3:], (output.parent / 'planner_server_planning.log').read_text()[-10000:])
            goal = ComputePathToPose.Goal()
            goal.planner_id, goal.use_start = 'GridBased', True
            for pose, x in ((goal.start, 2.0), (goal.goal, 10.0)):
                pose.header.frame_id = 'map'
                pose.header.stamp = node.get_clock().now().to_msg()
                pose.pose.position.x = x
                pose.pose.orientation.w = 1.0
            handle = await_future(action.send_goal_async(goal))
            assert handle.accepted
            result = await_future(handle.get_result_async())
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                print('Planner refusal evidence:\n' +
                      (output.parent / 'planner_server_planning.log').read_text()[-12000:], flush=True)
            return result
        initial = plan()
        assert initial.status == GoalStatus.STATUS_SUCCEEDED
        assert len(initial.result.path.poses) > 2
        hazards = np.array([[6.0, 0.0, -.1]], dtype=np.float32)
        pump(.4)
        blocked = plan()
        assert blocked.status == GoalStatus.STATUS_SUCCEEDED
        # With full rectangular sweeps, inflate_radius changes soft cost, not
        # the body's hard width. Both settings must clear the same real body.
        minimum_detour = .25
        assert any(abs(p.pose.position.y) > minimum_detour for p in blocked.result.path.poses
                   if abs(p.pose.position.x - 6) < .3)
        # A delayed packet from a previous, later ROS-clock epoch must neither
        # erase the current obstacle window nor poison its timestamp baseline.
        from rclpy.time import Time
        future_stamp = Time(nanoseconds=node.get_clock().now().nanoseconds + 10000000000).to_msg()
        obstacle_pub.publish(cloud_message(np.empty((0, 3)), future_stamp, 'odom'))
        pump(.2)
        after_future = plan()
        assert after_future.status == GoalStatus.STATUS_SUCCEEDED, (output.parent / 'planner_server_planning.log').read_text()[-10000:]
        assert any(abs(p.pose.position.y) > minimum_detour for p in after_future.result.path.poses
                   if abs(p.pose.position.x - 6) < .3)
        hazards = np.empty((0, 3), dtype=np.float32)
        pump(.4)
        cleared = plan()
        assert cleared.status == GoalStatus.STATUS_SUCCEEDED
        assert all(abs(p.pose.position.y) < .2 for p in cleared.result.path.poses
                   if abs(p.pose.position.x - 6) < .3)
        if clearance == .24:
            validity = node.create_client(IsPathValid, '/terrain/is_path_valid')
            assert validity.wait_for_service(timeout_sec=3)
            request = IsPathValid.Request(path=cleared.result.path)
            assert await_future(validity.call_async(request)).is_valid
            # Outside the centre's 0.3 m discretized inflation, inside the front
            # corner. Also inside both historical near-robot grace windows.
            hazards = np.array([[2.36, .22, -.1]], dtype=np.float32)
            pump(.4)
            assert not await_future(validity.call_async(request)).is_valid
            corner_blocked = plan()
            assert corner_blocked.status == GoalStatus.STATUS_ABORTED
            assert not corner_blocked.result.path.poses
            pump(.1)
            assert any('footprint_collision' in message for message in failures)
            assert any('rectangular_footprint' in message for message in failures)
            hazards = np.empty((0, 3), dtype=np.float32)
            pump(.4)
            assert plan().status == GoalStatus.STATUS_SUCCEEDED
        ready = False
        pump(.2)
        lost = plan()
        assert lost.status == GoalStatus.STATUS_ABORTED and not lost.result.path.poses
        # Nothing in this test can drive a robot.
        assert not node.get_publishers_info_by_topic('/api/sport/request')
    finally:
        import signal
        for process in processes:
            process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        for logfile in logfiles:
            logfile.close()
        node.destroy_node()
        rclpy.shutdown()


def test_full_launch_localizes_and_runs_nav2_without_robot_bridge(tmp_path):
    """Complete new entry with synthetic sensors; physical bridge stays absent."""
    import signal
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    from go2_offline_navigation.map_io import cloud_message
    database = tmp_path / 'database'
    (database / 'keyframes').mkdir(parents=True)
    points = []
    def plane(origin, a, b, length_a, length_b):
        for u in np.linspace(0, length_a, math.ceil(length_a/.24)+1):
            for v in np.linspace(0, length_b, math.ceil(length_b/.24)+1):
                points.append(np.array(origin) + np.array(a)*u + np.array(b)*v)
    def box(xmin, xmax, ymin, ymax, zmin, zmax):
        for x in (xmin, xmax):
            plane((x, ymin, zmin), (0,1,0), (0,0,1), ymax-ymin, zmax-zmin)
        for y in (ymin, ymax):
            plane((xmin, y, zmin), (1,0,0), (0,0,1), xmax-xmin, zmax-zmin)
        plane((xmin, ymin, zmax), (1,0,0), (0,1,0), xmax-xmin, ymax-ymin)
    plane((-3.4,-2.3,-.5), (1,0,0), (0,1,0), 7.6, 5.0)
    for y in (-2.3, 2.7):
        plane((-3.4,y,-.5), (1,0,0), (0,0,1), 7.6, 2.3)
    plane((4.2,-2.3,-.5), (0,1,0), (0,0,1), 5.0, 2.3)
    box(.9,1.5,1.1,1.8,-.5,2.1)
    box(-2,-1.3,.2,.9,-.5,.8)
    box(2.4,3.1,-1.6,-.7,-.5,.35)
    points = np.array(points, dtype=np.float32)
    for filename in ('map.pcd', 'keyframes/k0.pcd'):
        with (database / filename).open('w') as file:
            file.write('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                       f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n')
            np.savetxt(file, points, fmt='%.9g')
    (database / 'manifest.csv').write_text('id,x,y,z,qx,qy,qz,qw\nk0,0,0,0,0,0,0,1\n')
    output = tmp_path / 'navigation'
    executable = Path(get_package_prefix('go2_offline_navigation')) / 'lib/go2_offline_navigation/build_terrain_map'
    subprocess.run([sys.executable, str(executable), str(database), str(output)], check=True,
                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
    rclpy.init()
    node = rclpy.create_node('complete_offline_launch_client')
    logfile = (tmp_path / 'full_launch.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'go2_offline_navigation', 'offline_nav.launch.py',
                                'database:=' + str(database), 'map_directory:=' + str(output),
                                'start_odometry:=false', 'start_cmd_vel_bridge:=false'],
                               stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        state = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        ready, reasons, velocities = [], [], []
        subscriptions = [node.create_subscription(Bool, '/offline_localization/ready', lambda m: ready.append(m.data), state),
                         node.create_subscription(String, '/offline_localization/status', lambda m: reasons.append(m.data), state),
                         node.create_subscription(Twist, '/go2/offline/cmd_vel', velocities.append, 10)]
        raw_pub = node.create_publisher(Odometry, '/Odometry', 10)
        lio_pub = node.create_publisher(Odometry, '/lio/odom_base', 10)
        odom_pub = node.create_publisher(Odometry, '/odom', 10)
        cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered_body', 10)
        local_pub = node.create_publisher(PointCloud2, '/terrain/obstacles', 10)
        from sensor_msgs.msg import LaserScan
        clear_pub = node.create_publisher(LaserScan, '/terrain/clear_scan', 10)
        global_pub = node.create_publisher(PointCloud2, '/terrain/obstacles_measured_global', 10)
        static = StaticTransformBroadcaster(node)
        base_body = TransformStamped()
        base_body.header.frame_id, base_body.child_frame_id = 'base_link', 'body'
        base_body.header.stamp = node.get_clock().now().to_msg()
        base_body.transform.translation.x, base_body.transform.translation.z = -.02557, .04232
        base_body.transform.rotation.w = 1.0
        static.sendTransform(base_body)
        ekf_tf = TransformBroadcaster(node)
        def pump(duration=.1):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                stamp = node.get_clock().now().to_msg()
                raw = Odometry()
                raw.header.frame_id, raw.child_frame_id = 'odom', 'body'
                raw.header.stamp = stamp
                raw.pose.pose.orientation.w = 1.0
                raw_pub.publish(raw)
                lio = Odometry()
                lio.header.frame_id, lio.child_frame_id = 'odom', 'base_link'
                lio.header.stamp = stamp
                lio.pose.pose.position.x, lio.pose.pose.position.z = .02557, -.04232
                lio.pose.pose.orientation.w = 1.0
                lio_pub.publish(lio)
                odom = Odometry()
                odom.header, odom.child_frame_id = lio.header, 'base_link'
                odom.pose.pose.position.x = .02557
                odom.pose.pose.orientation.w = 1.0
                odom_pub.publish(odom)
                transform = TransformStamped()
                transform.header, transform.child_frame_id = odom.header, 'base_link'
                transform.transform.translation.x, transform.transform.rotation.w = .02557, 1.0
                ekf_tf.sendTransform(transform)
                cloud_pub.publish(cloud_message(points, stamp, 'body'))
                local_pub.publish(cloud_message(np.empty((0,3)), stamp, 'base_link'))
                # Both real local-costmap inputs have a freshness deadline.
                # Model the clear scan too instead of disabling that gate.
                scan = LaserScan()
                scan.header.frame_id, scan.header.stamp = 'base_link', stamp
                scan.angle_min, scan.angle_max = -math.pi, math.pi
                scan.angle_increment = 2 * math.pi / 63
                scan.range_min, scan.range_max = .05, 3.0
                scan.ranges = [float('inf')] * 64
                clear_pub.publish(scan)
                global_pub.publish(cloud_message(np.empty((0,3)), stamp, 'odom'))
                rclpy.spin_once(node, timeout_sec=.025)
                assert process.poll() is None, 'Full launch exited; inspect full_launch.log'
        def await_future(future, timeout=10):
            end = time.monotonic() + timeout
            while not future.done() and time.monotonic() < end:
                pump()
            assert future.done(), 'Request timed out; ' + str(reasons[-5:])
            return future.result()
        end = time.monotonic() + 30
        while not (ready and ready[-1]) and time.monotonic() < end:
            pump()
        assert ready and ready[-1], 'Real relocalizer did not confirm; ' + str(reasons[-10:])
        for name in ('controller_server', 'planner_server', 'bt_navigator', 'behavior_server', 'velocity_smoother'):
            client = node.create_client(GetState, '/' + name + '/get_state')
            end = time.monotonic() + 10
            activated = False
            while time.monotonic() < end:
                pump()
                if client.service_is_ready():
                    activated = await_future(client.call_async(GetState.Request())).current_state.id == 3
                    if activated:
                        break
            assert activated, name + ' did not activate'
        assert not node.get_publishers_info_by_topic('/api/sport/request')
        action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        assert action.wait_for_server(timeout_sec=3)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = 1.0
        goal.pose.pose.orientation.w = 1.0
        handle = await_future(action.send_goal_async(goal))
        assert handle.accepted
        end = time.monotonic() + 10
        while not any(m.linear.x > 0 or abs(m.angular.z) > 0 for m in velocities) and time.monotonic() < end:
            pump()
        assert any(m.linear.x > 0 or abs(m.angular.z) > 0 for m in velocities), 'No software controller output'
        assert not node.get_publishers_info_by_topic('/api/sport/request')
        await_future(handle.cancel_goal_async())
        pump(.3)
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        logfile.close()
        node.destroy_node()
        rclpy.shutdown()


def test_exported_hazard_diagnostics_preserve_planner_points(maps):
    """Each exported hazard keeps its measured/synthetic provenance separately."""
    import csv
    from collections import Counter
    _, output, _ = maps
    metadata, _, obstacles = load_map(output)
    with (output / 'hazard_sources.csv').open() as stream:
        hazards = list(csv.DictReader(stream))
    assert len(hazards) == len(obstacles)
    np.testing.assert_array_equal(
        np.asarray([[row[axis] for axis in ('x', 'y', 'z')] for row in hazards], dtype=np.float32),
        obstacles)
    counts = Counter(row['source'] for row in hazards)
    assert set(counts) <= {'measured', 'synthetic'}
    assert metadata['hazard_sources'] == {key: counts[key] for key in ('measured', 'synthetic')}
    with (output / 'terrain_tiles.csv').open() as stream:
        tiles = list(csv.DictReader(stream))
    assert sum(int(row['owned_support_points']) for row in tiles) == metadata['files']['support.csv']['points']
    assert sum(int(row['owned_measured_hazards']) for row in tiles) == counts['measured']
    assert sum(int(row['owned_synthetic_hazards']) for row in tiles) == counts['synthetic']
    reasons = ('insufficient_fit_cells', 'degenerate_fit_cells', 'insufficient_support_cells',
               'slope_exceeded_cells', 'rmse_exceeded_cells', 'step_exceeded_cells', 'traversable_cells')
    assert all(sum(int(row[key]) for key in reasons) == int(row['observed_cells']) for row in tiles)


@pytest.mark.parametrize('diagnostic_name', [
    'hazard_sources.csv', 'terrain_tiles.csv', 'terrain_cells.csv'])
def test_map_diagnostic_corruption_is_rejected(maps, tmp_path, diagnostic_name):
    import shutil
    _, output, _ = maps
    broken = tmp_path / 'diagnostics_corrupted'
    shutil.copytree(output, broken)
    with (broken / diagnostic_name).open('a') as stream:
        stream.write('corrupt\n')
    with pytest.raises(ValueError, match='diagnostics checksum mismatch'):
        load_map(broken)


def test_per_cell_export_explains_support_and_synthetic_hazards(maps):
    import csv
    _, output, _ = maps
    metadata, support, _ = load_map(output)
    assert metadata['sparse_ground_layer_recovery_used'] is False
    with (output / 'terrain_cells.csv').open() as stream:
        cells = list(csv.DictReader(stream))
    positions = {(float(cell['x']), float(cell['y'])) for cell in cells}
    assert len(positions) == len(cells), 'Fitting halos must not duplicate exported cells'
    assert sum(cell['outcome'] == 'traversable' for cell in cells) == len(support)
    assert sum(cell['synthetic_hazard'] == '1' for cell in cells) == metadata['hazard_sources']['synthetic']
    for cell in cells:
        assert int(cell['returns']) > 0
        assert float(cell['minimum_z']) <= float(cell['maximum_z'])
        if cell['outcome'] == 'traversable':
            assert cell['fitted'] == cell['observed_ground'] == '1'
