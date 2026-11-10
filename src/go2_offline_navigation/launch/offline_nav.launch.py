"""Independent known-map entry. Existing online launch defaults are unchanged."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import yaml


class MapRewrittenYaml(RewrittenYaml):
    """Keep SHA-256 values textual even when every hexadecimal digit is numeric."""
    def convert(self, text):
        if isinstance(text, str) and len(text) == 64 and all(
                character in '0123456789abcdefABCDEF' for character in text):
            return text
        return super().convert(text)

    def substitute_params(self, data, param_rewrites):
        super().substitute_params(data, param_rewrites)
        # Humble only rewrites existing keys. Materialize the complete offline
        # overlay in older custom files too, matching the replay evaluator.
        planner = data.get('planner_server', {}).get('ros__parameters', {}).get('GridBased')
        if isinstance(planner, dict):
            for key, value in param_rewrites.items():
                planner[key] = self.convert(value)


def start(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        result = value(name).lower()
        if result not in ('true', 'false'):
            raise ValueError(name + ' must be true or false')
        return result == 'true'

    from go2_offline_navigation.map_io import load_map, offline_planner_settings
    database = Path(value('database')).expanduser().resolve()
    map_directory = Path(value('map_directory')).expanduser().resolve()
    # Fail before any node starts if maps, frames or database identity differ.
    metadata, support, obstacles = load_map(map_directory, database)
    use_sim_time = flag('use_sim_time')
    if use_sim_time and flag('start_cmd_vel_bridge'):
        raise ValueError('Physical command bridge is forbidden with use_sim_time=true')
    if flag('start_cmd_vel_bridge') and not flag('start_navigation'):
        raise ValueError('Command bridge requires start_navigation=true')
    localization = Path(get_package_share_directory('go2_relocalization'))
    planner = yaml.safe_load(Path(value('params_file')).read_text())['planner_server']['ros__parameters']['GridBased']
    if 'use_static_map' not in planner or 'static_obstacle_topic' not in planner:
        raise ValueError('Offline params must include GridBased.use_static_map and static_obstacle_topic')
    params = MapRewrittenYaml(
        source_file=value('params_file'),
        param_rewrites={key: str(value) for key, value in offline_planner_settings(
            metadata, support, obstacles, planner,
            live_obstacle_topic=value('live_obstacle_topic'),
            measured_obstacle_decay_time=value('measured_obstacle_decay_time'),
            live_obstacle_freshness_timeout=value('live_obstacle_freshness_timeout')).items()}, convert_types=True)
    actions = []
    if flag('start_odometry'):
        actions.append(GroupAction(actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('fast_lio_nav2')) / 'launch/odom_fusion.launch.py')),
            launch_arguments={
                'publish_map_odom_tf': 'false', 'use_sim_time': value('use_sim_time'),
                'lidar_driver': value('lidar_driver'),
                'measured_obstacle_decay_time': value('measured_obstacle_decay_time'),
                'start_go2_imu_bridge': value('start_go2_imu_bridge'),
                'start_go2_odom_bridge': value('start_go2_odom_bridge'),
                'imu_topic': value('imu_topic'), 'lowstate_topic': value('lowstate_topic'),
                'sportmodestate_topic': value('sportmodestate_topic'),
            }.items())]))
    actions.extend([
        Node(package='go2_offline_navigation', executable='offline_map_server',
             name='offline_map_server', output='screen', parameters=[{
                 'map_directory': str(map_directory), 'database_directory': str(database),
                 'use_sim_time': use_sim_time}]),
        GroupAction(actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            localization / 'launch/relocalization.launch.py')), launch_arguments={
                'database': str(database), 'use_sim_time': value('use_sim_time'),
                'params_file': value('relocalization_params_file'),
                'candidate_threads': value('localization_candidate_threads'),
                'publish_known_map': value('publish_known_map'),
            }.items())]),
        Node(package='go2_offline_navigation', executable='localization_adapter',
             name='offline_localization', output='screen', parameters=[{
                 'use_sim_time': use_sim_time,
                 'live_obstacle_freshness_timeout': float(value('live_obstacle_freshness_timeout'))}]),
    ])
    if flag('start_navigation'):
        navigation = Path(get_package_share_directory('navigation_bringup'))
        actions.append(GroupAction(actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            navigation / 'launch/real_nav2.launch.py')), launch_arguments={
                'use_sim_time': value('use_sim_time'), 'params_file': params,
                'lio_height_offset': value('lio_height_offset'),
                'cmd_vel_topic': '/go2/offline/cmd_vel_unchecked',
                'start_cmd_vel_bridge': 'false',
            }.items())]))
    if flag('start_cmd_vel_bridge'):
        bridge = Path(get_package_share_directory('go2_cmd_vel_bridge'))
        actions.append(Node(package='go2_cmd_vel_bridge', executable='cmd_vel_bridge',
            name='offline_cmd_vel_bridge', output='screen', parameters=[
                str(bridge / 'config/go2_cmd_vel.yaml'), {
                    'input_topic': '/go2/offline/cmd_vel',
                    'request_topic': value('sport_request_topic'), 'enabled': True,
                    'use_sim_time': False,
                    'forward_compensation_enabled': flag('forward_compensation'),
                    'angular_compensation_enabled': flag('angular_compensation')}]))
    return actions


def generate_launch_description():
    own = Path(get_package_share_directory('go2_offline_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('database', description='Known-map keyframe database directory.'),
        DeclareLaunchArgument('map_directory', description='Matching exported navigation terrain directory.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_odometry', default_value='true'),
        DeclareLaunchArgument('measured_obstacle_decay_time', default_value='0.6',
                              description='Measured obstacle lifetime in source time; ground history is 2s.'),
        DeclareLaunchArgument('live_obstacle_freshness_timeout', default_value='2.0',
                              description='Global input consumption age; local perception gates stay unchanged.'),
        DeclareLaunchArgument('start_navigation', default_value='true'),
        DeclareLaunchArgument('start_cmd_vel_bridge', default_value='false',
                              description='Explicitly enable physical commands after validation.'),
        DeclareLaunchArgument('forward_compensation', default_value='true'),
        DeclareLaunchArgument('angular_compensation', default_value='true'),
        DeclareLaunchArgument('lidar_driver', default_value='hesai', choices=['hesai', 'none']),
        DeclareLaunchArgument('start_go2_imu_bridge', default_value='true'),
        DeclareLaunchArgument('start_go2_odom_bridge', default_value='true'),
        DeclareLaunchArgument('imu_topic', default_value='/go2/imu'),
        DeclareLaunchArgument('lowstate_topic', default_value='/lowstate'),
        DeclareLaunchArgument('sportmodestate_topic', default_value='/lf/sportmodestate'),
        DeclareLaunchArgument('publish_known_map', default_value='false'),
        DeclareLaunchArgument('live_obstacle_topic', default_value='/terrain/obstacles_measured_global',
                              description='Live global-planner obstacles; local terrain checks are unchanged.'),
        DeclareLaunchArgument('localization_candidate_threads', default_value='2', choices=['1', '2', '3', '4']),
        DeclareLaunchArgument('relocalization_params_file', default_value=str(Path(
            get_package_share_directory('go2_relocalization')) / 'config/relocalization.yaml')),
        DeclareLaunchArgument('sport_request_topic', default_value='/api/sport/request'),
        DeclareLaunchArgument('lio_height_offset', default_value='-0.50',
                              description='Nominal IMU-to-ground height; verify operating posture.'),
        DeclareLaunchArgument('params_file', default_value=str(own / 'config/nav2_mppi_params.yaml')),
        OpaqueFunction(function=start),
    ])
