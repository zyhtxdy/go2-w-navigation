import os.path
import tempfile
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_lidar_driver(context):
    driver = LaunchConfiguration('lidar_driver').perform(context)
    if driver == 'none':
        return []
    config_path = Path(LaunchConfiguration('hesai_config_file').perform(context)).resolve()
    with config_path.open() as source:
        config = yaml.safe_load(source)
    # SDK opens calibration files relative to its process directory. Resolve them
    # against this YAML so the installed workspace can be moved to the NX.
    for lidar in config['lidar']:
        for block_name in ('lidar_udp_type', 'pcap_type', 'rosbag_type'):
            block = lidar.get('driver', {}).get(block_name, {})
            for key in ('correction_file_path', 'firetimes_path', 'pcap_path'):
                value = block.get(key)
                if value and not os.path.isabs(value):
                    block[key] = str(config_path.parent / value)
    config_dir = tempfile.TemporaryDirectory(prefix='fast_lio_hesai_')
    resolved_config = os.path.join(config_dir.name, 'config.yaml')
    with open(resolved_config, 'w') as target:
        yaml.safe_dump(config, target)

    def cleanup(context):
        config_dir.cleanup()
        return []

    return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])), Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        name='hesai_lidar',
        parameters=[{'config_path': resolved_config}],
        output='screen',
    )]


def launch_mapping(context, parameters):
    overrides = {}
    for argument, parameter in (('lidar_topic', 'common.lid_topic'),
                                ('imu_topic', 'common.imu_topic')):
        value = LaunchConfiguration(argument).perform(context)
        if value:
            overrides[parameter] = value
    return [Node(package='fast_lio', executable='fastlio_mapping',
                 parameters=parameters + [overrides], output='screen')]


def launch_go2_imu_bridge(context):
    # Follow the same topic as FAST-LIO, including a YAML-only configuration.
    imu_topic = LaunchConfiguration('imu_topic').perform(context)
    if not imu_topic:
        config_file = os.path.join(
            LaunchConfiguration('config_path').perform(context),
            LaunchConfiguration('config_file').perform(context))
        with open(config_file) as source:
            config = yaml.safe_load(source)
        imu_topic = config['/**']['ros__parameters']['common']['imu_topic']
    return [Node(
        package='go2_imu_bridge',
        executable='lowstate_imu_bridge',
        name='go2_imu_bridge',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('lowstate_topic'),
            'output_topic': imu_topic,
            'frame_id': LaunchConfiguration('go2_imu_frame_id'),
            'publish_orientation': ParameterValue(
                LaunchConfiguration('go2_imu_publish_orientation'), value_type=bool),
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )]


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    publish_fast_lio_tf = LaunchConfiguration('publish_fast_lio_tf')
    publish_map_odom_tf = LaunchConfiguration('publish_map_odom_tf')
    publish_body_base_tf = LaunchConfiguration('publish_body_base_tf')
    publish_map = LaunchConfiguration('publish_map')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='xt16.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_publish_fast_lio_tf_cmd = DeclareLaunchArgument(
        'publish_fast_lio_tf', default_value='true',
        description='Publish the raw odom to body TF from FAST-LIO'
    )
    declare_publish_map_odom_tf_cmd = DeclareLaunchArgument(
        'publish_map_odom_tf', default_value='true',
        description='Publish a static identity map to odom TF'
    )
    declare_publish_body_base_tf_cmd = DeclareLaunchArgument(
        'publish_body_base_tf', default_value='false',
        description='Publish the configured IMU body to base_link static TF'
    )
    declare_publish_map_cmd = DeclareLaunchArgument(
        'publish_map', default_value='false',
        description=(
            'Publish the accumulated map (disabled by default because '
            'flattening and transmitting it is costly on embedded targets)'
        )
    )

    lidar_driver = OpaqueFunction(function=launch_lidar_driver)
    fast_lio_node = OpaqueFunction(function=launch_mapping,
                                  condition=IfCondition(LaunchConfiguration('start_mapping')), kwargs={
        'parameters': [PathJoinSubstitution([config_path, config_file]),
                    {
                        'use_sim_time': use_sim_time,
                        'publish.tf_publish_en': ParameterValue(
                            publish_fast_lio_tf, value_type=bool),
                        'publish.map_en': ParameterValue(
                            publish_map, value_type=bool),
                    }],
    })
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            'map',
            'odom',
        ],
        condition=IfCondition(publish_map_odom_tf),
    )
    body_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_base_tf',
        arguments=[
            '--x', LaunchConfiguration('body_to_base_x'),
            '--y', LaunchConfiguration('body_to_base_y'),
            '--z', LaunchConfiguration('body_to_base_z'),
            '--roll', LaunchConfiguration('body_to_base_roll'),
            '--pitch', LaunchConfiguration('body_to_base_pitch'),
            '--yaw', LaunchConfiguration('body_to_base_yaw'),
            '--frame-id', 'body',
            '--child-frame-id', 'base_link',
        ],
        condition=IfCondition(publish_body_base_tf),
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'start_mapping', default_value='true',
        description='False starts only the selected driver for sensor verification.'))
    ld.add_action(DeclareLaunchArgument(
        'lidar_driver', default_value='hesai', choices=['hesai', 'none'],
        description='Select driver; none uses an existing publisher or rosbag.'))
    ld.add_action(DeclareLaunchArgument(
        'hesai_config_file', default_value=os.path.join(
            package_path, 'config', 'hesai_xt16.yaml'),
        description='Hesai UDP driver configuration file.'))
    # Empty defaults let the selected YAML decide the input topics.
    # Overrides are added below only when a non-empty argument is supplied.
    ld.add_action(DeclareLaunchArgument(
        'lidar_topic', default_value='',
        description='Override YAML point cloud topic; empty uses the YAML value.'))
    ld.add_action(DeclareLaunchArgument(
        'imu_topic', default_value='',
        description='Override YAML external/robot IMU topic; empty uses the YAML value.'))
    ld.add_action(DeclareLaunchArgument(
        'start_go2_imu_bridge', default_value='false',
        description='Explicitly start the read-only Go2 LowState to Imu bridge.'))
    ld.add_action(DeclareLaunchArgument(
        'lowstate_topic', default_value='/lowstate',
        description='Unitree LowState input for the optional body IMU bridge.'))
    ld.add_action(DeclareLaunchArgument(
        'go2_imu_frame_id', default_value='go2_imu',
        description='Actual body IMU frame; this does not create a physical TF.'))
    ld.add_action(DeclareLaunchArgument(
        'go2_imu_publish_orientation', default_value='true',
        description='Expose body attitude for startup gyro stillness checks; not fused by FAST-LIO.'))
    # Unitree's Go2W URDF: T_base_imu has t=[-0.02557, 0, 0.04232], R=I.
    # Here body is FAST-LIO's IMU frame and base_link uses the official base
    # origin/axes, so publish T_body_base = inverse(T_base_imu).
    # These are nominal model values, not measurements of this robot.
    # The TF remains explicitly disabled unless publish_body_base_tf is true.
    body_to_base_defaults = {
        'x': '0.02557', 'y': '0.0', 'z': '-0.04232',
        'roll': '0.0', 'pitch': '0.0', 'yaw': '0.0',
    }
    for axis, value in body_to_base_defaults.items():
        ld.add_action(DeclareLaunchArgument(
            'body_to_base_' + axis, default_value=value,
            description='Configured body (IMU) to base_link component; Go2W model default.'))
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_publish_fast_lio_tf_cmd)
    ld.add_action(declare_publish_map_odom_tf_cmd)
    ld.add_action(declare_publish_body_base_tf_cmd)
    ld.add_action(declare_publish_map_cmd)

    ld.add_action(OpaqueFunction(
        function=launch_go2_imu_bridge,
        condition=IfCondition(LaunchConfiguration('start_go2_imu_bridge'))))
    ld.add_action(lidar_driver)
    ld.add_action(fast_lio_node)
    ld.add_action(map_to_odom_tf)
    ld.add_action(body_to_base_tf)
    ld.add_action(rviz_node)

    return ld
