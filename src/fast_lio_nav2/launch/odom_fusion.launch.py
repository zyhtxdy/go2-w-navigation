import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_body_imu_transforms(context):
    """Attach the two IMU frame names below the EKF-owned base_link."""
    imu_frame = LaunchConfiguration('go2_imu_frame_id').perform(context)
    if not imu_frame or imu_frame in ('base_link', 'odom', 'map'):
        raise ValueError('go2_imu_frame_id must be an IMU frame below base_link')
    components = []
    for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        components.extend([
            '--' + axis, LaunchConfiguration('base_to_imu_' + axis).perform(context)])
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    # FAST-LIO names this IMU frame body; the bridge names it go2_imu.
    # Both refer to the same physical IMU, so they share one mounting transform.
    frames = ['body'] if imu_frame == 'body' else ['body', imu_frame]
    return [Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_to_' + ('body' if frame == 'body' else 'go2_imu') + '_tf',
        arguments=components + [
            '--frame-id', 'base_link', '--child-frame-id', frame],
        parameters=[{'use_sim_time': use_sim_time}],
    ) for frame in frames]


def launch_go2_body_transform(context):
    """Use an explicit motion-state reference point, without renaming the data."""
    reference = LaunchConfiguration('sportstate_reference_point').perform(context)
    if reference == 'external':
        return []
    frame = LaunchConfiguration('go2_body_frame_id').perform(context)
    imu_frame = LaunchConfiguration('go2_imu_frame_id').perform(context)
    if not frame or frame in ('odom', 'map'):
        raise ValueError('go2_body_frame_id must be a robot frame below odom')
    if frame == 'base_link':
        if reference != 'base':
            raise ValueError('go2_body_frame_id=base_link requires sportstate_reference_point=base')
        return []
    if frame in ('body', imu_frame):
        if reference != 'imu':
            raise ValueError('An IMU frame name requires sportstate_reference_point=imu')
        # The shared IMU transform above, or an external robot model, owns it.
        return []
    if reference not in ('imu', 'base'):
        raise ValueError('sportstate_reference_point must be imu, base or external')
    components = []
    for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        value = (LaunchConfiguration('base_to_imu_' + axis).perform(context)
                 if reference == 'imu' else '0.0')
        components.extend(['--' + axis, value])
    return [Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_to_go2_body_tf',
        arguments=components + [
            '--frame-id', 'base_link', '--child-frame-id', frame],
        parameters=[{'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'), value_type=bool)}],
    )]


def generate_launch_description():
    fusion_share = get_package_share_directory('fast_lio_nav2')
    fast_lio_share = get_package_share_directory('fast_lio')
    terrain_share = get_package_share_directory('terrain_costmap')
    # Software startup thresholds; these are not a measured hardware calibration.
    imu_bias_defaults = {
        'calibration_samples': '400',
        'calibration_minimum_duration': '1.0',
        'calibration_maximum_gyro_norm': '0.05',
        'calibration_maximum_axis_variance': '0.0002',
        'calibration_maximum_orientation_change': '0.01',
        'calibration_maximum_sample_gap': '0.1',
    }
    imu_topic = LaunchConfiguration('imu_topic')
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'rviz': 'false',
            'lidar_driver': LaunchConfiguration('lidar_driver'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'imu_topic': imu_topic,
            'start_go2_imu_bridge': LaunchConfiguration('start_go2_imu_bridge'),
            'lowstate_topic': LaunchConfiguration('lowstate_topic'),
            'go2_imu_frame_id': LaunchConfiguration('go2_imu_frame_id'),
            'go2_imu_publish_orientation': LaunchConfiguration('go2_imu_publish_orientation'),
            'publish_fast_lio_tf': 'false',
            'publish_map_odom_tf': 'false',
            'publish_body_base_tf': 'false',
            # Accumulate registered scans in RViz instead of serializing the
            # full IKD tree on /Laser_map.
            'publish_map': 'false',
        }.items(),
    )

    # T_base_imu is the nominal installation in Unitree's Go2W URDF.
    # Keep base_link as parent: EKF owns odom -> base_link, and FAST-LIO's
    # odom -> body / standalone body -> base_link publishers remain disabled.
    body_imu_transforms = OpaqueFunction(
        function=launch_body_imu_transforms,
        condition=IfCondition(LaunchConfiguration('publish_body_imu_tf')),
    )
    # Unitree's official Go2W MuJoCo model places frame_vel at the imu site.
    # This is a traceable nominal reference, not verification of real firmware.
    go2_body_transform = OpaqueFunction(function=launch_go2_body_transform)

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        condition=IfCondition(LaunchConfiguration('publish_map_odom_tf')),
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    lio_base_odom = Node(
        package='fast_lio_nav2',
        executable='lio_base_odom_node',
        name='lio_base_odom',
        output='screen',
        parameters=[{
            'input_topic': '/Odometry',
            'output_topic': '/lio/odom_base',
            'odom_frame': 'odom',
            'body_frame': 'body',
            'base_frame': 'base_link',
            'position_stddev': 0.05,
            'yaw_stddev': 0.02,
            'max_initial_distance': 5.0,
            'max_planar_speed': 5.0,
            'use_sim_time': use_sim_time,
        }],
    )

    imu_bias_corrector = Node(
        package='fast_lio_nav2',
        executable='imu_bias_corrector_node',
        name='imu_bias_corrector',
        output='screen',
        parameters=[{
            'input_topic': imu_topic,
            'output_topic': '/imu/corrected',
            **{name: ParameterValue(
                LaunchConfiguration(name),
                value_type=int if name == 'calibration_samples' else float,
            ) for name in imu_bias_defaults},
            'use_sim_time': use_sim_time,
        }],
    )

    go2_odom_bridge = Node(
        package='go2_odom_bridge',
        executable='sportmodestate_odom_bridge',
        name='go2_odom_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_go2_odom_bridge')),
        parameters=[{
            'input_topic': LaunchConfiguration('sportmodestate_topic'),
            'output_topic': '/go2/odom_raw',
            'frame_id': 'go2_state_odom',
            'child_frame_id': LaunchConfiguration('go2_body_frame_id'),
            'velocity_frame': LaunchConfiguration('sportstate_velocity_frame'),
            'timestamp_source': LaunchConfiguration('sportstate_timestamp_source'),
            'linear_velocity_stddev': ParameterValue(
                LaunchConfiguration('sportstate_linear_velocity_stddev'),
                value_type=float),
            'use_sim_time': use_sim_time,
        }],
    )

    odom_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='odom_ekf',
        output='screen',
        parameters=[os.path.join(fusion_share, 'config', 'ekf.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/odom'),
        ],
    )

    terrain_costmap = Node(
        package='terrain_costmap',
        executable='terrain_costmap_node',
        name='terrain_costmap',
        output='screen',
        parameters=[
            os.path.join(
                terrain_share, 'config', 'terrain_costmap.yaml'),
            {'use_sim_time': use_sim_time,
             'measured_obstacle_decay_time': ParameterValue(
                 LaunchConfiguration('measured_obstacle_decay_time'), value_type=float)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        *[DeclareLaunchArgument(
            name, default_value=value,
            description='Startup gyro stationary-window threshold; see fast_lio_nav2 README.',
        ) for name, value in imu_bias_defaults.items()],
        DeclareLaunchArgument(
            'measured_obstacle_decay_time', default_value='0.6',
            description='Global measured-obstacle age in source time; ground fit history stays 2s.'),
        DeclareLaunchArgument(
            'publish_map_odom_tf', default_value='true',
            description='Publish identity map to odom; disable when map localization owns TF.'),
        DeclareLaunchArgument(
            'lidar_driver', default_value='hesai', choices=['hesai', 'none'],
            description='Use none for rosbag replay or an existing point-cloud publisher.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/go2/imu',
            description='Raw IMU topic shared by FAST-LIO and gyro bias correction.',
        ),
        DeclareLaunchArgument(
            'start_go2_imu_bridge', default_value='true',
            description='Start the body IMU bridge; disable if an input publisher already exists.',
        ),
        DeclareLaunchArgument('lowstate_topic', default_value='/lowstate'),
        DeclareLaunchArgument('go2_imu_frame_id', default_value='go2_imu'),
        DeclareLaunchArgument(
            'go2_imu_publish_orientation', default_value='true',
            description='Body attitude is used for bias stillness detection, not EKF orientation fusion.',
        ),
        DeclareLaunchArgument(
            'publish_body_imu_tf', default_value='true',
            description='Publish base_link to body and IMU TF; disable if supplied externally.',
        ),
        *[DeclareLaunchArgument(
            'base_to_imu_' + axis, default_value=value,
            description='Base to body IMU mounting component; nominal Go2W URDF value.',
        ) for axis, value in {
            'x': '-0.02557', 'y': '0.0', 'z': '0.04232',
            'roll': '0.0', 'pitch': '0.0', 'yaw': '0.0',
        }.items()],
        DeclareLaunchArgument(
            'start_go2_odom_bridge', default_value='true',
            description='Start the motion-state velocity bridge; disable if an input publisher already exists.',
        ),
        DeclareLaunchArgument(
            'sportmodestate_topic', default_value='/lf/sportmodestate',
            description='Live Go2W state topic verified on NX; override for other firmware.',
        ),
        DeclareLaunchArgument(
            'go2_body_frame_id', default_value='go2_body',
            description='Body-axis frame at the selected motion-state velocity reference point.',
        ),
        DeclareLaunchArgument(
            'sportstate_reference_point', default_value='imu',
            choices=['imu', 'base', 'external'],
            description=(
                'imu: nominal official Go2W simulator reference, unverified on hardware; '
                'base: only after confirming base origin; external: supply a measured TF.'),
        ),
        DeclareLaunchArgument(
            'sportstate_velocity_frame', default_value='body',
            choices=['odom', 'body'],
            description='Current Go2W recorded velocity matches body axes; odom is an explicit override.',
        ),
        DeclareLaunchArgument(
            'sportstate_timestamp_source', default_value='receive',
            choices=['receive', 'message'],
            description='Reception time approximation or explicitly verified message clock.',
        ),
        DeclareLaunchArgument(
            'sportstate_linear_velocity_stddev', default_value='0.2',
            description='Initial tuning value in m/s, not a measured noise figure.',
        ),
        map_to_odom,
        body_imu_transforms,
        go2_body_transform,
        lio_base_odom,
        imu_bias_corrector,
        go2_odom_bridge,
        odom_ekf,
        terrain_costmap,
        fast_lio,
    ])
