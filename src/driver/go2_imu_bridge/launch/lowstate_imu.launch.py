from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'input_topic': '/lowstate',
        'output_topic': '/go2/imu',
        'frame_id': 'go2_imu',
        'queue_depth': '20',
        # Relative orientation is used only to reject motion during gyro bias
        # calibration. FAST-LIO and the EKF do not fuse this quaternion.
        'publish_orientation': 'true',
        'use_sim_time': 'false',
    }
    arguments = [DeclareLaunchArgument(name, default_value=value)
                 for name, value in defaults.items()]
    parameters = {name: LaunchConfiguration(name) for name in
                  ('input_topic', 'output_topic', 'frame_id')}
    parameters['queue_depth'] = ParameterValue(
        LaunchConfiguration('queue_depth'), value_type=int)
    for name in ('publish_orientation', 'use_sim_time'):
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=bool)
    return LaunchDescription(arguments + [Node(
        package='go2_imu_bridge',
        executable='lowstate_imu_bridge',
        name='go2_imu_bridge',
        parameters=[parameters],
        output='screen',
    )])
