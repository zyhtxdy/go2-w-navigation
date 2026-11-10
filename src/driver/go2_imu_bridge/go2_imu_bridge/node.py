"""ROS 2 node for the read-only LowState to body IMU bridge."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from go2_imu_bridge.conversion import ConversionError, lowstate_to_imu


class LowstateImuBridge(Node):
    def __init__(self, lowstate_message_type):
        super().__init__('go2_imu_bridge')
        self.declare_parameter('input_topic', '/lowstate')
        self.declare_parameter('output_topic', '/go2/imu')
        self.declare_parameter('frame_id', 'go2_imu')
        self.declare_parameter('queue_depth', 20)
        self.declare_parameter('publish_orientation', False)
        # Node's ROS clock already honors the standard use_sim_time parameter.
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        self._publish_orientation = self.get_parameter('publish_orientation').value
        queue_depth = self.get_parameter('queue_depth').value
        if queue_depth < 1:
            raise ValueError('queue_depth must be at least 1')
        if not self._frame_id:
            raise ValueError('frame_id must be a nonempty string')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=queue_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(Imu, output_topic, qos)
        self._subscription = self.create_subscription(
            lowstate_message_type, input_topic, self._callback, qos)
        self.get_logger().info(
            f'Body IMU bridge: {input_topic} -> {output_topic}, frame={self._frame_id}, '
            'Best Effort / Volatile')
        self.get_logger().warning(
            'IMU header.stamp uses ROS reception time, not the sensor sampling time. '
            'LowState.tick is not interpreted. Strict LiDAR/IMU time synchronization, '
            'units, axes and extrinsics have not been verified on this robot.')

    def _callback(self, lowstate):
        stamp = self.get_clock().now().to_msg()
        try:
            message = lowstate_to_imu(
                lowstate, stamp, self._frame_id, self._publish_orientation)
        except ConversionError as exc:
            self.get_logger().warning(
                f'Dropping invalid LowState IMU sample: {exc}', throttle_duration_sec=5.0)
            return
        self._publisher.publish(message)


def main(args=None):
    try:
        from unitree_go.msg import LowState
    except ImportError as exc:
        raise SystemExit(
            'Cannot import unitree_go.msg.LowState. Build the unitree_go message '
            'package and go2_imu_bridge, then source that workspace install/setup.bash. '
            f'Original import error: {exc}') from exc
    rclpy.init(args=args)
    node = None
    try:
        node = LowstateImuBridge(LowState)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble can interrupt take_message while SIGINT shuts down the context.
        # Suppress only shutdown errors; preserve conversion failures while live.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
