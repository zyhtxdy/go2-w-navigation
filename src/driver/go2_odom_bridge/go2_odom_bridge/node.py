"""Receive SportModeState and publish velocity observations; no commands or TF."""

import rclpy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry

from go2_odom_bridge.conversion import (
    ConversionError, sportmodestate_to_odom, validate_options,
)


class SportmodestateOdomBridge(Node):
    def __init__(self, sportmodestate_message_type):
        super().__init__('go2_odom_bridge')
        defaults = {
            'input_topic': '/lf/sportmodestate',
            'output_topic': '/go2/odom_raw',
            'frame_id': 'go2_state_odom',
            'child_frame_id': 'go2_body',
            'velocity_frame': 'body',
            'timestamp_source': 'receive',
            'linear_velocity_stddev': 0.2,
            'queue_depth': 20,
            # Offline replay reads raw CDR directly, avoiding DDS publication
            # of firmware messages whose recorded layout exceeds local types.
            'bag_path': '',
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self._options = {name: self.get_parameter(name).value for name in (
            'frame_id', 'child_frame_id', 'velocity_frame',
            'timestamp_source', 'linear_velocity_stddev')}
        validate_options(**self._options)
        queue_depth = self.get_parameter('queue_depth').value
        if isinstance(queue_depth, bool) or not isinstance(queue_depth, int) or queue_depth < 1:
            raise ValueError('queue_depth must be a positive integer')
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        for name, topic in (('input_topic', input_topic), ('output_topic', output_topic)):
            if not isinstance(topic, str) or not topic.strip():
                raise ValueError(f'{name} must be a nonempty string')
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=queue_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=queue_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(Odometry, output_topic, output_qos)
        bag_path = self.get_parameter('bag_path').value
        if not isinstance(bag_path, str):
            raise ValueError('bag_path must be a string')
        self._subscription = None
        if bag_path:
            if not self.get_parameter('use_sim_time').value:
                raise ValueError('bag_path requires use_sim_time=true and a /clock publisher')
            import rosbag2_py
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=bag_path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
            topics = {topic.name: topic for topic in reader.get_all_topics_and_types()}
            metadata = topics.get(input_topic)
            if metadata is None or metadata.type != 'unitree_go/msg/SportModeState':
                raise ValueError(f'Bag needs raw {input_topic} (unitree_go/msg/SportModeState)')
            if metadata.serialization_format != 'cdr':
                raise ValueError('Recorded SportModeState must use CDR serialization')
            reader.set_filter(rosbag2_py.StorageFilter(topics=[input_topic]))
            self._records = []
            while reader.has_next():
                _, payload, stamp = reader.read_next()
                self._records.append((stamp, payload))
            self._records.sort(key=lambda record: record[0])
            if not self._records:
                raise ValueError(f'Bag has no messages on {input_topic}')
            self._record_index = 0
            self._last_replay_clock_ns = None
            self._state_type = sportmodestate_message_type
            self._replay_timer = self.create_timer(0.002, self._replay_tick)
            self.get_logger().info(
                f'Replaying {len(self._records)} raw state records from {bag_path}; '
                'using recorded reception timestamps, publishing only standard Odometry')
        else:
            self._subscription = self.create_subscription(
                sportmodestate_message_type, input_topic, self._callback, input_qos)
        self.get_logger().info(
            f'Velocity bridge: {input_topic} -> {output_topic}; '
            f'velocity_frame={self._options["velocity_frame"]}, '
            f'timestamp_source={self._options["timestamp_source"]}')
        self.get_logger().warning(
            'Body linear XYZ are measurements; EKF should select only vx/vy. '
            'Pose and angular velocity are placeholders. '
            'Velocity/quaternion frame agreement, body frame alignment and timestamp '
            'synchronization remain unverified on this Go2W. Covariance is an initial '
            'tuning value, not a measured accuracy.')

    def _callback(self, state):
        receive_stamp = self.get_clock().now().to_msg()
        self._publish_state(state, receive_stamp)

    def _publish_state(self, state, receive_stamp):
        try:
            message = sportmodestate_to_odom(state, receive_stamp, **self._options)
        except ConversionError as exc:
            self.get_logger().warning(
                f'Dropping invalid SportModeState sample: {exc}', throttle_duration_sec=5.0)
            return
        self._publisher.publish(message)

    def _replay_tick(self):
        now = self.get_clock().now().nanoseconds
        if now == 0:
            return
        if self._last_replay_clock_ns is not None and now < self._last_replay_clock_ns:
            raise RuntimeError(
                f'Replay clock moved backwards: {self._last_replay_clock_ns} -> {now}; '
                f'{self._record_index} states published; restart the whole mapping replay')
        self._last_replay_clock_ns = now
        while self._record_index < len(self._records):
            stamp, payload = self._records[self._record_index]
            if stamp > now:
                break
            state = deserialize_message(payload, self._state_type)
            self._publish_state(state, Time(nanoseconds=stamp).to_msg())
            self._record_index += 1
        if self._record_index == len(self._records):
            self._replay_timer.cancel()
            self.get_logger().info('Raw state replay finished; velocity output stopped')


def main(args=None):
    try:
        from unitree_go.msg import SportModeState
    except ImportError as exc:
        raise SystemExit(
            'Cannot import unitree_go.msg.SportModeState. Build the unitree_go message '
            'package and go2_odom_bridge, then source that workspace install/setup.bash. '
            f'Original import error: {exc}') from exc
    rclpy.init(args=args)
    node = None
    try:
        node = SportmodestateOdomBridge(SportModeState)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
