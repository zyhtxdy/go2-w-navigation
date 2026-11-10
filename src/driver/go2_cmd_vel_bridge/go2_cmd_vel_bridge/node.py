"""Planar velocity bridge. Constructing this node alone sends no requests."""

import math
import signal
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from unitree_api.msg import Request

from go2_cmd_vel_bridge.command import CommandGate, MOVE_API_ID
from go2_cmd_vel_bridge.compensation import ForwardCompensation, AngularCompensation


class CmdVelBridge(Node):
    def __init__(self, **kwargs):
        super().__init__('go2_cmd_vel_bridge', **kwargs)
        defaults = {
            'input_topic': '/go2/nav/cmd_vel',
            'request_topic': '/api/sport/request',
            'estop_topic': '/go2/nav/estop',
            'enabled': True,
            'publish_rate': 20.0,
            'command_timeout': 0.3,
            'stop_repetitions': 3,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        calibration_defaults = {
            'forward_compensation_enabled': True,
            'forward_compensation_gain': 1.111443,
            'forward_compensation_offset': 0.059202,
            'forward_compensation_blend_speed': 0.05,
            'forward_compensation_max_command': 0.70,
            'angular_compensation_enabled': True,
            'angular_compensation_gain': 0.882380,
            'angular_compensation_offset': 0.025898,
            'angular_compensation_blend_speed': 0.05,
            'angular_compensation_max_speed': 0.40,
        }
        for name, default in calibration_defaults.items():
            self.declare_parameter(name, default, ParameterDescriptor(
                read_only=True, description='Velocity calibration; set before starting the node.'))
        value = lambda name: self.get_parameter(name).value
        if value('use_sim_time'):
            raise ValueError('The physical Go2 command bridge cannot use simulated time')
        rate = value('publish_rate')
        if not math.isfinite(rate) or rate < 1.0 or rate > 100.0:
            raise ValueError('publish_rate must be finite and between 1 and 100 Hz')
        self.gate = CommandGate(
            timeout=value('command_timeout'),
            stop_repetitions=value('stop_repetitions'), enabled=value('enabled'),
            compensation=ForwardCompensation(
                enabled=value('forward_compensation_enabled'),
                gain=value('forward_compensation_gain'),
                offset=value('forward_compensation_offset'),
                blend_speed=value('forward_compensation_blend_speed'),
                max_command=value('forward_compensation_max_command')),
            angular_compensation=AngularCompensation(
                enabled=value('angular_compensation_enabled'),
                gain=value('angular_compensation_gain'),
                offset=value('angular_compensation_offset'),
                blend_speed=value('angular_compensation_blend_speed'),
                max_speed=value('angular_compensation_max_speed')))
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        request_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            lifespan=Duration(seconds=value('command_timeout')))
        self.publisher = self.create_publisher(Request, value('request_topic'), request_qos)
        # Observation only: these topics are never routed to the Sport API.
        self.requested_publisher = self.create_publisher(TwistStamped, '~/cmd_vel_requested', qos)
        self.sent_publisher = self.create_publisher(TwistStamped, '~/cmd_vel_sent', qos)
        self.subscription = self.create_subscription(Twist, value('input_topic'), self.receive, qos)
        self.estop_subscription = self.create_subscription(
            Bool, value('estop_topic'), self.estop, qos)
        self.enable_service = self.create_service(SetBool, '~/set_enabled', self.set_enabled)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / rate, self.tick, clock=self.steady_clock)
        mode = ('forward compensation=' + str(value('forward_compensation_enabled')) +
                ', yaw compensation=' + str(value('angular_compensation_enabled')))
        self.get_logger().info(
            f"{value('input_topic')} -> {value('request_topic')}; "
            f'{mode}; waiting for fresh velocity')

    def publish_velocity_observation(self, requested, sent):
        stamp = self.get_clock().now().to_msg()
        for publisher, values in ((self.requested_publisher, requested), (self.sent_publisher, sent)):
            message = TwistStamped()
            message.header.stamp = stamp
            # The input Twist has no frame ID: do not invent an independent
            # transform here. Both observations use the bridge's body axes.
            message.twist.linear.x, message.twist.linear.y, message.twist.angular.z = values
            publisher.publish(message)

    def receive(self, message):
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        try:
            self.gate.receive(values, time.monotonic())
        except (TypeError, ValueError, OverflowError) as error:
            self.get_logger().error(f'Rejected velocity; stopping active command: {error}')

    def publish_request(self, command):
        request = Request()
        request.header.identity.id = time.time_ns()
        request.header.identity.api_id, request.parameter = command
        request.header.policy.noreply = True
        self.publisher.publish(request)

    def tick(self):
        command = self.gate.tick(time.monotonic())
        if command is not None:
            self.publish_request(command)
            if command[0] == MOVE_API_ID:
                self.publish_velocity_observation(self.gate.requested_velocity, self.gate.velocity)
            else:
                self.publish_velocity_observation((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    def estop(self, message):
        self.gate.set_estop(message.data)
        self.tick()
        if message.data:
            self.get_logger().warn('Software stop set; clear it and send a fresh velocity to resume')

    def set_enabled(self, request, response):
        self.gate.set_enabled(request.data)
        self.tick()
        response.success = True
        response.message = 'Enabled; waiting for a fresh velocity' if request.data else 'Disabled'
        return response

    def stop_before_shutdown(self):
        self.gate.stop()
        while self.gate.stops_left:
            self.tick()


def main(args=None):
    # Delay rclpy shutdown until the active bridge gets a best-effort chance to
    # send StopMove on Ctrl-C. This does not cover SIGKILL or power/network loss.
    from rclpy.signals import SignalHandlerOptions
    def interrupt_handler(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt_handler)
    signal.signal(signal.SIGTERM, interrupt_handler)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = CmdVelBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop_before_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
