#!/usr/bin/env python3
"""Start a paused local bag only after its first queued message is ready."""

import time
from pathlib import Path

import yaml

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosbag2_interfaces.srv import PlayNext, Resume
from rosgraph_msgs.msg import Clock as ClockMessage
from std_srvs.srv import Trigger


class ReplayStarter(Node):
    def __init__(self):
        super().__init__('replay_starter')
        self.declare_parameter('bag_path', '')
        metadata = yaml.safe_load(
            (Path(self.get_parameter('bag_path').value) / 'metadata.yaml').read_text())
        info = metadata['rosbag2_bagfile_information']
        self.end_ns = (int(info['starting_time']['nanoseconds_since_epoch']) +
                       int(info['duration']['nanoseconds']))
        self.finish_service = self.create_service(Trigger, '~/finish', self.finish)
        self.final_clock_publisher = None
        self.final_clock_timer = None
        self.step = self.create_client(PlayNext, '/rosbag2_player/play_next')
        self.resume = self.create_client(Resume, '/rosbag2_player/resume')
        self.stage = 'waiting'
        self.started = time.monotonic()
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.1, self.tick, clock=self.steady_clock)

    def tick(self):
        if time.monotonic() - self.started > 30:
            raise RuntimeError('Bag startup did not complete within 30 seconds')
        if self.stage == 'waiting' and self.step.service_is_ready() and self.resume.service_is_ready():
            self.stage = 'priming'
            self.step.call_async(PlayNext.Request()).add_done_callback(self.primed)

    def primed(self, future):
        if not future.result().success:
            raise RuntimeError('Paused bag could not publish its first queued message')
        # PlayNext waits for the player's queue-ready condition. Initialization
        # jumps and queue filling have completed before this resume request.
        self.stage = 'resuming'
        self.resume.call_async(Resume.Request()).add_done_callback(self.resumed)

    def resumed(self, future):
        future.result()
        self.stage = 'started'
        self.timer.cancel()
        self.get_logger().info('Bag queue ready; resumed playback without an advancing startup clock')

    def finish(self, request, response):
        del request
        if self.stage != 'started' or self.step.service_is_ready():
            response.success = False
            response.message = 'Playback has not finished; no clock takeover'
            return response
        # The player's periodic clock can stop a few milliseconds before the
        # final recorded states. After process exit, finish at the original
        # bag end without moving time backwards or fabricating a state.
        final_ns = max(self.end_ns, self.get_clock().now().nanoseconds)
        clock = ClockMessage()
        clock.clock.sec, clock.clock.nanosec = divmod(final_ns, 1000000000)
        clock_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.final_clock_publisher = self.create_publisher(ClockMessage, '/clock', clock_qos)
        started = time.monotonic()

        def publish_end():
            self.final_clock_publisher.publish(clock)
            if time.monotonic() - started >= 1.0:
                self.final_clock_timer.cancel()

        self.final_clock_timer = self.create_timer(.05, publish_end, clock=self.steady_clock)
        self.stage = 'finished'
        self.get_logger().info(f'Player exited; completing the original bag timeline at {final_ns}')
        response.success = True
        response.message = 'Completed recorded bag time; no synthetic state messages'
        return response


def main():
    rclpy.init()
    node = ReplayStarter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
