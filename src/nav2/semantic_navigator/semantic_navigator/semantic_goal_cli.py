"""Command-line publisher for semantic navigation instructions."""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main(args=None):
    arguments = sys.argv[1:] if args is None else list(args)
    if not arguments:
        print('用法：ros2 run semantic_navigator semantic_goal 去会议室')
        return 2
    command = ' '.join(arguments).strip()
    rclpy.init()
    node = Node('semantic_goal_cli')
    publisher = node.create_publisher(
        String, '/semantic_navigation/command', 10
    )
    deadline = time.monotonic() + 2.0
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    message = String()
    message.data = command
    publisher.publish(message)
    rclpy.spin_once(node, timeout_sec=0.25)
    print(f'已发送语义导航指令：{command}')
    node.destroy_node()
    rclpy.shutdown()
    return 0
