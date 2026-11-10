#!/usr/bin/env python3
"""Request a global path for RViz goals, without executing robot motion."""

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


class RvizGoalPlanner(Node):
    def __init__(self):
        super().__init__('rviz_goal_planner')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('planner_id', 'GridBased')
        self.client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.pending_goal = None
        self.busy = False
        self.waiting_notice = False
        self.subscription = self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value, self.on_goal, 10)
        # Discovery retry must continue after the replay's /clock has stopped.
        self.retry_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.2, self.send_pending, clock=self.retry_clock)
        self.get_logger().info('RViz /goal_pose -> global planner; paths appear on /plan.')

    def on_goal(self, pose):
        if not pose.header.frame_id:
            self.get_logger().error('Goal has no frame. Set RViz Fixed Frame to map.')
            return
        self.pending_goal = pose
        self.waiting_notice = False
        self.send_pending()

    def send_pending(self):
        if self.pending_goal is None or self.busy:
            return
        if not self.client.server_is_ready():
            if not self.waiting_notice:
                self.get_logger().info('Waiting for /compute_path_to_pose; keeping the latest goal.')
                self.waiting_notice = True
            return
        goal = ComputePathToPose.Goal()
        goal.goal = self.pending_goal
        goal.planner_id = self.get_parameter('planner_id').value
        goal.use_start = False
        self.pending_goal = None
        self.busy = True
        self.get_logger().info(
            f'Computing path to ({goal.goal.pose.position.x:.2f}, '
            f'{goal.goal.pose.position.y:.2f}) in {goal.goal.header.frame_id}.')
        try:
            self.client.send_goal_async(goal).add_done_callback(self.on_response)
        except Exception as exc:
            self.finish(f'Could not send planning request: {exc}')

    def on_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.finish('Planner rejected the request.')
                return
            handle.get_result_async().add_done_callback(self.on_result)
        except Exception as exc:
            self.finish(f'Could not submit goal: {exc}')

    def on_result(self, future):
        try:
            response = future.result()
            if response.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f'Path ready on /plan: {len(response.result.path.poses)} poses.')
                self.finish()
            else:
                self.finish(
                    f'No new path (action status {response.status}); '
                    'check planner_server logs. An old RViz path may remain visible.')
        except Exception as exc:
            self.finish(f'Could not read planning result: {exc}')

    def finish(self, error=None):
        if error:
            self.get_logger().warning(error)
        self.busy = False
        self.send_pending()


def main():
    rclpy.init()
    node = RvizGoalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
