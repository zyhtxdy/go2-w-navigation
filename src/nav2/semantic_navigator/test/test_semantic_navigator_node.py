import json
import time

from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from semantic_navigator.semantic_navigator_node import SemanticNavigatorNode


def test_command_reaches_nav2_action_and_reports_success(tmp_path):
    map_file = tmp_path / 'semantic.yaml'
    map_file.write_text(
        """
semantic_map:
  frame_id: map
  locations:
    - id: meeting_room
      names: [会议室]
      poses:
        - {x: 1.25, y: -0.5, z: 0.0, yaw: 1.57}
""",
        encoding='utf-8',
    )

    rclpy.init()
    server_node = Node('mock_nav2_server')
    received_goals = []

    def execute(goal_handle):
        received_goals.append(goal_handle.request.pose)
        goal_handle.succeed()
        return NavigateToPose.Result()

    action_server = ActionServer(
        server_node,
        NavigateToPose,
        '/navigate_to_pose',
        execute_callback=execute,
    )
    navigator = SemanticNavigatorNode(
        parameter_overrides=[
            Parameter(
                'semantic_map_file',
                Parameter.Type.STRING,
                str(map_file),
            ),
            Parameter(
                'action_server_timeout',
                Parameter.Type.DOUBLE,
                0.2,
            ),
        ]
    )
    client_node = Node('semantic_navigation_test_client')
    command_publisher = client_node.create_publisher(
        String, '/semantic_navigation/command', 10
    )
    states = []

    def status_callback(message):
        states.append(json.loads(message.data)['state'])

    status_subscription = client_node.create_subscription(
        String,
        '/semantic_navigation/status',
        status_callback,
        10,
    )
    assert status_subscription is not None

    executor = MultiThreadedExecutor(num_threads=4)
    for node in (server_node, navigator, client_node):
        executor.add_node(node)
    try:
        discovery_deadline = time.monotonic() + 2.0
        while (
            command_publisher.get_subscription_count() == 0
            and time.monotonic() < discovery_deadline
        ):
            executor.spin_once(timeout_sec=0.05)
        command_publisher.publish(String(data='请带我去会议室'))

        result_deadline = time.monotonic() + 5.0
        while (
            'succeeded' not in states
            and time.monotonic() < result_deadline
        ):
            executor.spin_once(timeout_sec=0.05)

        assert 'resolved' in states
        assert 'navigating' in states
        assert 'succeeded' in states
        assert len(received_goals) == 1
        assert received_goals[0].header.frame_id == 'map'
        assert received_goals[0].pose.position.x == 1.25
        assert received_goals[0].pose.position.y == -0.5
    finally:
        action_server.destroy()
        for node in (client_node, navigator, server_node):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
