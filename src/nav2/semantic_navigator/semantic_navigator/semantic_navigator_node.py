"""ROS 2 semantic-command bridge to the Nav2 NavigateToPose action."""

from dataclasses import dataclass
import json
import math
import os
from typing import List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .semantic_map import (
    PoseCandidate,
    SemanticEntity,
    SemanticMap,
    is_cancel_command,
    load_semantic_map,
    rank_poses,
    resolve_entity,
)


@dataclass
class NavigationAttempt:
    generation: int
    command: str
    entity: SemanticEntity
    candidates: List[PoseCandidate]
    index: int = 0


class SemanticNavigatorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('semantic_navigator', **kwargs)
        self._semantic_map_file = self.declare_parameter(
            'semantic_map_file', ''
        ).value
        self._command_topic = self.declare_parameter(
            'command_topic', '/semantic_navigation/command'
        ).value
        self._status_topic = self.declare_parameter(
            'status_topic', '/semantic_navigation/status'
        ).value
        self._selected_goal_topic = self.declare_parameter(
            'selected_goal_topic', '/semantic_navigation/selected_goal'
        ).value
        self._marker_topic = self.declare_parameter(
            'marker_topic', '/semantic_navigation/markers'
        ).value
        self._action_name = self.declare_parameter(
            'navigate_to_pose_action', '/navigate_to_pose'
        ).value
        self._base_frame = self.declare_parameter(
            'base_frame', 'base_link'
        ).value
        self._fuzzy_threshold = float(
            self.declare_parameter('fuzzy_threshold', 0.62).value
        )
        self._action_server_timeout = float(
            self.declare_parameter('action_server_timeout', 1.0).value
        )
        self._retry_alternate_poses = bool(
            self.declare_parameter('retry_alternate_poses', True).value
        )
        self._reload_interval = float(
            self.declare_parameter('reload_interval', 2.0).value
        )

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String, self._status_topic, 10
        )
        self._goal_publisher = self.create_publisher(
            PoseStamped, self._selected_goal_topic, transient_qos
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray, self._marker_topic, transient_qos
        )
        self._command_subscription = self.create_subscription(
            String, self._command_topic, self._command_callback, 10
        )
        self._action_client = ActionClient(
            self, NavigateToPose, self._action_name
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._semantic_map = SemanticMap(frame_id='map', entities=())
        self._map_mtime: Optional[float] = None
        self._generation = 0
        self._attempt: Optional[NavigationAttempt] = None
        self._goal_handle = None
        self._load_map(report_unchanged=True)
        self.create_timer(max(0.5, self._reload_interval), self._reload_map)

        self.get_logger().info(
            f'Semantic navigation ready: {self._command_topic} -> '
            f'{self._action_name}, entities={len(self._semantic_map.entities)}'
        )

    def _publish_status(self, state: str, message: str, **fields):
        payload = {'state': state, 'message': message, **fields}
        output = String()
        output.data = json.dumps(payload, ensure_ascii=False)
        self._status_publisher.publish(output)
        if state in {'error', 'failed', 'unknown_target'}:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def _load_map(self, report_unchanged: bool = False) -> bool:
        if not self._semantic_map_file:
            self._publish_status(
                'error', 'semantic_map_file 参数为空，无法加载语义地图'
            )
            return False
        try:
            loaded = load_semantic_map(self._semantic_map_file)
            mtime = os.path.getmtime(
                os.path.expanduser(self._semantic_map_file)
            )
        except (OSError, ValueError) as error:
            self._publish_status(
                'error', f'加载语义地图失败：{error}'
            )
            return False
        changed = loaded != self._semantic_map
        self._semantic_map = loaded
        self._map_mtime = mtime
        if changed or report_unchanged:
            self._publish_markers()
            self._publish_status(
                'map_loaded',
                f'已加载 {len(loaded.entities)} 个语义实体',
                frame_id=loaded.frame_id,
                entity_count=len(loaded.entities),
            )
        return True

    def _reload_map(self):
        try:
            mtime = os.path.getmtime(
                os.path.expanduser(self._semantic_map_file)
            )
        except OSError:
            return
        if self._map_mtime is None or mtime > self._map_mtime + 1.0e-6:
            self._load_map()

    def _current_xy(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._semantic_map.frame_id,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.25),
            )
            translation = transform.transform.translation
            return translation.x, translation.y
        except TransformException as error:
            self.get_logger().warning(
                f'无法获取当前位置，将按配置顺序选择停靠位：{error}'
            )
            return None

    def _command_callback(self, message: String):
        command = message.data.strip()
        if not command:
            self._publish_status('error', '收到空的语义导航指令')
            return
        if is_cancel_command(command):
            self._cancel_navigation('收到取消指令')
            return

        match = resolve_entity(
            command,
            self._semantic_map.entities,
            self._fuzzy_threshold,
        )
        if match is None:
            available = [
                entity.names[0] for entity in self._semantic_map.entities
            ]
            self._publish_status(
                'unknown_target',
                f'无法从“{command}”匹配语义目标',
                command=command,
                available_targets=available,
            )
            return

        self._generation += 1
        attempt = NavigationAttempt(
            generation=self._generation,
            command=command,
            entity=match.entity,
            candidates=rank_poses(
                match.entity.poses, self._current_xy()
            ),
        )
        self._attempt = attempt
        self._publish_status(
            'resolved',
            f'“{command}”匹配到 {match.entity.entity_id}'
            f'（别名：{match.alias}）',
            command=command,
            entity_id=match.entity.entity_id,
            score=round(match.score, 3),
            candidate_count=len(attempt.candidates),
        )
        if self._goal_handle is not None:
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda _future, generation=attempt.generation:
                self._start_if_current(generation)
            )
        else:
            self._send_current_candidate(attempt)

    def _start_if_current(self, generation: int):
        self._goal_handle = None
        if self._attempt is not None and self._attempt.generation == generation:
            self._send_current_candidate(self._attempt)

    def _send_current_candidate(self, attempt: NavigationAttempt):
        if attempt.generation != self._generation:
            return
        if attempt.index >= len(attempt.candidates):
            self._attempt = None
            self._publish_status(
                'failed',
                f'{attempt.entity.entity_id} 的所有停靠位均导航失败',
                entity_id=attempt.entity.entity_id,
            )
            return
        if not self._action_client.wait_for_server(
            timeout_sec=self._action_server_timeout
        ):
            self._publish_status(
                'error',
                f'Nav2 action {self._action_name} 尚未就绪',
            )
            return

        pose = attempt.candidates[attempt.index]
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self._semantic_map.frame_id
        goal_pose.pose.position.x = pose.x
        goal_pose.pose.position.y = pose.y
        goal_pose.pose.position.z = pose.z
        goal_pose.pose.orientation.z = math.sin(pose.yaw * 0.5)
        goal_pose.pose.orientation.w = math.cos(pose.yaw * 0.5)
        self._goal_publisher.publish(goal_pose)

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future, generation=attempt.generation:
            self._goal_response_callback(future, generation)
        )
        self._publish_status(
            'navigating',
            f'正在前往 {attempt.entity.entity_id} 的停靠位 '
            f'{attempt.index + 1}/{len(attempt.candidates)}',
            entity_id=attempt.entity.entity_id,
            candidate_index=attempt.index,
            x=pose.x,
            y=pose.y,
            yaw=pose.yaw,
        )

    def _goal_response_callback(self, future, generation: int):
        if generation != self._generation:
            return
        try:
            goal_handle = future.result()
        except Exception as error:  # rclpy futures expose transport failures.
            self._candidate_failed(generation, f'发送目标失败：{error}')
            return
        if not goal_handle.accepted:
            self._candidate_failed(generation, 'Nav2 拒绝目标')
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, expected=generation:
            self._navigation_result_callback(result, expected)
        )

    def _navigation_result_callback(self, future, generation: int):
        if generation != self._generation:
            return
        self._goal_handle = None
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
        except Exception as error:
            self._candidate_failed(generation, f'导航结果异常：{error}')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            entity_id = self._attempt.entity.entity_id
            self._attempt = None
            self._publish_status(
                'succeeded',
                f'已到达语义目标 {entity_id}',
                entity_id=entity_id,
            )
            return
        self._candidate_failed(
            generation, f'Nav2 返回状态 {status}'
        )

    def _candidate_failed(self, generation: int, reason: str):
        if self._attempt is None or generation != self._generation:
            return
        attempt = self._attempt
        if self._retry_alternate_poses:
            attempt.index += 1
        else:
            attempt.index = len(attempt.candidates)
        self._publish_status(
            'candidate_failed',
            f'{attempt.entity.entity_id} 当前停靠位失败：{reason}',
            entity_id=attempt.entity.entity_id,
            reason=reason,
        )
        self._send_current_candidate(attempt)

    def _cancel_navigation(self, reason: str):
        self._generation += 1
        self._attempt = None
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._publish_status('canceled', reason)

    def _publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        marker_id = 0
        for entity in self._semantic_map.entities:
            for index, pose in enumerate(entity.poses):
                arrow = Marker()
                arrow.header.frame_id = self._semantic_map.frame_id
                arrow.header.stamp = stamp
                arrow.ns = 'semantic_goal_poses'
                arrow.id = marker_id
                marker_id += 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.position.x = pose.x
                arrow.pose.position.y = pose.y
                arrow.pose.position.z = pose.z + 0.05
                arrow.pose.orientation.z = math.sin(pose.yaw * 0.5)
                arrow.pose.orientation.w = math.cos(pose.yaw * 0.5)
                arrow.scale.x = 0.35
                arrow.scale.y = 0.08
                arrow.scale.z = 0.08
                arrow.color.r = 0.1
                arrow.color.g = 0.8
                arrow.color.b = 0.2
                arrow.color.a = 0.9
                markers.markers.append(arrow)
                if index != 0:
                    continue
                label = Marker()
                label.header = arrow.header
                label.ns = 'semantic_goal_labels'
                label.id = marker_id
                marker_id += 1
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = pose.x
                label.pose.position.y = pose.y
                label.pose.position.z = pose.z + 0.45
                label.pose.orientation.w = 1.0
                label.scale.z = 0.22
                label.color.r = 1.0
                label.color.g = 1.0
                label.color.b = 1.0
                label.color.a = 1.0
                label.text = entity.names[0]
                markers.markers.append(label)
        delete_old = Marker()
        delete_old.action = Marker.DELETEALL
        markers.markers.insert(0, delete_old)
        self._marker_publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
