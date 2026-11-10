"""The only offline map->odom TF authority and a private velocity gate.

Relocalization's ordinary result topic remains observation-only. This adapter
uses its timestamped diagnostics to qualify results; it never calls Go2 APIs.
"""
from collections import deque
import copy
import math
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from terrain_navigation_interfaces.srv import ValidateTerrainPath

from .alignment import AlignmentGate, matrix_from_pose, rotation_distance
from .navigation_guard import NavigationGuard
from .obstacle_health import PlannerObstacleHealth


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def valid_stamp(stamp):
    return stamp.sec >= 0 and 0 <= stamp.nanosec < 1000000000 and seconds(stamp) > 0


class LocalizationAdapter(Node):
    # Missing/expired observations may wait at zero speed. Explicit map,
    # localization ambiguity/correction and clock faults still cancel the task.
    recoverable_reasons = frozenset((
        'localization_global_verification_timeout', 'localization_tracking_timeout',
        'localization_no_time_matched_odometry', 'localization_input_stale',
        'localization_matching_result_stale', 'localization_confirmation_timeout',
        'localization_heartbeat_missing', 'alignment_expired', 'map_heartbeat_missing',
        'odometry_missing', 'odometry_stamp_stale', 'live_terrain_missing',
        'terrain_stamp_stale', 'global_obstacle_heartbeat_missing',
        'global_obstacle_progress_stale', 'global_obstacle_source_stale',
        'global_obstacle_status_stale', 'global_obstacle_live_obstacle_processing_stale',
        'global_obstacle_live_obstacle_source_stale',
        'global_obstacle_localization_heartbeat_missing',
        'global_obstacle_localization_heartbeat_stale',
        'global_obstacle_localization_reflection_pending',
        'global_obstacle_waiting_for_live_obstacle_update',
        'global_obstacle_waiting_for_map_or_fresh_input'))

    def __init__(self):
        super().__init__('offline_localization')
        defaults = dict(max_alignment_age=3.0, max_odometry_age=0.5, max_terrain_age=1.0,
                        correction_translation=0.2, correction_rotation=0.15,
                        recovery_frames=3, maximum_lio_ekf_xy_difference=0.2,
                        maximum_lio_ekf_yaw_difference=0.15,
                        command_timeout=0.3, input_recovery_timeout=10.0,
                        live_obstacle_freshness_timeout=2.0,
                        input_cmd_vel='/go2/offline/cmd_vel_unchecked',
                        output_cmd_vel='/go2/offline/cmd_vel')
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.values = {name: self.get_parameter(name).value for name in defaults}
        for name, value in self.values.items():
            if isinstance(value, (int, float)) and (not math.isfinite(value) or value <= 0):
                raise ValueError('Positive finite parameter required: ' + name)
        self.gate = AlignmentGate(self.values['correction_translation'],
                                  self.values['correction_rotation'], self.values['recovery_frames'],
                                  max_gap=self.values['max_alignment_age'])
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_pub = self.create_publisher(Bool, '~/ready', state_qos)
        # Planning may revalidate a path while velocity is still held at zero.
        self.planning_ready_pub = self.create_publisher(Bool, '~/planning_ready', state_qos)
        self.localization_available_pub = self.create_publisher(Bool, '~/localization_available', state_qos)
        self.status_pub = self.create_publisher(String, '~/status', state_qos)
        self.command_pub = self.create_publisher(Twist, self.values['output_cmd_vel'], 1)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.results = deque(maxlen=20)
        self.candidate_results = deque(maxlen=20)
        self.candidate_stamp = None
        self.lio_poses = deque(maxlen=2000)
        self.valid_stamp = None
        self.status_received = None
        self.map_received = None
        self.latest_map_ready = None
        self.odom_received = None
        self.terrain_received = None
        self.latest_odom_stamp = None
        self.latest_terrain_stamp = None
        self.accepted_transform = None
        self.ready = False
        self.command_active = False
        self.last_command_at = None
        self.reason = 'waiting_for_map_and_localization'
        self.obstacle_health = PlannerObstacleHealth(
            timeout=self.values['live_obstacle_freshness_timeout'], heartbeat_timeout=1.0)
        self.localization_reason = 'alignment_not_confirmed'
        self.latest_path = None
        self.path_generation = 0
        self.last_ready_transform = None
        self.recovery_started = None
        self.recovery_transform = None
        self.recovery_future = None
        self.recovery_request_at = 0.0
        self.recovery_path_generation = 0
        self.recovery_retry_at = 0.0
        self.validation_client = self.create_client(
            ValidateTerrainPath, '/terrain/is_path_valid_detailed')
        # Humble's Python Buffer has no automatic ROS-clock rewind handler.
        # Register against this node's clock rather than assuming a newer API.
        self.clock_jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True),
            post_callback=self.on_clock_jump)
        self.input_subscriptions = [
            self.create_subscription(DiagnosticArray, '/relocalization/status', self.on_status, state_qos),
            self.create_subscription(TransformStamped, '/relocalization/map_to_odom', self.on_result, 10),
            self.create_subscription(TransformStamped, '/relocalization/candidate_map_to_odom',
                                     self.on_candidate, 10),
            self.create_subscription(Bool, '/offline_map/ready', self.on_map, state_qos),
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data),
            self.create_subscription(Odometry, '/lio/odom_base', self.on_lio, qos_profile_sensor_data),
            self.create_subscription(PointCloud2, '/terrain/obstacles', self.on_terrain, qos_profile_sensor_data),
            self.create_subscription(String, '/planner/live_obstacle_status', self.on_obstacle_health, state_qos),
            self.create_subscription(Path, '/plan', self.on_plan, 1),
            self.create_subscription(Twist, self.values['input_cmd_vel'], self.on_command, 1),
        ]
        self.navigation_guard = NavigationGuard(
            self, self.values['command_timeout'], lambda: self.ready,
            on_unready_work=self.on_unready_navigation_work)
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.set_ready(False, self.reason, force=True)

    def on_plan(self, message):
        self.latest_path = message
        self.path_generation += 1

    def on_unready_navigation_work(self):
        # An action status may arrive AFTER an idle input outage. Apply the
        # same policy as timer/command callbacks instead of canceling on !ready.
        valid, reason = self.readiness()
        self.set_ready(False, self.reason if valid else reason)

    def discard_input(self, reason):
        # Invalid or out-of-order samples do not renew any freshness deadline.
        # The timer will stop motion when the last GOOD sample expires.
        self.get_logger().warning('Discarding input: ' + reason, throttle_duration_sec=1.0)

    def clear_recovery_request(self):
        if self.recovery_future is not None:
            self.validation_client.remove_pending_request(self.recovery_future)
            self.recovery_future.cancel()
        self.recovery_future = None

    def abort_recovery(self, reason):
        self.clear_recovery_request()
        self.recovery_started = self.recovery_transform = None
        self.navigation_guard.pause()
        self.get_logger().warning('Input recovery canceled: ' + reason)
        self.set_ready(False, reason, manage_outage=False)

    def poll_recovery(self, inputs_ready):
        if not self.navigation_guard.holding:
            self.clear_recovery_request()
            self.recovery_started = self.recovery_transform = None
            return
        now = time.monotonic()
        if now - self.recovery_started >= self.values['input_recovery_timeout']:
            self.abort_recovery('navigation_input_recovery_timeout')
            return
        if not inputs_ready:
            self.clear_recovery_request()
            return
        # Reconfirmation after a timeout must not silently move the world frame.
        if self.recovery_transform is None or self.accepted_transform is None:
            self.abort_recovery('navigation_recovery_alignment_missing')
            return
        before = matrix_from_pose(self.recovery_transform.transform.translation,
                                  self.recovery_transform.transform.rotation)
        after = matrix_from_pose(self.accepted_transform.transform.translation,
                                 self.accepted_transform.transform.rotation)
        translation = np.linalg.norm(before[:3, 3] - after[:3, 3])
        angle = rotation_distance(before, after)
        if translation > self.values['correction_translation'] or angle > self.values['correction_rotation']:
            self.abort_recovery('navigation_recovery_alignment_jump')
            return
        if self.recovery_future is not None:
            if self.recovery_path_generation != self.path_generation:
                self.clear_recovery_request()
            elif now - self.recovery_request_at >= .8:
                self.clear_recovery_request()
                self.recovery_retry_at = now + .2
            elif self.recovery_future.done():
                try:
                    result = self.recovery_future.result()
                    valid = result.status == result.VALID and result.is_valid
                except Exception:
                    valid = False
                self.clear_recovery_request()
                self.recovery_retry_at = now + .2
                if valid:
                    self.navigation_guard.end_hold()
                    self.recovery_started = self.recovery_transform = None
                    self.get_logger().info('Input recovery complete; current path revalidated')
                    return
            else:
                return
        if self.latest_path is None or len(self.latest_path.poses) < 2 or now < self.recovery_retry_at:
            return
        if self.validation_client.service_is_ready():
            request = ValidateTerrainPath.Request(path=copy.deepcopy(self.latest_path))
            try:
                self.recovery_future = self.validation_client.call_async(request)
                self.recovery_request_at = now
                self.recovery_path_generation = self.path_generation
            except Exception:
                self.recovery_retry_at = now + .2

    def invalidate(self, reason):
        self.gate.reset()
        self.accepted_transform = None
        self.valid_stamp = None
        self.results.clear()
        self.candidate_results.clear()
        self.candidate_stamp = None
        self.localization_reason = reason
        if 'clock_rewound' in reason:
            self.lio_poses.clear()
        self.set_ready(False, reason)

    def on_clock_jump(self, _jump):
        self.tf_buffer.clear()
        self.navigation_guard.on_clock_jump()
        self.odom_received = self.terrain_received = None
        self.latest_odom_stamp = self.latest_terrain_stamp = None
        self.obstacle_health.reset()
        self.invalidate('ros_clock_rewound_or_changed')

    def on_status(self, message):
        if len(message.status) != 1:
            self.discard_input('invalid_localization_diagnostics')
            return
        values = {item.key: item.value for item in message.status[0].values}
        if values.get('valid') not in ('true', 'false') or not values.get('reason'):
            self.discard_input('invalid_localization_diagnostics')
            return
        if values.get('valid') != 'true':
            # Pre-confirm only explicitly qualified, same-stamp candidates.
            # Never expose TF or velocity until upstream also declares valid.
            if values.get('reason') == 'awaiting_distinct_consistent_frames':
                try:
                    stamp = float(values.get('qualified_candidate_stamp', '-1'))
                    age = self.get_clock().now().nanoseconds * 1e-9 - stamp
                    if math.isfinite(stamp) and stamp > 0 and 0 <= age <= self.values['max_alignment_age']:
                        self.status_received = time.monotonic()
                        self.valid_stamp = None
                        self.accepted_transform = None
                        self.candidate_stamp = stamp
                        self.localization_reason = 'localization_awaiting_distinct_consistent_frames'
                        self.set_ready(False, self.localization_reason)
                        self.try_accept()
                        return
                except ValueError:
                    pass
                self.discard_input('invalid_candidate_timestamp')
                return
            self.status_received = time.monotonic()
            self.invalidate('localization_' + values.get('reason', 'invalid'))
            return
        try:
            verified = float(values['last_verified_stamp'])
            if not math.isfinite(verified) or verified <= 0:
                raise ValueError()
            now = self.get_clock().now().nanoseconds * 1e-9
            if verified > now or verified < self.gate.last_stamp:
                self.discard_input('out_of_order_or_future_verified_timestamp')
                return
            self.status_received = time.monotonic()
            self.valid_stamp = verified
            self.candidate_stamp = None
            self.try_accept()
        except (KeyError, ValueError):
            self.discard_input('invalid_verified_timestamp')

    def on_result(self, message):
        self.receive_alignment(message, candidate=False)

    def on_candidate(self, message):
        self.receive_alignment(message, candidate=True)

    def receive_alignment(self, message, candidate):
        if message.header.frame_id != 'map' or message.child_frame_id != 'odom':
            self.discard_input('invalid_alignment_frames')
            return
        if not valid_stamp(message.header.stamp):
            self.discard_input('invalid_alignment_timestamp')
            return
        try:
            matrix_from_pose(message.transform.translation, message.transform.rotation)
        except ValueError:
            self.discard_input('invalid_alignment_pose')
            return
        stamp = seconds(message.header.stamp)
        if stamp > self.get_clock().now().nanoseconds * 1e-9 or stamp < self.gate.last_stamp:
            self.discard_input('out_of_order_or_future_alignment')
            return
        (self.candidate_results if candidate else self.results).append(message)
        self.try_accept()

    def on_map(self, message):
        self.latest_map_ready = message.data
        self.map_received = time.monotonic() if message.data else None
        if not message.data:
            self.set_ready(False, 'map_not_ready')

    def on_odom(self, message):
        if message.header.frame_id != 'odom' or message.child_frame_id != 'base_link':
            self.discard_input('invalid_ekf_frames')
            return
        if not valid_stamp(message.header.stamp):
            self.discard_input('invalid_ekf_timestamp')
            return
        try:
            matrix_from_pose(message.pose.pose.position, message.pose.pose.orientation)
        except ValueError:
            self.discard_input('invalid_ekf_pose')
            return
        stamp = seconds(message.header.stamp)
        if stamp > self.get_clock().now().nanoseconds * 1e-9 or (
                self.latest_odom_stamp is not None and stamp <= self.latest_odom_stamp):
            self.discard_input('duplicate_out_of_order_or_future_ekf')
            return
        self.odom_received = time.monotonic()
        self.latest_odom_stamp = stamp

    def on_lio(self, message):
        if message.header.frame_id != 'odom' or message.child_frame_id != 'base_link':
            self.discard_input('invalid_lio_frames')
            return
        if not valid_stamp(message.header.stamp):
            self.discard_input('invalid_lio_timestamp')
            return
        try:
            matrix_from_pose(message.pose.pose.position, message.pose.pose.orientation)
        except ValueError:
            self.discard_input('invalid_lio_pose')
            return
        stamp = seconds(message.header.stamp)
        if stamp > self.get_clock().now().nanoseconds * 1e-9 or (
                self.lio_poses and stamp <= seconds(self.lio_poses[-1].header.stamp)):
            self.discard_input('duplicate_out_of_order_or_future_lio')
            return
        self.lio_poses.append(message)
        while self.lio_poses and seconds(self.lio_poses[0].header.stamp) < stamp - 10:
            self.lio_poses.popleft()
        self.try_accept()

    def on_terrain(self, message):
        if message.header.frame_id != 'base_link' or not valid_stamp(message.header.stamp):
            self.discard_input('invalid_live_terrain_header')
            return
        stamp = seconds(message.header.stamp)
        if stamp > self.get_clock().now().nanoseconds * 1e-9 or (
                self.latest_terrain_stamp is not None and stamp <= self.latest_terrain_stamp):
            self.discard_input('duplicate_out_of_order_or_future_terrain')
            return
        self.terrain_received = time.monotonic()
        self.latest_terrain_stamp = stamp

    def on_obstacle_health(self, message):
        self.obstacle_health.observe(message.data, self.get_clock().now().nanoseconds)
        valid, reason = self.obstacle_health.readiness(self.get_clock().now().nanoseconds)
        if not valid:
            self.set_ready(False, reason)

    def try_accept(self):
        release = self.valid_stamp is not None
        qualified_stamp = self.valid_stamp if release else self.candidate_stamp
        if qualified_stamp is None:
            return
        candidates = [item for item in (self.results if release else self.candidate_results)
                      if abs(seconds(item.header.stamp) - qualified_stamp) < 2e-6]
        if not candidates:
            return
        message = candidates[-1]
        stamp = seconds(message.header.stamp)
        if stamp < self.gate.last_stamp:
            return
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if stamp <= 0 or not 0 <= age <= self.values['max_alignment_age']:
            self.invalidate('alignment_clock_rewound' if age < 0 else 'alignment_expired')
            return
        # LIO and the EKF intentionally share odom, but their estimates differ.
        # Reject large planar disagreement rather than silently mixing frames.
        if not self.lio_poses:
            return
        lio = min(self.lio_poses, key=lambda item: abs(seconds(item.header.stamp) - stamp))
        if abs(seconds(lio.header.stamp) - stamp) > 0.05:
            return
        try:
            ekf = self.tf_buffer.lookup_transform('odom', 'base_link', Time.from_msg(message.header.stamp))
            lio_pose = matrix_from_pose(lio.pose.pose.position, lio.pose.pose.orientation)
            ekf_pose = matrix_from_pose(ekf.transform.translation, ekf.transform.rotation)
            xy = np.linalg.norm(lio_pose[:2, 3] - ekf_pose[:2, 3])
            lio_yaw = math.atan2(lio_pose[1, 0], lio_pose[0, 0])
            ekf_yaw = math.atan2(ekf_pose[1, 0], ekf_pose[0, 0])
            yaw = abs(math.remainder(lio_yaw - ekf_yaw, 2 * math.pi))
            if xy > self.values['maximum_lio_ekf_xy_difference'] or yaw > self.values['maximum_lio_ekf_yaw_difference']:
                self.get_logger().warning(
                    'LIO/EKF planar mismatch: xy=%.4fm limit=%.4fm yaw=%.4frad limit=%.4frad '
                    'alignment_stamp=%.9f lio_stamp=%.9f ekf_tf_stamp=%.9f lio_dt=%.4fs' % (
                        xy, self.values['maximum_lio_ekf_xy_difference'],
                        yaw, self.values['maximum_lio_ekf_yaw_difference'], stamp,
                        seconds(lio.header.stamp), seconds(ekf.header.stamp),
                        seconds(lio.header.stamp) - stamp), throttle_duration_sec=1.0)
                self.invalidate('lio_ekf_planar_disagreement')
                return
        except TransformException:
            return  # Retry on the timer after same-stamp EKF TF arrives.
        except ValueError:
            self.invalidate('invalid_lio_pose')
            return
        matrix = matrix_from_pose(message.transform.translation, message.transform.rotation)
        if stamp == self.gate.last_stamp:
            confirmed = self.gate.accepted is not None and np.allclose(
                matrix, self.gate.accepted, rtol=0, atol=1e-9)
        else:
            confirmed = self.gate.observe(stamp, matrix)
        if confirmed and release:
            self.accepted_transform = copy.deepcopy(message)
            self.localization_reason = 'ready'
        else:
            self.accepted_transform = None
            self.localization_reason = ('confirming_alignment_or_correction' if release else
                                        'localization_awaiting_distinct_consistent_frames')
            self.set_ready(False, self.localization_reason)

    def localization_available(self):
        """A valid TF remains useful when live terrain temporarily stops."""
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        now_wall = time.monotonic()
        if self.status_received is None or not 0 <= now_wall - self.status_received < self.values['max_alignment_age']:
            if self.accepted_transform is not None or self.gate.count:
                self.invalidate('localization_heartbeat_missing')
            return False, 'localization_heartbeat_missing'
        if self.accepted_transform is None:
            return False, self.localization_reason
        age = now_ros - seconds(self.accepted_transform.header.stamp)
        if not 0 <= age <= self.values['max_alignment_age']:
            reason = 'alignment_clock_rewound' if age < 0 else 'alignment_expired'
            self.invalidate(reason)
            return False, reason
        if self.odom_received is None or not 0 <= now_wall - self.odom_received < self.values['max_odometry_age']:
            return False, 'odometry_missing'
        if self.latest_odom_stamp is None or not 0 < self.latest_odom_stamp <= now_ros or \
                now_ros - self.latest_odom_stamp > self.values['max_odometry_age']:
            return False, 'odometry_stamp_stale'
        return True, 'ready'

    def readiness(self):
        # An explicit map rejection is not just a late heartbeat. Keep its
        # meaning if action status arrives after this callback.
        if self.latest_map_ready is False:
            return False, 'map_not_ready'
        valid, reason = self.localization_available()
        if not valid:
            return False, reason
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        now_wall = time.monotonic()
        for received, limit, reason in (
                (self.map_received, 2.0, 'map_heartbeat_missing'),
                (self.terrain_received, self.values['max_terrain_age'], 'live_terrain_missing')):
            if received is None or not 0 <= now_wall - received < limit:
                return False, reason
        stamp = self.latest_terrain_stamp
        if stamp is None or stamp <= 0 or not 0 <= now_ros - stamp <= self.values['max_terrain_age']:
            return False, 'terrain_stamp_stale'
        valid, reason = self.obstacle_health.readiness(self.get_clock().now().nanoseconds)
        if not valid:
            return False, reason
        return True, 'ready'

    def set_ready(self, ready, reason, force=False, manage_outage=True):
        if not ready and reason in (
                'global_obstacle_localization_unavailable',
                'global_obstacle_localization_heartbeat_missing',
                'global_obstacle_localization_heartbeat_stale'):
            # The planner mirrors OUR availability. Apply this normalization
            # for callbacks, timer and command paths alike: a reflected generic
            # status must not erase the actual localization failure.
            localized, local_reason = self.localization_available()
            if not localized:
                reason = local_reason
            else:
                # The planner may not yet have consumed our new true bit.
                # Wait for a fresh report, even when this is the FIRST fault;
                # never use a reflected bit to override a local hard failure.
                if reason == 'global_obstacle_localization_unavailable':
                    reason = 'global_obstacle_localization_reflection_pending'
        was_ready = self.ready
        if not ready and manage_outage:
            # A second outage invalidates even a completed, unconsumed result.
            self.clear_recovery_request()
            self.planning_ready_pub.publish(Bool(data=False))
            if (was_ready or self.navigation_guard.holding or (
                    not self.navigation_guard.blocked
                    and self.navigation_guard.first_ready_stamp is not None
                    and self.navigation_guard.has_navigation_work())):
                recoverable = reason in self.recoverable_reasons or (
                    self.navigation_guard.holding and
                    reason == 'localization_awaiting_distinct_consistent_frames')
                idle = not (self.command_active or self.navigation_guard.has_navigation_work()
                            or self.navigation_guard.blocked)
                if recoverable and idle:
                    # A standalone planning request is not a navigation task.
                    # It may leave /plan behind, but that path must not become
                    # a recovery prerequisite while the recorded robot moves.
                    self.recovery_started = self.recovery_transform = None
                    self.get_logger().warning('Navigation inputs unavailable while idle: reason=' + reason)
                elif recoverable and self.navigation_guard.begin_hold():
                    if self.recovery_started is None:
                        self.recovery_started = time.monotonic()
                        self.recovery_transform = copy.deepcopy(self.last_ready_transform or self.accepted_transform)
                        self.recovery_retry_at = 0.0
                        self.get_logger().warning('Navigation inputs unavailable: reason=' + reason +
                            '; stopping velocity while holding the task for bounded recovery')
                else:
                    self.clear_recovery_request()
                    self.recovery_started = self.recovery_transform = None
                    self.navigation_guard.pause()
                    self.get_logger().warning('Navigation gate lost readiness: reason=' + reason +
                        '; stopping velocity and requesting cancellation of active tasks')
        if ready:
            self.last_ready_transform = copy.deepcopy(self.accepted_transform)
            if not was_ready:
                self.get_logger().info('Navigation gate ready')
        if force or self.ready != ready or self.reason != reason:
            self.ready_pub.publish(Bool(data=ready))
            self.status_pub.publish(String(data=reason))
        self.ready, self.reason = ready, reason
        if not ready and self.command_active:
            self.command_pub.publish(Twist())
            self.command_active = False

    def on_command(self, message):
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        finite = all(math.isfinite(value) for value in values)
        nonzero = finite and any(value != 0.0 for value in values)
        self.navigation_guard.note_input(nonzero)
        ready, reason = self.readiness()
        if not ready:
            self.set_ready(False, reason)
        motion_ready, motion_reason = self.navigation_guard.poll()
        if not motion_ready:
            self.set_ready(False, motion_reason, manage_outage=False)
        if not ready or not motion_ready or not self.ready or not finite:
            if self.command_active:
                self.command_pub.publish(Twist())
                self.command_active = False
            return
        self.command_pub.publish(message)
        self.last_command_at = time.monotonic()
        self.command_active = nonzero
        self.navigation_guard.note_forwarded(nonzero)

    def tick(self):
        self.try_accept()
        localized, _ = self.localization_available()
        # Publish usable TF before announcing readiness to the planner.
        if localized:
            transform = copy.deepcopy(self.accepted_transform)
            transform.header.stamp = self.get_clock().now().to_msg()
            self.broadcaster.sendTransform(transform)
        # Bootstrap input processing without requiring the final motion gate.
        self.localization_available_pub.publish(Bool(data=localized))
        ready, reason = self.readiness()
        if not ready:
            self.set_ready(False, reason)
        self.planning_ready_pub.publish(Bool(data=ready and not self.navigation_guard.blocked))
        self.poll_recovery(ready)
        motion_ready, motion_reason = self.navigation_guard.poll()
        if ready and not motion_ready:
            ready, reason = False, motion_reason
        if ready:
            self.navigation_guard.note_ready()
        self.set_ready(ready, reason, manage_outage=False)
        # Repeat readiness as a heartbeat; the planner can detect adapter death.
        self.ready_pub.publish(Bool(data=ready))
        if self.command_active and time.monotonic() - self.last_command_at >= self.values['command_timeout']:
            self.command_pub.publish(Twist())
            self.command_active = False


def main():
    rclpy.init()
    node = None
    try:
        node = LocalizationAdapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.set_ready(False, 'adapter_shutdown', force=True)
                node.navigation_guard.poll()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
