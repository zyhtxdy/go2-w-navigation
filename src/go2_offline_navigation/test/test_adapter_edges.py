"""Fault injection for offline localization, isolated from live ROS domains."""
import copy
import json
import math
import os
import time

os.environ['ROS_DOMAIN_ID'] = '110'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
for key in ('CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
            'ROS_DISCOVERY_SERVER', 'ROS_STATIC_PEERS', 'ROS_AUTOMATIC_DISCOVERY_RANGE'):
    os.environ.pop(key, None)

import numpy as np
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.time import Time

from go2_offline_navigation.alignment import AlignmentGate
from go2_offline_navigation.localization_adapter import LocalizationAdapter


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))

    sendTransform = publish


@pytest.fixture
def adapter():
    rclpy.init()
    try:
        node = LocalizationAdapter()
    except Exception:
        rclpy.shutdown()
        raise
    node.broadcaster = Capture()
    node.command_pub = Capture()
    node.status_pub = Capture()
    node.ready_pub = Capture()
    node.planning_ready_pub = Capture()
    node.localization_available_pub = Capture()
    now_ros = node.get_clock().now().nanoseconds
    now_wall = time.monotonic()
    node.map_received = node.status_received = node.odom_received = node.terrain_received = now_wall
    node.latest_map_ready = True
    node.latest_odom_stamp = node.latest_terrain_stamp = now_ros * 1e-9
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
    transform.header.stamp = Time(nanoseconds=now_ros - 100000000).to_msg()
    transform.transform.rotation.w = 1.0
    node.accepted_transform = transform
    node.gate.accepted = np.eye(4)
    node.gate.last_stamp = now_ros * 1e-9 - .1
    node.obstacle_health.observe(json.dumps(dict(ready=True, consumed_stamp_ns=now_ros, processed_age_s=0.0)), now_ros)
    node.ready = True
    node.navigation_guard.note_ready()  # This fixture represents an already-running stack.
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_live_terrain_loss_stops_velocity_but_preserves_valid_localization_tf(adapter):
    adapter.command_active = True
    adapter.last_command_at = time.monotonic()
    adapter.terrain_received = time.monotonic() - 2
    adapter.tick()
    assert not adapter.ready
    assert adapter.command_pub.messages[-1].linear.x == 0
    assert adapter.broadcaster.messages, 'Valid map alignment should still be visible during terrain outage'


@pytest.mark.parametrize('reason', ['global_obstacle_source_stale', 'terrain_stamp_stale',
                                  'odometry_stamp_stale'])
def test_planning_only_outage_recovers_without_revalidating_old_plan(adapter, reason):
    from nav_msgs.msg import Path
    adapter.latest_path = Path()  # Old/empty /plan is irrelevant with no task.
    adapter.set_ready(False, reason)
    assert not adapter.ready
    assert not adapter.navigation_guard.holding
    assert not adapter.navigation_guard.blocked
    assert adapter.recovery_started is None
    adapter.tick()
    assert adapter.ready
    assert not adapter.navigation_guard.pending
    assert not any(message.linear.x or message.angular.z for message in adapter.command_pub.messages)


def test_global_only_outage_stops_commands_and_preserves_localization(adapter):
    adapter.command_active = True
    adapter.last_command_at = time.monotonic()
    adapter.obstacle_health.received_at = time.monotonic() - 2
    adapter.tick()
    assert not adapter.ready and 'global_obstacle' in adapter.reason
    assert adapter.command_pub.messages[-1].linear.x == 0
    assert adapter.broadcaster.messages
    assert adapter.localization_available_pub.messages[-1].data is True


def test_explicit_planner_failure_cancels_and_fresh_data_does_not_bypass_pending_cancellation(adapter):
    adapter.command_active = True
    adapter.navigation_guard.note_forwarded(True)
    now = adapter.get_clock().now().nanoseconds
    adapter.on_obstacle_health(String(data=json.dumps(dict(ready=False,
        consumed_stamp_ns=now, processed_age_s=0., reason='invalid_obstacle_cloud'))))
    assert not adapter.ready and adapter.navigation_guard.blocked
    assert adapter.navigation_guard.pending  # Cancellation must finish before motion resumes.
    assert adapter.command_pub.messages[-1].linear.x == 0
    adapter.on_obstacle_health(String(data=json.dumps(dict(ready=True,
        consumed_stamp_ns=adapter.get_clock().now().nanoseconds, processed_age_s=0.))))
    adapter.tick()
    command = Twist()
    command.linear.x = .2
    adapter.on_command(command)
    assert not adapter.ready
    assert not any(message.linear.x for message in adapter.command_pub.messages)


class ValidationClient:
    def __init__(self):
        self.requests = []
        self.available = True

    def service_is_ready(self):
        return self.available

    def call_async(self, request):
        from rclpy.task import Future
        future = Future()
        self.requests.append((request, future))
        return future

    def remove_pending_request(self, future):
        pass


def begin_input_outage(adapter, reason='global_obstacle_source_stale'):
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    adapter.validation_client = ValidationClient()
    path = Path()
    path.header.frame_id = 'map'
    path.poses = [PoseStamped(), PoseStamped()]
    path.poses[1].pose.position.x = 1.
    adapter.on_plan(path)
    adapter.command_active = True
    adapter.navigation_guard.note_forwarded(True)
    adapter.set_ready(False, reason)
    assert adapter.navigation_guard.holding and not adapter.navigation_guard.blocked
    assert adapter.command_pub.messages[-1].linear.x == 0


def valid_validation(future):
    from terrain_navigation_interfaces.srv import ValidateTerrainPath
    future.set_result(ValidateTerrainPath.Response(is_valid=True,
                                                 status=ValidateTerrainPath.Response.VALID))


def test_transient_outage_waits_for_correlated_validation_before_new_velocity(adapter):
    begin_input_outage(adapter)
    # Data may recover before the BT sees the outage. The old velocity stream
    # must still be blocked until our own current-path request completes.
    command = Twist()
    command.linear.x = .2
    adapter.on_command(command)
    adapter.tick()
    assert adapter.planning_ready_pub.messages[-1].data is True
    assert not adapter.ready and adapter.navigation_guard.holding
    assert len(adapter.validation_client.requests) == 1
    adapter.on_command(command)
    assert not any(m.linear.x for m in adapter.command_pub.messages)
    valid_validation(adapter.validation_client.requests[-1][1])
    adapter.tick()
    assert adapter.ready and not adapter.navigation_guard.holding
    assert not any(m.linear.x for m in adapter.command_pub.messages), 'Never replay a blocked command'
    adapter.on_command(command)
    assert adapter.command_pub.messages[-1].linear.x == .2


def test_path_replaced_during_validation_cannot_release_old_result(adapter):
    begin_input_outage(adapter)
    adapter.tick()
    old = adapter.validation_client.requests[-1][1]
    path = copy.deepcopy(adapter.latest_path)
    path.poses[-1].pose.position.x = 2.
    adapter.on_plan(path)
    valid_validation(old)
    adapter.tick()
    assert not adapter.ready and adapter.navigation_guard.holding
    assert len(adapter.validation_client.requests) == 2
    assert adapter.validation_client.requests[-1][0].path.poses[-1].pose.position.x == 2.
    valid_validation(adapter.validation_client.requests[-1][1])
    adapter.tick()
    assert adapter.ready


def test_second_outage_discards_completed_but_unconsumed_validation(adapter):
    begin_input_outage(adapter)
    adapter.tick()
    valid_validation(adapter.validation_client.requests[-1][1])
    adapter.set_ready(False, 'global_obstacle_source_stale')
    adapter.tick()
    assert not adapter.ready and adapter.navigation_guard.holding
    assert len(adapter.validation_client.requests) == 2
    valid_validation(adapter.validation_client.requests[-1][1])
    adapter.tick()
    assert adapter.ready


def test_invalid_path_during_recovery_never_opens_motion(adapter):
    from terrain_navigation_interfaces.srv import ValidateTerrainPath
    begin_input_outage(adapter)
    adapter.tick()
    adapter.validation_client.requests[-1][1].set_result(ValidateTerrainPath.Response(
        is_valid=False, status=ValidateTerrainPath.Response.COLLISION, reason='footprint_collision'))
    adapter.tick()
    assert not adapter.ready and adapter.navigation_guard.holding
    assert not any(m.linear.x for m in adapter.command_pub.messages)


def test_recovery_timeout_cancels_even_when_health_recovers(adapter):
    begin_input_outage(adapter)
    adapter.tick()
    adapter.recovery_started -= adapter.values['input_recovery_timeout'] + .01
    adapter.tick()
    assert adapter.navigation_guard.blocked and not adapter.navigation_guard.holding
    assert not adapter.ready


def test_repeated_or_changed_missing_inputs_cannot_extend_recovery_deadline(adapter):
    begin_input_outage(adapter)
    started = adapter.recovery_started
    adapter.set_ready(False, 'live_terrain_missing')
    assert adapter.recovery_started == started
    adapter.recovery_started -= adapter.values['input_recovery_timeout'] + .01
    adapter.terrain_received = time.monotonic() - 2
    adapter.tick()
    assert adapter.navigation_guard.blocked and not adapter.ready


def test_alignment_jump_after_reconfirmation_cancels_instead_of_resuming(adapter):
    begin_input_outage(adapter, 'localization_global_verification_timeout')
    adapter.accepted_transform.transform.translation.x += .3
    adapter.tick()
    assert adapter.navigation_guard.blocked and not adapter.navigation_guard.holding
    assert not adapter.validation_client.requests


def test_hard_fault_during_hold_cancels_immediately(adapter):
    begin_input_outage(adapter)
    adapter.set_ready(False, 'invalid_alignment_frames')
    assert adapter.navigation_guard.blocked and not adapter.navigation_guard.holding


def test_validation_failure_or_missing_service_stays_stopped(adapter):
    begin_input_outage(adapter)
    adapter.validation_client.available = False
    adapter.tick()
    assert not adapter.ready and not adapter.validation_client.requests
    adapter.validation_client.available = True
    adapter.tick()
    adapter.validation_client.requests[-1][1].set_exception(RuntimeError('RPC failed'))
    adapter.tick()
    assert not adapter.ready and adapter.navigation_guard.holding


def test_path_validation_rpc_timeout_cannot_unlock_motion(adapter):
    begin_input_outage(adapter)
    adapter.tick()
    future = adapter.validation_client.requests[-1][1]
    adapter.recovery_request_at -= .81
    adapter.tick()
    assert future.cancelled()
    assert not adapter.ready and adapter.navigation_guard.holding


def test_clock_rewind_during_hold_requires_old_task_cancellation(adapter):
    begin_input_outage(adapter)
    adapter.on_clock_jump(None)
    assert adapter.navigation_guard.blocked and not adapter.navigation_guard.holding
    assert not adapter.ready and adapter.accepted_transform is None


@pytest.mark.parametrize('status,valid', [(0, False), (1, True), (255, True)])
def test_inconsistent_or_unknown_validation_response_never_releases(adapter, status, valid):
    from terrain_navigation_interfaces.srv import ValidateTerrainPath
    begin_input_outage(adapter)
    adapter.tick()
    adapter.validation_client.requests[-1][1].set_result(
        ValidateTerrainPath.Response(status=status, is_valid=valid))
    adapter.tick()
    assert not adapter.ready and adapter.navigation_guard.holding
    assert not any(m.linear.x for m in adapter.command_pub.messages)


@pytest.mark.parametrize('missing', ['localization', 'odom', 'terrain', 'obstacles', 'map'])
def test_valid_rpc_cannot_override_newly_missing_input(adapter, missing):
    begin_input_outage(adapter)
    adapter.tick()
    valid_validation(adapter.validation_client.requests[-1][1])
    if missing == 'localization':
        adapter.status_received -= 4.
    elif missing == 'odom':
        adapter.odom_received -= 2.
    elif missing == 'terrain':
        adapter.terrain_received -= 2.
    elif missing == 'obstacles':
        adapter.obstacle_health.received_at -= 2.
    else:
        adapter.map_received -= 3.
    adapter.tick()
    assert not adapter.ready
    assert not any(m.linear.x for m in adapter.command_pub.messages)


@pytest.mark.parametrize('kind,amount,allowed', [
    ('translation', .199, True), ('translation', .201, False),
    ('rotation', .149, True), ('rotation', .151, False)])
def test_recovery_alignment_limits_on_both_sides(adapter, kind, amount, allowed):
    begin_input_outage(adapter)
    adapter.tick()
    if kind == 'translation':
        adapter.accepted_transform.transform.translation.x = amount
    else:
        adapter.accepted_transform.transform.rotation.z = math.sin(amount/2)
        adapter.accepted_transform.transform.rotation.w = math.cos(amount/2)
    valid_validation(adapter.validation_client.requests[-1][1])
    adapter.tick()
    assert adapter.ready is allowed
    if not allowed:
        assert adapter.navigation_guard.blocked
    assert not any(m.linear.x for m in adapter.command_pub.messages), 'No cached command replay'


def test_completed_validation_cannot_survive_hard_localization_fault(adapter):
    begin_input_outage(adapter)
    adapter.tick()
    valid_validation(adapter.validation_client.requests[-1][1])
    adapter.invalidate('invalid_alignment_frames')
    adapter.tick()
    command = Twist()
    command.linear.x = .2
    adapter.on_command(command)
    assert adapter.navigation_guard.blocked
    assert not adapter.ready and adapter.accepted_transform is None
    assert not any(m.linear.x for m in adapter.command_pub.messages)


def test_missing_path_keeps_recovery_stopped_until_deadline(adapter):
    from nav_msgs.msg import Path
    begin_input_outage(adapter)
    adapter.on_plan(Path())
    adapter.tick()
    assert not adapter.ready and not adapter.validation_client.requests
    adapter.recovery_started -= adapter.values['input_recovery_timeout'] + .01
    adapter.tick()
    assert adapter.navigation_guard.blocked
    assert not any(m.linear.x for m in adapter.command_pub.messages)


def test_new_plan_after_empty_path_requires_its_own_validation(adapter):
    from nav_msgs.msg import Path
    begin_input_outage(adapter)
    valid_path = copy.deepcopy(adapter.latest_path)
    adapter.on_plan(Path())
    adapter.tick()
    assert not adapter.validation_client.requests
    adapter.on_plan(valid_path)
    adapter.tick()
    assert len(adapter.validation_client.requests) == 1 and not adapter.ready
    valid_validation(adapter.validation_client.requests[-1][1])
    adapter.tick()
    assert adapter.ready


def test_paused_ros_clock_does_not_hide_global_consumption_stall(adapter):
    from rclpy.parameter import Parameter
    adapter.set_parameters([Parameter('use_sim_time', value=True)])
    adapter.get_clock().set_ros_time_override(Time(seconds=100))
    adapter.status_received = adapter.odom_received = adapter.terrain_received = adapter.map_received = time.monotonic()
    adapter.latest_odom_stamp = adapter.latest_terrain_stamp = 100.
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
    transform.header.stamp = Time(seconds=100).to_msg()
    transform.transform.rotation.w = 1.
    adapter.accepted_transform = transform
    adapter.obstacle_health.observe(json.dumps(dict(ready=True, consumed_stamp_ns=100000000000,
                                                   processed_age_s=0.)), 100000000000)
    adapter.obstacle_health.progress_at = time.monotonic() - 2.1
    adapter.ready = adapter.command_active = True
    adapter.last_command_at = time.monotonic()
    adapter.tick()
    assert not adapter.ready and adapter.reason == 'global_obstacle_progress_stale'
    assert adapter.command_pub.messages[-1].linear.x == 0
    assert adapter.broadcaster.messages


def test_invalid_localization_diagnostic_reason_is_retained(adapter):
    status = DiagnosticStatus(name='relocalization', values=[
        KeyValue(key='valid', value='false'), KeyValue(key='reason', value='ambiguous_match')])
    adapter.on_status(DiagnosticArray(status=[status]))
    adapter.tick()
    assert adapter.reason == 'localization_ambiguous_match'
    assert not adapter.broadcaster.messages


def test_command_cannot_enable_navigation_before_tf_publication(adapter):
    adapter.ready = False
    command = Twist()
    command.linear.x = .2
    adapter.on_command(command)
    assert not any(m.linear.x for m in adapter.command_pub.messages)
    adapter.tick()
    adapter.on_command(command)
    assert adapter.broadcaster.messages and adapter.command_pub.messages[-1].linear.x == .2


def test_localization_heartbeat_loss_discards_confirmation(adapter):
    adapter.status_received = time.monotonic() - 4
    adapter.tick()
    assert not adapter.ready
    assert adapter.accepted_transform is None
    assert adapter.gate.count == 0


def test_old_alignment_is_not_refreshed_by_repeated_status_messages(adapter):
    now = adapter.get_clock().now().nanoseconds
    adapter.accepted_transform.header.stamp = Time(nanoseconds=now - 4000000000).to_msg()
    adapter.tick()
    assert not adapter.ready
    assert not adapter.broadcaster.messages


def test_clock_rewind_clears_old_lio_history(adapter):
    old = Odometry()
    old.header.frame_id, old.child_frame_id = 'odom', 'base_link'
    old.pose.pose.orientation.w = 1.0
    adapter.lio_poses.append(old)
    adapter.accepted_transform.header.stamp = Time(
        nanoseconds=adapter.get_clock().now().nanoseconds + 1000000000).to_msg()
    adapter.tick()
    assert not adapter.ready and not adapter.lio_poses


def test_bad_lio_pose_preserves_last_good_alignment(adapter):
    bad = Odometry()
    bad.header.frame_id, bad.child_frame_id = 'odom', 'base_link'
    bad.header.stamp = adapter.get_clock().now().to_msg()
    bad.pose.pose.orientation.w = 1.0
    bad.pose.pose.position.x = float('nan')
    received = adapter.status_received
    adapter.on_lio(bad)
    assert adapter.ready and adapter.accepted_transform is not None
    assert adapter.status_received == received


def test_sparse_confirmation_frames_do_not_count_as_continuous():
    gate = AlignmentGate()
    matrix = np.eye(4)
    assert not gate.observe(1, matrix)
    assert not gate.observe(2, matrix)
    assert not gate.observe(10, matrix)
    assert not gate.observe(11, matrix)
    assert gate.observe(12, matrix)


def test_bad_transform_cannot_leave_a_confirmed_gate_available():
    gate = AlignmentGate()
    for stamp in (1, 2, 3):
        gate.observe(stamp, np.eye(4))
    bad = np.eye(4)
    bad[0, 3] = np.nan
    assert not gate.observe(4, bad)
    assert gate.accepted is None


def test_simulation_clock_rewind_clears_tf_and_confirmation(adapter):
    from rclpy.parameter import Parameter
    adapter.set_parameters([Parameter('use_sim_time', value=True)])
    adapter.get_clock().set_ros_time_override(Time(seconds=10))
    old = TransformStamped()
    old.header.frame_id, old.child_frame_id = 'odom', 'base_link'
    old.header.stamp = Time(seconds=10).to_msg()
    old.transform.rotation.w = 1.0
    adapter.tf_buffer.set_transform(old, 'test')
    assert adapter.tf_buffer.can_transform('odom', 'base_link', Time())
    adapter.ready = True
    adapter.get_clock().set_ros_time_override(Time(seconds=5))
    assert not adapter.tf_buffer.can_transform('odom', 'base_link', Time())
    assert not adapter.ready and adapter.accepted_transform is None


def test_lio_history_is_bounded_by_time_and_ignores_duplicates(adapter):
    end = adapter.get_clock().now().nanoseconds
    for index in range(201):
        message = Odometry()
        message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
        message.header.stamp = Time(nanoseconds=end - 20000000000 + index * 100000000).to_msg()
        message.pose.pose.orientation.w = 1.0
        adapter.on_lio(message)
    assert len(adapter.lio_poses) <= 102
    size = len(adapter.lio_poses)
    adapter.on_lio(message)
    assert len(adapter.lio_poses) == size


def test_bad_alignment_stamp_is_rejected_before_tf_lookup(adapter):
    bad = TransformStamped()
    bad.header.frame_id, bad.child_frame_id = 'map', 'odom'
    bad.header.stamp.nanosec = 1000000000
    bad.transform.rotation.w = 1.0
    adapter.on_result(bad)
    assert adapter.ready and adapter.accepted_transform is not None
    assert not adapter.results


def confirmation_pair(adapter, stamp_ns, valid, x=0.0):
    """Supply same-time, agreeing LIO/EKF without invoking any hardware."""
    from types import SimpleNamespace
    stamp = Time(nanoseconds=stamp_ns).to_msg()
    identity = TransformStamped()
    identity.header.frame_id, identity.child_frame_id = 'odom', 'base_link'
    identity.header.stamp = stamp
    identity.transform.rotation.w = 1.0
    adapter.tf_buffer = SimpleNamespace(lookup_transform=lambda *args: identity)
    lio = Odometry()
    lio.header.frame_id, lio.child_frame_id = 'odom', 'base_link'
    lio.header.stamp = stamp
    lio.pose.pose.orientation.w = 1.0
    adapter.lio_poses.append(lio)
    candidate = TransformStamped()
    candidate.header.frame_id, candidate.child_frame_id = 'map', 'odom'
    candidate.header.stamp = stamp
    candidate.transform.translation.x = x
    candidate.transform.rotation.w = 1.0
    values = [KeyValue(key='valid', value='true' if valid else 'false'),
              KeyValue(key='reason', value='accepted' if valid else 'awaiting_distinct_consistent_frames'),
              KeyValue(key='last_verified_stamp', value=str(stamp_ns * 1e-9)),
              KeyValue(key='qualified_candidate_stamp', value=str(stamp_ns * 1e-9))]
    adapter.on_candidate(candidate)
    if valid:
        adapter.on_result(candidate)
    adapter.on_status(DiagnosticArray(status=[DiagnosticStatus(values=values)]))
    return candidate


def test_qualified_candidates_overlap_confirmation_without_releasing_tf(adapter):
    adapter.invalidate('test_start')
    end = adapter.get_clock().now().nanoseconds - 100000000
    for index in (2, 1):
        confirmation_pair(adapter, end - index * 200000000, valid=False)
        adapter.tick()
        assert not adapter.ready and adapter.accepted_transform is None
        assert not adapter.broadcaster.messages
    confirmation_pair(adapter, end, valid=True)
    adapter.tick()
    assert adapter.ready and adapter.broadcaster.messages


def test_three_candidates_cannot_enable_tf_without_upstream_valid(adapter):
    adapter.invalidate('test_start')
    end = adapter.get_clock().now().nanoseconds - 100000000
    for index in (2, 1, 0):
        confirmation_pair(adapter, end - index * 200000000, valid=False)
    adapter.tick()
    assert adapter.gate.accepted is not None
    assert not adapter.ready and adapter.accepted_transform is None
    assert not adapter.broadcaster.messages


def test_duplicate_candidates_and_valid_status_do_not_count_as_new_frames(adapter):
    adapter.invalidate('test_start')
    stamp = adapter.get_clock().now().nanoseconds - 100000000
    confirmation_pair(adapter, stamp, valid=False)
    confirmation_pair(adapter, stamp, valid=False)
    confirmation_pair(adapter, stamp, valid=True)
    adapter.tick()
    assert adapter.gate.count == 1
    assert not adapter.ready and not adapter.broadcaster.messages


def test_candidate_jump_and_quality_failure_clear_preconfirmation(adapter):
    adapter.invalidate('test_start')
    end = adapter.get_clock().now().nanoseconds - 100000000
    confirmation_pair(adapter, end - 400000000, valid=False)
    confirmation_pair(adapter, end - 200000000, valid=False)
    confirmation_pair(adapter, end, valid=True, x=2.0)
    assert not adapter.ready and adapter.accepted_transform is None
    assert adapter.gate.count == 1
    status = DiagnosticStatus(values=[KeyValue(key='valid', value='false'),
                                     KeyValue(key='reason', value='low_quality')])
    adapter.on_status(DiagnosticArray(status=[status]))
    assert adapter.gate.count == 0 and adapter.candidate_stamp is None


def test_stale_candidate_diagnostics_cannot_preserve_confirmation(adapter):
    adapter.invalidate('test_start')
    stamp = adapter.get_clock().now().nanoseconds - 4000000000
    confirmation_pair(adapter, stamp, valid=False)
    assert adapter.gate.count == 0 and adapter.candidate_stamp is None


@pytest.mark.parametrize('field,value', [('translation', float('nan')), ('rotation', float('inf')),
                                       ('recovery_frames', 2.5)])
def test_gate_rejects_invalid_parameters(field, value):
    with pytest.raises(ValueError):
        AlignmentGate(**{field: value})


def planner_localization_unavailable(adapter):
    now = adapter.get_clock().now().nanoseconds
    adapter.on_obstacle_health(String(data=json.dumps(dict(
        ready=False, reason='localization_unavailable',
        consumed_stamp_ns=now, processed_age_s=0.))))


def test_reflected_localization_timeout_retains_original_bounded_recovery(adapter):
    begin_input_outage(adapter)
    started = adapter.recovery_started
    adapter.invalidate('localization_global_verification_timeout')
    planner_localization_unavailable(adapter)
    assert adapter.reason == 'localization_global_verification_timeout'
    assert adapter.navigation_guard.holding and not adapter.navigation_guard.blocked
    assert adapter.recovery_started == started
    assert not adapter.ready
    assert not any(m.linear.x or m.angular.z for m in adapter.command_pub.messages)
    # Repeated reflected failures must not restart the finite waiting window.
    planner_localization_unavailable(adapter)
    assert adapter.recovery_started == started
    adapter.recovery_started -= adapter.values['input_recovery_timeout'] + .01
    adapter.tick()
    assert adapter.navigation_guard.blocked and not adapter.navigation_guard.holding


@pytest.mark.parametrize('fault', ['localization_ambiguous_match', 'invalid_alignment_frames',
                                    'lio_ekf_planar_disagreement', 'invalid_alignment_timestamp'])
def test_reflected_hard_localization_fault_still_cancels(adapter, fault):
    begin_input_outage(adapter)
    adapter.invalidate(fault)
    planner_localization_unavailable(adapter)
    assert adapter.navigation_guard.blocked and not adapter.navigation_guard.holding
    assert adapter.reason == fault
    assert not adapter.ready


def test_first_planner_localization_reflection_waits_at_zero_when_local_state_is_good(adapter):
    adapter.command_active = True
    adapter.navigation_guard.note_forwarded(True)
    planner_localization_unavailable(adapter)
    assert adapter.navigation_guard.holding and not adapter.navigation_guard.blocked
    assert adapter.reason == "global_obstacle_localization_reflection_pending"
    assert not adapter.ready
    assert adapter.command_pub.messages[-1].linear.x == 0


def test_adapter_default_global_age_can_relax_without_relaxing_local_data(adapter):
    assert adapter.obstacle_health.timeout == 2.
    assert adapter.obstacle_health.heartbeat_timeout == 1.
    assert adapter.values['max_terrain_age'] == 1.
    assert adapter.values['max_odometry_age'] == .5
    assert adapter.values['command_timeout'] == .3
    assert adapter.values['input_recovery_timeout'] == 10.


def test_reflection_during_localization_reconfirmation_waits_for_fresh_health_and_path(adapter):
    saved = copy.deepcopy(adapter.accepted_transform)
    begin_input_outage(adapter)
    started = adapter.recovery_started
    adapter.invalidate('localization_global_verification_timeout')
    planner_localization_unavailable(adapter)
    # The actual localization then recovers before its mirrored planner bit.
    adapter.accepted_transform = saved
    adapter.localization_reason = 'ready'
    adapter.tick()  # Uses the cached unavailable report, not a new callback.
    assert adapter.navigation_guard.holding and not adapter.navigation_guard.blocked
    assert adapter.recovery_started == started
    assert not adapter.ready
    assert not adapter.validation_client.requests
    now = adapter.get_clock().now().nanoseconds
    adapter.on_obstacle_health(String(data=json.dumps(dict(
        ready=True, consumed_stamp_ns=now, processed_age_s=0.))))
    adapter.tick()
    assert not adapter.ready and len(adapter.validation_client.requests) == 1
    valid_validation(adapter.validation_client.requests[-1][1])
    adapter.tick()
    assert adapter.ready and not adapter.navigation_guard.holding


def test_direct_command_path_also_retains_underlying_localization_timeout(adapter):
    begin_input_outage(adapter)
    adapter.invalidate('localization_tracking_timeout')
    adapter.set_ready(False, 'global_obstacle_localization_unavailable')
    assert adapter.reason == 'localization_tracking_timeout'
    assert adapter.navigation_guard.holding and not adapter.navigation_guard.blocked


@pytest.mark.parametrize('reason', ['localization_heartbeat_missing',
                                  'localization_heartbeat_stale',
                                  'localization_unavailable'])
def test_first_planner_heartbeat_fault_recovers_only_after_path_revalidation(adapter, reason):
    begin_input_outage(adapter, 'global_obstacle_' + reason)
    started = adapter.recovery_started
    # Model the latest observed 0.63-second interruption without sleeping.
    adapter.recovery_started -= .63
    adapter.poll_recovery(False)
    assert adapter.navigation_guard.holding and not adapter.navigation_guard.blocked
    assert adapter.recovery_started == started - .63
    now = adapter.get_clock().now().nanoseconds
    adapter.on_obstacle_health(String(data=json.dumps(dict(
        ready=True, consumed_stamp_ns=now, processed_age_s=0.))))
    adapter.tick()
    assert not adapter.ready and adapter.navigation_guard.holding
    valid_validation(adapter.validation_client.requests[-1][1])
    adapter.tick()
    assert adapter.ready and not adapter.navigation_guard.holding
    assert not any(m.linear.x or m.angular.z for m in adapter.command_pub.messages)


@pytest.mark.parametrize('reason', ['localization_heartbeat_missing',
                                  'localization_heartbeat_stale', 'localization_unavailable'])
def test_planner_heartbeat_cannot_mask_actual_ambiguous_localization(adapter, reason):
    begin_input_outage(adapter)
    adapter.invalidate('localization_ambiguous_match')
    adapter.set_ready(False, 'global_obstacle_' + reason)
    assert adapter.navigation_guard.blocked and not adapter.navigation_guard.holding
    assert adapter.reason == 'localization_ambiguous_match'


def test_recovery_budget_allows_reconfirmation_but_remains_bounded(adapter):
    begin_input_outage(adapter)
    adapter.recovery_started -= 7.0  # Three ~2.3 s registrations need > 3 s.
    started = adapter.recovery_started
    adapter.poll_recovery(False)
    assert adapter.navigation_guard.holding and not adapter.navigation_guard.blocked
    adapter.set_ready(False, 'global_obstacle_localization_heartbeat_stale')
    assert adapter.recovery_started == started
    adapter.recovery_started -= 3.01
    adapter.poll_recovery(False)
    assert adapter.navigation_guard.blocked and not adapter.navigation_guard.holding
    assert adapter.reason == 'navigation_input_recovery_timeout'


@pytest.mark.parametrize('reason', ['no_time_matched_odometry', 'input_stale',
                                  'matching_result_stale', 'confirmation_timeout'])
def test_explicit_missing_localization_inputs_keep_task_stopped_for_reconfirmation(adapter, reason):
    begin_input_outage(adapter)
    started = adapter.recovery_started
    adapter.on_status(DiagnosticArray(status=[DiagnosticStatus(values=[
        KeyValue(key='valid', value='false'), KeyValue(key='reason', value=reason)])]))
    assert adapter.navigation_guard.holding and not adapter.navigation_guard.blocked
    assert adapter.recovery_started == started
    assert adapter.accepted_transform is None and not adapter.ready
    assert adapter.command_pub.messages[-1].linear.x == 0.


@pytest.mark.parametrize('reason', ['waiting_for_live_obstacle_update',
                                  'waiting_for_map_or_fresh_input'])
def test_recovered_alignment_waits_for_first_new_obstacle_consumption(adapter, reason):
    # A* resets dynamic obstacle health when our availability was false.
    # Its intermediate waiting state must not escalate the original hold.
    begin_input_outage(adapter, 'odometry_missing')
    started = adapter.recovery_started
    adapter.on_obstacle_health(String(data=json.dumps(dict(
        ready=False, reason=reason, consumed_stamp_ns=0, processed_age_s=-1.))))
    adapter.tick()
    assert adapter.navigation_guard.holding and not adapter.navigation_guard.blocked
    assert adapter.recovery_started == started
    assert not adapter.validation_client.requests
    now = adapter.get_clock().now().nanoseconds
    adapter.on_obstacle_health(String(data=json.dumps(dict(
        ready=True, consumed_stamp_ns=now, processed_age_s=0.))))
    adapter.tick()
    valid_validation(adapter.validation_client.requests[-1][1])
    adapter.tick()
    assert adapter.ready and not adapter.navigation_guard.holding


def test_cached_alignment_expiry_uses_same_recovery_policy_as_localization_timer(adapter):
    begin_input_outage(adapter)
    started = adapter.recovery_started
    old_ns = adapter.get_clock().now().nanoseconds - 4000000000
    adapter.accepted_transform.header.stamp = Time(nanoseconds=old_ns).to_msg()
    adapter.valid_stamp = adapter.gate.last_stamp = old_ns * 1e-9
    adapter.results.append(copy.deepcopy(adapter.accepted_transform))
    adapter.tick()  # try_accept() runs before localization_available().
    assert adapter.navigation_guard.holding and not adapter.navigation_guard.blocked
    assert adapter.recovery_started == started
    assert adapter.accepted_transform is None
    assert not adapter.ready


@pytest.mark.parametrize('status_first', [False, True])
@pytest.mark.parametrize('recoverable', [False, True])
def test_fault_and_active_status_callback_order_use_same_policy(adapter, status_first, recoverable):
    from action_msgs.msg import GoalStatus, GoalStatusArray
    status = GoalStatus(status=GoalStatus.STATUS_EXECUTING)
    status.goal_info.goal_id.uuid[0] = 1
    status.goal_info.stamp = adapter.get_clock().now().to_msg()
    message = GoalStatusArray(status_list=[status])
    def fault():
        if recoverable:
            adapter.obstacle_health.received_at = time.monotonic() - 1.1
            adapter.set_ready(False, 'global_obstacle_heartbeat_missing')
        else:
            adapter.invalidate('localization_ambiguous_match')
    if status_first:
        adapter.navigation_guard.on_status('navigate_to_pose', message)
        fault()
    else:
        fault()
        adapter.navigation_guard.on_status('navigate_to_pose', message)
    assert adapter.navigation_guard.holding is recoverable
    assert adapter.navigation_guard.blocked is (not recoverable)
    assert not adapter.ready
    command = Twist()
    command.linear.x = .2
    adapter.on_command(command)
    assert not any(m.linear.x or m.angular.z for m in adapter.command_pub.messages)


def test_explicit_map_false_is_not_reclassified_as_transient_missing_heartbeat(adapter):
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from std_msgs.msg import Bool
    adapter.on_map(Bool(data=False))
    assert adapter.readiness() == (False, 'map_not_ready')
    adapter.tick()
    status = GoalStatus(status=GoalStatus.STATUS_EXECUTING)
    status.goal_info.goal_id.uuid[0] = 1
    status.goal_info.stamp = adapter.get_clock().now().to_msg()
    adapter.navigation_guard.on_status('navigate_to_pose', GoalStatusArray(status_list=[status]))
    assert adapter.navigation_guard.blocked and not adapter.navigation_guard.holding
    assert not adapter.ready


def test_late_active_status_after_inputs_recover_still_requires_path_revalidation(adapter):
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    adapter.validation_client = ValidationClient()
    adapter.on_plan(Path(poses=[PoseStamped(), PoseStamped()]))
    adapter.set_ready(False, 'global_obstacle_heartbeat_missing')  # No known active task yet.
    assert not adapter.navigation_guard.holding
    assert adapter.readiness()[0]  # Inputs recovered before the next timer tick.
    status = GoalStatus(status=GoalStatus.STATUS_EXECUTING)
    status.goal_info.goal_id.uuid[0] = 1
    status.goal_info.stamp = adapter.get_clock().now().to_msg()
    adapter.navigation_guard.on_status('navigate_to_pose', GoalStatusArray(status_list=[status]))
    assert adapter.navigation_guard.holding and not adapter.navigation_guard.blocked
    adapter.tick()
    assert not adapter.ready
    valid_validation(adapter.validation_client.requests[-1][1])
    adapter.tick()
    assert adapter.ready


@pytest.mark.parametrize('stream', ['odom', 'lio', 'terrain', 'alignment', 'diagnostics'])
@pytest.mark.parametrize('kind', ['frame', 'stamp', 'pose'])
def test_bad_sample_preserves_good_state_without_refreshing_deadlines(adapter, stream, kind):
    from sensor_msgs.msg import PointCloud2
    transform = copy.deepcopy(adapter.accepted_transform)
    received = (adapter.status_received, adapter.odom_received, adapter.terrain_received)
    stamps = (adapter.latest_odom_stamp, adapter.latest_terrain_stamp)
    if stream == 'diagnostics':
        if kind == 'frame':
            message = DiagnosticArray()
        else:
            message = DiagnosticArray(status=[DiagnosticStatus(values=[
                KeyValue(key='valid', value='true'), KeyValue(key='reason', value='accepted'),
                KeyValue(key='last_verified_stamp', value='nan' if kind == 'stamp' else 'bad')])])
        adapter.on_status(message)
    elif stream == 'alignment':
        message = copy.deepcopy(transform)
        if kind == 'frame':
            message.child_frame_id = 'wrong'
        elif kind == 'stamp':
            message.header.stamp.nanosec = 1000000000
        else:
            message.transform.rotation.w = float('nan')
        adapter.on_result(message)
    elif stream == 'terrain':
        message = PointCloud2()
        message.header.frame_id = 'base_link' if kind != 'frame' else 'wrong'
        message.header.stamp = adapter.get_clock().now().to_msg()
        if kind != 'frame':
            message.header.stamp.nanosec = 1000000000
        adapter.on_terrain(message)
    else:
        message = Odometry()
        message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
        message.header.stamp = adapter.get_clock().now().to_msg()
        message.pose.pose.orientation.w = 1.
        if kind == 'frame':
            message.header.frame_id = 'wrong'
        elif kind == 'stamp':
            message.header.stamp.nanosec = 1000000000
        else:
            message.pose.pose.position.x = float('nan')
        getattr(adapter, 'on_' + stream)(message)
    assert adapter.ready and adapter.accepted_transform == transform
    assert (adapter.status_received, adapter.odom_received, adapter.terrain_received) == received
    assert (adapter.latest_odom_stamp, adapter.latest_terrain_stamp) == stamps
    assert not adapter.navigation_guard.blocked


@pytest.mark.parametrize('stream', ['odom', 'terrain'])
@pytest.mark.parametrize('offset', [0., -.1, 1.])
def test_duplicate_out_of_order_or_future_inputs_cannot_renew_last_good(adapter, stream, offset):
    from sensor_msgs.msg import PointCloud2
    stamp_field = 'latest_odom_stamp' if stream == 'odom' else 'latest_terrain_stamp'
    received_field = 'odom_received' if stream == 'odom' else 'terrain_received'
    stamp = getattr(adapter, stamp_field)
    received = getattr(adapter, received_field)
    message = Odometry() if stream == 'odom' else PointCloud2()
    message.header.frame_id = 'odom' if stream == 'odom' else 'base_link'
    message.header.stamp = Time(nanoseconds=int(stamp * 1e9) + int(offset * 1e9)).to_msg()
    if stream == 'odom':
        message.child_frame_id = 'base_link'
        message.pose.pose.orientation.w = 1.
    getattr(adapter, 'on_' + stream)(message)
    assert getattr(adapter, stamp_field) == stamp
    assert getattr(adapter, received_field) == received
    setattr(adapter, received_field, time.monotonic() - 2.)
    adapter.tick()
    assert not adapter.ready


def test_out_of_order_lio_is_dropped_without_resetting_alignment(adapter):
    message = Odometry()
    message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
    message.pose.pose.orientation.w = 1.
    message.header.stamp = adapter.get_clock().now().to_msg()
    adapter.on_lio(message)
    old = copy.deepcopy(message)
    old.header.stamp = Time(nanoseconds=Time.from_msg(message.header.stamp).nanoseconds - 100000000).to_msg()
    adapter.on_lio(old)
    assert adapter.ready and adapter.accepted_transform is not None
    assert len(adapter.lio_poses) == 1 and adapter.lio_poses[-1] == message


def test_repeated_malformed_diagnostics_do_not_keep_old_localization_alive(adapter):
    before = adapter.status_received = time.monotonic() - 4.
    adapter.on_status(DiagnosticArray())
    assert adapter.status_received == before
    adapter.tick()
    assert not adapter.ready and adapter.accepted_transform is None
