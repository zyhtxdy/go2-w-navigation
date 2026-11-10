"""Planning evaluation contract tests; fake futures, no ROS initialization/DDS."""
import copy
import csv
import json
import math
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration, Time as TimeMessage
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.serialization import deserialize_message, serialize_message
from std_msgs.msg import String
import pytest
import yaml

import go2_offline_navigation.planning_evaluation as module
from go2_offline_navigation.planning_evaluation import load_planning_cases, path_metrics


BASE = 1700000000000000000


def cases():
    return dict(schema_version=1, frame_id='map', checkpoints_s=[10, 30],
                expected_map_id='a' * 64, source=dict(kind='historical_case', file='notes'),
                goals=[dict(id='forward', x=2, y=0, yaw=0, category='navigation', source_line=8),
                       dict(id='outside', x=100, y=100, z=0, yaw=1.2,
                            category='negative_control')])


def path(points=((0, 0, 0), (1, 0, 0), (2, 0, 0)), yaw=0):
    result = Path()
    result.header.frame_id = 'map'
    for x, y, z in points:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), float(z)
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        result.poses.append(pose)
    return result


def response(status=GoalStatus.STATUS_SUCCEEDED, value=None):
    return SimpleNamespace(status=status, result=SimpleNamespace(
        path=path() if value is None else value, planning_time=Duration(sec=0, nanosec=2000000)))


class FakeFuture:
    def __init__(self):
        self.callbacks = []
        self.value = None
        self.error = None
        self.done = False

    def add_done_callback(self, callback):
        if self.done:
            callback(self)
        else:
            self.callbacks.append(callback)

    def result(self):
        if self.error:
            raise self.error
        return self.value

    def set_result(self, value):
        self.done, self.value = True, value
        callbacks, self.callbacks = self.callbacks, []
        for callback in callbacks:
            callback(self)


class FakeHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.future = FakeFuture()
        self.canceled = 0

    def get_result_async(self):
        return self.future

    def cancel_goal_async(self):
        self.canceled += 1
        return FakeFuture()


class FakeClient:
    def __init__(self, *_args):
        self.available = True
        self.sent = []
        self.destroyed = False

    def server_is_ready(self):
        return self.available

    def send_goal_async(self, goal):
        future = FakeFuture()
        self.sent.append((goal, future))
        return future

    def destroy(self):
        self.destroyed = True


class FakeBuffer:
    def __init__(self):
        self.error = None

    def lookup_transform(self, *_args):
        if self.error:
            raise self.error
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.rotation.w = 1.0
        return transform


class FakeListener:
    def __init__(self, *_args):
        self.closed = False

    def unregister(self):
        self.closed = True


class FakeNode:
    def __init__(self):
        self.clock_ns = BASE
        self.subscriptions = []
        self.destroyed_subscriptions = []

    def get_clock(self):
        return self

    def now(self):
        sec, nanosec = divmod(self.clock_ns, 1000000000)
        return SimpleNamespace(nanoseconds=self.clock_ns,
                               to_msg=lambda: TimeMessage(sec=sec, nanosec=nanosec))

    def create_subscription(self, kind, topic, callback, qos):
        subscription = SimpleNamespace(kind=kind, topic=topic, callback=callback, qos=qos)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        self.destroyed_subscriptions.append(subscription)


@pytest.fixture
def evaluation(monkeypatch):
    wall = [100.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: wall[0])
    monkeypatch.setattr(module, 'ActionClient', FakeClient)
    monkeypatch.setattr(module, 'Buffer', FakeBuffer)
    monkeypatch.setattr(module, 'TransformListener', FakeListener)
    node = FakeNode()
    evaluator = module.PlanningEvaluation(node, cases(), BASE, timeout_s=5)

    def ready(value=True, verified=True):
        evaluator._observe('ready', value=value)
        evaluator._observe('status', value='ready' if value else 'terrain_stamp_stale')
        evaluator._observe('map_status', value=dict(ready=verified, faulted=not verified,
                                                  reason='ready' if verified else 'map_id_mismatch'))

    def tick(offset):
        node.clock_ns = BASE + round(offset * 1e9)
        evaluator.tick(node.clock_ns)

    def accept(index=0, accepted=True):
        handle = FakeHandle(accepted=accepted)
        evaluator.client.sent[index][1].set_result(handle)
        return handle

    environment = SimpleNamespace(evaluator=evaluator, wall=wall, node=node,
                                  ready=ready, tick=tick, accept=accept)
    yield environment
    evaluator.close()


def test_loader_preserves_provenance_and_normalizes_only_numbers(tmp_path):
    original = cases()
    file = tmp_path / 'cases.yaml'
    file.write_text(yaml.safe_dump(original))
    loaded = load_planning_cases(file)
    assert loaded['source'] == original['source']
    assert loaded['expected_map_id'] == 'a' * 64
    assert loaded['goals'][0]['source_line'] == 8
    assert loaded['goals'][0]['z'] == 0
    assert loaded['checkpoints_s'] == [10.0, 30.0]
    assert original['goals'][0].get('z') is None


@pytest.mark.parametrize('change', [
    lambda value: value.update(schema_version=True),
    lambda value: value.update(frame_id='odom'),
    lambda value: value.update(checkpoints_s=[]),
    lambda value: value.update(checkpoints_s=[30, 10]),
    lambda value: value.update(checkpoints_s=[10, 10]),
    lambda value: value.update(checkpoints_s=[-1, 10]),
    lambda value: value.update(checkpoints_s=[True, 10]),
    lambda value: value.update(checkpoints_s=[math.inf]),
    lambda value: value.update(checkpoints_s=[math.nan]),
    lambda value: value.update(goals=[]),
    lambda value: value['goals'][1].update(id='forward'),
    lambda value: value['goals'][0].update(id=' '),
    lambda value: value['goals'][0].update(x=math.nan),
    lambda value: value['goals'][0].update(y=True),
    lambda value: value['goals'][0].update(yaw='0.5'),
    lambda value: value['goals'][0].update(z=math.inf),
    lambda value: value['goals'][0].update(category='unknown'),
    lambda value: value.update(source=dict(something=math.nan)),
])
def test_loader_rejects_malformed_cases(tmp_path, change):
    value = cases()
    change(value)
    file = tmp_path / 'cases.yaml'
    file.write_text(yaml.safe_dump(value))
    with pytest.raises(ValueError):
        load_planning_cases(file)


def test_path_metrics_records_detour_and_wrapped_terminal_yaw():
    value = path(((0, 0, 0), (1, 1, .1), (2, 0, 0)), yaw=-math.pi+.01)
    metrics = path_metrics(value, dict(x=2, y=.1, yaw=math.pi-.01))
    assert metrics['valid']
    assert metrics['point_count'] == 3
    assert metrics['length_xy_m'] == pytest.approx(2 * math.sqrt(2))
    assert metrics['length_to_straight_ratio'] == pytest.approx(math.sqrt(2))
    assert metrics['goal_error_xy_m'] == pytest.approx(.1)
    assert metrics['goal_error_yaw_rad'] == pytest.approx(.02)
    assert metrics['length_3d_m'] > metrics['length_xy_m']


@pytest.mark.parametrize('change,reason', [
    (lambda value: setattr(value.header, 'frame_id', 'odom'), 'path_frame_not_map'),
    (lambda value: value.poses.clear(), 'empty_path'),
    (lambda value: setattr(value.poses[0].pose.position, 'x', math.nan), 'nonfinite_path_pose'),
    (lambda value: setattr(value.poses[0].pose.orientation, 'w', 0.0), 'invalid_pose_quaternion'),
    (lambda value: setattr(value.poses[0].header, 'frame_id', 'odom'), 'path_pose_frame_not_map'),
])
def test_path_metrics_rejects_invalid_paths(change, reason):
    value = path()
    change(value)
    metrics = path_metrics(value, dict(x=2, y=0, yaw=0))
    assert not metrics['valid']
    assert reason in metrics['errors']


def test_not_ready_at_checkpoint_is_blocked_without_retrying_at_better_time(evaluation):
    env = evaluation
    env.ready(False)
    env.tick(10)
    assert not env.evaluator.client.sent
    assert len(env.evaluator.summary()['rows']) == 2
    assert all(row['status'] == 'blocked' for row in env.evaluator.summary()['rows'])
    env.ready()
    env.tick(10.5)
    assert not env.evaluator.client.sent
    assert len(env.evaluator.summary()['rows']) == 2
    env.tick(30)
    assert len(env.evaluator.client.sent) == 1
    assert len(env.evaluator.summary()['rows']) == 4


@pytest.mark.parametrize('kind', ['map_fault', 'server_missing', 'stale_ready', 'stale_map'])
def test_fixed_checkpoint_captures_readiness_failures(evaluation, kind):
    env = evaluation
    env.ready(verified=kind != 'map_fault')
    if kind == 'server_missing':
        env.evaluator.client.available = False
    if kind in ('stale_ready', 'stale_map'):
        env.wall[0] += 2
        if kind == 'stale_map':
            env.evaluator._observe('ready', value=True)
    env.tick(10)
    summary = env.evaluator.summary()
    assert summary['statistics']['blocked'] == 2
    assert summary['statistics']['action_attempts'] == 0
    assert summary['rows'][0]['checkpoint_snapshot']['eligible'] is False


def test_real_start_is_used_and_goal_acceptance_alone_is_not_success(evaluation):
    env = evaluation
    env.ready()
    env.tick(10)
    goal, _ = env.evaluator.client.sent[0]
    assert not goal.use_start and goal.planner_id == 'GridBased'
    assert goal.goal.header.frame_id == 'map'
    assert goal.goal.header.stamp.sec == (BASE + 10000000000) // 1000000000
    assert goal.goal.pose.position.x == 2
    handle = env.accept()
    summary = env.evaluator.summary()
    assert summary['statistics']['action_attempts'] == 1
    assert summary['statistics']['succeeded'] == 0
    assert summary['rows'][0]['accepted'] is True
    env.node.clock_ns += 20000000
    handle.future.set_result(response())
    assert env.evaluator.summary()['statistics']['succeeded'] == 1
    assert len(env.evaluator.client.sent) == 1
    env.wall[0] += .2
    env.ready()
    env.tick(10.2)
    assert len(env.evaluator.client.sent) == 2
    first = env.evaluator.summary()['rows'][0]
    assert first['start_tf']['child_frame_id'] == 'base_link'
    assert first['planning_time_s'] == pytest.approx(.002)
    assert first['path']['length_xy_m'] == 2
    assert first['path']['raw_start_to_goal_straight_xy_m'] == 2
    assert first['path']['raw_start_to_path_first_xy_m'] == 0


def test_success_action_with_empty_path_is_a_failure(evaluation):
    env = evaluation
    env.ready()
    env.tick(10)
    handle = env.accept()
    handle.future.set_result(response(value=path(())))
    row = env.evaluator.summary()['rows'][0]
    assert row['status'] == 'failed'
    assert row['reason'] == 'invalid_returned_path'
    assert row['path']['errors'] == ['empty_path']


def test_old_retained_failure_cannot_be_attached_to_new_request(evaluation):
    env = evaluation
    env.ready()
    env.evaluator._on_failure(String(data=json.dumps(dict(
        request_stamp_ns=BASE-1, reason='previous_run_retained_failure'))))
    env.tick(10)
    request_ns = env.evaluator.summary()['rows'][0]['request_clock_ns']
    # A retained event delivered late also remains outside the source interval.
    env.evaluator._on_failure(String(data=json.dumps(dict(
        request_stamp_ns=BASE, reason='late_old_failure'))))
    handle = env.accept()
    env.node.clock_ns += 50000000
    handle.future.set_result(response(status=GoalStatus.STATUS_ABORTED, value=path(())))
    # Diagnostics on another topic may be observed shortly after action result.
    env.evaluator._on_failure(String(data=json.dumps(dict(
        request_stamp_ns=request_ns+10000000, reason='actual_current_failure'))))
    env.wall[0] += .2
    env.ready()
    env.tick(10.2)
    summary = env.evaluator.summary()
    assert [item['diagnostic']['reason'] for item in summary['rows'][0]['failure_diagnostics']] == [
        'actual_current_failure']
    assert len(summary['unassigned_failure_diagnostics']) == 2


def test_timeout_cancels_late_acceptance_and_blocks_unresolved_following_queries(evaluation):
    env = evaluation
    env.ready()
    env.tick(10)
    old_future = env.evaluator.client.sent[0][1]
    env.wall[0] += 5.1
    env.ready()
    env.tick(15.1)
    assert env.evaluator.summary()['rows'][0]['status'] == 'timeout'
    env.wall[0] += .2
    env.ready()
    env.tick(15.3)
    assert len(env.evaluator.client.sent) == 1
    late = FakeHandle()
    old_future.set_result(late)
    assert late.canceled == 1
    late.future.set_result(response())
    summary = env.evaluator.summary()
    assert summary['rows'][0]['status'] == 'timeout'
    assert summary['rows'][1]['status'] == 'blocked'
    assert summary['rows'][1]['reason'] == 'blocked_by_unresolved_timeout'
    assert summary['statistics']['succeeded'] == 0
    assert summary['unresolved_timeout']
    env.ready()
    env.tick(30)
    assert len(env.evaluator.client.sent) == 1
    assert env.evaluator.summary()['statistics']['blocked'] == 3


def test_timeout_cancels_accepted_handle_and_drops_late_result(evaluation):
    env = evaluation
    env.ready()
    env.tick(10)
    old = env.accept()
    env.wall[0] += 5.1
    env.ready()
    env.tick(15.1)
    assert old.canceled == 1
    assert env.evaluator.summary()['rows'][0]['cancel_requested'] is True
    old.future.set_result(response())
    assert env.evaluator.summary()['statistics']['succeeded'] == 0


def test_finish_marks_unreached_cases_and_queued_work_but_preserves_inflight_grace(evaluation):
    env = evaluation
    env.ready()
    env.tick(10)
    handle = env.accept()
    env.evaluator.finish(BASE + 15000000000)
    before = env.evaluator.summary()
    assert before['statistics']['scheduled'] == 4
    assert before['statistics']['not_executed_end_of_replay'] == 1
    assert before['statistics']['not_reached'] == 2
    assert not env.evaluator.is_idle()
    env.node.clock_ns += 50000000
    handle.future.set_result(response())
    env.wall[0] += .2
    env.tick(15.2)
    assert env.evaluator.is_idle()
    assert len(env.evaluator.client.sent) == 1
    assert env.evaluator.summary()['statistics']['succeeded'] == 1


def test_navigation_and_negative_control_statistics_are_separate(evaluation):
    env = evaluation
    env.ready()
    env.tick(10)
    env.accept().future.set_result(response())
    env.wall[0] += .2
    env.ready()
    env.tick(10.2)
    env.accept(1).future.set_result(response(status=GoalStatus.STATUS_ABORTED, value=path(())))
    env.wall[0] += .2
    env.evaluator.finish(BASE + 11000000000)
    env.tick(11)
    summary = env.evaluator.summary()
    assert summary['by_category']['navigation']['succeeded'] == 1
    assert summary['by_category']['negative_control']['succeeded'] == 0
    assert summary['negative_control_rejected'] == 1
    assert summary['by_category']['navigation']['success_rate_of_attempts'] == 1
    assert summary['by_category']['navigation']['success_rate_of_scheduled'] == .5


def test_missing_robot_transform_does_not_create_fake_start_or_attempt(evaluation):
    env = evaluation
    env.ready()
    env.evaluator.tf_buffer.error = RuntimeError('missing map->base_link')
    env.tick(10)
    summary = env.evaluator.summary()
    assert not env.evaluator.client.sent
    assert summary['statistics']['blocked'] == 2
    assert all('robot_transform_unavailable' in row['reason'] for row in summary['rows'])


def test_dispatch_rechecks_gate_after_checkpoint_snapshot(evaluation):
    env = evaluation
    env.ready()
    env.tick(10)
    env.accept().future.set_result(response())
    env.ready(False)
    env.wall[0] += .2
    env.tick(10.2)
    second = env.evaluator.summary()['rows'][1]
    assert second['checkpoint_snapshot']['eligible']
    assert not second['request_snapshot']['eligible']
    assert second['status'] == 'blocked'
    assert len(env.evaluator.client.sent) == 1


def test_close_cleans_up_resources_and_does_not_accept_late_results(evaluation):
    env = evaluation
    env.ready()
    env.tick(10)
    handle = env.accept()
    subscriptions = len(env.node.subscriptions)
    env.evaluator.close()
    env.evaluator.close()
    assert handle.canceled == 1
    assert env.evaluator.client.destroyed
    assert env.evaluator.tf_listener.closed
    assert len(env.node.destroyed_subscriptions) == subscriptions
    handle.future.set_result(response())
    assert env.evaluator.summary()['statistics']['succeeded'] == 0


def test_probe_subscribes_to_real_inputs_and_never_creates_a_publisher(evaluation):
    assert {item.topic for item in evaluation.node.subscriptions} == {
        '/offline_localization/ready', '/offline_localization/status', '/planner/map_status',
        '/planner/start_failure', '/Odometry', '/odom', '/terrain/obstacles', '/relocalization/status'}
    # FakeNode intentionally has no publisher API; construction and queries must
    # succeed without generating fabricated inputs, readiness, TF or velocity.
    evaluation.ready()
    evaluation.tick(10)
    assert len(evaluation.evaluator.client.sent) == 1


@pytest.mark.parametrize('level', [0, 1, 2, 3, 255])
def test_localization_callback_accepts_real_humble_byte_after_cdr_roundtrip(evaluation, level):
    # In Humble the generated byte field is bytes, including after real CDR
    # deserialization. No ROS context, node, discovery or DDS transport is used.
    status = DiagnosticStatus(level=bytes([level]), name='relocalization/map_match',
                              message='CONFIRMING', hardware_id='software_prototype',
                              values=[KeyValue(key='reason', value='awaiting_distinct_consistent_frames')])
    status = deserialize_message(serialize_message(status), DiagnosticStatus)
    assert status.level == bytes([level])
    message = DiagnosticArray(status=[status])
    message.header.stamp = TimeMessage(sec=1700000010, nanosec=123)
    decoded = deserialize_message(serialize_message(message), DiagnosticArray)
    evaluation.evaluator._on_localization(decoded)
    observation = evaluation.evaluator._snapshot()['inputs']['localization']
    assert observation['stamp_ns'] == 1700000010000000123
    assert observation['statuses'][0]['level'] == level
    assert observation['statuses'][0]['message'] == 'CONFIRMING'
    assert observation['statuses'][0]['values']['reason'] == 'awaiting_distinct_consistent_frames'
    json.dumps(observation, allow_nan=False)


def test_localization_callback_keeps_integer_representation_compatibility(evaluation):
    message = SimpleNamespace(header=SimpleNamespace(stamp=TimeMessage(sec=1700000010)),
                              status=[SimpleNamespace(name='map_match', level=2, message='ERROR',
                                                      values=[KeyValue(key='valid', value='false')])])
    evaluation.evaluator._on_localization(message)
    statuses = evaluation.evaluator._snapshot()['inputs']['localization']['statuses']
    assert statuses == [dict(name='map_match', level=2, message='ERROR', values=dict(valid='false'))]


@pytest.mark.parametrize('value', [b'', b'\x00\x01', bytearray(), True, -1, 256, 1.0, '1', None])
def test_diagnostic_level_rejects_non_single_byte_or_invalid_integer(value):
    with pytest.raises(ValueError, match='DiagnosticStatus.level'):
        module._diagnostic_level(value)


def test_diagnostic_level_accepts_single_byte_without_decimal_text_parsing():
    assert module._diagnostic_level(b'\xff') == 255
    # The byte representing ASCII '1' is numeric 49, not decimal text 1.
    assert module._diagnostic_level(b'1') == 49
    assert module._diagnostic_level(bytearray([3])) == 3


def aborted_row(index, goal_id='G1', x=1.45606, y=-.703491, category='navigation'):
    return dict(index=index, checkpoint_s=10+index*20, status='failed', reason='action_not_succeeded',
                attempted=True, accepted=True, action_status=GoalStatus.STATUS_ABORTED,
                category=category, goal=dict(id=goal_id, x=x, y=y, z=0, yaw=0,
                                             category=category, source_line=8),
                failure_diagnostics=[], request_offset_s=10+index*20, elapsed_wall_s=.01,
                planning_time_s=.002, start_tf=dict(frame_id='map', child_frame_id='base_link',
                                                  position=[0, 0, 0], quaternion=[0, 0, 0, 1], yaw=0))


def failure_line(message, x='1.46', y='-0.70', stamp='1792923872.248491395'):
    return (f'[WARN] [{stamp}] [planner_server]: GridBased plugin failed to plan calculation '
            f'to ({x}, {y}): "{message}"')


def test_enrichment_matches_repeated_goal_in_order_without_changing_original_evidence():
    original = dict(config=cases(), rows=[aborted_row(0), aborted_row(1)])
    original_copy = copy.deepcopy(original)
    text = 'unrelated line\n' + failure_line('first explanation') + '\n' + \
        failure_line('second explanation', stamp='1792923892.396928514')
    enriched = module.enrich_planning_report(original, text)
    assert original == original_copy
    assert [row['server_failure_message'] for row in enriched['rows']] == [
        'first explanation', 'second explanation']
    assert [row['server_failure_event']['line_no'] for row in enriched['rows']] == [2, 3]
    assert enriched['rows'][0]['server_failure_event']['wallstamp'] == '1792923872.248491395'
    assert all(row['reason'] == 'action_not_succeeded' and row['status'] == 'failed'
               for row in enriched['rows'])
    assert enriched['server_failure_log_match']['matched_count'] == 2
    assert not enriched['server_failure_log_match']['unmatched_events']
    assert not enriched['server_failure_log_match']['rows_without_server_message']


def test_enrichment_does_not_attach_events_to_nonaborted_or_unaccepted_queries():
    success = aborted_row(0, x=2)
    success.update(status='succeeded', action_status=GoalStatus.STATUS_SUCCEEDED)
    unaccepted = aborted_row(1, x=3)
    unaccepted['accepted'] = False
    timeout = aborted_row(2, x=4)
    timeout['status'] = 'timeout'
    actual = aborted_row(3, x=5)
    original = dict(rows=[success, unaccepted, timeout, actual])
    text = '\n'.join(failure_line('message at ' + str(x), x=str(x)) for x in (2, 3, 4, 5))
    enriched = module.enrich_planning_report(original, text)
    assert all('server_failure_message' not in row for row in enriched['rows'][:3])
    assert enriched['rows'][3]['server_failure_message'] == 'message at 5'
    audit = enriched['server_failure_log_match']
    assert audit['eligible_aborted_rows'] == 1 and audit['matched_count'] == 1
    assert len(audit['unmatched_events']) == 3


def test_enrichment_does_not_guess_for_wrong_target_or_reuse_an_old_event():
    original = dict(rows=[aborted_row(0), aborted_row(1)])
    text = failure_line('wrong location', x='10.00', y='10.00') + '\n' + failure_line('one match')
    enriched = module.enrich_planning_report(original, text)
    assert all('server_failure_message' not in row for row in enriched['rows'])
    audit = enriched['server_failure_log_match']
    assert audit['matched_count'] == 0
    assert len(audit['unmatched_events']) == 2
    assert [row['index'] for row in audit['rows_without_server_message']] == [0, 1]
    assert audit['ambiguous_events'][0]['candidate_row_indices'] == [0, 1]
    assert audit['ambiguous_events'][0]['reason'] == 'insufficient_events_for_repeated_goal'


@pytest.mark.parametrize('event_count,matched,ambiguous_reason', [
    (1, 0, 'insufficient_events_for_repeated_goal'),
    (2, 0, 'insufficient_events_for_repeated_goal'),
    (3, 3, None),
    (4, 0, 'excess_events_for_goal_candidates'),
])
def test_repeated_goal_requires_all_its_log_events_without_excess(event_count, matched, ambiguous_reason):
    original = dict(rows=[aborted_row(index) for index in range(3)])
    text = '\n'.join(failure_line('message ' + str(index)) for index in range(event_count))
    enriched = module.enrich_planning_report(original, text)
    audit = enriched['server_failure_log_match']
    assert audit['matched_count'] == matched
    if ambiguous_reason:
        assert len(audit['ambiguous_events']) == event_count
        assert all(event['reason'] == ambiguous_reason for event in audit['ambiguous_events'])
        assert all(event['candidate_row_indices'] == [0, 1, 2] for event in audit['ambiguous_events'])
        assert len(audit['rows_without_server_message']) == 3
    else:
        assert audit['ambiguous_events'] == []
        assert [row['server_failure_message'] for row in enriched['rows']] == [
            'message 0', 'message 1', 'message 2']


def test_repeated_goals_complete_groups_can_interleave_in_serial_order():
    original = dict(rows=[aborted_row(0), aborted_row(1, x=3), aborted_row(2)])
    text = '\n'.join([failure_line('first A'), failure_line('middle B', x='3.00'),
                      failure_line('second A')])
    enriched = module.enrich_planning_report(original, text)
    assert enriched['server_failure_log_match']['matched_count'] == 3
    assert [row['server_failure_message'] for row in enriched['rows']] == [
        'first A', 'middle B', 'second A']


def test_out_of_order_log_events_do_not_get_reordered_to_match_cases():
    original = dict(rows=[aborted_row(0), aborted_row(1, x=3)])
    text = '\n'.join([failure_line('B before A', x='3.00'), failure_line('A after B')])
    enriched = module.enrich_planning_report(original, text)
    audit = enriched['server_failure_log_match']
    assert audit['matched_count'] == 0
    assert len(audit['ambiguous_events']) == 2
    assert all(event['reason'] == 'log_order_conflicts_with_serial_requests'
               for event in audit['ambiguous_events'])


@pytest.mark.parametrize('x,matched', [(1.45606, 1), (1.454, 1), (1.4539, 0)])
def test_log_printing_tolerance_is_explicit_and_bounded(x, matched):
    result = module.enrich_planning_report(dict(rows=[aborted_row(0, x=x)]), failure_line('x'))
    assert result['server_failure_log_match']['matched_count'] == matched


def test_unparsed_failure_lines_are_reported_without_inventing_message():
    text = '[WARN] GridBased plugin failed to plan calculation to (1.46,-0.70): truncated'
    result = module.enrich_planning_report(dict(rows=[aborted_row(0)]), text)
    audit = result['server_failure_log_match']
    assert audit['matched_count'] == 0
    assert len(audit['unparsed_failure_lines']) == 1
    assert audit['unparsed_failure_lines'][0]['line_no'] == 1
    assert len(audit['rows_without_server_message']) == 1


def test_enrichment_separates_negative_control_messages_from_navigation_counts():
    rows = [aborted_row(0), aborted_row(1, category='negative_control')]
    text = '\n'.join([failure_line('no safe ground'), failure_line('no safe ground')])
    result = module.enrich_planning_report(dict(rows=rows), text)
    groups = result['server_failure_log_match']['counts_by_category']
    assert groups['navigation'] == dict(matched=1, messages={'no safe ground': 1})
    assert groups['negative_control'] == dict(matched=1, messages={'no safe ground': 1})


def test_writer_preserves_all_queries_metadata_and_csv_quotes_newlines(tmp_path):
    first = aborted_row(0)
    first['reason'] = 'original "reason", with comma\nand newline'
    first['failure_diagnostics'] = [dict(diagnostic=dict(reason='observed_reason'))]
    blocked = aborted_row(1, goal_id='BLOCKED', category='negative_control')
    blocked.update(status='blocked', accepted=None, action_status=None,
                   attempted=False, reason='not ready')
    original = dict(config=cases(), rows=[first, blocked], extra_metadata=dict(version='unchanged'))
    message = 'server said "quoted", keep this text'
    log = tmp_path / 'planner.log'
    log.write_text(failure_line(message))
    output = tmp_path / 'reports'
    enriched = module.write_planning_reports(output, original, log)
    saved = json.loads((output / 'planning_report.json').read_text())
    assert saved == enriched
    assert saved['config']['source'] == original['config']['source']
    assert saved['extra_metadata'] == original['extra_metadata']
    with (output / 'planning_report.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 2
    assert rows[0]['reason'] == first['reason']
    assert rows[0]['server_message'] == message
    assert rows[0]['structured_reasons'] == 'observed_reason'
    assert rows[0]['start_tf_child_frame_id'] == 'base_link'
    assert rows[1]['goal_id'] == 'BLOCKED'
    assert rows[1]['category'] == 'negative_control' and rows[1]['status'] == 'blocked'
    assert rows[1]['server_message'] == ''
    assert 'server_failure_message' not in original['rows'][0]


@pytest.mark.parametrize('missing_path', [None, 'does-not-exist.log'])
def test_missing_planner_log_degrades_to_explicit_unmatched_rows(tmp_path, missing_path):
    rows = [aborted_row(0)]
    result = module.write_planning_reports(tmp_path / 'result', dict(rows=rows),
        None if missing_path is None else tmp_path / missing_path)
    audit = result['server_failure_log_match']
    assert audit['log_state'] == 'unavailable'
    assert audit['matched_count'] == 0 and len(audit['rows_without_server_message']) == 1
    assert 'server_failure_message' not in result['rows'][0]
    assert (tmp_path / 'result/planning_report.csv').is_file()
    if missing_path is not None:
        assert 'read_error' in audit


def test_optional_evidence_subscribes_to_actual_input_without_changing_default_evaluation(evaluation, tmp_path):
    value = module.PlanningEvaluation(evaluation.node, cases(), BASE,
        evidence_output=tmp_path, live_obstacle_topic='/selected/measured')
    try:
        topics = [subscription.topic for subscription in value._subscriptions]
        assert '/hesai/points' in topics
        assert '/cloud_registered_body' in topics
        assert '/selected/measured' in topics
        assert topics.count('/Odometry') == 2  # State timing and original-message evidence.
        assert evaluation.evaluator.evidence is None
        value._clock_ns = BASE
        diagnostic = dict(frame_id='map', request_stamp_ns=BASE,
            obstacle=dict(point_current_datum=[1., 2., 3.], point_observed_z=3.))
        value._on_failure(String(data=json.dumps(diagnostic)))
        assert value._failures[0]['obstacle_evidence_event_id'] == 1
        summary = value.write_evidence()
        assert summary['events'] == 1
        assert value.summary()['obstacle_evidence'] == summary
        assert value.write_evidence() == summary
        report = json.loads((tmp_path/'evidence.json').read_text())
        assert report['events'][0]['context'] == {'failure_sequence': 1}
    finally:
        value.close()
