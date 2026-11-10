"""Measure real replay localization and global planning, without motion nodes.

This client never publishes sensors, transforms, readiness, or velocity. Query
checkpoints are fixed before replay, including blocked and unreached queries.
"""
from collections import Counter, deque
import copy
import csv
import json
import math
from pathlib import Path
import re
import time

import yaml

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(label + ' must be a finite number')
    try:
        result = float(value)
    except (OverflowError, ValueError) as error:
        raise ValueError(label + ' must be a finite number') from error
    if not math.isfinite(result):
        raise ValueError(label + ' must be a finite number')
    return result


def _diagnostic_level(value):
    """Normalize Humble's single-byte field and integer ROS representations."""
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise ValueError('DiagnosticStatus.level must contain exactly one byte')
        return value[0]
    if type(value) is int and 0 <= value <= 255:
        return value
    raise ValueError('DiagnosticStatus.level must be one byte or a uint8 integer')


def _validated_config(config):
    if not isinstance(config, dict) or type(config.get('schema_version')) is not int or \
            config['schema_version'] != 1 or config.get('frame_id') != 'map':
        raise ValueError('Planning cases require schema_version=1 and frame_id=map')
    result = copy.deepcopy(config)
    checkpoints = result.get('checkpoints_s')
    if not isinstance(checkpoints, list) or not checkpoints:
        raise ValueError('checkpoints_s must be a nonempty, strictly increasing list')
    checkpoints = [_number(value, 'checkpoint') for value in checkpoints]
    if checkpoints[0] < 0 or any(b <= a for a, b in zip(checkpoints, checkpoints[1:])):
        raise ValueError('checkpoints_s must be nonnegative and strictly increasing')
    result['checkpoints_s'] = checkpoints
    goals = result.get('goals')
    if not isinstance(goals, list) or not goals:
        raise ValueError('goals must be a nonempty list')
    identifiers = set()
    for goal in goals:
        if not isinstance(goal, dict):
            raise ValueError('Each goal must be a mapping')
        identifier = goal.get('id')
        if not isinstance(identifier, str) or not identifier.strip() or \
                identifier != identifier.strip() or identifier in identifiers:
            raise ValueError('Each goal requires a unique nonempty id')
        identifiers.add(identifier)
        for key in ('x', 'y', 'yaw'):
            goal[key] = _number(goal.get(key), 'goal.' + key)
        goal['z'] = _number(goal.get('z', 0.0), 'goal.z')
        if goal.get('category') not in ('navigation', 'negative_control'):
            raise ValueError('Goal category must be navigation or negative_control')
    # Keep provenance, expected_map_id, source_line and any other metadata, but
    # do not allow non-finite or non-JSON values into the persisted report.
    try:
        json.dumps(result, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise ValueError('Planning case metadata must be finite JSON-compatible values') from error
    return result


def load_planning_cases(path):
    """Read JSON or YAML, preserving case provenance and expected map identity."""
    path = Path(path).expanduser().resolve(strict=True)
    result = _validated_config(yaml.safe_load(path.read_text(encoding='utf-8')))
    result.setdefault('source', str(path))
    return result


def _stamp_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def _quaternion_yaw(quaternion):
    x, y, z, w = quaternion
    norm = x*x + y*y + z*z + w*w
    if not math.isfinite(norm) or abs(norm - 1.0) > .01:
        raise ValueError('invalid_pose_quaternion')
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def path_metrics(path, target):
    """Validate an action's actual returned path and compute geometry metrics."""
    result = dict(valid=False, frame_id=path.header.frame_id,
                  stamp_ns=_stamp_ns(path.header.stamp), points=[], point_count=len(path.poses),
                  length_xy_m=None, length_3d_m=None, straight_xy_m=None,
                  length_to_straight_ratio=None, goal_error_xy_m=None,
                  goal_error_yaw_rad=None, errors=[])
    if path.header.frame_id != 'map':
        result['errors'].append('path_frame_not_map')
    if not path.poses:
        result['errors'].append('empty_path')
    for pose in path.poses:
        position, orientation = pose.pose.position, pose.pose.orientation
        values = [position.x, position.y, position.z,
                  orientation.x, orientation.y, orientation.z, orientation.w]
        if pose.header.frame_id not in ('', 'map'):
            result['errors'].append('path_pose_frame_not_map')
        if not all(math.isfinite(value) for value in values):
            result['errors'].append('nonfinite_path_pose')
            continue
        try:
            yaw = _quaternion_yaw(values[3:])
        except ValueError as error:
            result['errors'].append(str(error))
            continue
        result['points'].append(dict(x=position.x, y=position.y, z=position.z,
                                     quaternion=values[3:], yaw=yaw))
    if result['errors']:
        result['errors'] = sorted(set(result['errors']))
        return result
    points = result['points']
    length_xy = sum(math.hypot(b['x'] - a['x'], b['y'] - a['y'])
                    for a, b in zip(points, points[1:]))
    length_3d = sum(math.dist([a[key] for key in ('x', 'y', 'z')],
                              [b[key] for key in ('x', 'y', 'z')])
                    for a, b in zip(points, points[1:]))
    straight = math.hypot(points[-1]['x'] - points[0]['x'], points[-1]['y'] - points[0]['y'])
    residual = math.hypot(points[-1]['x'] - target['x'], points[-1]['y'] - target['y'])
    yaw_difference = points[-1]['yaw'] - target['yaw']
    if not all(math.isfinite(value) for value in (length_xy, length_3d, straight, residual)):
        result['errors'].append('nonfinite_path_geometry')
        return result
    result.update(valid=True, length_xy_m=length_xy, length_3d_m=length_3d,
                  straight_xy_m=straight,
                  length_to_straight_ratio=length_xy / straight if straight > 1e-9 else None,
                  goal_error_xy_m=residual,
                  goal_error_yaw_rad=abs(math.atan2(math.sin(yaw_difference), math.cos(yaw_difference))))
    return result


def enrich_planning_report(report, planner_log_text):
    """Attach observed server exception text without replacing client evidence.

    Humble's action result has no error-message field. These lines belong to the
    separately captured, exclusively owned planner process. Match aborted serial
    requests in order using the printed two-decimal goal coordinates. This is
    log correlation, not a new structured diagnostic or a root-cause inference.
    """
    result = copy.deepcopy(report)
    rows = result.get('rows', [])
    for row in rows:
        row.pop('server_failure_message', None)
        row.pop('server_failure_event', None)
    number = r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?'
    pattern = re.compile(
        r'\[WARN\]\s+\[(?P<wallstamp>\d+(?:\.\d+)?)\]\s+\[[^\]]+\]:\s+'
        r'GridBased plugin failed to plan calculation to\s*\(\s*'
        r'(?P<x>' + number + r')\s*,\s*(?P<y>' + number + r')\s*\):\s*'
        r'"(?P<message>.*)"\s*$')
    ansi = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
    events, unparsed = [], []
    for line_no, line in enumerate((planner_log_text or '').splitlines(), 1):
        if 'GridBased plugin failed to plan calculation' not in line:
            continue
        match = pattern.search(ansi.sub('', line))
        if match is None:
            unparsed.append(dict(line_no=line_no, raw=line))
            continue
        x, y = float(match['x']), float(match['y'])
        if not math.isfinite(x) or not math.isfinite(y):
            unparsed.append(dict(line_no=line_no, raw=line))
            continue
        events.append(dict(line_no=line_no, wallstamp=match['wallstamp'],
                           goal_x=x, goal_y=y, message=match['message']))
    eligible = [(index, row) for index, row in enumerate(rows)
                if row.get('accepted') is True and row.get('status') == 'failed' and
                row.get('action_status') == GoalStatus.STATUS_ABORTED]
    # Build full candidate sets before assigning anything. If two requests used
    # the same printed target but one log line was lost, forward-only matching
    # would silently attribute the remaining line to the wrong request.
    groups = {}
    for event_index, event in enumerate(events):
        candidates = []
        for row_index, row in eligible:
            goal = row.get('goal', {})
            if all(type(goal.get(key)) in (int, float) and math.isfinite(goal[key])
                   for key in ('x', 'y')) and \
                    abs(goal['x'] - event['goal_x']) <= .006 + 1e-12 and \
                    abs(goal['y'] - event['goal_y']) <= .006 + 1e-12:
                candidates.append(row_index)
        groups.setdefault(tuple(candidates), []).append(event_index)
    ambiguous, proposals = {}, []
    for candidates, event_indices in groups.items():
        if not candidates:
            continue
        overlap = any(set(candidates) & set(other) for other in groups
                      if other and other != candidates)
        if overlap or len(candidates) != len(event_indices):
            reason = ('overlapping_goal_candidate_sets' if overlap else
                      'insufficient_events_for_repeated_goal' if len(event_indices) < len(candidates)
                      else 'excess_events_for_goal_candidates')
            for event_index in event_indices:
                ambiguous[event_index] = dict(
                    **copy.deepcopy(events[event_index]), reason=reason,
                    candidate_row_indices=list(candidates),
                    candidate_rows=[dict(row_index=index, index=rows[index].get('index', index),
                                         goal_id=rows[index].get('goal', {}).get('id'),
                                         checkpoint_s=rows[index].get('checkpoint_s'))
                                    for index in candidates],
                    candidate_count=len(candidates), observed_event_count=len(event_indices))
            continue
        proposals.extend(zip(event_indices, candidates))
    proposals.sort()
    if any(later[1] <= earlier[1] for earlier, later in zip(proposals, proposals[1:])):
        # Even complete per-goal groups are insufficient if log ordering is
        # inconsistent with the serial request order. Do not repair it by guess.
        for event_index, row_index in proposals:
            ambiguous[event_index] = dict(**copy.deepcopy(events[event_index]),
                reason='log_order_conflicts_with_serial_requests', candidate_row_indices=[row_index])
        proposals = []
    matched = []
    for event_index, row_index in proposals:
        event, row = events[event_index], rows[row_index]
        row['server_failure_message'] = event['message']
        row['server_failure_event'] = copy.deepcopy(event)
        matched.append(dict(row_index=row_index, index=row.get('index', row_index),
                            goal_id=row.get('goal', {}).get('id'),
                            checkpoint_s=row.get('checkpoint_s'), **event))
    assigned_events = {event_index for event_index, _ in proposals}
    unmatched = [copy.deepcopy(event) for event_index, event in enumerate(events)
                 if event_index not in assigned_events]
    without = [dict(row_index=index, index=row.get('index', index),
                    goal_id=row.get('goal', {}).get('id'), checkpoint_s=row.get('checkpoint_s'),
                    goal=copy.deepcopy(row.get('goal', {})))
               for index, row in eligible if 'server_failure_message' not in row]
    category_counts = {}
    for category in ('navigation', 'negative_control'):
        messages = Counter(row['server_failure_message'] for row in rows
                           if row.get('category') == category and 'server_failure_message' in row)
        category_counts[category] = dict(matched=sum(messages.values()), messages=dict(messages))
    result['server_failure_log_match'] = dict(
        method='serial_order_and_two_decimal_goal_xy', coordinate_tolerance_m=.006,
        ambiguity_policy='Require complete equal-sized nonoverlapping goal groups and serial order; '
                         'never guess which repeated request owns a missing or extra log event.',
        log_available=planner_log_text is not None,
        log_state=('unavailable' if planner_log_text is None else
                   'empty' if not planner_log_text else 'read'),
        eligible_aborted_rows=len(eligible), parsed_events=len(events), matched_count=len(matched),
        matched=matched, unmatched_events=unmatched,
        ambiguous_events=[ambiguous[index] for index in sorted(ambiguous)],
        rows_without_server_message=without,
        unparsed_failure_lines=unparsed, counts_by_category=category_counts)
    return result


def write_planning_reports(output, report, planner_log_path):
    """Write complete JSON and one CSV row per scheduled query, including controls.

    Missing logs degrade explicitly to unmatched aborted rows. Original client
    reason/status and structured diagnostics are preserved in both artifacts.
    """
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    log_error = None
    text = None
    if planner_log_path is not None:
        try:
            text = Path(planner_log_path).read_text(encoding='utf-8', errors='replace')
        except OSError as error:
            log_error = str(error)
    enriched = enrich_planning_report(report, text)
    enriched['server_failure_log_match']['source_path'] = (
        str(planner_log_path) if planner_log_path is not None else None)
    if log_error:
        enriched['server_failure_log_match']['read_error'] = log_error
    (output / 'planning_report.json').write_text(
        json.dumps(enriched, ensure_ascii=False, allow_nan=False, indent=2) + '\n', encoding='utf-8')
    fields = [
        'index', 'goal_id', 'category', 'checkpoint_s', 'status', 'reason', 'attempted',
        'accepted', 'action_status', 'goal_x', 'goal_y', 'goal_z', 'goal_yaw', 'goal_source_line',
        'due_clock_ns', 'request_clock_ns', 'request_offset_s', 'dispatch_delay_ros_s',
        'elapsed_wall_s', 'planning_time_s', 'server_message', 'server_message_line_no',
        'server_message_wallstamp', 'structured_reasons', 'failure_diagnostics_json',
        'path_frame_id', 'path_point_count', 'path_length_xy_m', 'path_length_3d_m',
        'path_first_to_last_straight_xy_m', 'path_length_to_straight_ratio',
        'raw_start_to_goal_straight_xy_m', 'raw_start_to_path_first_xy_m',
        'goal_error_xy_m', 'goal_error_yaw_rad', 'start_tf_frame_id', 'start_tf_child_frame_id',
        'start_tf_stamp_ns', 'start_tf_x', 'start_tf_y', 'start_tf_z', 'start_tf_yaw',
        'start_tf_quaternion_json', 'checkpoint_reason', 'request_reason',
        'request_snapshot_json', 'result_snapshot_json']
    def encoded(value):
        return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
    with (output / 'planning_report.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in enriched.get('rows', []):
            goal, path = row.get('goal') or {}, row.get('path') or {}
            start, event = row.get('start_tf') or {}, row.get('server_failure_event') or {}
            position = list(start.get('position') or []) + [None, None, None]
            diagnostics = row.get('failure_diagnostics') or []
            value = {name: row.get(name) for name in (
                'index', 'category', 'checkpoint_s', 'status', 'reason', 'attempted',
                'accepted', 'action_status', 'due_clock_ns', 'request_clock_ns',
                'request_offset_s', 'dispatch_delay_ros_s', 'elapsed_wall_s', 'planning_time_s',
                'raw_start_to_goal_straight_xy_m', 'raw_start_to_path_first_xy_m')}
            value.update(goal_id=goal.get('id'), goal_x=goal.get('x'), goal_y=goal.get('y'),
                         goal_z=goal.get('z'), goal_yaw=goal.get('yaw'),
                         goal_source_line=goal.get('source_line'),
                         server_message=row.get('server_failure_message'),
                         server_message_line_no=event.get('line_no'),
                         server_message_wallstamp=event.get('wallstamp'),
                         structured_reasons=';'.join(str(item['diagnostic']['reason'])
                             for item in diagnostics if isinstance(item.get('diagnostic'), dict)
                             and 'reason' in item['diagnostic']),
                         failure_diagnostics_json=encoded(diagnostics),
                         path_frame_id=path.get('frame_id'), path_point_count=path.get('point_count'),
                         path_length_xy_m=path.get('length_xy_m'),
                         path_length_3d_m=path.get('length_3d_m'),
                         path_first_to_last_straight_xy_m=path.get('straight_xy_m'),
                         path_length_to_straight_ratio=path.get('length_to_straight_ratio'),
                         goal_error_xy_m=path.get('goal_error_xy_m'),
                         goal_error_yaw_rad=path.get('goal_error_yaw_rad'),
                         start_tf_frame_id=start.get('frame_id'),
                         start_tf_child_frame_id=start.get('child_frame_id'),
                         start_tf_stamp_ns=start.get('stamp_ns'), start_tf_x=position[0],
                         start_tf_y=position[1], start_tf_z=position[2], start_tf_yaw=start.get('yaw'),
                         start_tf_quaternion_json=encoded(start.get('quaternion')),
                         checkpoint_reason=(row.get('checkpoint_snapshot') or {}).get('reason'),
                         request_reason=(row.get('request_snapshot') or {}).get('reason'),
                         request_snapshot_json=encoded(row.get('request_snapshot')),
                         result_snapshot_json=encoded(row.get('result_snapshot')))
            writer.writerow(value)
    return enriched


class PlanningEvaluation:
    """Nonblocking, serial global-planning queries over the genuine replay TF."""
    diagnostic_settle_s = .15
    ready_heartbeat_max_age_s = .5
    map_status_max_age_s = 1.5

    def __init__(self, node, config, bag_start_ns, timeout_s=5.0, *,
                 evidence_output=None, live_obstacle_topic='/terrain/obstacles_measured_global'):
        self.node = node
        self.config = _validated_config(config)
        if isinstance(bag_start_ns, bool) or not isinstance(bag_start_ns, int) or bag_start_ns < 0:
            raise ValueError('bag_start_ns must be a nonnegative integer')
        self.bag_start_ns = bag_start_ns
        self.timeout_s = _number(timeout_s, 'timeout_s')
        if self.timeout_s <= 0:
            raise ValueError('timeout_s must be positive')
        self._clock_ns = 0
        self._previous_clock_ns = None
        self._checkpoint_index = 0
        self._queue = deque()
        self._active = None
        self._rows = []
        self._observations = {}
        self._failures = []
        self._failure_sequence = 0
        self._finished = False
        self._finish_reason = None
        self._closed = False
        # A timed-out planner action may still be executing after cancellation
        # was requested. Do not overlap any later query with that unknown work.
        self._unresolved_timeout = False
        self._events = []
        self.client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.evidence = None
        self.evidence_report = None
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._subscriptions = [
            node.create_subscription(Bool, '/offline_localization/ready',
                                     lambda message: self._observe('ready', value=bool(message.data)), state_qos),
            node.create_subscription(String, '/offline_localization/status',
                                     lambda message: self._observe('status', value=message.data), state_qos),
            node.create_subscription(String, '/planner/map_status', self._on_map_status, state_qos),
            node.create_subscription(String, '/planner/start_failure', self._on_failure, state_qos),
            node.create_subscription(Odometry, '/Odometry',
                                     lambda message: self._on_odometry('lio', message), qos_profile_sensor_data),
            node.create_subscription(Odometry, '/odom',
                                     lambda message: self._on_odometry('odom', message), qos_profile_sensor_data),
            node.create_subscription(PointCloud2, '/terrain/obstacles', self._on_terrain,
                                     qos_profile_sensor_data),
            node.create_subscription(DiagnosticArray, '/relocalization/status', self._on_localization,
                                     state_qos)]
        if evidence_output is not None:
            from .obstacle_evidence import ObstacleEvidence
            self.evidence = ObstacleEvidence(evidence_output, self.tf_buffer,
                                             obstacle_topic=live_obstacle_topic)
            for topic in dict.fromkeys((self.evidence.raw_topic, self.evidence.registered_topic,
                                         live_obstacle_topic)):
                self._subscriptions.append(node.create_subscription(PointCloud2, topic,
                    lambda message, topic=topic: self.evidence.observe(topic, message, self._clock_ns),
                    qos_profile_sensor_data))
            self._subscriptions.append(node.create_subscription(Odometry, '/Odometry',
                lambda message: self.evidence.observe_odometry(message, self._clock_ns),
                qos_profile_sensor_data))

    def write_evidence(self):
        if self.evidence is not None and self.evidence_report is None:
            self.evidence_report = self.evidence.write(self._clock_ns)
        return self.evidence_report

    def _ros_now_ns(self):
        return int(self.node.get_clock().now().nanoseconds)

    def _observe(self, name, **values):
        self._observations[name] = dict(received_wall=time.monotonic(),
                                       observed_clock_ns=self._clock_ns, **values)

    def _on_map_status(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('map status is not an object')
            self._observe('map_status', value=value)
        except (TypeError, ValueError):
            self._observe('map_status', value=None, error='invalid_map_status_json', raw=message.data)

    def _on_failure(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('failure is not an object')
        except (TypeError, ValueError):
            value = dict(raw=message.data, reason='unparsed_planner_failure')
        self._failure_sequence += 1
        event = dict(sequence=self._failure_sequence, received_wall=time.monotonic(),
                     observed_clock_ns=self._clock_ns, diagnostic=value)
        if self.evidence is not None:
            context = dict(failure_sequence=self._failure_sequence)
            if self._active is not None:
                row = self._active['row']
                context.update(active_row_index=row['index'], active_goal_id=row['goal']['id'],
                               active_checkpoint_s=row['checkpoint_s'],
                               source_request_matches_active=(type(value.get('request_stamp_ns')) is int
                                   and value['request_stamp_ns'] >= row.get('request_clock_ns', 0)))
            event['obstacle_evidence_event_id'] = self.evidence.capture(
                value, self._clock_ns, context=context)
        self._failures.append(event)

    def _on_odometry(self, name, message):
        p, q, twist = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
        self._observe(name, stamp_ns=_stamp_ns(message.header.stamp),
                      frame_id=message.header.frame_id, child_frame_id=message.child_frame_id,
                      position=[p.x, p.y, p.z], quaternion=[q.x, q.y, q.z, q.w],
                      linear_velocity=[twist.linear.x, twist.linear.y, twist.linear.z],
                      angular_velocity=[twist.angular.x, twist.angular.y, twist.angular.z])

    def _on_terrain(self, message):
        self._observe('terrain', stamp_ns=_stamp_ns(message.header.stamp),
                      frame_id=message.header.frame_id, points=int(message.width) * int(message.height))

    def _on_localization(self, message):
        statuses = [dict(name=status.name, level=_diagnostic_level(status.level), message=status.message,
                         values={value.key: value.value for value in status.values})
                    for status in message.status]
        self._observe('localization', stamp_ns=_stamp_ns(message.header.stamp), statuses=statuses)

    def _snapshot(self):
        wall, ros = time.monotonic(), self._ros_now_ns()
        result = dict(clock_ns=ros, checkpoint_clock_ns=self._clock_ns, inputs={})
        for name, observation in self._observations.items():
            value = copy.deepcopy(observation)
            value['received_age_wall_s'] = wall - value.pop('received_wall')
            if 'stamp_ns' in value:
                value['message_age_ros_s'] = (ros - value['stamp_ns']) * 1e-9
            result['inputs'][name] = value
        return result

    def _readiness(self):
        snapshot = self._snapshot()
        inputs = snapshot['inputs']
        ready = inputs.get('ready', {})
        map_status = inputs.get('map_status', {})
        reason = None
        if self._unresolved_timeout:
            reason = 'blocked_by_unresolved_timeout'
        elif ready.get('value') is not True:
            reason = 'localization_not_ready:' + str(inputs.get('status', {}).get('value', 'missing'))
        elif not 0 <= ready['received_age_wall_s'] <= self.ready_heartbeat_max_age_s:
            reason = 'localization_ready_heartbeat_stale'
        elif map_status.get('value') is None:
            reason = 'map_status_missing_or_invalid'
        elif map_status['value'].get('ready') is not True or map_status['value'].get('faulted') is True:
            reason = 'map_not_verified:' + str(map_status['value'].get('reason', 'unknown'))
        elif not 0 <= map_status['received_age_wall_s'] <= self.map_status_max_age_s:
            reason = 'map_status_heartbeat_stale'
        try:
            server_ready = bool(self.client.server_is_ready())
        except Exception as error:
            server_ready = False
            snapshot['action_server_error'] = str(error)
        snapshot['action_server_ready'] = server_ready
        if reason is None and not server_ready:
            reason = 'planner_action_server_not_ready'
        snapshot['eligible'] = reason is None
        snapshot['reason'] = reason or 'ready'
        return reason is None, snapshot

    def _new_row(self, checkpoint, goal):
        row = dict(index=len(self._rows), checkpoint_s=checkpoint,
                   due_clock_ns=self.bag_start_ns + round(checkpoint * 1e9),
                   goal=copy.deepcopy(goal), category=goal['category'], status='queued',
                   attempted=False, accepted=None, reason=None, failure_diagnostics=[])
        self._rows.append(row)
        return row

    def _block(self, row, reason, snapshot=None, status='blocked'):
        row.update(status=status, reason=reason,
                   observed_clock_ns=self._clock_ns,
                   dispatch_delay_ros_s=(self._clock_ns - row['due_clock_ns']) * 1e-9)
        if snapshot is not None:
            row['request_snapshot'] = snapshot

    def tick(self, clock_ns):
        if self._closed:
            return
        if isinstance(clock_ns, bool) or not isinstance(clock_ns, int) or clock_ns < 0:
            raise ValueError('clock_ns must be a nonnegative integer')
        if clock_ns and self._previous_clock_ns is not None and clock_ns < self._previous_clock_ns:
            self._events.append(dict(event='clock_rewound', previous_ns=self._previous_clock_ns,
                                     current_ns=clock_ns))
            self.finish(clock_ns, reason='clock_rewound')
            if self._active is not None and self._active['phase'] != 'settling':
                self._timeout(self._active, reason='clock_rewound')
        self._clock_ns = clock_ns
        if self.evidence is not None:
            self.evidence.tick(clock_ns)
        if clock_ns:
            self._previous_clock_ns = clock_ns
        if not self._finished and clock_ns >= self.bag_start_ns and clock_ns > 0:
            checkpoints = self.config['checkpoints_s']
            while self._checkpoint_index < len(checkpoints):
                checkpoint = checkpoints[self._checkpoint_index]
                if clock_ns < self.bag_start_ns + round(checkpoint * 1e9):
                    break
                self._checkpoint_index += 1
                eligible, snapshot = self._readiness()
                for goal in self.config['goals']:
                    row = self._new_row(checkpoint, goal)
                    row['checkpoint_snapshot'] = copy.deepcopy(snapshot)
                    if eligible:
                        self._queue.append(row)
                    else:
                        self._block(row, snapshot['reason'], copy.deepcopy(snapshot))
        active = self._active
        if active is not None:
            if active['phase'] == 'settling':
                if time.monotonic() >= active['settle_until']:
                    self._complete(active)
            elif time.monotonic() >= active['deadline']:
                self._timeout(active)
        if self._active is None and not self._finished:
            while self._queue and self._active is None:
                row = self._queue.popleft()
                eligible, snapshot = self._readiness()
                if not eligible:
                    self._block(row, snapshot['reason'], snapshot)
                else:
                    self._dispatch(row, snapshot)

    def _dispatch(self, row, snapshot):
        goal = ComputePathToPose.Goal()
        goal.planner_id, goal.use_start = 'GridBased', False
        goal.goal.header.frame_id = 'map'
        now = self.node.get_clock().now()
        goal.goal.header.stamp = now.to_msg()
        target = row['goal']
        goal.goal.pose.position.x, goal.goal.pose.position.y = target['x'], target['y']
        goal.goal.pose.position.z = target['z']
        goal.goal.pose.orientation.z = math.sin(target['yaw'] / 2)
        goal.goal.pose.orientation.w = math.cos(target['yaw'] / 2)
        row.update(request_clock_ns=int(now.nanoseconds),
                   request_offset_s=(int(now.nanoseconds) - self.bag_start_ns) * 1e-9,
                   dispatch_delay_ros_s=(int(now.nanoseconds) - row['due_clock_ns']) * 1e-9,
                   request_snapshot=snapshot)
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            t, q = transform.transform.translation, transform.transform.rotation
            values = [t.x, t.y, t.z, q.x, q.y, q.z, q.w]
            if not all(math.isfinite(value) for value in values):
                raise ValueError('nonfinite map->base_link transform')
            row['start_tf'] = dict(frame_id=transform.header.frame_id,
                                   child_frame_id=transform.child_frame_id,
                                   stamp_ns=_stamp_ns(transform.header.stamp),
                                   position=values[:3], quaternion=values[3:],
                                   yaw=_quaternion_yaw(values[3:]))
            row['raw_start_to_goal_straight_xy_m'] = math.hypot(
                target['x'] - t.x, target['y'] - t.y)
        except Exception as error:
            self._block(row, 'robot_transform_unavailable:' + str(error), snapshot)
            return
        wall = time.monotonic()
        active = dict(row=row, phase='sending', started_wall=wall, deadline=wall + self.timeout_s,
                      failure_baseline=self._failure_sequence, handle=None)
        row.update(attempted=True, status='pending')
        self._active = active
        try:
            future = self.client.send_goal_async(goal)
            active['send_future'] = future
            future.add_done_callback(lambda reply: self._on_goal_reply(active, reply))
        except Exception as error:
            self._terminal(active, 'failed', 'goal_send_exception:' + str(error))

    def _on_goal_reply(self, active, future):
        try:
            handle = future.result()
        except Exception as error:
            if self._active is active and active['phase'] != 'settling':
                self._terminal(active, 'failed', 'goal_reply_exception:' + str(error))
            return
        if self._active is not active or active['phase'] == 'settling' or self._closed:
            # A timed-out send may be accepted later. Cancel that old handle,
            # never adopt its result as the current query.
            if handle is not None and handle.accepted:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
            return
        active['handle'] = handle
        active['row']['accepted'] = bool(handle is not None and handle.accepted)
        if handle is None or not handle.accepted:
            self._terminal(active, 'failed', 'action_goal_rejected')
            return
        active['phase'] = 'result'
        try:
            result_future = handle.get_result_async()
            active['result_future'] = result_future
            result_future.add_done_callback(lambda result: self._on_result(active, result))
        except Exception as error:
            self._terminal(active, 'failed', 'get_result_exception:' + str(error))

    def _on_result(self, active, future):
        if self._active is not active or active['phase'] == 'settling' or self._closed:
            return
        try:
            response = future.result()
            result = response.result
            active['row']['action_status'] = int(response.status)
            active['row']['planning_time_s'] = _stamp_ns(result.planning_time) * 1e-9
            metrics = path_metrics(result.path, active['row']['goal'])
            raw_start = active['row']['start_tf']['position']
            metrics['raw_start_to_goal_straight_xy_m'] = active['row'][
                'raw_start_to_goal_straight_xy_m']
            metrics['raw_start_to_path_first_xy_m'] = (
                math.hypot(metrics['points'][0]['x'] - raw_start[0],
                           metrics['points'][0]['y'] - raw_start[1])
                if metrics['points'] else None)
            active['row']['raw_start_to_path_first_xy_m'] = metrics[
                'raw_start_to_path_first_xy_m']
            active['row']['path'] = metrics
            success = response.status == GoalStatus.STATUS_SUCCEEDED and metrics['valid']
            self._terminal(active, 'succeeded' if success else 'failed',
                           'valid_path' if success else ('invalid_returned_path' if
                               response.status == GoalStatus.STATUS_SUCCEEDED else 'action_not_succeeded'))
        except Exception as error:
            self._terminal(active, 'failed', 'result_exception:' + str(error))

    def _terminal(self, active, status, reason):
        if self._active is not active or active['phase'] == 'settling':
            return
        row = active['row']
        row.update(status=status, reason=reason, result_clock_ns=self._ros_now_ns(),
                   elapsed_wall_s=time.monotonic() - active['started_wall'],
                   result_snapshot=self._snapshot())
        active['phase'] = 'settling'
        active['settle_until'] = time.monotonic() + self.diagnostic_settle_s

    def _timeout(self, active, reason='action_result_timeout'):
        self._unresolved_timeout = True
        handle = active.get('handle')
        if handle is not None and handle.accepted:
            try:
                handle.cancel_goal_async()
                active['row']['cancel_requested'] = True
            except Exception as error:
                active['row']['cancel_error'] = str(error)
        self._terminal(active, 'timeout', reason)

    def _complete(self, active):
        row = active['row']
        # A retained failure can arrive after a new request. Require both the
        # receive sequence and its source request timestamp to belong to this
        # exact interval. Unstamped diagnostics remain visible separately.
        row['failure_diagnostics'] = [copy.deepcopy(item) for item in self._failures
            if item['sequence'] > active['failure_baseline'] and
            type(item['diagnostic'].get('request_stamp_ns')) is int and
            row['request_clock_ns'] <= item['diagnostic']['request_stamp_ns'] <= row['result_clock_ns']]
        self._active = None

    def finish(self, clock_ns, reason='end_of_replay'):
        """Stop new requests; retain an in-flight query for the caller's grace loop."""
        if self._finished:
            return
        self._clock_ns = int(clock_ns)
        self._finished = True
        self._finish_reason = reason
        while self._queue:
            self._block(self._queue.popleft(), reason, status='not_executed_end_of_replay')
        checkpoints = self.config['checkpoints_s']
        while self._checkpoint_index < len(checkpoints):
            checkpoint = checkpoints[self._checkpoint_index]
            self._checkpoint_index += 1
            for goal in self.config['goals']:
                self._block(self._new_row(checkpoint, goal), reason, status='not_reached')

    def is_idle(self):
        return self._active is None and not self._queue

    def summary(self):
        def statistics(rows):
            counts = Counter(row['status'] for row in rows)
            attempts = sum(row['attempted'] for row in rows)
            succeeded = counts['succeeded']
            return dict(scheduled=len(rows), action_attempts=attempts,
                        succeeded=succeeded, failed=counts['failed'], timeout=counts['timeout'],
                        blocked=counts['blocked'], not_reached=counts['not_reached'],
                        not_executed_end_of_replay=counts['not_executed_end_of_replay'],
                        pending=counts['pending'] + counts['queued'],
                        success_rate_of_attempts=succeeded / attempts if attempts else None,
                        success_rate_of_scheduled=succeeded / len(rows) if rows else None,
                        status_counts=dict(counts))
        by_category = {name: statistics([row for row in self._rows if row['category'] == name])
                       for name in ('navigation', 'negative_control')}
        assigned = {event['sequence'] for row in self._rows for event in row['failure_diagnostics']}
        return dict(schema_version=1, scope='Recorded-sensor localization and ComputePathToPose only; '
                    'no controller, behavior tree or robot movement.', config=copy.deepcopy(self.config),
                    bag_start_ns=self.bag_start_ns, timeout_s=self.timeout_s,
                    finished=self._finished, finish_reason=self._finish_reason,
                    is_idle=self.is_idle(), rows=copy.deepcopy(self._rows),
                    statistics=statistics(self._rows), by_category=by_category,
                    negative_control_rejected=sum(
                        row['status'] == 'failed' and row.get('accepted') is True and
                        row.get('action_status') == GoalStatus.STATUS_ABORTED
                        for row in self._rows if row['category'] == 'negative_control'),
                    negative_control_no_safe_goal_ground=sum(
                        row['status'] == 'failed' and row.get('accepted') is True and
                        row.get('action_status') == GoalStatus.STATUS_ABORTED and any(
                            event['diagnostic'].get('reason') == 'no_safe_goal_ground'
                            for event in row['failure_diagnostics'])
                        for row in self._rows if row['category'] == 'negative_control'),
                    negative_control_unexpected_success=by_category['negative_control']['succeeded'],
                    unresolved_timeout=self._unresolved_timeout,
                    unassigned_failure_diagnostics=[copy.deepcopy(event) for event in self._failures
                                                    if event['sequence'] not in assigned],
                    events=copy.deepcopy(self._events),
                    obstacle_evidence=copy.deepcopy(self.evidence_report))

    def close(self):
        if self._closed:
            return
        self.finish(self._clock_ns, reason='evaluation_closed')
        if self._active is not None:
            active = self._active
            if active['phase'] != 'settling':
                self._timeout(active, reason='evaluation_closed_before_result')
            self._complete(active)
        self._closed = True
        for subscription in self._subscriptions:
            self.node.destroy_subscription(subscription)
        self._subscriptions.clear()
        self.client.destroy()
        self.tf_listener.unregister()
