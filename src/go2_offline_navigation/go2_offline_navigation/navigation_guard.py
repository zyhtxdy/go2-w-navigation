"""Cancel old navigation work before reopening the offline velocity gate."""
from dataclasses import dataclass, field
import copy
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


ACTIVE = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING}
TERMINAL = {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED}


def goal_id(info):
    return bytes(info.goal_id.uuid)


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


@dataclass
class Cancellation:
    acknowledged: bool = False
    goals: set = field(default_factory=set)
    individual: set = field(default_factory=set)
    sent_individual: set = field(default_factory=set)
    future: object = None
    request_id: int = 0
    request_started: float = 0.0
    retry_at: float = 0.0
    error: str = ''


class NavigationGuard:
    """No manually enabled state: cancellation and a quiet input reopen the gate.

    A localization-only observer with no commands or active goals never needs
    navigation services. Once a command has passed, missing services fail closed.
    """
    # These are the actions enabled by real_nav2.launch.py / its behavior tree.
    # Recovery behaviors can publish velocity without going through follow_path.
    names = ('navigate_to_pose', 'navigate_through_poses', 'follow_waypoints',
             'follow_path', 'backup', 'drive_on_heading', 'wait')

    def __init__(self, node, command_timeout, is_ready, on_unready_work=None):
        self.node, self.command_timeout, self.is_ready = node, command_timeout, is_ready
        # The adapter owns the distinction between a recoverable input outage
        # and a hard fault. Action-status delivery must not bypass that policy.
        # Standalone users without a policy callback retain fail-closed cancel.
        self.on_unready_work = on_unready_work
        self.clients = {name: node.create_client(CancelGoal, '/' + name + '/_action/cancel_goal')
                        for name in self.names}
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = [node.create_subscription(
            GoalStatusArray, '/' + name + '/_action/status',
            lambda message, name=name: self.on_status(name, message), qos) for name in self.names]
        self.active = {name: {} for name in self.names}
        self.terminal = {name: {} for name in self.names}
        self.pending = {}
        self.blocked = False
        self.epoch = 0
        self.cutoff = None
        self.forwarded_nonzero = False
        self.motion_generation = 0
        self.terminal_motion_generation = None
        self.motion_goals = set()
        self.forwarded_zero_after_motion = False
        self.last_forwarded_nonzero_at = None
        self.last_nonzero_at = None
        self.first_ready_stamp = None
        self.uuid_only = False
        self.holding = False

    def begin_hold(self):
        """Keep the parent task during a bounded input outage; never unlock motion."""
        if self.blocked or self.first_ready_stamp is None or not self.has_navigation_work():
            return False
        self.holding = True
        return True

    def has_navigation_work(self):
        # A localization/planning observer has no task to recover or cancel.
        # Keep untracked forwarded motion conservative: missing action status
        # must never be interpreted as proof that an old controller is gone.
        active = any(self.active.values())
        quiet_since = max(value for value in (
            self.last_forwarded_nonzero_at, self.last_nonzero_at, float('-inf'))
            if value is not None)
        if (not self.blocked and not self.holding and not active
                and self.terminal_motion_generation == self.motion_generation
                and self.forwarded_zero_after_motion
                and time.monotonic() - quiet_since >= self.command_timeout):
            self._clear_forwarded_motion()
        return self.forwarded_nonzero or active

    def _clear_forwarded_motion(self):
        self.forwarded_nonzero = False
        self.motion_goals.clear()
        self.terminal_motion_generation = None
        self.forwarded_zero_after_motion = False

    def end_hold(self):
        self.holding = False

    def note_ready(self):
        if self.first_ready_stamp is None:
            self.first_ready_stamp = stamp_ns(self.node.get_clock().now().to_msg())
        if not self.blocked:
            self.uuid_only = False

    def on_clock_jump(self):
        if self.holding:
            self.pause()
        self.first_ready_stamp = None
        if not self.blocked:
            self.uuid_only = False
            return
        self.uuid_only = True
        # Goal acceptance times belong to the old ROS-clock epoch. Never retry
        # that epoch's time-based cancellation against newly accepted goals.
        self.epoch += 1
        self.cutoff = copy.deepcopy(self.node.get_clock().now().to_msg())
        for state in self.pending.values():
            state.request_id += 1
            if state.future is not None:
                state.future.cancel()
            state.future = None
            state.individual.update(state.goals)
            state.sent_individual.clear()
            state.retry_at = 0.0
            state.error = ''

    def note_input(self, nonzero):
        if nonzero:
            self.last_nonzero_at = time.monotonic()

    def note_forwarded(self, nonzero):
        if nonzero:
            self.forwarded_nonzero = True
            self.motion_generation += 1
            self.last_forwarded_nonzero_at = time.monotonic()
            self.forwarded_zero_after_motion = False
            self.terminal_motion_generation = None
            self.motion_goals.update((name, key) for name, goals in self.active.items() for key in goals)
        elif self.forwarded_nonzero:
            self.forwarded_zero_after_motion = True

    def on_status(self, name, message):
        previous = set(self.active[name])
        self.active[name] = {goal_id(item.goal_info): item.goal_info for item in message.status_list
                             if item.status in ACTIVE}
        if self.forwarded_nonzero:
            self.motion_goals.update((name, key) for key in self.active[name])
        new_motion_terminal = False
        for item in message.status_list:
            if item.status in TERMINAL:
                key = goal_id(item.goal_info)
                # Repeated transient-local terminal statuses cannot acknowledge
                # later, untracked motion. Only a new terminal transition of a
                # goal seen active during this motion is completion evidence.
                new_motion_terminal |= (key not in self.terminal[name]
                                        and (name, key) in self.motion_goals)
                self.motion_goals.discard((name, key))
                self.terminal[name][key] = time.monotonic()
        if new_motion_terminal and not any(self.active.values()) and not self.motion_goals:
            self.terminal_motion_generation = self.motion_generation
        # Retain terminal reports that arrived before the cancellation response.
        # Bound unrelated finished-goal history without evicting pending goals.
        if len(self.terminal[name]) > 2000:
            pending = self.pending.get(name, Cancellation()).goals
            self.terminal[name] = {key: value for key, value in self.terminal[name].items()
                                   if key in pending or time.monotonic() - value < 30.0}
        late_old_goal = self.first_ready_stamp is not None and any(
            key not in previous and key not in self.terminal[name]
            and stamp_ns(info.stamp) < self.first_ready_stamp
            for key, info in self.active[name].items())
        if not self.blocked and ((not self.is_ready() and not self.holding and self.active[name]) or late_old_goal):
            if late_old_goal or self.first_ready_stamp is None or self.on_unready_work is None:
                # An adapter restart must not silently resume an existing Nav2
                # goal, even when a recovery-policy callback is installed.
                self.pause()
            else:
                # A goal's status can arrive after the input outage. Let the
                # same adapter policy handle either callback ordering.
                self.on_unready_work()
        elif self.blocked:
            if self.uuid_only:
                # A new epoch cannot order an unfamiliar goal against the old
                # failure. Do not cancel such a goal based on its timestamp.
                # An idle status from an available server is authoritative when
                # the old pending request had no known goal UUID to cancel.
                state = self.pending.get(name)
                if state is not None and not self.active[name] and self.clients[name].service_is_ready():
                    state.acknowledged = True
                return
            old = {key: info for key, info in self.active[name].items()
                   if stamp_ns(info.stamp) <= stamp_ns(self.cutoff)}
            if old:
                state = self.pending.setdefault(name, Cancellation())
                new = set(old) - state.goals - self.terminal[name].keys()
                state.goals.update(old)
                if new and state.acknowledged:
                    state.acknowledged = False

    def pause(self):
        self.holding = False
        if self.blocked:
            return
        names = {name for name, client in self.clients.items() if client.service_is_ready()}
        names.update(name for name, goals in self.active.items() if goals)
        if not names and not self.forwarded_nonzero:
            return
        if not names:
            names.add('follow_path')
        self.epoch += 1
        self.cutoff = copy.deepcopy(self.node.get_clock().now().to_msg())
        # Zero ID and zero time means cancel *all*, including later goals. A
        # simulation waiting for /clock must use known old UUIDs instead.
        self.uuid_only |= stamp_ns(self.cutoff) == 0
        self.pending = {}
        for name in names:
            state = Cancellation(goals=set(self.active[name]))
            # After a ROS-clock rewind an old goal's acceptance stamp can exceed
            # the fixed cutoff; cancel those known old UUIDs individually.
            state.individual = {key for key, info in self.active[name].items()
                                if self.uuid_only or stamp_ns(info.stamp) > stamp_ns(self.cutoff)}
            self.pending[name] = state
        self.blocked = True

    def _request(self, name, state, target=None):
        client = self.clients[name]
        if not client.service_is_ready():
            state.error = 'navigation_cancel_service_missing:' + name
            return
        request = CancelGoal.Request()
        if target is None:
            if stamp_ns(self.cutoff) <= 0:
                state.error = 'navigation_cancel_clock_unavailable:' + name
                return
            # Keep the failure-time cutoff on every retry, protecting newer goals.
            request.goal_info.stamp = copy.deepcopy(self.cutoff)
        else:
            if not any(target):
                state.error = 'navigation_cancel_invalid_goal_id:' + name
                return
            request.goal_info.goal_id.uuid = list(target)
        state.request_id += 1
        epoch, request_id = self.epoch, state.request_id
        try:
            state.future = client.call_async(request)
            state.request_started = time.monotonic()
            state.error = ''
        except Exception:
            state.error = 'navigation_cancel_request_failed:' + name
            state.retry_at = time.monotonic() + .5
            return

        def completed(future):
            if epoch != self.epoch or self.pending.get(name) is not state or request_id != state.request_id:
                return
            state.future = None
            state.retry_at = time.monotonic() + .5
            try:
                response = future.result()
                if response.return_code == CancelGoal.Response.ERROR_NONE:
                    state.goals.update(goal_id(info) for info in response.goals_canceling)
                    if target is None:
                        state.acknowledged = True
                    else:
                        state.sent_individual.add(target)
                elif target is not None and response.return_code == CancelGoal.Response.ERROR_GOAL_TERMINATED:
                    # This service response is itself authoritative terminal evidence.
                    self.terminal[name][target] = time.monotonic()
                    state.sent_individual.add(target)
                else:
                    state.error = 'navigation_cancel_rejected:' + name
            except Exception:
                state.error = 'navigation_cancel_response_failed:' + name
        state.future.add_done_callback(completed)

    def poll(self):
        if self.holding:
            return False, 'navigation_input_recovery_wait'
        if not self.blocked:
            if self.first_ready_stamp is None and self.last_nonzero_at is not None and \
                    time.monotonic() - self.last_nonzero_at < self.command_timeout:
                return False, 'navigation_command_stream_not_quiet'
            return True, 'ready'
        now = time.monotonic()
        blocked_reason = None
        for name, state in self.pending.items():
            if state.future is not None and now - state.request_started >= 1.0:
                # A late response from this timed-out RPC must not unlock a retry.
                state.request_id += 1
                state.future.cancel()
                state.future = None
                state.error = 'navigation_cancel_response_timeout:' + name
                state.retry_at = now + .5
            if state.future is None and now >= state.retry_at:
                if self.uuid_only:
                    targets = state.goals - state.sent_individual - self.terminal[name].keys()
                    if targets:
                        self._request(name, state, next(iter(targets)))
                    if state.goals:
                        state.acknowledged = not targets
                elif not state.acknowledged:
                    self._request(name, state)
                else:
                    targets = state.individual - state.sent_individual - self.terminal[name].keys()
                    if targets:
                        self._request(name, state, next(iter(targets)))
            if state.future is not None:
                blocked_reason = blocked_reason or 'navigation_cancel_pending:' + name
            elif not state.acknowledged:
                if self.uuid_only and not state.goals:
                    state.error = 'navigation_clock_epoch_unconfirmed:' + name
                blocked_reason = blocked_reason or state.error or 'navigation_cancel_pending:' + name
            elif state.goals - self.terminal[name].keys():
                blocked_reason = blocked_reason or state.error or 'navigation_waiting_for_goal_terminal:' + name
        if blocked_reason:
            return False, blocked_reason
        if self.last_nonzero_at is not None and now - self.last_nonzero_at < self.command_timeout:
            return False, 'navigation_command_stream_not_quiet'
        self.blocked = False
        self.pending.clear()
        self._clear_forwarded_motion()
        self.uuid_only = False
        return True, 'ready'
