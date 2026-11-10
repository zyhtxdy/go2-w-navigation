"""Cancellation faults are tested without any network or robot command node."""
from copy import deepcopy
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Time

from go2_offline_navigation import navigation_guard as module
from go2_offline_navigation.navigation_guard import NavigationGuard


class Future:
    def __init__(self):
        self.callbacks = []
        self.value = None
        self.error = None
        self.cancelled = False

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def result(self):
        if self.error:
            raise self.error
        return self.value

    def finish(self, value=None, error=None):
        self.value, self.error = value, error
        for callback in self.callbacks:
            callback(self)

    def cancel(self):
        self.cancelled = True


class Client:
    def __init__(self):
        self.available = False
        self.requests, self.futures = [], []

    def service_is_ready(self):
        return self.available

    def call_async(self, request):
        self.requests.append(deepcopy(request))
        self.futures.append(Future())
        return self.futures[-1]


class Node:
    def __init__(self):
        self.clients = {}
        self.now = 100

    def create_client(self, _type, topic):
        client = Client()
        self.clients[topic.split('/')[1]] = client
        return client

    def create_subscription(self, *args):
        return args

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=self.now)))


def info(number=1, stamp=90):
    goal = GoalInfo(stamp=Time(sec=stamp))
    goal.goal_id.uuid = [number] * 16
    return goal


def status(goal, state=GoalStatus.STATUS_EXECUTING):
    return GoalStatusArray(status_list=[GoalStatus(goal_info=goal, status=state)])


def response(code=CancelGoal.Response.ERROR_NONE, goals=()):
    return CancelGoal.Response(return_code=code, goals_canceling=list(goals))


@pytest.fixture
def setup(monkeypatch):
    wall = SimpleNamespace(now=0.0)
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: wall.now))
    node, ready = Node(), SimpleNamespace(value=True)
    guard = NavigationGuard(node, .3, lambda: ready.value)
    return guard, node, ready, wall


def test_localization_only_observer_does_not_require_navigation_services(setup):
    guard, _, _, _ = setup
    guard.pause()
    assert guard.poll() == (True, 'ready')


def test_hold_preserves_running_parent_but_never_grants_motion(setup):
    guard, node, ready, _ = setup
    guard.note_ready()
    node.now = 110
    goal = info(1, 105)
    guard.on_status('navigate_to_pose', status(goal))
    ready.value = False
    assert guard.begin_hold()
    guard.on_status('navigate_to_pose', status(goal))
    assert not guard.blocked and not guard.poll()[0]
    assert not any(client.requests for client in node.clients.values())
    guard.end_hold()
    ready.value = True
    assert guard.poll()[0]


@pytest.mark.parametrize('name', NavigationGuard.names)
def test_current_action_status_during_hold_cannot_trigger_parallel_cancel(setup, name):
    guard, node, ready, _ = setup
    guard.note_ready()
    node.now = 110
    goal = info(1, 105)
    guard.on_status(name, status(goal))
    ready.value = False
    assert guard.begin_hold()
    for state in (GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING):
        guard.on_status(name, status(goal, state))
        assert not guard.blocked and not guard.poll()[0]
    assert not any(client.requests for client in node.clients.values())


@pytest.mark.parametrize('name', NavigationGuard.names)
def test_late_active_status_uses_adapter_recovery_policy_once(setup, name):
    _, node, ready, _ = setup
    calls = []

    def classify_outage():
        calls.append(name)
        assert guard.has_navigation_work()
        assert guard.begin_hold()

    guard = NavigationGuard(node, .3, lambda: ready.value,
                            on_unready_work=classify_outage)
    guard.note_ready()
    node.now = 110
    ready.value = False
    # The outage was observed while idle; its task status arrives afterwards.
    goal = info(1, 105)
    guard.on_status(name, status(goal))
    assert calls == [name]
    assert guard.holding and not guard.blocked
    assert not guard.poll()[0]
    # Further status heartbeats cannot start a parallel cancellation or renew
    # the adapter's recovery window once it has entered the hold.
    guard.on_status(name, status(goal))
    assert calls == [name]
    assert not any(client.requests for client in node.clients.values())


def test_adapter_policy_can_cancel_late_active_work_for_hard_fault(setup):
    guard, node, ready, _ = setup
    guard.on_unready_work = guard.pause
    guard.note_ready()
    node.now = 110
    ready.value = False
    guard.on_status('navigate_to_pose', status(info(1, 105)))
    assert guard.blocked and not guard.holding
    assert not guard.poll()[0]
    assert 'navigate_to_pose' in guard.pending


def test_policy_callback_cannot_resume_preexisting_goal_at_startup(setup):
    guard, node, ready, _ = setup
    calls = []
    guard.on_unready_work = lambda: calls.append(True)
    ready.value = False
    guard.on_status('navigate_to_pose', status(info(1, 90)))
    assert not calls
    assert guard.blocked and not guard.holding
    assert not guard.poll()[0]


def test_policy_callback_cannot_resume_late_goal_from_before_first_ready(setup):
    guard, node, ready, _ = setup
    calls = []
    guard.on_unready_work = lambda: calls.append(True)
    guard.note_ready()
    node.now = 110
    ready.value = False
    guard.on_status('navigate_to_pose', status(info(1, 90)))
    assert not calls
    assert guard.blocked and not guard.holding
    assert not guard.poll()[0]


def test_hold_does_not_hide_late_preexisting_goal_from_before_adapter_ready(setup):
    guard, node, ready, _ = setup
    guard.note_ready()
    guard.note_forwarded(True)
    ready.value = False
    assert guard.begin_hold()
    node.clients['follow_path'].available = True
    guard.on_status('follow_path', status(info(1, 90)))
    assert guard.blocked and not guard.holding
    assert not guard.poll()[0]


def test_idle_observer_has_no_task_to_hold_even_when_action_servers_exist(setup):
    guard, node, _, _ = setup
    guard.note_ready()
    for client in node.clients.values():
        client.available = True
    assert not guard.has_navigation_work()
    assert not guard.begin_hold()
    assert not guard.holding and not guard.blocked


def test_untracked_forwarded_motion_still_requires_recovery(setup):
    guard, _, _, _ = setup
    guard.note_ready()
    guard.note_forwarded(True)
    assert guard.has_navigation_work()
    assert guard.begin_hold()


@pytest.mark.parametrize('terminal', sorted(module.TERMINAL))
def test_completed_motion_becomes_idle_only_after_terminal_zero_and_quiet(setup, terminal):
    guard, _, _, wall = setup
    guard.note_ready()
    goal = info(stamp=105)
    guard.on_status('navigate_to_pose', status(goal))
    guard.note_input(True)
    guard.note_forwarded(True)
    guard.on_status('navigate_to_pose', status(goal, terminal))
    wall.now = 1.0
    assert guard.has_navigation_work(), 'A terminal report alone is not a zero command'
    guard.note_forwarded(False)
    assert not guard.has_navigation_work()
    assert not guard.begin_hold(), 'A finished task must not require its old /plan during an idle outage'


def test_completed_motion_waits_for_command_quiet_interval(setup):
    guard, _, _, wall = setup
    guard.note_ready()
    goal = info(stamp=105)
    guard.on_status('follow_path', status(goal))
    guard.note_input(True)
    guard.note_forwarded(True)
    guard.on_status('follow_path', status(goal, GoalStatus.STATUS_SUCCEEDED))
    guard.note_forwarded(False)
    wall.now = .29
    assert guard.has_navigation_work()
    wall.now = .31
    assert not guard.has_navigation_work()


def test_old_repeated_terminal_cannot_clear_later_untracked_motion(setup):
    guard, _, _, wall = setup
    guard.note_ready()
    goal = info(stamp=105)
    guard.on_status('follow_path', status(goal))
    guard.note_forwarded(True)
    terminal = status(goal, GoalStatus.STATUS_SUCCEEDED)
    guard.on_status('follow_path', terminal)
    guard.note_forwarded(False)
    wall.now = .31
    assert not guard.has_navigation_work()
    guard.note_forwarded(True)  # No newly tracked goal accompanies this command.
    guard.on_status('follow_path', terminal)
    guard.note_forwarded(False)
    wall.now = 1.0
    guard.on_status('follow_path', terminal)
    assert guard.has_navigation_work()
    assert guard.begin_hold()


def test_new_unrelated_terminal_does_not_acknowledge_untracked_motion(setup):
    guard, _, _, wall = setup
    guard.note_ready()
    guard.note_forwarded(True)
    guard.on_status('wait', status(info(stamp=105), GoalStatus.STATUS_SUCCEEDED))
    guard.note_forwarded(False)
    wall.now = .31
    assert guard.has_navigation_work()


def test_terminal_before_last_nonzero_does_not_acknowledge_later_command(setup):
    guard, _, _, wall = setup
    goal = info(stamp=105)
    guard.on_status('follow_path', status(goal))
    guard.note_forwarded(True)
    terminal = status(goal, GoalStatus.STATUS_SUCCEEDED)
    guard.on_status('follow_path', terminal)
    guard.note_forwarded(True)
    guard.note_forwarded(False)
    guard.on_status('follow_path', terminal)
    wall.now = .31
    assert guard.has_navigation_work()


def test_disappearing_parent_status_is_not_terminal_confirmation(setup):
    guard, _, _, wall = setup
    parent, child = info(1, 105), info(2, 105)
    guard.on_status('navigate_to_pose', status(parent))
    guard.on_status('follow_path', status(child))
    guard.note_forwarded(True)
    guard.on_status('navigate_to_pose', GoalStatusArray())
    guard.on_status('follow_path', status(child, GoalStatus.STATUS_SUCCEEDED))
    guard.note_forwarded(False)
    wall.now = .31
    assert guard.has_navigation_work(), 'Disappearance from a status array is not completion'
    guard.on_status('navigate_to_pose', status(parent, GoalStatus.STATUS_SUCCEEDED))
    assert not guard.has_navigation_work()


def test_active_status_can_follow_first_command_but_terminal_must_follow_motion(setup):
    guard, _, _, wall = setup
    guard.note_forwarded(True)
    goal = info(stamp=105)
    guard.on_status('follow_path', status(goal))
    guard.on_status('follow_path', status(goal, GoalStatus.STATUS_SUCCEEDED))
    guard.note_forwarded(False)
    wall.now = .31
    assert not guard.has_navigation_work()


def test_empty_status_and_zero_without_terminal_preserve_motion_history(setup):
    guard, _, _, wall = setup
    guard.note_forwarded(True)
    guard.on_status('follow_path', GoalStatusArray())
    guard.note_forwarded(False)
    wall.now = 1.0
    assert guard.has_navigation_work()


def test_late_old_task_is_canceled_during_idle_outage(setup):
    guard, node, ready, _ = setup
    guard.note_ready()
    ready.value = False
    assert not guard.begin_hold()
    node.clients['follow_path'].available = True
    guard.on_status('follow_path', status(info(1, 90)))
    assert guard.blocked and not guard.holding


def test_hold_cannot_bypass_startup_or_existing_cancellation(setup):
    guard, node, _, _ = setup
    assert not guard.begin_hold()
    guard.note_ready()
    node.clients['follow_path'].available = True
    guard.pause()
    assert not guard.begin_hold() and guard.blocked


def test_old_velocity_cannot_resume_when_cancel_services_are_missing(setup):
    guard, node, _, wall = setup
    guard.note_input(True)
    guard.note_forwarded(True)
    guard.pause()
    wall.now = 1
    assert not guard.poll()[0]
    assert not node.clients['follow_path'].requests
    node.clients['follow_path'].available = True
    guard.poll()
    node.clients['follow_path'].futures[-1].finish(response())
    assert guard.poll()[0]


@pytest.mark.parametrize('name', NavigationGuard.names)
def test_every_enabled_navigation_action_is_canceled(setup, name):
    guard, node, _, _ = setup
    client = node.clients[name]
    client.available = True
    goal = info()
    guard.on_status(name, status(goal))
    guard.pause()
    assert not guard.poll()[0]
    assert client.requests[-1].goal_info.stamp.sec == 100
    client.futures[-1].finish(response(goals=[goal]))
    assert not guard.poll()[0], 'A cancellation ACK is not a terminal action status'
    guard.on_status(name, status(goal, GoalStatus.STATUS_CANCELED))
    assert guard.poll()[0]


def test_all_endpoints_receive_cancel_without_waiting_for_parent_terminal(setup):
    guard, node, _, _ = setup
    for name in ('navigate_to_pose', 'follow_path', 'backup'):
        node.clients[name].available = True
        guard.on_status(name, status(info()))
    guard.pause()
    guard.poll()
    assert all(node.clients[name].requests for name in ('navigate_to_pose', 'follow_path', 'backup'))


def test_terminal_status_can_arrive_before_cancel_response(setup):
    guard, node, _, _ = setup
    node.clients['follow_path'].available = True
    goal = info()
    guard.on_status('follow_path', status(goal))
    guard.pause(); guard.poll()
    guard.on_status('follow_path', status(goal, GoalStatus.STATUS_ABORTED))
    assert not guard.poll()[0]
    node.clients['follow_path'].futures[-1].finish(response(goals=[goal]))
    assert guard.poll()[0]


def test_rejected_or_failed_cancel_remains_blocked_then_retries(setup):
    guard, node, _, wall = setup
    client = node.clients['follow_path']; client.available = True
    guard.pause(); guard.poll()
    client.futures[-1].finish(response(CancelGoal.Response.ERROR_REJECTED))
    assert not guard.poll()[0]
    wall.now = .6; guard.poll()
    client.futures[-1].finish(error=RuntimeError('service disappeared'))
    assert not guard.poll()[0]
    wall.now = 1.2; guard.poll()
    client.futures[-1].finish(response())
    assert guard.poll()[0]


def test_cancel_retry_keeps_failure_cutoff_and_leaves_new_goal_alone(setup):
    guard, node, _, wall = setup
    client = node.clients['follow_path']; client.available = True
    old = info(1, 90)
    guard.on_status('follow_path', status(old))
    guard.pause(); guard.poll()
    client.futures[-1].finish(response(CancelGoal.Response.ERROR_REJECTED))
    node.now = 110
    guard.on_status('follow_path', status(info(2, 105)))
    wall.now = 1; guard.poll()
    assert all(request.goal_info.stamp.sec == 100 for request in client.requests)
    client.futures[-1].finish(response(goals=[old]))
    guard.on_status('follow_path', status(old, GoalStatus.STATUS_CANCELED))
    assert guard.poll()[0]


def test_timeout_late_response_cannot_ack_a_new_request(setup):
    guard, node, _, wall = setup
    client = node.clients['follow_path']; client.available = True
    guard.pause(); guard.poll()
    first = client.futures[-1]
    wall.now = 1.1; guard.poll()
    assert first.cancelled and not guard.poll()[0]
    wall.now = 1.7; guard.poll()
    second = client.futures[-1]
    first.finish(response())
    assert not guard.poll()[0]
    second.finish(response())
    assert guard.poll()[0]


def test_late_previous_epoch_cannot_release_current_pause(setup):
    guard, node, _, wall = setup
    client = node.clients['follow_path']; client.available = True
    guard.pause(); guard.poll()
    old = client.futures[-1]
    old.finish(response())
    assert guard.poll()[0]
    guard.pause(); guard.poll()
    old.finish(response())
    assert not guard.poll()[0]
    client.futures[-1].finish(response())
    assert guard.poll()[0]


def test_continuing_old_nonzero_commands_keep_gate_closed_after_ack(setup):
    guard, node, _, wall = setup
    client = node.clients['follow_path']; client.available = True
    guard.note_forwarded(True)
    guard.pause(); guard.poll()
    client.futures[-1].finish(response())
    for instant in (.2, .4, .6, .8):
        wall.now = instant
        guard.note_input(True)
        assert not guard.poll()[0]
    wall.now = 1.09
    assert not guard.poll()[0]
    guard.note_input(False)
    wall.now = 1.11
    assert guard.poll()[0]


def test_adapter_restart_with_existing_goal_cancels_before_first_ready(setup):
    guard, node, ready, _ = setup
    client = node.clients['navigate_to_pose']; client.available = True
    ready.value = False
    goal = info()
    guard.on_status('navigate_to_pose', status(goal))
    assert guard.blocked
    guard.poll(); client.futures[-1].finish(response(goals=[goal]))
    assert not guard.poll()[0]
    guard.on_status('navigate_to_pose', status(goal, GoalStatus.STATUS_CANCELED))
    assert guard.poll()[0]


def test_late_discovered_preexisting_goal_blocks_even_after_initial_ready(setup):
    guard, node, _, _ = setup
    guard.note_ready()
    node.now = 110
    guard.on_status('follow_path', status(info(1, 90)))
    assert guard.blocked


def test_new_goal_after_first_ready_is_allowed(setup):
    guard, node, _, _ = setup
    guard.note_ready()
    node.now = 110
    guard.on_status('follow_path', status(info(1, 105)))
    assert not guard.blocked


def test_new_goal_at_same_paused_clock_stamp_as_first_ready_is_allowed(setup):
    guard, _, _, _ = setup
    guard.note_ready()
    guard.on_status('follow_path', status(info(1, 100)))
    assert not guard.blocked


def test_initial_old_velocity_stream_must_be_quiet_before_first_ready(setup):
    guard, _, _, wall = setup
    for instant in (.1, .3, .5, .7):
        wall.now = instant
        guard.note_input(True)
        assert not guard.poll()[0]
    wall.now = 1.01
    assert guard.poll()[0]
    guard.note_ready()
    # Normal commands after the initial quiet interval must keep flowing.
    guard.note_input(True)
    assert guard.poll()[0]


def test_clock_epoch_change_does_not_misidentify_new_low_stamp_goal(setup):
    guard, node, _, _ = setup
    guard.note_ready()
    guard.on_clock_jump()
    node.now = 10
    guard.note_ready()
    node.now = 12
    guard.on_status('follow_path', status(info(1, 11)))
    assert not guard.blocked


def test_clock_activation_before_ready_does_not_poison_later_idle_action_cancel(setup):
    guard, node, _, _ = setup
    client = node.clients['wait']; client.available = True
    node.now = 10
    guard.on_clock_jump()
    guard.note_ready()
    node.now = 20
    guard.pause(); guard.poll()
    assert not guard.uuid_only
    assert len(client.requests) == 1
    assert client.requests[-1].goal_info.stamp.sec == 20
    client.futures[-1].finish(response())
    assert guard.poll()[0], 'An idle action server need not publish periodic empty statuses'


def test_clock_rewind_cancels_known_future_stamped_old_goal_by_uuid(setup):
    guard, node, _, wall = setup
    client = node.clients['follow_path']; client.available = True
    goal = info(3, 150)
    guard.on_status('follow_path', status(goal))
    guard.pause(); guard.poll()
    client.futures[-1].finish(response())
    wall.now = .6; guard.poll()
    request = client.requests[-1]
    assert bytes(request.goal_info.goal_id.uuid) == bytes(goal.goal_id.uuid)
    assert request.goal_info.stamp.sec == 0
    client.futures[-1].finish(response(goals=[goal]))
    guard.on_status('follow_path', status(goal, GoalStatus.STATUS_CANCELED))
    assert guard.poll()[0]


def test_pending_clock_rewind_rebuilds_known_ids_and_protects_new_goals(setup):
    guard, node, _, wall = setup
    client = node.clients['follow_path']; client.available = True
    old, new = info(1, 90), info(2, 11)
    guard.on_status('follow_path', status(old))
    guard.pause(); guard.poll()
    old_future = client.futures[-1]
    node.now = 10
    guard.on_clock_jump()
    assert old_future.cancelled
    guard.poll()
    current_future = client.futures[-1]
    current_request = client.requests[-1]
    assert bytes(current_request.goal_info.goal_id.uuid) == bytes(old.goal_id.uuid)
    assert current_request.goal_info.stamp.sec == 0
    node.now = 12
    guard.on_status('follow_path', GoalStatusArray(status_list=[
        GoalStatus(goal_info=old, status=GoalStatus.STATUS_CANCELING),
        GoalStatus(goal_info=new, status=GoalStatus.STATUS_EXECUTING)]))
    assert bytes(new.goal_id.uuid) not in guard.pending['follow_path'].goals
    # A late old-epoch ACK must neither add new goals nor release the new barrier.
    old_future.finish(response(goals=[new]))
    assert not guard.poll()[0]
    assert bytes(new.goal_id.uuid) not in guard.pending['follow_path'].goals
    current_future.finish(response(goals=[old]))
    guard.on_status('follow_path', GoalStatusArray(status_list=[
        GoalStatus(goal_info=old, status=GoalStatus.STATUS_CANCELED),
        GoalStatus(goal_info=new, status=GoalStatus.STATUS_EXECUTING)]))
    wall.now = .6
    assert guard.poll()[0]


def test_zero_ros_clock_never_sends_wildcard_cancel_all(setup):
    guard, node, _, wall = setup
    client = node.clients['follow_path']; client.available = True
    old = info(1, 90)
    node.now = 0
    guard.on_status('follow_path', status(old))
    guard.pause(); guard.poll()
    assert len(client.requests) == 1
    assert any(client.requests[-1].goal_info.goal_id.uuid)
    client.futures[-1].finish(response(CancelGoal.Response.ERROR_REJECTED))
    node.now = 10
    wall.now = .6
    guard.on_status('follow_path', status(info(2, 11)))
    guard.poll()
    assert all(bytes(request.goal_info.goal_id.uuid) == bytes(old.goal_id.uuid)
               for request in client.requests)


def test_clock_change_without_known_goal_requires_fresh_idle_status(setup):
    guard, node, _, _ = setup
    client = node.clients['follow_path']; client.available = True
    guard.note_forwarded(True)
    guard.pause(); guard.poll()
    previous_request_count = len(client.requests)
    node.now = 10
    guard.on_clock_jump()
    assert not guard.poll()[0]
    assert len(client.requests) == previous_request_count
    # Availability alone is not proof that an old action stopped.
    client.available = False
    guard.on_status('follow_path', GoalStatusArray())
    assert not guard.poll()[0]
    client.available = True
    guard.on_status('follow_path', GoalStatusArray())
    assert guard.poll()[0]


def test_invalid_zero_goal_uuid_cannot_become_wildcard_request(setup):
    guard, node, _, _ = setup
    client = node.clients['follow_path']; client.available = True
    node.now = 0
    guard.on_status('follow_path', status(info(0, 90)))
    guard.pause()
    assert not guard.poll()[0]
    assert not client.requests
