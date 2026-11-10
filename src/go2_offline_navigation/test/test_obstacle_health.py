"""Pure successful-consumption contract: no DDS, sensor or control publishers."""
import json
import pytest
from go2_offline_navigation.obstacle_health import PlannerObstacleHealth


def report(stamp=10000000000, **changes):
    return json.dumps(dict(dict(ready=True, consumed_stamp_ns=stamp, processed_age_s=0.), **changes))


def test_startup_requires_successful_planner_consumption():
    gate = PlannerObstacleHealth()
    assert not gate.readiness(10000000000, 20)[0]
    gate.observe(report(), 10000000000, 20)
    assert gate.readiness(10000000000, 20)[0]


def test_receive_heartbeat_does_not_renew_old_consumed_stamp_under_paused_ros_clock():
    gate = PlannerObstacleHealth()
    gate.observe(report(), 10000000000, 20)
    for now in (20.2, 20.9, 21.01, 22):
        gate.observe(report(), 10000000000, now)
    assert gate.readiness(10000000000, 22) == (False, 'global_obstacle_progress_stale')
    gate.observe(report(10100000000), 10100000000, 22.1)
    assert gate.readiness(10100000000, 22.1)[0]


def test_reported_processing_age_is_not_discarded_on_new_stamp():
    gate = PlannerObstacleHealth()
    gate.observe(report(processed_age_s=.8), 10000000000, 20)
    assert not gate.readiness(10000000000, 20.21)[0]


def test_actual_source_age_and_status_silence_both_expire():
    gate = PlannerObstacleHealth()
    gate.observe(report(), 10000000000, 20)
    assert not gate.readiness(11000000001, 20.1)[0]
    assert not gate.readiness(10000000000, 21.01)[0]


MALFORMED_REPORTS = ['null', '{}', '[]', 'bad-json',
    report(ready=1), report(stamp=True), report(stamp=-1),
    report(processed_age_s=float('nan')), report(processed_age_s=True),
    report(processed_age_s='0.0'),
    json.dumps(dict(ready=True, processed_age_s=0.)),
    json.dumps(dict(ready=True, consumed_stamp_ns=10000000000)),
    report(ready=False), report(ready=False, reason={}),
    report(ready=False, reason='')]


@pytest.mark.parametrize('bad', MALFORMED_REPORTS)
def test_malformed_status_alone_cannot_authorize_motion(bad):
    gate = PlannerObstacleHealth()
    gate.observe(bad, 10000000000, 20)
    assert gate.readiness(10000000000, 20) == (False, 'invalid_global_obstacle_status')
    assert gate.received_at is None
    assert gate.progress_at is None
    assert gate.invalid_reports == 1


@pytest.mark.parametrize('bad', MALFORMED_REPORTS)
def test_malformed_status_preserves_last_good_without_renewing_its_deadlines(bad):
    gate = PlannerObstacleHealth()
    gate.observe(report(), 10000000000, 20)
    gate.observe(bad, 10000000000, 20.1)
    assert gate.readiness(10000000000, 20.1) == (True, 'ready')
    assert gate.received_at == 20
    assert gate.progress_at == 20
    assert gate.stamp_ns == 10000000000
    gate.observe(bad, 10000000000, 20.9)
    gate.observe(bad, 10000000000, 21.01)
    assert gate.readiness(10000000000, 21.01) == (False, 'global_obstacle_heartbeat_missing')
    assert gate.received_at == 20
    assert gate.progress_at == 20


@pytest.mark.parametrize('bad', [report(stamp=10000000001),
    report(processed_age_s=-.1), report(processed_age_s=1.01)])
def test_valid_report_of_unusable_snapshot_revokes_readiness(bad):
    gate = PlannerObstacleHealth()
    gate.observe(report(), 10000000000, 20)
    gate.observe(bad, 10000000000, 20.1)
    assert not gate.readiness(10000000000, 20.1)[0]


def test_malformed_status_cannot_hide_an_explicit_failure():
    gate = PlannerObstacleHealth()
    gate.observe(report(), 10000000000, 20)
    gate.observe(report(ready=False, reason='localization_unavailable'), 10000000000, 20.1)
    gate.observe('bad-json', 10000000000, 20.2)
    assert gate.readiness(10000000000, 20.2) == (False, 'global_obstacle_localization_unavailable')
    assert gate.received_at == 20.1


@pytest.mark.parametrize('stamp', [9900000000, 1])
def test_out_of_order_ready_status_preserves_fresh_last_good_without_renewal(stamp):
    gate = PlannerObstacleHealth()
    gate.observe(report(), 10000000000, 20)
    gate.observe(report(stamp), 10000000000, 20.2)
    assert gate.readiness(10000000000, 20.2) == (True, 'ready')
    assert gate.received_at == 20
    assert gate.progress_at == 20
    assert gate.stamp_ns == 10000000000
    assert gate.invalid_reports == 1


def test_old_ready_status_without_last_good_does_not_authorize_motion():
    gate = PlannerObstacleHealth()
    gate.observe(report(1), 10000000000, 20)
    assert gate.readiness(10000000000, 20) == (False, 'global_obstacle_status_stale')


def test_out_of_order_ready_status_cannot_hide_an_explicit_failure():
    gate = PlannerObstacleHealth()
    gate.observe(report(), 10000000000, 20)
    gate.observe(report(ready=False, reason='localization_unavailable'), 10000000000, 20.1)
    gate.observe(report(9900000000), 10000000000, 20.2)
    assert gate.readiness(10000000000, 20.2) == (False, 'global_obstacle_localization_unavailable')
    assert gate.received_at == 20.1
    assert gate.progress_at == 20
    assert gate.stamp_ns == 10000000000


def test_repeated_out_of_order_ready_status_does_not_extend_original_deadline():
    gate = PlannerObstacleHealth(timeout=2., heartbeat_timeout=1.)
    gate.observe(report(), 10000000000, 20)
    for now in (20.1, 20.9, 21.01, 22.1):
        gate.observe(report(9900000000), 10000000000, now)
    assert gate.readiness(10000000000, 22.1) == (False, 'global_obstacle_heartbeat_missing')
    assert gate.received_at == 20
    assert gate.progress_at == 20
    assert gate.stamp_ns == 10000000000


def test_explicit_startup_failure_with_no_processed_snapshot_is_valid():
    gate = PlannerObstacleHealth()
    gate.observe(report(stamp=0, processed_age_s=-1., ready=False,
                        reason='waiting_for_map_or_fresh_input'), 10000000000, 20)
    assert gate.readiness(10000000000, 20) == (False, 'global_obstacle_waiting_for_map_or_fresh_input')
    assert gate.received_at == 20
    assert gate.invalid_reports == 0


def test_explicit_failure_and_clock_rewind_require_recovery():
    gate = PlannerObstacleHealth()
    gate.observe(report(), 10000000000, 20)
    gate.observe(report(ready=False, reason='live_obstacle_processing_stale'), 10000000000, 20.1)
    assert not gate.readiness(10000000000, 20.1)[0]
    gate.observe(report(9000000000), 10000000000, 20.2)
    assert not gate.readiness(10000000000, 20.2)[0]
    gate.reset()
    assert not gate.readiness(5000000000, 20.3)[0]
    gate.observe(report(5000000000), 5000000000, 20.4)
    assert gate.readiness(5000000000, 20.4)[0]


def test_relaxed_global_data_age_still_requires_a_short_status_heartbeat():
    gate = PlannerObstacleHealth(timeout=2., heartbeat_timeout=1.)
    gate.observe(report(processed_age_s=1.5), 11500000000, 20)
    assert gate.readiness(11500000000, 20)[0]
    assert gate.readiness(12000000000, 20.5)[0]
    assert not gate.readiness(12000000001, 20.5)[0]
    gate.observe(report(11000000000), 11000000000, 21)
    assert gate.readiness(12500000000, 21.9)[0]
    assert gate.readiness(12500000000, 22.01) == (False, 'global_obstacle_heartbeat_missing')


def test_live_status_cannot_refresh_a_frozen_two_second_global_snapshot():
    gate = PlannerObstacleHealth(timeout=2.)
    gate.observe(report(), 10000000000, 20)
    for now in [20.8, 21.6, 22.01]:
        gate.observe(report(), 10000000000, now)
    assert gate.readiness(10000000000, 22.01) == (False, 'global_obstacle_progress_stale')


@pytest.mark.parametrize('bad', [0., -1., float('nan'), float('inf'), 3601., True])
def test_invalid_freshness_and_heartbeat_limits_are_rejected(bad):
    with pytest.raises(ValueError):
        PlannerObstacleHealth(timeout=bad)
    with pytest.raises(ValueError):
        PlannerObstacleHealth(heartbeat_timeout=bad)
