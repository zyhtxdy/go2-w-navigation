"""Validate the planner's successful-consumption heartbeat, independently of ROS."""
import json
import math
import time


class PlannerObstacleHealth:
    def __init__(self, timeout=1.0, heartbeat_timeout=1.0):
        if any(isinstance(v, bool) or not math.isfinite(v) or not 0 < v <= 3600
               for v in (timeout, heartbeat_timeout)):
            raise ValueError('Obstacle health timeouts must be positive, finite and bounded')
        self.timeout = timeout
        self.heartbeat_timeout = heartbeat_timeout
        self.reset()

    def reset(self):
        self.received_at = self.progress_at = None
        self.stamp_ns = 0
        self.report_ready = False
        self.reason = 'waiting_for_global_obstacle_map'
        self.invalid_reports = 0

    def observe(self, data, ros_ns, steady=None):
        steady = time.monotonic() if steady is None else steady
        # Parse before committing anything. A corrupt individual heartbeat must
        # neither revoke a still-fresh successful snapshot nor renew its age.
        try:
            value = json.loads(data)
            if not isinstance(value, dict) or type(value.get('ready')) is not bool:
                raise ValueError()
            stamp = value.get('consumed_stamp_ns')
            age = value.get('processed_age_s')
            if type(stamp) is not int or stamp < 0 or type(age) not in (int, float) or not math.isfinite(age):
                raise ValueError()
            reason = value.get('reason')
            if not value['ready'] and (not isinstance(reason, str) or not reason.strip()):
                raise ValueError()
        except (TypeError, ValueError, OverflowError):
            self.invalid_reports += 1
            if self.received_at is None:
                self.reason = 'invalid_global_obstacle_status'
            return

        if value['ready'] and stamp < self.stamp_ns:
            # DDS may deliver an older status after a newer one. Treat that
            # individual sample as out of order, not as a ROS clock rewind;
            # the adapter's clock-jump callback resets this gate explicitly.
            # In particular it must not replace an accepted not-ready report.
            self.invalid_reports += 1
            return

        self.received_at = steady
        self.report_ready = False
        if not value['ready']:
            self.reason = 'global_obstacle_' + reason
            return
        if stamp <= 0 or stamp > ros_ns or ros_ns-stamp > self.timeout*1e9 or not 0 <= age <= self.timeout:
            self.reason = 'global_obstacle_status_stale'
            return
        if stamp > self.stamp_ns:
            self.stamp_ns = stamp
            self.progress_at = steady - age
        # An unchanged consumed stamp cannot renew progress, even if the
        # status is repeatedly republished while ROS time is paused.
        self.report_ready = True
        self.reason = 'ready'

    def readiness(self, ros_ns, steady=None):
        steady = time.monotonic() if steady is None else steady
        if not self.report_ready:
            return False, self.reason
        if self.received_at is None or not 0 <= steady-self.received_at <= self.heartbeat_timeout:
            return False, 'global_obstacle_heartbeat_missing'
        if self.progress_at is None or not 0 <= steady-self.progress_at <= self.timeout:
            return False, 'global_obstacle_progress_stale'
        if not 0 < self.stamp_ns <= ros_ns or ros_ns-self.stamp_ns > self.timeout*1e9:
            return False, 'global_obstacle_source_stale'
        return True, 'ready'
