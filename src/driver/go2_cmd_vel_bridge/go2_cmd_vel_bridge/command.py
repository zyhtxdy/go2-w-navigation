"""Command validation and watchdog without ROS or any network access."""

import json
import math

from go2_cmd_vel_bridge.compensation import ForwardCompensation, AngularCompensation

MOVE_API_ID = 1008
STOP_MOVE_API_ID = 1003


def planar_velocity(values):
    """Validate Twist, then preserve body X/Y and yaw without limiting speed."""
    values = tuple(float(v) for v in values)
    if len(values) != 6 or any(not math.isfinite(v) for v in values):
        raise ValueError('Twist must contain six finite components')
    vx, vy, vz, wx, wy, wz = values
    if any(abs(v) > 1e-6 for v in (vz, wx, wy)):
        raise ValueError('Only linear X/Y and angular Z commands are supported')
    return vx, vy, wz


def move_parameter(velocity):
    return json.dumps(dict(zip(('x', 'y', 'z'), velocity)), allow_nan=False)


class CommandGate:
    """Send only fresh commands. Stop a previously commanded robot in a burst.

    After that burst, remain silent, allowing another operator to take over.
    A never-commanded bridge is silent, including on startup and shutdown.
    """

    def __init__(self, timeout=0.3, stop_repetitions=3, enabled=True, compensation=None,
                 angular_compensation=None):
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('command_timeout must be positive and finite')
        if isinstance(stop_repetitions, bool) or not isinstance(stop_repetitions, int):
            raise ValueError('stop_repetitions must be an integer')
        if stop_repetitions < 1:
            raise ValueError('stop_repetitions must be positive')
        self.timeout = timeout
        self.stop_repetitions = stop_repetitions
        self.enabled = bool(enabled)
        self.estopped = False
        self.compensation = compensation if compensation is not None else ForwardCompensation()
        self.angular_compensation = (angular_compensation if angular_compensation is not None
                                     else AngularCompensation())
        self.requested_velocity = None
        self.velocity = None
        self.received_at = None
        self.active = False
        self.stops_left = 0

    def stop(self):
        self.requested_velocity = None
        self.velocity = None
        self.received_at = None
        if self.active:
            self.stops_left = self.stop_repetitions
        self.active = False

    def set_enabled(self, enabled):
        self.stop()
        self.enabled = bool(enabled)

    def set_estop(self, estopped):
        if self.estopped == bool(estopped):
            return
        self.stop()
        self.estopped = bool(estopped)

    def receive(self, values, now):
        if not self.enabled or self.estopped:
            return
        try:
            requested = planar_velocity(values)
            if all(v == 0.0 for v in requested):
                self.stop()
                return
            vx, vy, wz = requested
            velocity = (self.compensation.apply(vx), vy, self.angular_compensation.apply(wz))
        except (TypeError, ValueError, OverflowError):
            self.stop()
            raise
        self.requested_velocity = requested
        self.velocity = velocity
        self.received_at = now

    def tick(self, now):
        if self.velocity is not None:
            age = now - self.received_at
            if not self.enabled or self.estopped or age < 0.0 or age >= self.timeout:
                self.stop()
            else:
                self.active = True
                self.stops_left = 0
                return MOVE_API_ID, move_parameter(self.velocity)
        if self.stops_left:
            self.stops_left -= 1
            return STOP_MOVE_API_ID, ''
        return None
