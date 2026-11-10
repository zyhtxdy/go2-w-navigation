"""Empirical velocity feedforward; no ROS, feedback or robot access."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ForwardCompensation:
    """Convert desired forward speed into a Go2 Move parameter.

    The fit is based on the user's command/EKF observations, not independent
    ground truth. Reverse, lateral and yaw commands have no calibration here.
    The extra command is bounded by the calibrated raw-command range; this is
    not a general bridge speed limiter. Inputs already above that range pass
    through unchanged, as they do when compensation is disabled.
    """

    enabled: bool = False
    gain: float = 1.111443
    offset: float = 0.059202
    blend_speed: float = 0.05
    max_command: float = 0.70

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError('forward_compensation_enabled must be a bool')
        values = (self.gain, self.offset, self.blend_speed, self.max_command)
        if any(isinstance(v, bool) or not math.isfinite(v) for v in values):
            raise ValueError('Forward compensation coefficients must be finite numbers')
        if self.gain < 1.0 or self.offset < 0.0:
            raise ValueError('Forward compensation requires gain >= 1 and offset >= 0')
        if self.blend_speed <= 0.0 or self.max_command <= 0.0:
            raise ValueError('Forward compensation blend_speed and max_command must be positive')

    def apply(self, velocity):
        if not math.isfinite(velocity):
            raise ValueError('Forward velocity must be finite')
        if not self.enabled or velocity <= 0.0 or velocity >= self.max_command:
            return velocity
        fraction = min(velocity / self.blend_speed, 1.0)
        # Smoothstep blends only the offset: zero has no fixed kick, and the
        # full affine fit is reached continuously at blend_speed.
        weight = fraction * fraction * (3.0 - 2.0 * fraction)
        command = self.gain * velocity + self.offset * weight
        # Saturate the added correction, never the original user command.
        # This avoids either extrapolating the fit or a downward jump at the
        # edge of its range. With the configured fit the curve is monotone.
        return max(velocity, min(command, self.max_command))


@dataclass(frozen=True)
class AngularCompensation:
    """Symmetric yaw inverse model fitted from positive Request/EKF observations.

    Negative yaw uses the same magnitude fit by explicit user choice; it has
    no independent measurements. Magnitudes may be increased OR decreased. Zero and the
    upper calibration edge blend continuously; there is no total speed cap.
    """

    enabled: bool = False
    gain: float = 0.882380
    offset: float = 0.025898
    blend_speed: float = 0.05
    max_speed: float = 0.40

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError('angular_compensation_enabled must be a bool')
        values = (self.gain, self.offset, self.blend_speed, self.max_speed)
        if any(isinstance(v, bool) or not math.isfinite(v) for v in values):
            raise ValueError('Angular compensation coefficients must be finite numbers')
        if self.gain <= 0 or self.offset < 0 or self.blend_speed <= 0:
            raise ValueError('Angular gain/blend must be positive and offset nonnegative')
        if self.max_speed <= self.blend_speed:
            raise ValueError('Angular max_speed must exceed blend_speed')
        edge_correction = (self.gain - 1.0) * self.max_speed + self.offset
        # The largest magnitude derivative of smoothstep is 1.5. Prevent a
        # positive correction from making the return to 1:1 non-monotone.
        if edge_correction * 1.5 >= self.blend_speed:
            raise ValueError('Angular blend_speed is too small for a monotone upper transition')

    def apply(self, velocity):
        if not math.isfinite(velocity):
            raise ValueError('Angular velocity must be finite')
        if not self.enabled or velocity == 0.0:
            return velocity
        return math.copysign(self._magnitude(abs(velocity)), velocity)

    def _magnitude(self, velocity):
        if velocity <= self.max_speed:
            fraction = min(velocity / self.blend_speed, 1.0)
            weight = fraction * fraction * (3.0 - 2.0 * fraction)
            return self.gain * velocity + self.offset * weight
        if velocity >= self.max_speed + self.blend_speed:
            return velocity
        # Retire the boundary correction outside the fitted target range,
        # without extrapolating its slope or adding a total speed limit.
        fraction = (velocity - self.max_speed) / self.blend_speed
        weight = fraction * fraction * (3.0 - 2.0 * fraction)
        edge_correction = (self.gain - 1.0) * self.max_speed + self.offset
        return velocity + edge_correction * (1.0 - weight)
