"""Convert SportModeState-like objects without rclpy or Unitree message classes."""

import copy
import math
from numbers import Integral

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry


UNKNOWN_VARIANCE = 1.0e6


class ConversionError(ValueError):
    """A sample or conversion option is not usable."""


def _finite_array(values, length, field):
    try:
        if isinstance(values, (str, bytes, dict)) or len(values) != length:
            raise ValueError('unexpected array length or type')
        result = tuple(float(values[index]) for index in range(length))
    except (IndexError, KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ConversionError(f'{field} must contain {length} finite numbers') from exc
    if not all(math.isfinite(value) for value in result):
        raise ConversionError(f'{field} must contain {length} finite numbers')
    return result


def _normalized_quaternion(values):
    """Normalize wxyz; scale first to avoid overflow or underflow of the norm."""
    quaternion = _finite_array(values, 4, 'imu_state.quaternion')
    scale = max(abs(value) for value in quaternion)
    if scale == 0.0:
        raise ConversionError('imu_state.quaternion must have a nonzero norm')
    scaled = tuple(value / scale for value in quaternion)
    norm = math.sqrt(math.fsum(value * value for value in scaled))
    return tuple(value / norm for value in scaled)


def _body_velocity(velocity, quaternion):
    """Compute R(q)^T v, assuming q maps body to the velocity's odom frame."""
    w, x, y, z = quaternion
    vx, vy, vz = velocity
    rows = (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + w * z),
         2.0 * (x * z - w * y)),
        (2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z + w * x)),
        (2.0 * (x * z + w * y), 2.0 * (y * z - w * x),
         1.0 - 2.0 * (x * x + y * y)),
    )
    try:
        result = tuple(math.fsum((a * vx, b * vy, c * vz)) for a, b, c in rows)
    except (OverflowError, ValueError) as exc:
        raise ConversionError('rotated linear velocity is not finite') from exc
    if not all(math.isfinite(value) for value in result):
        raise ConversionError('rotated linear velocity is not finite')
    return result


def _stamp_components(stamp, field, require_nonzero):
    try:
        sec, nanosec = stamp.sec, stamp.nanosec
    except AttributeError as exc:
        raise ConversionError(f'{field} is missing sec/nanosec') from exc
    if (isinstance(sec, bool) or isinstance(nanosec, bool)
            or not isinstance(sec, Integral) or not isinstance(nanosec, Integral)
            or not -(2**31) <= sec < 2**31 or not 0 <= nanosec < 1_000_000_000):
        raise ConversionError(f'{field} must contain int32 sec and nanosec in [0, 1e9)')
    if require_nonzero and sec == 0 and nanosec == 0:
        raise ConversionError(f'{field} must be nonzero in message timestamp mode')
    return int(sec), int(nanosec)


def validate_options(frame_id, child_frame_id, velocity_frame,
                     timestamp_source, linear_velocity_stddev):
    """Validate configuration and return the configured linear velocity variance."""
    for field, value in (('frame_id', frame_id), ('child_frame_id', child_frame_id)):
        if not isinstance(value, str) or not value.strip():
            raise ConversionError(f'{field} must be a nonempty string')
    if frame_id == child_frame_id:
        raise ConversionError('frame_id and child_frame_id must differ')
    if velocity_frame not in ('odom', 'body'):
        raise ConversionError('velocity_frame must be odom or body')
    if timestamp_source not in ('receive', 'message'):
        raise ConversionError('timestamp_source must be receive or message')
    try:
        stddev = float(linear_velocity_stddev)
        variance = stddev * stddev
    except (TypeError, ValueError, OverflowError) as exc:
        raise ConversionError('linear_velocity_stddev must be finite and positive') from exc
    if not math.isfinite(stddev) or stddev <= 0.0 or not math.isfinite(variance) or variance == 0.0:
        raise ConversionError('linear_velocity_stddev must have a finite positive square')
    return variance


def sportmodestate_to_odom(state, receive_stamp, frame_id='go2_state_odom',
                          child_frame_id='go2_body', velocity_frame='body',
                          timestamp_source='receive', linear_velocity_stddev=0.2):
    """Publish body velocity, with unmeasured pose/angular fields marked uncertain.

    Body velocity mode, the default for this workspace's recorded Go2W input,
    uses velocity directly. Odom velocity mode assumes the input wxyz quaternion
    maps body coordinates
    to the same odometry reference frame used by state.velocity. This assumption
    must be verified for the selected firmware before enabling odom mode.
    Input quaternion is validated in both modes; it is never output as pose.
    """
    variance = validate_options(
        frame_id, child_frame_id, velocity_frame,
        timestamp_source, linear_velocity_stddev)
    try:
        velocity_values = state.velocity
        quaternion_values = state.imu_state.quaternion
    except AttributeError as exc:
        raise ConversionError('SportModeState is missing velocity or imu_state.quaternion') from exc
    velocity = _finite_array(velocity_values, 3, 'velocity')
    quaternion = _normalized_quaternion(quaternion_values)
    vx, vy, vz = (_body_velocity(velocity, quaternion) if velocity_frame == 'odom'
                  else velocity)

    if timestamp_source == 'receive':
        if not isinstance(receive_stamp, Time):
            raise ConversionError('receive_stamp must be builtin_interfaces.msg.Time')
        _stamp_components(receive_stamp, 'receive_stamp', require_nonzero=False)
        stamp = copy.copy(receive_stamp)
    else:
        # Do not inspect state.stamp at all in the default receive-time mode.
        try:
            message_stamp = state.stamp
        except AttributeError as exc:
            raise ConversionError('SportModeState is missing stamp') from exc
        sec, nanosec = _stamp_components(message_stamp, 'stamp', require_nonzero=True)
        stamp = Time(sec=sec, nanosec=nanosec)

    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.child_frame_id = child_frame_id
    message.pose.pose.orientation.w = 1.0
    pose_covariance = [0.0] * 36
    twist_covariance = [0.0] * 36
    for index in (0, 7, 14, 21, 28, 35):
        pose_covariance[index] = UNKNOWN_VARIANCE
        twist_covariance[index] = UNKNOWN_VARIANCE
    twist_covariance[0] = variance
    twist_covariance[7] = variance
    twist_covariance[14] = variance
    message.pose.covariance = pose_covariance
    message.twist.covariance = twist_covariance
    message.twist.twist.linear.x = vx
    message.twist.twist.linear.y = vy
    message.twist.twist.linear.z = vz
    # Position, orientation and angular velocity are placeholders, not data.
    return message
