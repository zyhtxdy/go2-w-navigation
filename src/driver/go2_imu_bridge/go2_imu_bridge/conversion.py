"""Convert LowState-like values without requiring rclpy or Unitree messages."""

import copy
import math

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu


class ConversionError(ValueError):
    """A LowState message does not contain usable gyro/acceleration values."""


def _finite_triplet(values, field):
    try:
        result = tuple(float(values[index]) for index in range(3))
    except (IndexError, KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ConversionError(field + ' must contain three finite numbers') from exc
    if not all(math.isfinite(value) for value in result):
        raise ConversionError(field + ' must contain three finite numbers')
    return result


def _normalized_quaternion(values):
    """Return normalized wxyz, or None; scaling prevents norm overflow."""
    try:
        quaternion = tuple(float(values[index]) for index in range(4))
    except (IndexError, KeyError, TypeError, ValueError, OverflowError):
        return None
    if not all(math.isfinite(value) for value in quaternion):
        return None
    scale = max(abs(value) for value in quaternion)
    if scale == 0.0:
        return None
    scaled = tuple(value / scale for value in quaternion)
    norm = math.sqrt(math.fsum(value * value for value in scaled))
    return tuple(value / norm for value in scaled)


def lowstate_to_imu(lowstate, stamp, frame_id='go2_imu', publish_orientation=False):
    """Map raw SI gyro/acceleration values; stamp is the supplied reception time.

    No tick interpretation, time correction, gravity subtraction, rotation, or
    bias correction is performed. Invalid orientation does not discard otherwise
    valid gyro/acceleration samples. Zero covariances mean unknown covariance.
    """
    if not isinstance(stamp, Time):
        raise ConversionError('stamp must be a builtin_interfaces.msg.Time')
    if not isinstance(frame_id, str) or not frame_id:
        raise ConversionError('frame_id must be a nonempty string')
    try:
        imu_state = lowstate.imu_state
        gyro_values = imu_state.gyroscope
        acceleration_values = imu_state.accelerometer
    except AttributeError as exc:
        raise ConversionError('LowState is missing imu_state gyro/acceleration') from exc
    gyro = _finite_triplet(gyro_values, 'imu_state.gyroscope')
    acceleration = _finite_triplet(acceleration_values, 'imu_state.accelerometer')

    message = Imu()
    message.header.stamp = copy.copy(stamp)
    message.header.frame_id = frame_id
    message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = gyro
    (message.linear_acceleration.x,
     message.linear_acceleration.y,
     message.linear_acceleration.z) = acceleration
    message.orientation.w = 1.0
    message.orientation_covariance[0] = -1.0

    if publish_orientation:
        quaternion = _normalized_quaternion(getattr(imu_state, 'quaternion', None))
        if quaternion is not None:
            (message.orientation.w,
             message.orientation.x,
             message.orientation.y,
             message.orientation.z) = quaternion
            message.orientation_covariance[0] = 0.0
    return message
