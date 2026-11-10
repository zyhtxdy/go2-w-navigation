import math
from types import SimpleNamespace
import unittest

from builtin_interfaces.msg import Time

from go2_imu_bridge.conversion import ConversionError, lowstate_to_imu


def lowstate(gyro=(0.1, -0.2, 0.3), acceleration=(1.0, -2.0, 9.81),
             quaternion=(1.0, 0.0, 0.0, 0.0), tick=None):
    return SimpleNamespace(
        tick=tick,
        imu_state=SimpleNamespace(
            gyroscope=gyro, accelerometer=acceleration, quaternion=quaternion))


class ConversionTests(unittest.TestCase):
    def setUp(self):
        self.stamp = Time(sec=123, nanosec=456789)

    def test_preserves_raw_values_stamp_and_frame_without_gravity_removal(self):
        message = lowstate_to_imu(lowstate(), self.stamp, 'body_sensor')
        self.assertEqual(message.header.stamp, self.stamp)
        self.assertEqual(message.header.frame_id, 'body_sensor')
        self.assertEqual(message.angular_velocity.x, 0.1)
        self.assertEqual(message.angular_velocity.y, -0.2)
        self.assertEqual(message.angular_velocity.z, 0.3)
        self.assertEqual(message.linear_acceleration.x, 1.0)
        self.assertEqual(message.linear_acceleration.y, -2.0)
        self.assertEqual(message.linear_acceleration.z, 9.81)
        self.stamp.sec = 999
        self.assertEqual(message.header.stamp.sec, 123)

    def test_default_orientation_unavailable_and_covariances_unknown(self):
        message = lowstate_to_imu(lowstate(quaternion=(0.0, 0.0, 0.0, 2.0)), self.stamp)
        self.assertEqual(message.orientation.w, 1.0)
        self.assertEqual(message.orientation.x, 0.0)
        self.assertEqual(message.orientation.y, 0.0)
        self.assertEqual(message.orientation.z, 0.0)
        self.assertEqual(list(message.orientation_covariance), [-1.0] + [0.0] * 8)
        self.assertEqual(list(message.angular_velocity_covariance), [0.0] * 9)
        self.assertEqual(list(message.linear_acceleration_covariance), [0.0] * 9)

    def test_optional_orientation_normalizes_and_maps_wxyz(self):
        message = lowstate_to_imu(
            lowstate(quaternion=(2.0, 0.0, 0.0, 2.0)), self.stamp,
            publish_orientation=True)
        self.assertAlmostEqual(message.orientation.w, math.sqrt(0.5))
        self.assertEqual(message.orientation.x, 0.0)
        self.assertEqual(message.orientation.y, 0.0)
        self.assertAlmostEqual(message.orientation.z, math.sqrt(0.5))
        self.assertEqual(list(message.orientation_covariance), [0.0] * 9)

    def test_huge_finite_quaternion_does_not_overflow(self):
        message = lowstate_to_imu(
            lowstate(quaternion=(1.7e308, -1.7e308, 1.7e308, -1.7e308)),
            self.stamp, publish_orientation=True)
        self.assertEqual(message.orientation.w, 0.5)
        self.assertEqual(message.orientation.x, -0.5)
        self.assertEqual(message.orientation.y, 0.5)
        self.assertEqual(message.orientation.z, -0.5)
        self.assertEqual(message.orientation_covariance[0], 0.0)

    def test_invalid_orientation_retains_gyro_and_acceleration(self):
        invalid_quaternions = [
            None, [], [1.0], [0.0] * 4, [math.nan, 0.0, 0.0, 1.0],
            [1.0, math.inf, 0.0, 0.0], ['bad', 0.0, 0.0, 1.0],
        ]
        for quaternion in invalid_quaternions:
            with self.subTest(quaternion=quaternion):
                message = lowstate_to_imu(
                    lowstate(quaternion=quaternion), self.stamp, publish_orientation=True)
                self.assertEqual(message.angular_velocity.z, 0.3)
                self.assertEqual(message.linear_acceleration.z, 9.81)
                self.assertEqual(message.orientation.w, 1.0)
                self.assertEqual(message.orientation_covariance[0], -1.0)

    def test_rejects_nonfinite_gyro_or_acceleration(self):
        for invalid in (math.nan, math.inf, -math.inf):
            for field in ('gyro', 'acceleration'):
                with self.subTest(field=field, value=invalid):
                    with self.assertRaises(ConversionError):
                        lowstate_to_imu(lowstate(**{field: [0.0, invalid, 0.0]}), self.stamp)

    def test_rejects_missing_short_or_nonnumeric_raw_arrays(self):
        for invalid in (None, [], [0.0], [0.0, 0.0], ['bad', 0.0, 0.0]):
            for field in ('gyro', 'acceleration'):
                with self.subTest(field=field, value=invalid):
                    with self.assertRaises(ConversionError):
                        lowstate_to_imu(lowstate(**{field: invalid}), self.stamp)
        with self.assertRaises(ConversionError):
            lowstate_to_imu(SimpleNamespace(), self.stamp)

    def test_unrecognized_tick_does_not_change_or_drop_sample(self):
        for tick in (None, -1, 0, 2**64, 'unknown'):
            with self.subTest(tick=tick):
                message = lowstate_to_imu(lowstate(tick=tick), self.stamp)
                self.assertEqual(message.header.stamp, self.stamp)
                self.assertEqual(message.angular_velocity.z, 0.3)

    def test_rejects_invalid_stamp_or_empty_frame(self):
        with self.assertRaises(ConversionError):
            lowstate_to_imu(lowstate(), None)
        with self.assertRaises(ConversionError):
            lowstate_to_imu(lowstate(), self.stamp, '')


if __name__ == '__main__':
    unittest.main()
