import math
from types import SimpleNamespace
import unittest

from builtin_interfaces.msg import Time

from go2_odom_bridge.conversion import (
    ConversionError, UNKNOWN_VARIANCE, sportmodestate_to_odom,
)


def state(velocity=(1.0, 2.0, 3.0), quaternion=(1.0, 0.0, 0.0, 0.0), stamp=None):
    return SimpleNamespace(
        velocity=velocity, imu_state=SimpleNamespace(quaternion=quaternion),
        stamp=stamp, position=(math.nan, math.nan, math.nan), yaw_speed=math.nan)


class ConversionTests(unittest.TestCase):
    def setUp(self):
        self.receive_stamp = Time(sec=123, nanosec=456789)

    def convert(self, sample, **options):
        return sportmodestate_to_odom(sample, self.receive_stamp, **options)

    def test_identity_keeps_xy_and_uses_distinct_default_frames(self):
        message = self.convert(state())
        self.assertEqual(message.header.frame_id, 'go2_state_odom')
        self.assertEqual(message.child_frame_id, 'go2_body')
        self.assertEqual(message.twist.twist.linear.x, 1.0)
        self.assertEqual(message.twist.twist.linear.y, 2.0)
        self.assertEqual(message.twist.twist.linear.z, 3.0)

    def test_yaw_90_uses_inverse_rotation(self):
        root_half = math.sqrt(0.5)
        message = self.convert(state(
            velocity=(1.0, 0.0, 0.0), quaternion=(root_half, 0.0, 0.0, root_half)),
            velocity_frame='odom')
        self.assertAlmostEqual(message.twist.twist.linear.x, 0.0)
        self.assertAlmostEqual(message.twist.twist.linear.y, -1.0)
        self.assertAlmostEqual(message.twist.twist.linear.z, 0.0)

    def test_tilt_uses_vertical_input_when_computing_body_xy(self):
        root_half = math.sqrt(0.5)
        for quaternion, expected in (
                ((root_half, 0.0, root_half, 0.0), (-1.0, 0.0)),
                ((root_half, root_half, 0.0, 0.0), (0.0, 1.0))):
            with self.subTest(quaternion=quaternion):
                message = self.convert(
                    state(velocity=(0.0, 0.0, 1.0), quaternion=quaternion),
                    velocity_frame='odom')
                self.assertAlmostEqual(message.twist.twist.linear.x, expected[0])
                self.assertAlmostEqual(message.twist.twist.linear.y, expected[1])
                self.assertAlmostEqual(message.twist.twist.linear.z, 0.0)

    def test_preserves_rotated_z_for_downstream_frame_transform(self):
        root_half = math.sqrt(0.5)
        message = self.convert(state(
            velocity=(1.0, 0.0, 0.0), quaternion=(root_half, 0.0, root_half, 0.0)),
            velocity_frame='odom')
        self.assertAlmostEqual(message.twist.twist.linear.x, 0.0)
        self.assertAlmostEqual(message.twist.twist.linear.y, 0.0)
        self.assertAlmostEqual(message.twist.twist.linear.z, 1.0)
        self.assertAlmostEqual(message.twist.covariance[14], 0.04)

    def test_quaternion_sign_does_not_change_velocity(self):
        quaternion = (0.8, 0.1, -0.3, 0.4)
        positive = self.convert(state(quaternion=quaternion), velocity_frame='odom')
        negative = self.convert(
            state(quaternion=tuple(-v for v in quaternion)), velocity_frame='odom')
        self.assertEqual(positive.twist.twist.linear, negative.twist.twist.linear)

    def test_quaternion_normalization_handles_huge_and_tiny_values(self):
        reference = self.convert(
            state(quaternion=(1.0, -1.0, 1.0, -1.0)), velocity_frame='odom')
        for scale in (1.7e308, 1.0e-300):
            with self.subTest(scale=scale):
                message = self.convert(
                    state(quaternion=(scale, -scale, scale, -scale)),
                    velocity_frame='odom')
                self.assertEqual(message.twist.twist.linear, reference.twist.twist.linear)

    def test_explicit_body_mode_does_not_rotate(self):
        message = self.convert(state(quaternion=(1.0, 0.0, 0.0, 1.0)), velocity_frame='body')
        self.assertEqual(message.twist.twist.linear.x, 1.0)
        self.assertEqual(message.twist.twist.linear.y, 2.0)
        self.assertEqual(message.twist.twist.linear.z, 3.0)

    def test_default_body_preserves_forward_and_reverse_near_pi_yaw(self):
        for yaw in (math.pi, math.pi - 0.01, -math.pi + 0.01):
            quaternion = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
            for speed in (0.6, -0.6):
                with self.subTest(yaw=yaw, speed=speed):
                    message = self.convert(state(
                        velocity=(speed, 0.04, -0.03), quaternion=quaternion))
                    self.assertEqual(message.twist.twist.linear.x, speed)
                    self.assertEqual(message.twist.twist.linear.y, 0.04)
                    self.assertEqual(message.twist.twist.linear.z, -0.03)

    def test_finite_extreme_velocity_with_rotation_overflow_is_rejected(self):
        quaternion = (math.cos(math.pi / 8), 0.0, 0.0, -math.sin(math.pi / 8))
        with self.assertRaises(ConversionError):
            self.convert(
                state(velocity=(1.7e308, 1.7e308, 0.0), quaternion=quaternion),
                velocity_frame='odom')

    def test_pose_yaw_speed_and_angular_velocity_are_not_measurements(self):
        message = self.convert(state())
        self.assertEqual(message.pose.pose.position.x, 0.0)
        self.assertEqual(message.pose.pose.position.y, 0.0)
        self.assertEqual(message.pose.pose.position.z, 0.0)
        self.assertEqual(message.pose.pose.orientation.w, 1.0)
        self.assertEqual(message.twist.twist.linear.z, 3.0)
        self.assertEqual(message.twist.twist.angular.x, 0.0)
        self.assertEqual(message.twist.twist.angular.y, 0.0)
        self.assertEqual(message.twist.twist.angular.z, 0.0)
        for index in (0, 7, 14, 21, 28, 35):
            self.assertEqual(message.pose.covariance[index], UNKNOWN_VARIANCE)
        for index in (21, 28, 35):
            self.assertEqual(message.twist.covariance[index], UNKNOWN_VARIANCE)
        self.assertAlmostEqual(message.twist.covariance[0], 0.04)
        self.assertAlmostEqual(message.twist.covariance[7], 0.04)
        self.assertAlmostEqual(message.twist.covariance[14], 0.04)

    def test_variance_is_square_of_configured_stddev(self):
        message = self.convert(state(), linear_velocity_stddev=0.3)
        self.assertAlmostEqual(message.twist.covariance[0], 0.09)
        self.assertAlmostEqual(message.twist.covariance[7], 0.09)
        self.assertAlmostEqual(message.twist.covariance[14], 0.09)

    def test_invalid_velocity_arrays_are_rejected(self):
        for velocity in (None, [], [1.0], [1.0] * 4, '123', {0: 1, 1: 2, 2: 3},
                         [math.nan, 0.0, 0.0], [0.0, math.inf, 0.0], ['bad', 0.0, 0.0]):
            with self.subTest(velocity=velocity), self.assertRaises(ConversionError):
                self.convert(state(velocity=velocity))

    def test_invalid_quaternions_are_rejected_in_both_modes(self):
        for quaternion in (None, [], [1.0] * 3, [1.0] * 5, [0.0] * 4,
                           [math.nan, 0.0, 0.0, 1.0], [1.0, math.inf, 0.0, 0.0]):
            for mode in ('odom', 'body'):
                with self.subTest(quaternion=quaternion, mode=mode), self.assertRaises(ConversionError):
                    self.convert(state(quaternion=quaternion), velocity_frame=mode)

    def test_missing_fields_are_rejected(self):
        for sample in (None, SimpleNamespace(velocity=(1, 2, 3)),
                       SimpleNamespace(velocity=(1, 2, 3), imu_state=SimpleNamespace())):
            with self.subTest(sample=sample), self.assertRaises(ConversionError):
                self.convert(sample)

    def test_receive_mode_ignores_missing_or_invalid_message_stamp(self):
        sample = state(stamp=SimpleNamespace(sec='bad', nanosec=math.nan))
        message = self.convert(sample)
        self.assertEqual(message.header.stamp, self.receive_stamp)
        del sample.stamp
        self.assertEqual(self.convert(sample).header.stamp, self.receive_stamp)
        self.receive_stamp.sec = 999
        self.assertEqual(message.header.stamp.sec, 123)

    def test_message_mode_uses_message_time_without_offset(self):
        message = self.convert(state(stamp=SimpleNamespace(sec=77, nanosec=123)),
                               timestamp_source='message')
        self.assertEqual(message.header.stamp.sec, 77)
        self.assertEqual(message.header.stamp.nanosec, 123)

    def test_message_mode_preserves_valid_negative_seconds(self):
        message = self.convert(state(stamp=SimpleNamespace(sec=-1, nanosec=999999999)),
                               timestamp_source='message')
        self.assertEqual(message.header.stamp.sec, -1)
        self.assertEqual(message.header.stamp.nanosec, 999999999)

    def test_invalid_message_times_reject_without_receive_fallback(self):
        for stamp in (None, SimpleNamespace(sec=0, nanosec=0),
                      SimpleNamespace(sec=2**31, nanosec=0),
                      SimpleNamespace(sec=-(2**31) - 1, nanosec=0),
                      SimpleNamespace(sec=1, nanosec=1_000_000_000),
                      SimpleNamespace(sec=1, nanosec=-1),
                      SimpleNamespace(sec=1.5, nanosec=0),
                      SimpleNamespace(sec=True, nanosec=0)):
            with self.subTest(stamp=stamp), self.assertRaises(ConversionError):
                self.convert(state(stamp=stamp), timestamp_source='message')
        sample = state()
        del sample.stamp
        with self.assertRaises(ConversionError):
            self.convert(sample, timestamp_source='message')

    def test_receive_time_requires_ros_time_and_valid_nanoseconds(self):
        with self.assertRaises(ConversionError):
            sportmodestate_to_odom(state(), SimpleNamespace(sec=1, nanosec=0))
        with self.assertRaises(ConversionError):
            sportmodestate_to_odom(state(), Time(sec=1, nanosec=1_000_000_000))

    def test_zero_receive_time_is_preserved_for_simulation_clock(self):
        message = sportmodestate_to_odom(state(), Time())
        self.assertEqual(message.header.stamp, Time())

    def test_invalid_options_are_rejected(self):
        options_list = [
            {'frame_id': ''}, {'child_frame_id': ' '},
            {'frame_id': 'same', 'child_frame_id': 'same'},
            {'velocity_frame': 'unknown'}, {'timestamp_source': 'tick'},
        ]
        options_list.extend({'linear_velocity_stddev': value} for value in
                            (0.0, -1.0, math.nan, math.inf, 1.0e308, 1.0e-300, None))
        for options in options_list:
            with self.subTest(options=options), self.assertRaises(ConversionError):
                self.convert(state(), **options)


if __name__ == '__main__':
    unittest.main()
