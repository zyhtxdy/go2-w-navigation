"""Offline tests only: no rclpy initialization, topics or robot requests."""

import json
import unittest

from go2_cmd_vel_bridge.command import CommandGate, MOVE_API_ID, STOP_MOVE_API_ID


class CommandTests(unittest.TestCase):
    def test_startup_and_idle_are_silent(self):
        gate = CommandGate()
        self.assertIsNone(gate.tick(0))
        gate.receive((0, 0, 0, 0, 0, 0), 1)
        self.assertIsNone(gate.tick(1))
        gate.stop()
        self.assertIsNone(gate.tick(2))

    def test_body_axes_are_forwarded_without_speed_limits(self):
        gate = CommandGate()
        gate.receive((0.9, -0.9, 0, 0, 0, 0.9), 10)
        api, parameter = gate.tick(10.01)
        self.assertEqual(api, MOVE_API_ID)
        self.assertEqual(json.loads(parameter), {'x': 0.9, 'y': -0.9, 'z': 0.9})
        gate.receive((-0.9, 0, 0, 0, 0, -0.9), 10.02)
        self.assertEqual(json.loads(gate.tick(10.03)[1]), {'x': -0.9, 'y': 0.0, 'z': -0.9})

    def test_small_nonzero_velocity_is_not_changed_to_stop(self):
        gate = CommandGate()
        gate.receive((1e-7, 0, 0, 0, 0, 0), 0)
        api, parameter = gate.tick(0.01)
        self.assertEqual(api, MOVE_API_ID)
        self.assertEqual(json.loads(parameter)['x'], 1e-7)

    def test_timeout_stops_then_releases_control(self):
        gate = CommandGate()
        gate.receive((0.1, 0, 0, 0, 0, 0), 0)
        self.assertEqual(gate.tick(0)[0], MOVE_API_ID)
        for now in (0.3, 0.35, 0.4):
            self.assertEqual(gate.tick(now), (STOP_MOVE_API_ID, ''))
        self.assertIsNone(gate.tick(0.45))

    def test_a_command_expired_before_first_send_is_never_sent(self):
        gate = CommandGate()
        gate.receive((0.1, 0, 0, 0, 0, 0), 0)
        self.assertIsNone(gate.tick(1))

    def test_zero_command_stops_without_waiting_for_timeout(self):
        gate = CommandGate()
        gate.receive((0.1, 0, 0, 0, 0, 0), 0)
        gate.tick(0)
        gate.receive((0, 0, 0, 0, 0, 0), 0.01)
        self.assertEqual(gate.tick(0.01)[0], STOP_MOVE_API_ID)

    def test_invalid_input_cancels_previous_velocity(self):
        for invalid in ((float('nan'), 0, 0, 0, 0, 0),
                        (0.1, 0, 0.1, 0, 0, 0), (0.1, 0, 0, 0.1, 0, 0)):
            gate = CommandGate()
            gate.receive((0.1, 0, 0, 0, 0, 0), 0)
            gate.tick(0)
            with self.assertRaises(ValueError):
                gate.receive(invalid, 0.1)
            self.assertEqual(gate.tick(0.1)[0], STOP_MOVE_API_ID)

    def test_disable_requires_fresh_velocity_after_enable(self):
        gate = CommandGate()
        gate.receive((0.1, 0, 0, 0, 0, 0), 0)
        gate.tick(0)
        gate.set_enabled(False)
        gate.receive((0.2, 0, 0, 0, 0, 0), 0.1)
        self.assertEqual(gate.tick(0.1)[0], STOP_MOVE_API_ID)
        gate.set_enabled(True)
        for now in (0.2, 0.3, 0.4):
            result = gate.tick(now)
            self.assertTrue(result is None or result[0] == STOP_MOVE_API_ID)
        gate.receive((0.1, 0, 0, 0, 0, 0), 0.5)
        self.assertEqual(gate.tick(0.5)[0], MOVE_API_ID)

    def test_estop_requires_clear_and_fresh_velocity(self):
        gate = CommandGate()
        gate.receive((0.1, 0, 0, 0, 0, 0), 0)
        gate.tick(0)
        gate.set_estop(True)
        self.assertEqual(gate.tick(0.01)[0], STOP_MOVE_API_ID)
        gate.receive((0.1, 0, 0, 0, 0, 0), 0.02)
        gate.set_estop(False)
        for now in (0.1, 0.2, 0.3):
            result = gate.tick(now)
            self.assertTrue(result is None or result[0] == STOP_MOVE_API_ID)
        gate.receive((0.1, 0, 0, 0, 0, 0), 0.4)
        self.assertEqual(gate.tick(0.4)[0], MOVE_API_ID)

    def test_invalid_configuration(self):
        for timeout in (0, float('inf'), float('nan')):
            with self.assertRaises(ValueError):
                CommandGate(timeout=timeout)
        with self.assertRaises(ValueError):
            CommandGate(stop_repetitions=0)


if __name__ == '__main__':
    unittest.main()
