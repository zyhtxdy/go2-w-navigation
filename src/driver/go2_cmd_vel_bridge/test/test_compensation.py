"""Pure software checks; no ROS initialization or robot requests."""

import json
import math

import pytest

from go2_cmd_vel_bridge.command import CommandGate, MOVE_API_ID, STOP_MOVE_API_ID
from go2_cmd_vel_bridge.compensation import ForwardCompensation


def test_disabled_is_exactly_one_to_one_even_outside_calibration():
    model = ForwardCompensation()
    for speed in (-2.0, -0.1, 0.0, 1e-7, 0.1, 0.7, 2.0):
        assert model.apply(speed) == speed


def test_full_fit_predicts_the_requested_operating_speeds():
    model = ForwardCompensation(enabled=True)
    for desired, expected in ((0.10, 0.1703463), (0.20, 0.2814906),
                              (0.30, 0.3926349), (0.40, 0.5037792)):
        assert model.apply(desired) == pytest.approx(expected)


def test_zero_reverse_lateral_and_rotation_are_not_compensated():
    gate = CommandGate(compensation=ForwardCompensation(enabled=True))
    for values in ((-0.2, 0.1, 0, 0, 0, 0.3), (0, 0.2, 0, 0, 0, -0.3),
                   (0, 0, 0, 0, 0, 0.3)):
        gate.receive(values, 1.0)
        api, parameter = gate.tick(1.01)
        assert api == MOVE_API_ID
        assert json.loads(parameter) == dict(zip(('x', 'y', 'z'),
                                                (values[0], values[1], values[5])))


def test_active_command_reports_target_and_output_without_compounding():
    gate = CommandGate(compensation=ForwardCompensation(enabled=True))
    gate.receive((0.3, 0.02, 0, 0, 0, -0.2), 5.0)
    assert gate.requested_velocity == (0.3, 0.02, -0.2)
    for now in (5.0, 5.05, 5.1, 5.15):
        output = json.loads(gate.tick(now)[1])
        assert output == pytest.approx({'x': 0.3926349, 'y': 0.02, 'z': -0.2})
    # Repeated copies of an input are compensated once, not fed back as input.
    gate.receive((0.3, 0.02, 0, 0, 0, -0.2), 5.2)
    assert json.loads(gate.tick(5.2)[1])['x'] == pytest.approx(0.3926349)


def test_offset_blend_is_continuous_at_zero_and_blend_boundary():
    model = ForwardCompensation(enabled=True)
    assert model.apply(0.0) == 0.0
    assert model.apply(1e-9) < 2e-9
    boundary = model.blend_speed
    expected = model.gain * boundary + model.offset
    for speed in (boundary - 1e-9, boundary, boundary + 1e-9):
        assert model.apply(speed) == pytest.approx(expected, abs=2e-9)
    speed = boundary / 2
    assert model.apply(speed) == pytest.approx(model.gain * speed + model.offset / 2)


def test_calibration_ceiling_bounds_correction_without_reinstating_bridge_limits():
    model = ForwardCompensation(enabled=True)
    assert model.apply(0.59) == 0.7
    assert model.apply(0.7) == 0.7
    assert model.apply(0.9) == 0.9
    assert model.apply(-0.9) == -0.9


def test_no_downward_jump_at_saturation_or_edge_of_calibration():
    model = ForwardCompensation(enabled=True)
    values = [i / 10000 for i in range(10001)]
    outputs = [model.apply(v) for v in values]
    assert all(a <= b for a, b in zip(outputs, outputs[1:]))
    assert all(v <= u for v, u in zip(values, outputs))
    for boundary in ((model.max_command - model.offset) / model.gain,
                     model.max_command):
        assert abs(model.apply(boundary + 1e-8) - model.apply(boundary - 1e-8)) < 3e-8


@pytest.mark.parametrize('stop_kind', ['zero', 'timeout', 'disable', 'estop', 'invalid', 'shutdown'])
def test_all_stop_paths_bypass_compensation_and_release_control(stop_kind):
    gate = CommandGate(compensation=ForwardCompensation(enabled=True))
    gate.receive((0.2, 0, 0, 0, 0, 0), 0.0)
    assert gate.tick(0.0)[0] == MOVE_API_ID
    now = 0.1
    if stop_kind == 'zero':
        gate.receive((0, 0, 0, 0, 0, 0), now)
    elif stop_kind == 'timeout':
        now = 0.3
    elif stop_kind == 'disable':
        gate.set_enabled(False)
    elif stop_kind == 'estop':
        gate.set_estop(True)
    elif stop_kind == 'invalid':
        with pytest.raises(ValueError):
            gate.receive((math.nan, 0, 0, 0, 0, 0), now)
    else:
        gate.stop()
    for index in range(3):
        assert gate.tick(now + index * 0.05) == (STOP_MOVE_API_ID, '')
    assert gate.tick(now + 0.2) is None
    assert gate.requested_velocity is None
    assert gate.velocity is None


def test_enabled_compensation_cannot_create_startup_or_expired_motion():
    gate = CommandGate(compensation=ForwardCompensation(enabled=True))
    gate.receive((0, 0, 0, 0, 0, 0), 0.0)
    assert gate.tick(0.0) is None
    gate.receive((0.2, 0, 0, 0, 0, 0), 1.0)
    assert gate.tick(1.3) is None


@pytest.mark.parametrize('configuration', [
    {'enabled': 'true'}, {'gain': 0.9}, {'gain': math.inf}, {'offset': -0.1},
    {'offset': math.nan}, {'blend_speed': 0.0}, {'blend_speed': -0.1},
    {'max_command': 0.0}, {'max_command': math.inf}, {'gain': True},
])
def test_invalid_calibration_is_rejected_at_construction(configuration):
    with pytest.raises(ValueError):
        ForwardCompensation(**configuration)


@pytest.mark.parametrize('speed', [math.nan, math.inf, -math.inf])
def test_nonfinite_speed_is_rejected(speed):
    with pytest.raises(ValueError):
        ForwardCompensation(enabled=True).apply(speed)
