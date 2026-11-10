"""Measured yaw inverse model and command safety; no ROS or robot access."""
import json
import math
import pytest
from go2_cmd_vel_bridge.compensation import AngularCompensation, ForwardCompensation
from go2_cmd_vel_bridge.command import CommandGate, MOVE_API_ID, STOP_MOVE_API_ID


def test_disabled_is_exactly_one_to_one():
    model = AngularCompensation()
    for value in [-2, -.45, -.3, -.001, 0, .001, .3, .45, 2]:
        assert model.apply(value) == value


@pytest.mark.parametrize('target,raw',[(.05,.070017),(.1,.114136),(.15,.158255),
    (.2,.202374),(.25,.246493),(.3,.290612),(.35,.334731),(.4,.378850)])
def test_fitted_command_and_explicit_negative_symmetry(target,raw):
    model = AngularCompensation(enabled=True)
    assert model.apply(target) == pytest.approx(raw)
    assert model.apply(-target) == pytest.approx(-raw)


def test_zero_sign_continuity_monotonicity_and_range_exit():
    model = AngularCompensation(enabled=True)
    assert model.apply(0) == 0
    assert model.apply(1e-10) < 1e-9
    for boundary in [0, .05, .4, .45]:
        assert abs(model.apply(boundary+1e-9)-model.apply(boundary-1e-9)) < 1e-7
    for sign in [-1,1]:
        assert model.apply(sign*.6) == sign*.6
    values = [i/10000 for i in range(-10000,10001)]
    outputs = [model.apply(v) for v in values]
    assert all(a <= b for a,b in zip(outputs,outputs[1:]))
    assert all(u*v > 0 for u,v in zip(outputs,values) if v)


def test_two_axis_compensation_applies_once_and_keeps_lateral_velocity():
    gate = CommandGate(compensation=ForwardCompensation(enabled=True),
                       angular_compensation=AngularCompensation(enabled=True))
    values=(.3,.02,0,0,0,-.3)
    gate.receive(values,1)
    assert gate.requested_velocity == (.3,.02,-.3)
    for now in [1,1.05,1.1,1.2]:
        assert json.loads(gate.tick(now)[1]) == pytest.approx({'x':.3926349,'y':.02,'z':-.290612})
    gate.receive(values,1.21)
    assert gate.velocity == pytest.approx((.3926349,.02,-.290612))


@pytest.mark.parametrize('kind',['zero','timeout','disable','estop','invalid','shutdown'])
def test_every_stop_path_still_stops_pure_rotation(kind):
    gate=CommandGate(angular_compensation=AngularCompensation(enabled=True))
    gate.receive((0,0,0,0,0,-.1),1)
    assert gate.tick(1)[0] == MOVE_API_ID
    now=1.1
    if kind=='zero':gate.receive((0,0,0,0,0,0),now)
    elif kind=='timeout':now=1.31
    elif kind=='disable':gate.set_enabled(False)
    elif kind=='estop':gate.set_estop(True)
    elif kind=='invalid':
        with pytest.raises(ValueError):gate.receive((0,0,0,0,0,math.nan),now)
    else:gate.stop()
    for _ in range(3):assert gate.tick(now) == (STOP_MOVE_API_ID,'')
    assert gate.tick(now) is None
    assert gate.velocity is None


@pytest.mark.parametrize('params',[{'enabled':'true'},{'gain':0},{'gain':math.nan},
    {'offset':-.1},{'blend_speed':0},{'max_speed':.01},{'gain':2},{'offset':True}])
def test_bad_model_is_rejected(params):
    with pytest.raises(ValueError):AngularCompensation(**params)


@pytest.mark.parametrize('value',[math.nan,math.inf,-math.inf])
def test_nonfinite_input_is_rejected(value):
    with pytest.raises(ValueError):AngularCompensation(enabled=True).apply(value)
