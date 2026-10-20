"""Bench test mode: what it actually commands, read back from the ULog.

The unit tests (src/modules/bench_test/DtrgBenchProfileTest.cpp) cover the
profile maths. These check the module wiring: that the setpoints are only
published while armed in bench test, follow the direction switch, spin up
softly and excite only the selected axis.
"""

import numpy as np
import pytest

from rc_layout import CH_CMD_SIGN, CH_FLTMODE, PWM_CENTRE, PWM_MAX, PWM_MIN, SLOT_BENCH_TEST, slot_pwm
from ulog_checks import bench_test_armed, bench_test_disarmed, read_log, runs
from vehicle import MAIN_BENCH_TEST

pytestmark = pytest.mark.sih

HOVER = 0.2  # far below hover thrust, so the simulated vehicle stays on the ground
SPINUP_T = 1.0
STEP_DELAY = 1.0
STEP_MAG = 0.1
STEP_DUR = 1.0
TOL = 0.01

# vehicle_thrust_setpoint is logged every 20 ms
TIME_TOL = 0.1

PROFILE_PARAMS = {
    "BT_ARM_ENABLE": 1,
    "BT_AXIS": 2,  # thrust Z
    "BT_HOVER_THR": HOVER,
    "BT_SPINUP_T": SPINUP_T,
    "BT_STEP_DELAY": STEP_DELAY,
    "BT_STEP_MAG": STEP_MAG,
    "BT_STEP_DUR": STEP_DUR,
}

BENCH_SLOT = {CH_FLTMODE: slot_pwm(SLOT_BENCH_TEST)}


def run_profile(sitl, params, switch_pwm, excite_s):
    """Arm in bench test, spin up, hold the direction switch at ``switch_pwm`` for ``excite_s``."""
    vehicle, px4 = sitl(params={**PROFILE_PARAMS, **params}, rc=BENCH_SLOT)
    vehicle.wait_mode(MAIN_BENCH_TEST)
    vehicle.hold(1.0)  # some disarmed samples

    assert vehicle.arm().accepted
    vehicle.wait_armed()
    vehicle.hold(SPINUP_T + 1.0)

    vehicle.set_rc(CH_CMD_SIGN, switch_pwm)
    vehicle.hold(excite_s)
    vehicle.set_rc(CH_CMD_SIGN, PWM_CENTRE)
    vehicle.hold(1.0)

    assert vehicle.disarm().accepted
    vehicle.wait_armed(False)
    vehicle.hold(1.0)

    log = read_log(px4)
    thrust = log.topic("vehicle_thrust_setpoint")
    torque = log.topic("vehicle_torque_setpoint")
    assert len(thrust), "vehicle_thrust_setpoint was not logged"

    armed_spans = log.intervals(bench_test_armed)
    assert len(armed_spans) == 1, f"expected one armed bench test span, got {armed_spans}"

    return log, thrust, torque, armed_spans[0]


@pytest.mark.parametrize("switch_pwm, direction", [(PWM_MAX, 1), (PWM_MIN, -1)], ids=["up", "down"])
def test_step_profile(sitl, switch_pwm, direction):
    log, thrust, torque, (armed_start, armed_end) = run_profile(sitl, {"BT_MODE": 1}, switch_pwm,
                                                                excite_s=STEP_DELAY + STEP_DUR + 1.0)

    # from the end of the spin-up until just before disarming
    armed = thrust.between(armed_start + SPINUP_T + 0.1, armed_end - 0.1)
    armed_z = armed["xyz[2]"]

    # hover baseline reached after the spin-up and held before the switch moves
    baseline = armed.between(armed_start + SPINUP_T + 0.2, armed_start + SPINUP_T + 0.8)["xyz[2]"]
    np.testing.assert_allclose(baseline, -HOVER, atol=TOL)

    # exactly one step, of the right sign, size and length (NED: -Z is up)
    stepped = np.abs(armed_z + HOVER) > STEP_MAG / 2
    steps = runs(armed, stepped)
    assert len(steps) == 1, f"expected one step, got {steps}"
    step_start, step_end = steps[0]
    assert step_end - step_start == pytest.approx(STEP_DUR, abs=TIME_TOL)

    step_z = armed.between(step_start, step_end)["xyz[2]"]
    np.testing.assert_allclose(step_z, -HOVER - direction * STEP_MAG, atol=TOL)

    # back to the baseline afterwards
    after = armed.between(step_end + 0.1, armed_end - 0.1)["xyz[2]"]
    np.testing.assert_allclose(after, -HOVER, atol=TOL)

    # only thrust Z is excited
    for field in ("xyz[0]", "xyz[1]"):
        np.testing.assert_allclose(armed[field], 0.0, atol=1e-6, err_msg=field)
    armed_torque = torque.between(armed_start, armed_end)
    for field in ("xyz[0]", "xyz[1]", "xyz[2]"):
        np.testing.assert_allclose(armed_torque[field], 0.0, atol=1e-6, err_msg=f"torque {field}")

    # nothing commanded while disarmed in bench test
    for start, end in log.intervals(bench_test_disarmed):
        disarmed = thrust.between(start + 0.1, end)
        np.testing.assert_allclose(disarmed["xyz[2]"], 0.0, atol=1e-6)


def test_spinup_ramps_hover_thrust(sitl):
    _, thrust, _, (armed_start, _) = run_profile(sitl, {"BT_MODE": 0}, PWM_CENTRE, excite_s=0.5)

    spinup = thrust.between(armed_start, armed_start + SPINUP_T)
    z = spinup["xyz[2]"]
    assert len(z) > 10

    # starts from (almost) nothing and never jumps: monotonic towards -HOVER
    assert z[0] > -HOVER / 2
    assert np.all(np.diff(z) <= 1e-6)
    # linear over BT_SPINUP_T
    expected = -HOVER * np.clip((spinup.t - armed_start) / SPINUP_T, 0, 1)
    np.testing.assert_allclose(z, expected, atol=0.03)


def test_ramp_profile_stops_at_max_value(sitl):
    max_val = 0.05
    ramp_rate = 0.1  # reaches max_val after 0.5 s

    _, thrust, _, (armed_start, armed_end) = run_profile(
        sitl, {"BT_MODE": 2, "BT_RAMP_RATE": ramp_rate, "BT_MAX_VAL": max_val}, PWM_MAX, excite_s=2.0)

    armed = thrust.between(armed_start + SPINUP_T + 0.2, armed_end - 0.1)
    excitation = -(armed["xyz[2]"] + HOVER)

    # never beyond the clamp, and held there once reached
    assert excitation.max() == pytest.approx(max_val, abs=TOL)
    held = armed.where(excitation > max_val - TOL)
    assert held.t[-1] - held.t[0] > 1.0

    # back to the baseline once the switch is centred again
    assert excitation[-1] == pytest.approx(0.0, abs=TOL)
