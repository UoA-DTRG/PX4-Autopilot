"""Horizontal thrust (HT) in Stabilized: switch, sticks, aux tilt channels and mask.

These check the wiring from RC to the attitude setpoint that mc_att_control
publishes, so they run disarmed on any airframe: the quad simply ignores the
X/Y thrust.
"""

import math

import pytest

from rc_layout import CH_HT_MODE, CH_HT_PITCH, CH_HT_ROLL, CH_PITCH, CH_ROLL, PWM_CENTRE, PWM_MAX, PWM_MIN, normalized
from vehicle import MAIN_STABILIZED

pytestmark = pytest.mark.sih

HT_MAX = 0.5
TILT_MAX_DEG = 10.0

HT_PARAMS = {"DTRG_HT_EN": 1, "DTRG_HT_MAX": HT_MAX, "DTRG_HT_R_MAX": TILT_MAX_DEG, "DTRG_HT_P_MAX": TILT_MAX_DEG}

HT_ON = {CH_HT_MODE: PWM_MAX}
HT_OFF = {CH_HT_MODE: PWM_MIN}

# about half deflection; the RC deadzone makes it 0.49 rather than 0.5
HALF_UP = 1750
HALF = normalized(HALF_UP, CH_ROLL)

TOL = 1e-3
ANGLE_TOL_DEG = 0.5


def roll_pitch_deg(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


def attitude_setpoint(vehicle, px4, settle=1.5):
    """Latest vehicle_attitude_setpoint after the RC and the input filters have settled."""
    vehicle.hold(settle)
    sp = px4.listen("vehicle_attitude_setpoint")
    assert sp, "vehicle_attitude_setpoint was never published"
    roll, pitch = roll_pitch_deg(sp["q_d"])
    return sp["thrust_body"], roll, pitch


def start_stabilized(sitl, params=None, rc=None):
    vehicle, px4 = sitl(params={**HT_PARAMS, **(params or {})}, rc=rc or {})
    vehicle.wait_mode(MAIN_STABILIZED)
    return vehicle, px4


def test_switch_off_is_standard_stabilized(sitl):
    vehicle, px4 = start_stabilized(sitl, rc={**HT_OFF, CH_PITCH: PWM_MAX, CH_HT_ROLL: PWM_MAX})

    thrust, roll, pitch = attitude_setpoint(vehicle, px4)

    assert thrust[0] == pytest.approx(0.0, abs=TOL)
    assert thrust[1] == pytest.approx(0.0, abs=TOL)
    assert pitch < -5.0  # the pitch stick tilts the vehicle as usual (nose down)
    assert roll == pytest.approx(0.0, abs=ANGLE_TOL_DEG)  # the aux roll channel does nothing
    assert not px4.listen("horizontal_thrust_limit")


def test_switch_on_sticks_command_thrust_and_vehicle_stays_level(sitl):
    vehicle, px4 = start_stabilized(sitl, rc={**HT_ON, CH_PITCH: PWM_MAX, CH_ROLL: HALF_UP})

    thrust, roll, pitch = attitude_setpoint(vehicle, px4)

    assert thrust[0] == pytest.approx(HT_MAX, abs=TOL)
    assert thrust[1] == pytest.approx(HALF * HT_MAX, abs=TOL)
    assert roll == pytest.approx(0.0, abs=ANGLE_TOL_DEG)
    assert pitch == pytest.approx(0.0, abs=ANGLE_TOL_DEG)
    assert px4.listen("horizontal_thrust_limit")


def test_switch_toggles_ht_at_runtime(sitl):
    vehicle, px4 = start_stabilized(sitl, rc={**HT_OFF, CH_PITCH: PWM_MAX})

    thrust, _, pitch = attitude_setpoint(vehicle, px4)
    assert thrust[0] == pytest.approx(0.0, abs=TOL)
    assert pitch < -5.0

    vehicle.set_rc(CH_HT_MODE, PWM_MAX)
    thrust, _, pitch = attitude_setpoint(vehicle, px4)
    assert thrust[0] == pytest.approx(HT_MAX, abs=TOL)
    assert pitch == pytest.approx(0.0, abs=ANGLE_TOL_DEG)

    vehicle.set_rc(CH_HT_MODE, PWM_MIN)
    thrust, _, pitch = attitude_setpoint(vehicle, px4)
    assert thrust[0] == pytest.approx(0.0, abs=TOL)
    assert pitch < -5.0


def test_switch_ignored_when_ht_disabled(sitl):
    vehicle, px4 = start_stabilized(sitl, params={"DTRG_HT_EN": 0}, rc={**HT_ON, CH_PITCH: PWM_MAX})

    thrust, _, pitch = attitude_setpoint(vehicle, px4)

    assert thrust[0] == pytest.approx(0.0, abs=TOL)
    assert pitch < -5.0
    assert not px4.listen("horizontal_thrust_limit")


def test_aux_channels_command_tilt_up_to_limit(sitl):
    vehicle, px4 = start_stabilized(sitl, rc={**HT_ON, CH_HT_ROLL: PWM_MAX})

    _, roll, pitch = attitude_setpoint(vehicle, px4)
    assert roll == pytest.approx(TILT_MAX_DEG, abs=ANGLE_TOL_DEG)
    assert pitch == pytest.approx(0.0, abs=ANGLE_TOL_DEG)

    # pitch aux up = nose down, like the pitch stick
    vehicle.set_rc(CH_HT_ROLL, PWM_CENTRE)
    vehicle.set_rc(CH_HT_PITCH, PWM_MAX)
    _, roll, pitch = attitude_setpoint(vehicle, px4)
    assert roll == pytest.approx(0.0, abs=ANGLE_TOL_DEG)
    assert pitch == pytest.approx(-TILT_MAX_DEG, abs=ANGLE_TOL_DEG)


@pytest.mark.parametrize("pwm", [1505, 1515])
def test_aux_channel_deadzone(sitl, pwm):
    # HT ignores aux values up to 0.02 (1505 us is 0.01 on channel 9, which has no RC deadzone)
    value = normalized(pwm, CH_HT_ROLL)
    expected = 0.0 if value <= 0.02 else value * TILT_MAX_DEG

    vehicle, px4 = start_stabilized(sitl, rc={**HT_ON, CH_HT_ROLL: pwm})

    _, roll, _ = attitude_setpoint(vehicle, px4)
    assert roll == pytest.approx(expected, abs=0.02)


# (mask, expected X thrust, expected Y thrust) for full pitch stick and half roll stick:
# 0 both axes by HT, 1 X by HT, 2 Y by HT, 3 no HT
MASK_CASES = [
    (0, HT_MAX, HALF * HT_MAX),
    (1, HT_MAX, 0.0),
    (2, 0.0, HALF * HT_MAX),
    pytest.param(3, 0.0, 0.0, marks=pytest.mark.xfail(strict=True, reason=(
        "gap G6: mask 3 should move by tilting only, but applies horizontal thrust on both axes; "
        "fixed on branch salz167/DTRG_HT_refactor, drop this marker once it is merged"))),
]


@pytest.mark.parametrize("mask, expected_x, expected_y", MASK_CASES, ids=["mask0", "mask1", "mask2", "mask3"])
def test_mask_selects_thrust_axes(sitl, mask, expected_x, expected_y):
    vehicle, px4 = start_stabilized(sitl, params={"DTRG_HT_MASK": mask},
                                    rc={**HT_ON, CH_PITCH: PWM_MAX, CH_ROLL: HALF_UP})

    thrust, _, _ = attitude_setpoint(vehicle, px4)

    assert thrust[0] == pytest.approx(expected_x, abs=TOL)
    assert thrust[1] == pytest.approx(expected_y, abs=TOL)


@pytest.mark.xfail(strict=True, reason=(
    "known gap: in Stabilized the demand is stick * DTRG_HT_MAX, so X/Y can only reach the limit at "
    "exactly full stick; RC scaling gives 0.9999999, the thrust 0.49999994, and x_sat never sets"))
def test_full_stick_reports_ht_saturation(sitl):
    vehicle, px4 = start_stabilized(sitl, rc={**HT_ON, CH_PITCH: PWM_MAX})
    vehicle.hold(1.5)

    assert px4.listen("horizontal_thrust_limit")["x_sat"] == 1
    # SYS_STATUS.errors_count3 bit 0 mirrors horizontal_thrust_limit.x_sat
    assert vehicle.sys_status().errors_count3 & 0b1
