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

# half deflection
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

    # pitch aux up = nose up: the aux channel sets the tilt directly, opposite to the pitch stick
    vehicle.set_rc(CH_HT_ROLL, PWM_CENTRE)
    vehicle.set_rc(CH_HT_PITCH, PWM_MAX)
    _, roll, pitch = attitude_setpoint(vehicle, px4)
    assert roll == pytest.approx(0.0, abs=ANGLE_TOL_DEG)
    assert pitch == pytest.approx(TILT_MAX_DEG, abs=ANGLE_TOL_DEG)


@pytest.mark.parametrize("pwm", [1505, 1515])
def test_aux_channel_deadzone(sitl, pwm):
    # HT ignores aux values up to 0.02 (1505 us is 0.01)
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
]


@pytest.mark.parametrize("mask, expected_x, expected_y", MASK_CASES, ids=["mask0", "mask1", "mask2"])
def test_mask_selects_thrust_axes(sitl, mask, expected_x, expected_y):
    vehicle, px4 = start_stabilized(sitl, params={"DTRG_HT_MASK": mask},
                                    rc={**HT_ON, CH_PITCH: PWM_MAX, CH_ROLL: HALF_UP})

    thrust, _, _ = attitude_setpoint(vehicle, px4)

    assert thrust[0] == pytest.approx(expected_x, abs=TOL)
    assert thrust[1] == pytest.approx(expected_y, abs=TOL)


def test_full_stick_gives_ht_max(sitl):
    # The demand is stick * DTRG_HT_MAX, so full stick reaches the limit and is never clipped.
    # x_sat (|X| >= limit) is then decided by whether the RC scaling lands on exactly 1.0, which
    # it does on Linux but not on macOS (0.99999988), so it is not asserted here; the limit flags
    # are covered by DtrgHorizontalThrustTest.cpp, which feeds exact values.
    vehicle, px4 = start_stabilized(sitl, rc={**HT_ON, CH_PITCH: PWM_MAX})

    thrust, _, _ = attitude_setpoint(vehicle, px4)
    assert thrust[0] == pytest.approx(HT_MAX, abs=TOL)

    limit = px4.listen("horizontal_thrust_limit")
    assert limit["y_sat"] == 0


# Mask x split: how the sticks divide between horizontal thrust and tilt -----------------------
#
# On an HT axis (X for masks 0 and 1, Y for masks 0 and 2) the stick commands DTRG_HT_SPLIT of
# DTRG_HT_MAX as thrust and (1 - DTRG_HT_SPLIT) of MPC_MAN_TILT_MAX as tilt with the split enabled;
# without it, the thrust share is 1 and the aux channel sets the tilt. On the other axis the stick
# tilts as in standard Stabilized and commands no thrust.

SPLIT_PITCH_PWM = HALF_UP  # forward, 0.49
SPLIT_ROLL_PWM = 1625      # right, 0.23: different from pitch so an X/Y swap cannot pass
AUX_ROLL_PWM = 1750        # +0.5 of DTRG_HT_R_MAX, roll right
AUX_PITCH_PWM = 1300       # -0.4 of DTRG_HT_P_MAX, nose down

# (mask, DTRG_HT_SPLIT_EN, DTRG_HT_SPLIT)
SPLIT_CASES = [
    (0, 0, 0.25),
    (1, 0, 0.25),
    (2, 0, 0.25),
    (0, 1, 0.0),  # no horizontal thrust: standard Stabilized
    (0, 1, 0.25),
    (0, 1, 1.0),  # horizontal thrust only, level
    (1, 1, 0.25),
    (2, 1, 0.25),
]


def expected_split(mask, split_en, split, tilt_max_deg):
    """Expected body X/Y thrust and roll/pitch [deg] of the attitude setpoint, as mc_att_control computes them."""
    pitch_stick = normalized(SPLIT_PITCH_PWM, CH_PITCH)
    roll_stick = normalized(SPLIT_ROLL_PWM, CH_ROLL)
    uses_x = mask != 2
    uses_y = mask != 1

    thrust_share = split if split_en else 1.0
    x = pitch_stick * HT_MAX * thrust_share if uses_x else 0.0
    y = roll_stick * HT_MAX * thrust_share if uses_y else 0.0

    # tilt in the stick convention (pitch > 0 is nose down) [rad]
    tilt_max = math.radians(tilt_max_deg)
    aux_roll = normalized(AUX_ROLL_PWM, CH_HT_ROLL) * math.radians(TILT_MAX_DEG)
    aux_pitch = -normalized(AUX_PITCH_PWM, CH_HT_PITCH) * math.radians(TILT_MAX_DEG)

    def ht_axis_tilt(stick, aux):
        return stick * tilt_max * (1.0 - split) if split_en else aux

    roll = ht_axis_tilt(roll_stick, aux_roll) if uses_y else roll_stick * tilt_max
    pitch = ht_axis_tilt(pitch_stick, aux_pitch) if uses_x else pitch_stick * tilt_max

    # mc_att_control builds the setpoint as an axis-angle rotation about (roll, -pitch, 0)
    angle = math.hypot(roll, pitch)
    assert angle < tilt_max, "the case must stay inside MPC_MAN_TILT_MAX, which would rescale both axes"
    if angle > 0.0:
        s = math.sin(angle / 2) / angle
        q = (math.cos(angle / 2), roll * s, -pitch * s, 0.0)
    else:
        q = (1.0, 0.0, 0.0, 0.0)

    return x, y, *roll_pitch_deg(q)


@pytest.mark.parametrize("mask, split_en, split", SPLIT_CASES,
                         ids=[f"mask{m}-split{'%g' % s if e else 'off'}" for m, e, s in SPLIT_CASES])
def test_mask_and_split_divide_stick_between_thrust_and_tilt(sitl, mask, split_en, split):
    vehicle, px4 = start_stabilized(
        sitl, params={"DTRG_HT_MASK": mask, "DTRG_HT_SPLIT_EN": split_en, "DTRG_HT_SPLIT": split},
        rc={**HT_ON, CH_PITCH: SPLIT_PITCH_PWM, CH_ROLL: SPLIT_ROLL_PWM, CH_HT_ROLL: AUX_ROLL_PWM,
            CH_HT_PITCH: AUX_PITCH_PWM})
    tilt_max_deg = vehicle.get_param("MPC_MAN_TILT_MAX")

    thrust, roll, pitch = attitude_setpoint(vehicle, px4)
    x, y, expected_roll, expected_pitch = expected_split(mask, split_en, split, tilt_max_deg)

    assert thrust[0] == pytest.approx(x, abs=TOL), "body X thrust"
    assert thrust[1] == pytest.approx(y, abs=TOL), "body Y thrust"
    assert roll == pytest.approx(expected_roll, abs=0.3), "roll"
    assert pitch == pytest.approx(expected_pitch, abs=0.3), "pitch"
