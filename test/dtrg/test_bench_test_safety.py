"""Bench test mode: how it can be entered and when it may arm.

Bench test drives the motors with every control loop off, so:
- it is only reachable from an RC flight mode slot, never over MAVLink
- it cannot be entered while armed (e.g. in flight)
- it only arms with BT_ARM_ENABLE=1 and the direction switch centred
- it may always be disarmed
"""

import pytest

import rc_layout
from rc_layout import CH_CMD_SIGN, CH_FLTMODE, PWM_CENTRE, PWM_MAX, PWM_MIN, SLOT_BENCH_TEST, SLOT_STABILIZED, slot_pwm
from vehicle import MAIN_BENCH_TEST, MAIN_STABILIZED

pytestmark = pytest.mark.sih

# far below hover thrust: the simulated vehicle is not tied down like a real rig
BENCH_PARAMS = {"BT_ARM_ENABLE": 1, "BT_MODE": 1, "BT_HOVER_THR": 0.2}

BENCH_SLOT = {CH_FLTMODE: slot_pwm(SLOT_BENCH_TEST)}


def test_rc_slot_selects_bench_test_while_disarmed(sitl):
    vehicle, _ = sitl(params=BENCH_PARAMS, rc={})
    vehicle.wait_mode(MAIN_STABILIZED)

    vehicle.set_rc(CH_FLTMODE, slot_pwm(SLOT_BENCH_TEST))
    vehicle.wait_mode(MAIN_BENCH_TEST)


def test_bench_test_rejected_while_armed(sitl):
    vehicle, _ = sitl(params=BENCH_PARAMS, rc={})
    vehicle.wait_mode(MAIN_STABILIZED)
    assert vehicle.arm().accepted
    vehicle.wait_armed()

    mark = vehicle.now()
    vehicle.set_rc(CH_FLTMODE, slot_pwm(SLOT_BENCH_TEST))
    vehicle.wait_text("Bench test mode denied: disarm first", since=mark)

    vehicle.hold(1.0)
    assert vehicle.main_mode() == MAIN_STABILIZED
    assert vehicle.armed()

    # control case: once disarmed the same switch selects bench test
    assert vehicle.disarm().accepted
    vehicle.wait_armed(False)
    # the mode is re-evaluated on a switch change: move away and back, long enough for
    # rc_update to see the intermediate position
    vehicle.set_rc(CH_FLTMODE, slot_pwm(SLOT_STABILIZED))
    vehicle.hold(1.0)
    vehicle.set_rc(CH_FLTMODE, slot_pwm(SLOT_BENCH_TEST))
    vehicle.wait_mode(MAIN_BENCH_TEST)


def test_mavlink_cannot_select_bench_test(sitl):
    vehicle, _ = sitl(params=BENCH_PARAMS, rc={})
    vehicle.wait_mode(MAIN_STABILIZED)

    # Commander has no MAVLink mapping for the bench test main mode. The command is
    # still ACKed as accepted (an unknown custom mode is a no-op), so only the mode counts.
    vehicle.set_mode(MAIN_BENCH_TEST)
    vehicle.hold(1.0)
    assert vehicle.main_mode() == MAIN_STABILIZED


def test_arming_needs_bt_arm_enable(sitl):
    vehicle, _ = sitl(params={**BENCH_PARAMS, "BT_ARM_ENABLE": 0}, rc=BENCH_SLOT)
    vehicle.wait_mode(MAIN_BENCH_TEST)

    mark = vehicle.now()
    assert not vehicle.arm().accepted
    vehicle.wait_text("Arming denied: bench test not enabled", since=mark)
    assert not vehicle.armed()

    # control case: enabled, the same request arms
    vehicle.set_param("BT_ARM_ENABLE", 1)
    assert vehicle.arm().accepted
    vehicle.wait_armed()


@pytest.mark.parametrize("switch_pwm", [PWM_MAX, PWM_MIN], ids=["switch_up", "switch_down"])
def test_arming_needs_centred_direction_switch(sitl, switch_pwm):
    vehicle, _ = sitl(params=BENCH_PARAMS, rc={**BENCH_SLOT, CH_CMD_SIGN: switch_pwm})
    vehicle.wait_mode(MAIN_BENCH_TEST)

    mark = vehicle.now()
    assert not vehicle.arm().accepted
    vehicle.wait_text("Arming denied: centre the bench test direction switch", since=mark)
    assert not vehicle.armed()

    vehicle.set_rc(CH_CMD_SIGN, PWM_CENTRE)
    vehicle.hold(0.5)
    assert vehicle.arm().accepted
    vehicle.wait_armed()


@pytest.mark.parametrize("params", [
    {"BT_MODE": 0},
    {"RC_MAP_CMD_SIGN": 0},
], ids=["hover_only_profile", "no_switch_assigned"])
def test_off_centre_switch_allowed_when_it_cannot_excite(sitl, params):
    vehicle, _ = sitl(params={**BENCH_PARAMS, **params}, rc={**BENCH_SLOT, CH_CMD_SIGN: PWM_MAX})
    vehicle.wait_mode(MAIN_BENCH_TEST)

    assert vehicle.arm().accepted
    vehicle.wait_armed()


def test_disarm_allowed_in_bench_test(sitl):
    # On a real rig the land detector reports "in air" once the motors spin, and bench
    # test explicitly allows disarming anyway. The simulated vehicle stays landed, so
    # this only checks that a normal disarm is honoured in this mode.
    vehicle, _ = sitl(params=BENCH_PARAMS, rc=BENCH_SLOT)
    vehicle.wait_mode(MAIN_BENCH_TEST)
    assert vehicle.arm().accepted
    vehicle.wait_armed()

    vehicle.hold(2.0)
    assert vehicle.disarm().accepted
    vehicle.wait_armed(False)


def test_rc_layout_has_no_conflicts():
    # every test relies on the standard layout passing the RC channel conflict check
    channels = [v for k, v in rc_layout.RC_PARAMS.items() if k.startswith("RC_MAP_") and v > 0]
    assert len(channels) == len(set(channels))
