"""RC channel conflict check (commander rcChannelConflictCheck).

Two RC_MAP_* functions on the same raw channel must block arming, name both
parameters, and clear again once the overlap is removed. COM_ARM_RC_CONF=1
turns the failure into a warning.

These tests don't stream RC: the check only looks at the configuration, and
COM_RC_IN_MODE=1 (joystick) keeps a missing RC from failing the arming checks
for an unrelated reason.
"""

import re

import pytest

import rc_layout

pytestmark = pytest.mark.sih

# the standard layout, without requiring an RC link
LAYOUT = {**rc_layout.RC_PARAMS, "COM_RC_IN_MODE": 1}

# horizontal thrust switch moved onto the throttle channel
CONFLICT_PARAM = "RC_MAP_HT_MODE"
CONFLICT_CHANNEL = rc_layout.CH_THROTTLE

PREFLIGHT_FAIL = re.compile(
    r"Preflight Fail: (RC_MAP_HT_MODE and RC_MAP_THROTTLE|RC_MAP_THROTTLE and RC_MAP_HT_MODE) "
    rf"both use RC channel {CONFLICT_CHANNEL}")


def conflict_texts(vehicle, since):
    return [t for t in vehicle.texts(since) if PREFLIGHT_FAIL.search(t)]


def test_conflict_blocks_arming_and_names_both_parameters(sitl):
    vehicle, _ = sitl(params=LAYOUT)

    # the failure is reported when it appears (and then at most every 2 s), not on every arm request
    mark = vehicle.now()
    vehicle.set_param(CONFLICT_PARAM, CONFLICT_CHANNEL)
    vehicle.wait_prearm(ok=False, timeout=5)

    assert not vehicle.arm().accepted
    vehicle.wait_until(lambda: conflict_texts(vehicle, mark), 5, "the Preflight Fail text naming both parameters")
    assert not vehicle.armed()


def test_resolving_conflict_allows_arming_again(sitl):
    vehicle, _ = sitl(params=LAYOUT)

    vehicle.set_param(CONFLICT_PARAM, CONFLICT_CHANNEL)
    vehicle.wait_prearm(ok=False, timeout=5)

    mark = vehicle.now()
    vehicle.set_param(CONFLICT_PARAM, rc_layout.CH_HT_MODE)
    vehicle.wait_text("RC channel conflict resolved", since=mark)
    vehicle.wait_prearm(ok=True, timeout=5)

    assert vehicle.arm().accepted
    vehicle.wait_armed()


def test_conflict_configured_before_boot_blocks_arming(sitl):
    # the parameter table is scanned once at boot, so this checks the discovery itself
    vehicle, px4 = sitl(params={**LAYOUT, CONFLICT_PARAM: CONFLICT_CHANNEL}, wait_ready=False)

    # reported on the console at boot, before a ground station may be connected
    vehicle.wait_until(lambda: PREFLIGHT_FAIL.search(px4.console()), 60, "the Preflight Fail text on the console")

    # warning-only mode: everything else is healthy once this passes
    vehicle.set_param("COM_ARM_RC_CONF", 1)
    vehicle.wait_prearm(ok=True, timeout=90)

    # back to blocking: the boot-time conflict is what stops arming
    vehicle.set_param("COM_ARM_RC_CONF", 0)
    vehicle.wait_prearm(ok=False, timeout=5)
    assert not vehicle.arm().accepted
    assert not vehicle.armed()


def test_com_arm_rc_conf_warns_without_blocking(sitl):
    vehicle, _ = sitl(params={**LAYOUT, "COM_ARM_RC_CONF": 1})

    mark = vehicle.now()
    vehicle.set_param(CONFLICT_PARAM, CONFLICT_CHANNEL)
    vehicle.wait_text(f"RC ch {CONFLICT_CHANNEL}: ", since=mark)

    assert vehicle.prearm_ok()
    assert vehicle.arm().accepted
    vehicle.wait_armed()


def test_failsafe_channel_may_share_throttle(sitl):
    # RC_MAP_FAILSAFE is documented to sit on the throttle channel, so it is exempt
    vehicle, _ = sitl(params={**LAYOUT, "RC_MAP_FAILSAFE": rc_layout.CH_THROTTLE})

    assert vehicle.prearm_ok()


def test_flight_mode_buttons_only_count_without_mode_switch(sitl):
    # RC_MAP_FLTM_BTN (a channel bitmask) is only read while RC_MAP_FLTMODE is unassigned
    ht_mode_bit = 1 << (rc_layout.CH_HT_MODE - 1)
    vehicle, _ = sitl(params={**LAYOUT, "RC_MAP_FLTM_BTN": ht_mode_bit})

    assert vehicle.prearm_ok()

    vehicle.set_param("RC_MAP_FLTMODE", 0)
    vehicle.wait_prearm(ok=False, timeout=5)
