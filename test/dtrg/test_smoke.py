"""Boot SIH and check the harness and the DTRG firmware marker."""

import pytest

from vehicle import DTRG_FIRMWARE_MARKER, MAIN_STABILIZED

pytestmark = pytest.mark.sih


def test_boots_dtrg_firmware_and_is_ready_to_arm(sitl):
    vehicle, _ = sitl()

    assert vehicle.sys_status().errors_count4 == DTRG_FIRMWARE_MARKER
    assert vehicle.prearm_ok()


def test_rc_override_drives_the_flight_mode_switch(sitl):
    # the standard RC layout starts on the Stabilized slot
    vehicle, _ = sitl(rc={})

    vehicle.wait_mode(MAIN_STABILIZED)
    assert vehicle.get_param("COM_RC_IN_MODE") == 0
