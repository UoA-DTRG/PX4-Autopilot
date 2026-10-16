"""Tier 3: constant command allocation in flight, on the planetaryHex in SIH.

With the switch (RC_MAP_CC_MODE) on, the centre rotors, Motors 7 and 8, are held
at DTRG_CC_M_CMD and the six outer rotors take the rest of the demand. These
tests always run on sihsim_planetary_hex, whatever ``--airframe`` says. See
Tools/dtrg/DTRG_Automated_Testing.md.

Run: python3 -m pytest test/dtrg/test_flight_constant_command.py -v
"""

import numpy as np
import pytest

from airframes import AIRFRAMES
from flight import FLIGHT_PARAMS, altitude_error, horizontal_error, take_off, truth
from rc_layout import CH_CC_MODE, PWM_MAX, PWM_MIN
from ulog_checks import read_log
from vehicle import AUTO_LAND, MAIN_AUTO, MAIN_STABILIZED

pytestmark = [pytest.mark.sih, pytest.mark.flight, pytest.mark.fully_actuated]

# Below the ~0.72 / 0.58 the normal allocation gives the centre rotors in a hover, so the
# outer rotors have to make up the difference
CC_M_CMD = 0.5
CC_RAMP = 1.0

CC_PARAMS = {**FLIGHT_PARAMS, "DTRG_CC_EN": 1, "DTRG_CC_M_CMD": CC_M_CMD, "DTRG_CC_RAMP": CC_RAMP}

# time after a switch change by which the ramp has ended
SETTLED = CC_RAMP + 0.5

OUTER = [f"control[{i}]" for i in range(6)]


@pytest.fixture
def airframe():
    # overrides the session fixture of conftest.py: constant command needs the planetaryHex
    return AIRFRAMES["sihsim_planetary_hex"]


def fly(sitl, params=None):
    """Boot with the standard RC layout (constant command switch off), take off and hold."""
    vehicle, px4 = sitl(params={**CC_PARAMS, **(params or {})}, rc={})
    # the RC slot (Stabilized) is applied once after boot; let it happen before Takeoff
    vehicle.wait_mode(MAIN_STABILIZED)
    take_off(vehicle)
    return vehicle, px4


def outer_mean(motors):
    return float(np.mean([motors[name] for name in OUTER]))


def test_constant_command_holds_the_centre_rotors(sitl):
    vehicle, px4 = fly(sitl)

    before = vehicle.boot_time()
    vehicle.hold(5.0)

    on = vehicle.boot_time()
    since = vehicle.now()
    vehicle.set_rc(CH_CC_MODE, PWM_MAX)
    vehicle.wait_text("Constant command engaged", since=since)
    vehicle.hold(10.0)

    off = vehicle.boot_time()
    since = vehicle.now()
    vehicle.set_rc(CH_CC_MODE, PWM_MIN)
    vehicle.wait_text("Constant command released", since=since)
    vehicle.hold(5.0)
    end = vehicle.boot_time()

    log = read_log(px4)
    motors = log.topic("actuator_motors")
    normal = motors.between(before, on)
    held = motors.between(on + SETTLED, off)
    after = motors.between(off + SETTLED, end)

    assert np.allclose(held["control[6]"], CC_M_CMD, atol=1e-3), \
        f"Motor 7 at {held['control[6]'].min():.3f}-{held['control[6]'].max():.3f}, not {CC_M_CMD}"
    assert np.allclose(held["control[7]"], CC_M_CMD, atol=1e-3), \
        f"Motor 8 at {held['control[7]'].min():.3f}-{held['control[7]'].max():.3f}, not {CC_M_CMD}"

    # the centre rotors give up thrust, the outer rotors take it over
    assert normal["control[6]"].mean() > CC_M_CMD + 0.1
    assert outer_mean(held) > outer_mean(normal) + 0.03, \
        f"outer rotors at {outer_mean(held):.3f} held against {outer_mean(normal):.3f} normally"

    # released, the normal allocation is back
    assert after["control[6]"].mean() == pytest.approx(normal["control[6]"].mean(), abs=0.03)
    assert after["control[7]"].mean() == pytest.approx(normal["control[7]"].mean(), abs=0.03)

    status = log.topic("dtrg_constant_command")
    assert np.all(status.between(on + SETTLED, off)["blend"] == 1.0)
    assert np.all(status.between(off + SETTLED, end)["blend"] == 0.0)

    # the vehicle does not notice: same hold through engaging, holding and releasing
    error = altitude_error(log, on, end)
    assert abs(error).max() < 0.5, f"altitude off its setpoint by up to {abs(error).max():.2f} m"
    assert horizontal_error(log, on, end) < 1.0
    tilt = truth(log, on, end).tilt.max()
    assert tilt < 5.0, f"tilted {tilt:.1f} deg"


def test_switch_does_nothing_when_disabled(sitl):
    # control case: DTRG_CC_EN = 0 leaves the allocation alone
    vehicle, px4 = fly(sitl, params={"DTRG_CC_EN": 0})

    since = vehicle.now()
    on = vehicle.boot_time()
    vehicle.set_rc(CH_CC_MODE, PWM_MAX)
    vehicle.hold(5.0)
    end = vehicle.boot_time()

    assert not vehicle.saw_text("Constant command", since=since)

    log = read_log(px4)
    held = log.topic("actuator_motors").between(on + SETTLED, end)
    assert held["control[6]"].min() > CC_M_CMD + 0.1, "Motor 7 left the normal allocation with DTRG_CC_EN 0"


def test_landing_releases_the_centre_rotors(sitl):
    vehicle, px4 = fly(sitl)

    since = vehicle.now()
    vehicle.set_rc(CH_CC_MODE, PWM_MAX)
    vehicle.wait_text("Constant command engaged", since=since)
    vehicle.hold(3.0)

    since = vehicle.now()
    assert vehicle.set_mode(MAIN_AUTO, AUTO_LAND).accepted
    vehicle.wait_text("Constant command released", since=since, timeout=40)
    vehicle.wait_armed(False, timeout=30)
