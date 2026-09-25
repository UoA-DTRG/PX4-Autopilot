"""Tier 3: horizontal thrust (HT) in flight, on the fully actuated planarOcto in SIH.

The vehicle really flies here: SIH simulates the eight tilted rotors from the
airframe's CA_ROTOR* geometry, and the checks are on SIH's ground truth. Plan
IDs refer to Tools/dtrg/SITL_TESTING.md section 6.

Run: python3 -m pytest test/dtrg -m flight -v
"""

import math

import pytest

from flight import FLIGHT_PARAMS, altitude_error, estimated_attitude, horizontal_error, take_off, truth
from rc_layout import (CH_FLTMODE, CH_HT_MODE, CH_HT_PITCH, CH_HT_ROLL, CH_PITCH, CH_THROTTLE, PWM_CENTRE, PWM_MAX,
                       PWM_MIN, SLOT_BENCH_TEST, SLOT_POSITION, SLOT_STABILIZED, slot_pwm)
from ulog_checks import read_log
from vehicle import AUTO_LAND, AUTO_LOITER, MAIN_AUTO, MAIN_OFFBOARD, MAIN_POSCTL, MAIN_STABILIZED

pytestmark = [pytest.mark.sih, pytest.mark.flight, pytest.mark.fully_actuated]

HT_MAX = 0.5
TILT_MAX_DEG = 10.0

HT_PARAMS = {**FLIGHT_PARAMS, "DTRG_HT_EN": 1, "DTRG_HT_MAX": HT_MAX, "DTRG_HT_R_MAX": TILT_MAX_DEG,
             "DTRG_HT_P_MAX": TILT_MAX_DEG}

MOVE_NORTH = 5.0

# Holding a tilt with horizontal thrust, the true attitude is ~2 deg off the estimate (G14), so
# commanded tilts are checked tightly against the estimate and loosely against the truth.
EST_TILT_TOL_DEG = 2.0
TRUE_TILT_TOL_DEG = 3.5


class G11DriftWhileTilted(AssertionError):
    """Known gap G11. The xfail only accepts this, so a timeout or crash still fails the test."""


class G13PitchSign(AssertionError):
    """Known gap G13. The xfail only accepts this, so a timeout or crash still fails the test."""


class G8TiltNotLimited(AssertionError):
    """Known gap G8. The xfail only accepts this, so a timeout or crash still fails the test."""


def fly(sitl, params=None, rc=None):
    """Boot with the standard RC layout (HT switch off), take off and hold at TAKEOFF_ALT."""
    vehicle, px4 = sitl(params={**HT_PARAMS, **(params or {})}, rc=rc or {})
    # the RC slot (Stabilized) is applied once after boot; let it happen before Takeoff
    vehicle.wait_mode(MAIN_STABILIZED)
    take_off(vehicle)
    return vehicle, px4


def offboard_hold(vehicle):
    """Switch to Offboard, holding the current position. Returns (x, y, z, yaw)."""
    x, y, z = vehicle.position()
    yaw = vehicle.yaw()
    vehicle.set_offboard_position(x, y, z, yaw)
    vehicle.hold(1.0)
    assert vehicle.set_mode(MAIN_OFFBOARD).accepted
    vehicle.wait_mode(MAIN_OFFBOARD)
    vehicle.hold(2.0)
    return x, y, z, yaw


def offboard_move_north(vehicle, distance):
    """Hold in Offboard at the current position, then move ``distance`` north. Returns (start, end) PX4 times."""
    x, y, z, yaw = offboard_hold(vehicle)

    start = vehicle.boot_time()
    vehicle.set_offboard_position(x + distance, y, z, yaw)
    vehicle.wait_position(x + distance, y, z, tolerance=0.3, timeout=30)
    vehicle.hold(3.0)
    return start, vehicle.boot_time()


def test_a1_take_off_hold_and_land(sitl):
    vehicle, px4 = fly(sitl)

    start = vehicle.boot_time()
    vehicle.hold(10.0)
    end = vehicle.boot_time()

    assert vehicle.set_mode(MAIN_AUTO, AUTO_LAND).accepted
    vehicle.wait_armed(False, timeout=30)

    log = read_log(px4)
    error = altitude_error(log, start, end)
    assert abs(error).max() < 0.5, f"altitude off its setpoint by up to {abs(error).max():.2f} m during the hold"
    assert horizontal_error(log, start, end) < 1.0


def test_a2_ht_moves_the_vehicle_level(sitl):
    vehicle, px4 = fly(sitl)

    vehicle.set_rc(CH_HT_MODE, PWM_MAX)
    vehicle.hold(2.0)
    start, end = offboard_move_north(vehicle, MOVE_NORTH)

    tr = truth(read_log(px4), start, end)
    moved = tr.last(1.0).x.mean() - tr.x[0]
    assert moved == pytest.approx(MOVE_NORTH, abs=0.5)
    assert tr.tilt.max() < 3.0, f"tilted {tr.tilt.max():.1f} deg while moving with HT"


def test_a3_without_ht_the_vehicle_tilts_to_move(sitl):
    # control case for A2
    vehicle, px4 = fly(sitl)

    start, end = offboard_move_north(vehicle, MOVE_NORTH)

    tr = truth(read_log(px4), start, end)
    moved = tr.last(1.0).x.mean() - tr.x[0]
    assert moved == pytest.approx(MOVE_NORTH, abs=0.5)
    # accelerating north is nose down
    assert tr.pitch.min() < -5.0, f"pitched only {tr.pitch.min():.1f} deg while moving without HT"


def hold_sending_tilt(vehicle, roll, pitch, seconds):
    """Offboard HT tilt: stream DEBUG_FLOAT_ARRAY [roll, pitch] (rad) at 10 Hz for ``seconds``."""
    for _ in range(int(seconds * 10)):
        vehicle.send_debug_float_array([roll, pitch])
        vehicle.hold(0.1)


@pytest.mark.xfail(strict=True, raises=G11DriftWhileTilted,
                   reason="G11: HT gets 41% of the horizontal force the position controller asks for, "
                   "so it cannot hold position while tilted")
@pytest.mark.parametrize("channel, axis", [(CH_HT_ROLL, "roll"), (CH_HT_PITCH, "pitch")], ids=["roll", "pitch"])
def test_a4_aux_tilt_in_hover_holds_position(sitl, channel, axis):
    vehicle, px4 = fly(sitl)

    vehicle.set_rc(CH_HT_MODE, PWM_MAX)
    vehicle.hold(2.0)
    start = vehicle.boot_time()
    vehicle.set_rc(channel, PWM_MAX)
    vehicle.hold(8.0)
    end = vehicle.boot_time()

    log = read_log(px4)
    settled = truth(log, start, end).last(3.0)
    angle = getattr(settled, axis)
    assert abs(angle.mean()) == pytest.approx(TILT_MAX_DEG, abs=TRUE_TILT_TOL_DEG)  # its sign is checked in A4b
    error = horizontal_error(log, start, end)
    if error >= 0.5:
        raise G11DriftWhileTilted(f"{error:.2f} m off position holding {axis} at the HT limit")


@pytest.mark.parametrize("channel, axis, other", [
    (CH_HT_ROLL, "roll", "pitch"),
    pytest.param(CH_HT_PITCH, "pitch", "roll", marks=pytest.mark.xfail(
        strict=True, raises=G13PitchSign,
        reason="G13: in Position/Hold/Offboard the HT pitch aux channel pitches nose up, in Stabilized nose down")),
], ids=["roll", "pitch"])
def test_a4b_aux_tilt_reaches_the_limit(sitl, channel, axis, other):
    # the attitude half of A4, which does not depend on G11. Aux up: roll right, and for pitch
    # nose down, as in Stabilized (test_horizontal_thrust.py) and like the pitch stick.
    expected = TILT_MAX_DEG if axis == "roll" else -TILT_MAX_DEG
    vehicle, px4 = fly(sitl)

    vehicle.set_rc(CH_HT_MODE, PWM_MAX)
    vehicle.hold(2.0)
    vehicle.set_rc(channel, PWM_MAX)
    vehicle.hold(4.0)
    start = vehicle.boot_time()
    vehicle.hold(2.0)
    end = vehicle.boot_time()

    log = read_log(px4)
    tr = truth(log, start, end)
    estimated = dict(zip(("roll", "pitch"), estimated_attitude(log, start, end)))
    angle = getattr(tr, axis).mean()
    assert abs(estimated[axis].mean()) == pytest.approx(TILT_MAX_DEG, abs=EST_TILT_TOL_DEG)
    assert abs(angle) == pytest.approx(TILT_MAX_DEG, abs=TRUE_TILT_TOL_DEG)
    assert abs(getattr(tr, other)).max() < 3.0
    if axis == "pitch" and angle * expected < 0:
        raise G13PitchSign(f"pitch aux up pitched {angle:+.1f} deg in Hold, {expected:+.1f} deg in Stabilized")
    assert angle == pytest.approx(expected, abs=TRUE_TILT_TOL_DEG)


def test_a5_offboard_tilt_setpoint_in_hover(sitl):
    vehicle, px4 = fly(sitl)

    vehicle.set_rc(CH_HT_MODE, PWM_MAX)
    offboard_hold(vehicle)
    start = vehicle.boot_time()
    hold_sending_tilt(vehicle, 0.1, 0.0, 8.0)
    end = vehicle.boot_time()

    log = read_log(px4)
    tr = truth(log, start, end)
    settled = tr.last(3.0)
    estimated_roll, _ = estimated_attitude(log, end - 3.0, end)
    assert estimated_roll.mean() == pytest.approx(math.degrees(0.1), abs=EST_TILT_TOL_DEG)
    assert settled.roll.mean() == pytest.approx(math.degrees(0.1), abs=TRUE_TILT_TOL_DEG)
    assert horizontal_error(log, start, end) < 0.5
    assert vehicle.main_mode() == MAIN_OFFBOARD


@pytest.mark.xfail(strict=True, raises=G8TiltNotLimited,
                   reason="G8: the Offboard HT tilt is not limited to DTRG_HT_R_MAX")
def test_a5_offboard_tilt_is_limited(sitl):
    vehicle, px4 = fly(sitl)

    vehicle.set_rc(CH_HT_MODE, PWM_MAX)
    offboard_hold(vehicle)
    start = vehicle.boot_time()
    hold_sending_tilt(vehicle, math.radians(2 * TILT_MAX_DEG), 0.0, 5.0)
    end = vehicle.boot_time()

    tr = truth(read_log(px4), start, end)
    if tr.roll.max() >= TILT_MAX_DEG + 2.0:
        raise G8TiltNotLimited(f"rolled {tr.roll.max():.1f} deg, limit {TILT_MAX_DEG} deg")


def test_a7_toggling_ht_in_hover_is_smooth(sitl):
    vehicle, px4 = fly(sitl)

    # the same hover without toggling, as the reference for the noise of this run
    ref_start = vehicle.boot_time()
    vehicle.hold(8.0)
    start = vehicle.boot_time()

    for _ in range(5):
        vehicle.set_rc(CH_HT_MODE, PWM_MAX)
        vehicle.hold(2.0)
        vehicle.set_rc(CH_HT_MODE, PWM_MIN)
        vehicle.hold(2.0)

    end = vehicle.boot_time()

    log = read_log(px4)
    ref = truth(log, ref_start, start)
    tr = truth(log, start, end)
    error = altitude_error(log, start, end)
    assert abs(error).max() < 0.7, f"altitude off its setpoint by up to {abs(error).max():.2f} m while toggling"
    assert tr.tilt.max() < max(8.0, ref.tilt.max() + 2.0), \
        f"tilted {tr.tilt.max():.1f} deg while toggling, {ref.tilt.max():.1f} deg in the plain hover"
    assert horizontal_error(log, start, end) < 1.0


def test_a8_stabilized_ht_stick_moves_the_vehicle_level(sitl):
    # higher, as the manual throttle does not hold altitude exactly
    vehicle, px4 = sitl(params={**HT_PARAMS, "MIS_TAKEOFF_ALT": 6.0}, rc={})
    vehicle.wait_mode(MAIN_STABILIZED)
    take_off(vehicle, altitude=6.0)

    # the RC slot is still Stabilized from the boot; a slot only acts when it changes
    vehicle.set_rc(CH_THROTTLE, PWM_CENTRE)
    vehicle.set_rc(CH_FLTMODE, slot_pwm(SLOT_POSITION))
    vehicle.wait_mode(MAIN_POSCTL)
    vehicle.set_rc(CH_HT_MODE, PWM_MAX)
    vehicle.hold(2.0)
    vehicle.set_rc(CH_FLTMODE, slot_pwm(SLOT_STABILIZED))
    vehicle.wait_mode(MAIN_STABILIZED)

    start = vehicle.boot_time()
    vehicle.set_rc(CH_PITCH, PWM_MAX)
    vehicle.hold(3.0)
    end = vehicle.boot_time()
    vehicle.set_rc(CH_PITCH, PWM_CENTRE)
    vehicle.set_rc(CH_FLTMODE, slot_pwm(SLOT_POSITION))
    vehicle.wait_mode(MAIN_POSCTL)

    log = read_log(px4)
    tr = truth(log, start, end)
    velocity = log.topic("vehicle_local_position_groundtruth").between(start, end)
    yaw = math.radians(tr.yaw.mean())
    forward = velocity["vx"] * math.cos(yaw) + velocity["vy"] * math.sin(yaw)

    assert forward[-1] > 1.0, f"forward speed only {forward[-1]:.2f} m/s after 3 s of full HT stick"
    assert tr.tilt.max() < 3.0, f"tilted {tr.tilt.max():.1f} deg with HT"


def test_b1b_bench_test_rejected_in_flight(sitl):
    vehicle, px4 = fly(sitl)

    since = vehicle.now()
    vehicle.set_rc(CH_FLTMODE, slot_pwm(SLOT_BENCH_TEST))
    vehicle.wait_text("Bench test mode denied: disarm first", since=since)

    start = vehicle.boot_time()
    vehicle.hold(3.0)
    end = vehicle.boot_time()

    assert vehicle.armed()
    assert vehicle.mode() == (MAIN_AUTO, AUTO_LOITER)
    altitude = -truth(read_log(px4), start, end).z
    assert altitude.min() > 1.5, f"came down to {altitude.min():.2f} m"
