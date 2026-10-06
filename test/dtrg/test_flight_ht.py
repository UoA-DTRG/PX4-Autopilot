"""Tier 3: horizontal thrust (HT) in flight, on the fully actuated planarOcto in SIH.

The vehicle really flies here: SIH simulates the eight tilted rotors from the
airframe's CA_ROTOR* geometry, and the checks are on SIH's ground truth. See
Tools/dtrg/DTRG_Automated_Testing.md.

Run: python3 -m pytest test/dtrg -m flight -v
"""

import math

import numpy as np
import pytest

from flight import FLIGHT_PARAMS, altitude_error, estimated_attitude, horizontal_error, take_off, truth
from rc_layout import (CH_FLTMODE, CH_HT_MODE, CH_HT_PITCH, CH_HT_ROLL, CH_PITCH, CH_THROTTLE, PWM_CENTRE, PWM_MAX,
                       PWM_MIN, SLOT_BENCH_TEST, SLOT_POSITION, SLOT_STABILIZED, slot_pwm)
from ulog_checks import read_log
from vehicle import AUTO_LAND, AUTO_LOITER, MAIN_AUTO, MAIN_OFFBOARD, MAIN_POSCTL, MAIN_STABILIZED

pytestmark = [pytest.mark.sih, pytest.mark.flight, pytest.mark.fully_actuated]

HT_MAX = 0.5
TILT_MAX_DEG = 5.0

HT_PARAMS = {**FLIGHT_PARAMS, "DTRG_HT_EN": 1, "DTRG_HT_MAX": HT_MAX, "DTRG_HT_R_MAX": TILT_MAX_DEG,
             "DTRG_HT_P_MAX": TILT_MAX_DEG}

MOVE_NORTH = 5.0

# Holding a tilt with horizontal thrust, the true attitude is ~2 deg off the estimate, so
# commanded tilts are checked tightly against the estimate and loosely against the truth.
EST_TILT_TOL_DEG = 2.0
TRUE_TILT_TOL_DEG = 3.5


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


def offboard_move(vehicle, north, east):
    """Hold in Offboard at the current position, then move by ``north``/``east``. Returns (start, end) PX4 times."""
    x, y, z, yaw = offboard_hold(vehicle)

    start = vehicle.boot_time()
    vehicle.set_offboard_position(x + north, y + east, z, yaw)
    vehicle.wait_position(x + north, y + east, z, tolerance=0.3, timeout=30)
    vehicle.hold(3.0)
    return start, vehicle.boot_time()


def offboard_move_north(vehicle, distance):
    """Hold in Offboard at the current position, then move ``distance`` north. Returns (start, end) PX4 times."""
    return offboard_move(vehicle, distance, 0.0)


def test_take_off_hold_and_land(sitl):
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


def test_ht_moves_the_vehicle_level(sitl):
    vehicle, px4 = fly(sitl)

    vehicle.set_rc(CH_HT_MODE, PWM_MAX)
    vehicle.hold(2.0)
    start, end = offboard_move_north(vehicle, MOVE_NORTH)

    tr = truth(read_log(px4), start, end)
    moved = tr.last(1.0).x.mean() - tr.x[0]
    assert moved == pytest.approx(MOVE_NORTH, abs=0.5)
    assert tr.tilt.max() < 3.0, f"tilted {tr.tilt.max():.1f} deg while moving with HT"


def test_without_ht_the_vehicle_tilts_to_move(sitl):
    # control case for test_ht_moves_the_vehicle_level
    vehicle, px4 = fly(sitl)

    start, end = offboard_move_north(vehicle, MOVE_NORTH)

    tr = truth(read_log(px4), start, end)
    moved = tr.last(1.0).x.mean() - tr.x[0]
    assert moved == pytest.approx(MOVE_NORTH, abs=0.5)
    # accelerating north is nose down
    assert tr.pitch.min() < -5.0, f"pitched only {tr.pitch.min():.1f} deg while moving without HT"


def force_shares(log, start, end, yaw):
    """Force [normalised thrust] from horizontal thrust and from tilting, per vehicle_attitude_setpoint sample.

    Returns {"forward": (ht, tilt), "right": (ht, tilt)}, along and across the heading ``yaw``.
    """
    sp = log.topic("vehicle_attitude_setpoint").between(start, end)
    assert len(sp), f"no attitude setpoint between {start:.1f} and {end:.1f} s"
    w, x, y, z = (sp[f"q_d[{i}]"] for i in range(4))
    tx, ty, tz = (sp[f"thrust_body[{i}]"] for i in range(3))
    # first two rows of the body to NED rotation
    r00, r01, r02 = 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)
    r10, r11, r12 = 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)
    ht_n, ht_e = r00 * tx + r01 * ty, r10 * tx + r11 * ty
    tilt_n, tilt_e = r02 * tz, r12 * tz
    c, s = math.cos(yaw), math.sin(yaw)
    return {
        "forward": (c * ht_n + s * ht_e, c * tilt_n + s * tilt_e),
        "right": (-s * ht_n + c * ht_e, -s * tilt_n + c * tilt_e),
    }


# (DTRG_HT_MASK, DTRG_HT_SPLIT): the HT axes, X for masks 0 and 1 and Y for masks 0 and 2, move
# by `split` of horizontal thrust; the other axis by tilting only
SPLIT_FLIGHTS = [(0, 0.25), (0, 0.75), (1, 0.25), (2, 0.75)]
SPLIT_MOVE = 4.0  # forward and right, each


@pytest.mark.parametrize("mask, split", SPLIT_FLIGHTS, ids=[f"mask{m}-split{s}" for m, s in SPLIT_FLIGHTS])
def test_offboard_split_divides_thrust(sitl, mask, split):
    # DTRG_HT_SPLIT in Position/Offboard: of the force the position controller asks for while
    # moving diagonally (forward and right of the heading), horizontal thrust gives `split` on an
    # HT axis and nothing on the other, and tilting the rest. Checked on the published setpoint,
    # so the allocator's gain loss on X/Y (6.1 Open Issues) does not enter.
    vehicle, px4 = fly(sitl, params={"DTRG_HT_MASK": mask, "DTRG_HT_SPLIT_EN": 1, "DTRG_HT_SPLIT": split})

    vehicle.set_rc(CH_HT_MODE, PWM_MAX)
    vehicle.hold(2.0)
    yaw = vehicle.yaw()
    north = SPLIT_MOVE * (math.cos(yaw) - math.sin(yaw))
    east = SPLIT_MOVE * (math.sin(yaw) + math.cos(yaw))
    start, end = offboard_move(vehicle, north, east)

    log = read_log(px4)
    tr = truth(log, start, end)
    # along the diagonal: across it, and on each axis, the true position settles up to ~0.5 m
    # short (the HT gain loss, 6.1 Open Issues), which is not what this test is about
    settled = tr.last(1.0)
    along = ((settled.x.mean() - tr.x[0]) * north + (settled.y.mean() - tr.y[0]) * east) / math.hypot(north, east)
    assert along == pytest.approx(math.hypot(north, east), abs=0.6)

    expected = {"forward": split if mask != 2 else 0.0, "right": split if mask != 1 else 0.0}

    for axis, (ht, tilt) in force_shares(log, start, end, yaw).items():
        pushing = abs(ht + tilt) > 0.02
        assert pushing.sum() > 20, f"the position controller hardly asked for any {axis} force"
        share = np.median(ht[pushing] / (ht + tilt)[pushing])
        assert share == pytest.approx(expected[axis], abs=0.05), \
            f"horizontal thrust gave {share:.2f} of the {axis} force, expected {expected[axis]}"


def hold_sending_tilt(vehicle, roll, pitch, seconds):
    """Offboard HT tilt: stream DEBUG_FLOAT_ARRAY [roll, pitch] (rad) at 10 Hz for ``seconds``."""
    for _ in range(int(seconds * 10)):
        vehicle.send_debug_float_array([roll, pitch])
        vehicle.hold(0.1)



@pytest.mark.parametrize("channel, axis, other", [
    (CH_HT_ROLL, "roll", "pitch"),
    (CH_HT_PITCH, "pitch", "roll"),
], ids=["roll", "pitch"])
def test_aux_tilt_reaches_the_limit(sitl, channel, axis, other):
    # the attitude half of the aux tilt check, independent of the horizontal thrust gain loss. An
    # aux channel up tilts positively on
    # both axes: roll right and pitch nose up, here as in Stabilized (test_horizontal_thrust.py).
    # The aux channels set the tilt directly, so pitch goes the opposite way to the pitch stick.
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
    assert estimated[axis].mean() == pytest.approx(TILT_MAX_DEG, abs=EST_TILT_TOL_DEG)
    assert abs(getattr(tr, other)).max() < 3.0
    assert angle == pytest.approx(TILT_MAX_DEG, abs=TRUE_TILT_TOL_DEG)


def test_offboard_tilt_setpoint_in_hover(sitl):
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
    # assert horizontal_error(log, start, end) < 0.5
    assert vehicle.main_mode() == MAIN_OFFBOARD


def test_offboard_tilt_is_limited(sitl):
    # DEBUG_FLOAT_ARRAY is accepted from any MAVLink source, so its tilt is limited to
    # DTRG_HT_R_MAX, and a NaN levels the vehicle
    vehicle, px4 = fly(sitl)

    vehicle.set_rc(CH_HT_MODE, PWM_MAX)
    offboard_hold(vehicle)
    start = vehicle.boot_time()
    hold_sending_tilt(vehicle, math.radians(2 * TILT_MAX_DEG), 0.0, 5.0)
    tilted = vehicle.boot_time()
    hold_sending_tilt(vehicle, math.nan, math.nan, 5.0)
    end = vehicle.boot_time()

    log = read_log(px4)
    tr = truth(log, start, tilted)
    estimated_roll, _ = estimated_attitude(log, tilted - 2.0, tilted)
    assert estimated_roll.mean() == pytest.approx(TILT_MAX_DEG, abs=EST_TILT_TOL_DEG)
    assert tr.roll.max() < TILT_MAX_DEG + TRUE_TILT_TOL_DEG, \
        f"rolled {tr.roll.max():.1f} deg, limit {TILT_MAX_DEG} deg"

    # even a plain HT hover drifts ~2 deg in truth with a level estimate, so level is judged
    # tightly on the estimate and loosely on the truth, like the commanded tilt above
    level = truth(log, tilted, end).last(2.0)
    estimated_roll, estimated_pitch = estimated_attitude(log, end - 2.0, end)
    estimated_tilt = max(abs(estimated_roll).max(), abs(estimated_pitch).max())
    assert estimated_tilt < EST_TILT_TOL_DEG, f"estimate still tilted {estimated_tilt:.1f} deg after a NaN tilt setpoint"
    assert level.tilt.max() < TRUE_TILT_TOL_DEG, f"still tilted {level.tilt.max():.1f} deg after a NaN tilt setpoint"
    assert vehicle.main_mode() == MAIN_OFFBOARD


def test_toggling_ht_in_hover_is_smooth(sitl):
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


def test_stabilized_ht_stick_moves_the_vehicle_level(sitl):
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


def test_bench_test_rejected_in_flight(sitl):
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
