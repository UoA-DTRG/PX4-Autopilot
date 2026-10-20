"""Tier 3: horizontal thrust (HT) in flight, on the fully actuated planarOcto in SIH.

The vehicle really flies here: SIH simulates the eight tilted rotors from the
airframe's CA_ROTOR* geometry. See Tools/dtrg/DTRG_Automated_Testing.md.

Each test checks the logic (modes, and the setpoints PX4 publishes) with plain
asserts, which fail the test straight away, and how closely the vehicle followed
those setpoints (SIH's ground truth, or the estimate against its setpoint) with
``tracking.check()``. A flight that only misses its tracking checks is flown again,
up to flight.FLIGHT_ATTEMPTS times (the ``reflight`` fixture).

Run: python3 -m pytest test/dtrg -m flight -v
"""

import math

import numpy as np
import pytest

from flight import (FLIGHT_PARAMS, altitude_error, attitude_setpoint, estimated_attitude, horizontal_error, nav_states,
                    position_setpoint_at, take_off, truth)
from rc_layout import (CH_FLTMODE, CH_HT_MODE, CH_HT_PITCH, CH_HT_ROLL, CH_PITCH, CH_THROTTLE, PWM_CENTRE, PWM_MAX,
                       PWM_MIN, SLOT_BENCH_TEST, SLOT_POSITION, SLOT_STABILIZED, slot_pwm)
from ulog_checks import NAV_STATE_AUTO_LOITER, NAV_STATE_OFFBOARD, NAV_STATE_STAB, read_log
from vehicle import AUTO_LAND, AUTO_LOITER, MAIN_AUTO, MAIN_OFFBOARD, MAIN_POSCTL, MAIN_STABILIZED

pytestmark = [pytest.mark.sih, pytest.mark.flight, pytest.mark.fully_actuated]

HT_MAX = 0.5
TILT_MAX_DEG = 5.0

HT_PARAMS = {**FLIGHT_PARAMS, "DTRG_HT_EN": 1, "DTRG_HT_MAX": HT_MAX, "DTRG_HT_R_MAX": TILT_MAX_DEG,
             "DTRG_HT_P_MAX": TILT_MAX_DEG}

MOVE_NORTH = 5.0
# Offboard holds the estimate only to ~0.35 m of its setpoint, and the estimate wanders ~0.15 m
# from the truth, so the true distance of a move is checked loosely, like horizontal_error.
MOVE_TOL = 1.0

# Holding a tilt with horizontal thrust, the true attitude is ~2 deg off the estimate, so
# commanded tilts are checked tightly against the estimate and loosely against the truth.
EST_TILT_TOL_DEG = 2.0
TRUE_TILT_TOL_DEG = 3.5

# Logic: values the code under test sets exactly, read back from the published setpoints
SP_TILT_TOL_DEG = 0.05
SP_POSITION_TOL = 0.01
SP_THRUST_TOL = 1e-4
# Stabilized HT is stick * DTRG_HT_MAX, and full stick is not exactly 1.0 on every host (6.1 Open Issues)
SP_STICK_THRUST_TOL = 0.01
# time for an RC switch to reach the setpoints, left out of the logic checks after it
RC_LATENCY = 0.5


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


def offboard_move(vehicle, tracking, north, east):
    """Hold in Offboard at the current position, then move by ``north``/``east``.

    Returns (start, end, target): PX4 times of the move and the (x, y, z) setpoint sent.
    Not getting there in time is a tracking failure.
    """
    x, y, z, yaw = offboard_hold(vehicle)
    target = (x + north, y + east, z)

    start = vehicle.boot_time()
    vehicle.set_offboard_position(*target, yaw)
    with tracking.waiting():
        vehicle.wait_position(*target, tolerance=0.3, timeout=30)
    vehicle.hold(3.0)
    return start, vehicle.boot_time(), target


def offboard_move_north(vehicle, tracking, distance):
    """Hold in Offboard at the current position, then move ``distance`` north. Returns (start, end, target)."""
    return offboard_move(vehicle, tracking, distance, 0.0)


def assert_position_setpoint(log, t, target):
    """Logic: the position setpoint in force at ``t`` is the one sent over MAVLink."""
    setpoint = position_setpoint_at(log, t)
    assert setpoint == pytest.approx(target, abs=SP_POSITION_TOL), \
        f"position setpoint {setpoint}, sent {target}"


def test_take_off_hold_and_land(sitl, reflight):
    def flight(tracking):
        vehicle, px4 = fly(sitl)

        start = vehicle.boot_time()
        vehicle.hold(10.0)
        end = vehicle.boot_time()
        assert vehicle.mode() == (MAIN_AUTO, AUTO_LOITER)

        assert vehicle.set_mode(MAIN_AUTO, AUTO_LAND).accepted
        vehicle.wait_armed(False, timeout=30)

        log = read_log(px4)
        assert nav_states(log, start, end) == {NAV_STATE_AUTO_LOITER}
        # the HT switch is off: the standard attitude setpoint, without horizontal thrust
        ht = attitude_setpoint(log, start, end).horizontal_thrust.max()
        assert ht < SP_THRUST_TOL, f"horizontal thrust {ht:.3f} in the setpoint with the HT switch off"

        error = abs(altitude_error(log, start, end)).max()
        tracking.check(error < 0.5, f"altitude off its setpoint by up to {error:.2f} m during the hold")
        error = horizontal_error(log, start, end)
        tracking.check(error < 1.0, f"position off its setpoint by up to {error:.2f} m during the hold")

    reflight(flight)


def test_ht_moves_the_vehicle_level(sitl, reflight):
    def flight(tracking):
        vehicle, px4 = fly(sitl)

        vehicle.set_rc(CH_HT_MODE, PWM_MAX)
        vehicle.hold(2.0)
        start, end, target = offboard_move_north(vehicle, tracking, MOVE_NORTH)

        log = read_log(px4)
        assert nav_states(log, start, end) == {NAV_STATE_OFFBOARD}
        assert_position_setpoint(log, end, target)
        # HT on, no split, no offboard tilt: the setpoint stays level and horizontal thrust,
        # limited to DTRG_HT_MAX, pushes north (a heading of 0 makes "forward" north)
        sp = attitude_setpoint(log, start, end)
        assert sp.tilt.max() < SP_TILT_TOL_DEG, f"attitude setpoint tilted {sp.tilt.max():.2f} deg with HT"
        assert sp.horizontal_thrust.max() <= HT_MAX + SP_THRUST_TOL, \
            f"horizontal thrust {sp.horizontal_thrust.max():.3f} in the setpoint, limit {HT_MAX}"
        ht_north, _ = force_shares(log, start, end, 0.0)["forward"]
        assert ht_north.max() > 0.05, f"horizontal thrust pushed north by at most {ht_north.max():.3f}"

        tr = truth(log, start, end)
        moved = tr.last(1.0).x.mean() - tr.x[0]
        tracking.check(abs(moved - MOVE_NORTH) <= 0.5, f"moved {moved:.2f} m north, expected {MOVE_NORTH} +-0.5")
        tracking.check(tr.tilt.max() < 3.0, f"tilted {tr.tilt.max():.1f} deg while moving with HT")

    reflight(flight)


def test_without_ht_the_vehicle_tilts_to_move(sitl, reflight):
    # control case for test_ht_moves_the_vehicle_level
    def flight(tracking):
        vehicle, px4 = fly(sitl)

        start, end, target = offboard_move_north(vehicle, tracking, MOVE_NORTH)

        log = read_log(px4)
        assert nav_states(log, start, end) == {NAV_STATE_OFFBOARD}
        assert_position_setpoint(log, end, target)
        # HT off: no horizontal thrust, the position controller tilts to move
        sp = attitude_setpoint(log, start, end)
        assert sp.horizontal_thrust.max() < SP_THRUST_TOL, \
            f"horizontal thrust {sp.horizontal_thrust.max():.3f} in the setpoint with HT off"
        assert sp.tilt.max() > 5.0, f"attitude setpoint tilted only {sp.tilt.max():.1f} deg to move without HT"

        tr = truth(log, start, end)
        moved = tr.last(1.0).x.mean() - tr.x[0]
        tracking.check(abs(moved - MOVE_NORTH) <= 0.5, f"moved {moved:.2f} m north, expected {MOVE_NORTH} +-0.5")
        # accelerating north is nose down
        tracking.check(tr.pitch.min() < -5.0, f"pitched only {tr.pitch.min():.1f} deg while moving without HT")

    reflight(flight)


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
def test_offboard_split_divides_thrust(sitl, reflight, mask, split):
    # DTRG_HT_SPLIT in Position/Offboard: of the force the position controller asks for while
    # moving diagonally (forward and right of the heading), horizontal thrust gives `split` on an
    # HT axis and nothing on the other, and tilting the rest. Checked on the published setpoint,
    # so the allocator's gain loss on X/Y (6.1 Open Issues) does not enter.
    def flight(tracking):
        vehicle, px4 = fly(sitl, params={"DTRG_HT_MASK": mask, "DTRG_HT_SPLIT_EN": 1, "DTRG_HT_SPLIT": split})

        vehicle.set_rc(CH_HT_MODE, PWM_MAX)
        vehicle.hold(2.0)
        yaw = vehicle.yaw()
        north = SPLIT_MOVE * (math.cos(yaw) - math.sin(yaw))
        east = SPLIT_MOVE * (math.sin(yaw) + math.cos(yaw))
        start, end, target = offboard_move(vehicle, tracking, north, east)

        log = read_log(px4)
        assert nav_states(log, start, end) == {NAV_STATE_OFFBOARD}
        assert_position_setpoint(log, end, target)

        expected = {"forward": split if mask != 2 else 0.0, "right": split if mask != 1 else 0.0}

        for axis, (ht, tilt) in force_shares(log, start, end, yaw).items():
            pushing = abs(ht + tilt) > 0.02
            assert pushing.sum() > 20, f"the position controller hardly asked for any {axis} force"
            share = np.median(ht[pushing] / (ht + tilt)[pushing])
            assert share == pytest.approx(expected[axis], abs=0.05), \
                f"horizontal thrust gave {share:.2f} of the {axis} force, expected {expected[axis]}"

        tr = truth(log, start, end)
        # along the diagonal: across it, and on each axis, the true position settles up to ~0.5 m
        # short (the HT gain loss, 6.1 Open Issues), which is not what this test is about
        settled = tr.last(1.0)
        along = ((settled.x.mean() - tr.x[0]) * north + (settled.y.mean() - tr.y[0]) * east) / math.hypot(north, east)
        diagonal = math.hypot(north, east)
        tracking.check(abs(along - diagonal) <= 0.6,
                       f"moved {along:.2f} m along the diagonal, expected {diagonal:.2f} +-0.6")

    reflight(flight)


def hold_sending_tilt(vehicle, roll, pitch, seconds):
    """Offboard HT tilt: stream DEBUG_FLOAT_ARRAY [roll, pitch] (rad) at 10 Hz for ``seconds``."""
    for _ in range(int(seconds * 10)):
        vehicle.send_debug_float_array([roll, pitch])
        vehicle.hold(0.1)


@pytest.mark.parametrize("channel, axis, other", [
    (CH_HT_ROLL, "roll", "pitch"),
    (CH_HT_PITCH, "pitch", "roll"),
], ids=["roll", "pitch"])
def test_aux_tilt_reaches_the_limit(sitl, reflight, channel, axis, other):
    # the attitude half of the aux tilt check, independent of the horizontal thrust gain loss. An
    # aux channel up tilts positively on
    # both axes: roll right and pitch nose up, here as in Stabilized (test_horizontal_thrust.py).
    # The aux channels set the tilt directly, so pitch goes the opposite way to the pitch stick.
    def flight(tracking):
        vehicle, px4 = fly(sitl)

        vehicle.set_rc(CH_HT_MODE, PWM_MAX)
        vehicle.hold(2.0)
        vehicle.set_rc(channel, PWM_MAX)
        vehicle.hold(4.0)
        start = vehicle.boot_time()
        vehicle.hold(2.0)
        end = vehicle.boot_time()

        log = read_log(px4)
        assert nav_states(log, start, end) == {NAV_STATE_AUTO_LOITER}
        # the aux channel at full sets the tilt setpoint to the limit, the other axis stays level
        sp = attitude_setpoint(log, start, end)
        assert getattr(sp, axis) == pytest.approx(TILT_MAX_DEG, abs=SP_TILT_TOL_DEG), \
            f"{axis} setpoint {getattr(sp, axis).min():.2f}..{getattr(sp, axis).max():.2f} deg, expected {TILT_MAX_DEG}"
        assert abs(getattr(sp, other)).max() < SP_TILT_TOL_DEG, \
            f"{other} setpoint up to {abs(getattr(sp, other)).max():.2f} deg, expected level"

        tr = truth(log, start, end)
        estimated = dict(zip(("roll", "pitch"), estimated_attitude(log, start, end)))[axis].mean()
        angle = getattr(tr, axis).mean()
        tracking.check(abs(estimated - TILT_MAX_DEG) <= EST_TILT_TOL_DEG,
                       f"estimated {axis} {estimated:.1f} deg, setpoint {TILT_MAX_DEG} +-{EST_TILT_TOL_DEG}")
        tracking.check(abs(getattr(tr, other)).max() < 3.0, f"{other} up to {abs(getattr(tr, other)).max():.1f} deg")
        tracking.check(abs(angle - TILT_MAX_DEG) <= TRUE_TILT_TOL_DEG,
                       f"true {axis} {angle:.1f} deg, setpoint {TILT_MAX_DEG} +-{TRUE_TILT_TOL_DEG}")

    reflight(flight)


# inside DTRG_HT_R_MAX, so it reaches the setpoint unchanged (test_offboard_tilt_is_limited covers the limit)
OFFBOARD_ROLL_DEG = 0.8 * TILT_MAX_DEG


def test_offboard_tilt_setpoint_in_hover(sitl, reflight):
    def flight(tracking):
        vehicle, px4 = fly(sitl)

        vehicle.set_rc(CH_HT_MODE, PWM_MAX)
        x, y, z, _ = offboard_hold(vehicle)
        start = vehicle.boot_time()
        hold_sending_tilt(vehicle, math.radians(OFFBOARD_ROLL_DEG), 0.0, 8.0)
        end = vehicle.boot_time()
        assert vehicle.main_mode() == MAIN_OFFBOARD

        log = read_log(px4)
        assert nav_states(log, start, end) == {NAV_STATE_OFFBOARD}
        assert_position_setpoint(log, end, (x, y, z))
        sp = attitude_setpoint(log, end - 3.0, end)
        assert sp.roll == pytest.approx(OFFBOARD_ROLL_DEG, abs=SP_TILT_TOL_DEG), \
            f"roll setpoint {sp.roll.min():.2f}..{sp.roll.max():.2f} deg, sent {OFFBOARD_ROLL_DEG}"
        assert abs(sp.pitch).max() < SP_TILT_TOL_DEG, f"pitch setpoint up to {abs(sp.pitch).max():.2f} deg, sent 0"

        settled = truth(log, start, end).last(3.0)
        estimated_roll, _ = estimated_attitude(log, end - 3.0, end)
        tracking.check(abs(estimated_roll.mean() - OFFBOARD_ROLL_DEG) <= EST_TILT_TOL_DEG,
                       f"estimated roll {estimated_roll.mean():.1f} deg, "
                       f"setpoint {OFFBOARD_ROLL_DEG} +-{EST_TILT_TOL_DEG}")
        tracking.check(abs(settled.roll.mean() - OFFBOARD_ROLL_DEG) <= TRUE_TILT_TOL_DEG,
                       f"true roll {settled.roll.mean():.1f} deg, setpoint {OFFBOARD_ROLL_DEG} +-{TRUE_TILT_TOL_DEG}")

    reflight(flight)


def test_offboard_tilt_is_limited(sitl, reflight):
    # DEBUG_FLOAT_ARRAY is accepted from any MAVLink source, so its tilt is limited to
    # DTRG_HT_R_MAX, and a NaN levels the vehicle
    def flight(tracking):
        vehicle, px4 = fly(sitl)

        vehicle.set_rc(CH_HT_MODE, PWM_MAX)
        x, y, z, _ = offboard_hold(vehicle)
        start = vehicle.boot_time()
        hold_sending_tilt(vehicle, math.radians(2 * TILT_MAX_DEG), 0.0, 5.0)
        tilted = vehicle.boot_time()
        hold_sending_tilt(vehicle, math.nan, math.nan, 5.0)
        end = vehicle.boot_time()
        assert vehicle.main_mode() == MAIN_OFFBOARD

        log = read_log(px4)
        assert nav_states(log, start, end) == {NAV_STATE_OFFBOARD}
        assert_position_setpoint(log, end, (x, y, z))
        # the doubled roll is cut to the limit in the setpoint, and the NaN levels it
        sp = attitude_setpoint(log, start, tilted)
        assert sp.roll.max() < TILT_MAX_DEG + SP_TILT_TOL_DEG, \
            f"roll setpoint up to {sp.roll.max():.2f} deg, limit {TILT_MAX_DEG} deg"
        sp = attitude_setpoint(log, tilted - 2.0, tilted)
        assert sp.roll == pytest.approx(TILT_MAX_DEG, abs=SP_TILT_TOL_DEG), \
            f"roll setpoint {sp.roll.min():.2f}..{sp.roll.max():.2f} deg, expected the limit {TILT_MAX_DEG}"
        sp = attitude_setpoint(log, end - 2.0, end)
        assert sp.tilt.max() < SP_TILT_TOL_DEG, f"attitude setpoint still tilted {sp.tilt.max():.2f} deg after a NaN"

        tr = truth(log, start, tilted)
        estimated_roll, _ = estimated_attitude(log, tilted - 2.0, tilted)
        tracking.check(abs(estimated_roll.mean() - TILT_MAX_DEG) <= EST_TILT_TOL_DEG,
                       f"estimated roll {estimated_roll.mean():.1f} deg, limit {TILT_MAX_DEG} +-{EST_TILT_TOL_DEG}")
        tracking.check(tr.roll.max() < TILT_MAX_DEG + TRUE_TILT_TOL_DEG,
                       f"rolled {tr.roll.max():.1f} deg, limit {TILT_MAX_DEG} deg")

        # even a plain HT hover drifts ~2 deg in truth with a level estimate, so level is judged
        # tightly on the estimate and loosely on the truth, like the commanded tilt above
        level = truth(log, tilted, end).last(2.0)
        estimated_roll, estimated_pitch = estimated_attitude(log, end - 2.0, end)
        estimated_tilt = max(abs(estimated_roll).max(), abs(estimated_pitch).max())
        tracking.check(estimated_tilt < EST_TILT_TOL_DEG,
                       f"estimate still tilted {estimated_tilt:.1f} deg after a NaN tilt setpoint")
        tracking.check(level.tilt.max() < TRUE_TILT_TOL_DEG,
                       f"still tilted {level.tilt.max():.1f} deg after a NaN tilt setpoint")

    reflight(flight)


def test_toggling_ht_in_hover_is_smooth(sitl, reflight):
    def flight(tracking):
        vehicle, px4 = fly(sitl)

        # the same hover without toggling, as the reference for the noise of this run
        ref_start = vehicle.boot_time()
        vehicle.hold(8.0)
        start = vehicle.boot_time()

        # (switched on, switched off, next switch) PX4 times of each toggle
        toggles = []

        for _ in range(5):
            on = vehicle.boot_time()
            vehicle.set_rc(CH_HT_MODE, PWM_MAX)
            vehicle.hold(2.0)
            off = vehicle.boot_time()
            vehicle.set_rc(CH_HT_MODE, PWM_MIN)
            vehicle.hold(2.0)
            toggles.append((on, off, vehicle.boot_time()))

        end = vehicle.boot_time()

        log = read_log(px4)
        assert nav_states(log, ref_start, end) == {NAV_STATE_AUTO_LOITER}
        limit = log.topic("horizontal_thrust_limit")

        # every switch takes effect: HT on is level (aux channels centred) and publishes
        # horizontal_thrust_limit, HT off is the standard setpoint without horizontal thrust
        for i, (on, off, after) in enumerate(toggles, 1):
            sp = attitude_setpoint(log, on + RC_LATENCY, off)
            assert sp.tilt.max() < SP_TILT_TOL_DEG, f"toggle {i}: setpoint tilted {sp.tilt.max():.2f} deg with HT on"
            assert len(limit.between(on + RC_LATENCY, off)), f"toggle {i}: no horizontal_thrust_limit with HT on"
            sp = attitude_setpoint(log, off + RC_LATENCY, after)
            assert sp.horizontal_thrust.max() < SP_THRUST_TOL, \
                f"toggle {i}: horizontal thrust {sp.horizontal_thrust.max():.3f} in the setpoint with HT off"
            assert not len(limit.between(off + RC_LATENCY, after)), f"toggle {i}: horizontal_thrust_limit with HT off"

        ref = truth(log, ref_start, start)
        tr = truth(log, start, end)
        error = abs(altitude_error(log, start, end)).max()
        tracking.check(error < 0.7, f"altitude off its setpoint by up to {error:.2f} m while toggling")
        tracking.check(tr.tilt.max() < max(8.0, ref.tilt.max() + 2.0),
                       f"tilted {tr.tilt.max():.1f} deg while toggling, {ref.tilt.max():.1f} deg in the plain hover")
        error = horizontal_error(log, start, end)
        tracking.check(error < 1.0, f"position off its setpoint by up to {error:.2f} m while toggling")

    reflight(flight)


def test_stabilized_ht_stick_moves_the_vehicle_level(sitl, reflight):
    def flight(tracking):
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
        assert nav_states(log, start, end) == {NAV_STATE_STAB}
        # full pitch stick with HT: level, DTRG_HT_MAX forward and nothing sideways
        sp = attitude_setpoint(log, start + RC_LATENCY, end)
        assert sp.tilt.max() < SP_TILT_TOL_DEG, f"attitude setpoint tilted {sp.tilt.max():.2f} deg with HT"
        assert sp.horizontal_thrust.max() <= HT_MAX + SP_THRUST_TOL, \
            f"horizontal thrust {sp.horizontal_thrust.max():.3f} in the setpoint, limit {HT_MAX}"
        sp = attitude_setpoint(log, end - 1.0, end)
        assert sp.thrust[:, 0] == pytest.approx(HT_MAX, abs=SP_STICK_THRUST_TOL), \
            f"forward thrust setpoint {sp.thrust[:, 0].min():.3f}..{sp.thrust[:, 0].max():.3f}, expected {HT_MAX}"
        assert abs(sp.thrust[:, 1]).max() < SP_STICK_THRUST_TOL, \
            f"sideways thrust setpoint up to {abs(sp.thrust[:, 1]).max():.3f}, expected 0"

        tr = truth(log, start, end)
        velocity = log.topic("vehicle_local_position_groundtruth").between(start, end)
        yaw = math.radians(tr.yaw.mean())
        forward = velocity["vx"] * math.cos(yaw) + velocity["vy"] * math.sin(yaw)

        tracking.check(forward[-1] > 1.0, f"forward speed only {forward[-1]:.2f} m/s after 3 s of full HT stick")
        tracking.check(tr.tilt.max() < 3.0, f"tilted {tr.tilt.max():.1f} deg with HT")

    reflight(flight)


def test_bench_test_rejected_in_flight(sitl):
    # logic only (the bench test is refused and the vehicle keeps flying), so flown once
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
