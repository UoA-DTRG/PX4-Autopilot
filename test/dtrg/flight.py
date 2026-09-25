"""Helpers for the flight tests (tier 3): take off, and read back how the vehicle really moved.

The assertions use SIH's ground truth (``*_groundtruth`` topics in the ULog), not
the estimator, so they check what the vehicle did rather than what PX4 believed.
Phases of a test are marked with ``Vehicle.boot_time()``, which is on the same
clock as the ULog timestamps.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ulog_checks import Log
from vehicle import AUTO_LOITER, AUTO_TAKEOFF, MAIN_AUTO, Vehicle

TAKEOFF_ALT = 3.0

# parameters of every flight test
FLIGHT_PARAMS = {
    "MIS_TAKEOFF_ALT": TAKEOFF_ALT,
}


def take_off(vehicle: Vehicle, altitude: float = TAKEOFF_ALT, timeout: float = 40.0, settle: float = 3.0) -> None:
    """Take off in Takeoff mode and return once the vehicle holds at ``altitude`` (in Hold).

    With RC streaming, wait for the RC mode slot to be applied first (``wait_mode``), or it
    can override Takeoff. ``settle`` lets the climb overshoot die out before the test starts.
    """
    result = vehicle.set_mode(MAIN_AUTO, AUTO_TAKEOFF)
    assert result.accepted, f"Takeoff mode rejected: {result}"
    vehicle.wait_until(lambda: vehicle.mode() == (MAIN_AUTO, AUTO_TAKEOFF), 5.0, f"Takeoff mode (now {vehicle.mode()})")
    result = vehicle.arm()
    assert result.accepted, f"arming rejected: {result}; texts: {vehicle.texts()[-5:]}"

    vehicle.wait_until(lambda: vehicle.mode() == (MAIN_AUTO, AUTO_LOITER) and vehicle.position()[2] < -(altitude - 0.3),
                       timeout, f"takeoff to {altitude} m and Hold (mode {vehicle.mode()}, z {vehicle.position()[2]:.2f})")
    vehicle.hold(settle)


@dataclass
class Truth:
    """Ground truth of one time window: time [s], position NED [m], roll/pitch/yaw [deg]."""

    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    t_att: np.ndarray
    roll: np.ndarray
    pitch: np.ndarray
    yaw: np.ndarray

    @property
    def tilt(self) -> np.ndarray:
        """max(|roll|, |pitch|) per sample [deg]."""
        return np.maximum(np.abs(self.roll), np.abs(self.pitch))

    def horizontal_drift(self) -> float:
        """Largest horizontal distance from the first sample of the window [m]."""
        return float(np.max(np.hypot(self.x - self.x[0], self.y - self.y[0])))

    def last(self, seconds: float) -> Truth:
        return truth_window(self, self.t[-1] - seconds, self.t[-1])


def euler_deg(q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    w, x, y, z = q.T
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    yaw = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw


def truth(log: Log, start: float, end: float) -> Truth:
    """SIH ground truth between PX4 times ``start`` and ``end`` [s]."""
    pos = log.topic("vehicle_local_position_groundtruth").between(start, end)
    att = log.topic("vehicle_attitude_groundtruth").between(start, end)
    assert len(pos) and len(att), f"no ground truth logged between {start:.1f} and {end:.1f} s"

    roll, pitch, yaw = euler_deg(np.stack([att[f"q[{i}]"] for i in range(4)], axis=1))
    return Truth(pos.t, pos["x"], pos["y"], pos["z"], att.t, roll, pitch, yaw)


def estimated_attitude(log: Log, start: float, end: float) -> tuple[np.ndarray, np.ndarray]:
    """Roll and pitch [deg] as estimated by PX4 (what the attitude controller tracks)."""
    att = log.topic("vehicle_attitude").between(start, end)
    assert len(att), f"no attitude estimate between {start:.1f} and {end:.1f} s"
    roll, pitch, _ = euler_deg(np.stack([att[f"q[{i}]"] for i in range(4)], axis=1))
    return roll, pitch


def altitude_error(log: Log, start: float, end: float) -> np.ndarray:
    """Estimated altitude minus the altitude setpoint [m] between ``start`` and ``end``.

    Altitude hold is judged on this, not on the ground truth: the estimate itself drifts
    0.3-0.5 m from the truth with SIH's simulated baro and GPS, and the controller can only
    hold what it estimates.
    """
    pos = log.topic("vehicle_local_position").between(start, end)
    sp = log.topic("vehicle_local_position_setpoint").between(start - 1.0, end)
    valid = np.isfinite(sp["z"])
    assert len(pos) and valid.any(), f"no altitude estimate or setpoint between {start:.1f} and {end:.1f} s"
    return -(pos["z"] - np.interp(pos.t, sp.t[valid], sp["z"][valid]))


def horizontal_error(log: Log, start: float, end: float) -> float:
    """Largest horizontal distance [m] of the estimated position from its setpoint.

    Position hold is judged on this rather than the ground truth for the same reason as
    altitude: the simulated GPS makes the estimate wander from the truth.
    """
    pos = log.topic("vehicle_local_position").between(start, end)
    sp = log.topic("vehicle_local_position_setpoint").between(start - 1.0, end)
    valid = np.isfinite(sp["x"]) & np.isfinite(sp["y"])
    assert len(pos) and valid.any(), f"no position estimate or setpoint between {start:.1f} and {end:.1f} s"
    x_sp = np.interp(pos.t, sp.t[valid], sp["x"][valid])
    y_sp = np.interp(pos.t, sp.t[valid], sp["y"][valid])
    return float(np.max(np.hypot(pos["x"] - x_sp, pos["y"] - y_sp)))


def truth_window(tr: Truth, start: float, end: float) -> Truth:
    p = (tr.t >= start) & (tr.t <= end)
    a = (tr.t_att >= start) & (tr.t_att <= end)
    return Truth(tr.t[p], tr.x[p], tr.y[p], tr.z[p], tr.t_att[a], tr.roll[a], tr.pitch[a], tr.yaw[a])
