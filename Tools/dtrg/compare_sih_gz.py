#!/usr/bin/env python3
"""Compare the planarOcto in SIH with the planarOcto in Gazebo.

Two parts:

1. Static: the vehicle model each simulator builds, from the files alone. The
   Gazebo model (Tools/simulation/gz/models/planar_octo*) against the parameters
   SIH uses (12013_dtrg_planar_octo + 12016_sihsim_planar_octo): mass, inertia,
   rotor geometry and spin, thrust curve, drag and damping.

2. Dynamic: the same profile flown on the same firmware with the same parameters,
   only the simulator differs. It is flown in Offboard, so no pilot input differs:

     take-off to 3 m -> hover 10 s -> yaw +90 deg -> 5 m north without HT ->
     HT on, 5 m south -> HT roll tilt 0.1 rad -> climb 2 m -> descend 2 m -> land

   How the vehicle moved comes from the ground truth topics, what the controller
   asked for from actuator_motors / vehicle_thrust_setpoint. Step responses are
   measured on the estimate, which is what the controller tracks.

Simulators (--sims):
  sih         12016_sihsim_planar_octo as committed
  sih_gzdamp  the same with SIH_KDW set to the angular damping the Gazebo model has
  gz          12014_gz_planar_octo, Gazebo headless

Results and discussion: Tools/dtrg/SIH_VS_GAZEBO.md.

Usage (Gazebo Harmonic installed, make px4_sitl_default built):

  python3 Tools/dtrg/compare_sih_gz.py --static
  python3 Tools/dtrg/compare_sih_gz.py --runs 2 --out /tmp/dtrg_sih_vs_gz

Do not run another PX4 or gz sim at the same time.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import signal
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "test" / "dtrg"))

import rc_layout  # noqa: E402
from airframes import AIRFRAMES, Airframe  # noqa: E402
from flight import FLIGHT_PARAMS, euler_deg, take_off  # noqa: E402
from px4_process import Px4Sitl  # noqa: E402
from rc_layout import CH_HT_MODE, PWM_MAX, PWM_MIN  # noqa: E402
from ulog_checks import Log  # noqa: E402
from vehicle import AUTO_LAND, MAIN_AUTO, MAIN_OFFBOARD, MAIN_STABILIZED, Vehicle  # noqa: E402

GZ_MODELS = REPO / "Tools" / "simulation" / "gz" / "models"
AIRFRAME_DIRS = [REPO / "ROMFS" / "px4fmu_common" / "init.d" / "airframes",
                 REPO / "ROMFS" / "px4fmu_common" / "init.d-posix" / "airframes"]
SIH_PARAMS_C = REPO / "src" / "modules" / "simulation" / "simulator_sih" / "sih_params.c"

GZ_PLANAR_OCTO = Airframe(name="gz_planar_octo", autostart=12014, num_motors=8, fully_actuated=True)

NUM_MOTORS = 8
GRAVITY = 9.80665
STEP_NORTH = 5.0
STEP_UP = 2.0
HT_ROLL_RAD = 0.1

# same as the tier 3 flight tests (test/dtrg/test_flight_ht.py), plus what conftest.py always sets
PARAMS = {
    "COM_DISARM_PRFLT": -1,
    "SDLOG_MODE": 1,
    **rc_layout.RC_PARAMS,
    **FLIGHT_PARAMS,
    "DTRG_HT_EN": 1,
    "DTRG_HT_MAX": 0.5,
    "DTRG_HT_R_MAX": 10.0,
    "DTRG_HT_P_MAX": 10.0,
}


# --- static comparison -----------------------------------------------------------------------


def airframe_params(*names: str) -> dict:
    """``param set[-default] NAME VALUE`` of the given airframe files, later files win."""
    params = {}
    for name in names:
        path = next(d / name for d in AIRFRAME_DIRS if (d / name).exists())
        for m in re.finditer(r"^\s*param set(?:-default)? (\w+) (\S+)", path.read_text(), re.M):
            try:
                params[m.group(1)] = float(m.group(2))
            except ValueError:
                pass
    return params


def sih_default(name: str) -> float:
    m = re.search(rf"PARAM_DEFINE_\w+\({name},\s*([-\d.e]+)f?\)", SIH_PARAMS_C.read_text())
    return float(m.group(1))


def flu_to_frd(v) -> np.ndarray:
    return np.array([v[0], -v[1], -v[2]])


def gz_model() -> dict:
    """Mass, inertia, rotors and motor constants of the Gazebo planarOcto, in PX4 body FRD."""
    base = ET.parse(GZ_MODELS / "planar_octo_base" / "model.sdf").getroot().find(".//link[@name='base_link']")
    top = ET.parse(GZ_MODELS / "planar_octo" / "model.sdf").getroot()

    def inertial(link):
        i = link.find("inertial")
        m = float(i.find("mass").text)
        t = i.find("inertia")
        return m, np.array([[float(t.find(k).text) if t.find(k) is not None else 0.0 for k in row]
                            for row in (("ixx", "ixy", "ixz"), ("ixy", "iyy", "iyz"), ("ixz", "iyz", "izz"))])

    bodies = [(np.zeros(3), *inertial(base))]
    rotors = []
    plugins = {p.find("linkName").text: p for p in top.iter("plugin") if p.find("linkName") is not None}

    for link in top.find("model").findall("link"):
        pose = [float(v) for v in link.find("pose").text.split()]
        joint = top.find(f".//joint[@name='{link.get('name')}_joint']")
        axis = [float(v) for v in joint.find("axis/xyz").text.split()]
        p = plugins[link.get("name")]
        m, inertia = inertial(link)
        bodies.append((np.array(pose[:3]), m, inertia))
        rotors.append({
            "position": flu_to_frd(pose[:3]),
            "axis": flu_to_frd(axis),
            "ccw": p.find("turningDirection").text == "ccw",
            "actuator": int(p.find("actuator_number").text),
            "kf": float(p.find("motorConstant").text),
            "km": float(p.find("momentConstant").text),
            "w_max": float(p.find("maxRotVelocity").text),
            "tau_up": float(p.find("timeConstantUp").text),
            "tau_down": float(p.find("timeConstantDown").text),
            "drag": float(p.find("rotorDragCoefficient").text),
        })

    # composite inertia about the centre of mass (rotor inertia taken about its own frame, it is ~1e-5)
    mass = sum(m for _, m, _ in bodies)
    com = sum(m * r for r, m, _ in bodies) / mass
    inertia = np.zeros((3, 3))
    for r, m, i in bodies:
        d = r - com
        inertia += i + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))

    rotors.sort(key=lambda r: r["actuator"])
    return {"mass": mass, "com_flu": com, "inertia": inertia, "rotors": rotors}


def hover_command(ap: dict) -> float:
    """Thrust per rotor to hover level, relative to SIH_T_MAX: m g / (8 cos 31 deg T_max)."""
    return ap["SIH_MASS"] * GRAVITY / (NUM_MOTORS * abs(ap["CA_ROTOR0_AZ"]) * ap["SIH_T_MAX"])


def gz_hover_speed(gz: dict, ap: dict) -> float:
    """Rotor speed [rad/s] at which the Gazebo model hovers level."""
    return math.sqrt(hover_command(ap) * ap["SIH_T_MAX"] / gz["rotors"][0]["kf"])


def gz_rotor_drag(gz: dict, w: float, v_of_rotor) -> tuple[np.ndarray, np.ndarray]:
    """Force and moment of the Gazebo rotor drag at rotor speed ``w``, for air velocity ``v_of_rotor(position)``.

    gz-sim MulticopterMotorModel: F = -|w| rotorDragCoefficient v_perp on each rotor, v_perp being
    the velocity perpendicular to the rotor axis. The model has no other drag or damping.
    """
    force, moment = np.zeros(3), np.zeros(3)
    for r in gz["rotors"]:
        v = v_of_rotor(r["position"])
        f = -w * r["drag"] * (v - np.dot(v, r["axis"]) * r["axis"])
        force += f
        moment += np.cross(r["position"], f)
    return force, moment


def static_comparison() -> str:
    gz = gz_model()
    ap = airframe_params("12013_dtrg_planar_octo", "12016_sihsim_planar_octo", "12014_gz_planar_octo")
    kdw = ap.get("SIH_KDW", sih_default("SIH_KDW"))
    kdv = ap.get("SIH_KDV", sih_default("SIH_KDV"))
    t_max, fac = ap["SIH_T_MAX"], ap["SIH_THR_MDL_FAC"]
    r0 = gz["rotors"][0]
    w_min = ap["SIM_GZ_EC_MIN1"]
    w_max = ap["SIM_GZ_EC_MAX1"]

    rows = []

    def row(what, sih, gzv, note=""):
        rows.append(f"| {what} | {sih} | {gzv} | {note} |")

    row("mass [kg]", f"{ap['SIH_MASS']:.3f}", f"{gz['mass']:.3f}", "base 1.46 + 8 x 0.005 rotors")
    row("Ixx / Iyy / Izz [kg m^2]", f"{ap['SIH_IXX']:.4f} / {ap['SIH_IYY']:.4f} / {ap['SIH_IZZ']:.4f}",
        " / ".join(f"{gz['inertia'][i, i]:.4f}" for i in range(3)), "composite about the CoM")
    row("centre of mass vs CAD origin [mm]", "0 (CA_ROTOR positions used as is)",
        ", ".join(f"{1e3 * v:.1f}" for v in flu_to_frd(gz["com_flu"])), "FRD")

    pos_err = max(np.abs(r["position"] - [ap[f"CA_ROTOR{i}_P{a}"] for a in "XYZ"]).max()
                  for i, r in enumerate(gz["rotors"]))
    axis_err = max(np.abs(r["axis"] - [ap[f"CA_ROTOR{i}_A{a}"] for a in "XYZ"]).max()
                   for i, r in enumerate(gz["rotors"]))
    spin_ok = all((ap[f"CA_ROTOR{i}_KM"] > 0) == r["ccw"] for i, r in enumerate(gz["rotors"]))
    km_err = max(abs(abs(ap[f"CA_ROTOR{i}_KM"]) - r["km"]) for i, r in enumerate(gz["rotors"]))
    order_ok = [r["actuator"] for r in gz["rotors"]] == list(range(NUM_MOTORS))
    row("rotor n position (CA_ROTORn_P*) [m]", "CA_ROTORn_P*", f"max diff {pos_err:.1e}", "same")
    row("rotor n thrust axis (CA_ROTORn_A*)", "CA_ROTORn_A*", f"max diff {axis_err:.1e}", "same, 31 deg tilt")
    row("rotor n spin", "sign of CA_ROTORn_KM", "turningDirection", "same" if spin_ok else "DIFFERENT")
    row("motor n -> rotor n", "output n drives rotor n", "actuator_number n",
        "same" if order_ok else "DIFFERENT")
    row("drag torque / thrust [m]", f"{abs(ap['CA_ROTOR0_KM']):.5f}", f"{r0['km']:.5f}",
        "same" if km_err < 1e-6 else "DIFFERENT")
    row("max thrust per rotor [N]", f"{t_max:.3f}", f"{r0['kf'] * w_max ** 2:.3f}", "kF w_max^2")
    row("motor time constant [s]", f"{ap['SIH_T_TAU']:.3f} on thrust",
        f"{r0['tau_up']:.3f} up / {r0['tau_down']:.3f} down on rotor speed",
        "same lag for small changes around hover")

    # thrust curve. PX4 inverts THR_MDL_FAC for both, so compare thrust per thrust command c
    # (actuator_motors): output u solves fac u^2 + (1 - fac) u = c.
    def u_of(c):
        return (-(1 - fac) + math.sqrt((1 - fac) ** 2 + 4 * fac * c)) / (2 * fac)

    hover_c = hover_command(ap)
    curve = []
    for c in (0.0, ap.get("MPC_THR_MIN", 0.12), 0.25, 0.5, hover_c, 0.75, 1.0):
        u = u_of(c)
        gz_rel = r0["kf"] * (w_min + u * (w_max - w_min)) ** 2 / t_max
        curve.append(f"{c:.2f}: {c:.3f} / {gz_rel:.3f}")
    row("thrust / max thrust at command c (SIH / Gazebo)", "", "", "; ".join(curve))

    # the rotor speed and command Gazebo hovers at
    w_hover = gz_hover_speed(gz, ap)
    u_hover = (w_hover - w_min) / (w_max - w_min)
    gz_hover_c = fac * u_hover ** 2 + (1 - fac) * u_hover

    drag = [-gz_rotor_drag(gz, w_hover, lambda p, e=e: e)[0] @ e for e in np.eye(3)]
    damp = [-gz_rotor_drag(gz, w_hover, lambda p, e=e: np.cross(e, p))[1] @ e for e in np.eye(3)]
    row("linear drag x / y / z [N/(m/s)]", f"{kdv:.3f} (SIH_KDV, all axes)",
        " / ".join(f"{d:.3f}" for d in drag), f"rotor drag at hover speed {w_hover:.0f} rad/s")
    row("angular damping roll / pitch / yaw [Nm/(rad/s)]", f"{kdw:.4f} (SIH_KDW, all axes)",
        " / ".join(f"{d:.4f}" for d in damp), "rotor drag only; the gz body has no damping")
    row("hover thrust command c (level, no wobble)", f"{hover_c:.3f}", f"{gz_hover_c:.3f}",
        f"MPC_THR_HOVER is {ap['MPC_THR_HOVER']:.3f}")

    return "\n".join(["| | SIH | Gazebo | |", "|---|---|---|---|", *rows])


# --- flying ----------------------------------------------------------------------------------


class GzSitl(Px4Sitl):
    """Px4Sitl that starts Gazebo headless and makes sure the gz server dies with PX4.

    px4-rc.gzsim sources ../gz_env.sh relative to the rootfs, so it is copied next to it.
    """

    _pgid: int | None = None

    def start(self) -> None:
        self.workdir.mkdir(parents=True, exist_ok=True)
        shutil.copy(self.build_dir / "rootfs" / "gz_env.sh", self.workdir / "gz_env.sh")
        super().start()
        self._pgid = self._proc.pid

    def stop(self) -> None:
        super().stop()

        if self._pgid is not None:
            # gz sim is started by rcS in PX4's process group and outlives SIGINT to PX4
            try:
                os.killpg(self._pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self._pgid = None
            time.sleep(2.0)


def simulators() -> dict:
    """name -> (airframe, extra parameters)"""
    gz = gz_model()
    ap = airframe_params("12013_dtrg_planar_octo", "12016_sihsim_planar_octo")
    roll_damping = -gz_rotor_drag(gz, gz_hover_speed(gz, ap), lambda p: np.cross([1.0, 0.0, 0.0], p))[1][0]
    return {
        "sih": (AIRFRAMES["sihsim_planar_octo"], {}),
        "sih_gzdamp": (AIRFRAMES["sihsim_planar_octo"], {"SIH_KDW": round(float(roll_damping), 4)}),
        "gz": (GZ_PLANAR_OCTO, {}),
    }


def boot(sim: str, workdir: Path, build: Path) -> tuple[Px4Sitl, Vehicle]:
    airframe, extra = simulators()[sim]
    cls = GzSitl if sim == "gz" else Px4Sitl
    px4 = cls(build, workdir, airframe, params={**PARAMS, **extra})
    vehicle = Vehicle(px4.mavlink_port)
    vehicle.start_rc(rc_layout.initial_channels({}))
    px4.start()

    if not px4.wait_booted(timeout=120 if sim == "gz" else 40):
        vehicle.close()
        px4.stop()
        raise RuntimeError(f"{sim}: startup script did not finish, see {px4.log_path}")

    vehicle.wait_heartbeat(timeout=60)
    vehicle.wait_prearm(ok=True, timeout=120)
    return px4, vehicle


def mark(phases: dict, name: str, vehicle: Vehicle, **extra) -> None:
    phases[name] = {"t": vehicle.boot_time(), **extra}


def offboard_goto(vehicle: Vehicle, x, y, z, yaw, settle: float, timeout: float = 30.0) -> None:
    vehicle.set_offboard_position(x, y, z, yaw)
    vehicle.wait_position(x, y, z, tolerance=0.3, timeout=timeout)
    vehicle.hold(settle)


def fly_profile(vehicle: Vehicle) -> dict:
    """Fly the comparison profile. Returns the PX4 time [s] each phase starts, and its targets."""
    phases: dict = {}
    vehicle.wait_mode(MAIN_STABILIZED)

    mark(phases, "takeoff", vehicle)
    take_off(vehicle, settle=1.0)
    mark(phases, "hover", vehicle)
    vehicle.hold(10.0)

    x, y, z = vehicle.position()
    yaw = vehicle.yaw()
    vehicle.set_offboard_position(x, y, z, yaw)
    vehicle.hold(1.0)
    assert vehicle.set_mode(MAIN_OFFBOARD).accepted, "Offboard rejected"
    vehicle.wait_mode(MAIN_OFFBOARD)
    vehicle.hold(2.0)

    mark(phases, "yaw", vehicle, yaw0=math.degrees(yaw), yaw1=math.degrees(yaw) + 90.0)
    yaw += math.pi / 2
    vehicle.set_offboard_position(x, y, z, yaw)
    vehicle.hold(6.0)

    mark(phases, "north", vehicle, x0=x, x1=x + STEP_NORTH)
    offboard_goto(vehicle, x + STEP_NORTH, y, z, yaw, settle=4.0)

    vehicle.set_rc(CH_HT_MODE, PWM_MAX)
    vehicle.hold(2.0)
    mark(phases, "south_ht", vehicle, x0=x + STEP_NORTH, x1=x)
    offboard_goto(vehicle, x, y, z, yaw, settle=4.0)

    mark(phases, "ht_roll", vehicle, roll=math.degrees(HT_ROLL_RAD))
    for _ in range(80):
        vehicle.send_debug_float_array([HT_ROLL_RAD, 0.0])
        vehicle.hold(0.1)
    mark(phases, "ht_off", vehicle)
    vehicle.set_rc(CH_HT_MODE, PWM_MIN)
    vehicle.hold(4.0)

    mark(phases, "climb", vehicle, z0=z, z1=z - STEP_UP)
    offboard_goto(vehicle, x, y, z - STEP_UP, yaw, settle=4.0)
    mark(phases, "descend", vehicle, z0=z - STEP_UP, z1=z)
    offboard_goto(vehicle, x, y, z, yaw, settle=4.0)

    mark(phases, "land", vehicle)
    assert vehicle.set_mode(MAIN_AUTO, AUTO_LAND).accepted
    vehicle.wait_armed(False, timeout=40)
    mark(phases, "end", vehicle)
    vehicle.stop_offboard()
    return phases


# --- analysis --------------------------------------------------------------------------------


def step(t: np.ndarray, y: np.ndarray, y0: float, y1: float) -> dict:
    """Rise time 10-90 %, overshoot beyond the target (units of y), settling time into +-10 % of the step."""
    s = (y - y0) / (y1 - y0)
    t = t - t[0]
    above10, above90 = np.flatnonzero(s >= 0.1), np.flatnonzero(s >= 0.9)
    rise = float(t[above90[0]] - t[above10[0]]) if above10.size and above90.size else float("nan")
    outside = np.flatnonzero(np.abs(s - 1.0) > 0.1)
    settle = float(t[outside[-1] + 1]) if outside.size and outside[-1] + 1 < len(t) else float("nan")
    return {"rise_s": rise, "overshoot": float(max(0.0, s.max() - 1.0) * abs(y1 - y0)), "settle_s": settle}


def wrap180(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def dominant_hz(t: np.ndarray, y: np.ndarray) -> float:
    y = y - y.mean()
    f = np.fft.rfftfreq(len(y), float(np.median(np.diff(t))))
    return float(f[1 + np.argmax(np.abs(np.fft.rfft(y))[1:])])


def analyse(log: Log, phases: dict) -> dict:
    truth = log.topic("vehicle_local_position_groundtruth")
    est = log.topic("vehicle_local_position")
    att = log.topic("vehicle_attitude_groundtruth")
    att_est = log.topic("vehicle_attitude")
    att_sp = log.topic("vehicle_attitude_setpoint")
    motors = log.topic("actuator_motors")
    thrust = log.topic("vehicle_thrust_setpoint")
    hover_est = log.topic("hover_thrust_estimate")
    imu = log.topic("sensor_combined")

    roll, pitch, yaw = euler_deg(np.stack([att[f"q[{i}]"] for i in range(4)], axis=1))
    _, _, yaw_est = euler_deg(np.stack([att_est[f"q[{i}]"] for i in range(4)], axis=1))
    roll_sp, pitch_sp, _ = euler_deg(np.stack([att_sp[f"q_d[{i}]"] for i in range(4)], axis=1))
    motor = np.stack([motors[f"control[{i}]"] for i in range(NUM_MOTORS)], axis=1)
    thrust_norm = np.linalg.norm(np.stack([thrust[f"xyz[{i}]"] for i in range(3)], axis=1), axis=1)
    names = list(phases)

    def sel(t, name, trim=0.0):
        start, end = phases[name]["t"], phases[names[names.index(name) + 1]]["t"]
        return (t >= start + trim) & (t <= end)

    def attitude(mask):
        return {"roll_rms_deg": float(np.sqrt(np.mean(roll[mask] ** 2))),
                "pitch_rms_deg": float(np.sqrt(np.mean(pitch[mask] ** 2))),
                "tilt_max_deg": float(np.maximum(np.abs(roll[mask]), np.abs(pitch[mask])).max())}

    def motor_range(mask):
        m = motor[mask]
        return {"motor_min": float(m.min()), "motor_max": float(m.max()),
                "saturated_pct": float(100.0 * np.mean(np.any(m >= 0.999, axis=1)))}

    def heading_error(mask_att):
        return float(np.abs(wrap180(np.interp(att.t[mask_att], att_est.t, yaw_est) - yaw[mask_att])).max())

    r: dict = {}

    # take-off: from lifting off to 90 % of the take-off altitude, on the estimate as take_off() waits on it
    m = sel(est.t, "takeoff")
    alt = -(est["z"][m] - est["z"][m][0])
    lift, reached = np.flatnonzero(alt > 0.1), np.flatnonzero(alt >= 0.9 * FLIGHT_PARAMS["MIS_TAKEOFF_ALT"])
    r["takeoff"] = {"liftoff_to_90pct_s": float(est.t[m][reached[0]] - est.t[m][lift[0]]),
                    "peak_climb_mps": float(-truth["vz"][sel(truth.t, "takeoff")].min())}

    # hover, 2 s after the take-off settled
    m_att, m_tr, m_est = sel(att.t, "hover", 2.0), sel(truth.t, "hover", 2.0), sel(est.t, "hover", 2.0)
    m_mot, m_thr, m_imu = sel(motors.t, "hover", 2.0), sel(thrust.t, "hover", 2.0), sel(imu.t, "hover", 2.0)
    m_hov = sel(hover_est.t, "hover", 2.0)
    est_z = np.interp(truth.t[m_tr], est.t[m_est], est["z"][m_est])
    gyro = np.stack([imu[f"gyro_rad[{i}]"][m_imu] for i in range(3)], axis=1)
    accel = np.stack([imu[f"accelerometer_m_s2[{i}]"][m_imu] for i in range(3)], axis=1)
    r["hover"] = {
        **attitude(m_att),
        "wobble_hz": dominant_hz(att.t[m_att], roll[m_att]),
        # the setpoint swinging too means the position loop takes part, not only the attitude loop
        "roll_sp_rms_deg": float(np.sqrt(np.mean(roll_sp[sel(att_sp.t, "hover", 2.0)] ** 2))),
        "pitch_sp_rms_deg": float(np.sqrt(np.mean(pitch_sp[sel(att_sp.t, "hover", 2.0)] ** 2))),
        "thrust_cmd": float(thrust_norm[m_thr].mean()),
        "hover_thrust_est": float(hover_est["hover_thrust"][m_hov][-1]) if m_hov.any() else float("nan"),
        "motor_mean": float(motor[m_mot].mean()),
        "motor_spread": float(np.ptp(motor[m_mot].mean(axis=0))),
        **motor_range(m_mot),
        "alt_std_m": float(np.std(truth["z"][m_tr])),
        "drift_m": float(np.max(np.hypot(truth["x"][m_tr] - truth["x"][m_tr][0],
                                         truth["y"][m_tr] - truth["y"][m_tr][0]))),
        "est_alt_error_mean_m": float(np.mean(est_z - truth["z"][m_tr])),
        "est_alt_error_std_m": float(np.std(est_z - truth["z"][m_tr])),
        "heading_error_max_deg": heading_error(m_att),
        "gyro_noise_radps": float(np.std(np.diff(gyro, axis=0), axis=0).mean() / math.sqrt(2)),
        "accel_noise_mps2": float(np.std(np.diff(accel, axis=0), axis=0).mean() / math.sqrt(2)),
    }

    # yaw step, on the estimated yaw
    p = phases["yaw"]
    m = sel(att_est.t, "yaw")
    y_est = np.degrees(np.unwrap(np.radians(yaw_est[m])))
    y_est += 360.0 * round((p["yaw0"] - y_est[0]) / 360.0)
    r["yaw"] = {**step(att_est.t[m], y_est, p["yaw0"], p["yaw1"]),
                "peak_rate_degps": float(np.abs(np.gradient(y_est, att_est.t[m])).max()),
                **attitude(sel(att.t, "yaw")), **motor_range(sel(motors.t, "yaw"))}

    # horizontal steps without and with horizontal thrust, on the estimated position
    for name in ("north", "south_ht"):
        p = phases[name]
        m = sel(est.t, name)
        r[name] = {**step(est.t[m], est["x"][m], p["x0"], p["x1"]),
                   "peak_speed_mps": float(np.hypot(truth["vx"][sel(truth.t, name)],
                                                    truth["vy"][sel(truth.t, name)]).max()),
                   **attitude(sel(att.t, name)), **motor_range(sel(motors.t, name)),
                   "heading_error_max_deg": heading_error(sel(att.t, name))}

    # HT roll tilt from Offboard (DEBUG_FLOAT_ARRAY [0.1, 0] rad for 8 s): the last 3 s of it
    end = phases["ht_off"]["t"] - 0.3
    m_att = (att.t >= end - 3.0) & (att.t <= end)
    m_tr = sel(truth.t, "ht_roll")
    r["ht_roll"] = {"roll_mean_deg": float(roll[m_att].mean()), "roll_std_deg": float(roll[m_att].std()),
                    "pitch_mean_deg": float(pitch[m_att].mean()),
                    "drift_m": float(np.max(np.hypot(truth["x"][m_tr] - truth["x"][m_tr][0],
                                                     truth["y"][m_tr] - truth["y"][m_tr][0]))),
                    **motor_range((motors.t >= end - 3.0) & (motors.t <= end))}

    # vertical steps, on the estimated altitude
    for name in ("climb", "descend"):
        p = phases[name]
        m = sel(est.t, name)
        r[name] = {**step(est.t[m], est["z"][m], p["z0"], p["z1"]),
                   "peak_vz_mps": float(np.abs(truth["vz"][sel(truth.t, name)]).max()),
                   "thrust_cmd_min": float(thrust_norm[sel(thrust.t, name)].min()),
                   "thrust_cmd_max": float(thrust_norm[sel(thrust.t, name)].max()),
                   **attitude(sel(att.t, name)), **motor_range(sel(motors.t, name))}

    m = sel(truth.t, "land")
    r["land"] = {"land_to_disarm_s": float(phases["end"]["t"] - phases["land"]["t"]),
                 "peak_descent_mps": float(truth["vz"][m].max())}
    return r


# --- report ----------------------------------------------------------------------------------


def flatten(d: dict, prefix: str = "") -> dict:
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(flatten(v, f"{prefix}{k}."))
        else:
            out[f"{prefix}{k}"] = v
    return out


def table(results: dict) -> str:
    """Markdown table: one row per metric, one column per simulator (each run, then the mean)."""
    sims = list(results)
    flat = {sim: [flatten(r) for r in runs] for sim, runs in results.items()}
    keys = list(flat[sims[0]][0])
    lines = ["| metric | " + " | ".join(sims) + " |", "|---" * (len(sims) + 1) + "|"]

    for key in keys:
        cells = []
        for sim in sims:
            values = [run[key] for run in flat[sim]]
            mean = float(np.nanmean(values)) if not all(math.isnan(v) for v in values) else float("nan")
            runs = " / ".join(f"{v:.3g}" for v in values)
            cells.append(f"**{mean:.3g}** ({runs})" if len(values) > 1 else f"{mean:.3g}")
        lines.append(f"| {key} | " + " | ".join(cells) + " |")

    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--static", action="store_true", help="only print the static model comparison")
    parser.add_argument("--runs", type=int, default=2, help="flights per simulator (default: 2)")
    parser.add_argument("--sims", default="sih,sih_gzdamp,gz", help="comma-separated, see above (default: all)")
    parser.add_argument("--out", type=Path, default=Path("/tmp/dtrg_sih_vs_gz"), help="px4.log, ULogs and results")
    parser.add_argument("--analyse-only", action="store_true", help="re-analyse the flights already in --out")
    parser.add_argument("--px4-build", type=Path, default=REPO / "build" / "px4_sitl_default")
    args = parser.parse_args()

    static = static_comparison()
    print(static, flush=True)
    if args.static:
        return 0

    os.environ["HEADLESS"] = "1"
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "static.md").write_text(static + "\n")
    results: dict = {}

    for sim in args.sims.split(","):
        for run in range(args.runs):
            workdir = args.out / f"{sim}_run{run}"

            if not args.analyse_only:
                shutil.rmtree(workdir, ignore_errors=True)
                print(f"[{sim} run {run}] booting", flush=True)
                px4, vehicle = boot(sim, workdir, args.px4_build)

                try:
                    phases = fly_profile(vehicle)
                finally:
                    vehicle.close()
                    px4.stop()
                    px4.collect_artifacts()

                (workdir / "phases.json").write_text(json.dumps(phases, indent=1))

            phases = json.loads((workdir / "phases.json").read_text())
            metrics = analyse(Log(sorted(workdir.glob("*.ulg"))), phases)
            (workdir / "metrics.json").write_text(json.dumps(metrics, indent=1))
            results.setdefault(sim, []).append(metrics)
            print(f"[{sim} run {run}] done", flush=True)

    (args.out / "results.json").write_text(json.dumps(results, indent=1))
    report = table(results)
    (args.out / "results.md").write_text(report + "\n")
    print(report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
