"""MAVLink helper for the DTRG SITL tests.

A background thread reads everything PX4 sends and keeps the latest message of
each type, every STATUSTEXT and every COMMAND_ACK. A second thread sends a GCS
HEARTBEAT at 1 Hz (PX4 only streams STATUSTEXT to a link that has seen a GCS)
and, once started, RC_CHANNELS_OVERRIDE and an Offboard position setpoint at 50 Hz.
"""

from __future__ import annotations

import os
import struct
import threading
import time
from dataclasses import dataclass, field

# channels 9-18 of RC_CHANNELS_OVERRIDE only exist in MAVLink 2
os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "common")

from pymavlink import mavutil  # noqa: E402

mavlink = mavutil.mavlink

# PX4 custom main modes as reported in HEARTBEAT.custom_mode (src/modules/commander/px4_custom_mode.h)
MAIN_MANUAL = 1
MAIN_ALTCTL = 2
MAIN_POSCTL = 3
MAIN_AUTO = 4
MAIN_ACRO = 5
MAIN_OFFBOARD = 6
MAIN_STABILIZED = 7
MAIN_BENCH_TEST = 11

# PX4 custom sub modes of MAIN_AUTO
AUTO_TAKEOFF = 2
AUTO_LOITER = 3
AUTO_LAND = 6

# PX4 prints the reason of a rejected mode change only for this component id (Commander.cpp)
MAV_COMP_ID_MISSIONPLANNER = 190

# SET_POSITION_TARGET_LOCAL_NED: use position and yaw, ignore velocity, acceleration and yaw rate
POSITION_TARGET_POSITION_AND_YAW = (
    mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE | mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE | mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE | mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

# SYS_STATUS.errors_count4 is fixed to this value by DTRG firmware (streams/SYS_STATUS.hpp)
DTRG_FIRMWARE_MARKER = 706

RC_STREAM_HZ = 50
NUM_RC_CHANNELS = 18
STATUSTEXT_CHUNK_LEN = 50


class WaitTimeout(AssertionError):
    """A wait_* helper timed out. An AssertionError so pytest reports it as a test failure."""


@dataclass
class StatusText:
    time: float
    severity: int
    text: str


@dataclass
class CommandResult:
    command: int
    result: int | None  # MAV_RESULT, None when no ACK arrived

    @property
    def accepted(self) -> bool:
        return self.result == mavlink.MAV_RESULT_ACCEPTED


@dataclass
class _Chunks:
    parts: dict = field(default_factory=dict)
    severity: int = 0
    time: float = 0.0


def _int_to_param_float(value: int) -> float:
    """PX4 uses bytewise parameter encoding: an INT32 travels as its raw bits in the float field."""
    return struct.unpack("<f", struct.pack("<i", int(value)))[0]


def _param_float_to_int(value: float) -> int:
    return struct.unpack("<i", struct.pack("<f", value))[0]


class Vehicle:
    def __init__(self, port: int):
        self._conn = mavutil.mavlink_connection(
            f"udpin:0.0.0.0:{port}", source_system=255, source_component=MAV_COMP_ID_MISSIONPLANNER)
        self._send_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._stop = threading.Event()

        self._latest: dict[str, object] = {}
        self._texts: list[StatusText] = []
        self._chunks: dict[int, _Chunks] = {}
        self._acks: list[tuple[float, object]] = []
        self._params: dict[str, tuple[float, int]] = {}
        self._target_system: int | None = None

        self._rc: list[int] | None = None
        self._offboard: tuple[float, float, float, float] | None = None

        self._reader = threading.Thread(target=self._read_loop, name="mavlink-rx", daemon=True)
        self._sender = threading.Thread(target=self._send_loop, name="mavlink-tx", daemon=True)
        self._reader.start()
        self._sender.start()

    def close(self) -> None:
        self._stop.set()
        self._reader.join(timeout=2)
        self._sender.join(timeout=2)
        self._conn.close()

    # --- threads ---------------------------------------------------------------------------

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self._conn.recv_match(blocking=True, timeout=0.1)
            except OSError:
                continue

            if msg is None:
                continue

            msg_type = msg.get_type()

            if msg_type == "BAD_DATA":
                continue

            # only listen to the autopilot itself
            if msg.get_srcComponent() != mavlink.MAV_COMP_ID_AUTOPILOT1:
                continue

            now = time.monotonic()

            with self._state_lock:
                self._latest[msg_type] = msg

                if msg_type == "HEARTBEAT" and self._target_system is None:
                    self._target_system = msg.get_srcSystem()

                elif msg_type == "STATUSTEXT":
                    self._add_statustext(msg, now)

                elif msg_type == "COMMAND_ACK":
                    self._acks.append((now, msg))

                elif msg_type == "PARAM_VALUE":
                    self._params[msg.param_id] = (msg.param_value, msg.param_type)

    def _add_statustext(self, msg, now: float) -> None:
        """Reassemble chunked STATUSTEXT (PX4 splits texts longer than 50 characters)."""
        text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="replace")
        text = text.split("\x00", 1)[0]

        chunks = self._chunks.setdefault(msg.id, _Chunks(severity=msg.severity, time=now))
        chunks.parts[msg.chunk_seq] = text
        chunks.time = now

        # a chunk shorter than the maximum ends the text
        if len(text) < STATUSTEXT_CHUNK_LEN:
            self._flush_chunks(msg.id)

    def _flush_chunks(self, text_id: int) -> None:
        chunks = self._chunks.pop(text_id, None)
        if chunks:
            full = "".join(chunks.parts[k] for k in sorted(chunks.parts))
            self._texts.append(StatusText(chunks.time, chunks.severity, full))

    def _send_loop(self) -> None:
        next_heartbeat = 0.0
        period = 1.0 / RC_STREAM_HZ

        while not self._stop.is_set():
            now = time.monotonic()

            try:
                if now >= next_heartbeat:
                    self._send(lambda m: m.heartbeat_send(
                        mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavlink.MAV_STATE_ACTIVE))
                    next_heartbeat = now + 1.0

                rc = self._rc
                target = self._target_system

                if rc is not None and target is not None:
                    self._send(lambda m: m.rc_channels_override_send(target, 1, *rc))

                offboard = self._offboard

                if offboard is not None and target is not None:
                    x, y, z, yaw = offboard
                    self._send(lambda m: m.set_position_target_local_ned_send(
                        0, target, mavlink.MAV_COMP_ID_AUTOPILOT1, mavlink.MAV_FRAME_LOCAL_NED,
                        POSITION_TARGET_POSITION_AND_YAW, x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0))

            except OSError:
                # nothing received yet, so pymavlink does not know where to send to
                pass

            time.sleep(period)

    def _send(self, fn) -> None:
        with self._send_lock:
            fn(self._conn.mav)

    # --- waiting ---------------------------------------------------------------------------

    @staticmethod
    def wait_until(predicate, timeout: float, what: str, poll: float = 0.05):
        """Poll ``predicate`` until it returns something truthy, and return that."""
        deadline = time.monotonic() + timeout
        while True:
            value = predicate()
            if value:
                return value
            if time.monotonic() > deadline:
                raise WaitTimeout(f"timed out after {timeout:.0f} s waiting for {what}")
            time.sleep(poll)

    @staticmethod
    def hold(seconds: float) -> None:
        time.sleep(seconds)

    def latest(self, msg_type: str):
        with self._state_lock:
            return self._latest.get(msg_type)

    def wait_heartbeat(self, timeout: float = 60.0) -> None:
        self.wait_until(lambda: self._target_system is not None and self.latest("HEARTBEAT"), timeout,
                        "the first HEARTBEAT")

    # --- vehicle state ---------------------------------------------------------------------

    def mode(self) -> tuple[int, int]:
        """(main, sub) PX4 custom mode from the latest HEARTBEAT."""
        hb = self.latest("HEARTBEAT")
        if hb is None:
            return (0, 0)
        return ((hb.custom_mode >> 16) & 0xFF, (hb.custom_mode >> 24) & 0xFF)

    def main_mode(self) -> int:
        return self.mode()[0]

    def wait_mode(self, main: int, timeout: float = 10.0) -> None:
        self.wait_until(lambda: self.main_mode() == main, timeout, f"main mode {main} (now {self.main_mode()})")

    def armed(self) -> bool:
        hb = self.latest("HEARTBEAT")
        return bool(hb and hb.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def wait_armed(self, armed: bool = True, timeout: float = 5.0) -> None:
        self.wait_until(lambda: self.armed() == armed, timeout, "armed" if armed else "disarmed")

    def prearm_ok(self) -> bool:
        """Whether the arming checks pass for the current mode (SYS_STATUS prearm health bit)."""
        status = self.latest("SYS_STATUS")
        return bool(status and status.onboard_control_sensors_health & mavlink.MAV_SYS_STATUS_PREARM_CHECK)

    def wait_prearm(self, ok: bool = True, timeout: float = 60.0) -> None:
        self.wait_until(lambda: self.prearm_ok() == ok, timeout, "arming checks to pass" if ok else
                        "arming checks to fail")

    def sys_status(self):
        return self.wait_until(lambda: self.latest("SYS_STATUS"), 10.0, "SYS_STATUS")

    def boot_time(self) -> float:
        """PX4 time [s] of the latest ATTITUDE: the clock of the ULog timestamps (also in lockstep)."""
        att = self.wait_until(lambda: self.latest("ATTITUDE"), 10.0, "ATTITUDE")
        return att.time_boot_ms / 1e3

    def position(self) -> tuple[float, float, float]:
        """Estimated local position NED [m] from the latest LOCAL_POSITION_NED."""
        pos = self.wait_until(lambda: self.latest("LOCAL_POSITION_NED"), 10.0, "LOCAL_POSITION_NED")
        return pos.x, pos.y, pos.z

    def yaw(self) -> float:
        att = self.wait_until(lambda: self.latest("ATTITUDE"), 10.0, "ATTITUDE")
        return att.yaw

    def wait_position(self, x: float, y: float, z: float, tolerance: float, timeout: float) -> None:
        def near():
            px, py, pz = self.position()
            return ((px - x) ** 2 + (py - y) ** 2 + (pz - z) ** 2) ** 0.5 <= tolerance

        self.wait_until(near, timeout, f"position ({x:.1f}, {y:.1f}, {z:.1f}) +-{tolerance} m (now {self.position()})")

    # --- status texts ----------------------------------------------------------------------

    @staticmethod
    def now() -> float:
        return time.monotonic()

    def texts(self, since: float = 0.0) -> list[str]:
        with self._state_lock:
            done = [t.text for t in self._texts if t.time >= since]
            # include texts still waiting for their last chunk
            pending = ["".join(c.parts[k] for k in sorted(c.parts)) for c in self._chunks.values() if c.time >= since]
        return done + pending

    def saw_text(self, fragment: str, since: float = 0.0) -> bool:
        return any(fragment in t for t in self.texts(since))

    def wait_text(self, fragment: str, since: float = 0.0, timeout: float = 5.0) -> str:
        def find():
            return next((t for t in self.texts(since) if fragment in t), None)

        try:
            return self.wait_until(find, timeout, f"status text containing {fragment!r}")
        except WaitTimeout as e:
            raise WaitTimeout(f"{e}; texts seen: {self.texts(since)}") from None

    # --- commands --------------------------------------------------------------------------

    def command(self, command: int, *params: float, timeout: float = 5.0) -> CommandResult:
        p = list(params) + [0.0] * (7 - len(params))
        sent = time.monotonic()
        self._send(lambda m: m.command_long_send(self._target_system, mavlink.MAV_COMP_ID_AUTOPILOT1,
                                                 command, 0, *p))

        def ack():
            with self._state_lock:
                return next((msg for t, msg in self._acks if t >= sent and msg.command == command), None)

        try:
            msg = self.wait_until(ack, timeout, f"COMMAND_ACK for {command}")
            return CommandResult(command, msg.result)
        except WaitTimeout:
            return CommandResult(command, None)

    def arm(self) -> CommandResult:
        return self.command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)

    def disarm(self, force: bool = False) -> CommandResult:
        return self.command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196 if force else 0)

    def set_mode(self, main: int, sub: int = 0) -> CommandResult:
        return self.command(mavlink.MAV_CMD_DO_SET_MODE, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub)

    def send_debug_float_array(self, values: list[float], array_id: int = 0) -> None:
        data = list(values) + [0.0] * (58 - len(values))
        self._send(lambda m: m.debug_float_array_send(int(time.time() * 1e6), b"dtrg", array_id, data))

    # --- parameters ------------------------------------------------------------------------

    def get_param(self, name: str, timeout: float = 5.0) -> float | int:
        with self._state_lock:
            self._params.pop(name, None)

        self._send(lambda m: m.param_request_read_send(self._target_system, mavlink.MAV_COMP_ID_AUTOPILOT1,
                                                       name.encode(), -1))
        value, param_type = self.wait_until(lambda: self._params.get(name), timeout, f"PARAM_VALUE {name}")

        if param_type == mavlink.MAV_PARAM_TYPE_INT32:
            return _param_float_to_int(value)
        return value

    def set_param(self, name: str, value: float | int, timeout: float = 5.0) -> None:
        current = self.get_param(name, timeout)
        is_int = isinstance(current, int)

        with self._state_lock:
            self._params.pop(name, None)

        wire = _int_to_param_float(value) if is_int else float(value)
        param_type = mavlink.MAV_PARAM_TYPE_INT32 if is_int else mavlink.MAV_PARAM_TYPE_REAL32
        self._send(lambda m: m.param_set_send(self._target_system, mavlink.MAV_COMP_ID_AUTOPILOT1,
                                              name.encode(), wire, param_type))

        self.wait_until(lambda: self._params.get(name), timeout, f"PARAM_VALUE echo for {name}")
        readback = self.get_param(name, timeout)
        if (readback != int(value)) if is_int else abs(readback - float(value)) > 1e-6:
            raise AssertionError(f"{name} reads back as {readback}, expected {value}")

    # --- RC --------------------------------------------------------------------------------

    def start_rc(self, channels: dict[int, int]) -> None:
        """Stream RC_CHANNELS_OVERRIDE with the given 1-based channel PWM, 1500 for the rest."""
        rc = [1500] * NUM_RC_CHANNELS
        for ch, pwm in channels.items():
            rc[ch - 1] = int(pwm)
        self._rc = rc

    def set_rc(self, channel: int, pwm: int) -> None:
        if self._rc is None:
            raise RuntimeError("start_rc() first")
        rc = list(self._rc)
        rc[channel - 1] = int(pwm)
        self._rc = rc

    def stop_rc(self) -> None:
        self._rc = None

    # --- Offboard --------------------------------------------------------------------------

    def set_offboard_position(self, x: float, y: float, z: float, yaw: float) -> None:
        """Stream this local NED position and yaw setpoint at 50 Hz, until stop_offboard().

        PX4 only accepts Offboard mode once setpoints are streaming, so start this first.
        """
        self._offboard = (float(x), float(y), float(z), float(yaw))

    def stop_offboard(self) -> None:
        self._offboard = None
