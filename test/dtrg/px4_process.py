"""Start and stop one PX4 SITL (SIH) instance with a clean rootfs."""

from __future__ import annotations

import os
import re
import shutil
import signal
import subprocess
import time
from pathlib import Path

from airframes import Airframe


class Px4Sitl:
    """One PX4 SITL process running SIH, in its own rootfs so no state leaks between tests.

    Parameters are applied at boot through ``PX4_PARAM_<NAME>`` environment variables,
    which ``init.d-posix/rcS`` sets before the airframe script runs. Airframe defaults
    (``param set-default``) therefore do not override them.
    """

    def __init__(self, build_dir: Path, workdir: Path, airframe: Airframe, instance: int = 0,
                 params: dict | None = None, speed_factor: float = 1.0):
        self.build_dir = Path(build_dir).resolve()
        self.workdir = Path(workdir)
        self.rootfs = self.workdir / "rootfs"
        self.log_path = self.workdir / "px4.log"
        self.airframe = airframe
        self.instance = instance
        self.params = {**airframe.params, **(params or {})}
        self.speed_factor = speed_factor
        self._proc: subprocess.Popen | None = None
        self._log = None

    @property
    def mavlink_port(self) -> int:
        """UDP port the onboard MAVLink link (px4-rc.mavlink) sends to."""
        return 14540 + self.instance

    def start(self) -> None:
        px4 = self.build_dir / "bin" / "px4"
        if not px4.exists():
            raise FileNotFoundError(f"{px4} not found, build it first (make px4_sitl_default)")

        self.rootfs.mkdir(parents=True, exist_ok=True)

        env = os.environ.copy()
        # never inherit simulator selection from the calling shell
        for key in ("PX4_SIM_MODEL", "PX4_SIMULATOR", "PX4_GZ_WORLD", "PX4_GZ_MODEL_POSE"):
            env.pop(key, None)
        env = {k: v for k, v in env.items() if not k.startswith("PX4_PARAM_")}
        env["PX4_SYS_AUTOSTART"] = str(self.airframe.autostart)
        env["PX4_SIM_SPEED_FACTOR"] = str(self.speed_factor)

        for name, value in self.params.items():
            env[f"PX4_PARAM_{name}"] = str(value)

        self._log = open(self.log_path, "w")
        self._proc = subprocess.Popen(
            [str(px4), "-d", "-i", str(self.instance), "-w", str(self.rootfs),
             "-s", "etc/init.d-posix/rcS", str(self.build_dir / "etc")],
            env=env, stdout=self._log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
            start_new_session=True)

    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def wait_booted(self, timeout: float) -> bool:
        """Wait for the startup script (rcS) to finish. False if it has not after ``timeout`` s.

        Occasionally a ``px4-<module>`` client command in rcS never gets its reply from the
        PX4 server (seen on macOS), and the boot stops there; the caller can retry.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            console = self.console()
            if "Startup script returned successfully" in console:
                return True
            if "already running for instance" in console:
                raise RuntimeError(f"PX4 instance {self.instance} is in use by another PX4 process; stop it or "
                                   "pick a free one with --px4-instance")
            if "Startup script returned with return value" in console or not self.running():
                raise RuntimeError(f"PX4 failed to boot, see {self.log_path}")
            time.sleep(0.2)
        return False

    def client(self, *args: str, timeout: float = 5.0) -> str:
        """Run a px4 shell command (e.g. ``client("logger", "stop")``) against this instance."""
        px4 = self.build_dir / "bin" / "px4"
        # the client is selected by the argv[0] name px4-<module>
        link = self.build_dir / "bin" / f"px4-{args[0]}"
        cmd = [str(link) if link.exists() else str(px4), "--instance", str(self.instance), *args[1:]]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=False)
        return result.stdout + result.stderr

    def listen(self, topic: str, instance: int = 0) -> dict:
        """Latest sample of a uORB topic via ``listener``: field name -> float or list of floats.

        Values are printed with 5 decimals, which is plenty for setpoint checks. Empty when the
        topic was never published.
        """
        args = ["listener", topic, "-n", "1"]
        if instance:
            args += ["-i", str(instance)]

        # the px4 client very occasionally hangs connecting to the server; retry
        for attempt in range(3):
            try:
                out = self.client(*args)
                break
            except subprocess.TimeoutExpired:
                if attempt == 2:
                    raise

        fields = {}

        for line in out.splitlines():
            m = re.match(r"\s+(\w+): (.*)$", line)
            if not m:
                continue
            name, value = m.groups()
            value = value.split(" (", 1)[0].strip()  # drop "(Roll: .. deg ...)" style annotations
            try:
                if value.startswith("["):
                    fields[name] = [float(v) for v in value.strip("[]").split(",") if v.strip()]
                else:
                    fields[name] = float(value)
            except ValueError:
                fields[name] = value

        return fields

    def stop(self) -> None:
        if self._proc is None:
            return

        if self.running():
            # close the log file cleanly so the ULog can be parsed
            try:
                self.client("logger", "stop")
            except (subprocess.TimeoutExpired, OSError):
                pass

            try:
                os.killpg(self._proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

            try:
                self._proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self._proc.pid, signal.SIGKILL)
                self._proc.wait(timeout=10)

        self._proc = None

        if self._log:
            self._log.close()
            self._log = None

    def console(self) -> str:
        """Everything PX4 printed so far (the px4.log of this run)."""
        if self._log:
            self._log.flush()
        return self.log_path.read_text(errors="replace") if self.log_path.exists() else ""

    def ulogs(self) -> list[Path]:
        """ULog files written by this instance, oldest first."""
        return sorted(self.rootfs.rglob("*.ulg"), key=lambda p: p.stat().st_mtime)

    def collect_artifacts(self) -> None:
        """Copy the ULogs next to px4.log, so a CI artifact of workdir has everything."""
        for ulg in self.ulogs():
            shutil.copy(ulg, self.workdir / ulg.name)
