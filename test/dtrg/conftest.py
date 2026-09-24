"""pytest fixtures for the DTRG SITL tests (see README.md)."""

from __future__ import annotations

import os
import warnings
from pathlib import Path

import pytest

import rc_layout
from airframes import AIRFRAMES, DEFAULT_AIRFRAME
from px4_process import Px4Sitl
from vehicle import Vehicle

REPO = Path(__file__).resolve().parents[2]

# time for the startup script to finish; it takes ~6 s
BOOT_TIMEOUT = 40
BOOT_RETRIES = 2

# parameters applied to every test, on top of the airframe
BASE_PARAMS = {
    # do not auto-disarm while a test sits armed on the ground
    "COM_DISARM_PRFLT": -1,
    # log from boot, so the ULog also covers what happens before arming
    "SDLOG_MODE": 1,
}


def pytest_addoption(parser):
    group = parser.getgroup("dtrg", "DTRG SITL tests")
    group.addoption("--px4-build", default=os.environ.get("DTRG_PX4_BUILD", str(REPO / "build" / "px4_sitl_default")),
                    help="PX4 SITL build directory (default: build/px4_sitl_default)")
    group.addoption("--airframe", default=os.environ.get("DTRG_SIH_AIRFRAME", DEFAULT_AIRFRAME),
                    choices=sorted(AIRFRAMES), help="SIH airframe to run on")
    group.addoption("--px4-instance", type=int, default=int(os.environ.get("DTRG_PX4_INSTANCE", "0")),
                    help="PX4 instance number; use a free one to run next to another SITL (default: 0)")
    group.addoption("--speed-factor", type=float, default=float(os.environ.get("DTRG_SPEED_FACTOR", "1")),
                    help="PX4_SIM_SPEED_FACTOR (default: 1)")


def pytest_collection_modifyitems(config, items):
    airframe = AIRFRAMES[config.getoption("--airframe")]

    if not airframe.fully_actuated:
        skip = pytest.mark.skip(reason=f"needs a fully actuated airframe, {airframe.name} is not")
        for item in items:
            if "fully_actuated" in item.keywords:
                item.add_marker(skip)


@pytest.fixture(scope="session")
def airframe(request):
    return AIRFRAMES[request.config.getoption("--airframe")]


def check_params(vehicle: Vehicle, params: dict) -> None:
    """Fail if a requested parameter did not boot with its value.

    rcS applies ``PX4_PARAM_*`` before and again after the airframe script, because a
    value equal to the firmware default does not count as set and the airframe's
    ``param set-default`` would win (e.g. ``DTRG_HT_EN=0`` on the planarOcto).
    """
    wrong = {}

    for name, value in params.items():
        current = vehicle.get_param(name)

        if abs(float(current) - float(value)) > 1e-6:
            wrong[name] = (current, value)

    assert not wrong, f"parameters booted with other values (got, requested): {wrong}"


@pytest.fixture
def sitl(request, tmp_path, airframe):
    """Factory: ``vehicle, px4 = sitl(params={...}, rc={...})`` boots a fresh PX4 with SIH.

    - ``params``: parameters set at boot, on top of BASE_PARAMS and the airframe
    - ``rc``: when given, the standard RC layout (rc_layout.RC_PARAMS) is applied and
      RC_CHANNELS_OVERRIDE is streamed from the start, starting from
      ``rc_layout.initial_channels(rc)``
    - ``wait_ready``: wait until the arming checks pass before returning

    Everything is shut down at the end of the test. The px4.log and ULogs of the
    run stay in the test's tmp_path (``--basetemp`` on CI, uploaded on failure).
    """
    config = request.config
    started: list[tuple[Px4Sitl, Vehicle]] = []
    boots = 0

    def start(params: dict | None = None, rc: dict | None = None, wait_ready: bool = True):
        nonlocal boots
        all_params = dict(BASE_PARAMS)
        if rc is not None:
            all_params.update(rc_layout.RC_PARAMS)
        all_params.update(params or {})

        def boot():
            nonlocal boots
            px4 = Px4Sitl(Path(config.getoption("--px4-build")), tmp_path / f"run{boots}", airframe,
                          instance=config.getoption("--px4-instance"), params=all_params,
                          speed_factor=config.getoption("--speed-factor"))
            boots += 1
            vehicle = Vehicle(px4.mavlink_port)
            started.append((px4, vehicle))

            if rc is not None:
                vehicle.start_rc(rc_layout.initial_channels(rc))

            px4.start()
            return px4, vehicle

        px4, vehicle = boot()

        for retry in range(BOOT_RETRIES + 1):
            if px4.wait_booted(timeout=BOOT_TIMEOUT):
                break

            assert retry < BOOT_RETRIES, f"PX4 startup script did not finish {BOOT_RETRIES + 1} times, see {px4.log_path}"
            # a known flake of the PX4 client/server in rcS (~3% of boots on macOS, with or
            # without the code under test), so boot again
            warnings.warn(f"PX4 startup script did not finish within {BOOT_TIMEOUT} s ({px4.log_path}), "
                          "booting again")
            started.remove((px4, vehicle))
            vehicle.close()
            px4.stop()
            px4, vehicle = boot()

        vehicle.wait_heartbeat(timeout=60)
        check_params(vehicle, {**airframe.params, **all_params})

        if wait_ready:
            vehicle.wait_prearm(ok=True, timeout=90)

        return vehicle, px4

    yield start

    for px4, vehicle in started:
        vehicle.close()
        px4.stop()
        px4.collect_artifacts()

