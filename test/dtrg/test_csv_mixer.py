"""DTRG CSV mixer (DTRG_MIXER_CSV) in SITL.

The mixer file path is hardcoded to /fs/microsd/etc/mixer.csv, which does not
exist in SITL, so only the "no file" case can run here. The parser itself is
unit tested in src/lib/control_allocation/control_allocation/DtrgMixerCsvTest.cpp.
"""

import pytest

from vehicle import WaitTimeout

pytestmark = pytest.mark.sih


@pytest.mark.xfail(strict=True, reason=(
    "known gap: with DTRG_MIXER_CSV=1 and no mixer file the allocator keeps an all-zero mixer and "
    "nothing stops arming. Decide the behaviour (refuse to arm, or fall back to the geometry), fix it, "
    "then drop this marker"))
def test_csv_mixer_without_file_refuses_to_arm(sitl):
    vehicle, _ = sitl(params={"DTRG_MIXER_CSV": 1}, wait_ready=False)

    # give the estimator time to converge, so a refusal is not about something else
    try:
        vehicle.wait_prearm(ok=True, timeout=60)
    except WaitTimeout:
        pass

    assert not vehicle.arm().accepted
