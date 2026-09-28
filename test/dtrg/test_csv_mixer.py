"""DTRG CSV mixer (DTRG_MIXER_CSV) in SITL.

The mixer file path is hardcoded to /fs/microsd/etc/mixer.csv, which does not
exist in SITL, so only the "no file" case can run here. The parser, the other
rejection reasons (empty file, short row, bad value, row count, all zeros) and
the fallback to an all-zero mixer are unit tested in
src/lib/control_allocation/control_allocation/DtrgMixerCsvTest.cpp.
"""

import pytest

from vehicle import WaitTimeout

pytestmark = pytest.mark.sih

# msg/DtrgMixerStatus.msg
STATUS_DISABLED = 0
STATUS_FILE_NOT_FOUND = 2


def test_csv_mixer_without_file_refuses_to_arm(sitl):
    vehicle, px4 = sitl(params={"DTRG_MIXER_CSV": 1}, wait_ready=False)

    # control_allocator reports why the mixer was rejected
    vehicle.wait_until(lambda: px4.listen("dtrg_mixer_status").get("status") == STATUS_FILE_NOT_FOUND, 30,
                       "dtrg_mixer_status reporting STATUS_FILE_NOT_FOUND")

    # give the estimator time to converge, so a refusal is not about something else
    try:
        vehicle.wait_prearm(ok=True, timeout=60)
    except WaitTimeout:
        pass

    assert not vehicle.prearm_ok()

    # the reason is given on every arm attempt, not only when the failure first appears
    for _ in range(2):
        mark = vehicle.now()
        assert not vehicle.arm().accepted
        vehicle.wait_text("Arming denied: DTRG mixer file not found", since=mark)

    assert not vehicle.armed()


def test_csv_mixer_disabled_does_not_block_arming(sitl):
    vehicle, px4 = sitl(params={"DTRG_MIXER_CSV": 0})

    assert px4.listen("dtrg_mixer_status").get("status") == STATUS_DISABLED
    assert vehicle.arm().accepted
    vehicle.wait_armed()
