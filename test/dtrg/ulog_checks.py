"""Read uORB topics back from the ULog of a test run."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from pyulog import ULog

from px4_process import Px4Sitl

NAV_STATE_BENCH_TEST = 16
ARMING_STATE_ARMED = 2


@dataclass
class Series:
    """One topic instance as numpy arrays keyed by field name, plus ``t`` in seconds."""

    t: np.ndarray
    fields: dict[str, np.ndarray]

    def __getitem__(self, name: str) -> np.ndarray:
        return self.fields[name]

    def __len__(self) -> int:
        return len(self.t)

    def where(self, mask: np.ndarray) -> Series:
        return Series(self.t[mask], {k: v[mask] for k, v in self.fields.items()})

    def between(self, start: float, end: float) -> Series:
        return self.where((self.t >= start) & (self.t <= end))


class Log:
    def __init__(self, paths: list[Path]):
        if not paths:
            raise AssertionError("the run left no ULog")
        self._ulogs = [ULog(str(p)) for p in paths]

    def topic(self, name: str, instance: int = 0) -> Series:
        """All samples of ``name`` across the run's ULogs, in time order. Empty if never logged."""
        t, fields = [], {}

        for ulog in self._ulogs:
            for d in ulog.data_list:
                if d.name == name and d.multi_id == instance:
                    t.append(d.data["timestamp"] / 1e6)
                    for k, v in d.data.items():
                        fields.setdefault(k, []).append(v)

        if not t:
            return Series(np.array([]), {})

        order = np.argsort(np.concatenate(t))
        return Series(np.concatenate(t)[order], {k: np.concatenate(v)[order] for k, v in fields.items()})

    def intervals(self, predicate) -> list[tuple[float, float]]:
        """Time spans where ``predicate(vehicle_status sample)`` holds, from vehicle_status."""
        status = self.topic("vehicle_status")
        if not len(status):
            return []

        active = predicate(status)
        spans, start = [], None

        for t, a in zip(status.t, active):
            if a and start is None:
                start = t
            elif not a and start is not None:
                spans.append((start, t))
                start = None

        if start is not None:
            spans.append((start, float(status.t[-1])))

        return spans


def bench_test_armed(status: Series) -> np.ndarray:
    return (status["nav_state"] == NAV_STATE_BENCH_TEST) & (status["arming_state"] == ARMING_STATE_ARMED)


def bench_test_disarmed(status: Series) -> np.ndarray:
    return (status["nav_state"] == NAV_STATE_BENCH_TEST) & (status["arming_state"] != ARMING_STATE_ARMED)


def runs(series: Series, mask: np.ndarray) -> list[tuple[float, float]]:
    """(first, last) sample time of each run of consecutive samples where ``mask`` holds."""
    idx = np.flatnonzero(mask)
    if not idx.size:
        return []
    breaks = np.flatnonzero(np.diff(idx) > 1)
    starts = np.concatenate(([idx[0]], idx[breaks + 1]))
    ends = np.concatenate((idx[breaks], [idx[-1]]))
    return [(float(series.t[a]), float(series.t[b])) for a, b in zip(starts, ends)]


def read_log(px4: Px4Sitl) -> Log:
    """Stop ``px4``, which closes its ULog, and open the log."""
    px4.stop()
    return Log(px4.ulogs())
