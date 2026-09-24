"""SIH airframes the DTRG SITL tests can run on.

The tests pick one with ``--airframe`` (or ``DTRG_SIH_AIRFRAME``); the default,
and what CI runs, is the planarOcto. The logic tier asserts on setpoints and
arming decisions rather than on how the vehicle moves, so it also runs on the
quad, which is kept for checking that a DTRG feature does not depend on the
octo.

Tests marked ``@pytest.mark.fully_actuated`` need a vehicle that can produce
horizontal thrust without tilting (the planarOcto). They are skipped on an
airframe with ``fully_actuated=False``.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Airframe:
    name: str
    autostart: int
    num_motors: int
    fully_actuated: bool
    # extra parameters applied to every test on this airframe
    params: dict = field(default_factory=dict)


AIRFRAMES = {
    "sihsim_quadx": Airframe(
        name="sihsim_quadx",
        autostart=10040,
        num_motors=4,
        fully_actuated=False,
    ),
    # the planarOcto: SIH builds it from the CA_ROTOR* geometry of 12013_dtrg_planar_octo
    "sihsim_planar_octo": Airframe(
        name="sihsim_planar_octo",
        autostart=12016,
        num_motors=8,
        fully_actuated=True,
    ),
}

DEFAULT_AIRFRAME = "sihsim_planar_octo"
