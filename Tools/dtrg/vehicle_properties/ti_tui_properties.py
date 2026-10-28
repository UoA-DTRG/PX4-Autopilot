#!/usr/bin/env python3
"""Apply ti_tui_properties.yaml to the Ti-Tui (stacked octo) Gazebo models and PX4 airframe.

Edits in place (no CAD needed):
  Tools/simulation/gz/models/ti_tui_base/model.sdf   base mass, CoM and inertia
  Tools/simulation/gz/models/ti_tui/model.sdf        rotor link inertia, motor constants
  Tools/simulation/gz/models/ti_tui_rod/model.sdf    base CoM rebalanced for the rod
  ROMFS/px4fmu_common/init.d/airframes/12011_ti_tui_canted_coax
                                                     CA_ROTOR*_CT/KM, THR_MDL_FAC,
                                                     MPC_THR_HOVER (and MPC_MR_* if present)

All rotors are identical except that the lower four (motors 5-8) are scaled by
lower_rotor_effectiveness. Rotor geometry (CA_ROTOR*_P/A) and rotor speed limits
(SIM_GZ_EC_MIN/MAX in 12007_gz_ti_tui) are read from the airframes, not changed.
The allocation maths is shared with planetary_hex_properties.py.

    python3 Tools/dtrg/vehicle_properties/ti_tui_properties.py            # apply
    python3 Tools/dtrg/vehicle_properties/ti_tui_properties.py --dry-run  # show what would change
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import yaml

import planetary_hex_properties as base
from planetary_hex_properties import ROOT, body_sdfs, calibration, set_param, update_model_sdf, write_outputs

CONFIG = Path(__file__).with_suffix(".yaml")
BASE_SDF = ROOT / "Tools/simulation/gz/models/ti_tui_base/model.sdf"
MODEL_SDF = ROOT / "Tools/simulation/gz/models/ti_tui/model.sdf"
ROD_SDF = ROOT / "Tools/simulation/gz/models/ti_tui_rod/model.sdf"
HW_AIRFRAME = ROOT / "ROMFS/px4fmu_common/init.d/airframes/12011_ti_tui_canted_coax"
SITL_AIRFRAME = ROOT / "ROMFS/px4fmu_common/init.d-posix/airframes/12007_gz_ti_tui"
N_ROTORS = 8


class Rotor(base.Rotor):
    """A planetary_hex rotor whose single propulsion group is cfg['rotor'].

    No rotor is on the axis, so the base class treats all eight as 'outer' (the
    reference group); the lower rotors (positive FRD z) get the wash effectiveness.
    """

    def __init__(self, index: int, hw: str, sitl: str, cfg: dict):
        super().__init__(index, hw, sitl, {"outer": cfg["rotor"], "sun": cfg["rotor"]})
        if self.central:
            sys.exit("ti_tui rotors must be off the vehicle axis")
        if self.position[2] > 0:
            self.effectiveness = cfg["lower_rotor_effectiveness"]


def update_airframe(hw: str, rotors: list[Rotor], values: dict[str, float], cts: list[float], effectiveness: float) -> str:
    for r, ct in zip(rotors, cts):
        hw = set_param(hw, f"CA_ROTOR{r.index}_CT", repr(round(ct, 6)))
        hw = set_param(hw, f"CA_ROTOR{r.index}_KM", f"{r.km + 0.0:.8f}")
    fmt = {"THR_MDL_FAC": "{:.6f}", "MPC_THR_HOVER": "{:.6f}", "MPC_MR_MASS": "{:.4f}"}
    for name, value in values.items():
        # MPC_MR_* only exist on branches that carry the MRAC controller.
        hw = set_param(hw, name, fmt.get(name, "{:.4f}").format(value),
                       required=name in ("THR_MDL_FAC", "MPC_THR_HOVER"))
    eff = f"{effectiveness:g}"
    hw = re.sub(r"[\d.]+ lower-\n# rotor effectiveness", f"{eff} lower-\n# rotor effectiveness", hw)
    return re.sub(r"documented [\d.]+ coaxial-wash", f"documented {eff} coaxial-wash", hw)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--dry-run", action="store_true", help="print the diff instead of writing files")
    args = parser.parse_args()

    cfg = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    hw, sitl = HW_AIRFRAME.read_text(encoding="utf-8"), SITL_AIRFRAME.read_text(encoding="utf-8")
    rotors = [Rotor(i, hw, sitl, cfg) for i in range(N_ROTORS)]
    values, cts = calibration(rotors, cfg["mass_kg"])

    model = update_model_sdf(MODEL_SDF.read_text(encoding="utf-8"), rotors)
    outputs = {
        **body_sdfs(cfg, cfg["rotor"]["rotor_mass_kg"] * N_ROTORS, model, BASE_SDF, ROD_SDF),
        MODEL_SDF: model,
        HW_AIRFRAME: update_airframe(hw, rotors, values, cts, cfg["lower_rotor_effectiveness"]),
    }
    changed = write_outputs(outputs, args.dry_run)

    print("CA_ROTOR_CT:", " ".join(f"{ct:.4f}" for ct in cts))
    print(f"THR_MDL_FAC={values['THR_MDL_FAC']:.6f}  MPC_THR_HOVER={values['MPC_THR_HOVER']:.6f}")
    if not changed:
        print("no changes (files already match the YAML)")
    elif not args.dry_run:
        print("Rebuild so the airframe is baked into ROMFS: make px4_sitl_default gz_ti_tui")


if __name__ == "__main__":
    main()
