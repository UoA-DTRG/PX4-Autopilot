#!/usr/bin/env python3
"""Apply planar_octo_properties.yaml to the planarOcto Gazebo models and PX4 airframe.

Edits in place (no CAD needed):
  Tools/simulation/gz/models/planar_octo_base/model.sdf   base mass and inertia
  Tools/simulation/gz/models/planar_octo/model.sdf        rotor link inertia, motor constants
  ROMFS/px4fmu_common/init.d/airframes/12013_dtrg_planar_octo
                                                          CA_ROTOR*_KM, THR_MDL_FAC, MPC_THR_HOVER

All eight rotors are identical, so CA_ROTOR*_CT stays 1.0. Rotor geometry
(CA_ROTOR*_P/A) and rotor speed limits (SIM_GZ_EC_MIN/MAX in 12014_gz_planar_octo)
are read from the airframes, not changed. Shares its SDF editing with
planetary_hex_properties.py.

    python3 Tools/dtrg/vehicle_properties/planar_octo_properties.py            # apply
    python3 Tools/dtrg/vehicle_properties/planar_octo_properties.py --dry-run  # show what would change
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

from planetary_hex_properties import (GRAVITY, ROOT, param, set_param, update_base_sdf,
                                      update_model_sdf, write_outputs)

CONFIG = Path(__file__).with_suffix(".yaml")
BASE_SDF = ROOT / "Tools/simulation/gz/models/planar_octo_base/model.sdf"
MODEL_SDF = ROOT / "Tools/simulation/gz/models/planar_octo/model.sdf"
HW_AIRFRAME = ROOT / "ROMFS/px4fmu_common/init.d/airframes/12013_dtrg_planar_octo"
SITL_AIRFRAME = ROOT / "ROMFS/px4fmu_common/init.d-posix/airframes/12014_gz_planar_octo"
N_ROTORS = 8


@dataclass
class Rotor:
    index: int
    p: dict
    axis_z: float
    km_sign: float
    omega_min: float
    omega_max: float

    @property
    def motor_constant(self) -> float:
        return self.p["thrust_constant"]


def build_rotors(cfg: dict, hw: str, sitl: str) -> list[Rotor]:
    return [Rotor(i, cfg["rotor"], param(hw, f"CA_ROTOR{i}_AZ"),
                  1.0 if param(hw, f"CA_ROTOR{i}_KM") > 0 else -1.0,
                  param(sitl, f"SIM_GZ_EC_MIN{i + 1}"), param(sitl, f"SIM_GZ_EC_MAX{i + 1}"))
            for i in range(N_ROTORS)]


def hover(rotors: list[Rotor], mass: float) -> tuple[float, float]:
    """THR_MDL_FAC and MPC_THR_HOVER for identical rotors sharing the weight equally."""
    r = rotors[0]
    if any((q.omega_min, q.omega_max) != (r.omega_min, r.omega_max) for q in rotors):
        sys.exit("rotors must share one speed range")
    factor = (r.omega_max - r.omega_min) / (r.omega_max + r.omega_min)
    thrust = mass * GRAVITY / sum(-q.axis_z for q in rotors)  # axes point up: FRD z < 0
    out = (math.sqrt(thrust / r.motor_constant) - r.omega_min) / (r.omega_max - r.omega_min)
    return factor, (1.0 - factor) * out + factor * out ** 2


def update_airframe(hw: str, rotors: list[Rotor], mass: float) -> str:
    factor, thr_hover = hover(rotors, mass)
    for r in rotors:
        hw = set_param(hw, f"CA_ROTOR{r.index}_KM", f"{r.km_sign * r.p['moment_constant']:.8f}")
    hw = set_param(hw, "THR_MDL_FAC", f"{factor:.6f}")
    hw = set_param(hw, "MPC_THR_HOVER", f"{thr_hover:.6f}")
    r = rotors[0]
    return re.sub(r"is assumed \([\d.]+ kg\) and the propulsion model \(kF=[^,]+,",
                  f"is assumed ({float(mass)} kg) and the propulsion model (kF={r.motor_constant:g},", hw)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--dry-run", action="store_true", help="print the diff instead of writing files")
    args = parser.parse_args()

    cfg = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    hw, sitl = HW_AIRFRAME.read_text(encoding="utf-8"), SITL_AIRFRAME.read_text(encoding="utf-8")
    rotors = build_rotors(cfg, hw, sitl)
    outputs = {
        BASE_SDF: update_base_sdf(BASE_SDF.read_text(encoding="utf-8"), cfg,
                                  cfg["rotor"]["rotor_mass_kg"] * N_ROTORS),
        MODEL_SDF: update_model_sdf(MODEL_SDF.read_text(encoding="utf-8"), rotors),
        HW_AIRFRAME: update_airframe(hw, rotors, cfg["mass_kg"]),
    }
    changed = write_outputs(outputs, args.dry_run)

    factor, thr_hover = hover(rotors, cfg["mass_kg"])
    print(f"THR_MDL_FAC={factor:.6f}  MPC_THR_HOVER={thr_hover:.6f}")
    if not changed:
        print("no changes (files already match the YAML)")
    elif not args.dry_run:
        print("Rebuild so the airframe is baked into ROMFS: make px4_sitl_default gz_planar_octo_mocap")


if __name__ == "__main__":
    main()
