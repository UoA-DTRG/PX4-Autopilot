#!/usr/bin/env python3
"""Apply planetary_hex_properties.yaml to the planetaryHex Gazebo models and PX4 airframe.

Edits in place (no CAD needed):
  Tools/simulation/gz/models/planetary_hex_base/model.sdf   base mass and inertia
  Tools/simulation/gz/models/planetary_hex/model.sdf        rotor link inertia, motor constants
  ROMFS/px4fmu_common/init.d/airframes/12018_dtrg_planetary_hex
                                                            CA_ROTOR*_CT/KM, THR_MDL_FAC,
                                                            MPC_THR_HOVER (and MPC_MR_* if present)

Rotor geometry (CA_ROTOR*_P/A) and rotor speed limits (SIM_GZ_EC_MIN/MAX in
12019_gz_planetary_hex) are read from the airframes, not changed. The allocation
maths is taken from generate_planetary_hex_model.py.

    python3 Tools/dtrg/vehicle_properties/planetary_hex_properties.py            # apply
    python3 Tools/dtrg/vehicle_properties/planetary_hex_properties.py --dry-run  # show what would change
"""

from __future__ import annotations

import argparse
import difflib
import math
import re
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
CONFIG = Path(__file__).with_suffix(".yaml")
BASE_SDF = ROOT / "Tools/simulation/gz/models/planetary_hex_base/model.sdf"
MODEL_SDF = ROOT / "Tools/simulation/gz/models/planetary_hex/model.sdf"
HW_AIRFRAME = ROOT / "ROMFS/px4fmu_common/init.d/airframes/12018_dtrg_planetary_hex"
SITL_AIRFRAME = ROOT / "ROMFS/px4fmu_common/init.d-posix/airframes/12019_gz_planetary_hex"

GRAVITY = 9.80665
N_ROTORS = 8


def param(text: str, name: str) -> float:
    match = re.search(rf"^param set-default {name} (\S+)$", text, re.M)
    if not match:
        sys.exit(f"{name} not found")
    return float(match.group(1))


def set_param(text: str, name: str, value: str, required: bool = True) -> str:
    text, count = re.subn(rf"^(param set-default {name} )\S+$", rf"\g<1>{value}", text, flags=re.M)
    if required and count != 1:
        sys.exit(f"{name} not found exactly once")
    return text


class Rotor:
    def __init__(self, index: int, hw: str, sitl: str, cfg: dict):
        self.index = index
        self.position = np.array([param(hw, f"CA_ROTOR{index}_P{c}") for c in "XYZ"])
        self.axis = np.array([param(hw, f"CA_ROTOR{index}_A{c}") for c in "XYZ"])
        self.km_sign = 1.0 if param(hw, f"CA_ROTOR{index}_KM") > 0 else -1.0
        self.omega_min = param(sitl, f"SIM_GZ_EC_MIN{index + 1}")
        self.omega_max = param(sitl, f"SIM_GZ_EC_MAX{index + 1}")
        self.central = math.hypot(*self.position[:2]) < 0.05
        self.p = cfg["sun" if self.central else "outer"]
        self.effectiveness = 1.0  # set for the lower coaxial rotor by the caller

    @property
    def km(self) -> float:
        return self.km_sign * self.p["moment_constant"]

    @property
    def motor_constant(self) -> float:
        return self.p["thrust_constant"] * self.effectiveness

    @property
    def incremental_force(self) -> float:
        return self.motor_constant * (self.omega_max ** 2 - self.omega_min ** 2)


def build_rotors(cfg: dict, hw: str, sitl: str) -> list[Rotor]:
    rotors = [Rotor(i, hw, sitl, cfg) for i in range(N_ROTORS)]
    central = [r for r in rotors if r.central]
    if len(central) != 2:
        sys.exit("expected exactly two central rotors")
    # PX4 FRD: larger Z is lower, so the lower sun rotor has the largest Z.
    max(central, key=lambda r: r.position[2]).effectiveness = cfg["lower_coax_effectiveness"]
    return rotors


def reference_force(rotors: list[Rotor]) -> float:
    outer = next(r for r in rotors if not r.central)
    return outer.p["thrust_constant"] * (outer.omega_max ** 2 - outer.omega_min ** 2)


def wrench_columns(rotors: list[Rotor]) -> np.ndarray:
    return np.column_stack([
        np.concatenate((np.cross(r.position, r.axis) - r.km * r.axis, r.axis)) for r in rotors])


def allocation_scale(effectiveness: np.ndarray) -> np.ndarray:
    """PX4 pseudoinverse allocation-normalization (from generate_planetary_hex_model.py)."""
    mixer = np.linalg.pinv(effectiveness)
    scale = np.ones(6)
    roll_nonzero = np.count_nonzero(np.abs(mixer[:, 0]) > 1e-3)
    pitch_nonzero = np.count_nonzero(np.abs(mixer[:, 1]) > 1e-3)
    roll_scale = math.sqrt(np.dot(mixer[:, 0], mixer[:, 0]) / (roll_nonzero / 2.0))
    pitch_scale = math.sqrt(np.dot(mixer[:, 1], mixer[:, 1]) / (pitch_nonzero / 2.0))
    scale[0] = scale[1] = max(roll_scale, pitch_scale)
    scale[2] = np.max(mixer[:, 2])
    for axis in (2, 1, 0):
        nonzero = np.abs(mixer[:, 3 + axis]) > np.finfo(float).eps
        scale[3 + axis] = np.mean(np.abs(mixer[nonzero, 3 + axis]))
    return scale


def calibration(rotors: list[Rotor], mass: float) -> tuple[dict[str, float], list[float]]:
    ref = reference_force(rotors)
    cts = [r.incremental_force / ref for r in rotors]
    wrench = wrench_columns(rotors)
    scale = allocation_scale(wrench * np.array(cts))
    idle = np.array([r.motor_constant * r.omega_min ** 2 for r in rotors])
    offset = (wrench @ idle)[3:]
    force_scale = ref / scale[3:]
    outer = next(r for r in rotors if not r.central)
    result = {
        "THR_MDL_FAC": (outer.omega_max - outer.omega_min) / (outer.omega_max + outer.omega_min),
        "MPC_THR_HOVER": (mass * GRAVITY + offset[2]) / force_scale[2],
        "MPC_MR_MASS": mass,
    }
    for axis, suffix in enumerate("XYZ"):
        result[f"MPC_MR_F{suffix}_SCL"] = force_scale[axis]
        result[f"MPC_MR_F{suffix}_OFF"] = offset[axis] + 0.0
    return result, cts


def update_airframe(hw: str, rotors: list[Rotor], values: dict[str, float], cts: list[float]) -> str:
    for r, ct in zip(rotors, cts):
        hw = set_param(hw, f"CA_ROTOR{r.index}_CT", f"{ct:.6f}")
        hw = set_param(hw, f"CA_ROTOR{r.index}_KM", f"{r.km + 0.0:.8f}")
    fmt = {"THR_MDL_FAC": "{:.6f}", "MPC_THR_HOVER": "{:.6f}", "MPC_MR_MASS": "{:.4f}"}
    for name, value in values.items():
        # MPC_MR_* only exist on branches that carry the MRAC controller.
        hw = set_param(hw, name, fmt.get(name, "{:.4f}").format(value),
                       required=name in ("THR_MDL_FAC", "MPC_THR_HOVER"))
    return hw


def update_base_sdf(sdf: str, cfg: dict, rotor_mass: float) -> str:
    """Set the base link mass (total minus rotor links) and inertia."""
    base_mass = cfg["mass_kg"] - rotor_mass
    if base_mass <= 0:
        sys.exit(f"mass_kg {cfg['mass_kg']} is not larger than the rotor links ({rotor_mass:.3f} kg)")
    j = cfg["base_inertia_kgm2"]
    want = {"mass": (base_mass, ".3f"), **{k: (j[k], ".6f") for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}}

    def inertial(match: re.Match) -> str:
        # Keep the existing text for unchanged values so number formatting does not churn.
        def val(tag: str) -> str:
            old = re.search(rf"<{tag}>([^<]*)</{tag}>", match.group(0)).group(1)
            return old if float(old) == float(format(want[tag][0], want[tag][1])) else format(want[tag][0], want[tag][1])
        return (f"<inertial><mass>{val('mass')}</mass><inertia><ixx>{val('ixx')}</ixx><ixy>{val('ixy')}</ixy>"
                f"<ixz>{val('ixz')}</ixz><iyy>{val('iyy')}</iyy><iyz>{val('iyz')}</iyz>"
                f"<izz>{val('izz')}</izz></inertia></inertial>")

    sdf, count = re.subn(r"<inertial>.*?</inertial>", inertial, sdf, count=1, flags=re.S)
    if count != 1:
        sys.exit("base inertial block not found")
    return re.sub(r"Mass is ASSUMED \([^)]*\)", f"Mass is ASSUMED ({float(cfg['mass_kg'])} kg total)", sdf)


def update_model_sdf(sdf: str, rotors: list[Rotor]) -> str:
    """Set each rotor link's inertia and its motor plugin constants."""
    for r in rotors:
        p = r.p
        want = {"mass": p["rotor_mass_kg"], "ixx": p["rotor_ixx"], "iyy": p["rotor_ixx"], "izz": p["rotor_izz"]}

        def inertial(match: re.Match) -> str:
            # Keep the existing text for unchanged values so number formatting does not churn.
            def val(tag: str) -> str:
                old = re.search(rf"<{tag}>([^<]*)</{tag}>", match.group(2)).group(1)
                return old if float(old) == want[tag] else f"{want[tag]:g}"
            return (f"{match.group(1)}<inertial><mass>{val('mass')}</mass><inertia><ixx>{val('ixx')}</ixx>"
                    f"<iyy>{val('iyy')}</iyy><izz>{val('izz')}</izz></inertia></inertial>")

        link = re.compile(rf'(<link name="rotor_{r.index}">.*?)<inertial>(.*?)</inertial>', re.S)
        sdf, count = link.subn(inertial, sdf, count=1)
        plugin = re.compile(rf"(<jointName>rotor_{r.index}_joint</jointName>.*?)"
                            r"<timeConstantUp>[^<]*</timeConstantUp><timeConstantDown>[^<]*</timeConstantDown>"
                            r"(.*?)<motorConstant>[^<]*</motorConstant><momentConstant>[^<]*</momentConstant>", re.S)
        sdf, pcount = plugin.subn(
            lambda m: (f"{m.group(1)}<timeConstantUp>{p['time_constant_s']}</timeConstantUp>"
                       f"<timeConstantDown>{p['time_constant_s']}</timeConstantDown>{m.group(2)}"
                       f"<motorConstant>{r.motor_constant:.10g}</motorConstant>"
                       f"<momentConstant>{p['moment_constant']:.8f}</momentConstant>"), sdf, count=1)
        if count != 1 or pcount != 1:
            sys.exit(f"rotor_{r.index} link or motor plugin not found")
    return sdf


def write_outputs(outputs: dict[Path, str], dry_run: bool) -> bool:
    """Write (or diff, if dry_run) each file; return True if anything differs."""
    changed = False
    for path, new in outputs.items():
        old = path.read_text(encoding="utf-8")
        if new == old:
            continue
        changed = True
        if dry_run:
            name = str(path.relative_to(ROOT))
            sys.stdout.writelines(difflib.unified_diff(old.splitlines(True), new.splitlines(True), name, name, n=0))
        else:
            path.write_text(new, encoding="utf-8")
            print(f"updated {path.relative_to(ROOT)}")
    return changed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--dry-run", action="store_true", help="print the diff instead of writing files")
    args = parser.parse_args()

    cfg = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    hw, sitl = HW_AIRFRAME.read_text(encoding="utf-8"), SITL_AIRFRAME.read_text(encoding="utf-8")
    rotors = build_rotors(cfg, hw, sitl)
    values, cts = calibration(rotors, cfg["mass_kg"])

    outputs = {
        BASE_SDF: update_base_sdf(BASE_SDF.read_text(encoding="utf-8"), cfg,
                              sum(r.p["rotor_mass_kg"] for r in rotors)),
        MODEL_SDF: update_model_sdf(MODEL_SDF.read_text(encoding="utf-8"), rotors),
        HW_AIRFRAME: update_airframe(hw, rotors, values, cts),
    }
    changed = write_outputs(outputs, args.dry_run)

    print("CA_ROTOR_CT:", " ".join(f"{ct:.4f}" for ct in cts))
    print(f"THR_MDL_FAC={values['THR_MDL_FAC']:.6f}  MPC_THR_HOVER={values['MPC_THR_HOVER']:.6f}")
    if not changed:
        print("no changes (files already match the YAML)")
    elif not args.dry_run:
        print("Rebuild so the airframe is baked into ROMFS: make px4_sitl_default gz_planetary_hex_mocap")


if __name__ == "__main__":
    main()
