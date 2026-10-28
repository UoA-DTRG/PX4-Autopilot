# DTRG SITL

This page covers the simulated DTRG vehicles: the **planarOcto**, a flat
octocopter whose eight rotors are tilted 31° towards the tangent of the frame
so it can push sideways without tilting (fully actuated), and the
**planetaryHex**, a six-rotor ring with a central coaxial pair, and the
**Ti-Tui** (stacked octo), a compact octocopter with four upper and four lower
canted rotors. It describes Gazebo, SIH on the PC (used by the automated tests), SIH on the flight
controller, and HITL, plus the **interaction rod** for physical-contact tests,
the **SIH generic multirotor** that can simulate any rotor geometry PX4 can
describe, and the tool for changing mass, inertia and thrust constants between
runs.

| | planarOcto | planetaryHex | Ti-Tui (stacked octo) |
| --- | --- | --- | --- |
| Airframes | `12013_dtrg_planar_octo` (the vehicle), `12014_gz_planar_octo`, `12016_sihsim_planar_octo` (SITL), `12018_gz_planar_octo_rod` (SITL with the interaction rod), `12015_dtrg_planar_octo_hitl.hil`, `12017_dtrg_planar_octo_sih.hil` (hardware) | `12018_dtrg_planetary_hex` (the vehicle), `12019_gz_planetary_hex` (Gazebo SITL), `12020_gz_planetary_hex_rod` (SITL with the interaction rod) | `12011_ti_tui_canted_coax` (the vehicle), `12007_gz_ti_tui` (Gazebo SITL), `12021_gz_ti_tui_rod` (SITL with the interaction rod) |
| Gazebo model | `Tools/simulation/gz/models/planar_octo` (+ `planar_octo_base`, `planar_octo_rod`) | `Tools/simulation/gz/models/planetary_hex` (+ `planetary_hex_base`, `planetary_hex_rod`) | `Tools/simulation/gz/models/ti_tui` (+ `ti_tui_base`, `ti_tui_rod`) |
| Simulators | Gazebo, SIH, HITL | Gazebo | Gazebo |
| Properties tool | `Tools/dtrg/vehicle_properties/planar_octo_properties.py` | `Tools/dtrg/vehicle_properties/planetary_hex_properties.py` | `Tools/dtrg/vehicle_properties/ti_tui_properties.py` |

The Gazebo models live in the submodule `Tools/simulation/gz`
(`UoA-DTRG/PX4-gazebo-models`, branch `dtrg-main`). SIH is
`src/modules/simulation/simulator_sih` (`SIH_VEHICLE_TYPE 6`,
`SIH_THR_MDL_FAC`). Scripts are in `Tools/dtrg/`: `keyboard_teleop.py`,
`rc_override.py`, `compare_sih_gz.py`, `log_interaction_force.py`.

---

## 1. Quick start

```sh
# Gazebo (needs Gazebo Harmonic), default world
make px4_sitl gz_planar_octo

# Gazebo in another world from Tools/simulation/gz/worlds, e.g. the mocap room
make px4_sitl gz_planar_octo_mocap

# SIH: no Gazebo, no GUI, fast; this is what the automated tests fly
make px4_sitl sihsim_planar_octo

# planetaryHex in Gazebo (there is no SIH variant of it yet)
make px4_sitl gz_planetary_hex
make px4_sitl gz_planetary_hex_mocap

# Ti-Tui (stacked octo) in Gazebo
make px4_sitl gz_ti_tui
make px4_sitl gz_ti_tui_mocap

# any of the three with the interaction rod, flying against the push panel
make px4_sitl gz_planar_octo_rod_mocap_interaction
make px4_sitl gz_planetary_hex_rod_mocap_interaction
make px4_sitl gz_ti_tui_rod_mocap_interaction
```

`gz_<model>_<world>` is generated for every `*_gz_*` airframe and every world
in `Tools/simulation/gz/worlds`, so `gz_planetary_hex_mocap` needs both the
airframe `12019_gz_planetary_hex` and the world `mocap.sdf`. The same holds for
`gz_ti_tui_*` and `12007_gz_ti_tui`.

Then in a second terminal fly it from the keyboard:

```sh
pip install pymavlink
python3 Tools/dtrg/keyboard_teleop.py            # listens on udpin:0.0.0.0:14540
```

| Key | |
| --- | --- |
| `m` / `n` | arm / disarm |
| `t` / `g` | take off / land |
| `p` / `h` | Position / Hold mode |
| `i` `k` | pitch stick forward / back |
| `j` `l` | roll stick left / right |
| `a` `d` | yaw left / right |
| `w` `s` | throttle up / down (holds); space centres the sticks, throttle mid |
| `5`–`8`, `0` | cycle aux 1–4 (low / mid / high), all aux low |
| `e` | toggle the HT switch (RC channel 8) |
| `z` `x` | HT roll knob − / + (RC channel 9) |
| `c` `v` | HT pitch knob − / + (RC channel 10) |
| `b` | centre both HT knobs |
| `q` | quit |

The sticks spring back to centre; the HT knobs hold. QGroundControl connects
on UDP 14550 as usual and shows the vehicle.

### Why a script

Horizontal thrust reads `rc_channels` (the HT switch and knobs are RC channels,
see [HT mode](DTRG_Horizontal_Thrust_Mode.md)). SITL has no RC receiver, and a
QGroundControl joystick sends `MANUAL_CONTROL`, which never reaches
`rc_channels`. The scripts additionally send `RC_CHANNELS_OVERRIDE`, which PX4
turns into `input_rc` and then `rc_channels`:

| Script | Sends | Use |
| --- | --- | --- |
| `keyboard_teleop.py` | `MANUAL_CONTROL` (sticks, aux 1–4) and, after the first HT key, `RC_CHANNELS_OVERRIDE` (channels 8–10) | fly by hand |
| `rc_override.py` | `RC_CHANNELS_OVERRIDE` only | hold fixed channel values, e.g. `Tools/dtrg/rc_override.py --norm 8=1 --norm 9=0.5`; `--help` for the rest |

Run only one of them at a time. Both send `RC_CHANNELS_OVERRIDE` and would
override each other. See [Tools/dtrg/README.md](../../Tools/dtrg/README.md).

### Changing parameters at launch

`PX4_PARAM_<name>=<value>` in the environment sets a parameter at start-up and
wins over the airframe's defaults (the DTRG `rcS` applies them again after the
airframe script, because a value equal to the firmware default would
otherwise be lost):

```sh
PX4_PARAM_DTRG_HT_MASK=1 PX4_PARAM_DTRG_HT_MAX=0.3 make px4_sitl sihsim_planar_octo
```

---

## 2. The airframes

### planarOcto

| ID | Name | Where | What it adds |
| --- | --- | --- | --- |
| 12013 | DTRG planarOcto | real vehicle | geometry (`CA_ROTOR*`), flown gains, thrust model, outputs AUX1–8 = Motors 1–8, `DTRG_HT_EN 1`, `RC_MAP_HT_MODE 8` |
| 12014 | Gazebo DTRG planarOcto | SITL, Gazebo | 12013 + Gazebo ESC mapping (`SIM_GZ_EC_FUNC1-8`, 350–1774 rad/s), SITL gains, `RC_MAP_HT_ROLL 9`, `RC_MAP_HT_PITCH 10` |
| 12016 | SIH DTRG planarOcto | SITL, SIH | 12013 + SIH vehicle model (below), outputs 1–8 = Motors 1–8, SITL gains, HT knobs 9/10 |
| 12015 | DTRG planarOcto HITL | flight controller, HITL | 12013 + `SYS_HITL 1`, `HIL_ACT_FUNC1-8` = Motors 1–8 for an external HITL simulator |
| 12018 | Gazebo DTRG planarOcto with interaction rod | SITL, Gazebo | 12014 with `PX4_SIM_MODEL planar_octo_rod`; allocation and gains unchanged (see [section 8](#8-the-interaction-rod)) |
| 12017 | DTRG planarOcto SIH | flight controller, SIH | 12013 + `SYS_HITL 2`: the real flight controller flies a planarOcto simulated on the board itself. No PC needed. Real outputs stay off (remove the props anyway) |

The number 12018 is also used by the planetaryHex vehicle airframe. They live
in different directories (`init.d-posix` for SITL, `init.d` for hardware): SITL
selects its airframe by model name from `init.d-posix`, so it is unambiguous
there, but on a board `SYS_AUTOSTART 12018` is the planetaryHex.

All of them source 12013, so a change to the geometry or the gains there
reaches every variant. The SITL airframes set `PLANAR_OCTO_NATIVE_GZ_SITL=1`
before sourcing it, which skips the hardware output mapping.

The SITL variants raise the rate P/I gains (×4) and lower the attitude P
(4.5) compared with the flown gains of 12013: with the flown gains the
simulated vehicle wobbles at ~1.1 Hz. These are simulation gains only, tuned
in SIH. See [Tools/dtrg/SIH_VS_GAZEBO.md](SIH_VS_GAZEBO.md).

To fly the SIH-on-hardware variant: flash a board whose build includes SIH
(`CONFIG_MODULES_SIMULATION_SIMULATOR_SIH`, e.g. `px4_fmu-v5_default`,
`px4_fmu-v6c_default`), set `SYS_AUTOSTART 12017`, reboot, connect
QGroundControl over USB and fly with your transmitter.

### planetaryHex

| ID | Name | Where | What it adds |
| --- | --- | --- | --- |
| 12018 | DTRG planetaryHex | real vehicle | geometry (`CA_ROTOR*`), flown rate/attitude/position gains, thrust model, outputs AUX1–8 = Motors 1–8 |
| 12019 | Gazebo DTRG planetaryHex | SITL, Gazebo | 12018 + Gazebo ESC mapping (`SIM_GZ_EC_FUNC1-8`, separate speed limits for outer and central rotors), `MPC_XY_VEL_P_ACC 1.8`, HT knobs 9/10 |
| 12020 | Gazebo DTRG planetaryHex with interaction rod | SITL, Gazebo | 12019 with `PX4_SIM_MODEL planetary_hex_rod`; allocation and gains unchanged (see [section 8](#8-the-interaction-rod)) |

12019 sets `PLANETARY_HEX_NATIVE_GZ_SITL=1` before sourcing 12018, which skips
the hardware output mapping (same scheme as the planarOcto).

12019 lowers `MPC_XY_VEL_P_ACC` from the flown 11 to 1.8. With 11 the
simulated vehicle holds height but limit-cycles ±30–40° in roll and pitch and
saturates the outer rotors; PX4's default-style gain holds a level hover. This
is tuned in Gazebo, not in flight.

### Ti-Tui

| ID | Name | Where | What it adds |
| --- | --- | --- | --- |
| 12011 | DTRG Ti-Tui Canted Coaxial | real vehicle | geometry (`CA_ROTOR*`), conservative rate/position gains inherited from the R1 baseline, thrust model, `HIL_ACT_FUNC1-8` = Motors 1–8, `DTRG_HT_EN 1`, `RC_MAP_HT_MODE 8` |
| 12007 | Gazebo DTRG Ti-Tui Canted Coaxial | SITL, Gazebo | 12011 + Gazebo ESC mapping (`SIM_GZ_EC_FUNC1-8`, 350–1774 rad/s), `RC_MAP_HT_ROLL 9`, `RC_MAP_HT_PITCH 10` |
| 12021 | Gazebo DTRG Ti-Tui with interaction rod | SITL, Gazebo | 12007 with `PX4_SIM_MODEL ti_tui_rod`; allocation and gains unchanged (see [section 8](#8-the-interaction-rod)) |

12007 sets `TI_TUI_NATIVE_GZ_SITL=1` before sourcing 12011, which skips the
`HIL_ACT_FUNC` mapping (same scheme as the other vehicles). The upstream Ti-Tui
airframes also set the MRAC position-controller parameters (`MPC_POS_CTRL`,
`MPC_ACT_MODE`, `MPC_MR_*`). Those parameters do not exist on this branch, so
they were left out of 12011, as for the planetaryHex. The MRAC SITL variants
(12008, 12009, 12012) are not included.

---

## 3. The planarOcto vehicle

### Geometry

Rotor centres from CAD, thrust axes tilted 31° from vertical, body FRD
(metres). All rotors: $p_z = -0.0524$, $a_z = -\cos 31° = -0.857$,
`CT = 1`, $|$`KM`$| = 0.00858$.

| Motor | `CA_ROTORn` | $p_x$ | $p_y$ | $a_x$ | $a_y$ | `KM` | Spin |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 0 | 0.2310 | 0.0742 | 0 | −0.515 | −0.00858 | CW |
| 2 | 1 | −0.2310 | −0.0742 | 0 | 0.515 | −0.00858 | CW |
| 3 | 2 | 0.0742 | 0.2310 | −0.515 | 0 | 0.00858 | CCW |
| 4 | 3 | −0.2310 | 0.0742 | 0 | −0.515 | 0.00858 | CCW |
| 5 | 4 | 0.2310 | −0.0742 | 0 | 0.515 | 0.00858 | CCW |
| 6 | 5 | −0.0742 | −0.2310 | 0.515 | 0 | 0.00858 | CCW |
| 7 | 6 | 0.0742 | −0.2310 | −0.515 | 0 | −0.00858 | CW |
| 8 | 7 | −0.0742 | 0.2310 | 0.515 | 0 | −0.00858 | CW |

($0.515 = \sin 31°$.) Four rotors lean along body X and four along body Y, so
the vehicle can push in both horizontal directions.

### Mass and propulsion

Provisional values: the mass is assumed and the propulsion is inherited from
the FT-X8 reference, not measured on the planarOcto. Gazebo and SIH use the
same numbers.

| | Value | |
| --- | --- | --- |
| Mass | 1.5 kg | 1.46 kg body + 8 × 5 g rotors |
| Inertia $I_{xx}, I_{yy}, I_{zz}$ | 0.0254, 0.0254, 0.0420 kg m² | body + rotors as point masses |
| Motor constant | $k_F = 1.2138\times10^{-6}$ N/(rad/s)² | |
| Rotor speed | 350–1774 rad/s | |
| Max thrust per rotor | $T_{max} = k_F\,\omega_{max}^2 = 3.82$ N | `SIH_T_MAX` |
| Thrust curve | `THR_MDL_FAC` = `SIH_THR_MDL_FAC` = 0.670 | |
| Motor time constant | 0.03 s | `SIH_T_TAU` |
| Hover thrust | `MPC_THR_HOVER` = 0.544 | |

Sanity check of the hover thrust: at full output the eight rotors give
$8 \times 3.82 \times \cos 31° = 26.2$ N upwards against a weight of
$1.5 \times 9.81 = 14.7$ N, so hover needs $14.7 / 26.2 = 0.56$ of full
thrust. The tilt costs $1 - \cos 31° = 14\%$ of the vertical thrust, in
exchange for the sideways authority.

---

## 4. The planetaryHex vehicle

Six outer rotors on a ring, each tilted 47° and leaning mainly tangentially,
plus a central (sun) coaxial pair. Body FRD, metres, origin on the sun axis at
the centroid height of the stationary CAD. Forward (+X) lies in the gap
between motors 1 and 6, whose thrust leans inward towards each other; motors 2
and 5 sit at the sides and thrust straight back. The frame is mirror
symmetric about X. This is the CAD frame rotated 30° (until 28/10/2026 motor 1
pointed forward), so on the real vehicle the flight controller's forward
arrow must point into the motor 1–6 gap, or `SENS_BOARD_Z_OFF` must carry the
mounting yaw.

### Geometry

| Motor | `CA_ROTORn` | Position | $p_x$ | $p_y$ | $p_z$ | `KM` | Spin |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 0 | outer, −28.3° | 0.2645 | −0.1425 | −0.0190 | −0.012 | CW |
| 2 | 1 | outer, −91.7° | −0.0088 | −0.3003 | −0.0190 | 0.012 | CCW |
| 3 | 2 | outer, −148.3° | −0.2557 | −0.1578 | −0.0190 | −0.012 | CW |
| 4 | 3 | outer, +148.3° | −0.2557 | 0.1578 | −0.0190 | 0.012 | CCW |
| 5 | 4 | outer, +91.7° | −0.0088 | 0.3003 | −0.0190 | −0.012 | CW |
| 6 | 5 | outer, +28.3° | 0.2645 | 0.1425 | −0.0190 | 0.012 | CCW |
| 7 | 6 | centre, upper | 0 | 0 | −0.0826 | 0.020 | CCW |
| 8 | 7 | centre, lower | 0 | 0 | 0.0619 | −0.020 | CW |

The outer thrust axes have $a_z \approx -0.682$ (47° from vertical) and a
horizontal part that is almost entirely tangential, with a 1° radial component.
The central axes are straight up, $a = (0, 0, -1)$. Spin comes from the STEP
assembly (`*_propeller` = CCW, `*_propeller_MIR` = CW) and the handedness is
assumed. Check the propellers on the real vehicle.

### Mass and propulsion

Provisional values: the mass, the battery and both propulsion models are
assumed, not measured. Gazebo and the controller use the same numbers.

| | Value | |
| --- | --- | --- |
| Mass | 3.0 kg | 2.892 kg base + 6 × 8 g outer rotors + 2 × 30 g central rotors |
| Base inertia $I_{xx}, I_{yy}, I_{zz}$ | 0.0574, 0.0572, 0.1066 kg m² | uniform-density CAD, scaled to the base mass |
| Outer propulsion | $k_F = 1.0\times10^{-6}$, $k_M/k_F = 0.012$ m, 600–3000 rad/s, $\tau = 0.05$ s | 5.1 in propeller, about 9 N at full speed |
| Central propulsion | $k_F = 1.6\times10^{-5}$, $k_M/k_F = 0.020$ m, 190–950 rad/s, $\tau = 0.12$ s | 13 in propeller, about 14.4 N at full speed |
| Lower coaxial effectiveness | 0.8 | the lower central rotor works in the upper one's wash |
| `CA_ROTORn_CT` | outer 1.0, upper centre 1.604, lower centre 1.284 | incremental thrust relative to an outer rotor |
| Thrust curve | `THR_MDL_FAC` = 0.667 | both groups have $\omega_{min}/\omega_{max} = 0.2$, so one value fits every rotor |
| Hover thrust | `MPC_THR_HOVER` = 0.388 | |

At a level hover the outer rotors run at about 0.30–0.31 of their range and
the central rotors at about 0.71 (upper) and 0.57 (lower), so most of the lift
comes from the central pair.

### Open items

- The flown parameters use a 43° tilt, exactly tangential tilt and arms exactly
  60° apart. The CAD gives 47°, a 1° radial component and arms alternating
  56.6°/63.4°. The model follows the CAD. If the arms move in flight, this
  fixed-geometry model represents a single pose.
- The CAD spin direction of the outer rotors is opposite to the sign of the
  flown `KM`. The flown `KM` is only ±0.001, so the tilt signs are what flight
  would have validated. Check the propellers.
- The central-rotor heights are −0.0826 / +0.0619 m in the CAD against
  −0.010 / +0.0065 m in the flown parameters.
- Still to be measured: takeoff and battery mass, centre of mass and full
  inertia, propeller/motor/ESC models, $k_F$, $k_M$, $\tau$, speed limits and
  the rotating inertia.

### Gazebo

The model `planetary_hex` merge-includes `planetary_hex_base` (CAD body,
inertia, collision, sensors) and adds eight rotor links with one
`MulticopterMotorModel` plugin each. The six landing feet are 10 mm collision
spheres at the CAD feet. `SIM_GZ_EC_MIN/MAX` are 600/3000 rad/s for motors 1–6
and 190/950 rad/s for motors 7–8. The CAD meshes are in the original CAD frame;
the model turns the body mesh, the rotor links and the feet 30° so that body
+X is the motor 1–6 gap, matching `CA_ROTOR*` in 12018. The red front marker
sits on +X.

---

## 5. The Ti-Tui vehicle

A compact octocopter built as two stacked layers of four rotors, each rotor
canted 30°. The upper layer leans along body X (motors 1–4) and the lower layer
along body Y (motors 5–8), so the vehicle can push in both horizontal
directions. The lower rotors sit under the upper ones in their wash, which is
modelled as a lower thrust effectiveness. Body FRD, metres.

### Geometry

| Motor | `CA_ROTORn` | Layer | $p_x$ | $p_y$ | $p_z$ | $a_x$ | $a_y$ | `KM` | Spin |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 0 | upper, front-right | 0.0783 | 0.0949 | −0.0471 | −0.5 | 0 | 0.00858 | CCW |
| 2 | 1 | upper, front-left | 0.0783 | −0.0949 | −0.0471 | −0.5 | 0 | −0.00858 | CW |
| 3 | 2 | upper, rear-left | −0.0783 | −0.0949 | −0.0471 | 0.5 | 0 | 0.00858 | CCW |
| 4 | 3 | upper, rear-right | −0.0783 | 0.0949 | −0.0471 | 0.5 | 0 | −0.00858 | CW |
| 5 | 4 | lower, front-left | 0.0955 | −0.1107 | 0.0474 | 0 | 0.5 | 0.00858 | CCW |
| 6 | 5 | lower, front-right | 0.0955 | 0.1107 | 0.0474 | 0 | −0.5 | −0.00858 | CW |
| 7 | 6 | lower, rear-right | −0.0955 | 0.1107 | 0.0474 | 0 | −0.5 | 0.00858 | CCW |
| 8 | 7 | lower, rear-left | −0.0955 | −0.1107 | 0.0474 | 0 | 0.5 | −0.00858 | CW |

All axes have $a_z = -\cos 30° = -0.866$ and the horizontal part is
$\sin 30° = 0.5$. Rotor centres are from CAD. Motors 1–8 follow the order in the
airframe header, which matches the FT-X8 reference.

### Mass and propulsion

Provisional values: the mass is assumed and the propulsion is inherited from the
FT-X8 reference, not measured on the Ti-Tui. All eight rotors are assumed
identical apart from the lower-rotor effectiveness. Gazebo and the controller
use the same numbers.

| | Value | |
| --- | --- | --- |
| Mass | 1.212 kg | 1.172 kg body + 8 × 5 g rotors |
| Base inertia $I_{xx}, I_{yy}, I_{zz}$ | 0.00965, 0.00965, 0.01737 kg m² | |
| Motor constant | $k_F = 1.2138\times10^{-6}$ N/(rad/s)² | upper rotors |
| Lower-rotor effectiveness | 0.85 | lower $k_F$ = 0.85 × upper, `CA_ROTOR4-7_CT` = 0.85 |
| Moment constant | $k_M/k_F = 0.00858$ m | |
| Rotor speed | 350–1774 rad/s | |
| Motor time constant | 0.15 s | slower than the planarOcto's 0.03 s |
| Thrust curve | `THR_MDL_FAC` = 0.670 | |
| Hover thrust | `MPC_THR_HOVER` = 0.462 | |

The upstream Ti-Tui files use a lower-rotor effectiveness of 0.8. This repo uses
0.85 as the default; change `lower_rotor_effectiveness` in the YAML to alter it
(see [section 9](#9-changing-mass-inertia-and-thrust-constants)).

### Gazebo

The model `ti_tui` merge-includes `ti_tui_base` (body, inertia, collision, skids,
sensors) and adds eight rotor links with one `MulticopterMotorModel` plugin each.
The model is placed 0.102 m above the ground.

The visual meshes (`rotor_*.stl`, `stationary_assembly.stl`) are cosmetic:
physics, collision and the controller use the SDF and the airframe. Overall the model is 0.318 m × 0.333 m × 0.182 m including the
propellers. The propellers are drawn light and semi-transparent
(`cad_propeller` material in `ti_tui/model.sdf`). The front is set by the skid and upper-layer
orientation; the red `front_marker` in `ti_tui_base/model.sdf` marks it. If you
edit the SDF rotor poses, the meshes are not moved with them.

---

## 6. SIH generic multirotor (`SIH_VEHICLE_TYPE 6`)

Upstream SIH has a fixed set of vehicles (quad, hex, fixed wing, VTOL, rover).
The DTRG fork adds a sixth type that **builds the vehicle from the control
allocation parameters**, so one airframe file describes both the controller's
model and the simulated vehicle. Any multirotor `CA_AIRFRAME 0` can describe,
including tilted and fully actuated ones, flies without code changes.

| Parameter | Used as |
| --- | --- |
| `SIH_VEHICLE_TYPE` | 6 = generic multirotor (reboot required) |
| `CA_ROTOR_COUNT` | number of rotors (up to 12) |
| `CA_ROTORn_PX/PY/PZ` | rotor position $r_i$ from the centre of mass [m] |
| `CA_ROTORn_AX/AY/AZ` | thrust direction $a_i$ (normalised by SIH) |
| `CA_ROTORn_CT` | relative thrust: a rotor with the average `CT` gives `SIH_T_MAX` at full output |
| `CA_ROTORn_KM` | drag torque per thrust [m]; positive = CCW |
| `SIH_T_MAX` | max thrust of an average rotor [N] |
| `SIH_THR_MDL_FAC` | thrust curve factor $f$ (new parameter, only for this type) |
| `SIH_T_TAU` | motor time constant [s] |
| `SIH_MASS`, `SIH_IXX/IYY/IZZ/...` | rigid body |
| `SIH_KDV`, `SIH_KDW` | linear and angular drag |
| `SIH_Q_MAX`, `SIH_L_ROLL`, `SIH_L_PITCH` | not used by this type |

Output $n$ drives rotor $n$: map `PWM_MAIN_FUNCn` (SITL) to Motor $n$ in order.

### Model

Each rotor $i$ follows its output command $u_{sp,i}$ with a first-order lag,
$\dot u_i = (u_{sp,i} - u_i)/\tau$ with $\tau$ = `SIH_T_TAU`, and with
$u_i$ limited to $[0, 1]$ produces

$$
T_i = T_{max}\,\frac{CT_i}{\overline{CT}}\,\big(f\,u_i^2 + (1-f)\,u_i\big)
$$

$$
F_B = \sum_i T_i\, a_i,\qquad
M_B = \sum_i \big( r_i \times T_i a_i \;-\; KM_i\, T_i\, a_i \big)
$$

plus first-order drag $F_{aero} = -K_{DV}\, v$ (NED) and
$M_{aero} = -K_{DW}\, \omega$ (body). The thrust curve is the one PX4's
`THR_MDL_FAC` inverts, so with `SIH_THR_MDL_FAC = THR_MDL_FAC` the simulated
thrust is linear in the controller's thrust setpoint. The moment uses the same
convention as PX4's control allocation (`ActuatorEffectivenessRotors`), so the
allocator's effectiveness matrix is exactly the simulated vehicle's.

`sih status` prints the vehicle type, the rotor count and the current body
thrust.

> The parameter descriptions in `sih_params.c` and some older notes say
> "`SIH_VEHICLE_TYPE 4`" for the generic multirotor. 4 is the hexacopter: the
> generic multirotor is **6**, as the airframes set it.

### SIH vs Gazebo

Both simulate the same vehicle. HT behaves the same in both; with HT off,
Gazebo's hover is wobblier and SIH's IMU is noisier. Measurements and causes: [SIH_vs_GAZEBO](SIH_VS_GAZEBO.md), with script `Simulation Comparison/compare_sih_gz.py`.

---

## 7. Gazebo notes

- The CAD geometry (meshes, rotor poses and axes) and the airframes were first
  generated from CAD by `generate_planar_octo_model.py` and
  `generate_planetary_hex_model.py`, which are not in this repository. Do not
  hand-edit the rotor geometry. Mass, inertia and the propulsion constants are
  edited with the properties tool, see [section 9](#9-changing-mass-inertia-and-thrust-constants).
- Rotor speed limits come from the airframe: `SIM_GZ_EC_MIN` 350 and
  `SIM_GZ_EC_MAX` 1774 rad/s for the planarOcto. At zero command the rotors still idle at
  350 rad/s, about 4 % thrust.
- With `PX4_GZ_STANDALONE=1`, PX4 now detects the name of the world already
  running in Gazebo, so you can start Gazebo yourself with any world.
- On macOS the Gazebo plugins find Homebrew's `qt@5` automatically
  (`brew install qt@5` if the configuration fails on gz-gui).

---

## 8. The interaction rod

For physical-interaction experiments, `gz_planar_octo_rod_mocap_interaction`
flies a planarOcto with a rigid rod on its nose against a push panel on a floor
pole. The planetaryHex and the Ti-Tui carry the same rod
(`gz_planetary_hex_rod_mocap_interaction`, `gz_ti_tui_rod_mocap_interaction`). The full description, with the gotchas, is in the
[Physical-interaction testing](../../Tools/dtrg/README.md#physical-interaction-testing)
section of `Tools/dtrg/README.md`. This is the summary.

```sh
make px4_sitl gz_planar_octo_rod_mocap_interaction
make px4_sitl gz_planetary_hex_rod_mocap_interaction
make px4_sitl gz_ti_tui_rod_mocap_interaction
```

| Piece | Where | What |
| --- | --- | --- |
| Vehicle | model `planar_octo_rod`, airframe `12018_gz_planar_octo_rod` | `planar_octo` merge-included, plus a rod. The airframe sources 12014, so the allocation, gains and `THR_MDL_FAC` are unchanged |
| | model `planetary_hex_rod`, airframe `12020_gz_planetary_hex_rod` | `planetary_hex` merge-included, plus a rod. Sources 12019 |
| | model `ti_tui_rod`, airframe `12021_gz_ti_tui_rod` | `ti_tui` merge-included, plus a rod. Sources 12007 |
| Rod | in each `*_rod/model.sdf` | 0.35 m long, 16 mm diameter, along body +X, with an 18 mm radius contact ball on the tip. Assumed mass 0.030 kg: a 0.025 kg shaft welded to the airframe and a 0.005 kg tip, joined by the load-cell joint. Mounted at the height of the centre of mass, so an axial contact force makes no pitching moment. Same joint and sensor names on every vehicle, so `gz_bridge` finds it without changes |
| Pole | model `interaction_pole` | base plate, mast and a flat 0.60 m × 1.20 m panel spanning z = 0.80–2.00 m. 25 kg (assumed). Rigid: a prismatic joint of 1e6 N/m, 50 µm under a 50 N push |
| World | `Tools/simulation/gz/worlds/mocap_interaction.sdf` | the mocap room plus the pole, with the panel face 2 m ahead of the takeoff marker (+X) |

Where each rod sits:

| Vehicle | Rod root | Reach from body centre | Contact with the vehicle centred at | Notes |
| --- | --- | --- | --- | --- |
| planarOcto | body edge, x = 0.2475 m | 0.6155 m | x = 1.38 m | 0.368 m clear of the body |
| planetaryHex | edge of the ring, x = 0.209 m | 0.577 m | x = 1.42 m | runs through the motor 1–6 gap: the nearest propeller disc is 85 mm from the rod surface |
| Ti-Tui | front of the body, x = 0.0725 m | 0.4405 m | x = 1.56 m | |

**Centre of mass.** Every vehicle model, with or without the rod, has its
centre of mass at the body origin: the point the `CA_ROTOR*` positions are
measured from, so zero roll and pitch torque keeps it level. The plain
`*_base` models offset the base-link CoM to cancel the rotor links (planarOcto
1.4 mm, planetaryHex 0.5 mm, both vertical). Each `*_rod` model includes the
plain vehicle and overrides only the base-link CoM pose (an
`<experimental:params>` block on the include), moving it aft to balance the
rod: planarOcto 9.3 mm, planetaryHex 4.3 mm, Ti-Tui 7.1 mm. This stands in
for moving the battery aft; the rod's 0.030 kg is the only mass added. The
properties tools (section 9) write both poses, so they follow any change of
mass or rotor links. Unbalanced, the Ti-Tui rod pitched the vehicle 13° nose
down at liftoff, sent it 2.2 m forward under the panel and flipped it.

Hover at 1.0–1.5 m and fly forward: the tip reaches the panel with the vehicle
centred at the x above (planarOcto 1.38 m). Horizontal thrust (`e`, then `z/x/c/v` in
`keyboard_teleop.py`) pushes without pitching the airframe. The rod's 0.030 kg
is not part of the properties-tool `mass_kg`; the hover-thrust estimator
absorbs it.

### Recording the force

Both ends of the contact carry a Gazebo `force_torque` sensor:

| Side | Recorded as | Sign |
| --- | --- | --- |
| Vehicle (load cell at the rod tip) | `interaction_wrench` uORB topic, in the ulog at 250 Hz, body FRD (`force[0]`), torque about the tip centre | pressing forward gives a **negative** `force[0]`: it is what the surface feeds back into the rod |
| Pole (panel) | CSV from `Tools/dtrg/log_interaction_force.py`, pole frame (world aligned) | a push in +X gives a **positive** `fx` |

```sh
px4-listener interaction_wrench                     # live, from the pxh console
Tools/dtrg/log_interaction_force.py --duration 60 -o push.csv
```

The CSV is tared by default to remove the pole's standing weight, so keep clear
of the panel for the first second. Line it up against a ulog with
`sim_time_s`, which is Gazebo's clock. A cross-check over one steady push read
a median −2.48 N on the tip and +2.32 N on the pole: equal and opposite.
Only the 5 g tip sits in front of the load cell, so `interaction_wrench`
carries just its weight (+0.049 N on `force[2]` away from contact) and a small
inertial reaction; it is close to the contact force even while manoeuvring.
It is SITL only and is never published on hardware.

The `contact` sensors on the tip and the panel say *where* contact happened,
not *how hard*: gz-sim leaves their wrench at zero. Use the `force_torque`
sensors for force.

Three things to know when changing the setup:

- `gz` commands typed by hand need `GZ_IP=127.0.0.1`, which PX4 sets for the
  server. Without it `gz topic -l` returns nothing.
- A `force_torque` sensor reads no contact load if its chain is welded to the
  world. That is why the pole hangs from a stiff prismatic joint, not a weld.
- Never put a `<plugin>` in a world SDF. It replaces PX4's whole
  `server.config` plugin set. The contact and force-torque systems are loaded
  from `src/modules/simulation/gz_bridge/server.config`.

Checked in Gazebo with an offboard push 0.2 m past contact: hovering clear of
the panel each vehicle reads `force[2]` = +0.049 N, the tip's weight, and
pressing gives a steady negative `force[0]` (planarOcto −2.5 N, planetaryHex
−3.0 N, Ti-Tui −0.5 N) that returns to zero on release. The first touch gives
a one-sample spike (tens of newtons at about 1 m/s); skip it when analysing.

---

## 9. Changing mass, inertia and thrust constants

The physics of the Gazebo vehicle (mass, inertia, motor constants) is read from
the SDF when the model loads, and PX4 parameters cannot change it. The
controller's hover model (`MPC_THR_HOVER`, `CA_ROTOR*_CT`) has to agree with it.
`Tools/dtrg/vehicle_properties/` keeps both in step from one YAML per vehicle,
for changes **between runs**.

| Vehicle | YAML (edit this) | Script |
| --- | --- | --- |
| planarOcto | `planar_octo_properties.yaml` | `planar_octo_properties.py` |
| planetaryHex | `planetary_hex_properties.yaml` | `planetary_hex_properties.py` |
| Ti-Tui | `ti_tui_properties.yaml` | `ti_tui_properties.py` |

```sh
# edit the YAML, then, from the PX4 root:
python3 Tools/dtrg/vehicle_properties/planetary_hex_properties.py --dry-run   # preview the diff
python3 Tools/dtrg/vehicle_properties/planetary_hex_properties.py             # apply
make px4_sitl gz_planetary_hex_mocap                                          # rebuild
```

| YAML value | Written to |
| --- | --- |
| `mass_kg` | base link mass (total minus the rotor links) in `*_base/model.sdf`, `MPC_THR_HOVER` |
| (derived) | base link CoM pose in `*_base/model.sdf` and in the `*_rod/model.sdf` override, so the vehicle CoM stays at the body origin with and without the rod |
| `base_inertia_kgm2` | base link inertia in `*_base/model.sdf` |
| `thrust_constant` ($k_F$) | `motorConstant` per rotor, `CA_ROTORn_CT` (planetaryHex), `MPC_THR_HOVER` |
| `moment_constant` | `momentConstant`, $|$`CA_ROTORn_KM`$|$ |
| `time_constant_s` | `timeConstantUp/Down` |
| `rotor_mass_kg`, `rotor_ixx`, `rotor_izz` | rotor link inertia (the motor inertia) |
| `lower_coax_effectiveness` (planetaryHex) | `CA_ROTORn_CT` and `motorConstant` of the lower central rotor |
| `lower_rotor_effectiveness` (Ti-Tui) | `CA_ROTOR4-7_CT` and `motorConstant` of the lower four rotors |

Rotor geometry (`CA_ROTOR*_P/A`) and the speed limits (`SIM_GZ_EC_MIN/MAX`) are
read from the airframes and not changed. The defaults reproduce the committed
files exactly, so nothing changes until you edit a YAML. Mass and base
inertia are independent: change both together. The SDFs are inside the
`Tools/simulation/gz` submodule, so commit there as well. The commented-out
`RLS_EST_*` lines in `12014_gz_planar_octo` hardcode the planarOcto mass,
inertia and $k_F$ and do not follow the YAML.
