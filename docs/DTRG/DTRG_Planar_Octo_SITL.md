# DTRG planarOcto SITL

The planarOcto is a flat octocopter whose eight rotors are tilted 31° towards
the tangent of the frame, so it can push sideways without tilting (fully
actuated). This page covers the simulated planarOcto: in Gazebo, in SIH on
the PC (used by the automated tests), in SIH on the flight controller, and
HITL. It also covers the SIH *generic multirotor* added for it, which can
simulate any rotor geometry PX4 can describe.

| | |
| --- | --- |
| Airframes | `12013_dtrg_planar_octo` (the vehicle), `12014_gz_planar_octo`, `12016_sihsim_planar_octo` (SITL), `12015_dtrg_planar_octo_hitl.hil`, `12017_dtrg_planar_octo_sih.hil` (hardware) |
| Gazebo model | `Tools/simulation/gz/models/planar_octo` (submodule `UoA-DTRG/PX4-gazebo-models`, branch `dtrg-main`) |
| SIH | `src/modules/simulation/simulator_sih` (`SIH_VEHICLE_TYPE 6`, `SIH_THR_MDL_FAC`) |
| Tools | `Tools/dtrg/keyboard_teleop.py`, `Tools/dtrg/rc_override.py`, `Tools/dtrg/compare_sih_gz.py` |

---

## 1. Quick start

```sh
# Gazebo (needs Gazebo Harmonic), default world
make px4_sitl gz_planar_octo

# Gazebo in another world from Tools/simulation/gz/worlds, e.g. the mocap room
make px4_sitl gz_planar_octo_mocap

# SIH: no Gazebo, no GUI, fast; this is what the automated tests fly
make px4_sitl sihsim_planar_octo
```

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

| ID | Name | Where | What it adds |
| --- | --- | --- | --- |
| 12013 | DTRG planarOcto | real vehicle | geometry (`CA_ROTOR*`), flown gains, thrust model, outputs AUX1–8 = Motors 1–8, `DTRG_HT_EN 1`, `RC_MAP_HT_MODE 8` |
| 12014 | Gazebo DTRG planarOcto | SITL, Gazebo | 12013 + Gazebo ESC mapping (`SIM_GZ_EC_FUNC1-8`, 350–1774 rad/s), SITL gains, `RC_MAP_HT_ROLL 9`, `RC_MAP_HT_PITCH 10` |
| 12016 | SIH DTRG planarOcto | SITL, SIH | 12013 + SIH vehicle model (below), outputs 1–8 = Motors 1–8, SITL gains, HT knobs 9/10 |
| 12015 | DTRG planarOcto HITL | flight controller, HITL | 12013 + `SYS_HITL 1`, `HIL_ACT_FUNC1-8` = Motors 1–8 for an external HITL simulator |
| 12017 | DTRG planarOcto SIH | flight controller, SIH | 12013 + `SYS_HITL 2`: the real flight controller flies a planarOcto simulated on the board itself. No PC needed. Real outputs stay off (remove the props anyway) |

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

---

## 3. The vehicle

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

## 4. SIH generic multirotor (`SIH_VEHICLE_TYPE 6`)

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

## 5. Gazebo notes

- The model is generated by `generate_planar_octo_model.py` (see the header of
  the airframe and `model.sdf`); edit the generator, not the SDF.
- Rotor speed limits come from the airframe: `SIM_GZ_EC_MIN` 350 and
  `SIM_GZ_EC_MAX` 1774 rad/s. At zero command the rotors still idle at
  350 rad/s, about 4 % thrust.
- With `PX4_GZ_STANDALONE=1`, PX4 now detects the name of the world already
  running in Gazebo, so you can start Gazebo yourself with any world.
- On macOS the Gazebo plugins find Homebrew's `qt@5` automatically
  (`brew install qt@5` if the configuration fails on gz-gui).
