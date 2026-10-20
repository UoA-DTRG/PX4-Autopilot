# SIH vs Gazebo: planarOcto comparison log

25/09/2026. Question: does the planarOcto simulated by SIH (`SIH_VEHICLE_TYPE 4`,
used by the tier 3 flight tests) behave like the Gazebo model it took its values
from? Neither simulator has been checked against flight logs of the real
vehicle, so this compares the two simulators with each other, not with reality.

Script: [compare_sih_gz.py](compare_sih_gz.py). Setup: commit `cb40685d96`
plus the working tree's `rcS` (`PX4_PARAM_*` applied twice) and `drv_hrt.cpp`
(G12) changes, `make px4_sitl_default`, Gazebo Harmonic `gz sim` 8.15.0 headless,
macOS, speed factor 1, two flights per simulator.

## Summary

**Same:**
- The vehicle model: mass, inertia, rotor positions, 31 degree thrust axes,
  spin directions, motor order, drag torque coefficient, maximum thrust and the
  motor time constant all agree (geometry to 1e-7 m).
- Horizontal thrust (HT): with HT on, both move the vehicle level and reach the
  commanded HT roll tilt (4.8-5.7 degrees for a 5.7 degree command).
- Position, altitude and yaw steps: rise times agree within about 20 %, and
  the peak speeds are similar. The exception is one slow Gazebo climb.

**Different:**
1. **Attitude wobble with HT off (the main difference).** Both simulators show
   the same ~1.1 Hz coupled roll/pitch oscillation in hover. In Gazebo it is
   3-5x larger: 12-14 degrees RMS with peaks of 21 degrees, against 3-5 degrees
   RMS in SIH. The peaks are twice `MPC_TILTMAX_AIR` (10 degrees).
   - About half of the gap is angular damping: SIH's default `SIH_KDW` (0.025)
     is 3.4x the roll/pitch damping of the Gazebo rotors (0.0073).
   - SIH with the Gazebo damping wobbles at 7.5-9 degrees RMS.
   - With HT on, neither vehicle wobbles.
2. **Sensors and estimation.** SIH's IMU is far noisier (accelerometer 0.6 vs
   0.004 m/s^2 per sample). As a result, SIH's altitude estimate is 7x noisier
   and it drifts more in hover.
   - Gazebo's heading estimate drifts to 45 degrees off the truth during the
     first hover after take-off; SIH stays within 6 degrees.
3. **Thrust curve.** At the same thrust command, Gazebo's rotors give 1.7-3.4 %
   more thrust than SIH's in mid-range. Gazebo also still gives 3.9 % thrust at
   zero command, because its rotors idle at 350 rad/s. As a result, Gazebo
   hovers level at a command of 0.544 (which is the airframe's
   `MPC_THR_HOVER`), and SIH at 0.562.
4. **Vertical drag.** SIH applies `SIH_KDV` equally on all axes, so its
   vertical drag is 3x Gazebo's. Horizontal drag agrees within 6 %. This had
   no visible effect on the climb and descent steps, which are limited by the
   velocity limits.

**What this means for the tests:** the tier 3 tests check HT behaviour, and HT
behaves the same in both simulators. However, SIH's plain HT-off hover is much
calmer than Gazebo's. A7's reference hover would see 21 degree peaks in Gazebo.

**Which simulator is closer to the real vehicle is not known.** The damping,
thrust curve and noise are all assumed values in both. A flight log of the real
planarOcto hovering with HT off would settle it.

## 1. Static comparison (from the files)

`python3 Tools/dtrg/compare_sih_gz.py --static` reads the Gazebo model
(`Tools/simulation/gz/models/planar_octo*`, converted from FLU to FRD). It
compares it with the parameters SIH runs with (`12013_dtrg_planar_octo` plus
`12016_sihsim_planar_octo`, and the `sih_params.c` defaults).

| | SIH | Gazebo | |
|---|---|---|---|
| mass [kg] | 1.500 | 1.500 | base 1.46 + 8 x 0.005 rotors |
| Ixx / Iyy / Izz [kg m^2] | 0.0254 / 0.0254 / 0.0420 | 0.0254 / 0.0254 / 0.0421 | composite about the CoM |
| centre of mass vs CAD origin [mm] | 0 (CA_ROTOR positions used as is) | 0, 0, -1.4 | FRD, from the rotor links |
| rotor n position (CA_ROTORn_P*) [m] | CA_ROTORn_P* | max diff 4.9e-08 | same |
| rotor n thrust axis (CA_ROTORn_A*) | CA_ROTORn_A* | max diff 2.5e-08 | same, 31 deg tilt |
| rotor n spin | sign of CA_ROTORn_KM | turningDirection | same |
| motor n -> rotor n | output n drives rotor n | actuator_number n | same |
| drag torque / thrust [m] | 0.00858 | 0.00858 | same |
| max thrust per rotor [N] | 3.820 | 3.820 | kF w_max^2 |
| motor time constant [s] | 0.030 on thrust | 0.030 up / 0.030 down on rotor speed | same lag for small changes around hover |
| linear drag x / y / z [N/(m/s)] | 0.700 (SIH_KDV, all axes) | 0.744 / 0.744 / 0.228 | rotor drag at hover speed 1329 rad/s |
| angular damping roll / pitch / yaw [Nm/(rad/s)] | 0.0250 (SIH_KDW, all axes) | 0.0073 / 0.0073 / 0.0383 | rotor drag only; the gz body has no damping |
| hover thrust command (level) | 0.562 | 0.544 | `MPC_THR_HOVER` is 0.544 |

Thrust per rotor, relative to the maximum, at thrust command c
(`actuator_motors`). PX4 inverts `THR_MDL_FAC` = 0.670 for both simulators.
SIH's curve is `SIH_THR_MDL_FAC` = `THR_MDL_FAC`, so its thrust is exactly c.
Gazebo maps the output linearly onto 350-1774 rad/s, and its thrust is kF w^2:

| c | 0 | 0.12 (`MPC_THR_MIN`) | 0.25 | 0.50 | 0.56 | 0.75 | 1 |
|---|---|---|---|---|---|---|---|
| SIH | 0.000 | 0.120 | 0.250 | 0.500 | 0.562 | 0.750 | 1.000 |
| Gazebo | 0.039 | 0.154 | 0.279 | 0.519 | 0.579 | 0.760 | 1.000 |

The drag and damping of the Gazebo model come from the rotor drag of gz-sim's
`MulticopterMotorModel`: F = -|w| `rotorDragCoefficient` v_perp on each rotor.
Here v_perp is the air velocity perpendicular to the rotor axis. The values
above are evaluated at the hover rotor speed, for a unit translation along each
body axis or a unit rotation about it. The model has no other damping.

## 2. Flights

Every run uses the same firmware and the same parameters: the tier 3 flight
parameters (`DTRG_HT_EN 1`, `DTRG_HT_MAX 0.5`, `DTRG_HT_R/P_MAX 10`, take-off
to 3 m) with the standard RC layout. The profile is flown in Offboard, so no
pilot input differs between runs:

take-off to 3 m -> hover 10 s -> yaw +90 deg -> 5 m north (HT off) -> HT on,
5 m back south -> HT roll 0.1 rad via `DEBUG_FLOAT_ARRAY` for 8 s -> HT off,
climb 2 m -> descend 2 m -> land

The runs:

| Run | Airframe | Changes |
|---|---|---|
| `sih` | `12016_sihsim_planar_octo` | none |
| `sih_gzdamp` | `12016_sihsim_planar_octo` | `SIH_KDW` 0.0073, the Gazebo roll/pitch damping |
| `gz` | `12014_gz_planar_octo` | none |

The attitude and position of the vehicle come from the ground truth topics.
Step responses are measured on the estimate, because that is what the
controller tracks.

### Key results

Each cell is the mean of the two flights, with the individual flights in
brackets.

| | sih | sih_gzdamp | gz |
|---|---|---|---|
| **Hover, HT off** | | | |
| roll / pitch RMS [deg] | 4.6 / 3.0 (3.2-6.0 / 2.1-4.0) | 7.5 / 9.0 (6.4-8.6 / 7.5-10.4) | 12.3 / 14.4 (12.3-12.4 / 14.4) |
| tilt peak [deg] | 10.6 (7.3 / 14.0) | 15.5 (13.9 / 17.1) | 21.0 (21.0 / 21.0) |
| wobble frequency [Hz] | 1.08 | 1.12 | 1.07 |
| attitude **setpoint** roll / pitch RMS [deg] | 3.7 / 2.3 | 5.2 / 5.4 | 6.6 / 7.5 |
| thrust command / hover thrust estimate | 0.565 / 0.564 | 0.576 / 0.566 | 0.577 / 0.546 |
| motor range | 0.47-0.68 | 0.45-0.72 | 0.41-0.75 |
| altitude std (truth) [m] | 0.17 | 0.23 | 0.024 |
| horizontal drift in 8 s [m] | 0.91 | 1.01 | 0.23 |
| estimated minus true altitude, std [m] | 0.089 | 0.14 | 0.013 |
| heading error, max [deg] | 3.9 | 2.5 | 45.4 |
| accel / gyro noise per sample [m/s^2, rad/s] | 0.61 / 0.041 | 0.60 / 0.043 | 0.0036 / 0.021 |
| **Yaw step +90 deg** | | | |
| rise 10-90 % [s] / overshoot [deg] | 0.46 / 9.0 | 0.42 / 10.7 | 0.54 / 0.4 |
| **5 m north, HT off** | | | |
| rise [s] / overshoot [m] / peak speed [m/s] | 2.33 / 0.21 / 2.04 | 2.35 / 0.14 / 1.93 | 2.54 / 0.27 / 2.18 |
| tilt peak [deg] | 12.7 | 14.8 | 15.0 |
| **5 m south, HT on** | | | |
| rise [s] / overshoot [m] / peak speed [m/s] | 3.60 / 0.67 / 1.53 | 3.34 / 0.57 / 1.62 | 4.13 / 0.00 / 1.16 |
| tilt peak [deg] | 2.4 | 2.4 | 0.5 |
| **HT roll 0.1 rad (5.73 deg), last 3 s** | | | |
| true roll mean / std [deg] | 5.19 / 0.38 | 5.47 / 0.42 | 5.64 / 0.02 |
| drift [m] | 1.03 | 0.72 | 0.26 |
| **Climb / descend 2 m (HT off)** | | | |
| climb rise [s] / peak vz [m/s] | 1.56 / 1.50 | 1.47 / 1.57 | 2.68 / 1.64 |
| descend rise [s] / peak vz [m/s] | 1.80 / 1.43 | 1.73 / 1.38 | 1.79 / 1.42 |
| motors saturated during the climb [% of samples] | 2.6 | 2.7 | 2.3 |
| **Land** | | | |
| land to disarm [s] | 12.8 | 16.0 | 10.5 |

The full table, with every metric and both flights, is in the appendix.

## 3. Findings

### 3.1 Same model, same HT behaviour

The static comparison shows that SIH builds the Gazebo model: the same rigid
body, geometry, rotor spin and motor order, with the thrust and torque
coefficients matched. In flight, the things the tier 3 tests check behave the
same way:

- With HT on, both move 5 m while staying level: at most 2.6 degrees of tilt in
  SIH and 0.5 degrees in Gazebo. With HT off, the same move tilts the vehicle
  by 12-16 degrees.
- The Offboard HT roll tilt lands within 0.9 degrees of the command in both.
- The yaw step, the horizontal steps and the vertical steps have similar rise
  times and peak speeds. The climb saturates a motor for about 2.5 % of the
  time in all three runs.

### 3.2 The HT-off wobble is 3-5x larger in Gazebo; damping explains about half

Both simulators oscillate in roll and pitch at the same ~1.1 Hz whenever HT is
off: in hover, during the yaw step, and during the climb and descent. The
oscillation is a coning motion, in which the tilt direction rotates:

- In Gazebo it reaches 21 degrees of tilt. A still hover never happens.
- In SIH it is 3-5 degrees RMS, with peaks of 7-14 degrees.

The attitude setpoint swings as well (6.6-7.5 degrees RMS in Gazebo), and the
vehicle overshoots it about 2x. So the position loop and the attitude loop
oscillate together. This is not only a rate loop problem.

With HT on (the south move and the HT roll hold) the oscillation disappears in
both simulators. Gazebo then holds 5.64 ± 0.02 degrees. The attitude setpoint
is then fixed by HT and no longer comes from the position loop, which breaks
the loop that oscillates.

Angular damping is the largest model difference. SIH damps roll and pitch with
`SIH_KDW` = 0.025. The Gazebo model's only damping is its rotor drag, worth
0.0073. SIH with the Gazebo value (`sih_gzdamp`) wobbles at 7.5-9 degrees RMS
instead of 3-5. That closes 40-50 % of the gap (roll RMS 38 %, pitch RMS
52 %, peak tilt 47 %).

The remainder was not isolated. The likely causes are:
- the rotor speed dynamics and the command transport delay of Gazebo;
- Gazebo's lower sensor noise, which changes the estimator's filtering.

The Gazebo wobble also explains why its thrust command in hover (0.577) is
above its level hover command (0.544): 0.544 / cos(19 deg) is about 0.575.
Gazebo does not saturate in hover (motors 0.41-0.75).
[SITL_TESTING.md](SITL_TESTING.md) section 6 mentions "the saturation seen in
Gazebo". These runs only saturate during the 2 m climb, and all three
simulators do so equally.

Which amplitude is realistic is open. The real vehicle's autotuned gains
(`12013`) were tuned in flight, which suggests the real vehicle does not cone
at 21 degrees. However, 0.025 is SIH's generic default, not a planarOcto
value.

### 3.3 Sensors and estimation differ a lot

- SIH's IMU noise when armed is hard-coded in `sih.cpp` (accelerometer
  0.5/1.7/1.4 m/s^2, gyro 0.14/0.07/0.03 rad/s). That is about 170x Gazebo's
  accelerometer noise and 2x its gyro noise per sample.
- As a result, the SIH estimate wanders: altitude estimate error std 0.09-0.14 m
  (Gazebo 0.013 m), true altitude std 0.17 m (Gazebo 0.024 m), and hover drift
  about 1 m (Gazebo 0.23 m).
- Gazebo instead has a constant +0.25 m offset between estimated and true
  altitude.
- **Gazebo heading:** in both Gazebo flights, the heading estimate drifts to
  45 degrees off the truth during the first HT-off hover after take-off. It
  recovers only once the vehicle translates (the north move) and does not come
  back in the later HT-off phases, although the wobble does.
  - The IMU shows the 0.37 rad/s body yaw rate that coning produces (the true
    world yaw rate is 0), which is correct kinematics.
  - The cause was not investigated. SIH stays within 6 degrees.
  - With the heading 45 degrees off, the Gazebo north step flies in a direction
    rotated from true north. The metrics use the estimated frame.

### 3.4 Thrust curve and vertical drag

- Gazebo's rotors idle at 350 rad/s, which is 3.9 % of maximum thrust at zero
  command. SIH has none, so the curves differ most at low command: 0.154 vs
  0.120 at `MPC_THR_MIN`.
  - This makes Gazebo slower to shed thrust.
  - It may be why the Gazebo landing reaches disarm 2-5 s sooner. That is not
    confirmed.
- SIH's vertical drag (0.70 N/(m/s)) is 3x Gazebo's (0.23). The climb and
  descent peak speeds still agree (1.4-1.6 m/s), because the velocity limits
  cap them.
- The Gazebo climb rise time of 2.68 s is the mean of 1.77 and 3.58. The slow
  flight coincides with a 26 degree wobble peak.

## 4. Limitations

- Two flights per simulator. The run-to-run spread is in the full table. Some
  SIH metrics vary by 2x between flights, for example hover roll RMS
  3.2 / 6.0 degrees.
- The profile is timed by the host clock. The durations in PX4 time differ by a
  few seconds between runs (78-94 s).
- The Gazebo drag and damping values are computed from the model at hover rotor
  speed, not measured in flight.
- Settling times are `nan` when the response had not settled within ±10 % of
  the step by the end of its phase.
- No real flight data was used.

## Reproduce

```bash
make px4_sitl_default
pip3 install -r test/dtrg/requirements.txt
python3 Tools/dtrg/compare_sih_gz.py --static                              # section 1, no simulator
python3 Tools/dtrg/compare_sih_gz.py --runs 2 --out /tmp/dtrg_sih_vs_gz    # about 15 min
python3 Tools/dtrg/compare_sih_gz.py --analyse-only --out /tmp/dtrg_sih_vs_gz
```

Each run leaves `px4.log`, its ULog, `phases.json` (the PX4 time at which each
phase starts) and `metrics.json` in `--out/<sim>_run<n>/`. The combined results
are in `results.md` and `results.json`. `--sims gz` or `--sims sih,sih_gzdamp`
runs a subset. Do not run another PX4 or `gz sim` at the same time.

## Appendix: all metrics

Each cell is the mean of the two flights in bold, followed by the individual
flights. Units are in the metric names: `_deg`, `_s`, `_m`, `_mps` (m/s), and
`_hz`. Motor values and commands are normalised to 0-1.

| metric | sih | sih_gzdamp | gz |
|---|---|---|---|
| takeoff.liftoff_to_90pct_s | **5.74** (4.51 / 6.97) | **5.58** (7.21 / 3.95) | **4.71** (4.82 / 4.61) |
| takeoff.peak_climb_mps | **1.07** (0.998 / 1.14) | **1.12** (1.13 / 1.11) | **1.05** (1.05 / 1.05) |
| hover.roll_rms_deg | **4.58** (3.2 / 5.96) | **7.5** (6.4 / 8.61) | **12.3** (12.3 / 12.4) |
| hover.pitch_rms_deg | **3.04** (2.07 / 4.01) | **8.98** (7.53 / 10.4) | **14.4** (14.4 / 14.4) |
| hover.tilt_max_deg | **10.6** (7.26 / 14) | **15.5** (13.9 / 17.1) | **21** (21 / 21) |
| hover.wobble_hz | **1.08** (1.12 / 1.05) | **1.12** (1.36 / 0.88) | **1.07** (1.07 / 1.08) |
| hover.roll_sp_rms_deg | **3.65** (2.68 / 4.62) | **5.16** (4.69 / 5.63) | **6.62** (6.64 / 6.6) |
| hover.pitch_sp_rms_deg | **2.25** (1.85 / 2.65) | **5.42** (4.56 / 6.29) | **7.49** (7.48 / 7.5) |
| hover.thrust_cmd | **0.565** (0.563 / 0.568) | **0.576** (0.572 / 0.581) | **0.577** (0.577 / 0.576) |
| hover.hover_thrust_est | **0.564** (0.563 / 0.565) | **0.566** (0.565 / 0.566) | **0.546** (0.546 / 0.546) |
| hover.motor_mean | **0.566** (0.563 / 0.568) | **0.576** (0.572 / 0.581) | **0.577** (0.577 / 0.576) |
| hover.motor_spread | **0.00356** (0.00166 / 0.00547) | **0.00478** (0.00471 / 0.00486) | **0.0126** (0.0113 / 0.014) |
| hover.motor_min | **0.467** (0.476 / 0.458) | **0.446** (0.453 / 0.439) | **0.405** (0.406 / 0.403) |
| hover.motor_max | **0.675** (0.653 / 0.696) | **0.72** (0.71 / 0.73) | **0.747** (0.748 / 0.746) |
| hover.saturated_pct | **0** (0 / 0) | **0** (0 / 0) | **0** (0 / 0) |
| hover.alt_std_m | **0.172** (0.131 / 0.214) | **0.233** (0.185 / 0.281) | **0.0243** (0.0211 / 0.0275) |
| hover.drift_m | **0.914** (0.436 / 1.39) | **1.01** (1.41 / 0.607) | **0.228** (0.254 / 0.202) |
| hover.est_alt_error_mean_m | **0.00111** (0.158 / -0.156) | **0.0311** (-0.177 / 0.239) | **0.246** (0.263 / 0.229) |
| hover.est_alt_error_std_m | **0.0893** (0.0822 / 0.0964) | **0.141** (0.0819 / 0.2) | **0.0132** (0.0142 / 0.0123) |
| hover.heading_error_max_deg | **3.85** (3.19 / 4.5) | **2.47** (3.68 / 1.26) | **45.4** (45.5 / 45.2) |
| hover.gyro_noise_radps | **0.0412** (0.0414 / 0.041) | **0.043** (0.0424 / 0.0436) | **0.0211** (0.0212 / 0.0209) |
| hover.accel_noise_mps2 | **0.607** (0.606 / 0.609) | **0.603** (0.609 / 0.597) | **0.00358** (0.00361 / 0.00355) |
| yaw.rise_s | **0.46** (0.46 / 0.46) | **0.42** (0.416 / 0.424) | **0.536** (0.548 / 0.524) |
| yaw.overshoot | **8.96** (8.84 / 9.08) | **10.7** (11.2 / 10.1) | **0.421** (0.469 / 0.373) |
| yaw.settle_s | **0.89** (0.712 / 1.07) | **1.11** (1.12 / 1.11) | **0.858** (0.864 / 0.852) |
| yaw.peak_rate_degps | **182** (181 / 184) | **193** (198 / 188) | **191** (191 / 191) |
| yaw.roll_rms_deg | **3.52** (3.54 / 3.49) | **7.37** (8.2 / 6.53) | **10.7** (11.3 / 10.1) |
| yaw.pitch_rms_deg | **4.81** (2.04 / 7.58) | **8.82** (7.54 / 10.1) | **13.7** (14.2 / 13.2) |
| yaw.tilt_max_deg | **12** (8.2 / 15.8) | **14.9** (13.4 / 16.5) | **21.2** (21.4 / 21) |
| yaw.motor_min | **0.401** (0.396 / 0.405) | **0.346** (0.398 / 0.295) | **0.316** (0.31 / 0.322) |
| yaw.motor_max | **0.723** (0.718 / 0.727) | **0.778** (0.747 / 0.809) | **0.842** (0.847 / 0.836) |
| yaw.saturated_pct | **0** (0 / 0) | **0** (0 / 0) | **0** (0 / 0) |
| north.rise_s | **2.33** (2.48 / 2.18) | **2.35** (2.25 / 2.45) | **2.54** (2.51 / 2.58) |
| north.overshoot | **0.209** (0.107 / 0.311) | **0.143** (0.241 / 0.0438) | **0.268** (0.376 / 0.159) |
| north.settle_s | **3.24** (3.35 / 3.13) | **3.38** (3.4 / 3.36) | **3.36** (3.25 / 3.47) |
| north.peak_speed_mps | **2.04** (1.92 / 2.16) | **1.93** (2.1 / 1.76) | **2.18** (2.25 / 2.12) |
| north.roll_rms_deg | **5.79** (5.74 / 5.84) | **5.75** (6.11 / 5.39) | **5.23** (5.38 / 5.07) |
| north.pitch_rms_deg | **1.84** (1.36 / 2.32) | **4.13** (2.92 / 5.33) | **5.99** (5.7 / 6.29) |
| north.tilt_max_deg | **12.7** (13.3 / 12.1) | **14.8** (14.9 / 14.6) | **15** (16.2 / 13.8) |
| north.motor_min | **0.468** (0.473 / 0.464) | **0.439** (0.418 / 0.46) | **0.438** (0.487 / 0.388) |
| north.motor_max | **0.662** (0.664 / 0.659) | **0.708** (0.717 / 0.698) | **0.683** (0.655 / 0.711) |
| north.saturated_pct | **0** (0 / 0) | **0** (0 / 0) | **0** (0 / 0) |
| north.heading_error_max_deg | **4.01** (2.63 / 5.38) | **4.18** (4.72 / 3.64) | **43.6** (45.8 / 41.4) |
| south_ht.rise_s | **3.6** (4.27 / 2.92) | **3.34** (3.13 / 3.54) | **4.13** (4.1 / 4.16) |
| south_ht.overshoot | **0.667** (0.228 / 1.1) | **0.571** (0.848 / 0.295) | **0** (0 / 0) |
| south_ht.settle_s | **5.45** (5.45 / nan) | **4.62** (nan / 4.62) | **5.52** (5.5 / 5.54) |
| south_ht.peak_speed_mps | **1.53** (1.23 / 1.82) | **1.62** (1.77 / 1.47) | **1.16** (1.16 / 1.15) |
| south_ht.roll_rms_deg | **1.31** (0.821 / 1.79) | **1.4** (1.64 / 1.16) | **0.274** (0.266 / 0.282) |
| south_ht.pitch_rms_deg | **0.665** (1.03 / 0.301) | **0.409** (0.384 / 0.434) | **0.181** (0.176 / 0.185) |
| south_ht.tilt_max_deg | **2.4** (2.21 / 2.59) | **2.37** (2.5 / 2.23) | **0.506** (0.511 / 0.502) |
| south_ht.motor_min | **0.381** (0.375 / 0.386) | **0.386** (0.379 / 0.393) | **0.405** (0.406 / 0.403) |
| south_ht.motor_max | **0.745** (0.747 / 0.742) | **0.746** (0.748 / 0.745) | **0.681** (0.681 / 0.682) |
| south_ht.saturated_pct | **0** (0 / 0) | **0** (0 / 0) | **0** (0 / 0) |
| south_ht.heading_error_max_deg | **3.04** (1.77 / 4.3) | **3.88** (3.76 / 4) | **5.85** (6.39 / 5.32) |
| ht_roll.roll_mean_deg | **5.19** (4.84 / 5.54) | **5.47** (5.68 / 5.27) | **5.64** (5.65 / 5.63) |
| ht_roll.roll_std_deg | **0.382** (0.293 / 0.472) | **0.42** (0.313 / 0.527) | **0.019** (0.0167 / 0.0212) |
| ht_roll.pitch_mean_deg | **-0.0334** (-0.257 / 0.191) | **0.0494** (0.396 / -0.297) | **0.32** (0.279 / 0.361) |
| ht_roll.drift_m | **1.03** (0.674 / 1.39) | **0.722** (0.818 / 0.626) | **0.264** (0.271 / 0.257) |
| ht_roll.motor_min | **0.327** (0.34 / 0.314) | **0.31** (0.303 / 0.316) | **0.336** (0.337 / 0.335) |
| ht_roll.motor_max | **0.796** (0.781 / 0.811) | **0.819** (0.816 / 0.821) | **0.746** (0.746 / 0.747) |
| ht_roll.saturated_pct | **0** (0 / 0) | **0** (0 / 0) | **0** (0 / 0) |
| climb.rise_s | **1.56** (1.56 / nan) | **1.47** (nan / 1.47) | **2.68** (1.77 / 3.58) |
| climb.overshoot | **0.116** (0.232 / 0) | **0.123** (0 / 0.246) | **0** (0 / 0) |
| climb.settle_s | **nan** (nan / nan) | **nan** (nan / nan) | **3.14** (2.1 / 4.18) |
| climb.peak_vz_mps | **1.5** (1.66 / 1.35) | **1.57** (1.46 / 1.68) | **1.64** (1.61 / 1.67) |
| climb.thrust_cmd_min | **0.526** (0.534 / 0.517) | **0.52** (0.504 / 0.536) | **0.504** (0.492 / 0.517) |
| climb.thrust_cmd_max | **1** (1 / 1) | **1** (1 / 1) | **1** (1 / 1) |
| climb.roll_rms_deg | **7.48** (5.97 / 8.99) | **8.99** (9.84 / 8.13) | **14.3** (12.3 / 16.3) |
| climb.pitch_rms_deg | **1.84** (1.78 / 1.89) | **2.02** (1.05 / 2.99) | **13.2** (9.49 / 16.9) |
| climb.tilt_max_deg | **13.8** (11.9 / 15.8) | **16.6** (16.8 / 16.4) | **23.5** (20.8 / 26.1) |
| climb.motor_min | **0.434** (0.463 / 0.405) | **0.419** (0.402 / 0.437) | **0.367** (0.376 / 0.358) |
| climb.motor_max | **1** (1 / 1) | **1** (1 / 1) | **1** (1 / 1) |
| climb.saturated_pct | **2.57** (1.92 / 3.23) | **2.67** (1.49 / 3.85) | **2.29** (1.72 / 2.86) |
| descend.rise_s | **1.8** (2.13 / 1.46) | **1.73** (1.46 / 2.01) | **1.79** (1.78 / 1.81) |
| descend.overshoot | **0.114** (0 / 0.228) | **0.2** (0.229 / 0.171) | **0.000183** (0.000366 / 0) |
| descend.settle_s | **3.77** (2.7 / 4.85) | **3.78** (4.98 / 2.58) | **2.13** (2.12 / 2.14) |
| descend.peak_vz_mps | **1.43** (1.42 / 1.44) | **1.38** (1.41 / 1.34) | **1.42** (1.44 / 1.4) |
| descend.thrust_cmd_min | **0.204** (0.251 / 0.156) | **0.216** (0.161 / 0.271) | **0.235** (0.223 / 0.247) |
| descend.thrust_cmd_max | **0.62** (0.604 / 0.635) | **0.621** (0.638 / 0.604) | **0.642** (0.629 / 0.655) |
| descend.roll_rms_deg | **4.07** (2.39 / 5.76) | **7.86** (8.14 / 7.59) | **12.8** (10.4 / 15.3) |
| descend.pitch_rms_deg | **2.48** (1.97 / 2.99) | **5.18** (1.94 / 8.41) | **16** (13.7 / 18.2) |
| descend.tilt_max_deg | **7.93** (4.98 / 10.9) | **15.5** (14.4 / 16.7) | **23.6** (20.6 / 26.5) |
| descend.motor_min | **0.206** (0.223 / 0.189) | **0.144** (0.132 / 0.156) | **0.11** (0.112 / 0.109) |
| descend.motor_max | **0.658** (0.638 / 0.679) | **0.713** (0.721 / 0.705) | **0.794** (0.763 / 0.824) |
| descend.saturated_pct | **0** (0 / 0) | **0** (0 / 0) | **0** (0 / 0) |
| land.land_to_disarm_s | **12.8** (14.1 / 11.5) | **16** (18.5 / 13.6) | **10.5** (10 / 11) |
| land.peak_descent_mps | **0.799** (0.72 / 0.878) | **0.81** (0.853 / 0.767) | **0.706** (0.712 / 0.699) |
