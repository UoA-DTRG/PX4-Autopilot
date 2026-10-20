# DTRG bench test mode

A flight mode for **system identification on a rigidly mounted vehicle**. It switches every control loop off and feeds the control allocator directly: a hover thrust, plus a clean, repeatable step or ramp on one chosen axis (thrust X/Y/Z or roll/pitch/yaw torque), started from one RC switch. Log the setpoints, motor outputs and load cell or IMU response to identify the vehicle's effectiveness (for example to build a [CSV mixer](DTRG_CSV_Mixer.md)) or the motor dynamics.

|            |                                                                                                                                                                                                                             |
| ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Mode       | *Bench Test*: `COM_FLTMODEx = 17`, nav state 16, MAVLink custom main mode 12                                                                                                                                                |
| Parameters | `BT_MODE`, `BT_AXIS`, `BT_HOVER_THR`, `BT_STEP_MAG`, `BT_STEP_DELAY`, `BT_STEP_DUR`, `BT_RAMP_RATE`, `BT_MAX_VAL`, `BT_NUM_MOTORS`, `BT_SAT_MARGIN`, `BT_ARM_ENABLE`, `BT_SPINUP_T`, `RC_MAP_CMD_SIGN` (group *Bench Test*) |
| Code       | `src/modules/bench_test` (module, maths in `bench_test_profile.h`, switch decoding in `bench_test_switch.h`), commander changes in `Commander.cpp`, `UserModeIntention.cpp`, `ModeUtil/control_mode.cpp`                    |
| Tests      | `DtrgBenchProfileTest.cpp`, `DtrgBenchSwitchTest.cpp` (tier 1), `test_bench_test_outputs.py`, `test_bench_test_safety.py` (tier 2)                                                                                          |

> **Safety:** this mode spins the motors with **no attitude or rate control**. Only use it with the vehicle rigidly and securely mounted to a  test stand, propellers removed or the rig fully guarded, and a kill switch at hand. Nothing in this mode can keep a free vehicle upright.

---

## 1. What it does

- Active only while the vehicle is in Bench Test mode. In any other mode the
  module publishes nothing.
- In the mode, the controllers are off: only the control allocator runs. The
  module publishes `vehicle_thrust_setpoint` and `vehicle_torque_setpoint`
  itself, at 400 Hz.
- Armed **and** `BT_ARM_ENABLE = 1`: it commands a hover baseline
  (`BT_HOVER_THR`, soft-started over `BT_SPINUP_T`) and, while the direction
  switch is off centre, the excitation profile on `BT_AXIS`. Otherwise it
  publishes zero.
- One 3-position switch (`RC_MAP_CMD_SIGN`) runs the profile: **centre** = no
  excitation (hover only), **up** = positive, **down** = negative. Moving off
  centre starts the profile from $t = 0$; moving to the other side restarts it
  with the other sign; back to centre ends it.
- **Step** (`BT_MODE 1`): hover for `BT_STEP_DELAY`, then $\pm$`BT_STEP_MAG`
  for `BT_STEP_DUR`, then hover again.
- **Ramp** (`BT_MODE 2`): increases at `BT_RAMP_RATE` until a motor comes
  within `BT_SAT_MARGIN` of its limit or the ramp reaches `BT_MAX_VAL`, then
  holds that value.
- Changing a profile parameter (`BT_MODE`, `BT_AXIS`, `BT_STEP_*`,
  `BT_RAMP_RATE`, `BT_MAX_VAL`) in the middle of a run restarts the profile
  from $t = 0$. `BT_HOVER_THR` and `BT_SPINUP_T` do not (that would drop and
  re-ramp the motors).
- RC loss reads as centre: the excitation stops and the hover baseline stays.

### Safety interlocks (commander)

| Situation | Result | Message |
| --- | --- | --- |
| Switching to Bench Test while armed | refused | `Bench test mode denied: disarm first` |
| Arming with `BT_ARM_ENABLE = 0` | refused | `Arming denied: bench test not enabled, set BT_ARM_ENABLE=1` |
| Arming with the direction switch off centre (`BT_MODE ≠ 0` and `RC_MAP_CMD_SIGN ≠ 0`) | refused | `Arming denied: centre the bench test direction switch` |
| Arming from RC (stick, switch, button) in this non-manual mode | allowed | |
| Disarming while the land detector says "in air" (it will, on a stand) | allowed | |

Staying in Bench Test after arming is fine; only *entering* it armed is
refused.

---

## 2. How to use it

1. **Mount** the vehicle on the stand. Remove the props or guard the rig.
2. **Mode switch:** put Bench Test on a flight-mode slot: `COM_FLTMODEx = 17`
   (*Bench Test* in the parameter's list of values). It cannot be selected over
   MAVLink: QGroundControl does not know the mode and PX4 does not accept
   custom main mode 12 in `DO_SET_MODE`.
3. **Direction switch:** `RC_MAP_CMD_SIGN` = the channel of a 3-position
   switch. It must not be used by any other `RC_MAP_*`
   ([conflict check](DTRG_RC_Channel_Conflict_Check.md)).
4. **Profile:** set `BT_AXIS`, `BT_MODE`, `BT_HOVER_THR` and either
   `BT_STEP_*` or `BT_RAMP_RATE`/`BT_MAX_VAL`.
5. **Saturation detection (ramp):** `BT_NUM_MOTORS` (0 = auto) and
   `BT_SAT_MARGIN`.
6. **Logging:** make sure the logger runs while armed (default). For
   high-rate data set `SDLOG_PROFILE` to include *high rate* / *system
   identification*.
7. **Enable:** `BT_ARM_ENABLE = 1` only when ready.
8. **Run:** disarmed, select Bench Test, centre the direction switch, arm.
   The motors ramp up to the hover baseline over `BT_SPINUP_T`. Flip the
   switch up (or down) to run the profile; back to centre to stop. Repeat as
   needed: each flip runs an identical profile.
9. **Finish:** disarm (from the transmitter is fine), leave the mode, set
   `BT_ARM_ENABLE = 0`.

Console:

```sh
bench_test status     # mode, axis, switch channel and live sign, arm gate, ramp state
bench_test stop
bench_test start
```

---

## 3. Parameters

### Mode and axis

| Parameter | Default | Description |
| --- | --- | --- |
| `BT_MODE` | 0 | 0 hover only, 1 step, 2 ramp |
| `BT_AXIS` | 2 | 0 thrust X (forward), 1 thrust Y (right), 2 thrust Z (collective), 3 roll torque, 4 pitch torque, 5 yaw torque. X/Y thrust only does something on a fully actuated airframe; on a normal multirotor the allocator drops it. |
| `RC_MAP_CMD_SIGN` | 0 | Raw RC channel (1–16) of the 3-position direction switch: > 1700 µs positive, < 1300 µs negative, otherwise centre. 0 = no excitation, hover only. |
| `BT_HOVER_THR` | 0.5 | Hover baseline, normalised 0–1, published as $-$`BT_HOVER_THR` on body Z (up is $-Z$). |

### Step (`BT_MODE 1`)

| Parameter | Default | Unit | Description |
| --- | --- | --- | --- |
| `BT_STEP_MAG` | 0.1 | normalised | step size |
| `BT_STEP_DELAY` | 2.0 | s | hover time before the step |
| `BT_STEP_DUR` | 1.0 | s | step duration |

### Ramp (`BT_MODE 2`)

| Parameter | Default | Unit | Description |
| --- | --- | --- | --- |
| `BT_RAMP_RATE` | 0.05 | 1/s | slope |
| `BT_MAX_VAL` | 0.3 | normalised | hard limit: the ramp stops and holds here if no motor saturates first |
| `BT_NUM_MOTORS` | 0 | | number of motors checked for saturation (outputs 0..N−1 of `actuator_motors`). 0 = count the finite outputs. Set it if unused outputs would otherwise be checked. |
| `BT_SAT_MARGIN` | 0.05 | | a motor counts as saturated above $1 - m$ or below $m$ ($-1 + m$ for reversible motors). 0.05 → 0.95 / 0.05. |

### Activation

| Parameter | Default | Unit | Description |
| --- | --- | --- | --- |
| `BT_ARM_ENABLE` | 0 | bool | Master gate. 0: arming is refused in the mode and the module publishes only zeros. |
| `BT_SPINUP_T` | 2.0 | s | time to ramp the hover baseline from 0 each time the outputs become active. 0 = immediate. |

---

## 4. The maths

Let $t$ be the time since the switch left centre (reset at centre, on a side
change and on a profile parameter change), $t_{out}$ the time since the
outputs became active (armed with `BT_ARM_ENABLE = 1`), and
$\sigma \in \{-1, 0, +1\}$ the switch position.

**Spin-up** ($T_s$ = `BT_SPINUP_T`):

$$
\lambda(t_{out}) = \begin{cases}\mathrm{clamp}(t_{out}/T_s,\ 0,\ 1) & T_s > 0\\ 1 & T_s \le 0\end{cases}
$$

**Excitation** $e(t)$ ($e = 0$ when $\sigma = 0$ or `BT_MODE 0`):

Step:

$$
e(t) = \begin{cases}\sigma\, A & t_d \le t < t_d + t_w \\ 0 & \text{otherwise}\end{cases}
$$

with $A$ = `BT_STEP_MAG`, $t_d$ = `BT_STEP_DELAY`, $t_w$ = `BT_STEP_DUR`.

Ramp ($\rho$ = `BT_RAMP_RATE`, $e_{max}$ = `BT_MAX_VAL`):

$$
e(t) = \mathrm{clamp}(\sigma\,\rho\,t,\ -e_{max},\ e_{max})
\quad\text{until frozen, then held}
$$

The ramp freezes at the first sample where $|e| \ge e_{max}$ or any of the
first $N$ motors is saturated:

$$
\exists\, i < N:\quad u_i \ge 1 - m \quad\text{or}\quad u_i \le \begin{cases}-1 + m & \text{reversible}\\ m & \text{otherwise}\end{cases}
$$

with $u$ = `actuator_motors.control` (the normalised allocator output, before
the PWM/DShot scaling) and $m$ = `BT_SAT_MARGIN` (clamped to 0–0.5).

**Setpoints**, with $h$ = `BT_HOVER_THR`:

$$
F = \begin{bmatrix} e\,[\text{axis} = X] \\ e\,[\text{axis} = Y] \\ -h\,\lambda - e\,[\text{axis} = Z] \end{bmatrix},\qquad
\tau = \begin{bmatrix} e\,[\text{axis} = \text{roll}] \\ e\,[\text{axis} = \text{pitch}] \\ e\,[\text{axis} = \text{yaw}] \end{bmatrix}
$$

(each component then clamped to $[-1, 1]$). On the Z axis a positive
excitation adds upward thrust on top of the hover baseline. $F$ and $\tau$
go to the control allocator as `vehicle_thrust_setpoint` and
`vehicle_torque_setpoint`, and are mixed exactly as in flight (including
[sequential desaturation](DTRG_Sequential_Desaturation.md) and the
[CSV mixer](DTRG_CSV_Mixer.md) if enabled).

### Using the data (TBC)

For an effectiveness estimate on axis $j$, run steps of $\pm A$ on that axis
and fit the measured force/torque change against the motor command change,
$\Delta w = B\,\Delta u$, from the `actuator_motors` and load cell logs. With
the ramp, the frozen value tells you the largest command on that axis the
motors can deliver around the chosen hover thrust.

---

## 5. Logging

| Topic | |
| --- | --- |
| `vehicle_thrust_setpoint`, `vehicle_torque_setpoint` | what the module commanded |
| `actuator_motors` | normalised motor commands (and the saturation reference) |
| `actuator_outputs` | PWM / DShot values |
| `input_rc` | the direction switch |
| `vehicle_status.nav_state` | 16 while in the mode |
