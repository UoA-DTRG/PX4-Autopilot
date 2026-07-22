# Bench Test Mode

`bench_test` is a dedicated flight mode for **system identification** of a
rigidly mounted (bench) vehicle. It bypasses the attitude / rate controllers and
publishes `vehicle_thrust_setpoint` and `vehicle_torque_setpoint` directly to the
control allocator, so a single axis can be excited with a clean, repeatable
signal while thrust/torque, motor outputs, and IMU response are logged.

> ⚠️ **Safety**: This mode spins motors with the control loops disabled. Only run
> it on a vehicle that is **rigidly and securely mounted** to a test bench, with
> propellers removed or the rig fully guarded. The module is gated behind an
> explicit `BT_ARM_ENABLE` parameter so it cannot produce output by accident.

## How it works

- The mode is active only while `vehicle_status.nav_state ==
  NAVIGATION_STATE_BENCH_TEST`. Outside this mode the module publishes nothing.
- While active **and** armed **and** `BT_ARM_ENABLE == 1`, the module commands a
  hover baseline thrust and, on request, an excitation profile (step or ramp) on
  one chosen axis.
- The hover baseline is **soft-started**: it ramps up from zero to `BT_HOVER_THR`
  over `BT_SPINUP_T` seconds each time the outputs become active.
- The step/ramp profile is **started by an RC switch** (`BT_START_SW`), so you
  control exactly when the excitation begins.
- In **ramp** mode the excitation increases until a motor **saturates** (within a
  configurable margin) or the safety clamp `BT_MAX_VAL` is reached, then holds.
- **Arming and disarming** are permitted directly from the transmitter in this
  mode, even though a benched vehicle reports as "in air".

## Suggested test process

1. **Mount and secure** the vehicle to the bench. Remove props or fully guard the
   rig.
2. **Assign the mode** to a flight-mode switch position via `COM_FLTMODEx = 16`
   (Bench Test).
3. **Assign the start switch**: set `BT_START_SW` to the RC channel you will use
   to trigger the profile (e.g. `16` for CH16). Make sure that channel is mapped
   on your transmitter to a 2-position switch.
4. **Choose the excitation**:
   - `BT_AXIS` — which axis to excite (collective thrust, roll/pitch/yaw torque…).
   - `BT_MODE` — `1` for a step, `2` for a ramp (`0` = hover only).
   - `BT_SIGN` — direction of the excitation.
   - Set the hover baseline `BT_HOVER_THR` and the profile magnitudes
     (`BT_STEP_*` or `BT_RAMP_RATE` / `BT_MAX_VAL`).
5. **Set the saturation limits**: `BT_NUM_MOTORS` (or leave `0` to auto-detect)
   and `BT_SAT_MARGIN` (headroom before the hard limit, e.g. `0.05` → 0.95/0.05).
6. **Enable output**: set `BT_ARM_ENABLE = 1`. Keep this `0` until you are ready.
7. **Start logging** (`logger on` / SD card) so the response is captured.
8. **Switch to Bench Test mode** and **arm**. The motors soft-start to the hover
   baseline over `BT_SPINUP_T`.
9. **Flip the start switch high** to run the profile:
   - *Step*: waits `BT_STEP_DELAY`, applies `±BT_STEP_MAG` for `BT_STEP_DUR`,
     then returns to hover.
   - *Ramp*: increases at `BT_RAMP_RATE` until a motor saturates or `BT_MAX_VAL`
     is hit, then holds.
10. **Flip the start switch low** to end the run and return to the hover baseline.
    Each low→high flip re-runs the profile from the start.
11. **Disarm** (directly from the transmitter) and **switch out** of Bench Test
    mode when finished. Set `BT_ARM_ENABLE = 0`.

## Parameters

All parameters are in the **Bench Test** group.

### Mode and axis

| Parameter      | Type  | Default | Description |
|----------------|-------|---------|-------------|
| `BT_MODE`      | int   | `0`     | Excitation profile: `0` hover only, `1` step, `2` ramp. |
| `BT_AXIS`      | int   | `2`     | Axis under excitation: `0` thrust X, `1` thrust Y, `2` thrust Z (collective), `3` roll torque, `4` pitch torque, `5` yaw torque. Horizontal thrust axes (X, Y) are only realisable on fully-actuated / omni / tilt-rotor airframes. |
| `BT_SIGN`      | int   | `1`     | Sign/direction applied to the step and ramp output: `+1` or `-1`. |
| `BT_HOVER_THR` | float | `0.5`   | Baseline hover thrust (normalised, 0–1), published as `-BT_HOVER_THR` on the Z body axis (NED: −Z is up). |

### Step profile (`BT_MODE = 1`)

| Parameter       | Type  | Default | Unit | Description |
|-----------------|-------|---------|------|-------------|
| `BT_STEP_MAG`   | float | `0.1`   | norm | Absolute magnitude of the step on the chosen axis. |
| `BT_STEP_DELAY` | float | `2.0`   | s    | Delay from profile start to the step (hover only during this window). |
| `BT_STEP_DUR`   | float | `1.0`   | s    | How long the step is held before returning to hover. |

### Ramp profile (`BT_MODE = 2`)

| Parameter      | Type  | Default | Unit | Description |
|----------------|-------|---------|------|-------------|
| `BT_RAMP_RATE` | float | `0.05`  | 1/s  | Slope of the linear ramp on the chosen axis. |
| `BT_MAX_VAL`   | float | `0.3`   | norm | Safety clamp. The ramp normally stops when a motor saturates; this is the hard limit at which it also stops and holds, in case saturation is never reached. |

### Motor saturation (ramp stop condition)

| Parameter       | Type  | Default | Description |
|-----------------|-------|---------|-------------|
| `BT_NUM_MOTORS` | int   | `0`     | Number of connected motors to inspect for saturation (indices `0..N-1` of the motor outputs). `0` auto-detects from the finite `actuator_motors` channels. |
| `BT_SAT_MARGIN` | float | `0.05`  | Margin before the hard limit. A motor is saturated when its normalised output rises above `1 - margin` or falls below `0 + margin` (`-1 + margin` for reversible motors). E.g. `0.05` → 0.95 (upper) / 0.05 (lower). |

### Activation and safety

| Parameter       | Type  | Default | Unit | Description |
|-----------------|-------|---------|------|-------------|
| `BT_ARM_ENABLE` | int   | `0`     | bool | Master output gate. No thrust/torque is published unless this is `1`, even when armed and in the mode. |
| `BT_START_SW`   | int   | `0`     | —    | Raw RC channel (`input_rc`) used as the start switch for the step/ramp, e.g. `16` for CH16. Switch is high above 1500 µs. `0` disables the gate and starts on mode entry. |
| `BT_SPINUP_T`   | float | `2.0`   | s    | Time over which the hover baseline ramps up from zero when the outputs first become active (throttle soft-start). `0` applies hover thrust immediately. |

## Notes and behaviour details

- **Output gate vs. start switch** are independent: `BT_ARM_ENABLE` must be `1`
  for *any* output; `BT_START_SW` only controls when the excitation profile runs
  on top of the hover baseline.
- **Saturation** is evaluated on `actuator_motors.control` (the normalised
  allocator output), limited to the connected motors. This is the signal the
  allocator can actually deliver, before per-output PWM/DShot scaling.
- **Repeatability**: flipping the start switch low resets the profile clock (and
  the ramp freeze), so the next high flip runs an identical profile.
- **Disarm on the bench**: normally PX4 refuses to disarm while airborne and
  refuses to arm from RC into a non-manual mode. Both restrictions are lifted
  specifically for `NAVIGATION_STATE_BENCH_TEST`.

## Console

```
bench_test status     # shows mode, axis, sign, arm gate, start switch, ramp state
bench_test stop
bench_test start
```

The module is started automatically from `rc.mc_apps` and only produces output in
Bench Test mode.
