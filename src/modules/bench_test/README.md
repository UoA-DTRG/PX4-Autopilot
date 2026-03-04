# bench_test — Motor Bench Testing Module

Pre-arm motor excitation and characterisation module for multi-rotor vehicles.
Runs a suite of open-loop motor tests while the vehicle remains **disarmed**,
giving you clean, reproducible datasets for system identification, ESC
characterisation, and battery modelling without any flight risk.

---

## Table of Contents

1. [Safety Requirements](#1-safety-requirements)
2. [Building](#2-building)
3. [Starting the Module](#3-starting-the-module)
4. [Logging](#4-logging)
5. [Tests](#5-tests)
   - 5.1 [step — Single-Motor Step](#51-step--single-motor-step)
   - 5.2 [step_all — Sequential Step (All Motors)](#52-step_all--sequential-step-all-motors)
   - 5.3 [step_idle — Step with Background Throttle](#53-step_idle--step-with-background-throttle)
   - 5.4 [step_sim — Simultaneous Step (All Motors)](#54-step_sim--simultaneous-step-all-motors)
   - 5.5 [impulse — Single-Motor Impulse](#55-impulse--single-motor-impulse)
   - 5.6 [impulse_all — Sequential Impulse (All Motors)](#56-impulse_all--sequential-impulse-all-motors)
   - 5.7 [impulse_sim — Simultaneous Impulse (All Motors)](#57-impulse_sim--simultaneous-impulse-all-motors)
   - 5.8 [tweet — Chirp / Frequency Sweep](#58-tweet--chirp--frequency-sweep)
   - 5.9 [tweet_all — Chirp on All Motors Sequentially](#59-tweet_all--chirp-on-all-motors-sequentially)
   - 5.10 [multi — All Motors Simultaneously](#510-multi--all-motors-simultaneously)
   - 5.11 [sweep — Current-Draw Staircase](#511-sweep--current-draw-staircase)
   - 5.12 [sweep_all — Current Sweep (All Motors)](#512-sweep_all--current-sweep-all-motors)
   - 5.13 [flight — Full Flight-Profile Test](#513-flight--full-flight-profile-test)
6. [Voltage Compensator (`-c` flag)](#6-voltage-compensator--c-flag)
7. [Parameter Reference](#7-parameter-reference)
   - 7.1 [General](#71-general)
   - 7.2 [Step Tests](#72-step-tests)
   - 7.3 [Impulse Tests](#73-impulse-tests)
   - 7.4 [Chirp / Tweet Tests](#74-chirp--tweet-tests)
   - 7.5 [Multi-Motor Test](#75-multi-motor-test)
   - 7.6 [Current-Sweep Tests](#76-current-sweep-tests)
   - 7.7 [Flight Test](#77-flight-test)
   - 7.8 [Voltage Compensator](#78-voltage-compensator)
8. [Log Format & uORB Topics](#8-log-format--uorb-topics)
9. [Implementation Notes](#9-implementation-notes)

---

## 1. Safety Requirements

> **WARNING — Remove all propellers before running any bench test.**

The module enforces the following hard safety rules before and during every
test:

| Check | Behaviour |
|---|---|
| Vehicle armed | Refused at start; aborts immediately if armed during a test |
| RC kill switch (`manual_control_switches.kill_switch`) | Polled every cycle (100 Hz); immediately aborts and releases all motors |
| ESC actuator_test timeout | Every motor command carries a timeout so ESCs stop even if this module crashes |

---

## 2. Building

The module is **disabled by default**. Enable it via `menuconfig` or by
passing the Kconfig symbol on the command line:

```bash
# Interactive menuconfig (navigate to Modules → bench_test)
make <board_target> menuconfig

# Or set directly in the board's default config / overlay
CONFIG_MODULES_BENCH_TEST=y
```

The module depends on `SystemIdentification` (the multisine excitation
library) and the standard `px4_work_queue` support library, both of which
are included in all flight-controller builds.

---

## 3. Starting the Module

```bash
# Start the background work-queue task
bench_test start

# Check state, motor count, kill-switch, ESC telemetry, power monitor
bench_test status

# Stop the module
bench_test stop
```

Once started the module idles at 100 Hz, monitoring the kill switch and the
arming state. All test subcommands are blocking: they run the full test
sequence synchronously inside `custom_command()` and return when done.

---

## 4. Logging

The PX4 SD-logger captures all relevant data. Two convenience commands wrap
the `logger on` / `logger off` shell commands via a short-lived spawned task
(necessary to avoid a mutex deadlock — see [Implementation Notes](#9-implementation-notes)):

```bash
bench_test log_start   # start SD logging (arm-override not needed — vehicle is disarmed)
bench_test log_stop    # stop logging
```

Key topics logged by the normal uLog pipeline during a bench test:

| Topic | Content |
|---|---|
| `esc_status` | Per-ESC eRPM, voltage, current (bidirectional DShot) |
| `actuator_test` | Raw motor commands published by this module |
| `battery_status` | Pack voltage, current, state-of-charge |
| `power_monitor` | Voltage, current, power from the power module (INA226 etc.) |
| `bench_test_vc_status` | Voltage-compensator internals — see §8 |
| `log_message` | Phase markers injected during the flight test (§5.13) |
| `multisine_excitation_status` | Multisine progress published by the lib |

A typical session:

```bash
bench_test start
bench_test log_start
bench_test step -m 1
bench_test step -m 2
# ... more tests ...
bench_test log_stop
bench_test stop
```

---

## 5. Tests

All tests share common options:

| Flag | Meaning |
|---|---|
| `-m <N>` | Motor number, 1-based (required for single-motor subcommands) |
| `-c` | Enable the battery voltage compensator (see §6) |

### 5.1 `step` — Single-Motor Step

```bash
bench_test step -m <motor> [-c]
```

Applies a step excitation to one motor while holding all other motors at
`BT_BASE_LVL`. The sequence is:

```
[optional] ramp all → BT_BASE_LVL  (BT_RAMP_TIME ms)
[optional] hold all at BT_BASE_LVL (BT_BG_SETL ms, only if BT_BASE_LVL > 0)
ramp target motor → BT_STEP_LVL   (BT_RAMP_TIME ms)
hold target at    BT_STEP_LVL     (BT_STEP_DUR ms)
ramp target back  → BT_BASE_LVL   (BT_RAMP_TIME ms)
ramp all → 0                       (BT_RAMP_TIME ms)
```

**Use case**: ESC / motor step-response identification. Set `BT_BASE_LVL = 0`
for cold-start tests; set it to `~0.10` to measure small-signal dynamics
around an idle operating point.

### 5.2 `step_all` — Sequential Step (All Motors)

```bash
bench_test step_all [-c]
```

Runs `step` on each motor from 1 to `BT_NUM_MOTORS` in sequence, with
`BT_INTER_DLY` ms between each motor.

### 5.3 `step_idle` — Step with Background Throttle

```bash
bench_test step_idle -m <motor> [-c]
```

Like `step`, but all *other* motors spin at `BT_BG_LVL` (the "idle wind"
level) throughout, while the target motor starts from `BT_BASE_LVL`. The
test is otherwise identical in amplitude and timing to a plain step, so
results are directly comparable.

```
ramp bg motors → BT_BG_LVL  (target stays at BT_BASE_LVL)  (BT_RAMP_TIME ms)
hold bg at BT_BG_LVL        (target at BT_BASE_LVL)         (BT_BG_SETL ms)
ramp target → BT_STEP_LVL   (bg stays at BT_BG_LVL)         (BT_RAMP_TIME ms)
hold                                                          (BT_STEP_DUR ms)
ramp target back → BT_BASE_LVL                               (BT_RAMP_TIME ms)
ramp all → 0                                                  (BT_RAMP_TIME ms)
```

**Use case**: Quantify the aerodynamic coupling between motors (blade-passing
frequency, induced-velocity interference) under realistic loaded conditions.

### 5.4 `step_sim` — Simultaneous Step (All Motors)

```bash
bench_test step_sim [-c]
```

Steps all `BT_NUM_MOTORS` motors simultaneously to `BT_STEP_LVL`, using the
same ramp/settle/hold/ramp-down sequence as `step`.

**Use case**: Battery voltage-sag characterisation under full-vehicle thrust.

### 5.5 `impulse` — Single-Motor Impulse

```bash
bench_test impulse -m <motor> [-c]
```

Fires a short burst (`BT_IMP_DUR` ms) on one motor at `BT_BASE_LVL +
BT_IMP_LVL` (clamped to 1.0), optionally preceded by a settle phase at
`BT_BASE_LVL`.

**Use case**: ESC non-minimum-phase behaviour; high-bandwidth current spike
characterisation.

### 5.6 `impulse_all` — Sequential Impulse (All Motors)

```bash
bench_test impulse_all [-c]
```

Runs `impulse` on each motor sequentially, with `BT_INTER_DLY` ms between
motors.

### 5.7 `impulse_sim` — Simultaneous Impulse (All Motors)

```bash
bench_test impulse_sim [-c]
```

Fires all motors simultaneously at `BT_BASE_LVL + BT_IMP_LVL` for
`BT_IMP_DUR` ms.

**Use case**: Battery impedance identification under maximum pulse current.

### 5.8 `tweet` — Chirp / Frequency Sweep

```bash
bench_test tweet -m <motor>
```

Modulates one motor with a linear chirp (linearly increasing frequency)
around a DC bias:

$$
u(t) = \text{BT\_TWE\_BIAS} + \text{BT\_TWE\_AMP} \cdot \sin\!\left(2\pi \left(f_\text{start} \cdot t + \frac{f_\text{end} - f_\text{start}}{2T} t^2\right)\right)
$$

The output is clamped to [0, 1]. Commands are sent at ~500 Hz (2 ms loop)
for a smooth signal.

**Use case**: Motor / ESC frequency-response identification without requiring
the full orthogonal multisine setup. Good for quick bandwidth estimation.

### 5.9 `tweet_all` — Chirp on All Motors Sequentially

```bash
bench_test tweet_all
```

Runs `tweet` on each motor from 1 to `BT_NUM_MOTORS`, with `BT_INTER_DLY`
between motors.

### 5.10 `multi` — All Motors Simultaneously

```bash
bench_test multi
```

Spins all `BT_NUM_MOTORS` motors simultaneously at `BT_MULTI_LVL` for
`BT_MULTI_DUR` ms, with ramp-up and ramp-down.

**Use case**: Total power / thermal verification; propeller-off vibration
baseline.

### 5.11 `sweep` — Current-Draw Staircase

```bash
bench_test sweep -m <motor>
```

Steps one motor through `BT_ISWEEP_N` linearly spaced throttle levels
from `BT_ISWEEP_STA` to `BT_ISWEEP_END`, dwelling `BT_ISWEEP_DWL` ms
at each step while logging voltage, current, and eRPM from the ESC.

**Use case**: Deriving the static thrust–current–eRPM map used to calibrate
the voltage compensator's speed estimator (`BT_VC_TW*`, `BT_VC_TI*`).

### 5.12 `sweep_all` — Current Sweep (All Motors)

```bash
bench_test sweep_all
```

Runs `sweep` on each motor sequentially.

### 5.13 `flight` — Full Flight-Profile Test

```bash
bench_test flight
```

A fully automated multi-phase profile intended to be run while the vehicle
is **on a test stand or in the air under external control**. All motors are
commanded at the same level (no attitude or position control).

Sequence:

| Phase | What happens |
|---|---|
| 1 | Ramp all motors from 0 → `BT_FLT_HOVR` over `BT_RAMP_TIME` ms |
| 2 | Hold hover throttle for `BT_FLT_HOVT` ms (settle) |
| 3 | **Orthogonal multisine excitation** (reads `DTRG_MSINE_*` params; 250 Hz update; see below) |
| 4 | Slow ramp to `BT_FLT_HIHI`, hold `BT_FLT_STPH` ms, ramp back to hover |
| 5 | Slow ramp to `BT_FLT_LOLO`, hold `BT_FLT_STPH` ms, ramp back to hover |
| 6 | `BT_FLT_BLPN` simultaneous impulse pairs: UP burst → stabilise → DOWN burst → stabilise |
| 7 | Ramp all motors to 0 |

A `log_message` marker is injected into the ULog at the start of each phase:
`[bench_test] Phase N: <description>`. These markers appear in Flight Review
and are extractable with `pyulog`.

**Phase 3 — Multisine parameters** (from `DTRG_MSINE_*`):

| DTRG Parameter | Default | Meaning |
|---|---|---|
| `DTRG_MSINE_NMOT` | 8 | Motors to excite (capped to `BT_NUM_MOTORS`) |
| `DTRG_MSINE_AMP` | 0.1 | Excitation amplitude (normalised, added to hover throttle) |
| `DTRG_MSINE_T` | 30.0 s | Period per motor |
| `DTRG_MSINE_FMIN` | 0.1 Hz | Lowest excitation frequency |
| `DTRG_MSINE_FMAX` | 1.0 Hz | Highest excitation frequency |
| `DTRG_MSINE_SEQ` | 0 | 0 = simultaneous, 1 = sequential per motor |

**Phase 6 — SoC-based stop condition** (`BT_FLT_BLPS > 0`):  
Instead of always running exactly `BT_FLT_BLPN` pairs, the loop continues
until the battery state of charge drops to or below `BT_FLT_BLPS` % — useful
for battery-discharge characterisation across the full SoC range. Count
`BT_FLT_BLPN` remains a hard cap.

**Voltage cutoff** (`BT_FLT_VMIN > 0`):  
If pack voltage drops below `BT_FLT_VMIN` V at any phase boundary, the test
is immediately aborted and motors are ramped to zero.

---

## 6. Voltage Compensator (`-c` flag)

Several tests accept the optional `-c` flag to activate the battery voltage
compensator. Its purpose is to keep the effective voltage applied to each
motor (δ · V_b) constant across the test, regardless of battery state of
charge or mid-burst voltage sag, enabling repeatable excitation signals that
are independent of pack condition.

The compensator runs on the **target motor only**; background motors track
the raw commanded value so the battery model stays current-accurate.

### Algorithm

At every motor command the compensator:

1. Estimates rotor speed from the commanded delta and the last known speed:

$$
\omega_k = \frac{\theta_{\omega 1} \, V_\delta + \theta_{\omega 2} \sqrt{V_\delta} + \theta_{\omega 3} - \theta_{\omega 4} \, \omega_{k-1}}{1 + \theta_{\omega 4}}
$$

where $V_\delta = \delta \cdot V_{b,\text{op}}$.

2. Estimates total current draw from all rotors:

$$
I_\text{tot} = \sum_{i=1}^{N} \left( \theta_{I1} \, \omega_i + \theta_{I2} \, \omega_i^2 + \theta_{I3} \right)
$$

3. Evaluates a first-order RC battery model with SoC-dependent parameters
   (each a 3rd-order polynomial in SoC $s \in [0,1]$):

$$
V_0(s) = c_0 + c_1 s + c_2 s^2 + c_3 s^3 \quad (\text{OCV})
$$
$$
V_{\text{RC},k} = V_{\text{RC},k-1} + T_s \left( \frac{R_1}{\tau_1} I_\text{tot} - \frac{1}{\tau_1} V_{\text{RC},k-1} \right)
$$
$$
\hat{V}_b = V_0 - I_\text{tot} \cdot R_0 - V_{\text{RC}}
$$

4. Scales the command:

$$
\delta' = \frac{V_{b,\text{op}}}{\hat{V}_b} \cdot \delta \quad \text{(clamped to } [0, \text{BT\_VC\_MAXCMD}]\text{)}
$$

### Configuring the Compensator

1. Run `sweep_all` to obtain static eRPM–throttle–voltage–current data.
2. Fit the speed estimator coefficients (`BT_VC_TW1..TW4`) and current
   estimator coefficients (`BT_VC_TI1..TI3`) to the measured data.
3. From a battery characterisation ramp or cell datasheet, fit the four
   SoC polynomials (`V0`, `R0`, `R1`, `τ1`) and set `BT_VC_V0C0..T1C3`.
4. Set `BT_VC_VBOP` to the mid-SoC terminal voltage of your pack (e.g.
   `22.2` for a 6S LiPo at ~50% SoC).
5. Verify with `bench_test step -m 1 -c` and inspect `bench_test_vc_status`
   in the log: `c_delta` should be close to 1.0 at the start and drift
   upward as the battery sags.

---

## 7. Parameter Reference

All parameters are in the `Bench Test` group and use the `BT_` prefix.

### 7.1 General

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `BT_NUM_MOTORS` | 8 | — | Total number of motors on the vehicle |
| `BT_BASE_LVL` | 0.00 | — | Baseline throttle held before and between step/impulse excitations |
| `BT_RAMP_TIME` | 200 | ms | Linear ramp time at test start/end (prevents current-limit trips) |
| `BT_INTER_DLY` | 500 | ms | Delay between motors in sequential `*_all` tests |

### 7.2 Step Tests

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `BT_STEP_LVL` | 0.15 | — | Target throttle for the step excitation |
| `BT_STEP_DUR` | 1000 | ms | Duration to hold the step level |
| `BT_BG_LVL` | 0.10 | — | Background ("idle") throttle for other motors in `step_idle` |
| `BT_BG_SETL` | 5000 | ms | Settle time at `BT_BG_LVL` before the step fires (also used as baseline settle when `BT_BASE_LVL > 0`) |

### 7.3 Impulse Tests

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `BT_IMP_LVL` | 0.30 | — | Impulse amplitude (added on top of `BT_BASE_LVL`, clamped to 1.0) |
| `BT_IMP_DUR` | 150 | ms | Duration of the impulse burst |

### 7.4 Chirp / Tweet Tests

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `BT_TWE_F_STA` | 5.0 | Hz | Start frequency of the chirp sweep |
| `BT_TWE_F_END` | 80.0 | Hz | End frequency of the chirp sweep |
| `BT_TWE_AMP` | 0.10 | — | Peak amplitude of the sinusoidal chirp |
| `BT_TWE_BIAS` | 0.20 | — | DC bias throttle around which the chirp oscillates |
| `BT_TWE_DUR` | 5000 | ms | Total duration of the chirp sweep |

### 7.5 Multi-Motor Test

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `BT_MULTI_LVL` | 0.15 | — | Throttle for the all-motors-simultaneous test |
| `BT_MULTI_DUR` | 2000 | ms | Hold duration |

### 7.6 Current-Sweep Tests

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `BT_ISWEEP_STA` | 0.05 | — | First throttle step level |
| `BT_ISWEEP_END` | 0.60 | — | Last throttle step level |
| `BT_ISWEEP_N` | 6 | — | Number of discrete steps |
| `BT_ISWEEP_DWL` | 2000 | ms | Dwell time at each step |

### 7.7 Flight Test

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `BT_FLT_HOVR` | 0.35 | — | Hover throttle level (Phase 1–2 target, baseline for impulses) |
| `BT_FLT_HOVT` | 3000 | ms | Hover settle time before multisine excitation |
| `BT_FLT_HIHI` | 0.60 | — | High-throttle step target (Phase 4) |
| `BT_FLT_LOLO` | 0.15 | — | Low-throttle step target (Phase 5) |
| `BT_FLT_STPH` | 10000 | ms | Hold time at high/low step level |
| `BT_FLT_STPR` | 1000 | ms | Ramp time for the slow step transitions |
| `BT_FLT_BLPN` | 5 | — | Number of impulse pairs (Phase 6) |
| `BT_FLT_BLPA` | 0.20 | — | Impulse amplitude (added/subtracted from hover) |
| `BT_FLT_BLPD` | 300 | ms | Duration of each impulse burst |
| `BT_FLT_BLPI` | 2000 | ms | Stabilisation interval between impulse bursts |
| `BT_FLT_BLPS` | 0.0 | % | Stop SoC level for impulse loop (0 = fixed count only) |
| `BT_FLT_VMIN` | 0.0 | V | Hard voltage cutoff (0 = disabled) |

### 7.8 Voltage Compensator

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `BT_VC_VBOP` | 0.0 | V | Nominal operating-point voltage. Set > 0 to enable the compensator |
| `BT_VC_TW1` | 0.0 | — | Speed estimator θ_ω1 (linear voltage term) |
| `BT_VC_TW2` | 0.0 | — | Speed estimator θ_ω2 (√voltage term) |
| `BT_VC_TW3` | 0.0 | — | Speed estimator θ_ω3 (constant term) |
| `BT_VC_TW4` | 0.0 | — | Speed estimator θ_ω4 (backward-Euler dynamics) |
| `BT_VC_TI1` | 0.0 | — | Current estimator θ_I1 (linear speed term per rotor) |
| `BT_VC_TI2` | 0.0 | — | Current estimator θ_I2 (quadratic speed term) |
| `BT_VC_TI3` | 0.0 | — | Current estimator θ_I3 (constant / no-load term) |
| `BT_VC_V0C0..V0C3` | 0.0 | V | OCV polynomial coefficients c0–c3 |
| `BT_VC_R0C0..R0C3` | 0.0 | Ω | Series resistance polynomial coefficients |
| `BT_VC_R1C0..R1C3` | 0.0 | Ω | RC-branch resistance polynomial coefficients |
| `BT_VC_T1C0..T1C3` | 0.0 | s | RC-branch time-constant polynomial coefficients |
| `BT_VC_MAXCMD` | 1.0 | — | Hard cap on the compensated command (prevents over-speed) |

---

## 8. Log Format & uORB Topics

### `bench_test_vc_status` (published when `-c` is active)

Only published for the **target motor** on each command cycle. Array fields
are indexed 0–7 by motor slot.

| Field | Type | Description |
|---|---|---|
| `timestamp` | `uint64` | Time since boot (µs) |
| `motor_index` | `uint8` | 0-based index of the motor being compensated |
| `n_motors` | `uint8` | Number of configured motors |
| `delta_raw[8]` | `float32` | Raw normalised commands sent to each motor slot |
| `delta_comp[8]` | `float32` | Compensated command for target; raw for background motors |
| `c_delta[8]` | `float32` | Correction factor = `delta_comp / delta_raw`; 1.0 for non-targets |
| `soc` | `float32` | Battery state of charge [0, 1]; −1 if unavailable |
| `v_b_pred` | `float32` | Predicted terminal voltage (V) |
| `i_total` | `float32` | Predicted total current draw (A) |

### `log_message` (flight test phase markers)

Published at the start of each flight-test phase with severity INFO (6):

```
[bench_test] Phase 1: Ramp to hover
[bench_test] Phase 2: Hover settle
[bench_test] Phase 3: Multisine excitation
[bench_test] Phase 4: Step up
[bench_test] Phase 5: Step down
[bench_test] Phase 6: Impulse pairs
[bench_test] Phase 7: Ramp down to zero
```

These appear in the Flight Review timeline and can be extracted with:

```bash
ulog_messages <logfile.ulg>
```

or with `pyulog`:

```python
from pyulog import ULog
log = ULog('log.ulg')
msgs = log.get_dataset('log_message')
```

### High-Rate ESC Telemetry (`esc_status`)

Bidirectional DShot feeds eRPM back to `esc_status` independently of this
module. To maximise data rate, configure the logger to log `esc_status` at
the full publication rate (typically 100–400 Hz depending on ESC firmware).

---

## 9. Implementation Notes

### Architecture

- `BenchTest` inherits `ScheduledWorkItem` (work queue: `lp_default`) and
  `ModuleBase<BenchTest>`.
- The scheduled callback runs at **100 Hz** (10 ms interval). It only polls
  for kill-switch and arming changes; all test sequences run synchronously
  in `custom_command()` (the calling thread).
- State machine: `TestState { IDLE, RUNNING, ABORTING }` and
  `TestType { NONE, STEP_SINGLE, STEP_ALL, STEP_IDLE_BG, STEP_SIMULTANEOUS,
  IMPULSE_SINGLE, IMPULSE_ALL, IMPULSE_SIMULTANEOUS, TWEET_SINGLE, TWEET_ALL,
  MULTI, CURRENT_SWEEP_SINGLE, CURRENT_SWEEP_ALL, FLIGHT }`.

### Motor Commands

All motor commands use `actuator_test_s` published to the `actuator_test`
uORB topic. Each message carries:

- `function` = `FUNCTION_MOTOR1 + motor_index`
- `value` in [0, 1] (or `NaN` for `ACTION_RELEASE_CONTROL`)
- `timeout_ms` — the ESC driver stops the motor if no refresh arrives within
  this window. Every test sets a generous timeout so the ESC acts as a
  passive watchdog.

`releaseAllMotors()` publishes `ACTION_RELEASE_CONTROL` with `value = NaN`
to all motor slots, returning control to the normal ESC driver immediately.

### Chirp (tweet) Signal Generation

The tweet test uses a **linear chirp** with analytically integrated phase:

```cpp
float phase = 2π * (f_start * t + 0.5 * (f_end - f_start) * t² / T);
float value = bias + amp * sinf(phase);
```

Update interval is 2 ms (~500 Hz), well above the `f_end` default of 80 Hz.

### Multisine Excitation (flight test, Phase 3)

Uses `multisine::MultisineExcitation` from
`src/lib/system_identification/multisine_excitation.hpp`:

- **Harmonic combing**: the frequency band [f_min, f_max] is divided into
  non-overlapping harmonic subsets, one per motor, ensuring orthogonality
  between motor channels.
- **Schroeder phases**: coefficients are chosen to minimise the peak-to-RMS
  ratio (crest factor) of the composite signal.
- In **simultaneous mode** (`DTRG_MSINE_SEQ = 0`), all motors excite for the
  full period T at the same time. In **sequential mode**, each motor is
  excited for T/N_motors seconds in turn.
- Commands run at **250 Hz** (4 ms `px4_usleep`) and are clamped to [0, 1].

### Logger Command Deadlock Workaround

Calling `logger_main()` directly from `custom_command()` would deadlock:
`BenchTest::main()` holds `px4_modules_mutex` (global, shared across all
`ModuleBase<T>` specialisations), and `Logger::main()` → `custom_command()`
tries to acquire the same mutex.

The workaround spawns a tiny short-lived task outside the lock:

```cpp
int tid = px4_task_spawn_cmd("bench_logger_cmd", ...,
    [](int, char *av[]) -> int { return logger_main(2, av); },
    spawn_argv);
```

This is equivalent to typing `logger on` / `logger off` at the NSH console.

### Voltage Compensator Thread Safety

The `VoltageCompensator` struct stores `delta_prev[8]` — the most recent
commanded delta for every motor. When a non-target motor's command is
updated, `compensatedCommandMotor(… is_target=false …)` writes its slot
directly. Because all tests run serially (no parallel threads), there is
no race condition on this array.
