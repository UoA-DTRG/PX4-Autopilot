# Admittance Control — Implementation Plan

Make the module build, run, and drive Position mode.

## Context

`src/modules/admittance_control/` is a new, **untracked** module implementing a gain-scheduled
admittance controller (virtual mass–spring–damper driven by the external wrench from
`rls_wrench_estimator`). Today it is inert: it is in no board config, no startup script, has no
`Kconfig`, **cannot compile**, and nothing consumes its output.

Three things are needed:

1. Its output must actually reach the position controller, with a bypass that still computes and
   logs the expected output.
2. It must behave correctly in Position mode, where the setpoint is frequently velocity-only
   (`position[] = NAN`) — the module currently rejects that case outright.
3. It must be built and started on `px4_fmu-v5`, `px4_fmu-v6c`, `px4_fmu-v6xrt`.

Along the way the hard blockers and the flight-safety bugs get fixed.

---

## Part 0 — Analysis findings (what is broken today)

### Will not compile

| # | Issue | Location |
|---|---|---|
| B1 | `ORB_ID(admittance_setpoint)` does not exist — no msg, no `# TOPICS` alias anywhere | `AdmittanceControlModule.hpp:97` |
| B2 | `px4::params::RLS_EST_N_ROTORS` does not exist — deleted by the RLS refactor (`../rls_wrench_estimator/REFACTOR_PLAN.md:76`) | `AdmittanceControlModule.hpp:115` |
| B3 | `vehicle_local_position_setpoint_s` has **no `jerk` member** on this branch — 6 references | `AdmittanceControl/AdmittanceControl.cpp:144-146` |
| B4 | No `Kconfig` → `add_subdirectory()` is never called, module is never built, and its `ADM_CTR_*` params are never generated | this directory |

### Wrong at runtime

- **H4 — type-punned subscription.** `_trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)}` is copied
  into a `vehicle_local_position_setpoint_s` (`AdmittanceControlModule.cpp:264`). `Subscription::copy()`
  takes `void*`, so it compiles. Both structs are 64 bytes and by coincidence pos/vel/accel/yaw/yawspeed
  all land correctly — but `jerk[3]` lands on `thrust[3]`. Silent, and breaks the moment either msg changes.
- **H5 — `U.abs()` result discarded.** `matrix::Matrix::abs()` returns a new matrix
  (`AdmittanceControl.cpp:74`). So `U.max()` is the max of *signed* `pwm-1500`: lower-rail saturation is
  invisible, and when all outputs sit below 1500 `_sat_factor` goes negative, which *stiffens* instead of
  softening — inverted behaviour.
- **M15 — unused-rotor zeroing interacts badly.** `pwm[4..7] = 0` happens *before* the `-1500` offset.
  With H5 fixed, those become `|−1500| = 1500` and permanently peg `_sat_factor` at 3.0.
- **M14 — hardcoded PWM 1500/500.** Wrong for DShot, and for `actuator_outputs_sim` (SITL, range
  `[-1,1]`) it yields `_sat_factor = −3`.
- **M9 — divide by zero.** `a[i] = A[i] − _sat_factor` is used as a denominator with no guard; `A` may be
  as low as 0.01. `NaN`/`inf` propagates through `M`→`C`→ the integrator → the published setpoint, and
  **the output is never finiteness-checked**.
- **M13 — param change zeroes the integrator in flight.** `updateParams()` → `_control.initialize()` →
  `_y = zeros()`, without clearing `_valid`, so `reset()` is not called. Any 1 Hz param write snaps the
  published setpoint toward the origin.
- **H6 — frame handling.** Absolute NED positions are rotated by `v_att_sp.q_d` (the *full desired*
  attitude, incl. roll/pitch tilt), the ODE state `_y` is stored in that rotating frame across steps, and
  `reset()` seeds it with unrotated NED values. Yaw is never wrapped to ±π despite the msg contract.
- **H7 — no gating.** The only enable is `debug_vect.y > 0.5`, and the 10-char `name` field is never
  checked, so *any* publisher of `debug_vect` enables the controller. No arming, mode, or `wrench.valid`
  check.
- **H8 — `stop` hangs.** `Run()` only ever executes from the `rls_wrench_estimator` callback. If the
  estimator is not running (`RLS_EST_EN != 1`), `admittance_control stop` blocks forever.
- **Test cannot pass:** `EXPECT_EQ(0, cosf(-M_PI/4))` — `AdmittanceControl/AdmittanceControlTest.cpp:200`.

### Not registered anywhere

Repo-wide, `admittance` appears outside this directory in exactly two places: a `README.md` bullet, and a
commented-out consumer at `../mc_pos_control/MulticopterPositionControl.hpp:118`. Nothing in `boards/`,
nothing in `ROMFS/`.

---

## Part 1 — How this works in Position mode (the velocity-only question)

**The live Position-mode task is `FlightTaskManualAcceleration`**, not `FlightTaskManualPosition`
(`MPC_POS_MODE` defaults to 4 — `../flight_mode_manager/FlightModeManager.cpp:206-230`). Sticks map to
*acceleration*, which is forward-integrated to velocity
(`../flight_mode_manager/tasks/Utility/StickAccelerationXY.cpp:108-137`). The position setpoint is opened
and closed by `lockPosition()` (`StickAccelerationXY.cpp:202-221`):

- **Stick centred and stopped** → `_position_setpoint = pos.xy()` (position lock). Finite pos + ~0 vel.
- **Stick deflected** → `_position_setpoint.setNaN()`, and `_velocity_setpoint` is re-seeded from
  `vel_sp_feedback` to avoid a jump. Only velocity + acceleration are finite.

So in Position mode `position[0..1]` is NAN roughly half the time. The current module's
`copyAndCheckAllFinite()` returns `false` whenever `sp.x/y/z/yaw` is non-finite, so **admittance would
never engage while the pilot is moving** — exactly when contact is most likely.

The constraint that governs any fix is `PositionControl::_inputValid()`
(`../mc_pos_control/PositionControl/PositionControl.cpp:224-250`): X and Y must be **NAN-or-finite in
pairs, per level**. You cannot make `position[0]` finite while `position[1]` stays NAN.

### Design: deviation-form ODE + an internal reference position

**Reformulate the ODE in terms of the deviation `Δ` rather than absolute position.** The current form is

```
M ẍ + C ẋ + K x = We + Fd ,   Fd = M·a_sp + C·v_sp + K·p_sp
```

Substituting `x = p_sp + Δ` cancels `Fd` exactly:

```
M Δ̈ + C Δ̇ + K Δ = We
```

This is **mathematically equivalent** for a kinematically consistent reference, and it removes four
separate problems at once: `Δ` is a *relative* vector so rotating it is meaningful (H6); `reset()`
becomes `Δ = 0` with no discontinuity; a param change zeroing the state is now harmless (M13); and a NAN
input position no longer breaks the integration.

Then per axis:

| Incoming setpoint | Reference `p_ref` | Published output |
|---|---|---|
| `position[i]` finite (stick centred / lock) | `p_ref(i) = position[i]` | `position[i] = p_ref(i) + Δ_ned(i)` |
| `position[i]` NAN (stick deflected) | `p_ref(i) += velocity[i]·dt`, seeded from measured position on the finite→NAN transition, then **clamped to measured position ± `ADM_CTR_REF_MAX`** | `position[i] = p_ref(i) + Δ_ned(i)` |

Velocity and acceleration are always `velocity_in[i] + Δ̇_ned(i)` and `acceleration_in[i] + Δ̈_ned(i)`
(NAN inputs substituted with 0). Because the module always emits a finite XY position pair when engaged,
the pair invariant holds.

**Why the clamp is the safety story.** Emitting a finite position while the pilot commands velocity
closes the position loop on the reference error with gain `MPC_XY_P`. The clamp bounds the extra velocity
command to `MPC_XY_P × ADM_CTR_REF_MAX` — with defaults `0.95 × 0.5 m = 0.475 m/s`. Without it, any
mismatch between commanded and achieved velocity integrates without bound and the vehicle overshoots when
the pilot re-centres.

Additionally: engage/disengage ramps `Δ` in and out over `ADM_CTR_RAMP` (default 0.5 s) so neither
transition is a step.

---

## Part 2 — Getting the output into the position controller

Chosen topology (matches the intent already recorded at `../mc_pos_control/MulticopterPositionControl.hpp:112,118`):

```
flight_mode_manager ──trajectory_setpoint──┬──────────────────────────────┐
                                           │                              │
                                  admittance_control                      │
                                    │            │                        │
                          admittance_setpoint  admittance_status          │
                          (TrajectorySetpoint)  (always published)        │
                                    │            │                        │
                                    └──> mc_pos_control <─── engaged? ────┘
                                            (falls back to trajectory_setpoint)
```

**Two topics, deliberately:**

- `admittance_setpoint` — a plain **`# TOPICS` alias of `TrajectorySetpoint`**, so mc_pos_control's
  consumption is a type-identical drop-in with zero conversion code. (`msg/versioned/` messages already
  carry TOPICS aliases — see `VehicleAttitudeSetpoint.msg:18`, `ManualControlSetpoint.msg:44` — so this
  is a supported pattern here.)
- `admittance_status` — a new fork-local message carrying the `engaged` flag **plus diagnostics**. This
  is what makes bypass-with-logging work, and it keeps mc_pos_control decoupled: it reads a topic field
  rather than an `ADM_CTR_*` parameter, so mc_pos_control still compiles on boards where the module is
  not built.

**Failsafe by construction:** if the module is stopped, crashed, bypassed, or stale, `engaged` is false
or the topic is old → mc_pos_control uses `trajectory_setpoint` unchanged.

### `ADM_CTR_EN` semantics

| Value | Module started | Computes + publishes + logs | Affects flight |
|---|---|---|---|
| 0 | no | — | no |
| 1 | yes | yes (`engaged = false`) | **no — this is the bypass** |
| 2 | yes | yes (`engaged = true` when all gates pass) | yes |

The RC aux switch (`ADM_CTR_RC_CH`, default 0 = unused) can force `engaged = false` in flight without a
param write. Gates for `engaged`: `ADM_CTR_EN == 2` **and** RC switch high (if configured) **and**
`wrench.valid` **and** armed **and** `nav_state` in the `ADM_CTR_NAV` bitmask (default POSCTL | OFFBOARD)
**and** the computed output is finite.

---

## Part 3 — File-by-file changes

### New files

| File | Content |
|---|---|
| `src/modules/admittance_control/Kconfig` | `menuconfig MODULES_ADMITTANCE_CONTROL` / `bool "admittance_control"` / `default n` / `---help---`. Copy `../rls_wrench_estimator/Kconfig` verbatim, renamed. |
| `msg/AdmittanceStatus.msg` | `timestamp`, `bool engaged`, `bool valid`, `uint8 bypass_reason`, `float32[4] wrench_used`, `float32[4] mass`, `float32[4] damping`, `float32[4] stiffness`, `float32[3] deviation`, `float32[3] deviation_rate`, `float32[3] reference_position`, `float32 sat_factor`, `float32 target_dist`. Publishing the *expected* deviation even when bypassed is the whole point. |

### uORB registration

- `msg/versioned/TrajectorySetpoint.msg` — append `# TOPICS trajectory_setpoint admittance_setpoint`.
  The default name **must** be re-listed or the existing alias is lost.
- `msg/CMakeLists.txt` — add `AdmittanceStatus.msg` next to the other fork-local entries
  (`HorizontalThrustLimit.msg`, `SequentialDesaturation.msg`, ~line 268).

### `AdmittanceControl/AdmittanceControl.{hpp,cpp}`

- Switch the carrier type from `vehicle_local_position_setpoint_s` to `trajectory_setpoint_s`
  (fixes **B3** and **H4** together — `jerk` exists on `TrajectorySetpoint`, `thrust` does not).
- Rewrite the state as the deviation `Δ` (Part 1). `Fd` disappears; `func()` becomes
  `M Δ̈ + C Δ̇ + K Δ = We`. Keep the existing RK4 (`_integrate_rk4`) unchanged.
- Rotate only the **wrench**, and only into the **yaw-aligned heading frame**, using measured attitude
  from `vehicle_attitude` (not `v_att_sp.q_d`): `We_heading = R_yaw(ψ)ᵀ · R(q) · fe_body`. Rotate `Δ`,
  `Δ̇`, `Δ̈` back with `R_yaw(ψ)`. This preserves the per-axis (fore/aft vs lateral vs vertical) tuning the
  parameters were written for.
- Guard `a[i] = math::max(A[i] − _sat_factor, 0.05f)` (**M9**); clamp `b[i] ≥ 0.01`.
- `wrap_pi()` the yaw output.
- Split `initialize()` into `setParams()` (copies bell params only) and `reset()` (zeroes `Δ`) so a param
  update no longer zeroes state (**M13**).
- Clamp `|Δ|` to `ADM_CTR_DEV_MAX` and `|Δ̇|` to `ADM_CTR_DEVR_MAX`.

### `AdmittanceControlModule.{hpp,cpp}`

- **Drop `RLS_EST_N_ROTORS` entirely** and switch the saturation source from `actuator_outputs` to
  **`actuator_motors`** (normalised `[0,1]`, unused slots already `NAN`):
  `sat = max over finite mᵢ of 2·|mᵢ − 0.5|`. This fixes **B2**, **M14**, **M15** and the rotor-count
  bookkeeping in one move, and works identically in SITL.
- `U = U.abs();` (**H5**) — assign the result.
- Add subscriptions: `vehicle_attitude`, `vehicle_local_position`, `vehicle_status`, `actuator_armed`,
  `rc_channels`. Drop `vehicle_attitude_setpoint`.
- Keep `debug_vect` **only** for `target_dist`, and check `strncmp(name, "ADM", 3) == 0` before accepting
  it (**H7**). Retain the 1 s timeout → `target_dist = 99`.
- Gate on `wrench.valid`, armed, `nav_state ∈ ADM_CTR_NAV`, `ADM_CTR_EN`, RC switch.
- `PX4_ISFINITE`-check every published field; on failure publish passthrough and `engaged = false`.
- Stamp the output with `hrt_absolute_time()`, not the input timestamp (**M16**).
- Publish `admittance_status` **every cycle regardless of engage state** — that is the bypass logging.
- Publish `admittance_setpoint` at the wrench rate, not only when `trajectory_setpoint` updates, so
  mc_pos_control's staleness check is meaningful.
- Re-arm a `ScheduleDelayed(50_ms)` watchdog at the end of `Run()` in addition to the wrench callback, so
  `stop` terminates (**H8**) and status still logs when `rls_wrench_estimator` is silent.
- Enrich `print_status()` with engage state, bypass reason, `sat_factor`, and current M/K.

### `admittance_control_params.c` — new params

| Param | Type | Default | Purpose |
|---|---|---|---|
| `ADM_CTR_EN` | int32 | 0 | 0 off / 1 bypass+log / 2 active |
| `ADM_CTR_RC_CH` | int32 | 0 | RC aux channel to force bypass; 0 = unused |
| `ADM_CTR_NAV` | int32 | 3 | nav-state bitmask (bit0 POSCTL, bit1 OFFBOARD, bit2 AUTO) |
| `ADM_CTR_REF_MAX` | float | 0.5 | m — clamp on virtual reference vs measured position |
| `ADM_CTR_DEV_MAX` | float | 1.0 | m — clamp on admittance deviation |
| `ADM_CTR_DEVR_MAX` | float | 1.0 | m/s — clamp on deviation rate |
| `ADM_CTR_RAMP` | float | 0.5 | s — engage/disengage ramp |

Existing `ADM_CTR_BEL_*` / `ADM_CTR_WRE_*` are unchanged.

### `../mc_pos_control/MulticopterPositionControl.{hpp,cpp}`

Replace the commented-out line at `.hpp:118` with two live subscriptions
(`_admittance_setpoint_sub{ORB_ID(admittance_setpoint)}`, `_admittance_status_sub{ORB_ID(admittance_status)}`)
and insert, immediately after `_trajectory_setpoint_sub.update(&_setpoint);` (`.cpp:451`) and **before**
`adjustSetpointForEKFResets`:

```cpp
// Admittance controller override: only when it declares itself engaged and fresh.
admittance_status_s adm_status;
if (_admittance_status_sub.copy(&adm_status)
    && adm_status.engaged
    && (hrt_elapsed_time(&adm_status.timestamp) < 100_ms)) {

    trajectory_setpoint_s adm_sp;
    if (_admittance_setpoint_sub.copy(&adm_sp)
        && (hrt_elapsed_time(&adm_sp.timestamp) < 100_ms)) {
        _setpoint = adm_sp;
    }
}
```

Keeping `_trajectory_setpoint_sub.update()` unconditional preserves the existing failsafe
(`.cpp:456-464`) and `_last_valid_setpoint` fallback (`.cpp:587-611`) untouched.

### Build + startup registration

- `boards/px4/fmu-v5/default.px4board`, `boards/px4/fmu-v6c/default.px4board`,
  `boards/px4/fmu-v6xrt/default.px4board` — add `CONFIG_MODULES_ADMITTANCE_CONTROL=y`, alphabetically
  first in the `CONFIG_MODULES_*` block. Non-`default` labels (`rover`, `lto`, `allyes`, …) merge on top
  of `default.px4board` and need no edit; `bootloader` never sees it. Note `allyes` on v6xrt
  force-enables everything, so the module must compile cleanly there too.
- `boards/px4/sitl/default.px4board` — same line, needed for the SITL verification below.
- `ROMFS/px4fmu_common/init.d/rc.mc_apps` — insert after the `rls_wrench_estimator` block (line 39) and
  before `flight_mode_manager start`:

  ```sh
  #
  # Start the admittance controller if enabled.
  #
  if param greater -s ADM_CTR_EN 0
  then
  	admittance_control start
  fi
  ```

  `greater 0` (not `compare 1`) so the bypass-and-log mode also starts.
- `src/modules/logger/logged_topics.cpp` — beside the existing fork-local entries (~line 155):
  ```cpp
  add_optional_topic("admittance_setpoint", 100);
  add_optional_topic("admittance_status", 100);
  ```
  `trajectory_setpoint` is already logged at line 131, so bypassed vs applied is directly comparable.
- `git add src/modules/admittance_control/` — the directory is currently untracked.

### Test

`AdmittanceControl/AdmittanceControlTest.cpp` — fix `EXPECT_EQ(0, cosf(-M_PI/4))` (line 200, always
false), drop the 500-iteration `printf` loop and the unused `vehicle_attitude` include/member, and update
to the deviation-form API. Add cases for: zero wrench → `Δ = 0`; step wrench → `Δ` settles at `We/K`;
NAN input position → finite output position within `ADM_CTR_REF_MAX` of the seed.

---

## Verification

1. **Compile all three targets** (the primary ask):
   ```
   make px4_fmu-v5_default && make px4_fmu-v6c_default && make px4_fmu-v6xrt_default
   ```
   Confirm each links and check the fmu-v5 flash headroom in the build summary.
2. **Unit tests:** `make tests TESTFILTER=AdmittanceControl`
3. **SITL end-to-end:** `make px4_sitl gz_x500`
   - `admittance_control status` → module present, engage state shown.
   - `param set RLS_EST_EN 1; param set ADM_CTR_EN 1` → bypass. `listener admittance_status` shows
     non-zero `deviation` under simulated contact while `engaged` stays false; confirm
     `listener trajectory_setpoint` and `listener admittance_setpoint` differ but flight is unaffected.
   - `param set ADM_CTR_EN 2` → `engaged` true, vehicle now responds. Toggle `ADM_CTR_RC_CH` and verify
     the switch forces bypass mid-flight.
   - Take off in Position mode, hold a stick deflection, confirm `admittance_setpoint.position[0..1]` is
     finite (not NAN) and stays within `ADM_CTR_REF_MAX` of `vehicle_local_position` — this is the
     velocity-only path.
   - `admittance_control stop` returns promptly (H8 regression check), both with and without
     `rls_wrench_estimator` running.
4. **Log review:** pull the ULog and plot `admittance_status.deviation` against `trajectory_setpoint` vs
   `admittance_setpoint` to confirm the bypass-logging path records what *would* have been commanded.
5. **Bench check before flight:** with `ADM_CTR_EN 2` and props off, verify `admittance_status.sat_factor`
   sits in `[0,1]` and never goes negative (the H5/M15 regression), and that `mass`/`stiffness` stay
   finite across the full wrench range.

## Risks to keep in view

- Emitting a finite position while the pilot commands velocity changes Position-mode feel. Authority is
  bounded by `MPC_XY_P × ADM_CTR_REF_MAX`; start flight testing with `ADM_CTR_REF_MAX = 0.2`.
- The deviation-form rewrite is algebraically equivalent but is a real change to the existing math —
  the unit tests above are the check on that.
- `admittance_control` chains off `rls_wrench_estimator` on the same `nav_and_controllers` work queue
  (2240-byte stack, priority −13). RK4 over 8 states adds stack pressure; watch `work_queue status`
  and the `perf` counter on hardware.
