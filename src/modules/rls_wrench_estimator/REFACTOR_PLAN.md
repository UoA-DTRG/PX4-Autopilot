# RLS Wrench Estimator — Generalization Plan

## Context

`src/modules/rls_wrench_estimator/` is a new (untracked) module by Pedro Mendes that runs a Recursive Least Squares (RLS) identification of rotor thrust/CoM parameters plus a momentum-based external wrench (force/torque) observer. It is currently **hardcoded for one specific airframe** — a coaxial tilt-rotor X8 octocopter (with a quad fallback) — and has two structural problems that block reuse:

1. **It does not build today.** It publishes `ORB_ID(rls_wrench_estimator)` and includes `<uORB/topics/rls_wrench_estimator.h>`, but **no `.msg` file exists** anywhere in the repo, and there is **no `Kconfig`**. These must be created before anything compiles.
2. **Everything is hardcoded to the X8 geometry**: fixed `Vector<float,8>` sizing, hardcoded per-motor arrays for tilt axis (`T[8]`), spin (`R[8]`), azimuth (`A[8]`, π/4 spacing), motor heights (`H[8]`), equal arm length (`diameter/2`), coaxial top/bottom grouping and wake cross-map (`Fr[8]`), and a **single shared** `k_f`/`k_m` for all propellers. Motor speed is derived only from a PWM→speed affine model passed through a first-order LPF.

**Goal:** make the module airframe-agnostic (per-rotor geometry, grouped thrust coefficients), let it consume measured ESC RPM when available, and build+start it on `fmu-v5`, `fmu-v6c`, `fmu-v6xrt`.

**Confirmed design decisions:**
- Geometry comes from the **control allocator's `CA_ROTOR*` params** (single source of truth).
- Thrust coefficients are **grouped**: a param sets the number of groups; each rotor is assigned to a group; the RLS estimates one `k_f` per group. This **replaces the coaxial-reduction (`k_r`) model** — a bottom/coaxial prop simply goes in its own group whose `k_f` absorbs the wake loss.
- ESC speed: a param selects source, with **auto-fallback** to the PWM model per-motor when `esc_rpm` is unavailable.
- Startup: **param-gated** start line in `rc.mc_apps`, built into all three boards.

**Observability note (informs the group design):** the thrust RLS derives its measurement from a single 3-axis acceleration, so at most ~3 independent thrust unknowns are observable. Groups let the user trade generality against observability (`N_GRP=1` = current behavior; more groups need geometric excitation to converge). This is the user's call at param-set time; the code just supports it.

**Note on dropping `k_r`:** the original coaxial term multiplied each bottom rotor's reduction by the *upstream* rotor's speed² (`w_upstream²`). The group approach instead scales each rotor by its *own* speed² with a group-specific `k_f`. This is a small physical approximation (exact only when paired rotors spin at similar speeds) but is the clean, observable, airframe-agnostic choice and is what the new message encodes.

---

## Task 0 — Make the module buildable (prerequisite)

### 0.1 Create the uORB message `msg/RlsWrenchEstimator.msg`
Fields (fixed-size arrays, sized for the group max):
```
uint64 timestamp        # time since system start (microseconds)
float32[3] fe           # external force  (body frame)
float32[3] me           # external moment (body frame)
float32[3] fi           # actuator force vector
float32[3] mi           # actuator moment vector
float32[16] k_f         # estimated thrust of each motor (unused groups = NAN)
float32[3] x_offset     # estimated CoM offset [x, y, z]
uint8 n_groups          # number of active thrust groups
uint8[16] motor_group   # group number for each motor
bool interaction_flag
bool valid
```
Add `RlsWrenchEstimator.msg` to the list in `msg/CMakeLists.txt` (alphabetical block). The generated topic/type stay `rls_wrench_estimator` / `rls_wrench_estimator_s` — matching the existing includes. This replaces the old `x_thrust[2]` field with per-motor `k_f[16]` + `motor_group[16]` + `n_groups`, and **drops the coaxial `k_r` term entirely** (see Task 1.4). Internally the RLS still estimates one coefficient per group; `publishStatus()` expands it to each motor via its group index (`k_f[i] = k_f_of_group(motor_group[i])`), and fills unused motor slots (`i ≥ num_rotors`) with NAN. No external consumer exists (verified), so no downstream breakage.

### 0.2 Create `src/modules/rls_wrench_estimator/Kconfig`
Mirror `src/modules/mc_hover_thrust_estimator/Kconfig`:
```
menuconfig MODULES_RLS_WRENCH_ESTIMATOR
	bool "rls_wrench_estimator"
	default n
	---help---
		Enable support for rls_wrench_estimator
```
(`src/modules/Kconfig` already globs `*/Kconfig`, so no central registration is needed.)

---

## Task 1 — Generalize geometry + grouped coefficients (`RLSIdentification`)

**Key idea:** replace the hardcoded `T/R/A/H/Fr/diameter/tilt` arrays with a per-rotor geometry table read from the control-allocator params, and expand the single `k_f` into one `k_f` per group.

### 1.1 Geometry source — read `CA_ROTOR*` params
Reference implementation: `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.cpp:48-125` (constructor `param_find` loop + `updateParams()`), struct `RotorGeometry{position, axis, thrust_coef, moment_ratio, tilt_index}` at `ActuatorEffectivenessRotors.hpp:64-70`.

In `RLSWrenchEstimator` (the module class, which is a `ModuleParams`), read the same params directly with `param_find`/`param_get` into a lightweight local geometry array — do **not** pull in the whole `control_allocation` lib. Read for `i` in `0..CA_ROTOR_COUNT-1`:
- position: `CA_ROTOR{i}_PX/PY/PZ` → `Vector3f position`
- axis: `CA_ROTOR{i}_AX/AY/AZ` → `Vector3f axis` (normalized; encodes tilt direction, so `RLS_EST_TILT` is no longer needed)
- thrust coef: `CA_ROTOR{i}_CT` → fixed per-rotor relative thrust scale
- moment ratio: `CA_ROTOR{i}_KM` → its **sign** gives spin direction (replaces `R[8]`), magnitude gives yaw-torque/thrust ratio (replaces `_km`)

Set `MAX_ROTORS = 12` to match the control allocator's `NUM_ROTORS_MAX` (it only defines `CA_ROTOR0..11`); geometry reads use `min(CA_ROTOR_COUNT, 12)`. The message `k_f[16]`/`motor_group[16]` arrays stay at 16 as headroom — only slots `0..num_rotors-1` are filled, the rest NAN. `esc_status` reports only 8 ESCs, so the ESC-RPM path (Task 2) covers motors 0–7; motors 8–11 always use the PWM model. Pass this geometry into `RLSIdentification::initialize()` via an expanded config struct (replace the current `VehicleParameters` 8-scalar struct with one carrying the geometry array + group map + counts).

### 1.2 Grouped thrust coefficients — new params
- `RLS_EST_N_GRP` (int, 1..12, default 1) — number of thrust-coefficient groups.
- `RLS_ROTOR{i}_GRP` for `i=0..11` (int, default 0) — group index each rotor belongs to. Put coaxial/bottom props in their own group so their `k_f` absorbs the wake reduction (replaces the old `k_r` model).
- Keep `RLS_EST_KF_INIT`/`RLS_EST_KF_CONF` as a **single shared** initial guess/confidence applied to every group (per-group init can be a later refinement).
- **Remove** `RLS_EST_KR_INIT`/`RLS_EST_KR_CONF` — the coaxial reduction coefficient is gone.
- **Remove** the now-redundant geometry params from `rls_wrench_estimator_params.c`: `RLS_EST_TILT`, `RLS_EST_D`, `RLS_EST_TOP_H`, `RLS_EST_BOT_H`, `RLS_EST_N_ROTORS`, `RLS_EST_KM` (superseded entirely by `CA_ROTOR*` — no override fallback). Mass, inertia, noise, tau, offset, ESC-speed, and battery params stay.

### 1.3 Rework the thrust regressor `_createHThrust` (`RLSIdentification.cpp:145-166`)
- Thrust state becomes `x = [k_f_group0 … k_f_group(G-1)]` (no `k_r` column). Use a **fixed max-size** state (`MAX_GROUPS = 12`) so the `matrix` fixed-size templates still work; zero out the H columns for unused groups (`g ≥ N_GRP`) so they never update (the `inv(...)` in `_computeKThrust` acts on the always-`3×3` measurement covariance `Q = H·P·Hᵀ + R`, so it stays well-conditioned regardless of state dim).
- Build H column-by-column: for each rotor `r`, its body-frame force contribution is `CT_r · axis_r · w_r²`; accumulate it into the column of `r`'s group (`motor_group[r]`).

### 1.4 Coaxial reduction — subsumed by groups (no separate `k_r`)
The old `Fr[8]` cross-map and per-rotor upstream mapping are dropped. A coaxial/bottom prop instead gets its own group; that group's `k_f`, scaling the rotor's own `w²`, captures its reduced effectiveness. This removes the need for `RLS_ROTOR{i}_UP`, `RLS_EST_KR_*`, and the top/bottom-height geometry. (See the "Note on dropping `k_r`" in Context for the approximation this trades.)

### 1.5 Rework the moment model `_createMomentVector` (`RLSIdentification.cpp:243-312`)
Drop the hardcoded `T/R/A/H/Fr` arrays and the quad remap. Loop over `0..num_rotors-1` using the geometry table:
- per-rotor force `Fi = CT_r · k_f[group_r] · axis_r · w_r²` (no `k_r` reduction term),
- per-rotor torque `Qi += position_r × Fi + axis_r · (−KM_r · k_f[group_r] · w_r²)` (spin sign carried by `sign(KM_r)`; the drag-torque/thrust ratio by `|KM_r|`).

The offset RLS (`_createHOffset`, `updateOffset`) is geometry-independent (uses attitude + mass·g) and stays as-is.

### 1.6 Update the identification API + tests
- Expand `VehicleParameters` → a geometry/group config struct; update `initialize()` and `updateThrust()` signatures (state/output vectors sized to `MAX_ROTORS`/`MAX_GROUPS = 12`).
- Update `RLSIdentification/RLSIdentificationTest.cpp` (constructs `VehicleParameters{...}` with 8 scalars at line ~40) and `WrenchEstimator/WrenchEstimatorTest.cpp` if signatures shift.

---

## Task 2 — ESC speed source with auto-fallback (`RLSWrenchEstimator`)

### 2.1 New param
`RLS_EST_SPD_SRC` (int, default 0): `0` = PWM-affine model (current), `1` = ESC RPM with per-motor fallback.

### 2.2 Subscribe & consume `esc_status`
- Add `#include <uORB/topics/esc_status.h>` and `uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};` (`RLSWrenchEstimator.hpp:110-121`).
- Idiom to follow: `VehicleAngularVelocity.cpp:574-627` — loop `math::min(esc_status.esc_count, MAX_NUM_ESCS)`, read `esc_status.esc[i].esc_rpm`, treat a motor as valid when online (`esc_online_flags`) and `esc_rpm != 0`, and check freshness against `esc_status.timestamp`.
- Replace the fixed `float speed[8]` block (`RLSWrenchEstimator.cpp:206-217`) with a per-motor build over `0..num_rotors-1` (up to `MAX_ROTORS = 12`): in ESC mode, if rotor `i`'s ESC report is valid → `speed_i = |esc_rpm_i| · (2π/60)` (rad/s); else **fall back** to the PWM model for that motor. In PWM mode, always use the PWM model. Map ESC index → rotor index by array position (consistent with the existing `actuator_outputs.output[i]` assumption); optionally refine via `esc[i].actuator_function`.
- **ESC coverage caps at 8**: `esc_status` only carries 8 reports, so motors 8–11 can never source RPM and always use the PWM model — the per-motor fallback handles this automatically.
- **All speeds are rad/s.** The PWM-affine model must therefore be calibrated so its output is rad/s (i.e. `RLS_EST_SPE_P1/P2` produce rad/s), matching the ESC-RPM path. This keeps `k_f` in a single consistent unit regardless of source.

### 2.3 Bypass the motor LPF for measured speed
The first-order LPF (`_updateLpf`, `RLSIdentification.cpp:137-143`, `tau = RLS_EST_LPF_M`) models command→actual-speed dynamics. Measured `esc_rpm` **is** the actual speed, so ESC-mode should skip the LPF (pass a flag into `updateThrust`, or filter only PWM-derived speeds). PWM mode keeps the LPF unchanged.

### 2.4 Document the unit/scale change
Both sources now yield **rad/s**, so `k_f` has one consistent unit and does not change meaning between sources (a correctly-calibrated PWM model and measured ESC RPM agree). Because the speed unit changed from the old arbitrary PWM "speed" to rad/s, re-check `_kf_multiplier` (currently `1e-6`) and `RLS_EST_KF_INIT` against the new `w²` magnitude (rad/s squared ~ `1e4–1e5`). Note the rad/s convention in the `RLS_EST_SPE_*` and `RLS_EST_KF_INIT` param docs.

---

## Task 3 — Build & start on fmu-v5 / v6c / v6xrt

### 3.1 Enable the build (needs Task 0.2 Kconfig)
Add `CONFIG_MODULES_RLS_WRENCH_ESTIMATOR=y` to each board's `default.px4board` (in the `CONFIG_MODULES_*` block, near the other estimators — e.g. after `CONFIG_MODULES_MC_HOVER_THRUST_ESTIMATOR=y`):
- `boards/px4/fmu-v5/default.px4board`
- `boards/px4/fmu-v6c/default.px4board`
- `boards/px4/fmu-v6xrt/default.px4board`

### 3.2 Param-gated startup
- New param `RLS_EST_EN` (int, 0/1, default 0) in `rls_wrench_estimator_params.c`.
- Add to `ROMFS/px4fmu_common/init.d/rc.mc_apps` (near `mc_hover_thrust_estimator start`, line 31), following the `mag_bias_estimator` gate idiom (`rcS:473-476`):
```sh
if param compare -s RLS_EST_EN 1
then
	rls_wrench_estimator start
fi
```
If `CONFIG_MODULES_RLS_WRENCH_ESTIMATOR` is not set the command simply won't exist and the line no-ops — so both 3.1 and 3.2 are required.

---

## Files to create / modify

**Create:**
- `msg/RlsWrenchEstimator.msg`
- `src/modules/rls_wrench_estimator/Kconfig`

**Modify:**
- `msg/CMakeLists.txt` — register the new message
- `src/modules/rls_wrench_estimator/rls_wrench_estimator_params.c` — add `RLS_EST_N_GRP`, `RLS_ROTOR{i}_GRP`, `RLS_EST_SPD_SRC`, `RLS_EST_EN`; remove `RLS_EST_TILT/D/TOP_H/BOT_H/N_ROTORS/KM` and `RLS_EST_KR_INIT/KR_CONF`
- `src/modules/rls_wrench_estimator/RLSWrenchEstimator.hpp/.cpp` — read `CA_ROTOR*` geometry, per-rotor group map, `esc_status` sub + speed build, updated `DEFINE_PARAMETERS`, publish new msg fields (`k_f[16]`/`motor_group[16]`/`n_groups`, no `k_r`)
- `src/modules/rls_wrench_estimator/RLSIdentification/RLSIdentification.hpp/.cpp` — grouped/geometry-driven `H_thrust`, moment model, expanded state, new config struct
- `src/modules/rls_wrench_estimator/RLSIdentification/RLSIdentificationTest.cpp` (+ `WrenchEstimator/WrenchEstimatorTest.cpp` if API shifts)
- `boards/px4/fmu-v5|fmu-v6c|fmu-v6xrt/default.px4board`
- `ROMFS/px4fmu_common/init.d/rc.mc_apps`

---

## Verification

1. **Builds:** `make px4_fmu-v5_default`, `make px4_fmu-v6c_default`, `make px4_fmu-v6xrt_default` — all link the module.
2. **Unit tests:** `make tests` (runs the two gtests) after updating `RLSIdentificationTest.cpp`; confirm `N_GRP=1` + single-group reproduces prior thrust identification on the X8 fixture.
3. **SITL smoke test:** `make px4_sitl gz_x500`; set `RLS_EST_EN 1`, configure `CA_ROTOR*` + `RLS_ROTOR*_GRP`, arm, take off, then `listener rls_wrench_estimator` — confirm `k_f`/`motor_group`/`n_groups`/`fe`/`me` populate and `valid` latches.
4. **ESC-RPM path:** with `RLS_EST_SPD_SRC 1`, verify speeds track `listener esc_status` when RPM is reported and that motors with `esc_rpm == 0` fall back to the PWM model (temporarily force one to 0 and confirm no NaN/estimation blow-up).
5. **Regression:** confirm `RLS_EST_SPD_SRC 0` + `N_GRP 1` matches pre-refactor behavior on the coaxial X8 params.

---

## Resolved decisions

All prior open questions are settled and folded into the tasks above:

1. **`k_r` (coaxial reduction):** dropped — groups subsume it (own-`w²` per group).
2. **Redundant geometry params** (`RLS_EST_TILT/D/TOP_H/BOT_H/N_ROTORS/KM`): removed entirely, no override fallback — `CA_ROTOR*` is the sole geometry source.
3. **Speed units:** everything in **rad/s** — ESC RPM converted via `·2π/60`, and the PWM model calibrated to output rad/s.
4. **`k_f` init/confidence:** a single shared `RLS_EST_KF_INIT`/`RLS_EST_KF_CONF` applied to all groups.
5. **`MAX_ROTORS = 12`** (the `CA` max). ESC RPM covers motors 0–7; motors 8–11 fall back to the PWM model. Message arrays stay `[16]` as headroom.
