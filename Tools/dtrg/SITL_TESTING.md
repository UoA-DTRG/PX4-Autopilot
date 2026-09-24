# DTRG automated tests

Automated tests of the DTRG features (horizontal thrust, bench test mode, RC
channel conflict check, CSV mixer, sequential desaturation) that run locally and
on GitHub Actions on every push and pull request to `dtrg-main`.

| Tier | Covers | Runs on | CI | Status |
|---|---|---|---|---|
| 1. Unit | Pure logic: desaturation order, CSV parser, bench test switch and profile, HT switch / aux tilt / mask | gtest, no simulator (`make tests TESTFILTER=Dtrg`) | job `unit` | done |
| 2. SIH logic | Arming and mode rules, bench test outputs, HT wiring from RC to setpoints | PX4 SITL + SIH on the planarOcto (`pytest test/dtrg -m "sih and not flight"`) | job `sih` | done |
| 3. SIH flight | HT behaviour in flight: take-off and hold, moving level with HT, tilt while holding position, HT toggling, bench test in the air | SIH with the fully actuated planarOcto (`pytest test/dtrg -m flight`) | job `flight` | done (section 6) |

Both tiers run on the planarOcto, the default airframe of the tests. Tier 2
asserts on decisions and setpoints, not on how the vehicle moves (it also
passes on `--airframe=sihsim_quadx`, which skips the `fully_actuated` tests).
Tier 3 flies the planarOcto: SIH builds the vehicle from
the same `CA_ROTOR*` geometry as control allocation (`SIH_VEHICLE_TYPE 4`), and
the tests assert on SIH's ground truth. The Gazebo model is not needed in CI
(keep it for manual checks of the model).

CI: [.github/workflows/dtrg_tests.yml](../../.github/workflows/dtrg_tests.yml).
Tier 2 how-to and gotchas: [test/dtrg/README.md](../../test/dtrg/README.md).

---

## 1. Review of the first version of this plan

The structure (fast deterministic tiers gating PRs, slow flight tests apart) held
up. Checking it against the code turned up these problems, now fixed in the
tests:

1. **Bench test is main mode 11, not 9.** `PX4_CUSTOM_MAIN_MODE_BENCH_TEST` is the
   11th entry of `px4_custom_mode.h`; 9 is `SIMPLE`. B1's `custom_mode()[0] != 9`
   would always pass and B4 tested the wrong mode. Three numbers are involved:
   HEARTBEAT main mode 11, `COM_FLTMODEx` value 16, nav state 16.
2. **DO_SET_MODE to bench test is ACKed as accepted.** Commander ignores an
   unknown custom main mode and still ACKs `ACCEPTED`. B4 must check the mode,
   not the ACK.
3. **RC override needs `RC_CHAN_CNT > 0`.** Without it rc_update never marks
   manual control valid, so the flight mode slots do nothing.
4. **STATUSTEXT only reaches a link that has seen a GCS HEARTBEAT**, and texts
   over 50 characters arrive in chunks. The helper sends a heartbeat and
   reassembles chunks.
5. **`Preflight Fail: ...` is printed when the failure set changes**, at most every
   2 s, not on each arm request. Take the "since" mark before causing the failure.
6. **D1 was wrong.** `DTRG_MIXER_CSV=1` does not set `DTRG_WINDUP_EN=0`; the
   rate controller skips anti-windup while the CSV mixer is on and the parameter
   keeps its value. D1 is dropped.
7. **RC deadzone.** `RCx_DZ` is 10 us on channels 1-8 and 0 on 9-18, so the HT
   switch (0.5) is at ~1755 us on channel 8, and half stick is 0.49, not 0.5.
8. **B1 does not need a flight.** The rule is "armed", so it is tested armed on the
   ground. The in-air variant belongs to tier 3.
9. **B6 cannot isolate its exception in SIH.** The simulated vehicle stays landed,
   so disarming is allowed anyway. It is kept as a sanity check; the real check
   needs a vehicle that reports "in air" on a bench (tier 3 or a rig).
10. **`make tests` did not build on the fork.** `unit-ControlAllocationPseudoInverse`
    failed to link once the DTRG CSV parameters made the allocator depend on the
    parameter system (now a functional test), and
    `AirmodeDisabledReducedThrustAndYaw` still expected the upstream order (thrust
    given up for yaw). It now expects the DTRG order (yaw given up first).
11. **A6's telemetry check could not have worked**, see gap G1 below.
12. **SIH is no longer "quad only"**: the airframe is selectable
    (`test/dtrg/airframes.py`), and tests needing full actuation are marked
    `fully_actuated` and skipped on the quad.

---

## 2. Tier 1: unit tests

Run: `make tests TESTFILTER=Dtrg` (and `TESTFILTER=ControlAllocation` for the
upstream allocator tests). `TESTFILTER` is a ctest regex, but `|` breaks the
Makefile's shell quoting, so run one filter at a time.

What each test checks: [TESTS.md](TESTS.md).

| Test binary | File | Covers |
|---|---|---|
| `unit-DtrgBenchSwitch` | `src/modules/bench_test/DtrgBenchSwitchTest.cpp` | direction switch thresholds (1300/1700 us, exclusive), unpopulated channel, RC loss, out of range channel |
| `unit-DtrgBenchProfile` | `src/modules/bench_test/DtrgBenchProfileTest.cpp` | step timing and sign, ramp clamp and freeze on motor saturation, spin-up, axis mapping, saturation margin, reversible motors |
| `functional-DtrgSequentialDesaturation` | `src/lib/control_allocation/control_allocation/DtrgSequentialDesaturationTest.cpp` | on a planar octo with +-31 deg tangential tilt: X and Y given up before roll and yaw, yaw before thrust, thrust before roll, thrust never increased |
| `functional-DtrgMixerCsv` | `src/lib/control_allocation/control_allocation/DtrgMixerCsvTest.cpp` | CSV parser: full matrix, BOM, CRLF, blank lines, missing file, extra rows and columns |

To make this testable, the bench test profile maths was moved into the
header-only `bench_test_profile.h` (used by `BenchTest.cpp`, no behaviour
change), and `readMixerFromCSV` is now a public static. The horizontal thrust
helper and its unit tests are on branch `salz167/DTRG_HT_refactor`; until that
is merged, HT is covered by the SIH tests only.

Tests named `DISABLED_*` describe known gaps (section 4). They build but do not
run; run them with `--gtest_also_run_disabled_tests` on the test binary in
`build/px4_sitl_test/`, and remove the prefix once fixed.

---

## 3. Tier 2: SIH logic tests

Every test boots a fresh PX4 with a clean rootfs, sets its parameters at boot
(`PX4_PARAM_*`), optionally streams RC_CHANNELS_OVERRIDE at 50 Hz using one fixed
RC layout (`test/dtrg/rc_layout.py`), and shuts PX4 down again. A test takes
10-25 s. On failure CI uploads each test's `px4.log` and ULogs.

| Plan ID | Test | Checks |
|---|---|---|
| - | `test_smoke.py` | boots, DTRG marker `SYS_STATUS.errors_count4 == 706`, RC override drives the mode slot |
| B1 | `test_bench_test_rejected_while_armed` | "Bench test mode denied: disarm first", mode and arming unchanged; then reachable once disarmed |
| B2 | `test_arming_needs_bt_arm_enable` | "Arming denied: bench test not enabled", arms once enabled |
| B3 | `test_arming_needs_centred_direction_switch[up/down]` | "centre the bench test direction switch", arms once centred |
| B3b | `test_off_centre_switch_allowed_when_it_cannot_excite` | hover-only profile, or no switch assigned |
| B4 | `test_mavlink_cannot_select_bench_test` | DO_SET_MODE main mode 11 leaves the mode unchanged |
| B5 | `test_bench_test_outputs.py` | from the ULog: spin-up ramp, step size/sign/length, ramp clamp and hold, only the selected axis, zero while disarmed |
| B6 | `test_disarm_allowed_in_bench_test` | disarm honoured (see review item 9) |
| - | `test_rc_slot_selects_bench_test_while_disarmed` | control case for B1/B4 |
| C1 | `test_rc_conflict.py` | runtime and boot-time conflict blocks arming and names both parameters; resolving re-allows; `COM_ARM_RC_CONF=1` warns only; `RC_MAP_FAILSAFE` exempt; `RC_MAP_FLTM_BTN` only counts without `RC_MAP_FLTMODE` |
| HT | `test_horizontal_thrust.py` | Stabilized: switch off = normal; on = sticks to X/Y thrust at `DTRG_HT_MAX` with level attitude; runtime toggle; `DTRG_HT_EN=0`; aux tilt to `DTRG_HT_R/P_MAX`; aux deadzone; thrust axes per mask (mask 3 xfail, G6) |
| D2 | `test_csv_mixer_without_file_refuses_to_arm` | strict xfail, gap G4 |

---

## 4. Known gaps found while writing the tests

Each one is pinned by a test that turns red (xfail strict) or can be enabled
(`DISABLED_`) once it is fixed.

| | Gap | Tracked by |
|---|---|---|
| G1 | `sequential_desaturation.*_sat` reports only the gain of the second, half-strength desaturation pass, which is ~0 whenever the first pass succeeds (X cut from 5.0 reports 1.7e-8). `SYS_STATUS.errors_count1` then tests `> 0.01`, which also drops every negative gain | `DISABLED_TopicReportsHorizontalThrustReduction` |
| G2 | The X, Y and yaw desaturation steps slide along their axis to relieve *any* saturation, not only to shrink their own demand: a pure roll demand near full thrust comes out with unrequested X thrust (-0.02) and yaw (-0.12) | `DISABLED_DesaturationDoesNotAddUnrequestedAxes` |
| G3 | In Stabilized, HT X/Y is `stick * DTRG_HT_MAX`, which RC scaling keeps just below the limit (0.49999994), so `horizontal_thrust_limit.x_sat` and `errors_count3` never set | xfail `test_full_stick_reports_ht_saturation` |
| G4 | `DTRG_MIXER_CSV=1` without a file leaves an all-zero mixer and nothing stops arming. The path `/fs/microsd/etc/mixer.csv` is hardcoded (the parameter description says `/etc/mixer.csv`) and does not exist in SITL | xfail `test_csv_mixer_without_file_refuses_to_arm` |
| G5 | CSV parser: a full-precision row (~20 chars per cell) overflows the 100 byte line buffer and is split into two rows; an empty cell shifts the row left (`strtok`); an empty file, a short row and a blank CRLF line are accepted | `DISABLED_*` in `DtrgMixerCsvTest.cpp` |
| G6 | `DTRG_HT_MASK` docs say 0 = disabled, 1 = roll only, ...; and mask 3 applies horizontal thrust on top of tilting. Intended: 0 HT on X and Y, 1 HT on X / roll for Y, 2 HT on Y / pitch for X, 3 no HT, pitch and roll only. Fixed on branch `salz167/DTRG_HT_refactor`, which also moves the HT logic of mc_att_control and mc_pos_control into a shared, unit tested helper | xfail `test_mask_selects_thrust_axes[mask3]` |
| G7 | In Stabilized, mask 3 swaps the sticks: the pitch stick commands roll and the roll stick pitch (marked "swapped" in the code, so possibly intended). Position control's mask 3 does not swap. Masks 1-3 update a tilt filter a second time in the same cycle, with a different input | not covered |
| G8 | Offboard HT roll/pitch from `DEBUG_FLOAT_ARRAY` are neither limited to `DTRG_HT_R/P_MAX` nor checked for NaN; any MAVLink source can command any tilt | not covered, needs an armed Offboard test |
| G9 | `BT_SAT_MARGIN` allows 0.5, at which any standard motor output reads as saturated and the ramp freezes at 0 | documented in `DtrgBenchProfileTest.cpp` |
| G10 | `DTRG_OFFBOARD` (MAVLink 9003) is received into `dtrg_custom` but nothing reads it; `streams/DTRG_OFFBOARD.hpp` does not compile and is not registered | not covered |
| G11 | HT delivers 41% of the horizontal force the position controller asks for on the planarOcto. mc_pos_control writes its NED thrust (normalised to full collective thrust) into `thrust_body[0/1]`, but the allocator normalises each thrust axis on its own (`ControlAllocationPseudoInverse::updateControlAllocationMatrixScale`), and on this geometry X/Y = 1 is 0.41 of full thrust while Z = 1 is all of it. In a hover with an HT tilt the position controller cannot hold position: at 10 deg roll it needs 0.25 body Y thrust for a physical 0.10, which `MPC_TILTMAX_AIR 10` caps below, so the vehicle slides ~0.5 m/s (and Hold then yaws towards its setpoint). With `MPC_TILTMAX_AIR 30` it holds, 0.55 m off. Moving level (A2) works because the integrators absorb the gain loss. Stabilized HT (`stick * DTRG_HT_MAX`) is scaled the same way | strict xfail `test_a4_aux_tilt_in_hover_holds_position` |
| G13 | The HT pitch aux channel (`RC_MAP_HT_PITCH`) tilts the vehicle the opposite way depending on the mode: Stabilized negates it (`mc_att_control_main.cpp`, nose down for aux up, like the pitch stick), Position/Hold/Offboard do not (`MulticopterPositionControl.cpp`, nose up). Which sign is intended needs deciding; the test expects the Stabilized one | strict xfail `test_a4b_aux_tilt_reaches_the_limit[pitch]` |
| G14 | Observation, not a DTRG bug: while holding a tilt with horizontal thrust, EKF2's roll/pitch estimate drifts ~2 deg from the truth and its accel bias grows (Y bias 0.01 -> 0.08 m/s^2 in 7 s at 5.7 deg roll). With HT the specific force is no longer along body Z, so a tilt error and a horizontal accel bias are hard to tell apart; a real vehicle's estimator faces the same. The controller tracks its estimate; the tests allow for this (estimate +-2 deg, truth +-3.5 deg) | A4b, A5 tolerances |
| G12 | Fixed. PX4 SITL: after arming, all interval-scheduled work items could stop (sensor_baro/mag/gps_sim, battery, cpuload), leading to "No valid data from Baro 0" and a blind-land failsafe; 3 of 4 take-offs on the stock SIH quad. `hrt_call_invoke()` runs a callout unlocked, and GyroCalibration / MagBiasEstimator call `ScheduleOnInterval()` on arming from their own thread; the periodic entry was then queued twice, unlinking every call between the two positions. `platforms/posix/src/px4/common/drv_hrt.cpp` now skips re-entering an entry that is already queued. NuttX is not affected (callouts run in the ISR). Tier 2 never flies, so it never hit this | every flight test (0 of 8 take-offs stall after the fix) |

---

## 5. CI

`.github/workflows/dtrg_tests.yml` runs on every push to and pull request into
`dtrg-main`, plus manually:

- `unit`: `make tests TESTFILTER=Dtrg`, `TESTFILTER=ControlAllocation`, then the
  full `make tests` as a non-blocking step until it has been seen green on this fork.
- `sih`: builds `px4_sitl_default` and runs `pytest test/dtrg -m "sih and not flight"`
  on `sihsim_planar_octo`, including the tests marked `fully_actuated`.
- `flight`: builds `px4_sitl_default` and runs `pytest test/dtrg -m flight` on
  `sihsim_planar_octo`.

Make `Unit tests (tier 1)`, `SIH logic tests (tier 2)` and, once it has been
green for a while, `SIH flight tests (tier 3)` required checks in the branch
protection of `dtrg-main`. To also test pushes to feature branches before
a PR exists, widen `on.push.branches`.

About 3% of SITL boots hang on macOS: a `px4-<module>` client command in rcS
never gets its reply from the PX4 server (seen in 2 of 60 boots with and without
the G12 fix). The `sitl` fixture detects it and boots again (up to twice).

Locally, `make tests` fails `sitl-*` tests if another SITL is running (instance 0
is taken), and on macOS `PurePursuit` and `gps_blending` fail on float rounding;
neither involves DTRG code.

---

## 6. Tier 3: flight tests on the fully actuated planarOcto

Run: `python3 -m pytest test/dtrg -m flight -v`
(about 10 flights, ~10 minutes). Tests are in `test/dtrg/test_flight_ht.py`,
helpers in `test/dtrg/flight.py`.

### The simulated vehicle

`SIH_VEHICLE_TYPE 4` (generic multirotor) reads `CA_ROTOR_COUNT` and
`CA_ROTORn_PX/PY/PZ/AX/AY/AZ/CT/KM`, the parameters control allocation uses, and
sums per rotor `F = T * axis` and `M = r x F - KM * T * axis`, with
`T = SIH_T_MAX * (SIH_THR_MDL_FAC * u^2 + (1 - SIH_THR_MDL_FAC) * u)`. Any
multirotor that control allocation can describe flies without code changes.
Output n drives rotor n, so map Motor 1..N to outputs 1..N in order.

| Airframe | Where | |
|---|---|---|
| `12016_sihsim_planar_octo` | SITL (`make px4_sitl sihsim_planar_octo`) | CI and local runs |
| `12017_dtrg_planar_octo_sih.hil` | the real flight controller (`SYS_HITL 2`) | the planarOcto flight stack flying on the actual hardware, against a vehicle simulated on the board; real outputs stay off (remove the props anyway) |

Mass, inertia and propulsion are the Gazebo model's values (assumed / FT-X8,
not measured), documented in `12016_sihsim_planar_octo`. With them the
planarOcto hovers at 0.56 thrust with all eight motors between 0.49 and 0.65:
no saturation, so the saturation seen in Gazebo comes from that model, not from
the geometry or allocation.

### Tests

| ID | Test | Pass criteria (ground truth) | Status |
|---|---|---|---|
| A1 | `test_a1_take_off_hold_and_land` | altitude within 0.5 m of its setpoint over a 10 s hold, drift < 1 m; lands and disarms within 30 s | pass |
| A2 | `test_a2_ht_moves_the_vehicle_level` | HT on, Offboard 5 m north: moves 5 +-0.5 m, `max(|roll|, |pitch|) < 3 deg` throughout | pass |
| A3 | `test_a3_without_ht_the_vehicle_tilts_to_move` | control case for A2: pitch < -5 deg | pass |
| A4 | `test_a4_aux_tilt_in_hover_holds_position[roll/pitch]` | HT on, Hold, aux channel full: tilt of `DTRG_HT_R/P_MAX` +-2 deg, drift < 0.5 m | strict xfail, G11 |
| A4b | `test_a4b_aux_tilt_reaches_the_limit[roll/pitch]` | the attitude half of A4: aux up gives roll `+DTRG_HT_R_MAX` / pitch `-DTRG_HT_P_MAX` (as in Stabilized) +-2 deg, other axis < 3 deg | roll pass; pitch strict xfail, G13 |
| A5 | `test_a5_offboard_tilt_setpoint_in_hover` | HT on, Offboard hold, `DEBUG_FLOAT_ARRAY` roll 0.1 rad (5.7 deg): estimated roll +-2 deg, true roll +-3.5 deg (G14), drift < 0.5 m | pass |
| A5b | `test_a5_offboard_tilt_is_limited` | Offboard tilt of 2 x `DTRG_HT_R_MAX` stays within the limit | strict xfail, G8 |
| A6 | - | `DTRG_HT_MAX=0.1`, 10 m move, allocated X/Y cut before roll/pitch | not written: needs the allocator's per-axis output in the log (see G1) |
| A7 | `test_a7_toggling_ht_in_hover_is_smooth` | toggle HT 5x in Hold: altitude within 0.7 m of its setpoint, tilt < max(8 deg, plain hover + 2 deg), drift < 1 m | pass |
| A8 | `test_a8_stabilized_ht_stick_moves_the_vehicle_level` | Stabilized, HT on, full pitch stick 3 s: forward speed > 1 m/s, tilt < 3 deg | pass |
| B1b | `test_b1b_bench_test_rejected_in_flight` | "Bench test mode denied: disarm first", still armed, in Hold and at altitude | pass |

Holding position and altitude is judged on the estimate against the setpoint;
tilt and distance moved on the ground truth. With SIH's simulated baro and GPS
the estimate wanders 0.3-0.5 m (altitude) and up to ~1 m (horizontal) from the
truth, which is estimator error, not the behaviour under test. The "drift"
criteria in the table are that estimate-to-setpoint distance.

The tolerances are first guesses; tighten them after ~20 green CI runs. In a
plain hover the vehicle tilts ~3 deg (std) and up to ~9 deg: that is the
position controller correcting the simulated GPS/IMU noise with HT off.

---

## 7. Reference: facts the tests rely on

Re-check these if tests start failing for no obvious reason.

1. HT and bench test read `rc_channels` / `input_rc`, which in SITL only
   `RC_CHANNELS_OVERRIDE` feeds (`MANUAL_CONTROL` does not reach them).
2. SITL defaults to `COM_RC_IN_MODE 1` (joystick). RC tests set `0` and
   `RC_CHAN_CNT 18`, and stream RC for the whole test.
3. Bench test is only reachable from an RC slot (`COM_FLTMODEx = 16`), and a slot
   is only acted on when it changes (or first seen while disarmed).
4. HT is on when the `RC_MAP_HT_MODE` channel is `> 0.5`. The bench test
   direction switch is `>1700` up, `<1300` down (raw us).
5. Channels 9-18 need MAVLink 2 (`MAVLINK20=1` before importing pymavlink).
6. Status texts to assert on (match by substring, they end in `\t`):
   `Bench test mode denied: disarm first`,
   `Arming denied: bench test not enabled, set BT_ARM_ENABLE=1`,
   `Arming denied: centre the bench test direction switch`,
   `Preflight Fail: <A> and <B> both use RC channel <N>`,
   `RC ch <N>: <A> and <B>`, `RC channel conflict resolved`.
7. DTRG telemetry in `SYS_STATUS`: `errors_count1` desaturation gains (see G1),
   `errors_count2` motors > 0.9, `errors_count3` HT `x_sat` bit 0 / `y_sat` bit 2
   (see G3), `errors_count4 == 706`.
8. `SYS_STATUS.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK` is
   "can arm in the current mode"; the tests use it as the readiness signal.
9. Parameters set through `PX4_PARAM_<NAME>` are applied before the airframe
   script. The airframe's `param set-default` does not override them, except
   when the value set equals the firmware default: that does not count as a
   change, so the airframe's default would win (e.g. `DTRG_HT_EN=0` on the
   planarOcto, whose airframe defaults it to 1). `init.d-posix/rcS` therefore
   applies `PX4_PARAM_*` a second time after the airframe script, and the `sitl`
   fixture fails a test whose parameters did not boot with the requested values.
