# DTRG test list

What each DTRG test checks and why it matters. The plan, CI and known gaps
are in [SITL_TESTING.md](SITL_TESTING.md).

- [Tier 1: unit tests](#tier-1-unit-tests)
  - [DtrgSequentialDesaturation](#dtrgsequentialdesaturation): desaturation order
  - [DtrgMixerCsv](#dtrgmixercsv): CSV mixer parser
  - [DtrgBenchSwitch](#dtrgbenchswitch): bench test direction switch
  - [DtrgBenchProfile](#dtrgbenchprofile): bench test excitation profile
- [Tier 2: SIH logic tests](#tier-2-sih-logic-tests)
  - [test_smoke.py](#test_smokepy): harness and DTRG firmware marker
  - [test_rc_conflict.py](#test_rc_conflictpy): RC channel conflict check
  - [test_bench_test_safety.py](#test_bench_test_safetypy): entering and arming bench test
  - [test_bench_test_outputs.py](#test_bench_test_outputspy): bench test setpoints
  - [test_horizontal_thrust.py](#test_horizontal_thrustpy): horizontal thrust in Stabilized
  - [test_csv_mixer.py](#test_csv_mixerpy): CSV mixer without a file
- [Tier 3: SIH flight tests](#tier-3-sih-flight-tests)
  - [test_flight_ht.py](#test_flight_htpy): horizontal thrust in flight

---

## Tier 1: unit tests

```
make tests TESTFILTER=Dtrg
```

58 tests in 4 groups. They exercise PX4 code directly: there is no simulator, no
airframe and no running PX4. Each test is listed by name (`Group.Test`).
Rerun one with:

```
cd build/px4_sitl_test && ctest -R DtrgBenchProfile.StepFollowsSign --output-on-failure
```

**Known gap** tests describe bugs that are not fixed yet and fail against the
current code. Keep them prefixed `DISABLED_` so CI stays green: ctest then
reports them as "Not Run (Disabled)". Remove the prefix once the bug is fixed.
Run them anyway with `--gtest_also_run_disabled_tests` on the test binary in
`build/px4_sitl_test/`.

### DtrgSequentialDesaturation

File: `src/lib/control_allocation/control_allocation/DtrgSequentialDesaturationTest.cpp`
Code under test: `ControlAllocationSequentialDesaturation` (airmode off)

When the rotors cannot deliver everything that is asked for, the allocator gives
up some axes to keep the others. The DTRG order is: thrust X, then thrust Y,
then yaw, then thrust Z (reduce only, never increased), and roll and pitch last.
In practice: the vehicle drops the sideways push before its attitude, and never
climbs to make room for a manoeuvre.

The vehicle is a planar octocopter built in the test, not read from an airframe:
8 rotors evenly spaced on a circle, each tilted tangentially by +-31 deg with
alternating direction and spin, like the DTRG planarOcto. The tilt is what lets
it push sideways without tilting the body. Values are normalised; "hover" is the
thrust Z that puts every rotor at 0.5.

Each saturation test first checks its own premise: the plain pseudo-inverse
allocation of the demand really pushes a rotor outside [0, 1] (or does not,
for the reference case). Otherwise a test could pass without anything being
desaturated.

| Test | Scenario | Passes when |
|---|---|---|
| `GeometryIsFullyActuated` | At hover, +0.2 on one axis at a time (roll, pitch, yaw, X, Y) | Every case is delivered exactly on all 6 axes. Checks the test vehicle, not the allocator: without the rotor tilt, X and Y would come out as 0. Guards the tests below against a broken setup. Nothing saturates, so the desaturation order is not exercised |
| `UnsaturatedDemandIsAllocatedExactly` | Small demand on all 6 axes at once, within the rotor limits | Delivered exactly: desaturation leaves a feasible demand alone |
| `HorizontalThrustIsReducedFirst` | Hover plus far more X thrust than the rotors can give | X is reduced but still forward; Z, Y, roll, pitch and yaw are untouched. The vehicle keeps altitude and attitude and pushes as hard forward as it can |
| `HorizontalThrustIsGivenUpForRoll` | Roll 0.3 (feasible on its own) plus an impossible X demand | Roll and Z are kept exactly, X is reduced |
| `SidewaysThrustIsGivenUpForYaw` | Yaw 0.1 (feasible on its own) plus an impossible Y demand | Yaw and Z are kept exactly, Y is reduced but keeps its direction |
| `YawIsReducedBeforeThrust` | Rotors at 0.9 plus an impossible yaw demand | Thrust Z is kept exactly, yaw is reduced but keeps its direction, roll and pitch stay 0. Upstream PX4 instead gives up to 15 % thrust for yaw |
| `ThrustIsReducedBeforeRoll` | Rotors at 0.9 plus roll 0.6, which pushes the upper rotors past 1 | Roll is kept exactly, thrust Z is lowered, no rotor ends above 1 |
| `ThrustIsNeverIncreasedToDesaturate` | Rotors at 0.1 plus roll 1, which pushes the lower rotors below 0 | Thrust Z is not raised to make room (airmode off), roll is reduced instead |
| `PublishesTopic` | One allocation at hover | The `sequential_desaturation` uORB topic is published, all six `*_sat` gains 0 |
| `TopicReportsHorizontalThrustReduction` | **Known gap.** Impossible X demand at hover | Should report `x_sat` > 0.01 and roll / pitch near 0. Fails: `desaturateActuators()` returns only the gain of its second, half strength pass, so when the first pass already fixes the saturation the reported gain is 0 although X was cut. The sign is also lost on the way to `SYS_STATUS.errors_count1` |
| `DesaturationDoesNotAddUnrequestedAxes` | **Known gap.** Rotors at 0.9 plus roll 1 | Should add no X, Y, yaw or pitch. Fails: the X, Y and yaw steps slide along their axis to reduce any saturation, so the output contains X thrust (about -0.02) and yaw (about -0.12) that nobody asked for |

### DtrgMixerCsv

File: `src/lib/control_allocation/control_allocation/DtrgMixerCsvTest.cpp`
Code under test: `ControlAllocationPseudoInverse::readMixerFromCSV()`

With `DTRG_MIXER_CSV` set, the allocator replaces the pseudo-inverse of the
effectiveness matrix with a matrix read from `/fs/microsd/etc/mixer.csv`: one row
per actuator, one column per axis (roll, pitch, yaw, thrust X, Y, Z). These
tests write temporary CSV files and check what the parser makes of them.

Before each test the matrix is filled with 99 ("untouched"), and the file cells
hold `row + 0.1 * (column + 1)`, so a value in the wrong place is obvious.

| Test | File | Passes when |
|---|---|---|
| `MissingFileIsRejectedAndMixerUntouched` | Does not exist | Rejected, the matrix is left as it was |
| `ReadsOctoMatrix` | 8 rows of 6 values | All 48 values land in the right row and column |
| `RowsBeyondTheFileAreUntouched` | 8 rows | Rows 9 onwards keep their previous values: the parser does not clear them |
| `Utf8BomIsSkipped` | UTF-8 byte order mark first (Excel "CSV UTF-8") | Read as if the mark were absent |
| `CrlfLineEndings` | Windows line endings | Read correctly |
| `NoTrailingNewline` | Last row without a newline | Last row still read |
| `BlankLinesAreSkipped` | Blank lines before and between rows | Blank lines do not count as rows |
| `NegativeAndScientificValues` | `-0.5,1e-1,-2.5E-2,0,+1,-1` | Every notation parsed to the right value |
| `ExtraColumnsAreIgnored` | 8 values on a row | First 6 used, the extra ones do not spill into the next row |
| `ExtraRowsAreIgnored` | More rows than the allocator has actuators | Stops at the last actuator, no overflow |
| `FullPrecisionRowsAreNotSplit` | **Known gap.** Full precision export (e.g. MATLAB `writematrix`, ~20 characters per cell) | Should read 2 rows. Fails: a row is longer than the 100 byte line buffer, the rest of the line becomes the next actuator's row and every following row shifts |
| `EmptyCellKeepsColumnPosition` | **Known gap.** `1,,3,4,5,6` (how spreadsheets write an empty cell) | Should keep 3 in column 3. Fails: `strtok()` merges the two commas and the rest of the row shifts one column left |
| `EmptyFileIsRejected` | **Known gap.** Empty | Should be rejected. Fails: accepted, leaving the allocator without a mixer |
| `ShortRowIsRejected` | **Known gap.** `1,2,3` | Should be rejected. Fails: accepted, and the missing axes keep whatever the matrix held before |
| `BlankCrlfLineIsSkipped` | **Known gap.** Blank Windows line before the data | Should be skipped. Fails: read as a row of zeros, shifting the real rows down |

### DtrgBenchSwitch

File: `src/modules/bench_test/DtrgBenchSwitchTest.cpp`
Code under test: `bench_test_switch.h`

The bench test direction comes from a 3-position switch on the RC channel set by
`RC_MAP_CMD_SIGN`: above 1700 us is +1, below 1300 us is -1, anything else is
centre (0). Commander uses the same decoding for its "centre the switch" arming
check, so a bug here affects both the excitation and the arming gate. Every
doubtful case must decode as centre, the safe position.

| Test | Input | Passes when |
|---|---|---|
| `PulseThresholds` | 1, 1000, 1299, 1300, 1500, 1700, 1701, 2000 us | 1701 and up is +1, 1299 and down is -1, 1300 to 1700 inclusive is centre: the thresholds themselves are centre |
| `ZeroPulseIsCentre` | 0 us (unpopulated channel) | Centre, not "switch down" |
| `InputRcUsesOneBasedChannel` | Channel 6 high, channel 7 low | `RC_MAP_CMD_SIGN = 6` reads `values[5]`: channel numbers start at 1 |
| `DisabledChannelIsCentre` | `RC_MAP_CMD_SIGN` 0 or negative, every channel high | Centre |
| `ChannelBeyondChannelCountIsCentre` | 8 channels received, switch on channel 8, 9, or 19 | Channel 8 is read, a channel the receiver does not send is centre |
| `ChannelBeyondArrayIsCentreEvenIfCountClaimsMore` | Corrupt `channel_count` of 255 | Never reads past the 18 channel array |
| `RcLossIsCentre` | Switch high with RC lost, switch low with RC failsafe | Centre: a stale value is not used |

### DtrgBenchProfile

File: `src/modules/bench_test/DtrgBenchProfileTest.cpp`
Code under test: `bench_test_profile.h` (the maths of `BenchTest.cpp`)

The bench test mode holds a hover thrust and excites one axis with a step or a
ramp, in the direction of the switch above. Each group below maps to the
parameters that drive it.

**Step** (`BT_STEP_DELAY`, `BT_STEP_DUR`, `BT_STEP_MAG`). Tests use 2 s delay,
1 s duration, magnitude 0.1.

| Test | Passes when |
|---|---|
| `StepIsZeroBeforeDelay` | Output is 0 up to the end of the delay |
| `StepIsActiveForDuration` | Output is 0.1 from 2 s up to (not including) 3 s, then 0 for good |
| `StepFollowsSign` | Switch down gives -0.1, centre gives 0 |
| `StepWithoutDelayStartsImmediately` | With no delay the step is on at t = 0 |

**Ramp** (`BT_RAMP_RATE`, `BT_MAX_VAL`). Tests use 0.1 per second, clamp 0.5.

| Test | Passes when |
|---|---|
| `RampIncreasesLinearly` | 0, 0.1, 0.2 at 0, 1, 2 s, not frozen |
| `RampNegativeSign` | Switch down gives -0.2 at 2 s |
| `RampStopsAtMaxValAndFreezes` | Clamped to 0.5 and frozen, still 0.5 later |
| `RampNegativeStopsAtMinusMaxVal` | Clamped to -0.5 and frozen |
| `RampFreezesOnMotorSaturation` | Freezes at the value where a motor saturated and holds it, even after the motor leaves saturation |
| `RampResetClearsFreeze` | After a reset (profile restarted) the ramp is unfrozen and starts again from 0 |

**Spin-up** (`BT_SPINUP_T`): the hover thrust is soft-started when the outputs
come on.

| Test | Passes when |
|---|---|
| `SpinupRampsToOne` | Over 2 s the factor goes 0, 0.5, 1, and stays 1 |
| `SpinupDisabled` | A spin-up time of 0 or less gives 1 straight away |

**Axis mapping** (`BT_AXIS`, `BT_HOVER_THR`): how the hover thrust and the
excitation become thrust and torque setpoints. Body frame, NED, so -Z is up.

| Test | Passes when |
|---|---|
| `HoverBaselineIsUpwardThrustOnly` | Hover 0.4 at half spin-up, no excitation: thrust Z is -0.2 and everything else 0 |
| `EachAxisDrivesOnlyItsSetpoint` | X, Y, roll, pitch and yaw each drive only their own setpoint; thrust Z keeps the hover baseline |
| `ThrustZExcitationAddsUpwardThrust` | Excitation on Z adds lift: hover 0.2 plus 0.1 gives thrust Z -0.3 |
| `ExcitationIsNotScaledBySpinup` | At spin-up 0 the hover thrust is 0 but the excitation is applied in full: only the hover baseline is soft-started |

**Motor saturation** (`BT_NUM_MOTORS`, `BT_SAT_MARGIN`): decides when the ramp
freezes. Tests use a margin of 0.05.

| Test | Passes when |
|---|---|
| `CountFiniteMotors` | Counts the outputs that are not NaN (0, 4, 8). Used when `BT_NUM_MOTORS` is 0 (auto-detect) |
| `MidRangeIsNotSaturated` | All motors at 0.5: not saturated |
| `UpperMarginSaturates` | One motor at 0.95 is saturated, at 0.94 it is not |
| `LowerMarginSaturates` | One motor at 0.05 is saturated, at 0.06 it is not |
| `ReversibleMotorUsesSymmetricRange` | A reversible motor spans [-1, 1]: 0 is mid-range, -0.95 is saturated |
| `UnconnectedMotorsAreIgnored` | Motor 5 at full output is ignored with 4 motors, counted with 5 |
| `NonFiniteOutputsAreIgnored` | A NaN output does not count as saturated |
| `MarginIsClamped` | A margin above 0.5 behaves as 0.5, a negative margin as 0 (only exactly 0 or 1 counts) |
| `MotorCountAboveArrayIsBounded` | A motor count of 1000 never reads past the output array |

---

## Tier 2: SIH logic tests

```
make px4_sitl_default
pip3 install -r test/dtrg/requirements.txt
python3 -m pytest test/dtrg -m "sih and not flight" -v
```

These run the whole PX4 SITL with the SIH simulator on the planarOcto
(`sihsim_planar_octo`) and talk to it over MAVLink like a ground station. They
check decisions and setpoints (arming accepted or denied, mode, status texts,
the published setpoints), not how the vehicle moves: nothing takes off here.
How to run, options and gotchas are in [test/dtrg/README.md](../../test/dtrg/README.md).

Every test boots a fresh PX4 with a clean rootfs (10-25 s per test). Its
parameters are set at boot through `PX4_PARAM_*`, and the fixture fails the
test if one did not boot with the requested value. Tests that need RC stream
`RC_CHANNELS_OVERRIDE` at 50 Hz with one fixed layout (`test/dtrg/rc_layout.py`):

| Channel | Function |
|---|---|
| 1-4 | roll, pitch, throttle, yaw |
| 5 | flight mode switch: slot 1 Stabilized, slot 4 Position, slot 6 Bench test |
| 6 | bench test direction switch (`RC_MAP_CMD_SIGN`) |
| 8 | horizontal thrust on/off (`RC_MAP_HT_MODE`) |
| 9, 10 | horizontal thrust aux roll / pitch tilt (`RC_MAP_HT_ROLL`, `RC_MAP_HT_PITCH`) |

RC starts safe: sticks centred, throttle low, Stabilized slot, switches off.

Rerun one test, keeping its `px4.log` and ULogs:

```
python3 -m pytest test/dtrg -k bench_test_rejected_while_armed -v --basetemp=/tmp/dtrg
```

**Known gap** tests describe bugs that are not fixed yet. They are marked
`xfail(strict=True)`: they pass while the bug is there and turn red once it is
fixed, as a reminder to drop the marker.

### test_smoke.py

Checks the harness itself before anything else is blamed.

| Test | Passes when |
|---|---|
| `test_boots_dtrg_firmware_and_is_ready_to_arm` | PX4 boots, `SYS_STATUS.errors_count4` is 706 (the DTRG firmware marker, so this is not an upstream build) and the arming checks pass |
| `test_rc_override_drives_the_flight_mode_switch` | With RC streamed, the mode switch puts the vehicle in Stabilized and `COM_RC_IN_MODE` is 0 (RC only): RC override really reaches the mode logic |

### test_rc_conflict.py

Code under test: commander `rcChannelConflictCheck`

Two `RC_MAP_*` functions on the same raw channel would make one switch do two
things (e.g. the throttle stick also toggling horizontal thrust). Commander must
refuse to arm, name both parameters, and allow arming again once the overlap is
gone. `COM_ARM_RC_CONF=1` turns the failure into a warning. The conflict used is
`RC_MAP_HT_MODE` moved onto the throttle channel (3).

No RC is streamed: the check only reads the configuration, and
`COM_RC_IN_MODE=1` keeps a missing RC link from failing arming for another reason.

| Test | Scenario | Passes when |
|---|---|---|
| `test_conflict_blocks_arming_and_names_both_parameters` | Conflict created at runtime | Arming is refused, `Preflight Fail: RC_MAP_HT_MODE and RC_MAP_THROTTLE both use RC channel 3` is sent, vehicle stays disarmed |
| `test_resolving_conflict_allows_arming_again` | Conflict created, then `RC_MAP_HT_MODE` moved back to channel 8 | `RC channel conflict resolved` is sent, the arming checks pass and the vehicle arms |
| `test_conflict_configured_before_boot_blocks_arming` | Conflict already in the parameters at boot | The failure is printed on the console at boot; with `COM_ARM_RC_CONF=1` arming is allowed, back to 0 it is refused again. Checks the boot-time scan of the parameter table, not only the change handler |
| `test_com_arm_rc_conf_warns_without_blocking` | `COM_ARM_RC_CONF=1`, conflict created | `RC ch 3: ...` warning is sent and the vehicle still arms |
| `test_failsafe_channel_may_share_throttle` | `RC_MAP_FAILSAFE` on the throttle channel | Not a conflict: the failsafe channel is documented to sit on throttle |
| `test_flight_mode_buttons_only_count_without_mode_switch` | `RC_MAP_FLTM_BTN` includes channel 8 (the HT switch) | Not a conflict while `RC_MAP_FLTMODE` is set (the buttons are ignored then); a conflict once `RC_MAP_FLTMODE` is 0 |

### test_bench_test_safety.py

Code under test: commander (mode and arming rules for bench test)

Bench test drives the motors with every control loop off, so it is fenced in:
only reachable from an RC mode slot, never over MAVLink; never entered while
armed; only armed with `BT_ARM_ENABLE=1` and the direction switch centred; and it
may always be disarmed. Tests use a hover thrust of 0.2, far below what lifts
the vehicle, since SIH does not tie it down like a real rig.

| Plan ID | Test | Passes when |
|---|---|---|
| - | `test_rc_slot_selects_bench_test_while_disarmed` | Moving the mode switch to slot 6 while disarmed enters bench test (HEARTBEAT main mode 11). Control case for B1 and B4 |
| B1 | `test_bench_test_rejected_while_armed` | Armed in Stabilized, switching to slot 6 gives `Bench test mode denied: disarm first`; the vehicle stays armed in Stabilized. Once disarmed, the same switch (moved away and back) enters bench test |
| B4 | `test_mavlink_cannot_select_bench_test` | `DO_SET_MODE` to main mode 11 leaves the vehicle in Stabilized. Commander ACKs it as accepted (an unknown custom mode is a no-op), so the test checks the mode, not the ACK |
| B2 | `test_arming_needs_bt_arm_enable` | In bench test with `BT_ARM_ENABLE=0`, arming is refused with `Arming denied: bench test not enabled`; after setting it to 1 the same request arms |
| B3 | `test_arming_needs_centred_direction_switch[switch_up/switch_down]` | Direction switch up or down: arming is refused with `Arming denied: centre the bench test direction switch`; once centred, it arms. Otherwise the excitation would start the moment the motors spin up |
| B3b | `test_off_centre_switch_allowed_when_it_cannot_excite[hover_only_profile/no_switch_assigned]` | With `BT_MODE=0` (hover only) or no switch assigned (`RC_MAP_CMD_SIGN=0`), an off-centre switch cannot excite anything, so arming is allowed |
| B6 | `test_disarm_allowed_in_bench_test` | Armed in bench test, disarm is honoured. On a real rig the land detector reports "in air" once the motors spin and bench test allows disarming anyway; SIH stays landed, so this only checks a normal disarm (the in-air case needs a rig) |
| - | `test_rc_layout_has_no_conflicts` | No two functions in `rc_layout.RC_PARAMS` share a channel. Guards every RC test against failing on the conflict check instead of what it tests. Does not boot PX4 |

### test_bench_test_outputs.py

Code under test: `bench_test` module wiring (the maths is unit tested in
[DtrgBenchProfile](#dtrgbenchprofile))

Each test arms in bench test, waits for the spin-up, moves the direction switch
for a while, centres it, disarms, then reads `vehicle_thrust_setpoint` and
`vehicle_torque_setpoint` back from the ULog. The excited axis is thrust Z
(`BT_AXIS=2`), hover 0.2, spin-up 1 s. Setpoints are NED body frame: -Z is up.

| Plan ID | Test | Profile | Passes when |
|---|---|---|---|
| B5 | `test_step_profile[up/down]` | Step: delay 1 s, magnitude 0.1, duration 1 s | Hover baseline -0.2 is reached after the spin-up; exactly one step of 1 s (+-0.1 s) to -0.3 (switch up) or -0.1 (switch down); back to -0.2 afterwards; thrust X/Y and all torques stay 0; thrust Z is 0 whenever bench test is disarmed |
| B5 | `test_spinup_ramps_hover_thrust` | Hover only | Thrust Z starts near 0 when arming, never jumps, and follows a linear ramp to -0.2 over `BT_SPINUP_T` (+-0.03) |
| B5 | `test_ramp_profile_stops_at_max_value` | Ramp: 0.1 per second, `BT_MAX_VAL` 0.05 | The excitation never exceeds 0.05, is held there for over 1 s, and drops back to 0 once the switch is centred |

### test_horizontal_thrust.py

Code under test: mc_att_control horizontal thrust (HT) in Stabilized

HT lets a fully actuated vehicle move sideways without tilting: with the HT
switch on, the roll and pitch sticks command body X/Y thrust instead of
attitude, and the aux channels 9 and 10 command a tilt. These tests check the
wiring from RC to the `vehicle_attitude_setpoint` that mc_att_control publishes
(`thrust_body` and the roll / pitch of `q_d`), disarmed, 1.5 s after each RC
change. Parameters: `DTRG_HT_MAX` 0.5, `DTRG_HT_R_MAX` and `DTRG_HT_P_MAX` 10 deg.
"Half stick" is 1750 us, which the RC deadzone turns into 0.49.

| Test | RC | Passes when |
|---|---|---|
| `test_switch_off_is_standard_stabilized` | HT off, full pitch stick, aux roll full | No X/Y thrust, pitch below -5 deg (normal nose down), aux roll does nothing, `horizontal_thrust_limit` not published |
| `test_switch_on_sticks_command_thrust_and_vehicle_stays_level` | HT on, full pitch stick, half roll stick | Thrust X = 0.5 (`DTRG_HT_MAX`), Y = 0.49 x 0.5, roll and pitch 0 (+-0.5 deg), `horizontal_thrust_limit` published |
| `test_switch_toggles_ht_at_runtime` | Full pitch stick, HT switch off, on, off | Setpoint follows each change: tilt, then level with X thrust, then tilt again |
| `test_switch_ignored_when_ht_disabled` | `DTRG_HT_EN=0`, HT on, full pitch stick | The switch is ignored: no X thrust, normal tilt, `horizontal_thrust_limit` not published |
| `test_aux_channels_command_tilt_up_to_limit` | HT on, aux roll full, then aux pitch full | Roll +10 deg with pitch 0, then pitch -10 deg (nose down, like the pitch stick) with roll 0 |
| `test_aux_channel_deadzone[1505/1515]` | HT on, aux roll at 1505 or 1515 us | 1505 us (0.01) is inside the 0.02 aux deadzone: roll 0. 1515 us (0.03) tilts by 0.03 x 10 deg |
| `test_mask_selects_thrust_axes[mask0/1/2]` | `DTRG_HT_MASK` 0-2, HT on, full pitch, half roll | 0: X and Y by thrust; 1: X only; 2: Y only |
| `test_mask_selects_thrust_axes[mask3]` | **Known gap (G6).** `DTRG_HT_MASK=3` | Should give no X/Y thrust (move by tilting only). Fails: HT is applied on both axes. Fixed on branch `salz167/DTRG_HT_refactor` |
| `test_full_stick_reports_ht_saturation` | **Known gap (G3).** HT on, full pitch stick | Should set `horizontal_thrust_limit.x_sat` and `SYS_STATUS.errors_count3` bit 0. Fails: the demand is `stick * DTRG_HT_MAX`, and RC scaling gives 0.9999999, so X is 0.49999994 and never reaches the limit |

### test_csv_mixer.py

Code under test: control allocation with `DTRG_MIXER_CSV` (the parser is unit
tested in [DtrgMixerCsv](#dtrgmixercsv))

The mixer file path is hardcoded to `/fs/microsd/etc/mixer.csv`, which does not
exist in SITL, so only the "no file" case can run here.

| Plan ID | Test | Passes when |
|---|---|---|
| D2 | `test_csv_mixer_without_file_refuses_to_arm` | **Known gap (G4).** Should refuse to arm with `DTRG_MIXER_CSV=1` and no file. Fails: the allocator keeps an all-zero mixer and nothing stops arming, so the motors would not respond. Decide the behaviour (refuse to arm, or fall back to the geometry) before fixing |

---

## Tier 3: SIH flight tests

```
python3 -m pytest test/dtrg -m flight -v
```

About 10 flights, ~10 minutes. Same harness, RC layout and known gap
convention as [tier 2](#tier-2-sih-logic-tests), but here the planarOcto really
flies: `SIH_VEHICLE_TYPE 4` builds the vehicle from the airframe's `CA_ROTOR*`
geometry (the same one control allocation uses), so the eight tilted rotors can
push sideways without tilting the body. These tests are marked `flight` and
`fully_actuated`, so they are skipped on `--airframe=sihsim_quadx`.

Each test takes off in Takeoff mode to 3 m (`MIS_TAKEOFF_ALT`), waits until the
vehicle holds there, and then does its manoeuvre. Afterwards it reads two
things back from the ULog:

- **SIH ground truth** (`vehicle_*_groundtruth`), for how the vehicle really
  moved and tilted.
- **The estimate against its setpoint**, for "holds altitude / position"
  (called drift below). With SIH's simulated baro and GPS, the estimate wanders
  0.3-0.5 m in altitude and up to ~1 m horizontally from the truth. That is
  estimator error, not the behaviour under test, so holding is judged the way
  the controller sees it.

HT parameters: `DTRG_HT_MAX` 0.5, `DTRG_HT_R_MAX` and `DTRG_HT_P_MAX` 10 deg.
"Tilt" is max(|roll|, |pitch|). The tolerances are first guesses: in a plain
hover with HT off the vehicle tilts ~3 deg (std, up to ~9 deg) while the
position controller corrects the simulated GPS/IMU noise. Tighten them after
~20 green CI runs.

Known gap tests here use `xfail(strict=True, raises=...)`: only the assertion
that describes the gap counts as the expected failure, so a timeout or crash
in the same test still fails it.

### test_flight_ht.py

Code under test: mc_pos_control and mc_att_control horizontal thrust, and
commander, in flight

| Plan ID | Test | Manoeuvre | Passes when |
|---|---|---|---|
| A1 | `test_a1_take_off_hold_and_land` | Take off, hold 10 s, Land | Altitude within 0.5 m of its setpoint and drift < 1 m during the hold; lands and disarms within 30 s. Baseline: if this fails, nothing below means anything |
| A2 | `test_a2_ht_moves_the_vehicle_level` | HT on, Offboard position setpoint 5 m north | Moves 5 m (+-0.5) north and the true tilt stays below 3 deg throughout: HT moves the vehicle without tilting it |
| A3 | `test_a3_without_ht_the_vehicle_tilts_to_move` | Same move with HT off | Moves 5 m (+-0.5) and pitches nose down past -5 deg. Control case for A2: shows A2's tilt limit would catch a vehicle that moves by tilting |
| A4 | `test_a4_aux_tilt_in_hover_holds_position[roll/pitch]` | **Known gap (G11).** HT on, Hold, aux roll or pitch channel full for 8 s | Should tilt to 10 deg (+-3.5, true) and drift < 0.5 m. Fails on the drift: HT delivers only 41% of the horizontal force the position controller asks for, so it cannot hold against the tilt and slides ~0.5 m/s |
| A4b | `test_a4b_aux_tilt_reaches_the_limit[roll]` | HT on, Hold, aux roll full | The attitude half of A4, independent of G11: roll +10 deg (estimate +-2, truth +-3.5), pitch below 3 deg |
| A4b | `test_a4b_aux_tilt_reaches_the_limit[pitch]` | **Known gap (G13).** HT on, Hold, aux pitch full | Should pitch -10 deg (nose down, as the same channel does in Stabilized, see `test_aux_channels_command_tilt_up_to_limit`). Fails: in Position / Hold / Offboard the channel pitches nose up. Which sign is intended still needs deciding |
| A5 | `test_a5_offboard_tilt_setpoint_in_hover` | HT on, Offboard hold, `DEBUG_FLOAT_ARRAY` roll 0.1 rad (5.7 deg) at 10 Hz for 8 s | Estimated roll 5.7 deg (+-2), true roll +-3.5 (the estimate drifts ~2 deg from the truth while tilted, G14), drift < 0.5 m, still in Offboard |
| A5b | `test_a5_offboard_tilt_is_limited` | **Known gap (G8).** Same, roll setpoint 20 deg (twice `DTRG_HT_R_MAX`) | Should stay within 10 deg (+2). Fails: the Offboard HT tilt is neither limited nor checked for NaN, so any MAVLink source can command any tilt |
| A7 | `test_a7_toggling_ht_in_hover_is_smooth` | Hold 8 s as a reference, then HT switch on / off 5 times, 2 s each | Altitude within 0.7 m of its setpoint, drift < 1 m, and tilt below max(8 deg, the reference hover's tilt + 2 deg): switching HT does not kick the vehicle |
| A8 | `test_a8_stabilized_ht_stick_moves_the_vehicle_level` | Take off to 6 m, HT on, Stabilized, full pitch stick for 3 s | Forward speed (true, along the heading) above 1 m/s and tilt below 3 deg. The manual counterpart of A2 |
| B1b | `test_b1b_bench_test_rejected_in_flight` | Hovering in Hold, mode switch to the bench test slot | `Bench test mode denied: disarm first`; still armed and in Hold, and stays above 1.5 m. The in-air case of tier 2's B1 |

A6 (allocator cuts X/Y before roll/pitch in flight) is not written: it needs
the allocator's per-axis output in the log, which the desaturation topic cannot
give yet (G1).
