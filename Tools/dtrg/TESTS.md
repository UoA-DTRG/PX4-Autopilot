# DTRG test list

What each DTRG test checks and why it matters. The plan, CI and known gaps
are in [SITL_TESTING.md](SITL_TESTING.md).

- [Tier 1: unit tests](#tier-1-unit-tests)
  - [DtrgSequentialDesaturation](#dtrgsequentialdesaturation): desaturation order
  - [DtrgMixerCsv](#dtrgmixercsv): CSV mixer parser
  - [DtrgBenchSwitch](#dtrgbenchswitch): bench test direction switch
  - [DtrgBenchProfile](#dtrgbenchprofile): bench test excitation profile

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
