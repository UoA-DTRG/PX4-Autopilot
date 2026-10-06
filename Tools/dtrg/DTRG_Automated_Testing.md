# DTRG automated testing

Everything about the automated tests of the DTRG features: what they cover, how
to run them, how CI runs them, how to add a new one, what each existing test
asserts, and the known gaps they pin. Replaces the earlier split between
`SITL_TESTING.md` (plan, CI, gaps) and `TESTS.md` (test list).

## Contents

| Section | |
| --- | --- |
| [1. What is tested](#1-what-is-tested) | the features, the three tiers, the simulated vehicle |
| [2. Running the tests](#2-running-the-tests) | per tier, one test, options, artifacts |
| [3. CI](#3-ci) | jobs, triggers, artifacts, required checks |
| [4. Adding a test](#4-adding-a-test) | where it goes, the fixtures, the traps |
| [5. The current tests](#5-the-current-tests) | every test and what it asserts |
| [6. Notes](#6-notes) | known gaps, facts the tests rely on, local quirks |

Related: [test/dtrg/README.md](../../test/dtrg/README.md) (tier 2/3 harness),
[.github/workflows/dtrg_tests.yml](../../.github/workflows/dtrg_tests.yml) (CI),
[README.md](README.md) (manual SITL input tools),
[SIH_VS_GAZEBO.md](SIH_VS_GAZEBO.md) (simulator comparison).

---

## 1. What is tested

The DTRG changes to PX4, as automated tests that run locally and on GitHub
Actions on every push and pull request to `dtrg-main`:

| Feature | Parameters | Where it lives |
| --- | --- | --- |
| **Horizontal thrust (HT)** — a fully actuated vehicle moves sideways without tilting; sticks command body X/Y thrust, aux channels or an offboard setpoint command the tilt | `DTRG_HT_EN`, `DTRG_HT_MAX`, `DTRG_HT_MASK`, `DTRG_HT_SPLIT_EN`, `DTRG_HT_SPLIT`, `DTRG_HT_R_MAX`, `DTRG_HT_P_MAX`, `RC_MAP_HT_MODE/ROLL/PITCH` | `src/lib/dtrg_horizontal_thrust`, mc_att_control, mc_pos_control |
| **Bench test mode** — holds a hover thrust and excites one axis with a step or a ramp, with every control loop off, for rig measurements | `BT_*`, `RC_MAP_CMD_SIGN`, `COM_FLTMODEx = 16` | `src/modules/bench_test` |
| **Sequential desaturation order** — which axes are given up when the rotors cannot deliver everything | - | `ControlAllocationSequentialDesaturation` |
| **CSV mixer** — replace the pseudo-inverse with a matrix read from `/fs/microsd/etc/mixer.csv` | `DTRG_MIXER_CSV`, `DTRG_MIXER_NORM` | `ControlAllocationPseudoInverse` |
| **RC channel conflict check** — refuse to arm when two `RC_MAP_*` functions share a raw channel | `COM_ARM_RC_CONF` | commander |

### The three tiers

| Tier | Question it answers | How | Tests | Runtime | CI job |
| --- | --- | --- | --- | --- | --- |
| **1. Unit** | Is the maths and parsing right? | gtest on the code directly, no simulator, no PX4 running | 125 in 7 groups | seconds (after the build) | `unit` |
| **2. SIH logic** | Are the decisions right? (arming, modes, status texts, published setpoints) | full PX4 SITL + SIH simulator, driven over MAVLink like a GCS; nothing takes off | 43 | 10-25 s per test | `sih` |
| **3. SIH flight** | Does the vehicle really behave that way? | the fully actuated planarOcto flies in SIH; assertions on SIH ground truth | 16 | ~10 min total | `flight` |

Each tier exists because the one below it cannot answer the question:

- Tier 1 can check every edge case cheaply (a 43 character CSV cell, a NaN RC
  channel, a corrupt channel count) but knows nothing about PX4's wiring: a
  correct helper called with the wrong argument still passes.
- Tier 2 catches the wiring — that the RC channel reaches the setpoint, that
  commander really refuses to arm, that the status text says what the tests
  claim — but it asserts on *intent*, not on motion. A test can pass on a
  vehicle that could never fly the manoeuvre.
- Tier 3 is the only tier that can fail on physics: gain loss, drift while
  tilted, a kick when HT is toggled. It is also the slowest and the noisiest,
  so it is kept small and its tolerances are loose.

Both SITL tiers run on the planarOcto (`sihsim_planar_octo`), the default
airframe. Tier 2 also passes on `--airframe=sihsim_quadx`, which skips the tests
marked `fully_actuated`; tier 3 is fully actuated throughout.

### The simulated vehicle

`SIH_VEHICLE_TYPE 4` (generic multirotor) reads `CA_ROTOR_COUNT` and
`CA_ROTORn_PX/PY/PZ/AX/AY/AZ/CT/KM` — the same parameters control allocation
uses — and sums per rotor `F = T * axis` and `M = r x F - KM * T * axis`, with
`T = SIH_T_MAX * (SIH_THR_MDL_FAC * u^2 + (1 - SIH_THR_MDL_FAC) * u)`. Any
multirotor that control allocation can describe therefore flies without code
changes, and the eight tangentially tilted rotors of the planarOcto can push
sideways without tilting the body. Output n drives rotor n, so map Motor 1..N to
outputs 1..N in order.

| Airframe | Where | |
| --- | --- | --- |
| `12016_sihsim_planar_octo` | SITL (`make px4_sitl sihsim_planar_octo`) | CI and local runs |
| `12017_dtrg_planar_octo_sih.hil` | the real flight controller (`SYS_HITL 2`) | the planarOcto stack flying on the actual hardware against a vehicle simulated on the board; real outputs stay off (remove the props anyway) |

Mass, inertia and propulsion are the Gazebo model's values (assumed / FT-X8, not
measured), documented in `12016_sihsim_planar_octo`. With them the planarOcto
hovers at 0.56 thrust with all eight motors between 0.49 and 0.65: no
saturation, so the saturation seen in Gazebo comes from that model, not from the
geometry or the allocation. The Gazebo model is not needed in CI (keep it for
manual checks of the model).

---

## 2. Running the tests

### Prerequisites

```bash
make px4_sitl_default                             # tiers 2 and 3
python3 -m venv ~/.venvs/px4-dtrg                 # once
~/.venvs/px4-dtrg/bin/pip install -r test/dtrg/requirements.txt   # once
source ~/.venvs/px4-dtrg/bin/activate             # in every new terminal
```

A virtualenv is needed where `python3` is an externally managed install
(Homebrew on macOS, recent Debian/Ubuntu): there `pip3 install` is refused and
pytest then fails with `No module named pytest`. Without activating, call the
venv's Python directly: `~/.venvs/px4-dtrg/bin/python -m pytest ...`.

### Tier 1: unit tests

```bash
make tests TESTFILTER=Dtrg                  # every DTRG unit test
make tests TESTFILTER=ControlAllocation     # the upstream allocator tests DTRG changes
make tests                                  # the whole PX4 suite
```

`TESTFILTER` is a ctest regex, but `|` breaks the Makefile's shell quoting, so
run one filter at a time.

One test, or one group, from the build directory:

```bash
cd build/px4_sitl_test
ctest -R DtrgBenchProfile.StepFollowsSign --output-on-failure
ctest -R DtrgMixerCsv --output-on-failure
./unit-DtrgHorizontalThrust --gtest_filter='*Split*'
./unit-DtrgHorizontalThrustSplit
```

### Tier 2: SIH logic tests

```bash
python -m pytest test/dtrg -m "sih and not flight" -v
```

### Tier 3: SIH flight tests

```bash
python -m pytest test/dtrg -m flight -v      # ~10 flights, ~10 minutes
```

### One SIH test, or one file

```bash
python -m pytest test/dtrg -k bench_test_rejected_while_armed -v --basetemp=/tmp/dtrg
python -m pytest test/dtrg/test_rc_conflict.py -v
python -m pytest "test/dtrg/test_flight_ht.py::test_aux_tilt_reaches_the_limit[roll]" -v
```

`--basetemp` keeps the run's `px4.log` and ULogs where you can find them; without
it pytest keeps the last three runs under the system temp directory.

### Options

Pass them as `--opt=value`: with a space, pytest reads the value as a test path.

| Option | Env | Default | |
| --- | --- | --- | --- |
| `--px4-build=DIR` | `DTRG_PX4_BUILD` | `build/px4_sitl_default` | any SITL build, e.g. `build/px4_sitl_test` from `make tests` |
| `--px4-instance=N` | `DTRG_PX4_INSTANCE` | `0` | a free instance to run next to another SITL (ports 14540+N) |
| `--airframe=NAME` | `DTRG_SIH_AIRFRAME` | `sihsim_planar_octo` | see `test/dtrg/airframes.py`; `sihsim_quadx` skips the `fully_actuated` tests |
| `--speed-factor=X` | `DTRG_SPEED_FACTOR` | `1` | `PX4_SIM_SPEED_FACTOR` |

### What a failing SITL test leaves behind

Per test, under its `tmp_path` (`--basetemp` on CI): `px4.log` (the full console
output of that PX4 boot) and the run's ULogs. Tier 3 assertions are all read
back from the ULog, so the log of a failed flight contains everything the test
looked at.

---

## 3. CI

[.github/workflows/dtrg_tests.yml](../../.github/workflows/dtrg_tests.yml), named
**DTRG Tests**. Triggers: every push to `dtrg-main`, every pull request into
`dtrg-main`, and `workflow_dispatch`. Documentation-only changes are skipped
(`paths-ignore`: `docs/**`, `Documentation/**`, `**.md`). A new push to the same
ref cancels the previous run.

All three jobs run on `ubuntu-latest` in the `px4io/px4-dev-base-focal:2021-09-08`
container, restore a ccache, and start with the `git config --system --add
safe.directory '*'` workaround (the container runs as a different user than the
one that checked out).

| Job | Name | Steps | Timeout |
| --- | --- | --- | --- |
| `unit` | Unit tests (tier 1) | `make tests TESTFILTER=Dtrg`, then `TESTFILTER=ControlAllocation`, then the full `make tests` as a **non-blocking** step (`continue-on-error`) until it has been seen green on this fork | 45 min |
| `sih` | SIH logic tests (tier 2) | build `px4_sitl_default`, install `test/dtrg/requirements.txt`, `pytest test/dtrg -m "sih and not flight" --airframe=sihsim_planar_octo --px4-build=build/px4_sitl_default --basetemp=test-out --junitxml=test-out/junit.xml` | 60 min |
| `flight` | SIH flight tests (tier 3) | same build and install, `pytest test/dtrg -m flight --airframe=sihsim_planar_octo --basetemp=test-out --junitxml=test-out/junit.xml` | 60 min |

On failure `sih` and `flight` upload `test-out/` as `dtrg-sih-test-output` /
`dtrg-flight-test-output`: the `px4.log` and ULogs of every test, plus the JUnit
XML. `ccache -s` runs at the end of every job.

Caches: `unit` uses `sitl-test-ccache-*` (the `px4_sitl_test` build), `sih` and
`flight` share `sitl-ccache-*` with `dtrg_build.yml`'s `px4_sitl_default` build.

**Branch protection.** Make `Unit tests (tier 1)` and `SIH logic tests (tier 2)`
required checks on `dtrg-main`, and `SIH flight tests (tier 3)` once it has been
green for a while. To test pushes to feature branches before a PR exists, widen
`on.push.branches`.

---

## 4. Adding a test

### Tier 1: a unit test

1. Put the test next to the code, named `Dtrg<Thing>Test.cpp`. The `Dtrg` prefix
   is what `make tests TESTFILTER=Dtrg` matches, so a differently named file is
   not run by CI's DTRG step.
2. Register it in that directory's `CMakeLists.txt`:

   ```cmake
   px4_add_unit_gtest(SRC DtrgBenchSwitchTest.cpp DISCOVER)
   px4_add_functional_gtest(SRC DtrgMixerCsvTest.cpp DISCOVER LINKLIBS ControlAllocation)
   ```

   `px4_add_unit_gtest` for code that needs nothing from PX4,
   `px4_add_functional_gtest` for code that needs the parameter system or uORB
   (the CSV mixer and the allocator do). `DISCOVER` registers each `TEST` with
   ctest by name, so `ctest -R Group.Test` works. A header-only target that pulls
   in generated uORB headers also needs
   `add_dependencies(unit-DtrgX uorb_headers)`.
3. If the logic to test is buried in a module's `Run()`, lift it into a
   header-only helper first and include that from the module, with no behaviour
   change. That is how `bench_test_profile.h`, `bench_test_switch.h` and
   `dtrg_horizontal_thrust.hpp` came to be, and why they are testable at all.
4. Assert the premise as well as the result. The desaturation tests first check
   that the plain allocation really does push a rotor outside [0, 1], otherwise
   a test can pass without anything being desaturated.
5. A test that describes a bug you are not fixing yet goes in prefixed
   `DISABLED_`, with a comment naming the gap. ctest then reports it as "Not Run
   (Disabled)" and CI stays green; run it with
   `--gtest_also_run_disabled_tests`. Drop the prefix in the commit that fixes
   the bug. (No DTRG test is currently disabled.)

### Tier 2: a SIH logic test

Add to an existing `test/dtrg/test_*.py` or create one; `pytest.ini` collects the
whole directory. Give the module `pytestmark = pytest.mark.sih`, and mark a test
`fully_actuated` if it needs horizontal thrust without tilting (it is then
skipped on `sihsim_quadx`).

The `sitl` fixture boots a fresh PX4 with a clean rootfs per test:

```python
vehicle, px4 = sitl(params={"DTRG_HT_EN": 1, "DTRG_HT_MAX": 0.5},
                    rc={CH_HT_MODE: PWM_MAX, CH_PITCH: PWM_MAX},
                    wait_ready=True)
```

- `params`: set at boot through `PX4_PARAM_*`, on top of `BASE_PARAMS` and the
  airframe. The fixture **fails the test** if a parameter did not boot with the
  value asked for, so a typo cannot silently pass.
- `rc`: when given, the standard layout (`rc_layout.RC_PARAMS`) is applied and
  `RC_CHANNELS_OVERRIDE` is streamed at 50 Hz from before boot, starting from
  `rc_layout.initial_channels(rc)` — sticks centred, throttle low, Stabilized
  slot, switches off. Change a channel later with `vehicle.set_rc(ch, pwm)`.
- `wait_ready=False` when the test is *about* arming being refused.
- Everything is shut down and the artifacts collected at the end of the test.

Helpers: `vehicle.py` (arming, modes, parameters, status texts, RC override),
`px4_process.py` (`px4.listen("topic")` for a uORB topic, `px4-<cmd>` shell
commands), `ulog_checks.py` (read a topic back from the run's ULog),
`rc_layout.py` (the channel layout and the RC scaling / mode slot maths),
`airframes.py`, `flight.py` (tier 3).

Traps, all of which have cost someone an afternoon:

- PX4 only sends STATUSTEXT to a link that has seen a GCS heartbeat (`Vehicle`
  sends one at 1 Hz), and texts over 50 characters arrive in chunks (the helper
  reassembles them). Match by substring; they end in `\t`.
- `Preflight Fail: ...` is printed when the failure *set changes*, at most every
  2 s — not on every arm request. Take the `since` mark **before** causing the
  failure.
- RC override only becomes manual control with `RC_CHAN_CNT > 0`, and SITL
  defaults to `COM_RC_IN_MODE 1` (joystick); `rc_layout.RC_PARAMS` sets both.
- `RCx_DZ` is 10 us on channels 1-8 and 0 on 9-18, so half stick is 0.49, not
  0.5. Use `rc_layout.normalized(pwm, channel)` rather than dividing by 500.
- Channels 9-18 need MAVLink 2 (`MAVLINK20=1` before importing pymavlink).
- A mode switch is only acted on when it *changes* (or is first seen while
  disarmed): hold the intermediate position ~1 s.
- Three different numbers mean "bench test": HEARTBEAT main mode 11, RC slot
  value `COM_FLTMODEx = 16`, nav state 16.
- `param set-default` in the airframe script does not override `PX4_PARAM_*`,
  except when the requested value equals the firmware default — that does not
  count as a change. `init.d-posix/rcS` therefore applies `PX4_PARAM_*` a second
  time after the airframe script.

### Tier 3: a flight test

In `test_flight_ht.py`, marked `flight` and `fully_actuated`. Take off with
`flight.take_off()` (Takeoff mode to `MIS_TAKEOFF_ALT`, 3 m, waits until the
vehicle holds there), do the manoeuvre, then assert on what the ULog says.

- Mark the phases of the test with `vehicle.boot_time()`: that is PX4 time, the
  clock of the ULog.
- Assert how the vehicle *moved and tilted* on `flight.truth()`
  (`vehicle_*_groundtruth`), never on the estimator.
- Assert *holding* altitude or position on the estimate against its setpoint:
  with SIH's simulated baro and GPS the estimate wanders 0.3-0.5 m in altitude
  and up to ~1 m horizontally from the truth, which is estimator error rather
  than the behaviour under test.
- With RC streaming, `wait_mode(MAIN_STABILIZED)` before `take_off()`, or the RC
  slot being applied after boot overrides Takeoff mode.
- Write the control case too. "HT moves the vehicle level" only means something
  next to "without HT it tilts", which proves the tilt threshold would have
  caught a tilting vehicle.
- Tolerances: in a plain hover the planarOcto tilts ~3 deg (std) and up to ~9 deg
  while the position controller corrects the simulated GPS/IMU noise. Start
  loose, tighten after ~20 green CI runs.

Every test either passes or fails: no test is written to expect a failure. A
behaviour that is still broken is recorded in [section 6.1](#61-open-issues)
instead, and the test that would pin it is only added once the fix is in.

### Keeping this document true

A new test means a row in [section 5](#5-the-current-tests). A new gap means a
row in [section 6](#61-open-issues) plus the test that pins it; a fixed gap means
updating that row and removing the `DISABLED_` prefix from the test that covers it.

---

## 5. The current tests

### 5.1 Tier 1: unit tests

125 tests in 7 gtest groups (6 binaries). They exercise PX4 code directly: no
simulator, no airframe, no running PX4. Each is listed as `Group.Test`.

| Group | File | Code under test |
| --- | --- | --- |
| [`DtrgSequentialDesaturation`](#dtrgsequentialdesaturation) (18) | `src/lib/control_allocation/control_allocation/DtrgSequentialDesaturationTest.cpp` | `ControlAllocationSequentialDesaturation` |
| [`DtrgMixerCsv`](#dtrgmixercsv) (24) + [`DtrgMixerCsvLoad`](#dtrgmixercsvload) (8) | `src/lib/control_allocation/control_allocation/DtrgMixerCsvTest.cpp` | `ControlAllocationPseudoInverse::readMixerFromCSV()`, `loadCsvMixer()` |
| [`DtrgHorizontalThrust`](#dtrghorizontalthrust) (36) | `src/lib/dtrg_horizontal_thrust/DtrgHorizontalThrustTest.cpp` | `dtrg_horizontal_thrust.hpp` |
| [`DtrgHorizontalThrustSplit`](#dtrghorizontalthrustsplit) (7) | `src/lib/dtrg_horizontal_thrust/DtrgHorizontalThrustSplitTest.cpp` | `dtrg_horizontal_thrust.hpp` with `ControlMath` (the HT branch of mc_pos_control) |
| [`DtrgBenchSwitch`](#dtrgbenchswitch) (7) | `src/modules/bench_test/DtrgBenchSwitchTest.cpp` | `bench_test_switch.h` |
| [`DtrgBenchProfile`](#dtrgbenchprofile) (25) | `src/modules/bench_test/DtrgBenchProfileTest.cpp` | `bench_test_profile.h` |

#### DtrgSequentialDesaturation

When the rotors cannot deliver everything that is asked for, the allocator gives
up some axes to keep the others. The DTRG order is: thrust X, then thrust Y,
then yaw, then thrust Z (reduced only, never increased), and roll and pitch last.
In practice: the vehicle drops the sideways push before its attitude, and never
climbs to make room for a manoeuvre.

The vehicle is a planar octocopter built in the test, not read from an airframe:
8 rotors evenly spaced on a circle, each tilted tangentially by +-31 deg with
alternating direction and spin, like the DTRG planarOcto. The tilt is what lets
it push sideways without tilting the body. Values are normalised; "hover" is the
thrust Z that puts every rotor at 0.5. Airmode is off.

Each saturation test first checks its own premise — that the plain
pseudo-inverse allocation of the demand really does push a rotor outside [0, 1]
(or does not, for the reference case) — and then that desaturation leaves every
rotor within [0, 1], i.e. that nothing is still saturated.

| Test | Scenario | Passes when |
| --- | --- | --- |
| `GeometryIsFullyActuated` | At hover, +0.2 on one axis at a time (roll, pitch, yaw, X, Y) | Every case is delivered exactly on all 6 axes. Checks the test vehicle, not the allocator: without the rotor tilt, X and Y would come out as 0. Guards the tests below against a broken setup |
| `UnsaturatedDemandIsAllocatedExactly` | Small demand on all 6 axes at once, within the rotor limits | Delivered exactly: desaturation leaves a feasible demand alone |
| `HorizontalThrustIsReducedFirst` | Hover plus far more X thrust than the rotors can give | X is reduced but still forward; Z, Y, roll, pitch and yaw untouched: the vehicle keeps altitude and attitude and pushes as hard forward as it can |
| `HorizontalThrustIsGivenUpForRoll` | Roll 0.3 (feasible alone) plus an impossible X demand | Roll and Z are kept exactly, X is reduced |
| `SidewaysThrustIsGivenUpForYaw` | Yaw 0.1 (feasible alone) plus an impossible Y demand | Yaw and Z are kept exactly, Y is reduced but keeps its direction |
| `YawIsReducedBeforeThrust` | Rotors at 0.9 plus an impossible yaw demand | Thrust Z kept exactly, yaw reduced but keeps its direction, roll and pitch stay 0. Upstream PX4 instead gives up to 15 % thrust for yaw |
| `ThrustIsReducedBeforeRoll` | Rotors at 0.9 plus roll 0.6, which pushes the upper rotors past 1 | Roll kept exactly, thrust Z lowered, no rotor above 1 |
| `ThrustIsNeverIncreasedToDesaturate` | Rotors at 0.1 plus roll 1, which pushes the lower rotors below 0 | Thrust Z is not raised to make room (airmode off), roll is reduced instead |
| `RollAndPitchAreReducedButKeepTheirSign` | Rotors at 0.1 plus roll or pitch +-1 | Thrust Z kept, the attitude axis cut towards zero but keeping its sign |
| `UnsaturatedNegativeDemandIsAllocatedExactly` | Small negative demand on all axes, within the limits | Delivered exactly. Mirror of `UnsaturatedDemandIsAllocatedExactly`: a clamp with the sign wrong doubles negative demands even when nothing saturates |
| `ImpossibleDemandIsReducedButKeepsItsSign` | Hover plus +-5 on X, Y and yaw, one at a time | The demanded axis is cut but keeps its sign, every other axis is kept exactly |
| `OppositeHorizontalDemandsAreNotFlipped` | Hover plus X +5 and Y -5 | X may drop to 0 but never turns negative, Y is cut but never turns positive; Z, roll, pitch and yaw kept |
| `HorizontalThrustIsUsedToKeepRollAndPitch` | Rotors at 0.1 and 0.9 plus roll or pitch +-0.36, just past the rotor limits | Roll (or pitch) and thrust Z delivered exactly: X (for roll) or Y (for pitch) is moved even with no X/Y demand, to absorb the saturation. The other horizontal axis and yaw stay 0. By design |
| `YawIsUsedOnceHorizontalThrustRunsOut` | Rotors at 0.1 and 0.9, roll or pitch +-1 | X or Y is used (about -+0.017) but is not enough, so yaw is used too (about +-0.12). The other of roll and pitch is not added: the vehicle does not tilt sideways |
| `PublishesTopic` | One allocation at hover | The `sequential_desaturation` uORB topic is published, all six `*_sat` gains 0 |
| `TopicReportsHorizontalThrustReduction` | Impossible X demand at hover | `x_sat` > 0.01, roll and pitch near 0. Regression test: `desaturateActuators()` used to return only the gain of its second, half-strength pass, which is ~0 when the first pass already fixes the saturation |
| `TopicReportsOnlyTheAxesThatWereUsed` | Rotors at 0.9 plus roll 1 | `z_sat` > 0.01, and `x_sat` / `yaw_sat` equal the X and yaw added to keep roll; `y_sat` and `pitch_sat` are 0: axes left alone report no gain |
| `TopicGainIsTheAmountRemoved` | Hover plus X +-5 | `x_sat` has the opposite sign to the demand, and allocated X equals the demand plus `x_sat` |

#### DtrgMixerCsv

With `DTRG_MIXER_CSV` set, the allocator replaces the pseudo-inverse of the
effectiveness matrix with a matrix read from `/fs/microsd/etc/mixer.csv`: one row
per actuator, one column per axis (roll, pitch, yaw, thrust X, Y, Z). These tests
write temporary CSV files and check what the parser makes of them. Before each
test the matrix is filled with 99 ("untouched") and the file cells hold
`row + 0.1 * (column + 1)`, so a value in the wrong place is obvious.

| Test | File | Passes when |
| --- | --- | --- |
| `MissingFileIsRejectedAndMixerUntouched` | Does not exist | Rejected, the matrix left as it was |
| `ReadsOctoMatrix` | 8 rows of 6 values | All 48 values land in the right row and column |
| `RowsBeyondTheFileAreUntouched` | 8 rows | Rows 9 onwards keep their previous values: the parser does not clear them |
| `Utf8BomIsSkipped` | UTF-8 byte order mark first (Excel "CSV UTF-8") | Read as if the mark were absent |
| `CrlfLineEndings` | Windows line endings | Read correctly |
| `NoTrailingNewline` | Last row without a newline | Last row still read |
| `BlankLinesAreSkipped` | Blank lines before and between rows | Blank lines do not count as rows |
| `NegativeAndScientificValues` | `-0.5,1e-1,-2.5E-2,0,+1,-1` | Every notation parsed to the right value |
| `ExtraColumnsAreIgnored` | 8 values on a row | First 6 used, the extras do not spill into the next row |
| `ExtraRowsAreIgnored` | More rows than the allocator has actuators | Stops at the last actuator, no overflow |
| `FullPrecisionRowsAreNotSplit` | Full-precision export (MATLAB `writematrix`, ~20 characters per cell) | 2 rows read. Regression test: a row longer than the 100 byte line buffer was split in two and every following row shifted. The parser now reads one character at a time, so line length is unlimited |
| `OverlongCellIsRejected` | A 43 character cell | Rejected, the matrix left as it was: only one cell has to fit the parser's 32 byte buffer, and a longer one is not a number worth cutting short |
| `EmptyCellKeepsColumnPosition` | `1,,3,4,5,6` | 3 stays in column 3 and the empty cell reads as 0. Regression test: `strtok()` merged the two commas and shifted the row one column left |
| `EmptyFileIsRejected` | Empty | Rejected. Regression test: it was accepted, leaving the allocator without a mixer |
| `ShortRowIsRejected` | `1,2,3` | Rejected, the matrix left as it was. Regression test: it was accepted and the missing axes kept whatever the matrix held before |
| `BlankCrlfLineIsSkipped` | Blank Windows line before the data | Skipped. Regression test: it was read as a row of zeros, shifting the real rows down |
| `ResultOfValidFile` | 8 rows | Status `LOADED`, 8 rows, no error line |
| `ResultOfMissingFile` | Does not exist | Status `FILE_NOT_FOUND` |
| `ResultOfEmptyFile` | Only blank lines | Status `EMPTY` |
| `ShortRowReportsItsLine` | Blank line, a row, `1,2,3`, a row | Status `SHORT_ROW` on line 3: blank lines are counted, so the line matches what an editor shows |
| `HeaderRowIsRejected` | `roll,pitch,yaw,x,y,z` then 8 rows | Status `INVALID_VALUE` on line 1, matrix left as it was. A header used to read as a row of zeros |
| `NonNumericCellReportsItsLine` | `3x` on line 3 (CRLF) | Status `INVALID_VALUE` on line 3 |
| `NanAndInfAreRejected` | `nan` or `inf` | Rejected |
| `OverlongCellIsAnInvalidValue` | A 43 character cell | Status `INVALID_VALUE` on line 1 |

The reported reason and line are what the commander arming check turns into the
"Arming denied" message.

#### DtrgMixerCsvLoad

The whole allocator (`DTRG_MIXER_CSV=1`, `DTRG_MIXER_NORM=0`) on a quad X
geometry, with the file path pointed at a temporary file.

| Test | Setup | Passes when |
| --- | --- | --- |
| `ValidFileIsUsed` | 4 rows, 4 actuators | Status `LOADED`, the mixer holds the file values |
| `DisabledReportsDisabled` | `DTRG_MIXER_CSV=0` | Status `DISABLED`, the mixer is the pseudo-inverse |
| `MissingFileGivesEmptyMixer` | Pseudo-inverse first, then CSV on with no file | Status `FILE_NOT_FOUND`, the mixer is all zero (not the previous pseudo-inverse) |
| `InvalidFileGivesEmptyMixer` | Header row | Status `INVALID_VALUE` on line 1, all-zero mixer |
| `FewerRowsThanActuatorsIsRejected` | 4 rows, 8 actuators | Status `ROW_COUNT_MISMATCH` (4 rows), all-zero mixer. The missing rows used to keep a previous mixer |
| `MoreRowsThanActuatorsIsRejected` | 8 rows, 4 actuators | Status `ROW_COUNT_MISMATCH` (8 rows) |
| `AllZeroFileIsRejected` | 4 rows of zeros | Status `ALL_ZERO` |
| `FailedReloadKeepsLastValidMixer` | Valid file, then deleted, then fixed | Deleted: status `FILE_NOT_FOUND` but the last valid mixer is kept — the file is re-read whenever the effectiveness is updated (e.g. a parameter change in flight) and that must not cut the motors. Fixed: the new values are used |

#### DtrgHorizontalThrust

The shared HT helper that mc_att_control and mc_pos_control both include: RC
channel handling, the HT switch, the aux tilt channels, the `DTRG_HT_MASK` axis
selection, the `DTRG_HT_SPLIT` share between thrust and tilt, and the saturation
flags. Values are exact, which is why the limit flags are asserted here and not
in SITL, where `SYS_STATUS` only mirrors them while armed. The tilt limit in the tests is 0.1745 rad (10 deg).

| Test | Passes when |
| --- | --- |
| `ChannelIndexIsZeroBased` | `RC_MAP_*` 1, 8, 18 map to array indices 0, 7, 17 |
| `UnassignedChannelIsInvalid` | 0 and a negative channel give -1, which is not a valid index |
| `ChannelIndexRange` | 0 and 17 are valid, 18 and -1 are not: never read past the 18 channel array |
| `SwitchOnAboveThreshold` | HT is on above 0.5 (1.0, 0.51) and off at and below it (0.5, 0, -1): the threshold itself is off |
| `SwitchNeedsHtEnabled` | With `DTRG_HT_EN=0` a channel at full is still off |
| `SwitchOffWhenUnassigned` | An unassigned or out of range channel is off, even with every channel at full |
| `SwitchOffOnNan` | A NaN channel is off, not on |
| `AuxTiltScalesToLimit` | Aux 1.0 gives the full limit, -0.5 gives half of it the other way |
| `AuxTiltDeadzone` | +-0.02 is inside the deadzone and gives 0; 0.03 gives `0.03 * limit` — just outside the deadzone the raw value is used, not a rescaled one |
| `AuxTiltIsClamped` | A mis-calibrated channel at 1.5 or -3 gives exactly +-the limit |
| `AuxTiltDisabledChannel` | An unassigned or out of range channel gives 0 tilt |
| `AuxTiltNonFinite` | NaN and infinity give 0 tilt (level) |
| `MaskAxes` | Mask 0 uses X and Y, mask 1 X only, mask 2 Y only |
| `SelectableMask` | 0, 1, 2 pass through; out of range (3, 7, -1 — including the old mask 3) falls back to 0 |
| `MaskZeroUsesBothAxes` | Both demands are delivered, neither flagged saturated |
| `MaskOneIsXOnly` | X delivered, Y forced to 0 |
| `MaskTwoIsYOnly` | Y delivered, X forced to 0 |
| `SplitShares` | Thrust and tilt shares add to 1: split 0.5 gives 0.5/0.5, 0.25 gives 0.25/0.75, 0 is tilt only, 1 is horizontal thrust only |
| `SplitDisabledIsHorizontalThrustOnly` | With `DTRG_HT_SPLIT_EN=0` the HT axes are thrust only whatever `DTRG_HT_SPLIT` says |
| `SplitIsClamped` | 1.5 clamps to 1, -0.5 to 0, and NaN falls back to the default split |
| `TiltThrustScalesHtAxesOnly` | At yaw 0 the tilt-producing thrust is scaled on the HT axes only: mask 0 scales both, mask 1 leaves the east component at full (Y still moves by rolling), mask 2 leaves north |
| `TiltThrustFollowsHeading` | At yaw 90 deg, mask 1 scales the component along the heading (east), not the north one: the mask is in body axes |
| `TiltThrustSplitZeroIsUnchanged` | Split 0 leaves the thrust untouched: the vehicle tilts for all of it, as without HT |
| `ThrustIsClampedAndFlaggedSaturated` | Demands of 0.8 / -0.9 against a 0.5 limit come out +-0.5 with both saturation flags set |
| `ExactlyAtLimitIsSaturated` | A demand exactly at the limit is flagged, one just below is not |
| `UnusedAxisIsNeverSaturated` | Mask 1 ignores a Y demand of 10 entirely: Y is 0 and not flagged |
| `StickToThrustAsInManualMode` | `stick * DTRG_HT_MAX` as mc_att_control feeds it: full stick lands exactly on the limit (flagged), half stick at half of it (not flagged) |
| `PositionControlTiltPerMask` | Without the split, the tilt of an HT axis comes from HT (aux / offboard) and the other axis from the position controller: mask 0 both from HT, mask 1 pitch from HT and roll from the controller, mask 2 the other way round |
| `PositionControlTiltWithSplit` | With the split enabled, both axes tilt with the (scaled) controller for every mask |
| `UnknownMaskBehavesAsFullHt` | An out of range mask never tilts with the controller and keeps both thrust axes: an invalid parameter cannot turn HT into something else |
| `ManualModeWithoutSplitHtAxesAreThrustOnly` | Every mask, three stick pairs, `DTRG_HT_SPLIT_EN=0`: an HT axis gets `stick * DTRG_HT_MAX` of thrust and the aux channel's tilt (`DTRG_HT_SPLIT` ignored); the other axis gets no thrust and `stick * MPC_MAN_TILT_MAX` of tilt |
| `ManualModeSplitDividesTheStick` | Every mask x split 0, 0.25, 0.5, 0.8, 1 x three stick pairs: on an HT axis the thrust fraction (of `DTRG_HT_MAX`) is `split * stick` and the tilt fraction (of `MPC_MAN_TILT_MAX`) `(1 - split) * stick`, adding up to the stick; the other axis has no thrust and tilts for all of the stick. The aux channels are not used |
| `ManualModeSplitZeroIsNoHorizontalThrust` | `DTRG_HT_SPLIT_EN=1`, `DTRG_HT_SPLIT=0` (the old mask 3), every mask: no X/Y thrust, not flagged, and the stick tilt is unchanged: standard Manual Mode |
| `ManualModeSplitOneIsLevel` | Split 1, full sticks: X/Y at `DTRG_HT_MAX` (flagged) and level, the aux channels still unused |
| `ManualModeSplitNeverSaturates` | Split 0.8, full sticks: 0.8 of `DTRG_HT_MAX`, not flagged |
| `ManualModeSplitOutOfRange` | Split 1.5 behaves as 1, -0.5 as 0, NaN as the default 0.5, for the thrust and the tilt alike |

#### DtrgHorizontalThrustSplit

How Position and Offboard divide the position controller's horizontal thrust
between horizontal thrust and tilt. There the split is not one multiplication:
mc_pos_control tilts for `(1 - split)` of the thrust (`tiltThrust()`), then
rotates the *full* thrust into the tilted body frame, and what is left on body
X/Y becomes horizontal thrust. The test runs that branch of
`MulticopterPositionControl::Run()` step by step with the real `ControlMath`
(linked from `PositionControl`) and checks the resulting forces in the heading
frame. Demand: 0.08 forward, 0.05 left, 0.5 up (about 10 deg of tilt), at yaw 0,
0.7, 90 deg and -2.5 rad; tolerance 0.002 (second order effects of the tilt).
mc_pos_control itself is not compiled in; the tier 2/3 tests cover its wiring.

| Test | Passes when |
| --- | --- |
| `SplitDividesTheDemand` | Every mask x split 0, 0.25, 0.5, 0.8, 1 x four yaws: on an HT axis horizontal thrust gives `split` of the demand and tilting the rest; the other axis is moved by tilting only; together they deliver the whole demand, vertical included. The mask is in heading (body) axes at any yaw |
| `SplitKeepsTheHtAxesFromTheAuxTilt` | With the split an aux / offboard tilt changes neither attitude nor thrust |
| `WithoutSplitHtAxesAreThrustOnly` | `DTRG_HT_SPLIT_EN=0`, every mask and yaw: an HT axis stays level and gets the whole demand as horizontal thrust; the other axis tilts for all of it |
| `SplitZeroIsNoHorizontalThrust` | Split 0 (the old mask 3), every mask and yaw: no X/Y thrust, and attitude and collective thrust equal the standard position controller's |
| `SplitOneIsLevelOnTheHtAxes` | Split 1: the HT axes stay level and get the whole demand |
| `ThrustShareIsLimitedButTiltShareIsNot` | Split 0.8 of 0.7 forward: horizontal thrust is clipped at `DTRG_HT_MAX` and flagged, the tilt is still exactly that for the other 0.2 |
| `SplitIsExactOnlyForSmallTilts` | Pins [section 6.1](#61-open-issues): split 0.5 of 1.2 forward over a 0.5 hover tilts 50 deg and leaves 0.38 of body X thrust rather than 0.6, while the total force is still exactly the demand |

#### DtrgBenchSwitch

The bench test direction comes from a 3-position switch on the channel set by
`RC_MAP_CMD_SIGN`: above 1700 us is +1, below 1300 us is -1, anything else is
centre (0). Commander uses the same decoding for its "centre the switch" arming
check, so a bug here affects both the excitation and the arming gate. Every
doubtful case must decode as centre, the safe position.

| Test | Input | Passes when |
| --- | --- | --- |
| `PulseThresholds` | 1, 1000, 1299, 1300, 1500, 1700, 1701, 2000 us | 1701 and up is +1, 1299 and down is -1, 1300 to 1700 inclusive is centre: the thresholds themselves are centre |
| `ZeroPulseIsCentre` | 0 us (unpopulated channel) | Centre, not "switch down" |
| `InputRcUsesOneBasedChannel` | Channel 6 high, channel 7 low | `RC_MAP_CMD_SIGN = 6` reads `values[5]`: channel numbers start at 1 |
| `DisabledChannelIsCentre` | `RC_MAP_CMD_SIGN` 0 or negative, every channel high | Centre |
| `ChannelBeyondChannelCountIsCentre` | 8 channels received, switch on channel 8, 9 or 19 | Channel 8 is read, a channel the receiver does not send is centre |
| `ChannelBeyondArrayIsCentreEvenIfCountClaimsMore` | Corrupt `channel_count` of 255 | Never reads past the 18 channel array |
| `RcLossIsCentre` | Switch high with RC lost, switch low with RC failsafe | Centre: a stale value is not used |

#### DtrgBenchProfile

The maths of `BenchTest.cpp`: hold a hover thrust and excite one axis with a step
or a ramp, in the direction of the switch above. Each group maps to the
parameters that drive it.

**Step** (`BT_STEP_DELAY`, `BT_STEP_DUR`, `BT_STEP_MAG`); tests use 2 s delay, 1 s
duration, magnitude 0.1.

| Test | Passes when |
| --- | --- |
| `StepIsZeroBeforeDelay` | Output is 0 up to the end of the delay |
| `StepIsActiveForDuration` | Output is 0.1 from 2 s up to (not including) 3 s, then 0 for good |
| `StepFollowsSign` | Switch down gives -0.1, centre gives 0 |
| `StepWithoutDelayStartsImmediately` | With no delay the step is on at t = 0 |

**Ramp** (`BT_RAMP_RATE`, `BT_MAX_VAL`); tests use 0.1 per second, clamp 0.5.

| Test | Passes when |
| --- | --- |
| `RampIncreasesLinearly` | 0, 0.1, 0.2 at 0, 1, 2 s, not frozen |
| `RampNegativeSign` | Switch down gives -0.2 at 2 s |
| `RampStopsAtMaxValAndFreezes` | Clamped to 0.5 and frozen, still 0.5 later |
| `RampNegativeStopsAtMinusMaxVal` | Clamped to -0.5 and frozen |
| `RampFreezesOnMotorSaturation` | Freezes at the value where a motor saturated and holds it, even after the motor leaves saturation |
| `RampResetClearsFreeze` | After a reset the ramp is unfrozen and starts again from 0 |

**Spin-up** (`BT_SPINUP_T`): the hover thrust is soft-started when the outputs
come on.

| Test | Passes when |
| --- | --- |
| `SpinupRampsToOne` | Over 2 s the factor goes 0, 0.5, 1, and stays 1 |
| `SpinupDisabled` | A spin-up time of 0 or less gives 1 straight away |

**Axis mapping** (`BT_AXIS`, `BT_HOVER_THR`): how the hover thrust and the
excitation become thrust and torque setpoints. Body frame, NED, so -Z is up.

| Test | Passes when |
| --- | --- |
| `HoverBaselineIsUpwardThrustOnly` | Hover 0.4 at half spin-up, no excitation: thrust Z is -0.2 and everything else 0 |
| `EachAxisDrivesOnlyItsSetpoint` | X, Y, roll, pitch and yaw each drive only their own setpoint; thrust Z keeps the hover baseline |
| `ThrustZExcitationAddsUpwardThrust` | Excitation on Z adds lift: hover 0.2 plus 0.1 gives thrust Z -0.3 |
| `ExcitationIsNotScaledBySpinup` | At spin-up 0 the hover thrust is 0 but the excitation is applied in full: only the hover baseline is soft-started |

**Motor saturation** (`BT_NUM_MOTORS`, `BT_SAT_MARGIN`): decides when the ramp
freezes. Tests use a margin of 0.05.

| Test | Passes when |
| --- | --- |
| `CountFiniteMotors` | Counts the outputs that are not NaN (0, 4, 8). Used when `BT_NUM_MOTORS` is 0 (auto-detect) |
| `MidRangeIsNotSaturated` | All motors at 0.5: not saturated |
| `UpperMarginSaturates` | One motor at 0.95 is saturated, at 0.94 it is not |
| `LowerMarginSaturates` | One motor at 0.05 is saturated, at 0.06 it is not |
| `ReversibleMotorUsesSymmetricRange` | A reversible motor spans [-1, 1]: 0 is mid-range, -0.95 is saturated |
| `UnconnectedMotorsAreIgnored` | Motor 5 at full output is ignored with 4 motors, counted with 5 |
| `NonFiniteOutputsAreIgnored` | A NaN output does not count as saturated |
| `MarginIsClamped` | A margin above 0.5 behaves as 0.5, a negative margin as 0 (only exactly 0 or 1 counts) |
| `MotorCountAboveArrayIsBounded` | A motor count of 1000 never reads past the output array |

### 5.2 Tier 2: SIH logic tests

35 tests in `test/dtrg/`. Every one boots a fresh PX4 with a clean rootfs
(10-25 s), sets its parameters at boot, and is shut down again. They check
decisions and setpoints — arming accepted or denied, the mode, status texts, the
published setpoints — not how the vehicle moves: nothing takes off here.

Tests that need RC stream `RC_CHANNELS_OVERRIDE` at 50 Hz with one fixed layout
(`test/dtrg/rc_layout.py`):

| Channel | Function |
| --- | --- |
| 1-4 | roll, pitch, throttle, yaw |
| 5 | flight mode switch: slot 1 Stabilized, slot 4 Position, slot 6 Bench test |
| 6 | bench test direction switch (`RC_MAP_CMD_SIGN`) |
| 8 | horizontal thrust on/off (`RC_MAP_HT_MODE`) |
| 9, 10 | horizontal thrust aux roll / pitch tilt (`RC_MAP_HT_ROLL`, `RC_MAP_HT_PITCH`) |

RC starts safe: sticks centred, throttle low, Stabilized slot, switches off.

#### test_smoke.py

Checks the harness itself before anything else is blamed.

| Test | Passes when |
| --- | --- |
| `test_boots_dtrg_firmware_and_is_ready_to_arm` | PX4 boots, `SYS_STATUS.errors_count4` is 706 (the DTRG firmware marker, so this is not an upstream build) and the arming checks pass |
| `test_rc_override_drives_the_flight_mode_switch` | With RC streamed, the mode switch puts the vehicle in Stabilized and `COM_RC_IN_MODE` is 0 (RC only): RC override really reaches the mode logic |

#### test_rc_conflict.py

Code under test: commander `rcChannelConflictCheck`. Two `RC_MAP_*` functions on
the same raw channel would make one switch do two things (e.g. the throttle stick
also toggling HT). Commander must refuse to arm, name both parameters, and allow
arming again once the overlap is gone; `COM_ARM_RC_CONF=1` turns the failure into
a warning. The conflict used is `RC_MAP_HT_MODE` moved onto the throttle channel
(3). No RC is streamed: the check only reads the configuration, and
`COM_RC_IN_MODE=1` keeps a missing RC link from failing arming for another reason.

| Test | Scenario | Passes when |
| --- | --- | --- |
| `test_conflict_blocks_arming_and_names_both_parameters` | Conflict created at runtime | Arming refused, `Preflight Fail: RC_MAP_HT_MODE and RC_MAP_THROTTLE both use RC channel 3` sent, vehicle stays disarmed |
| `test_resolving_conflict_allows_arming_again` | Conflict, then `RC_MAP_HT_MODE` moved back to channel 8 | `RC channel conflict resolved` sent, the arming checks pass and the vehicle arms |
| `test_conflict_configured_before_boot_blocks_arming` | Conflict already in the parameters at boot | The failure is printed on the console at boot; with `COM_ARM_RC_CONF=1` arming is allowed, back to 0 it is refused again. Checks the boot-time scan of the parameter table, not only the change handler |
| `test_com_arm_rc_conf_warns_without_blocking` | `COM_ARM_RC_CONF=1`, conflict created | `RC ch 3: ...` warning sent and the vehicle still arms |
| `test_failsafe_channel_may_share_throttle` | `RC_MAP_FAILSAFE` on the throttle channel | Not a conflict: the failsafe channel is documented to sit on throttle |
| `test_flight_mode_buttons_only_count_without_mode_switch` | `RC_MAP_FLTM_BTN` includes channel 8 (the HT switch) | Not a conflict while `RC_MAP_FLTMODE` is set (the buttons are ignored then); a conflict once `RC_MAP_FLTMODE` is 0 |

#### test_bench_test_safety.py

Code under test: commander's mode and arming rules for bench test. Bench test
drives the motors with every control loop off, so it is fenced in: only reachable
from an RC mode slot, never over MAVLink; never entered while armed; only armed
with `BT_ARM_ENABLE=1` and the direction switch centred; and it may always be
disarmed. Tests use a hover thrust of 0.2, far below what lifts the vehicle,
since SIH does not tie it down like a real rig.

| Test | Passes when |
| --- | --- |
| `test_rc_slot_selects_bench_test_while_disarmed` | Moving the mode switch to slot 6 while disarmed enters bench test (HEARTBEAT main mode 11). Control case for the two tests below |
| `test_bench_test_rejected_while_armed` | Armed in Stabilized, switching to slot 6 gives `Bench test mode denied: disarm first`; the vehicle stays armed in Stabilized. Once disarmed, the same switch (moved away and back) enters bench test |
| `test_mavlink_cannot_select_bench_test` | `DO_SET_MODE` to main mode 11 leaves the vehicle in Stabilized. Commander ACKs it as accepted (an unknown custom mode is a no-op), so the test checks the mode, not the ACK |
| `test_arming_needs_bt_arm_enable` | In bench test with `BT_ARM_ENABLE=0`, arming is refused with `Arming denied: bench test not enabled`; after setting it to 1 the same request arms |
| `test_arming_needs_centred_direction_switch[switch_up/switch_down]` | Direction switch up or down: arming refused with `Arming denied: centre the bench test direction switch`; once centred it arms. Otherwise the excitation would start the moment the motors spin up |
| `test_off_centre_switch_allowed_when_it_cannot_excite[hover_only_profile/no_switch_assigned]` | With `BT_MODE=0` (hover only) or no switch assigned (`RC_MAP_CMD_SIGN=0`), an off-centre switch cannot excite anything, so arming is allowed |
| `test_disarm_allowed_in_bench_test` | Armed in bench test, disarm is honoured. On a real rig the land detector reports "in air" once the motors spin and bench test allows disarming anyway; SIH stays landed, so this only checks a normal disarm (the in-air case needs a rig) |
| `test_rc_layout_has_no_conflicts` | No two functions in `rc_layout.RC_PARAMS` share a channel. Guards every RC test against failing on the conflict check instead of what it tests. Does not boot PX4 |

#### test_bench_test_outputs.py

Code under test: the `bench_test` module wiring (the maths is unit tested in
[DtrgBenchProfile](#dtrgbenchprofile)). Each test arms in bench test, waits for
the spin-up, moves the direction switch for a while, centres it, disarms, then
reads `vehicle_thrust_setpoint` and `vehicle_torque_setpoint` back from the ULog.
The excited axis is thrust Z (`BT_AXIS=2`), hover 0.2, spin-up 1 s. Setpoints are
NED body frame: -Z is up.

| Test | Profile | Passes when |
| --- | --- | --- |
| `test_step_profile[up/down]` | Step: delay 1 s, magnitude 0.1, duration 1 s | Hover baseline -0.2 reached after the spin-up; exactly one step of 1 s (+-0.1 s) to -0.3 (switch up) or -0.1 (switch down); back to -0.2 afterwards; thrust X/Y and all torques stay 0; thrust Z is 0 whenever bench test is disarmed |
| `test_spinup_ramps_hover_thrust` | Hover only | Thrust Z starts near 0 when arming, never jumps, and follows a linear ramp to -0.2 over `BT_SPINUP_T` (+-0.03) |
| `test_ramp_profile_stops_at_max_value` | Ramp: 0.1 per second, `BT_MAX_VAL` 0.05 | The excitation never exceeds 0.05, is held there for over 1 s, and drops back to 0 once the switch is centred |

#### test_horizontal_thrust.py

Code under test: mc_att_control HT in Stabilized. With the HT switch on, the roll
and pitch sticks command body X/Y thrust instead of attitude and the aux channels
command a tilt. These check the wiring from RC to the `vehicle_attitude_setpoint`
mc_att_control publishes (`thrust_body`, and roll / pitch out of `q_d`), disarmed,
1.5 s after each RC change. Parameters: `DTRG_HT_MAX` 0.5, `DTRG_HT_R_MAX` and
`DTRG_HT_P_MAX` 10 deg. "Half stick" is 1750 us, which the RC deadzone turns
into 0.49.

| Test | RC | Passes when |
| --- | --- | --- |
| `test_switch_off_is_standard_stabilized` | HT off, full pitch stick, aux roll full | No X/Y thrust, pitch below -5 deg (normal nose down), aux roll does nothing, `horizontal_thrust_limit` not published |
| `test_switch_on_sticks_command_thrust_and_vehicle_stays_level` | HT on, full pitch stick, half roll stick | Thrust X = 0.5 (`DTRG_HT_MAX`), Y = 0.49 x 0.5, roll and pitch 0 (+-0.5 deg), `horizontal_thrust_limit` published |
| `test_switch_toggles_ht_at_runtime` | Full pitch stick, HT switch off, on, off | The setpoint follows each change: tilt, then level with X thrust, then tilt again |
| `test_switch_ignored_when_ht_disabled` | `DTRG_HT_EN=0`, HT on, full pitch stick | The switch is ignored: no X thrust, normal tilt, `horizontal_thrust_limit` not published |
| `test_aux_channels_command_tilt_up_to_limit` | HT on, aux roll full, then aux pitch full | Roll +10 deg with pitch 0, then pitch +10 deg (nose up — the aux channels set the tilt directly, so the opposite way round to the pitch stick) with roll 0 |
| `test_aux_channel_deadzone[1505/1515]` | HT on, aux roll at 1505 or 1515 us | 1505 us (0.01) is inside the 0.02 aux deadzone: roll 0. 1515 us (0.03) tilts by 0.03 x 10 deg |
| `test_mask_selects_thrust_axes[mask0/1/2]` | `DTRG_HT_MASK` 0-2, HT on, full pitch, half roll | 0: X and Y by thrust; 1: X only; 2: Y only |
| `test_mask_and_split_divide_stick_between_thrust_and_tilt[...]` | Pitch stick 0.49, roll stick 0.23, aux roll +0.5 and aux pitch -0.4; `DTRG_HT_MASK` 0-2 with the split off, mask 0 with split 0, 0.25 and 1, masks 1 and 2 with split 0.25 | Thrust X/Y (+-0.001) and roll/pitch of the attitude setpoint (+-0.3 deg) as mc_att_control should compute them: on an HT axis `split` of `DTRG_HT_MAX` as thrust and `(1 - split)` of `MPC_MAN_TILT_MAX` as tilt, or all thrust and the aux tilt with the split off; the other axis tilts with the stick and has no thrust. The expected attitude is built as the same axis-angle rotation. Split 0 is the old mask 3 (standard Stabilized), split 1 is level |
| `test_full_stick_gives_ht_max` | HT on, full pitch stick | Thrust X = 0.5 (`DTRG_HT_MAX`, +-0.001) and `y_sat` 0. The demand is `stick * DTRG_HT_MAX`, never clipped, so `x_sat` sits exactly on the limit: it sets on Linux (X = 0.5) but not on macOS (0.99999988 of it), so it is not asserted here, nor is the `SYS_STATUS.errors_count3` mirror, which only carries the flags while armed. The flags are covered by `DtrgHorizontalThrustTest.cpp`, which feeds exact values |

#### test_csv_mixer.py

Code under test: `dtrg_mixer_status` from control_allocator and the commander
arming check `dtrgMixerCheck` (the parser and the other rejection reasons are
unit tested in [DtrgMixerCsv](#dtrgmixercsv)). The mixer file path is hardcoded
to `/fs/microsd/etc/mixer.csv`, which does not exist in SITL, so only the "no
file" case can run here.

| Test | Passes when |
| --- | --- |
| `test_csv_mixer_without_file_refuses_to_arm` | `DTRG_MIXER_CSV=1`, no file: `dtrg_mixer_status` reports `FILE_NOT_FOUND`, prearm fails, and two arm attempts are both refused with "Arming denied: DTRG mixer file not found" (the reason is given on every attempt, not only when the failure first appears). Regression test: the allocator kept an all-zero mixer and nothing stopped arming |
| `test_csv_mixer_disabled_does_not_block_arming` | `DTRG_MIXER_CSV=0`: status `DISABLED`, arms |

### 5.3 Tier 3: SIH flight tests

16 tests in `test/dtrg/test_flight_ht.py`, helpers in `test/dtrg/flight.py`; about
14 flights, ~14 minutes. Same harness, RC layout and conventions as tier 2, but
here the planarOcto really flies. All are marked `flight` and `fully_actuated`,
so they are skipped on `--airframe=sihsim_quadx`.

Each test takes off in Takeoff mode to 3 m (`MIS_TAKEOFF_ALT`), waits until the
vehicle holds there, does its manoeuvre, and then reads two things back from the
ULog: **SIH ground truth** (`vehicle_*_groundtruth`) for how the vehicle really
moved and tilted, and **the estimate against its setpoint** for "holds altitude /
position" (called drift below). HT parameters: `DTRG_HT_MAX` 0.5, `DTRG_HT_R_MAX`
and `DTRG_HT_P_MAX` 10 deg. "Tilt" is max(|roll|, |pitch|).

Code under test: mc_pos_control and mc_att_control horizontal thrust, and
commander, in flight.

| Test | Manoeuvre | Passes when |
|  --- | --- | --- |
| `test_take_off_hold_and_land` | Take off, hold 10 s, Land | Altitude within 0.5 m of its setpoint and drift < 1 m during the hold; lands and disarms within 30 s. Baseline: if this fails, nothing below means anything |
| `test_ht_moves_the_vehicle_level` | HT on, Offboard position setpoint 5 m north | Moves 5 m (+-0.5) north and the true tilt stays below 3 deg throughout: HT moves the vehicle without tilting it |
| `test_offboard_split_divides_thrust[mask0-split0.25/mask0-split0.75/mask1-split0.25/mask2-split0.75]` | `DTRG_HT_SPLIT_EN` 1, HT on, Offboard move 4 m forward and 4 m right of the heading | Moves the 5.7 m diagonal (true distance along it +-0.6; each axis settles up to ~0.5 m short from the HT gain loss), and of the force in the published attitude setpoint (samples asking for more than 0.02 on that axis), horizontal thrust gives `DTRG_HT_SPLIT` on an HT axis and nothing on the other (median, +-0.05), tilting the rest. Measured within 0.01 of 0.25 / 0.75 on HT axes and of 0 on the tilt-only axis for every mask. On the setpoint, so the allocator gain loss ([section 6.1](#61-open-issues)) does not enter |
| `test_without_ht_the_vehicle_tilts_to_move` | Same move with HT off | Moves 5 m (+-0.5) and pitches nose down past -5 deg. Control case for `test_ht_moves_the_vehicle_level`: shows its tilt limit would catch a vehicle that moves by tilting |
| `test_aux_tilt_reaches_the_limit[roll]` | HT on, Hold, aux roll full | The attitude half of the aux tilt check, independent of the horizontal thrust gain loss ([section 6.1](#61-open-issues)): roll +10 deg (estimate +-2, truth +-3.5), pitch below 3 deg |
| `test_aux_tilt_reaches_the_limit[pitch]` | HT on, Hold, aux pitch full | pitch +10 deg nose up (estimate +-2, truth +-3.5), roll below 3 deg. The aux channels set the tilt directly, so pitch is the opposite way round to the pitch stick, and Hold agrees with Stabilized |
| `test_offboard_tilt_setpoint_in_hover` | HT on, Offboard hold, `DEBUG_FLOAT_ARRAY` roll 0.1 rad (5.7 deg) at 10 Hz for 8 s | Estimated roll 5.7 deg (+-2), true roll +-3.5 (the estimate drifts ~2 deg from the truth while tilted), drift < 0.5 m, still in Offboard |
| `test_offboard_tilt_is_limited` | Same, roll setpoint twice `DTRG_HT_R_MAX` for 5 s, then NaN roll and pitch for 5 s | Estimated roll at `DTRG_HT_R_MAX` (+-2 deg) and true roll below the limit + 3.5 deg, not the doubled setpoint; after the NaN the vehicle is level (estimated tilt < 2 deg, true tilt < 3.5 deg) and still in Offboard. Pins the offboard tilt limit fix |
| `test_toggling_ht_in_hover_is_smooth` | Hold 8 s as a reference, then HT switch on / off 5 times, 2 s each | Altitude within 0.7 m of its setpoint, drift < 1 m, and tilt below max(8 deg, the reference hover's tilt + 2 deg): switching HT does not kick the vehicle |
| `test_stabilized_ht_stick_moves_the_vehicle_level` | Take off to 6 m, HT on, Stabilized, full pitch stick for 3 s | Forward speed (true, along the heading) above 1 m/s and tilt below 3 deg. The manual counterpart of `test_ht_moves_the_vehicle_level` |
| `test_bench_test_rejected_in_flight` | Hovering in Hold, mode switch to the bench test slot | `Bench test mode denied: disarm first`; still armed and in Hold, and stays above 1.5 m. The in-air case of `test_bench_test_rejected_while_armed` |


Holding position and altitude is judged on the estimate against the setpoint;
tilt and distance moved on the ground truth. The tolerances are first guesses;
tighten them after ~20 green CI runs. In a plain hover the vehicle tilts ~3 deg
(std) and up to ~9 deg: that is the position controller correcting the simulated
GPS/IMU noise with HT off.

---

## 6. Notes

### 6.1 Open Issues

| Gap | Tracked by |
|  --- | --- |
| **Open, by design.** In Stabilized, HT X/Y is `stick * DTRG_HT_MAX`: full stick gives exactly the limit and the demand is never clipped, so `horizontal_thrust_limit.x_sat` (`abs(X) >= limit`) sits on the boundary and depends on whether the RC scaling lands on exactly 1.0 — it does on Linux but not on macOS — so the SITL test does not assert it. `x_sat` means something in Position/Hold/Offboard, where mc_pos_control clips its demand. Separately, `SYS_STATUS.errors_count3` only carries the flags while armed (`streams/SYS_STATUS.hpp`), so the disarmed tier 2 tests cannot cover that mirror at all | `test_full_stick_gives_ht_max`; the flags in `DtrgHorizontalThrustTest.cpp` |
| **Open.** `BT_SAT_MARGIN` allows 0.5, at which any standard motor output reads as saturated and the ramp freezes at 0 | documented in `DtrgBenchProfileTest.cpp` (`MarginIsClamped`) |
| **Open.** `DTRG_OFFBOARD` (MAVLink 9003) is received into `dtrg_custom` but nothing reads it; `streams/DTRG_OFFBOARD.hpp` does not compile and is not registered | not covered |
| **Open.** HT delivers 41 % of the horizontal force the position controller asks for on the planarOcto. mc_pos_control writes its NED thrust (normalised to full collective thrust) into `thrust_body[0/1]`, but the allocator normalises each thrust axis on its own (`ControlAllocationPseudoInverse::updateControlAllocationMatrixScale`), and on this geometry X/Y = 1 is 0.41 of full thrust while Z = 1 is all of it. In a hover with an HT tilt the position controller cannot hold position: at 10 deg roll it needs 0.25 body Y thrust for a physical 0.10, which `MPC_TILTMAX_AIR 10` caps below, so the vehicle slides ~0.5 m/s (and Hold then yaws towards its setpoint). With `MPC_TILTMAX_AIR 30` it holds, 0.55 m off. Moving level (`test_ht_moves_the_vehicle_level`) works because the integrators absorb the gain loss. Stabilized HT is scaled the same way |  |
| **By design.** In Position/Offboard the split is exact only for small tilts: mc_pos_control tilts for `(1 - DTRG_HT_SPLIT)` of the thrust and rotates the full thrust into that tilted body, so the horizontal thrust share shrinks as the tilt grows (split 0.5 at a 50 deg tilt gives 0.32 of the demand instead of 0.5; within 0.002 at ~10 deg). The total force is always the demand, only the division changes. Stabilized is exact | `SplitIsExactOnlyForSmallTilts` |
| **By Design.** Desaturation can demand more input in horizontal force to desaturate roll/pitch |  |


### 6.2 Not covered yet

- `DTRG_HT_SPLIT` in flight in Stabilized (tier 1 and tier 2 cover every
  mask x split; tier 3 flies every mask with a split in Offboard only).
- The allocator giving up X/Y before roll/pitch in flight (needs the allocator's
  per-axis output in the log).
- Bench test's in-air disarm exception needs a vehicle that reports "in air" on a
  bench; SIH stays landed.
- Horizontal thrust mode flight with pitch/roll command flight test need to be developed better

### 6.3 Facts the tests rely on

Re-check these if tests start failing for no obvious reason.

1. HT and bench test read `rc_channels` / `input_rc`, which in SITL only
   `RC_CHANNELS_OVERRIDE` feeds (`MANUAL_CONTROL` does not reach them).
2. SITL defaults to `COM_RC_IN_MODE 1` (joystick). RC tests set `0` and
   `RC_CHAN_CNT 18`, and stream RC for the whole test.
3. Bench test is only reachable from an RC slot (`COM_FLTMODEx = 16`), and a slot
   is only acted on when it changes (or is first seen while disarmed).
4. HT is on when the `RC_MAP_HT_MODE` channel is `> 0.5`. The bench test
   direction switch is `>1700` up, `<1300` down (raw us).
5. Channels 9-18 need MAVLink 2 (`MAVLINK20=1` before importing pymavlink).
6. Status texts to assert on (match by substring, they end in `\t`):
   `Bench test mode denied: disarm first`,
   `Arming denied: bench test not enabled, set BT_ARM_ENABLE=1`,
   `Arming denied: centre the bench test direction switch`,
   `Arming denied: DTRG mixer file not found`,
   `Preflight Fail: <A> and <B> both use RC channel <N>`,
   `RC ch <N>: <A> and <B>`, `RC channel conflict resolved`.
7. DTRG telemetry in `SYS_STATUS`: `errors_count1` desaturation gains,
   `errors_count2` motors > 0.9, `errors_count3` HT `x_sat` bit 0 / `y_sat` bit 2
   (armed only), `errors_count4 == 706` (the DTRG firmware marker).
8. `SYS_STATUS.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK` is
   "can arm in the current mode"; the tests use it as the readiness signal.
9. Parameters set through `PX4_PARAM_<NAME>` are applied before the airframe
   script. The airframe's `param set-default` does not override them, except when
   the value set equals the firmware default: that does not count as a change, so
   the airframe's default would win (e.g. `DTRG_HT_EN=0` on the planarOcto, whose
   airframe defaults it to 1). `init.d-posix/rcS` therefore applies `PX4_PARAM_*`
   a second time after the airframe script, and the `sitl` fixture fails a test
   whose parameters did not boot with the requested values.

### 6.4 Local and CI quirks

- About 3 % of SITL boots hang on macOS: a `px4-<module>` client command in rcS
  never gets its reply from the PX4 server (2 of 60 boots, with and without the
  `drv_hrt.cpp` scheduling fix). The `sitl` fixture detects it, warns, and boots again (up to twice).
- `make tests` fails the `sitl-*` tests if another SITL is running (instance 0 is
  taken). Use `--px4-instance=N` to run the pytest tiers next to another SITL.
- On macOS `PurePursuit` and `gps_blending` fail on float rounding in the full
  `make tests`; neither involves DTRG code. That is why CI's full-suite step is
  non-blocking.
- `make tests TESTFILTER=...` takes a ctest regex, but `|` breaks the Makefile's
  shell quoting: run one filter at a time.
- Earlier breakage worth remembering: `unit-ControlAllocationPseudoInverse`
  stopped linking once the DTRG CSV parameters made the allocator depend on the
  parameter system (it is now a functional test), and
  `AirmodeDisabledReducedThrustAndYaw` expected the upstream desaturation order
  (thrust given up for yaw); it now expects the DTRG order (yaw given up first).
