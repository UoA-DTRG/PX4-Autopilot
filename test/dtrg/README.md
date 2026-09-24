# DTRG SITL tests (tiers 2 and 3)

Tier 2: logic tests of the DTRG features on PX4 SITL with the SIH simulator: RC
channel conflict check, bench test mode, horizontal thrust wiring and the CSV
mixer. They assert on arming decisions, modes, status texts and setpoints, not
on how the vehicle flies.

Both tiers run on the planarOcto (`sihsim_planar_octo`, the default airframe).

Tier 3 (`test_flight_ht.py`, marked `flight`): the fully actuated planarOcto
flies in SIH (`SIH_VEHICLE_TYPE 4` builds it from the airframe's `CA_ROTOR*`
geometry), and the tests check SIH's ground truth: take-off and hold, moving
level with horizontal thrust, holding position while tilted, HT toggling, bench
test in the air.

The plan, the tier structure and the list of known gaps are in
[Tools/dtrg/SITL_TESTING.md](../../Tools/dtrg/SITL_TESTING.md). CI runs this in
`.github/workflows/dtrg_tests.yml`.

## Run

```bash
make px4_sitl_default
pip3 install -r test/dtrg/requirements.txt

python3 -m pytest test/dtrg -m "sih and not flight" -v

# tier 3, about 10 minutes
python3 -m pytest test/dtrg -m flight -v

# one test, keeping px4.log and the ULogs
python3 -m pytest test/dtrg -k bench_test_rejected_while_armed -v --basetemp=/tmp/dtrg
```

Options (also settable through environment variables):

| Option | Env | Default | |
|---|---|---|---|
| `--px4-build=DIR` | `DTRG_PX4_BUILD` | `build/px4_sitl_default` | any SITL build, e.g. `build/px4_sitl_test` from `make tests` |
| `--px4-instance=N` | `DTRG_PX4_INSTANCE` | `0` | use a free instance to run next to another SITL (ports 14540+N) |
| `--airframe=NAME` | `DTRG_SIH_AIRFRAME` | `sihsim_planar_octo` | see `airframes.py`; `sihsim_quadx` skips the `fully_actuated` tests |
| `--speed-factor=X` | `DTRG_SPEED_FACTOR` | `1` | `PX4_SIM_SPEED_FACTOR` |

Pass options as `--opt=value`: with a space, pytest reads the value as a test path.

## Layout

| File | |
|---|---|
| `conftest.py` | `sitl` fixture: fresh PX4 + clean rootfs per test, parameters at boot, optional RC stream |
| `px4_process.py` | starts/stops PX4, runs `px4-<cmd>` shell commands, reads topics with `listener` |
| `vehicle.py` | MAVLink helper: arming, modes, parameters, status texts, RC override |
| `rc_layout.py` | the one RC channel layout every test uses, and the RC scaling / mode slot maths |
| `ulog_checks.py` | reads topics back from the run's ULog |
| `airframes.py` | SIH airframes: the fully actuated `sihsim_planar_octo` (default) and `sihsim_quadx` |
| `flight.py` | tier 3: take-off, and SIH ground truth (position, attitude) read back from the ULog |
| `test_*.py` | the tests |

Things that are easy to get wrong when adding tests:

- PX4 only sends STATUSTEXT to a link that has seen a GCS heartbeat. `Vehicle` sends one at 1 Hz.
- Health failures (`Preflight Fail: ...`) are printed when the failure set changes, at most every
  2 s, not on every arm request. Take the `since` mark before causing the failure.
- RC override only becomes manual control with `RC_CHAN_CNT > 0` (`rc_layout.RC_PARAMS` sets it).
- `RCx_DZ` is 10 us on channels 1-8 and 0 on 9-18: use `rc_layout.normalized(pwm, channel)`.
- The bench test main mode in HEARTBEAT is 11 (`PX4_CUSTOM_MAIN_MODE_BENCH_TEST`); the RC slot
  value is 16 (`COM_FLTMODEx`); the nav state is 16.
- A mode switch is only acted on when it changes: hold the intermediate position ~1 s.

Tests marked `fully_actuated` are skipped on an airframe that cannot make horizontal thrust
without tilting.

Flight tests (tier 3):

- Mark test phases with `vehicle.boot_time()`: it is PX4 time, the clock of the ULog.
- Assert on `flight.truth()` (SIH ground truth), not on the estimator.
- With RC streaming, `wait_mode(MAIN_STABILIZED)` before `take_off()`, or the RC slot being
  applied after boot can override Takeoff mode.
- If the startup script hangs on a `px4-<module>` client call (seen on macOS), the fixture
  warns and boots again once.
