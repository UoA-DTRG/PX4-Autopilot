# DTRG boards, builds and CI

What the fork changes in the build: the flight controller targets, which DTRG
features each build includes, the forked submodules, the GitHub Actions
workflows and the release naming.

---

## 1. Build targets

| Target | Board | DTRG specifics |
| --- | --- | --- |
| `goosetech_fmu-v6xrt_default` | GooseTech FMU-v6XRT (DTRG-specific target, `boards/goosetech/fmu-v6xrt`) | bench test, `dtrg` MAVLink dialect, see below |
| `px4_fmu-v6xrt_default` | Pixhawk FMU-v6XRT | bench test; RC drivers reduced to CRSF (`CONFIG_DRIVERS_RC_CRSF_RC` instead of `CONFIG_COMMON_RC`) |
| `px4_fmu-v6c_default` | Pixhawk 6C | bench test, `dtrg_test` example, SIH |
| `px4_fmu-v5_default` | Pixhawk 4 | bench test, SIH |
| `px4_sitl_default` | SITL | bench test, `dtrg` MAVLink dialect, SIH generic multirotor, planarOcto airframes |

Every build gets horizontal thrust, the CSV mixer, the DTRG sequential
desaturation and the RC channel conflict check: these live in modules that
every multirotor build already has (`mc_pos_control`, `mc_att_control`,
`control_allocator`, `commander`).

```sh
make goosetech_fmu-v6xrt_default            # build
make goosetech_fmu-v6xrt_default upload     # build and flash over USB
```

The firmware is `build/<target>/<target>.px4`. Flash it with QGroundControl
(*Vehicle Setup → Firmware → Advanced → Custom firmware file*) or with
`upload` as above.

### GooseTech FMU-v6XRT

Derived from `px4_fmu-v6xrt`. Compared with the Pixhawk target:

| | GooseTech | Pixhawk FMU-v6XRT |
| --- | --- | --- |
| IO coprocessor (`PX4IO`) | not built | built |
| RC input | `rc_input` and CRSF on the FMU | PX4IO and CRSF |
| Extra serial port | `EXT2` = `/dev/ttyS7` | `RC` = `/dev/ttyS4` |
| Ethernet (`CONFIG_BOARD_ETHERNET`) | not enabled | enabled |
| IMUs | started unconditionally (no hardware-revision check) | per hardware revision |
| Power module | Auterion PM selector on HW base 009/010, else INA2xx autostart | INA2xx autostart |
| Fixed-wing position control | none (see note) | `fw_mode_manager`, `fw_lateral_longitudinal_control` |
| Zenoh, hardfault stream, BMM350/LIS2MDL mags, AUAV airspeed | not built | built |
| MAVLink dialect | `dtrg` | upstream |

The bootloader and IO binaries are in `boards/goosetech/fmu-v6xrt/extras`.

> **Note:** the GooseTech config still lists `CONFIG_MODULES_FW_POS_CONTROL`
> and `CONFIG_MODULES_ROVER_POS_CONTROL` from v1.16. Neither module exists in
> v1.17 (they were split into `fw_mode_manager` /
> `fw_lateral_longitudinal_control` and the `rover_*` modules), so the lines
> have no effect and this build has no fixed-wing or rover position control.
> Multirotor builds are not affected.

### Adding a DTRG feature to a board

The features that are separate modules are switched on in the board's
`default.px4board`:

| Feature | Kconfig |
| --- | --- |
| [Bench test mode](DTRG_Bench_Test_Mode.md) | `CONFIG_MODULES_BENCH_TEST=y` |
| [DTRG MAVLink dialect](DTRG_MAVLink_Dialect.md) | `CONFIG_MAVLINK_DIALECT="dtrg"` |
| `dtrg_test` example | `CONFIG_EXAMPLES_DTRG_TEST=y` |
| [SIH on the board](DTRG_Planar_Octo_SITL.md) (airframe 12017) | `CONFIG_MODULES_SIMULATION_SIMULATOR_SIH=y` |

Watch the flash size: on `px4_fmu-v6c` there is about 32 kB left with the
toolchain CI uses.

---

## 2. Forked submodules

| Path | Fork | Branch | Why |
| --- | --- | --- | --- |
| `src/modules/mavlink/mavlink` | `UoA-DTRG/dtrg-mavlink` | `dtrg-v1.17` | the `dtrg` dialect (`DTRG_OFFBOARD`) |
| `Tools/simulation/gz` | `UoA-DTRG/PX4-gazebo-models` | `dtrg-main` | the `planar_octo` Gazebo model |

After switching branches run `git submodule update --init --recursive`.

---

## 3. CI

Both workflows run on every push and pull request to `dtrg-main`, and by hand
(*Actions → Run workflow*). A newer push to the same branch cancels the
running one.

| Workflow | Jobs | Output |
| --- | --- | --- |
| `.github/workflows/dtrg_build.yml` (*DTRG Build*) | one job per NuttX target (`px4_fmu-v5`, `px4_fmu-v6c`, `px4_fmu-v6xrt`, `goosetech_fmu-v6xrt`) and `px4_sitl_default` | the `.px4` firmware of each NuttX target, as artifacts of the run |
| `.github/workflows/dtrg_tests.yml` (*DTRG Tests*) | unit (tier 1), SIH logic (tier 2), SIH flight (tier 3) | test results and logs; see [Automated testing](DTRG_Automated_Testing.md) |

*DTRG Build* skips changes that only touch `docs/` or Markdown files. The
NuttX jobs use Arm GNU Toolchain 13.2.rel1 (the same as the local builds),
not the container's 9.3.1: the older compiler produces ~2 % larger code,
which overflows the flash of `px4_fmu-v6c`.

The upstream PX4 workflows are still in `.github/workflows` but target
upstream's branches; the upstream ITCM check and docs deployment were removed.

---

## 4. Releases

From the [README](../../README.md): develop on `yourname/feature-name`,
open a pull request to `dtrg-main`, and after merging publish a release with
the commonly used binaries (the *DTRG Build* artifacts), tagged

```
v{PX4 version}-{DTRG version}      e.g. v1.17.0-1.2.3
```

---

## 5. Other fork fixes

Small changes outside the features, listed so they are not mistaken for
upstream behaviour:

| Change | Where | Effect |
| --- | --- | --- |
| HRT re-queue fix | `platforms/posix/src/px4/common/drv_hrt.cpp` | In SITL, a periodic work item that re-scheduled itself from another thread (e.g. gyro calibration on arming) could be queued twice and silently stop all other periodic work. Fixed. |
| `PX4_PARAM_*` applied again | `ROMFS/px4fmu_common/init.d-posix/rcS` | `PX4_PARAM_<name>` environment overrides now win over the airframe's `param set-default`, also when the value equals the firmware default. |
| Gazebo standalone world | `ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim` | With `PX4_GZ_STANDALONE`, the world name is read from the running Gazebo. |
| macOS Gazebo build | `src/modules/simulation/gz_plugins`, `gz_bridge` | Finds Homebrew `qt@5`; tolerates newer protobuf warnings; the optical-flow plugin is skipped without a usable OpenCV. |
| Range finder consistency | `ekf2/.../range_finder_consistency_check.cpp` | The kinematic consistency check is only updated while there is vertical motion. |
| `SYS_STATUS` | `src/modules/mavlink/streams/SYS_STATUS.hpp` | `errors_count1..4` carry desaturation / motor / HT saturation flags for the [Status Monitor](https://github.com/UoA-DTRG/status_monitor). |
