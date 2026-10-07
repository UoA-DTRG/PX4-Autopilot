# DTRG fork documentation

Documentation of the [DTRG](https://dtrg.org/) additions to PX4 (v1.17). Each
page says what the feature does, how to set it up (with every parameter), and
the maths behind it where there is some.

> Several of these features drive the motors in ways upstream PX4 never does
> (no control loops, a hand-written mixer, sideways thrust). Read the page,
> and try the feature in [SITL](DTRG_Planar_Octo_SITL.md) or on a stand,
> before flying it.

## Flight features

| Page | Summary | Main parameters |
| --- | --- | --- |
| [CSV mixer](DTRG_CSV_Mixer.md) | Replace the geometry-derived mixer with a matrix from `/fs/microsd/etc/mixer.csv` | `DTRG_MIXER_CSV`, `DTRG_MIXER_NORM`, `DTRG_WINDUP_EN` |
| [Horizontal thrust mode](DTRG_Horizontal_Thrust_Mode.md) | Fully actuated flight: move with body X/Y thrust instead of tilting, tilt set separately. In Stabilized the sticks command X/Y thrust (level mode); in Position/Altitude/Offboard/Auto the position controller does; X/Y thrust cap | `DTRG_HT_*`, `RC_MAP_HT_*` |
| [Sequential desaturation](DTRG_Sequential_Desaturation.md) | Give up X/Y thrust first and attitude last when the motors saturate | `MC_AIRMODE` |
| [Bench test mode](DTRG_Bench_Test_Mode.md) | Hover + step/ramp excitation on one axis with every control loop off, for system identification on a stand | `BT_*`, `RC_MAP_CMD_SIGN` |
| [RC channel conflict check](DTRG_RC_Channel_Conflict_Check.md) | Refuse to arm when two `RC_MAP_*` functions share a channel | `COM_ARM_RC_CONF` |

## Simulation, tooling and infrastructure

| Page | Summary |
| --- | --- |
| [planarOcto SITL](DTRG_Planar_Octo_SITL.md) | The planarOcto in Gazebo, SIH (PC and flight controller) and HITL; the SIH generic multirotor; keyboard and RC-override tools |
| [Automated testing](DTRG_Automated_Testing.md) | Unit, SIH logic and SIH flight tests of the features above; CI |
| [DTRG MAVLink dialect](DTRG_MAVLink_Dialect.md) | The `dtrg` dialect and the `DTRG_OFFBOARD` message (received and logged, not yet used by a controller) |
| [Boards, builds and CI](DTRG_Boards_Builds_and_CI.md) | Build targets including GooseTech FMU-v6XRT, forked submodules, build workflow, releases, smaller fork fixes |

Related, outside this folder: [Tools/dtrg](../../Tools/dtrg/README.md) (SITL
input scripts, [SIH vs Gazebo](SIH_VS_GAZEBO.md)),
[test/dtrg](../../test/dtrg/README.md) (test harness),
[src/modules/bench_test/README.md](../../src/modules/bench_test/README.md),
[Status Monitor](https://github.com/UoA-DTRG/status_monitor).

## New uORB topics

| Topic | Published by | Page |
| --- | --- | --- |
| `dtrg_mixer_status` | `control_allocator` | [CSV mixer](DTRG_CSV_Mixer.md) |
| `horizontal_thrust_limit` | `mc_pos_control`, `mc_att_control` | [HT mode](DTRG_Horizontal_Thrust_Mode.md) |
| `sequential_desaturation`, `dtrg_desaturated_control` | `control_allocator` | [Sequential desaturation](DTRG_Sequential_Desaturation.md) |
| `dtrg_custom` | `mavlink` (from `DTRG_OFFBOARD`) | [MAVLink dialect](DTRG_MAVLink_Dialect.md) |

All are logged by default.
