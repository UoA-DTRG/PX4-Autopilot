# DTRG SITL input tools

Host-side scripts for flying the DTRG planarOcto in SITL. Both need `pymavlink`
(`pip install pymavlink`).

| Script | What it sends | Use it for |
|---|---|---|
| `keyboard_teleop.py` | `MANUAL_CONTROL` (sticks, aux1-4) + `RC_CHANNELS_OVERRIDE` (HT ch8-10 and constant command ch7, on first HT or constant command key) | Flying the vehicle by hand: arm, takeoff, modes, sticks, horizontal thrust, constant command |
| `rc_override.py` | `RC_CHANNELS_OVERRIDE` only | Holding fixed RC channel values, or driving HT channels alone |
| `compare_sih_gz.py` | Offboard setpoints, `RC_CHANNELS_OVERRIDE` | Flying the same profile in SIH and Gazebo and comparing them; results in [SIH_VS_GAZEBO.md](SIH_VS_GAZEBO.md) |

## Quick start

```bash
make px4_sitl gz_planar_octo_mocap          # terminal 1
Tools/dtrg/keyboard_teleop.py               # terminal 2, listens on udpin:0.0.0.0:14540
```

Press the keys shown on screen (`m` arm, `t` takeoff, `e` HT mode, `z/x/c/v` HT roll/pitch, `u` constant command, `q` quit).

Fixed channel values instead of the keyboard:

```bash
Tools/dtrg/rc_override.py --norm 8=1 --norm 9=0.5 --norm 10=-1
```

Run `Tools/dtrg/rc_override.py --help` for all options and gotchas.

HT channels 8/9/10 match the planarOcto airframe defaults (`RC_MAP_HT_MODE/ROLL/PITCH`).
The constant command switch, channel 7, matches the planetaryHex simulation airframes
(`12019_gz_planetary_hex`, `12021_sihsim_planetary_hex`: `RC_MAP_CC_MODE 7`, `DTRG_CC_EN 1`).
It only engages in flight; set the held commands with `DTRG_CC_M_CMD`.
Run only one of these scripts at a time: both send `RC_CHANNELS_OVERRIDE`, and they would override each other.
