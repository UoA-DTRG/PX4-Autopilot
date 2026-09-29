# DTRG SITL input tools

Host-side scripts for flying the DTRG planarOcto in SITL. Both need `pymavlink`
(`pip install pymavlink`).

| Script | What it sends | Use it for |
|---|---|---|
| `keyboard_teleop.py` | `MANUAL_CONTROL` (sticks, aux1-4) + `RC_CHANNELS_OVERRIDE` (HT ch8-10, on first HT key) | Flying the vehicle by hand: arm, takeoff, modes, sticks, horizontal thrust |
| `rc_override.py` | `RC_CHANNELS_OVERRIDE` only | Holding fixed RC channel values, or driving HT channels alone |

## Quick start

```bash
make px4_sitl gz_planar_octo_mocap          # terminal 1
Tools/dtrg/keyboard_teleop.py               # terminal 2, listens on udpin:0.0.0.0:14540
```

Press the keys shown on screen (`m` arm, `t` takeoff, `e` HT mode, `z/x/c/v` HT roll/pitch, `q` quit).

Fixed channel values instead of the keyboard:

```bash
Tools/dtrg/rc_override.py --norm 8=1 --norm 9=0.5 --norm 10=-1
```

Run `Tools/dtrg/rc_override.py --help` for all options and gotchas.

HT channels 8/9/10 match the planarOcto airframe defaults (`RC_MAP_HT_MODE/ROLL/PITCH`).
Run only one of these scripts at a time: both send `RC_CHANNELS_OVERRIDE`, and they would override each other.
