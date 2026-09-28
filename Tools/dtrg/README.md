# DTRG SITL input tools

Host-side scripts for flying the DTRG planarOcto in SITL, and for recording
what it pushes against.

| Script | What it does | Use it for |
|---|---|---|
| `keyboard_teleop.py` | Sends `MANUAL_CONTROL` (sticks, aux1-4) + `RC_CHANNELS_OVERRIDE` (HT ch8-10, on first HT key) | Flying the vehicle by hand: arm, takeoff, modes, sticks, horizontal thrust |
| `rc_override.py` | Sends `RC_CHANNELS_OVERRIDE` only | Holding fixed RC channel values, or driving HT channels alone |
| `log_interaction_force.py` | Records the interaction pole's force sensor to CSV | Measuring what the vehicle pushes the surface with |

The two flying scripts need `pymavlink` (`pip install pymavlink`).
`log_interaction_force.py` needs nothing beyond the `gz` CLI that running the
simulation already requires.

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

## Physical-interaction testing

`gz_planar_octo_rod_mocap_interaction` flies a planarOcto with a rigid 0.35 m
rod on its nose, in a copy of the mocap laboratory that also holds a floor
pole with a vertical push panel:

```bash
make px4_sitl gz_planar_octo_rod_mocap_interaction
```

The panel face is 2 m straight ahead of the takeoff marker (+X) and spans
z = 0.80 m to 2.00 m, so hovering at 1.0-1.5 m and flying forward brings the
rod tip onto it; contact happens with the vehicle centred near x = 1.38 m.
The pole is rigid and immovable, which is the usual reference condition for
interaction-force control.

Horizontal thrust (`e`, then `z/x/c/v` in `keyboard_teleop.py`) is what pushes
without pitching the airframe: the rod is mounted at the height of the centre
of mass, so an axial contact force produces no pitching moment.

### Where the force is recorded

Both ends of the contact are instrumented with a Gazebo `force_torque` sensor,
and each is recorded in the place that suits it.

**Vehicle side, into the ulog.** A sensor on the rod mount measures the wrench
the rod transmits into the airframe. `gz_bridge` subscribes to it, converts
body FLU to body FRD and publishes the `interaction_wrench` uORB topic, which
the logger records at the sensor's full 250 Hz:

```bash
px4-listener interaction_wrench          # live, from the pxh console
```

```python
from pyulog import ULog
d = ULog("log/.../xx.ulg", ["interaction_wrench"]).data_list[0]
d.data["force[0]"]                       # N, body FRD
```

Pressing the rod forward into the panel gives a **negative** `force[0]`: the
logged quantity is what the environment feeds back into the vehicle, not what
the vehicle exerts. It includes the rod's own weight and its inertial reaction,
so it is the external contact force only near steady flight. The topic is
registered as optional, so models without a rod and real hardware simply never
publish it.

**Surface side, into a CSV.** PX4 never sees the pole, so there is nothing to
put it in the ulog. `log_interaction_force.py` records it instead:

```bash
Tools/dtrg/log_interaction_force.py                       # until Ctrl-C
Tools/dtrg/log_interaction_force.py --duration 60 -o push.csv
```

Columns are `sim_time_s,wall_time_s,fx,fy,fz,tx,ty,tz`, in the pole frame,
which is world aligned. A push in +X gives a **positive** `fx`. Taring is on by
default and removes the pole's standing weight, so keep clear of the panel for
the first second. Use `sim_time_s` to line the CSV up against a ulog; it is
Gazebo's clock, and it stops when the simulation pauses.

A cross-check of both records over one contact: the rod read a -136.6 N peak on
`force[0]` while the pole read +120.0 N on `fx`. Equal and opposite, with the
gap being the rod's inertial reaction during an impulsive hit.

### What the contact sensors do and do not give

The rod tip and the panel also carry `contact` sensors:

```bash
gz topic -e -t /world/mocap_interaction/model/interaction_pole/link/pole/sensor/push_surface_contact/contact
gz topic -e -t /world/mocap_interaction/model/planar_octo_rod_0/link/interaction_rod/sensor/rod_tip_contact/contact
```

These answer *where* contact happened, not *how hard*: point positions, normals
and penetration depth. Their `wrench` field exists in the message definition
but gz-sim leaves it at zero, checked against a 10 kg load that must carry
98 N. Use them to detect and locate contact; use the force_torque sensors for
force.

### Gotchas

1. **`GZ_IP=127.0.0.1`.** PX4 starts the Gazebo server with it set, and a `gz`
   client without it discovers nothing at all: `gz topic -l` comes back empty
   and `gz topic -e` blocks on a topic that is plainly working. The CSV logger
   retries with it set, so it works either way, but any `gz` command you type
   by hand needs it.
2. **A force_torque sensor reports no external contact load when its chain is
   rigidly welded to the world.** Welded bodies merge into the world's
   infinite-mass body, which swallows the contact, and the sensor then reports
   only the child's own weight. This is why the pole hangs off a stiff
   prismatic joint (1e6 N/m, 50 um under a 50 N push) rather than a weld. The
   vehicle is unaffected, because it is a free body; its rod joint is a plain
   weld and reports contact correctly.
3. **Never put a `<plugin>` in a world SDF.** Any world-level plugin there
   replaces PX4's entire `server.config` plugin set, leaving a world with no
   physics, no sensors and no spawn service. Both sensor systems
   (`gz-sim-contact-system`, `gz-sim-forcetorque-system`) are loaded from
   `src/modules/simulation/gz_bridge/server.config` instead.

The pieces live in `Tools/simulation/gz/models/planar_octo_rod`,
`Tools/simulation/gz/models/interaction_pole` and
`Tools/simulation/gz/worlds/mocap_interaction.sdf`; the plain `planar_octo`
model and `mocap` world are untouched.
