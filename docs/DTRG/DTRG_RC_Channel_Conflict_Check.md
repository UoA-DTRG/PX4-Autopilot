# DTRG RC channel conflict check

A preflight check that **refuses to arm when two RC functions are mapped to
the same RC channel**. The DTRG features add several channel parameters
(`RC_MAP_HT_MODE`, `RC_MAP_HT_ROLL`, `RC_MAP_HT_PITCH`, `RC_MAP_CMD_SIGN`),
and PX4 does not stop two `RC_MAP_*` parameters naming the same channel. With
such an overlap one switch drives two functions at once: moving the bench test
direction switch would also tilt the vehicle, or the HT switch would also be
the kill switch.

| | |
| --- | --- |
| Parameter | `COM_ARM_RC_CONF` (group *Commander*) |
| Code | `src/modules/commander/HealthAndArmingChecks/checks/rcChannelConflictCheck.cpp` |
| Tests | `test/dtrg/test_rc_conflict.py` (tier 2) |

---

## 1. What it does

- At boot it scans the parameter table and picks up **every integer parameter
  whose name starts with `RC_MAP`**: the PX4 channel-mapping convention.
  Nothing has to be registered: a new feature joins the check just by naming
  its parameter `RC_MAP_...`. Parameters of modules not in the build are
  skipped.
- On every parameter change it works out which raw channels each parameter
  occupies, and finds the channels claimed more than once.
- While disarmed, a conflict:
  - fails the arming check (unless `COM_ARM_RC_CONF = 1`), shown as
    `Preflight Fail: RC_MAP_A and RC_MAP_B both use RC channel N`;
  - is announced as a status text as soon as it appears, and again every
    30 s while it stands (a ground station that connects later also gets it):
    `RC ch N: RC_MAP_A and RC_MAP_B` (up to three pairs at a time);
  - when fixed: `RC channel conflict resolved`.
- While armed it is not evaluated (the mapping cannot change meaningfully in
  flight).

### Which parameters are checked

| Parameters | How they are read |
| --- | --- |
| All `RC_MAP_*` holding a channel number (`ROLL`, `PITCH`, `YAW`, `THROTTLE`, `FLTMODE`, `ARM_SW`, `KILL_SW`, `RETURN_SW`, `LOITER_SW`, `OFFB_SW`, `GEAR_SW`, `TRANS_SW`, `TERM_SW`, `PAY_SW`, `ENG_MOT`, `FLAPS`, `AUX1-6`, `PARAM1-3`, `HT_MODE`, `HT_ROLL`, `HT_PITCH`, `CMD_SIGN`, ...) | 1-based channel; 0 = unassigned; above 18 is ignored |
| `RC_MAP_FLTM_BTN` | bitmask of up to 6 channels, only while `RC_MAP_FLTMODE = 0` (PX4 ignores the buttons when a mode switch is set) |
| `RC_MAP_MODE_SW` | not checked: deprecated, no longer read |
| `RC_MAP_FAILSAFE` | not checked: it is meant to share the throttle channel |

---

## 2. How to use it

Nothing to do: it is always on. If arming is refused with a conflict message:

```sh
param show RC_MAP_*
```

and give one of the two functions a different channel (or set it to 0).

To keep the message but allow arming (for example on a bench setup where a
channel deliberately drives two things):

| Parameter | Default | Values |
| --- | --- | --- |
| `COM_ARM_RC_CONF` | 0 | 0 deny arming, 1 warning only |

The status text is shown in both cases.

---

## 3. How the conflict is computed

Each parameter $k$ is turned into a channel bitmask $c_k$ (bit 0 = channel 1):

$$
c_k = \begin{cases}
2^{\,v_k - 1} & 1 \le v_k \le 18 \text{ (channel parameter)}\\
v_k \;\&\; (2^{18}-1) & \text{bitmask parameter, when active}\\
0 & \text{unassigned, out of range or inactive}
\end{cases}
$$

Walking through the parameters once, with $S$ the channels seen so far:

$$
C \leftarrow C \,|\, (S \,\&\, c_k),\qquad S \leftarrow S \,|\, c_k
$$

$C \ne 0$ means a conflict; each pair $(k, l)$ with $c_k \,\&\, c_l \ne 0$ is
reported with the lowest shared channel.
