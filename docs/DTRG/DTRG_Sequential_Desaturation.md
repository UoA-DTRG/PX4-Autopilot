# DTRG sequential desaturation

When the motors cannot deliver everything that is asked of them, the control allocator has to decide what to give up. Upstream PX4 never desaturates the horizontal (X/Y) thrust of a multirotor, because a normal multirotor has none. On a fully actuated vehicle flying [horizontal thrust](DTRG_Horizontal_Thrust_Mode.md) that is the wrong choice: X/Y thrust would be clipped motor by motor and spill into roll, pitch and yaw. The DTRG fork changes the order so that **horizontal thrust is given up first and attitude last**, and logs how much of each axis was given up.

The HT thrust limit `DTRG_HT_MAX`, which caps the X/Y thrust before it reaches the allocator, is described in [HT mode, thrust limit](DTRG_Horizontal_Thrust_Mode.md#54-ht-thrust-limit-dtrg_ht_max), together with how to tune it against the desaturation logged here.

| | |
| --- | --- |
| Active when | `MC_AIRMODE = 0` (default) and `CA_METHOD` = 1 (sequential desaturation) or 2 (automatic, the multirotor default) |
| Parameters | `MC_AIRMODE`, `CA_METHOD` (upstream) |
| Code | `src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp` (`mixAirmodeDisabled()`) |
| uORB | `sequential_desaturation`, `dtrg_desaturated_control` (both logged) |
| Tests | `DtrgSequentialDesaturationTest.cpp`, `ControlAllocationSequentialDesaturationTest.cpp` (tier 1) |

---

## 1. What it does

The allocator first mixes the whole demand (all six axes) with the mixer $M$ (the pseudo-inverse of the geometry, or the [CSV mixer](DTRG_CSV_Mixer.md)):

$$
u = u_{trim} + M\,(c - c_{trim}),\qquad c = [\tau_x, \tau_y, \tau_z, F_x, F_y, F_z]^T
$$

If some $u_i$ is outside $[u_{min,i}, u_{max,i}]$ (for motors, normally $[0, 1]$), it then walks through the axes in a fixed order. At each step it changes the demand **on that axis only**, by just enough to bring the motors back inside their limits, before moving on to the next axis. An axis late in the list is only touched if the earlier ones could not fix the saturation.

| Step | Axis         | Direction allowed           |                |
| ---- | ------------ | --------------------------- | -------------- |
| 1    | thrust X     | both                        | given up first |
| 2    | thrust Y     | both                        |                |
| 3    | yaw torque   | both                        |                |
| 4    | thrust Z     | only **less** upward thrust |                |
| 5    | roll torque  | both                        |                |
| 6    | pitch torque | both                        | kept longest   |

Upstream (airmode off) does: thrust Z (reduce only), roll, pitch, then yaw is
added separately with 15 % extra headroom at the top. X/Y thrust is never
desaturated and is simply clipped with the rest.

The other airmode settings (`MC_AIRMODE` 1 = roll/pitch, 2 = roll/pitch/yaw)
are unchanged from upstream and do not publish the DTRG topics.

---

## 2. The maths

### One desaturation step

For an axis $j$, the direction is the mixer column $d = M e_j$. Moving the actuators along $d$ by $k$ is exactly the same as changing the demand on axis $j$ by $k$:

$$
u + k\,d = u_{trim} + M\,(c + k\,e_j - c_{trim})
$$

The gain $k$ is computed from the actuators that violate a limit, using only actuators with a meaningful effect on that axis ($|d_i| \ge 0.2$, so that a weak actuator does not ask for a huge gain):

$$
\kappa_i = \begin{cases}
\dfrac{u_{min,i} - u_i}{d_i} & u_i < u_{min,i} \\[2mm]
\dfrac{u_{max,i} - u_i}{d_i} & u_i > u_{max,i}
\end{cases}
\qquad
g(d, u) = \min\big(0, \min_i \kappa_i\big) + \max\big(0, \max_i \kappa_i\big)
$$

If only one side is violated, $g$ moves just enough to fix the worst actuator. If actuators are violated on both sides, the two pulls partly cancel. The step is applied twice, the second time at half gain, to share what is left between the upper and the lower violations:

$$
k_1 = g(d, u),\quad u \leftarrow u + k_1 d,\qquad
k_2 = \tfrac{1}{2}\, g(d, u),\quad u \leftarrow u + k_2 d,\qquad
k_j = k_1 + k_2
$$

For thrust Z, a negative $k_1$ (which would add upward thrust: the Z column is negative because up is $-Z$) is not allowed, and the step is skipped ($k_Z = 0$). So $k_Z \ge 0$ always, and $k_Z > 0$ means collective thrust was reduced.

### What the vehicle actually got

After the six steps, the demand that was really mixed is

$$
c_{achieved} = c_{sp} + [k_{roll}, k_{pitch}, k_{yaw}, k_x, k_y, k_z]^T
$$

This is published as `dtrg_desaturated_control` (`torque_sp`/`thrust_sp` before, `torque`/`thrust` after). Clipping and slew-rate limits applied after the allocator are not included.

### Example

Hovering with all motors at 0.6, HT asks for so much forward thrust that motor 3 (X column entry $d_3 = 0.5$) is at $u_3 = 1.15$ and nothing else saturates. Step 1: $k_1 = (1 - 1.15)/0.5 = -0.3$, so the X demand is reduced by 0.3 and motor 3 ends exactly at 1.0. The second pass finds no violation, so $k_x = -0.3$. Y, yaw, Z, roll and pitch are untouched: `sequential_desaturation.x_sat = -0.3`, and the vehicle accelerates less than asked, but keeps its attitude.

---

## 3. Logging

| Topic | Fields | Meaning |
| --- | --- | --- |
| `sequential_desaturation` | `x_sat`, `y_sat`, `z_sat`, `roll_sat`, `pitch_sat`, `yaw_sat` | the gains $k_j$ of the last allocation: 0 = nothing given up on that axis; the sign is the direction of the correction |
| `dtrg_desaturated_control` | `torque_sp`, `thrust_sp`, `torque`, `thrust` | demand before / after desaturation, same units as `vehicle_torque_setpoint` / `vehicle_thrust_setpoint` |

Both allocator topics are published on every allocation while
`MC_AIRMODE = 0`, also while disarmed. A disarmed vehicle at idle thrust shows
large gains as soon as the attitude controller asks for any torque; ignore
them.

The same information is sent to the [Status Monitor](https://github.com/UoA-DTRG/status_monitor)
in the `SYS_STATUS` `errors_count1..3` fields while armed.
