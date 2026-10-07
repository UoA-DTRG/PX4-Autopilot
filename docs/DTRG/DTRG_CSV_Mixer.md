# DTRG CSV Mixer

Replace the mixing matrix that PX4 computes from the vehicle geometry with a matrix read from a CSV file on the SD card. Use it to fly a mixer without touching the `CA_ROTOR*` geometry.



| Parameters | `DTRG_MIXER_CSV`, `DTRG_MIXER_NORM`, `DTRG_WINDUP_EN` (group *DTRG Mixer*) |
| File | `/fs/microsd/etc/mixer.csv` (fixed path) |
| Code | `src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp`, `src/modules/control_allocator`, `src/modules/commander/HealthAndArmingChecks/checks/dtrgMixerCheck.cpp` |
| uORB | `dtrg_mixer_status` (logged) |
| Tests | `DtrgMixerCsvTest.cpp` (tier 1), `test/dtrg/test_csv_mixer.py` (tier 2) |

> **Safety:** the CSV mixer bypasses every check PX4 does on the geometry. A wrong sign or a swapped row makes the vehicle flip on take-off. Check the mixer with `control_allocator status` and test it on a bench with the propellers removed before flying.

---

## 1. What it does

PX4 normally builds the **effectiveness matrix** $B$ (6 × N) from the `CA_ROTOR*` parameters. $B$ maps the actuator commands $u$ to the torque and thrust they produce:

$$
\begin{bmatrix} \tau \\ F \end{bmatrix} = B\,u ,\qquad
\tau = [\tau_x,\ \tau_y,\ \tau_z]^T,\quad F = [F_x,\ F_y,\ F_z]^T
$$

The control allocator then needs the inverse mapping, the **mixer** $M$ (N × 6), from the control setpoint $c = [\tau_x, \tau_y, \tau_z, F_x, F_y, F_z]^T$ to the actuators:

$$
u = u_{trim} + M\,(c - c_{trim})
$$

|  | Mixer $M$ |
| --- | --- |
| `DTRG_MIXER_CSV = 0` (default) | $M = \mathrm{normalise}\big(B^{+}\big)$, the Moore-Penrose pseudo-inverse of the geometry, then normalised (upstream PX4) |
| `DTRG_MIXER_CSV = 1` | $M$ = the matrix in `mixer.csv`, normalised only if `DTRG_MIXER_NORM = 1` |

Only the mixer is replaced. The effectiveness matrix $B$ still comes from `CA_ROTOR*`, and PX4 still uses it to compute the "unallocated torque" that the rate controller's anti-windup reads. That is why anti-windup is turned off with the CSV mixer (see [`DTRG_WINDUP_EN`](#dtrg_windup_en)).

This works with `CA_METHOD` pseudo-inverse and sequential desaturation (the multirotor default). Both share the same mixer code.

---

## 2. The file

`/fs/microsd/etc/mixer.csv`: **one row per actuator, one column per control
axis**, in this order:

| Column | 1 | 2 | 3 | 4 | 5 | 6 |
| --- | --- | --- | --- | --- | --- | --- |
| Axis | roll torque | pitch torque | yaw torque | thrust X (forward) | thrust Y (right) | thrust Z (down) |

Row *i* drives allocator actuator *i*, which is Motor *i* for a multirotor (motors come first, then servos). Axes are body FRD, as everywhere in PX4, so upward thrust is negative Z.  In a normal mixer the thrust-Z column is therefore negative.

Example: a symmetric quadrotor in X configuration, in PX4's motor order (1 front right CCW, 2 rear left CCW, 3 front left CW, 4 rear right CW), already normalised (`DTRG_MIXER_NORM = 0`):

```csv
-0.707107,0.707107,1,0,0,-1
0.707107,-0.707107,1,0,0,-1
0.707107,0.707107,-1,0,0,-1
-0.707107,-0.707107,-1,0,0,-1
```

A roll command of +1 (right side down) slows the right motors (1, 4) and speeds up the left ones (2, 3). A thrust-Z command of -0.5 sets every motor to 0.5.

Parser rules:
- The separator is a comma. Whitespace (also the `\r` of a Windows line ending) and a UTF-8 BOM are ignored.
- Blank lines are skipped. They do not count as rows.
- An empty cell (`1,,3`) reads as 0.
- Cells after the 6th are ignored, so a 7th "comment" column is allowed if it holds a number. Text is rejected.
- No header row: text is not a number and makes the file invalid.
- A cell can be at most 31 characters, so full double precision  (`-0.35355339059327373`) is fine.
- Any line length is accepted. At most 16 rows (the allocator maximum) are read.
- Entries with $|m_{ij}| < 10^{-3}$ are set to 0 after loading (the same clean-up PX4 does for its own mixer).
- Every value is printed on the console while loading (`Row 0, Col 0: -0.707107`), which helps to find a typo.

### When the file is rejected

The vehicle refuses to arm until the file is fixed. The reason is reported
in `dtrg_mixer_status` and shown in the ground station:

| `status` | Ground station message | Cause |
| --- | --- | --- |
| 0 `DISABLED` | `DTRG CSV mixer not loaded, reboot` | `DTRG_MIXER_CSV` was set to 1 but the board was not rebooted |
| 1 `LOADED` | (arming allowed) |  |
| 2 `FILE_NOT_FOUND` | `DTRG mixer file not found` | no SD card, or no file at the path |
| 3 `EMPTY` | `DTRG mixer file is empty` | no rows |
| 4 `SHORT_ROW` | `DTRG mixer line N: < 6 values` | a row with fewer than 6 cells |
| 5 `INVALID_VALUE` | `DTRG mixer line N: bad value` | text (for example a header), `nan`, `inf`, or a cell longer than 31 characters |
| 6 `ROW_COUNT_MISMATCH` | `DTRG mixer R rows, A actuators` | the number of rows is not the number of configured actuators (for example `CA_ROTOR_COUNT`) |
| 7 `ALL_ZERO` | `DTRG mixer is all zeros` | every value is 0 |

A rejected file never drives the motors. If no valid file was ever loaded, the mixer is all zero. If a valid file was loaded earlier in the session, that mixer is kept. The file is read gain whenever the effectiveness matrix is updated (for example after a `CA_*` parameter change), and a failed re-read in flight must not cut the motors.

`DTRG_MIXER_CSV` is only read at boot. If the file was rejected, arming stays blocked even after you set the parameter back to 0, until you reboot.

---

## 3. How to use it

1. Write the CSV (one row per motor, 6 columns, see above). The easiest starting point is PX4's own mixer for the vehicle, see [Getting the current mixer](#getting-the-current-mixer).
2. Copy it to the SD card as `etc/mixer.csv` (the `etc` folder at the card's root): take the card out and copy it on a PC, or upload it over MAVLink FTP (for example AVProxy's `ftp put mixer.csv /fs/microsd/etc/mixer.csv`).
3. Set the parameters:
	- DTRG_MIXER_CSV → 1
4. Check what is loaded:
	```sh
   control_allocator status
	```
   This prints `DTRG CSV OVERIDE IS Enabled`, the load status  (`DTRG CSV mixer status: 1 (line 0, 8 rows, 8 actuators)`, 1 = loaded) and the mixer in use, **transposed** (6 rows × 16 columns).
5. Test with the propellers off. Arm in *Stabilized* at low throttle, tilt the vehicle by hand and check that the motors on the low side speed up. The QGroundControl motor test does not go through the mixer, so use it to identify which motor is which.
6. Before flying, check `listener dtrg_mixer_status` shows `status: 1`.

To go back to the geometry mixer set DTRG_MIXER_CSV → 0 and reboot.

### Getting the current mixer

With `DTRG_MIXER_CSV = 0`, `control_allocator status` prints the mixer PX4 built from the geometry, after normalisation, under `PX4 C++ Mixer transpose=`. Transpose it back (rows = motors), keep the first N columns, and save it as CSV. Loaded with `DTRG_MIXER_NORM = 0`, it reproduces exactly the default behaviour. Edit from there.

---

## 4. Parameters

| Parameter | Default | Values | Reboot | Description |
| --- | --- | --- | --- | --- |
| `DTRG_MIXER_CSV` | 0 | 0 disabled, 1 enabled | yes | Use `/fs/microsd/etc/mixer.csv` instead of the pseudo-inverse of the geometry. Arming is refused while the file is missing or invalid. |
| `DTRG_MIXER_NORM` | 0 | 0 disabled, 1 enabled | no (read when the mixer is loaded) | Apply PX4's normalisation (below) to the CSV mixer. Has no effect with `DTRG_MIXER_CSV = 0`, where PX4 always normalises. |
| `DTRG_WINDUP_EN` <a id="dtrg_windup_en"></a> | 1 | 0 disabled, 1 enabled | no | Rate-controller anti-windup from the allocator's saturation feedback. Forced to 0 (and saved) by `mc_rate_control` as soon as `DTRG_MIXER_CSV = 1`. |

### `DTRG_MIXER_NORM`: the normalisation

The control setpoints are normalised (`[-1, 1]`), not in N and Nm. PX4 scales the columns of its pseudo-inverse so that a full roll, pitch or yaw command uses roughly the motors' full range.With `DTRG_MIXER_NORM = 1` the same scaling is applied to the CSV matrix $M$ (N actuators, column $m_j$):

$$
s_{roll} = \sqrt{\frac{\lVert m_{roll}\rVert^2}{n_{roll}/2}},\quad
s_{pitch} = \sqrt{\frac{\lVert m_{pitch}\rVert^2}{n_{pitch}/2}},\quad
s_{rp} = \max(s_{roll}, s_{pitch})
$$

$$
s_{yaw} = \max_i\, m_{i,yaw},\qquad
s_{k} = \frac{1}{n_k}\sum_i |m_{i,k}|\quad (k = x, y, z)
$$

$$
m_{roll} \leftarrow \frac{m_{roll}}{s_{rp}},\quad
m_{pitch} \leftarrow \frac{m_{pitch}}{s_{rp}},\quad
m_{yaw} \leftarrow \frac{m_{yaw}}{s_{yaw}},\quad
m_{k} \leftarrow \frac{m_{k}}{s_{k}}
$$

where $n_j$ is the number of actuators with a non-zero entry in column $j$  ($|m_{ij}| > 10^{-3}$ for roll and pitch, $> \varepsilon$ for thrust). Roll and pitch share one scale. An axis with no actuator takes the Z scale. The torque scales are only computed for multirotor airframes (`CA_AIRFRAME` 0); for the other airframe types they are 1, as upstream.

With `DTRG_MIXER_NORM = 0` the matrix is used as written (apart from zeroing entries below $10^{-3}$). The values must then already be "normalised setpoint → normalised motor command". A mixer printed by `control_allocator status` is already in those units.

### `DTRG_WINDUP_EN`: why anti-windup is off with the CSV mixer

The rate controller stops integrating on an axis when the allocator reports
that axis as saturated. The allocator gets this from the unallocated torque

$$
\tau_{unalloc} = c_\tau - B\,(u - u_{trim})
$$

which uses the geometry $B$. With a CSV mixer $u$ comes from a matrix that is
not $B^{+}$, so $\tau_{unalloc}$ is not zero even when nothing saturates, and
the integrators would be frozen at random. So with the CSV mixer the
saturation feedback is not used, and `mc_rate_control` sets
`DTRG_WINDUP_EN = 0` itself. When you go back to the geometry mixer, set
`DTRG_WINDUP_EN` back to 1 by hand: it is not restored automatically.

---

## 5. Logging

`dtrg_mixer_status` is published on change and at 1 Hz, and logged:

| Field |  |
| --- | --- |
| `status` | see [the table above](#when-the-file-is-rejected) |
| `line` | 1-based line of the file with the error (`SHORT_ROW`, `INVALID_VALUE`) |
| `num_rows` | rows read |
| `num_actuators` | actuators configured |

---

## 6. Note: how PX4 computes $B$ and $M$ without the CSV mixer

With `DTRG_MIXER_CSV = 0` both matrices come from the geometry parameters. This is upstream PX4 and is useful to understand what the CSV replaces.

### Effectiveness matrix $B$

`ActuatorEffectivenessRotors::computeEffectivenessMatrix()` builds one column per rotor $i$ from:

| Symbol | Parameter | Meaning |
| --- | --- | --- |
| $p_i$ | `CA_ROTORi_PX`, `_PY`, `_PZ` | position relative to the centre of gravity (m, body FRD) |
| $a_i$ | `CA_ROTORi_AX`, `_AY`, `_AZ` | thrust axis, normalised to unit length. Upwards is $[0, 0, -1]$ |
| $C_{T,i}$ | `CA_ROTORi_CT` | thrust coefficient |
| $k_{M,i}$ | `CA_ROTORi_KM` | moment ratio, positive for CCW, negative for CW rotation |

$$
F_i = C_{T,i}\,a_i ,\qquad
\tau_i = C_{T,i}\,\big(p_i \times a_i\big) - C_{T,i}\,k_{M,i}\,a_i ,\qquad
B_{:,i} = \begin{bmatrix} \tau_i \\ F_i \end{bmatrix}
$$

The first term of $\tau_i$ is the torque from the thrust acting at an arm $p_i$, the second the reaction torque of the propeller. A rotor with $C_T = 0$ or a zero axis gets an all-zero column.

Before the allocator sees $B$, `ControlAllocator` changes it in two ways:
- **Weak axes are removed.** If every entry of a row is $|B_{j,i}| \le 0.05$, the whole row is set to 0, so the allocator does not try to control an axis it has almost no authority over (for example $F_x$ and $F_y$ on a planar multirotor).
- **Failed motors are removed.** With failure detection, the column of a failed motor is set to 0 and the mixer is recomputed.

The trim follows from the linearisation point $u_{lin}$ (0 for a multirotor): $u_{trim} = u_{lin}$, $c_{trim} = B\,u_{lin}$.

### Mixer $M$

1. **Pseudo-inverse.** $M_0 = B^{+}$ (`matrix::geninv`). If $B$ has full row rank this is $B^T (B B^T)^{-1}$. Rows zeroed above give zero columns in $M_0$.
2. **Normalisation.** The column scales $s_{rp}$, $s_{yaw}$, $s_x$, $s_y$, $s_z$ are computed from $M_0$ with the formulas of [`DTRG_MIXER_NORM`](#dtrg_mixer_norm-the-normalisation), and each column is divided by its scale. This makes $M$ independent of the units of $C_T$ and of the arm lengths: only the *ratios* between rotors matter.
3. **Clean-up.** Entries with $|m_{ij}| < 10^{-3}$ are set to 0.

The scales are only recomputed on a configuration change (a `CA_*` parameter). After a motor failure $M$ is recomputed from the reduced $B$ but keeps the old scales, so a setpoint keeps the same meaning and the remaining motors take over the failed motor's share.

### Example

A symmetric X quad with arm length $r = 0.25$ m, $C_T = 6.5$, $|k_M| = 0.05$ (motor 1 at $p = [0.177, 0.177, 0]$, CCW):

$$
B = \begin{bmatrix}
-1.149 & 1.149 & 1.149 & -1.149 \\
1.149 & -1.149 & 1.149 & -1.149 \\
0.325 & 0.325 & -0.325 & -0.325 \\
0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 \\
-6.5 & -6.5 & -6.5 & -6.5
\end{bmatrix}
$$

The $F_x$ and $F_y$ rows are 0 (all axes point up). Pseudo-inverse and normalisation give exactly the matrix of the [example in section 2](#2-the-file): $\pm 0.707$ on roll and pitch, $\pm 1$ on yaw, $-1$ on thrust Z. Changing $r$, $C_T$ or $k_M$ by the same factor on every rotor leaves $M$ unchanged.

With the CSV mixer, steps 1 to 3 are replaced by the file (step 2 only if `DTRG_MIXER_NORM = 1`), but $B$ is still built as above and used for $c_{trim}$ and the unallocated torque (see [`DTRG_WINDUP_EN`](#dtrg_windup_en-why-anti-windup-is-off-with-the-csv-mixer)).
